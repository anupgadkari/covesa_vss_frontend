//! Turn Indicator — activates the correct direction indicator when
//! the driver moves the turn signal stalk.
//!
//! Subscribes to:
//!   - Body.Switches.TurnIndicator.Direction (overlay sensor — stalk)
//!     Values: "OFF", "LEFT", "RIGHT"
//!   - Vehicle.LowVoltageSystemState (ignition / power mode)
//!     Turn signals only operate when ignition is ON or START.
//!   - Body.Lights.DirectionIndicator.{Left,Right}.Lamp.Front.IsOn
//!     (feedback from BlinkRelay, used to count comfort-blink flashes)
//!
//! Outputs (via Lighting arbiter):
//!   - Body.Lights.DirectionIndicator.Left.IsSignaling  @ MEDIUM (2)
//!   - Body.Lights.DirectionIndicator.Right.IsSignaling @ MEDIUM (2)
//!
//! ## Auto lane change (comfort blink / tip-to-signal)
//!
//! The feature counts complete flash cycles (on+off) while the stalk
//! is held. When the stalk returns to OFF:
//!   - If the indicator has already completed `lane_change_flash_count`
//!     (default 3) or more flashes, it releases **immediately** (normal
//!     full-engagement turn signal).
//!   - If fewer than `lane_change_flash_count` flashes have completed
//!     (quick tap / tip), it continues signaling for the remaining
//!     flashes so the total always reaches the configured count.
//!
//! This means a quick tap always produces exactly N flashes total,
//! while a long hold stops as soon as the driver releases the stalk.
//!
//! A "flash" is one complete on+off cycle of the physical lamps. The
//! feature counts falling edges (on→off transitions) from the
//! BlinkRelay's lamp feedback signal.
//!
//! The comfort-blink countdown is **immediately cancelled** (indicator
//! released) when:
//!   - Ignition leaves ON/START (REQ-TURN-008 takes precedence)
//!   - The opposite direction stalk is engaged (switch sides immediately)
//!
//! Like HazardLighting, this feature does NOT control blink timing.
//! The stalk's self-cancelling behavior (via steering angle feedback)
//! is handled by the body ECU firmware — this feature just tracks the
//! stalk position signal.
//!
//! Unlike HazardLighting, turn signals require ignition ON. When
//! ignition leaves ON/START, any active indicator is deactivated.
//! When ignition returns to ON/START, the current stalk position is
//! re-evaluated and indicators re-activated if the stalk is not OFF.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::arbiter::{ActuatorRequest, DomainArbiter};
use crate::config::PlatformConfig;
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

/// Physical turn signal stalk input (overlay signal).
const TURN_STALK: VssPath = "Body.Switches.TurnIndicator.Direction";

/// Power state signal — standard VSS v4.0.
const POWER_STATE: VssPath = "Vehicle.LowVoltageSystemState";

/// Actuator outputs — direction indicators.
const LEFT_INDICATOR: VssPath = "Body.Lights.DirectionIndicator.Left.IsSignaling";
const RIGHT_INDICATOR: VssPath = "Body.Lights.DirectionIndicator.Right.IsSignaling";

/// Lamp feedback signals from BlinkRelay — used to count flashes.
const LEFT_LAMP_FRONT: VssPath = "Body.Lights.DirectionIndicator.Left.Lamp.Front.IsOn";
const RIGHT_LAMP_FRONT: VssPath = "Body.Lights.DirectionIndicator.Right.Lamp.Front.IsOn";

/// Returns true when ignition is ON or START (turn signals allowed).
fn is_ignition_on(val: &SignalValue) -> bool {
    matches!(val, SignalValue::String(s) if s == "ON" || s == "START")
}

/// Which side is in comfort-blink countdown.
#[derive(Debug, Clone, Copy, PartialEq)]
enum ComfortBlink {
    None,
    Left(u8),  // remaining flashes
    Right(u8), // remaining flashes
}

impl ComfortBlink {
    fn is_active(&self) -> bool {
        !matches!(self, ComfortBlink::None)
    }

    fn is_left(&self) -> bool {
        matches!(self, ComfortBlink::Left(_))
    }

    fn is_right(&self) -> bool {
        matches!(self, ComfortBlink::Right(_))
    }
}

pub struct TurnIndicator<B: SignalBus> {
    arbiter: Arc<DomainArbiter>,
    bus: Arc<B>,
    config: Arc<PlatformConfig>,
}

impl<B: SignalBus> TurnIndicator<B> {
    pub fn new(arbiter: Arc<DomainArbiter>, bus: Arc<B>) -> Self {
        Self {
            arbiter,
            bus,
            config: PlatformConfig::defaults(),
        }
    }

    pub fn with_config(
        arbiter: Arc<DomainArbiter>,
        bus: Arc<B>,
        config: Arc<PlatformConfig>,
    ) -> Self {
        Self {
            arbiter,
            bus,
            config,
        }
    }

    /// Main run loop. Call via `tokio::spawn(turn.run())`.
    ///
    /// Three-phase state machine:
    /// - **Ignition OFF**: ignores stalk inputs, waits for ON/START.
    /// - **Ignition ON**: processes stalk inputs normally. If ignition
    ///   leaves ON/START, deactivates both indicators and returns to
    ///   the OFF phase. When ignition returns, re-evaluates the stalk.
    /// - **Comfort blink**: stalk returned to OFF while signaling;
    ///   indicator stays active while counting down remaining flashes.
    pub async fn run(self) {
        tracing::info!("TurnIndicator feature started");

        let mut stalk_stream = self.bus.subscribe(TURN_STALK).await;
        let mut power_stream = self.bus.subscribe(POWER_STATE).await;
        let mut left_lamp_stream = self.bus.subscribe(LEFT_LAMP_FRONT).await;
        let mut right_lamp_stream = self.bus.subscribe(RIGHT_LAMP_FRONT).await;

        // Track the last stalk position so we can re-apply it when
        // ignition returns to ON.
        let mut last_stalk: String = "OFF".to_string();
        let mut ignition_on = false;
        // Which side is currently active via the stalk (not comfort blink).
        let mut active_side: Option<&str> = None;
        // Number of complete flash cycles counted since the stalk was
        // engaged. Reset to 0 whenever a new direction is activated.
        let mut flashes_counted: u8 = 0;
        // Comfort blink countdown state (remaining flashes after stalk OFF).
        let mut comfort = ComfortBlink::None;
        // Track lamp state for edge detection (we count on→off transitions).
        let mut left_lamp_was_on = false;
        let mut right_lamp_was_on = false;

        let flash_count = self.config.lane_change_flash_count();

        loop {
            select! {
                Some(val) = stalk_stream.next() => {
                    // Always track the stalk position, even when ignition is off.
                    if let SignalValue::String(s) = &val {
                        last_stalk = s.clone();
                    }

                    if !ignition_on {
                        tracing::debug!(stalk = %last_stalk, "TurnIndicator: stalk change ignored (ignition off)");
                        continue;
                    }

                    match last_stalk.as_str() {
                        "LEFT" => {
                            // Cancel any comfort blink (including same side — driver
                            // re-engaged the stalk) and activate left.
                            comfort = ComfortBlink::None;
                            flashes_counted = 0;
                            active_side = Some("LEFT");
                            self.set_both(true, false).await;
                        }
                        "RIGHT" => {
                            comfort = ComfortBlink::None;
                            flashes_counted = 0;
                            active_side = Some("RIGHT");
                            self.set_both(false, true).await;
                        }
                        _ => {
                            // Stalk returned to OFF.
                            if let Some(side) = active_side.take() {
                                let remaining = flash_count.saturating_sub(flashes_counted);
                                if remaining > 0 {
                                    // Quick tap — fewer than N flashes so far.
                                    // Continue for the remaining flashes.
                                    comfort = match side {
                                        "LEFT" => ComfortBlink::Left(remaining),
                                        "RIGHT" => ComfortBlink::Right(remaining),
                                        _ => ComfortBlink::None,
                                    };
                                    tracing::info!(
                                        side,
                                        flashes_so_far = flashes_counted,
                                        remaining,
                                        "TurnIndicator: comfort blink started"
                                    );
                                    // Keep the arbiter claim active — don't release yet.
                                } else {
                                    // Already completed N+ flashes, or comfort
                                    // blink disabled (flash_count=0) — release
                                    // immediately.
                                    tracing::debug!(
                                        side,
                                        flashes_so_far = flashes_counted,
                                        "TurnIndicator: stalk OFF, no comfort blink needed"
                                    );
                                    flashes_counted = 0;
                                    self.set_both(false, false).await;
                                }
                            } else if comfort.is_active() {
                                // Stalk went OFF again while already in comfort blink — no-op.
                            } else {
                                // No active indicator — just release.
                                self.set_both(false, false).await;
                            }
                        }
                    }
                }
                Some(val) = power_stream.next() => {
                    let was_on = ignition_on;
                    ignition_on = is_ignition_on(&val);

                    if was_on && !ignition_on {
                        // Ignition left ON/START — deactivate immediately,
                        // cancel any comfort blink.
                        tracing::info!(state = ?val, "TurnIndicator: ignition off, deactivating");
                        comfort = ComfortBlink::None;
                        active_side = None;
                        flashes_counted = 0;
                        self.set_both(false, false).await;
                    } else if !was_on && ignition_on {
                        // Ignition entered ON/START — re-apply current stalk position.
                        tracing::info!(state = ?val, stalk = %last_stalk, "TurnIndicator: ignition on, re-evaluating stalk");
                        self.apply_stalk(&last_stalk, &mut active_side).await;
                    }
                }
                Some(val) = left_lamp_stream.next() => {
                    let is_on = matches!(val, SignalValue::Bool(true));
                    // Count falling edges (on → off = one complete flash).
                    if left_lamp_was_on && !is_on {
                        if comfort.is_left() {
                            // Counting during comfort blink countdown.
                            if let ComfortBlink::Left(ref mut remaining) = comfort {
                                *remaining = remaining.saturating_sub(1);
                                tracing::debug!(remaining = *remaining, "TurnIndicator: left comfort flash counted");
                                if *remaining == 0 {
                                    tracing::info!("TurnIndicator: left comfort blink complete");
                                    comfort = ComfortBlink::None;
                                    flashes_counted = 0;
                                    self.set_both(false, false).await;
                                }
                            }
                        } else if active_side == Some("LEFT") {
                            // Counting while stalk is held — for tip-to-signal logic.
                            flashes_counted = flashes_counted.saturating_add(1);
                            tracing::debug!(flashes_counted, "TurnIndicator: left flash while stalk held");
                        }
                    }
                    left_lamp_was_on = is_on;
                }
                Some(val) = right_lamp_stream.next() => {
                    let is_on = matches!(val, SignalValue::Bool(true));
                    if right_lamp_was_on && !is_on {
                        if comfort.is_right() {
                            if let ComfortBlink::Right(ref mut remaining) = comfort {
                                *remaining = remaining.saturating_sub(1);
                                tracing::debug!(remaining = *remaining, "TurnIndicator: right comfort flash counted");
                                if *remaining == 0 {
                                    tracing::info!("TurnIndicator: right comfort blink complete");
                                    comfort = ComfortBlink::None;
                                    flashes_counted = 0;
                                    self.set_both(false, false).await;
                                }
                            }
                        } else if active_side == Some("RIGHT") {
                            flashes_counted = flashes_counted.saturating_add(1);
                            tracing::debug!(flashes_counted, "TurnIndicator: right flash while stalk held");
                        }
                    }
                    right_lamp_was_on = is_on;
                }
                else => break,
            }
        }

        tracing::warn!("TurnIndicator: streams closed, exiting");
    }

    /// Translate stalk position to indicator requests.
    async fn apply_stalk(&self, stalk: &str, active_side: &mut Option<&'static str>) {
        let (left, right) = match stalk {
            "LEFT" => {
                *active_side = Some("LEFT");
                (true, false)
            }
            "RIGHT" => {
                *active_side = Some("RIGHT");
                (false, true)
            }
            _ => {
                *active_side = None;
                (false, false)
            }
        };

        tracing::debug!(left, right, stalk, "turn stalk applied");
        self.set_both(left, right).await;
    }

    /// Request both indicator states through the arbiter.
    ///
    /// Claim the active side at MEDIUM priority; release the inactive side
    /// so higher-priority claims (e.g. Hazard) are not blocked.
    async fn set_both(&self, left: bool, right: bool) {
        for (signal, active) in [(LEFT_INDICATOR, left), (RIGHT_INDICATOR, right)] {
            let result = if active {
                self.arbiter
                    .request(ActuatorRequest {
                        signal,
                        value: SignalValue::Bool(true),
                        priority: Priority::Medium,
                        feature_id: FeatureId::TurnIndicator,
                    })
                    .await
            } else {
                self.arbiter.release(signal, FeatureId::TurnIndicator).await
            };

            if let Err(e) = result {
                tracing::error!(error = %e, "TurnIndicator: arbiter op failed");
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::lighting_arbiter;
    use crate::plant_models::blink_relay::BlinkRelay;
    use std::time::Duration;
    use tokio::time::{advance, sleep};

    /// Helper: set up TurnIndicator + BlinkRelay with ignition ON (normal operation).
    /// Uses paused time for deterministic blink counting.
    async fn setup() -> (
        Arc<MockBus>,
        Arc<DomainArbiter>,
        tokio::task::JoinHandle<()>,
    ) {
        let bus = Arc::new(MockBus::new());
        let (arbiter, arbiter_fut) = lighting_arbiter(Arc::clone(&bus));
        tokio::spawn(arbiter_fut);
        let arbiter = Arc::new(arbiter);

        // Spawn BlinkRelay plant model so we get lamp feedback signals
        let relay = BlinkRelay::new(Arc::clone(&bus));
        tokio::spawn(relay.run());

        let feature = TurnIndicator::new(Arc::clone(&arbiter), Arc::clone(&bus));
        let handle = tokio::spawn(feature.run());
        tokio::task::yield_now().await;

        // Turn on ignition so the feature is active
        bus.inject(POWER_STATE, s("ON"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(20)).await;
        bus.clear_history();

        (bus, arbiter, handle)
    }

    /// Setup with a custom flash count config.
    async fn setup_with_flash_count(
        count: u8,
    ) -> (
        Arc<MockBus>,
        Arc<DomainArbiter>,
        tokio::task::JoinHandle<()>,
    ) {
        let bus = Arc::new(MockBus::new());
        let (arbiter, arbiter_fut) = lighting_arbiter(Arc::clone(&bus));
        tokio::spawn(arbiter_fut);
        let arbiter = Arc::new(arbiter);

        let relay = BlinkRelay::new(Arc::clone(&bus));
        tokio::spawn(relay.run());

        let cfg = PlatformConfig::defaults_with_lane_change_flash_count(count);
        let feature = TurnIndicator::with_config(
            Arc::clone(&arbiter),
            Arc::clone(&bus),
            cfg,
        );
        let handle = tokio::spawn(feature.run());
        tokio::task::yield_now().await;

        bus.inject(POWER_STATE, s("ON"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(20)).await;
        bus.clear_history();

        (bus, arbiter, handle)
    }

    /// Helper: set up TurnIndicator WITHOUT turning on ignition.
    async fn setup_ignition_off() -> (
        Arc<MockBus>,
        Arc<DomainArbiter>,
        tokio::task::JoinHandle<()>,
    ) {
        let bus = Arc::new(MockBus::new());
        let (arbiter, arbiter_fut) = lighting_arbiter(Arc::clone(&bus));
        tokio::spawn(arbiter_fut);
        let arbiter = Arc::new(arbiter);

        let relay = BlinkRelay::new(Arc::clone(&bus));
        tokio::spawn(relay.run());

        let feature = TurnIndicator::new(Arc::clone(&arbiter), Arc::clone(&bus));
        let handle = tokio::spawn(feature.run());
        tokio::task::yield_now().await;

        (bus, arbiter, handle)
    }

    fn s(val: &str) -> SignalValue {
        SignalValue::String(val.to_string())
    }

    /// Advance time enough for N complete flash cycles at the normal
    /// blink rate (333ms half-period = 666ms per flash).
    async fn advance_flashes(n: u32) {
        for _ in 0..n {
            // ON half-period
            advance(Duration::from_millis(333)).await;
            tokio::task::yield_now().await;
            // OFF half-period
            advance(Duration::from_millis(333)).await;
            tokio::task::yield_now().await;
        }
    }

    /// Settle: yield enough times for all tasks to process.
    async fn settle() {
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
        advance(Duration::from_millis(1)).await;
        for _ in 0..4 {
            tokio::task::yield_now().await;
        }
    }

    // -----------------------------------------------------------------------
    // Normal operation (ignition ON) — existing tests
    // -----------------------------------------------------------------------

    #[tokio::test(start_paused = true)]
    async fn stalk_left_activates_left_indicator() {
        let (bus, _arb, _handle) = setup().await;

        bus.inject(TURN_STALK, s("LEFT"));
        settle().await;

        let history = bus.history();
        assert!(
            history
                .iter()
                .any(|(sig, val)| { *sig == LEFT_INDICATOR && *val == SignalValue::Bool(true) }),
            "left indicator should be TRUE, history: {:?}",
            history
        );
        assert!(
            !history
                .iter()
                .any(|(sig, val)| { *sig == RIGHT_INDICATOR && *val == SignalValue::Bool(true) }),
            "right indicator should never be TRUE, history: {:?}",
            history
        );
    }

    #[tokio::test(start_paused = true)]
    async fn stalk_right_activates_right_indicator() {
        let (bus, _arb, _handle) = setup().await;

        bus.inject(TURN_STALK, s("RIGHT"));
        settle().await;

        let history = bus.history();
        assert!(
            history
                .iter()
                .any(|(sig, val)| { *sig == RIGHT_INDICATOR && *val == SignalValue::Bool(true) }),
            "right indicator should be TRUE, history: {:?}",
            history
        );
        assert!(
            !history
                .iter()
                .any(|(sig, val)| { *sig == LEFT_INDICATOR && *val == SignalValue::Bool(true) }),
            "left indicator should never be TRUE, history: {:?}",
            history
        );
    }

    #[tokio::test(start_paused = true)]
    async fn stalk_off_enters_comfort_blink() {
        // With default config (3 flashes), stalk OFF should NOT immediately
        // release — the indicator stays signaling during comfort blink.
        let (bus, _arb, _handle) = setup().await;

        bus.inject(TURN_STALK, s("LEFT"));
        settle().await;

        bus.clear_history();
        bus.inject(TURN_STALK, s("OFF"));
        settle().await;

        // Left indicator should still be TRUE (comfort blink active)
        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(true)),
            "left indicator should remain TRUE during comfort blink"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn comfort_blink_completes_after_configured_flashes() {
        let (bus, _arb, _handle) = setup().await;

        // Activate left turn
        bus.inject(TURN_STALK, s("LEFT"));
        settle().await;

        // Return stalk to OFF — enters comfort blink (3 flashes)
        bus.inject(TURN_STALK, s("OFF"));
        settle().await;

        // Advance through 3 complete flash cycles (on+off each)
        advance_flashes(3).await;
        settle().await;

        // After 3 flashes, the indicator should be released (FALSE)
        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(false)),
            "left indicator should be FALSE after 3 comfort flashes"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn comfort_blink_still_active_before_count_reached() {
        let (bus, _arb, _handle) = setup().await;

        bus.inject(TURN_STALK, s("LEFT"));
        settle().await;

        bus.inject(TURN_STALK, s("OFF"));
        settle().await;

        // Only advance 2 flashes — should still be signaling
        advance_flashes(2).await;
        settle().await;

        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(true)),
            "left indicator should remain TRUE with 1 flash remaining"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn comfort_blink_cancelled_by_opposite_stalk() {
        let (bus, _arb, _handle) = setup().await;

        // Activate left, then OFF (comfort blink starts)
        bus.inject(TURN_STALK, s("LEFT"));
        settle().await;
        bus.inject(TURN_STALK, s("OFF"));
        settle().await;

        // Before comfort blink completes, engage RIGHT stalk
        advance_flashes(1).await;
        settle().await;

        bus.clear_history();
        bus.inject(TURN_STALK, s("RIGHT"));
        settle().await;

        // Left should be released, right should be active
        let history = bus.history();
        assert!(
            history
                .iter()
                .any(|(sig, val)| { *sig == LEFT_INDICATOR && *val == SignalValue::Bool(false) }),
            "left should be FALSE after opposite stalk, history: {:?}",
            history
        );
        assert!(
            history
                .iter()
                .any(|(sig, val)| { *sig == RIGHT_INDICATOR && *val == SignalValue::Bool(true) }),
            "right should be TRUE after opposite stalk, history: {:?}",
            history
        );
    }

    #[tokio::test(start_paused = true)]
    async fn comfort_blink_cancelled_by_ignition_off() {
        let (bus, _arb, _handle) = setup().await;

        // Activate left, then OFF (comfort blink starts)
        bus.inject(TURN_STALK, s("LEFT"));
        settle().await;
        bus.inject(TURN_STALK, s("OFF"));
        settle().await;

        // Before comfort blink completes, turn ignition off
        advance_flashes(1).await;
        settle().await;

        bus.clear_history();
        bus.inject(POWER_STATE, s("OFF"));
        settle().await;

        // Left should be immediately released
        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(false)),
            "left should be FALSE after ignition off"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn comfort_blink_disabled_when_flash_count_zero() {
        let (bus, _arb, _handle) = setup_with_flash_count(0).await;

        bus.inject(TURN_STALK, s("LEFT"));
        settle().await;

        bus.clear_history();
        bus.inject(TURN_STALK, s("OFF"));
        settle().await;

        // With flash_count=0, should release immediately
        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(false)),
            "left should be immediately FALSE when comfort blink disabled"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn left_to_right_switches_indicators() {
        let (bus, _arb, _handle) = setup().await;

        bus.inject(TURN_STALK, s("LEFT"));
        settle().await;

        bus.clear_history();
        bus.inject(TURN_STALK, s("RIGHT"));
        settle().await;

        let history = bus.history();
        assert!(
            history
                .iter()
                .any(|(sig, val)| { *sig == LEFT_INDICATOR && *val == SignalValue::Bool(false) }),
            "left should turn OFF when switching to RIGHT, history: {:?}",
            history
        );
        assert!(
            history
                .iter()
                .any(|(sig, val)| { *sig == RIGHT_INDICATOR && *val == SignalValue::Bool(true) }),
            "right should turn ON when switching to RIGHT, history: {:?}",
            history
        );
    }

    #[tokio::test(start_paused = true)]
    async fn hazard_overrides_turn_signal() {
        let (bus, arb, _handle) = setup().await;

        // Turn LEFT is active
        bus.inject(TURN_STALK, s("LEFT"));
        settle().await;

        // Hazard engages at HIGH — overrides the MEDIUM turn request
        use crate::features::hazard_lighting::HazardLighting;
        let hazard = HazardLighting::new(Arc::clone(&arb), Arc::clone(&bus));
        tokio::spawn(hazard.run());
        settle().await;

        bus.clear_history();
        bus.inject("Body.Switches.Hazard.IsEngaged", SignalValue::Bool(true));
        settle().await;

        let history = bus.history();
        // Both indicators should be TRUE (hazard HIGH wins over turn MEDIUM)
        assert!(
            history
                .iter()
                .any(|(sig, val)| { *sig == RIGHT_INDICATOR && *val == SignalValue::Bool(true) }),
            "hazard should override: right should be TRUE, history: {:?}",
            history
        );
    }

    // -----------------------------------------------------------------------
    // Ignition gating (REQ-TURN-008)
    // -----------------------------------------------------------------------

    #[tokio::test(start_paused = true)]
    async fn stalk_ignored_when_ignition_off() {
        let (bus, _arb, _handle) = setup_ignition_off().await;

        // Stalk to LEFT while ignition is off
        bus.inject(TURN_STALK, s("LEFT"));
        settle().await;

        let history = bus.history();
        assert!(
            !history
                .iter()
                .any(|(sig, val)| { *sig == LEFT_INDICATOR && *val == SignalValue::Bool(true) }),
            "left indicator should NOT activate when ignition off, history: {:?}",
            history
        );
    }

    #[tokio::test(start_paused = true)]
    async fn stalk_ignored_when_ignition_acc() {
        let (bus, _arb, _handle) = setup_ignition_off().await;

        // Set ignition to ACC (not ON)
        bus.inject(POWER_STATE, s("ACC"));
        settle().await;
        bus.clear_history();

        // Stalk to LEFT — should be ignored
        bus.inject(TURN_STALK, s("LEFT"));
        settle().await;

        let history = bus.history();
        assert!(
            !history
                .iter()
                .any(|(sig, val)| { *sig == LEFT_INDICATOR && *val == SignalValue::Bool(true) }),
            "left indicator should NOT activate in ACC, history: {:?}",
            history
        );
    }

    #[tokio::test(start_paused = true)]
    async fn ignition_off_deactivates_active_turn() {
        let (bus, _arb, _handle) = setup().await;

        // Activate LEFT turn
        bus.inject(TURN_STALK, s("LEFT"));
        settle().await;

        bus.clear_history();

        // Ignition goes to OFF
        bus.inject(POWER_STATE, s("OFF"));
        settle().await;

        let history = bus.history();
        assert!(
            history
                .iter()
                .any(|(sig, val)| { *sig == LEFT_INDICATOR && *val == SignalValue::Bool(false) }),
            "left should deactivate when ignition goes OFF, history: {:?}",
            history
        );
    }

    #[tokio::test(start_paused = true)]
    async fn ignition_acc_deactivates_active_turn() {
        let (bus, _arb, _handle) = setup().await;

        // Activate RIGHT turn
        bus.inject(TURN_STALK, s("RIGHT"));
        settle().await;

        bus.clear_history();

        // Ignition goes to ACC
        bus.inject(POWER_STATE, s("ACC"));
        settle().await;

        let history = bus.history();
        assert!(
            history
                .iter()
                .any(|(sig, val)| { *sig == RIGHT_INDICATOR && *val == SignalValue::Bool(false) }),
            "right should be FALSE when ignition goes to ACC, history: {:?}",
            history
        );
    }

    #[tokio::test(start_paused = true)]
    async fn ignition_on_reactivates_stalk_position() {
        let (bus, _arb, _handle) = setup_ignition_off().await;

        // Move stalk to LEFT while ignition off (should be tracked but not applied)
        bus.inject(TURN_STALK, s("LEFT"));
        settle().await;

        bus.clear_history();

        // Turn ignition ON — should re-apply the stalk position
        bus.inject(POWER_STATE, s("ON"));
        settle().await;

        let history = bus.history();
        assert!(
            history
                .iter()
                .any(|(sig, val)| { *sig == LEFT_INDICATOR && *val == SignalValue::Bool(true) }),
            "left should activate when ignition returns to ON, history: {:?}",
            history
        );
    }

    #[tokio::test(start_paused = true)]
    async fn start_state_also_enables_turn() {
        let (bus, _arb, _handle) = setup_ignition_off().await;

        bus.inject(TURN_STALK, s("RIGHT"));
        settle().await;

        bus.clear_history();

        // START should also enable turn signals
        bus.inject(POWER_STATE, s("START"));
        settle().await;

        let history = bus.history();
        assert!(
            history
                .iter()
                .any(|(sig, val)| { *sig == RIGHT_INDICATOR && *val == SignalValue::Bool(true) }),
            "right should activate in START state, history: {:?}",
            history
        );
    }

    // -----------------------------------------------------------------------
    // Tip-to-signal: long hold releases immediately
    // -----------------------------------------------------------------------

    #[tokio::test(start_paused = true)]
    async fn long_hold_releases_immediately_on_stalk_off() {
        let (bus, _arb, _handle) = setup().await;

        // Activate left turn
        bus.inject(TURN_STALK, s("LEFT"));
        settle().await;

        // Hold stalk for 3 complete flash cycles
        advance_flashes(3).await;
        settle().await;

        // Return stalk to OFF — should release immediately (no comfort blink)
        bus.clear_history();
        bus.inject(TURN_STALK, s("OFF"));
        settle().await;

        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(false)),
            "left should be FALSE immediately — held long enough, no comfort blink"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn long_hold_more_than_configured_releases_immediately() {
        let (bus, _arb, _handle) = setup().await;

        bus.inject(TURN_STALK, s("LEFT"));
        settle().await;

        // Hold stalk for 5 complete flash cycles (more than the 3 configured)
        advance_flashes(5).await;
        settle().await;

        bus.clear_history();
        bus.inject(TURN_STALK, s("OFF"));
        settle().await;

        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(false)),
            "left should be FALSE immediately — held well past configured count"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn partial_hold_gets_remaining_comfort_flashes() {
        // Hold stalk for 1 flash, then release — should get 2 more comfort flashes
        let (bus, _arb, _handle) = setup().await;

        bus.inject(TURN_STALK, s("LEFT"));
        settle().await;

        // Hold for 1 flash
        advance_flashes(1).await;
        settle().await;

        // Release stalk
        bus.inject(TURN_STALK, s("OFF"));
        settle().await;

        // Should still be signaling (2 remaining comfort flashes)
        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(true)),
            "left should remain TRUE — 2 comfort flashes remaining"
        );

        // Advance 2 more flashes to complete the total of 3
        advance_flashes(2).await;
        settle().await;

        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(false)),
            "left should be FALSE after 3 total flashes (1 held + 2 comfort)"
        );
    }

    // -----------------------------------------------------------------------
    // Comfort blink with right indicator
    // -----------------------------------------------------------------------

    #[tokio::test(start_paused = true)]
    async fn comfort_blink_works_for_right_indicator() {
        let (bus, _arb, _handle) = setup().await;

        bus.inject(TURN_STALK, s("RIGHT"));
        settle().await;

        bus.inject(TURN_STALK, s("OFF"));
        settle().await;

        // Still signaling during comfort blink
        assert_eq!(
            bus.latest_value(RIGHT_INDICATOR),
            Some(SignalValue::Bool(true)),
            "right indicator should remain TRUE during comfort blink"
        );

        advance_flashes(3).await;
        settle().await;

        assert_eq!(
            bus.latest_value(RIGHT_INDICATOR),
            Some(SignalValue::Bool(false)),
            "right indicator should be FALSE after 3 comfort flashes"
        );
    }
}
