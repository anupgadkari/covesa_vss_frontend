//! Turn Indicator — activates the correct direction indicator when
//! the driver moves the turn signal stalk.
//!
//! Subscribes to:
//!   - Body.Switches.TurnIndicator.Direction (overlay sensor — stalk)
//!     Values: "OFF", "LEFT", "RIGHT"
//!   - Vehicle.LowVoltageSystemState (ignition / power mode)
//!     Turn signals only operate when ignition is ON or START.
//!
//! Outputs (via Lighting arbiter):
//!   - Body.Lights.DirectionIndicator.Left.IsSignaling  @ MEDIUM (2)
//!   - Body.Lights.DirectionIndicator.Right.IsSignaling @ MEDIUM (2)
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
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

/// Physical turn signal stalk input (overlay signal).
const TURN_STALK: VssPath = "Body.Switches.TurnIndicator.Direction";

/// Power state signal — standard VSS v4.0.
const POWER_STATE: VssPath = "Vehicle.LowVoltageSystemState";

/// Actuator outputs — direction indicators.
const LEFT_INDICATOR: VssPath = "Body.Lights.DirectionIndicator.Left.IsSignaling";
const RIGHT_INDICATOR: VssPath = "Body.Lights.DirectionIndicator.Right.IsSignaling";

/// Returns true when ignition is ON or START (turn signals allowed).
fn is_ignition_on(val: &SignalValue) -> bool {
    matches!(val, SignalValue::String(s) if s == "ON" || s == "START")
}

pub struct TurnIndicator<B: SignalBus> {
    arbiter: Arc<DomainArbiter>,
    bus: Arc<B>,
}

impl<B: SignalBus> TurnIndicator<B> {
    pub fn new(arbiter: Arc<DomainArbiter>, bus: Arc<B>) -> Self {
        Self { arbiter, bus }
    }

    /// Main run loop. Call via `tokio::spawn(turn.run())`.
    ///
    /// Two-phase state machine:
    /// - **Ignition OFF**: ignores stalk inputs, waits for ON/START.
    /// - **Ignition ON**: processes stalk inputs normally. If ignition
    ///   leaves ON/START, deactivates both indicators and returns to
    ///   the OFF phase. When ignition returns, re-evaluates the stalk.
    pub async fn run(self) {
        tracing::info!("TurnIndicator feature started");

        let mut stalk_stream = self.bus.subscribe(TURN_STALK).await;
        let mut power_stream = self.bus.subscribe(POWER_STATE).await;

        // Track the last stalk position so we can re-apply it when
        // ignition returns to ON.
        let mut last_stalk: String = "OFF".to_string();
        let mut ignition_on = false;

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

                    self.apply_stalk(&last_stalk).await;
                }
                Some(val) = power_stream.next() => {
                    let was_on = ignition_on;
                    ignition_on = is_ignition_on(&val);

                    if was_on && !ignition_on {
                        // Ignition left ON/START — deactivate both indicators.
                        tracing::info!(state = ?val, "TurnIndicator: ignition off, deactivating");
                        self.set_both(false, false).await;
                    } else if !was_on && ignition_on {
                        // Ignition entered ON/START — re-apply current stalk position.
                        tracing::info!(state = ?val, stalk = %last_stalk, "TurnIndicator: ignition on, re-evaluating stalk");
                        self.apply_stalk(&last_stalk).await;
                    }
                }
                else => break,
            }
        }

        tracing::warn!("TurnIndicator: streams closed, exiting");
    }

    /// Translate stalk position to indicator requests.
    async fn apply_stalk(&self, stalk: &str) {
        let (left, right) = match stalk {
            "LEFT" => (true, false),
            "RIGHT" => (false, true),
            _ => (false, false),
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
    use std::time::Duration;
    use tokio::time::sleep;

    /// Helper: set up TurnIndicator with ignition ON (normal operation).
    async fn setup() -> (
        Arc<MockBus>,
        Arc<DomainArbiter>,
        tokio::task::JoinHandle<()>,
    ) {
        let bus = Arc::new(MockBus::new());
        let (arbiter, arbiter_fut) = lighting_arbiter(Arc::clone(&bus));
        tokio::spawn(arbiter_fut);
        let arbiter = Arc::new(arbiter);

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

        let feature = TurnIndicator::new(Arc::clone(&arbiter), Arc::clone(&bus));
        let handle = tokio::spawn(feature.run());
        tokio::task::yield_now().await;

        (bus, arbiter, handle)
    }

    fn s(val: &str) -> SignalValue {
        SignalValue::String(val.to_string())
    }

    // -----------------------------------------------------------------------
    // Normal operation (ignition ON)
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn stalk_left_activates_left_indicator() {
        let (bus, _arb, _handle) = setup().await;

        bus.inject(TURN_STALK, s("LEFT"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        let history = bus.history();
        assert!(
            history
                .iter()
                .any(|(sig, val)| { *sig == LEFT_INDICATOR && *val == SignalValue::Bool(true) }),
            "left indicator should be TRUE, history: {:?}",
            history
        );
        // Under claim/release semantics the right side is never claimed, so
        // the arbiter publishes nothing for it (it stays at default-off).
        assert!(
            !history
                .iter()
                .any(|(sig, val)| { *sig == RIGHT_INDICATOR && *val == SignalValue::Bool(true) }),
            "right indicator should never be TRUE, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn stalk_right_activates_right_indicator() {
        let (bus, _arb, _handle) = setup().await;

        bus.inject(TURN_STALK, s("RIGHT"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

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

    #[tokio::test]
    async fn stalk_off_deactivates_both() {
        let (bus, _arb, _handle) = setup().await;

        // Activate left, then go to OFF
        bus.inject(TURN_STALK, s("LEFT"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        bus.clear_history();
        bus.inject(TURN_STALK, s("OFF"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        let history = bus.history();
        // Left was claimed true, so releasing it publishes false via
        // the default-off fallback.
        assert!(
            history
                .iter()
                .any(|(sig, val)| { *sig == LEFT_INDICATOR && *val == SignalValue::Bool(false) }),
            "left should be FALSE after OFF, history: {:?}",
            history
        );
        // Right was never claimed so no false publish is emitted.
        assert!(
            !history.iter().any(|(sig, _)| *sig == RIGHT_INDICATOR),
            "right should not be republished, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn left_to_right_switches_indicators() {
        let (bus, _arb, _handle) = setup().await;

        bus.inject(TURN_STALK, s("LEFT"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        bus.clear_history();
        bus.inject(TURN_STALK, s("RIGHT"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

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

    #[tokio::test]
    async fn hazard_overrides_turn_signal() {
        let (bus, arb, _handle) = setup().await;

        // Turn LEFT is active
        bus.inject(TURN_STALK, s("LEFT"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        // Hazard engages at HIGH — overrides the MEDIUM turn request
        use crate::features::hazard_lighting::HazardLighting;
        let hazard = HazardLighting::new(Arc::clone(&arb), Arc::clone(&bus));
        tokio::spawn(hazard.run());
        tokio::task::yield_now().await;

        bus.clear_history();
        bus.inject("Body.Switches.Hazard.IsEngaged", SignalValue::Bool(true));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

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

    #[tokio::test]
    async fn stalk_ignored_when_ignition_off() {
        let (bus, _arb, _handle) = setup_ignition_off().await;

        // Stalk to LEFT while ignition is off
        bus.inject(TURN_STALK, s("LEFT"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        let history = bus.history();
        assert!(
            !history
                .iter()
                .any(|(sig, val)| { *sig == LEFT_INDICATOR && *val == SignalValue::Bool(true) }),
            "left indicator should NOT activate when ignition off, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn stalk_ignored_when_ignition_acc() {
        let (bus, _arb, _handle) = setup_ignition_off().await;

        // Set ignition to ACC (not ON)
        bus.inject(POWER_STATE, s("ACC"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(20)).await;
        bus.clear_history();

        // Stalk to LEFT — should be ignored
        bus.inject(TURN_STALK, s("LEFT"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        let history = bus.history();
        assert!(
            !history
                .iter()
                .any(|(sig, val)| { *sig == LEFT_INDICATOR && *val == SignalValue::Bool(true) }),
            "left indicator should NOT activate in ACC, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn ignition_off_deactivates_active_turn() {
        let (bus, _arb, _handle) = setup().await;

        // Activate LEFT turn
        bus.inject(TURN_STALK, s("LEFT"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        bus.clear_history();

        // Ignition goes to OFF
        bus.inject(POWER_STATE, s("OFF"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        let history = bus.history();
        assert!(
            history
                .iter()
                .any(|(sig, val)| { *sig == LEFT_INDICATOR && *val == SignalValue::Bool(false) }),
            "left should deactivate when ignition goes OFF, history: {:?}",
            history
        );
        // Right was never claimed by Turn so no publish is emitted for it.
        assert!(
            !history.iter().any(|(sig, _)| *sig == RIGHT_INDICATOR),
            "right should not be republished, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn ignition_acc_deactivates_active_turn() {
        let (bus, _arb, _handle) = setup().await;

        // Activate RIGHT turn
        bus.inject(TURN_STALK, s("RIGHT"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        bus.clear_history();

        // Ignition goes to ACC
        bus.inject(POWER_STATE, s("ACC"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        let history = bus.history();
        // Left was never claimed, no publish for it.
        assert!(
            !history.iter().any(|(sig, _)| *sig == LEFT_INDICATOR),
            "left should not be republished, history: {:?}",
            history
        );
        assert!(
            history
                .iter()
                .any(|(sig, val)| { *sig == RIGHT_INDICATOR && *val == SignalValue::Bool(false) }),
            "right should be FALSE when ignition goes to ACC, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn ignition_on_reactivates_stalk_position() {
        let (bus, _arb, _handle) = setup_ignition_off().await;

        // Move stalk to LEFT while ignition off (should be tracked but not applied)
        bus.inject(TURN_STALK, s("LEFT"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        bus.clear_history();

        // Turn ignition ON — should re-apply the stalk position
        bus.inject(POWER_STATE, s("ON"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        let history = bus.history();
        assert!(
            history
                .iter()
                .any(|(sig, val)| { *sig == LEFT_INDICATOR && *val == SignalValue::Bool(true) }),
            "left should activate when ignition returns to ON, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn start_state_also_enables_turn() {
        let (bus, _arb, _handle) = setup_ignition_off().await;

        bus.inject(TURN_STALK, s("RIGHT"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        bus.clear_history();

        // START should also enable turn signals
        bus.inject(POWER_STATE, s("START"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        let history = bus.history();
        assert!(
            history
                .iter()
                .any(|(sig, val)| { *sig == RIGHT_INDICATOR && *val == SignalValue::Bool(true) }),
            "right should activate in START state, history: {:?}",
            history
        );
    }
}
