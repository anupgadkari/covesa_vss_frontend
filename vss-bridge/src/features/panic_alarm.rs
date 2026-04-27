//! Panic Alarm — synchronized direction-indicator blink + horn chirps
//! triggered by a paired keyfob's PANIC button (or any other source that
//! engages `Body.Switches.Panic.IsEngaged`).
//!
//! # Inputs
//!   - `Body.Switches.Panic.IsEngaged` (Bool) — toggled by RKE on a
//!     paired-keyfob PANIC press, or by other sources (HMI test button,
//!     telematics remote panic, intrusion sensor).
//!
//! # Outputs
//!   - `Body.Lights.DirectionIndicator.Left.IsSignaling`  via Lighting arbiter @ HIGH
//!   - `Body.Lights.DirectionIndicator.Right.IsSignaling` via Lighting arbiter @ HIGH
//!   - `Body.Horn.IsActive`                                via Horn arbiter @ HIGH
//!   - `Vehicle.Body.Alarm.IsActive` (Bool) — direct publish (single-owner
//!     status flag for telematics / HMI / fault logging).
//!
//! # Behaviour
//! When `IsEngaged` transitions FALSE→TRUE, a background task runs a
//! 1 Hz pulse loop where lights and horn share the same on/off edges:
//!
//! ```text
//! ┌── ON_MS ──┐┌── OFF_MS ─┐┌── ON_MS ──┐┌── OFF_MS ─┐ …
//! lights:     ON           OFF          ON           OFF
//! horn:       ON           OFF          ON           OFF
//! ```
//!
//! When `IsEngaged` transitions TRUE→FALSE, the loop is aborted, the
//! lighting + horn arbiter claims are released, and
//! `Vehicle.Body.Alarm.IsActive` is published `false`.
//!
//! # Ignition independence
//! Like Hazard, panic alarm is a security feature and operates regardless
//! of `Vehicle.LowVoltageSystemState`.  It must work when the vehicle is
//! parked and locked.
//!
//! # Re-engage idempotence
//! A redundant TRUE while already engaged is a no-op.  A FALSE while
//! disengaged is a no-op.
//!
//! # Cancel-on-unlock
//! Any successful unlock command on the central-lock feedback bus
//! (`Body.Doors.CentralLock.FeedbackRequest = "unlock"`) cancels the
//! alarm — matches typical OEM behaviour where returning to the vehicle
//! and unlocking it (RKE / smart entry / phone / BLE / NFC) is treated
//! as proof that the user is back.  When this happens, PanicAlarm
//! self-publishes `Body.Switches.Panic.IsEngaged = false` so the source
//! of truth stays consistent with internal state.

use std::sync::Arc;

use futures::StreamExt;
use tokio::task::JoinHandle;
use tokio::time::{sleep, Duration};

use crate::arbiter::{ActuatorRequest, DomainArbiter, FEEDBACK_REQUEST};
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

// ── Signal constants ───────────────────────────────────────────────────────

const PANIC_SWITCH: VssPath = "Body.Switches.Panic.IsEngaged";

const LEFT_INDICATOR: VssPath = "Body.Lights.DirectionIndicator.Left.IsSignaling";
const RIGHT_INDICATOR: VssPath = "Body.Lights.DirectionIndicator.Right.IsSignaling";
const HORN: VssPath = "Body.Horn.IsActive";
const ALARM_STATUS: VssPath = "Vehicle.Body.Alarm.IsActive";

// ── Pulse cadence ──────────────────────────────────────────────────────────

/// Indicator + horn ON duration per pulse (ms).
const ON_MS: u64 = 400;
/// Indicator + horn OFF duration per pulse (ms).
/// 400 ON + 600 OFF → 1 Hz pulse rate, matches typical OEM panic alarms.
const OFF_MS: u64 = 600;

// ── Feature struct ─────────────────────────────────────────────────────────

pub struct PanicAlarm<B: SignalBus> {
    lighting_arb: Arc<DomainArbiter>,
    horn_arb: Arc<DomainArbiter>,
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> PanicAlarm<B> {
    pub fn new(
        lighting_arb: Arc<DomainArbiter>,
        horn_arb: Arc<DomainArbiter>,
        bus: Arc<B>,
    ) -> Self {
        Self {
            lighting_arb,
            horn_arb,
            bus,
        }
    }

    pub async fn run(self) {
        tracing::info!("PanicAlarm feature started");

        // Cancel-on-unlock watcher — runs independently of the main switch
        // loop.  Whenever an authenticated source publishes
        // `FEEDBACK_REQUEST = "unlock"`, this task injects
        // `PANIC_SWITCH = false` on the bus.  The main loop then sees the
        // transition like any other disengage and tears the alarm down.
        // Doing this is idempotent — a redundant FALSE while already
        // disengaged is a no-op (PanicAlarm dedups same-state edges).
        let bus_for_watcher = Arc::clone(&self.bus);
        tokio::spawn(async move {
            let mut feedback_rx = bus_for_watcher.subscribe(FEEDBACK_REQUEST).await;
            while let Some(val) = feedback_rx.next().await {
                if matches!(&val, SignalValue::String(s) if s == "unlock") {
                    tracing::debug!(
                        "PanicAlarm: unlock feedback observed — synthesising panic cancel"
                    );
                    let _ = bus_for_watcher
                        .publish(PANIC_SWITCH, SignalValue::Bool(false))
                        .await;
                }
            }
        });

        let mut switch_rx = self.bus.subscribe(PANIC_SWITCH).await;
        let mut current: Option<JoinHandle<()>> = None;
        let mut engaged = false;

        while let Some(val) = switch_rx.next().await {
            let want = matches!(val, SignalValue::Bool(true));
            if want == engaged {
                // Idempotent — repeated TRUE/FALSE while already in that state.
                continue;
            }
            engaged = want;

            if engaged {
                tracing::info!("PanicAlarm: ENGAGED — starting blink + chirp loop");
                let _ = self
                    .bus
                    .publish(ALARM_STATUS, SignalValue::Bool(true))
                    .await;

                let lighting = Arc::clone(&self.lighting_arb);
                let horn = Arc::clone(&self.horn_arb);
                current = Some(tokio::spawn(async move {
                    pulse_loop(lighting, horn).await;
                }));
            } else {
                tracing::info!("PanicAlarm: DISENGAGED — stopping alarm");
                if let Some(handle) = current.take() {
                    handle.abort();
                    let _ = handle.await;
                }
                release_all(&self.lighting_arb, &self.horn_arb).await;
                let _ = self
                    .bus
                    .publish(ALARM_STATUS, SignalValue::Bool(false))
                    .await;
            }
        }

        // Bus stream closed — clean up before exiting.
        if let Some(handle) = current.take() {
            handle.abort();
            let _ = handle.await;
        }
        release_all(&self.lighting_arb, &self.horn_arb).await;
        tracing::warn!("PanicAlarm: switch stream closed, exiting");
    }
}

// ── Pulse loop ─────────────────────────────────────────────────────────────

/// Runs forever (until aborted) — alternating ON/OFF claims on both indicators
/// and the horn.  Lights and horn share the same edges so chirps are
/// perfectly synchronized with the flash.
async fn pulse_loop(lighting: Arc<DomainArbiter>, horn: Arc<DomainArbiter>) {
    loop {
        claim_all(&lighting, &horn, true).await;
        sleep(Duration::from_millis(ON_MS)).await;
        claim_all(&lighting, &horn, false).await;
        sleep(Duration::from_millis(OFF_MS)).await;
    }
}

/// Claim both indicators and horn at HIGH priority with the given value.
async fn claim_all(lighting: &Arc<DomainArbiter>, horn: &Arc<DomainArbiter>, on: bool) {
    for &sig in &[LEFT_INDICATOR, RIGHT_INDICATOR] {
        let _ = lighting
            .request(ActuatorRequest {
                signal: sig,
                value: SignalValue::Bool(on),
                priority: Priority::High,
                feature_id: FeatureId::PanicAlarm,
            })
            .await;
    }
    let _ = horn
        .request(ActuatorRequest {
            signal: HORN,
            value: SignalValue::Bool(on),
            priority: Priority::High,
            feature_id: FeatureId::PanicAlarm,
        })
        .await;
}

/// Release all PanicAlarm claims (lighting indicators + horn).
async fn release_all(lighting: &Arc<DomainArbiter>, horn: &Arc<DomainArbiter>) {
    for &sig in &[LEFT_INDICATOR, RIGHT_INDICATOR] {
        let _ = lighting.release(sig, FeatureId::PanicAlarm).await;
    }
    let _ = horn.release(HORN, FeatureId::PanicAlarm).await;
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::{horn_arbiter, lighting_arbiter};
    use tokio::time::advance;

    /// Spin up the lighting + horn arbiters and the PanicAlarm feature.
    /// Returns the bus and the feature task handle.
    async fn setup() -> (Arc<MockBus>, tokio::task::JoinHandle<()>) {
        let bus = Arc::new(MockBus::new());
        let (light_arb, light_fut) = lighting_arbiter(Arc::clone(&bus));
        let (horn_arb, horn_fut) = horn_arbiter(Arc::clone(&bus));
        tokio::spawn(light_fut);
        tokio::spawn(horn_fut);
        let light_arb = Arc::new(light_arb);
        let horn_arb = Arc::new(horn_arb);

        let feature = PanicAlarm::new(
            Arc::clone(&light_arb),
            Arc::clone(&horn_arb),
            Arc::clone(&bus),
        );
        let handle = tokio::spawn(feature.run());

        // Yield enough times for every spawned task to reach its first
        // .subscribe().await so injections aren't dropped.
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
        (bus, handle)
    }

    /// Advance virtual time + yield so timers + arbiters settle.
    async fn settle(ms: u64) {
        advance(Duration::from_millis(ms)).await;
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
    }

    #[tokio::test(start_paused = true)]
    async fn engage_publishes_alarm_active_true_and_starts_pulses() {
        let (bus, _h) = setup().await;

        bus.inject(PANIC_SWITCH, SignalValue::Bool(true));
        settle(1).await;

        // Status flag asserted on engage transition.
        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(true)),
            "Vehicle.Body.Alarm.IsActive should be TRUE after engage"
        );

        // First ON-edge: indicators + horn all TRUE.
        settle(1).await;
        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(true))
        );
        assert_eq!(
            bus.latest_value(RIGHT_INDICATOR),
            Some(SignalValue::Bool(true))
        );
        assert_eq!(bus.latest_value(HORN), Some(SignalValue::Bool(true)));
    }

    #[tokio::test(start_paused = true)]
    async fn disengage_releases_all_outputs() {
        let (bus, _h) = setup().await;

        bus.inject(PANIC_SWITCH, SignalValue::Bool(true));
        settle(1).await;

        bus.inject(PANIC_SWITCH, SignalValue::Bool(false));
        settle(1).await;

        // After disengage, status flag, indicators, and horn all default-off.
        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(false)),
            "ALARM_STATUS should fall to FALSE on disengage"
        );
        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(false))
        );
        assert_eq!(
            bus.latest_value(RIGHT_INDICATOR),
            Some(SignalValue::Bool(false))
        );
        assert_eq!(bus.latest_value(HORN), Some(SignalValue::Bool(false)));
    }

    #[tokio::test(start_paused = true)]
    async fn lights_and_horn_pulse_synchronously() {
        let (bus, _h) = setup().await;
        bus.inject(PANIC_SWITCH, SignalValue::Bool(true));
        settle(1).await;

        // Capture the pattern over a couple of cycles.  ON window first.
        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(true))
        );
        assert_eq!(bus.latest_value(HORN), Some(SignalValue::Bool(true)));

        // Halfway through ON — still on.
        settle(ON_MS / 2).await;
        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(true))
        );
        assert_eq!(bus.latest_value(HORN), Some(SignalValue::Bool(true)));

        // Just past ON window → enter OFF window: both off.
        settle(ON_MS / 2 + 5).await;
        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(false))
        );
        assert_eq!(
            bus.latest_value(RIGHT_INDICATOR),
            Some(SignalValue::Bool(false))
        );
        assert_eq!(bus.latest_value(HORN), Some(SignalValue::Bool(false)));

        // Just past OFF window → next ON window: all on again.
        settle(OFF_MS).await;
        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(true))
        );
        assert_eq!(
            bus.latest_value(RIGHT_INDICATOR),
            Some(SignalValue::Bool(true))
        );
        assert_eq!(bus.latest_value(HORN), Some(SignalValue::Bool(true)));
    }

    #[tokio::test(start_paused = true)]
    async fn alarm_status_does_not_duty_cycle() {
        let (bus, _h) = setup().await;
        bus.inject(PANIC_SWITCH, SignalValue::Bool(true));
        settle(1).await;

        // Run through several pulse cycles and then count Vehicle.Body.Alarm.IsActive
        // publishes — should be exactly one (the initial TRUE), regardless of
        // how many times the lights/horn cycled.
        bus.clear_history();
        settle((ON_MS + OFF_MS) * 3).await;

        let status_publishes = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == ALARM_STATUS)
            .count();
        assert_eq!(
            status_publishes, 0,
            "ALARM_STATUS must not be re-published per pulse, got {status_publishes}"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn works_with_ignition_off() {
        let (bus, _h) = setup().await;

        // Ignition explicitly OFF before engaging panic.
        bus.inject(
            "Vehicle.LowVoltageSystemState",
            SignalValue::String("OFF".into()),
        );
        settle(1).await;

        bus.inject(PANIC_SWITCH, SignalValue::Bool(true));
        settle(1).await;

        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(true))
        );
        assert_eq!(
            bus.latest_value(RIGHT_INDICATOR),
            Some(SignalValue::Bool(true))
        );
        assert_eq!(bus.latest_value(HORN), Some(SignalValue::Bool(true)));
        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(true))
        );
    }

    #[tokio::test(start_paused = true)]
    async fn re_engage_while_running_is_idempotent() {
        let (bus, _h) = setup().await;

        bus.inject(PANIC_SWITCH, SignalValue::Bool(true));
        settle(ON_MS / 2).await;
        bus.clear_history();

        // Inject TRUE again while already engaged.  Should be a no-op:
        // no new ALARM_STATUS publish, and the running pulse loop is not
        // restarted (no extra claim publishes).
        bus.inject(PANIC_SWITCH, SignalValue::Bool(true));
        settle(1).await;

        let status_publishes = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == ALARM_STATUS)
            .count();
        assert_eq!(
            status_publishes, 0,
            "Re-engage must NOT re-publish ALARM_STATUS"
        );

        // Verify the loop is still actively pulsing by stepping one full
        // cycle and checking the latched value at each known transition.
        // Step past the first ON-window remainder + into OFF-window.
        settle(ON_MS).await;
        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(false)),
            "after ON window expires lights should be OFF"
        );
        assert_eq!(
            bus.latest_value(HORN),
            Some(SignalValue::Bool(false)),
            "horn should be OFF in sync with lights"
        );

        // Step past the OFF window into the next ON-window.
        settle(OFF_MS).await;
        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(true)),
            "after OFF window lights should be ON again"
        );
        assert_eq!(
            bus.latest_value(HORN),
            Some(SignalValue::Bool(true)),
            "horn should be ON in sync with lights"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn unlock_feedback_cancels_running_alarm() {
        let (bus, _h) = setup().await;

        // Engage the alarm.
        bus.inject(PANIC_SWITCH, SignalValue::Bool(true));
        settle(1).await;
        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(true))
        );
        assert_eq!(bus.latest_value(HORN), Some(SignalValue::Bool(true)));

        // Simulate a successful authenticated unlock — RKE / smart entry /
        // phone / BLE / NFC all converge on FEEDBACK_REQUEST = "unlock".
        bus.inject(FEEDBACK_REQUEST, SignalValue::String("unlock".into()));
        settle(1).await;

        // Alarm must stop: status flag false, indicators + horn released.
        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(false)),
            "ALARM_STATUS must fall to FALSE on unlock cancel"
        );
        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(false)),
            "indicators must release on unlock cancel"
        );
        assert_eq!(
            bus.latest_value(RIGHT_INDICATOR),
            Some(SignalValue::Bool(false))
        );
        assert_eq!(bus.latest_value(HORN), Some(SignalValue::Bool(false)));

        // PanicAlarm must self-publish the switch FALSE so internal state
        // tracked by RKE / HMI stays in sync.
        assert_eq!(
            bus.latest_value(PANIC_SWITCH),
            Some(SignalValue::Bool(false)),
            "PanicAlarm must self-publish the switch FALSE on unlock cancel"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn lock_feedback_does_not_cancel_alarm() {
        let (bus, _h) = setup().await;

        // Engage the alarm.
        bus.inject(PANIC_SWITCH, SignalValue::Bool(true));
        settle(1).await;
        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(true))
        );

        // A "lock" feedback (e.g. AutoRelock, WalkAwayLock) must NOT cancel
        // an active panic alarm — only "unlock" does.
        bus.inject(FEEDBACK_REQUEST, SignalValue::String("lock".into()));
        settle(1).await;

        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(true)),
            "ALARM_STATUS must remain TRUE on lock-feedback (not unlock)"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn unlock_feedback_when_disengaged_is_state_noop() {
        let (bus, _h) = setup().await;

        // Alarm never engaged.
        bus.clear_history();
        bus.inject(FEEDBACK_REQUEST, SignalValue::String("unlock".into()));
        settle(1).await;

        // The watcher publishes PANIC_SWITCH=false unconditionally on every
        // "unlock" — that's benign because PanicAlarm dedups same-state
        // transitions.  Verify the *state* of the alarm is unchanged:
        //   - no ALARM_STATUS toggle
        //   - no indicator / horn claim transitions
        let h = bus.history();
        assert!(
            !h.iter().any(|(s, _)| *s == ALARM_STATUS),
            "no ALARM_STATUS publish expected when alarm was never engaged: {h:?}"
        );
        assert!(
            !h.iter()
                .any(|(s, v)| *s == LEFT_INDICATOR && *v == SignalValue::Bool(true)),
            "no LEFT_INDICATOR=TRUE expected when alarm was never engaged"
        );
        assert!(
            !h.iter()
                .any(|(s, v)| *s == HORN && *v == SignalValue::Bool(true)),
            "no HORN=TRUE expected when alarm was never engaged"
        );
    }
}
