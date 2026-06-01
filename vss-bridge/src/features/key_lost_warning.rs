//! Key-Lost Warning — short chime + cluster flag when the last paired
//! key disappears from the cabin while the vehicle is moving.
//!
//! # Trigger
//!
//! Three conditions must all hold at the moment
//! `Vehicle.Controller.Body.PEPS.ApproachKeys` transitions from a value
//! ≥ 1 to exactly 0:
//!
//! 1. `Vehicle.LowVoltageSystemState` is `ON` or `START` (engine
//!    running; the only state in which "the car is moving and the
//!    key just left" is a real problem).
//! 2. `Vehicle.Speed` exceeds `SPEED_THRESHOLD_KMH` (5 km/h —
//!    matches the typical OEM "vehicle is genuinely under way"
//!    threshold; sub-threshold drops are common at parking lots
//!    and don't warrant a chime).
//! 3. The previous `ApproachKeys` reading was ≥ 1 (rising-edge of
//!    "no keys present" — a fresh `0` after another `0` is a
//!    no-op).
//!
//! # Action
//!
//! - Publish `Vehicle.Controller.Starting.KeyLostWarning = true`
//!   (cluster polls this to render its "key lost" icon).
//! - Publish `Vehicle.Controller.Body.Chime.IsActive = true` for
//!   `WARNING_DURATION` (2 s), then publish `false`.
//!
//! # Auto-clear
//!
//! Either the 2 s timer expires, or `ApproachKeys` returns to ≥ 1
//! (the user retrieved the key) — whichever happens first.  Both
//! `KeyLostWarning` and `Chime.IsActive` are published `false`.
//!
//! # Notes on the chime channel
//!
//! The chime path today is a shared `Vehicle.Controller.Body.Chime.IsActive`
//! Bool with no arbiter — `LockFeedback`, `PerimeterAlarm`, and now
//! this feature all publish directly.  When chime usages collide, the
//! plant model's `IsSounding` follows the latest writer.  That's a
//! known limitation tracked elsewhere; for the key-lost case the chime
//! is short and the collision risk is low (the user actively dropping
//! a key while their RKE is also chirping a feedback would be an odd
//! sequence).

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;
use tokio::time::{sleep_until, Duration, Instant};

use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

// ── Signal constants ───────────────────────────────────────────────────────

const APPROACH_KEYS: VssPath = "Vehicle.Controller.Body.PEPS.ApproachKeys";
const SPEED: VssPath = "Vehicle.Speed";
const POWER_STATE: VssPath = "Vehicle.LowVoltageSystemState";
const CHIME: VssPath = "Vehicle.Controller.Body.Chime.IsActive";
const KEY_LOST_WARNING_OUT: VssPath = "Vehicle.Controller.Starting.KeyLostWarning";

// ── Tunables ───────────────────────────────────────────────────────────────

/// Minimum vehicle speed (km/h) at which a key-loss is treated as a
/// real warning rather than a parking-lot blip.  5 km/h matches the
/// typical OEM "under way" threshold used for door-lock auto-engage
/// and brake-hold release.
pub const SPEED_THRESHOLD_KMH: f64 = 5.0;

/// How long the chime + cluster flag stay active after the trigger
/// edge.  Long enough for the driver to hear and react, short
/// enough not to annoy.
pub const WARNING_DURATION: Duration = Duration::from_secs(2);

// ── Helpers ────────────────────────────────────────────────────────────────

fn is_power_on(val: &SignalValue) -> bool {
    matches!(val, SignalValue::String(s) if s == "ON" || s == "START")
}

/// `Vehicle.Speed` may arrive as Float (km/h), Uint16 (km/h), or
/// Uint8 — accept all three so we don't drop edges to type-mismatch.
/// Same heuristic FollowMeHome uses for illuminance.
fn speed_kmh(val: &SignalValue) -> Option<f64> {
    match val {
        SignalValue::Float(v) => Some(*v as f64),
        SignalValue::Uint16(v) => Some(*v as f64),
        SignalValue::Uint8(v) => Some(*v as f64),
        SignalValue::Int16(v) => Some(*v as f64),
        _ => None,
    }
}

fn approach_keys_count(val: &SignalValue) -> Option<u8> {
    match val {
        SignalValue::Uint8(v) => Some(*v),
        _ => None,
    }
}

// ── Feature struct ─────────────────────────────────────────────────────────

pub struct KeyLostWarning<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> KeyLostWarning<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    pub async fn run(self) {
        tracing::info!(
            speed_threshold_kmh = SPEED_THRESHOLD_KMH,
            duration_s = WARNING_DURATION.as_secs(),
            "KeyLostWarning feature started"
        );

        let mut approach_rx = self.bus.subscribe(APPROACH_KEYS).await;
        let mut speed_rx = self.bus.subscribe(SPEED).await;
        let mut power_rx = self.bus.subscribe(POWER_STATE).await;

        // Caches updated on each edge; the trigger fires off
        // ApproachKeys's value change only.
        let mut prev_approach: u8 = 0;
        let mut speed: f64 = 0.0;
        let mut power_on: bool = false;
        let mut warning_deadline: Option<Instant> = None;

        loop {
            let warning_expiry = async {
                match warning_deadline {
                    Some(dl) => sleep_until(dl).await,
                    None => std::future::pending().await,
                }
            };

            select! {
                Some(val) = approach_rx.next() => {
                    let count = match approach_keys_count(&val) {
                        Some(c) => c,
                        None => continue,
                    };
                    let dropped_to_zero = prev_approach >= 1 && count == 0;
                    let recovered_from_zero = prev_approach == 0 && count >= 1;
                    prev_approach = count;

                    // Auto-clear path: the user retrieved a key before
                    // the timer ran out.  Cancel + publish false.
                    if recovered_from_zero && warning_deadline.is_some() {
                        warning_deadline = None;
                        self.clear_warning().await;
                        continue;
                    }

                    // Trigger path: rising edge of "no keys" while
                    // moving + ignition on.  Idempotent — if a warning
                    // is already running, just extend the deadline so
                    // a quick recovery doesn't truncate it.
                    if dropped_to_zero && power_on && speed > SPEED_THRESHOLD_KMH {
                        let already_active = warning_deadline.is_some();
                        warning_deadline = Some(Instant::now() + WARNING_DURATION);
                        if !already_active {
                            tracing::info!(
                                speed_kmh = speed,
                                "KeyLostWarning: ApproachKeys dropped to 0 while \
                                 moving — chime + cluster warning"
                            );
                            self.assert_warning().await;
                        }
                    }
                }
                Some(val) = speed_rx.next() => {
                    if let Some(v) = speed_kmh(&val) {
                        speed = v;
                    }
                }
                Some(val) = power_rx.next() => {
                    let was_on = power_on;
                    power_on = is_power_on(&val);
                    // Ignition off cancels any in-flight warning —
                    // a parked car with no key isn't a warning case.
                    if was_on && !power_on && warning_deadline.is_some() {
                        warning_deadline = None;
                        self.clear_warning().await;
                    }
                }
                _ = warning_expiry => {
                    warning_deadline = None;
                    self.clear_warning().await;
                }
                else => break,
            }
        }

        tracing::info!("KeyLostWarning feature stopped");
    }

    async fn assert_warning(&self) {
        let _ = self
            .bus
            .publish(KEY_LOST_WARNING_OUT, SignalValue::Bool(true))
            .await;
        let _ = self.bus.publish(CHIME, SignalValue::Bool(true)).await;
    }

    async fn clear_warning(&self) {
        let _ = self.bus.publish(CHIME, SignalValue::Bool(false)).await;
        let _ = self
            .bus
            .publish(KEY_LOST_WARNING_OUT, SignalValue::Bool(false))
            .await;
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use tokio::time::advance;

    async fn setup() -> (Arc<MockBus>, tokio::task::JoinHandle<()>) {
        let bus = Arc::new(MockBus::new());
        let feature = KeyLostWarning::new(Arc::clone(&bus));
        let handle = tokio::spawn(feature.run());
        // Let the run loop reach its subscribe() awaits.
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        (bus, handle)
    }

    async fn settle(ms: u64) {
        advance(Duration::from_millis(ms)).await;
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
    }

    /// Drop from 1 → 0 while moving + ignition ON: chime + cluster flag fire.
    #[tokio::test(start_paused = true)]
    async fn drop_to_zero_while_moving_fires_warning() {
        let (bus, _h) = setup().await;

        // Boot conditions: 1 key present, ignition ON, moving at 30 km/h.
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(SPEED, SignalValue::Float(30.0));
        bus.inject(APPROACH_KEYS, SignalValue::Uint8(1));
        settle(1).await;

        // The key disappears.
        bus.inject(APPROACH_KEYS, SignalValue::Uint8(0));
        settle(1).await;

        assert_eq!(
            bus.latest_value(KEY_LOST_WARNING_OUT),
            Some(SignalValue::Bool(true)),
            "KeyLostWarning must be published TRUE on drop"
        );
        assert_eq!(
            bus.latest_value(CHIME),
            Some(SignalValue::Bool(true)),
            "Chime must claim TRUE on drop"
        );

        // After the warning duration, both clear.
        settle(WARNING_DURATION.as_millis() as u64 + 50).await;
        assert_eq!(
            bus.latest_value(KEY_LOST_WARNING_OUT),
            Some(SignalValue::Bool(false)),
            "KeyLostWarning auto-clears after WARNING_DURATION"
        );
        assert_eq!(
            bus.latest_value(CHIME),
            Some(SignalValue::Bool(false)),
            "Chime auto-clears after WARNING_DURATION"
        );
    }

    /// Parked (speed = 0): a drop to zero must NOT fire the warning.
    #[tokio::test(start_paused = true)]
    async fn parked_drop_does_not_fire() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(SPEED, SignalValue::Float(0.0));
        bus.inject(APPROACH_KEYS, SignalValue::Uint8(1));
        settle(1).await;

        bus.inject(APPROACH_KEYS, SignalValue::Uint8(0));
        settle(1).await;

        assert!(
            bus.history()
                .iter()
                .all(|(s, _)| *s != KEY_LOST_WARNING_OUT),
            "no KeyLostWarning publish expected at speed = 0"
        );
        assert!(
            bus.history().iter().all(|(s, _)| *s != CHIME),
            "no Chime publish expected at speed = 0"
        );
    }

    /// Below the 5 km/h threshold: drop must not fire (parking-lot crawl).
    #[tokio::test(start_paused = true)]
    async fn sub_threshold_drop_does_not_fire() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(SPEED, SignalValue::Float(3.0));
        bus.inject(APPROACH_KEYS, SignalValue::Uint8(1));
        settle(1).await;

        bus.inject(APPROACH_KEYS, SignalValue::Uint8(0));
        settle(1).await;

        assert!(
            bus.history()
                .iter()
                .all(|(s, _)| *s != KEY_LOST_WARNING_OUT),
            "no warning expected below SPEED_THRESHOLD_KMH"
        );
    }

    /// Ignition OFF: a drop must not fire even if speed > threshold
    /// (rare edge case — coasting in neutral with the key out).
    #[tokio::test(start_paused = true)]
    async fn ignition_off_drop_does_not_fire() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        bus.inject(SPEED, SignalValue::Float(30.0));
        bus.inject(APPROACH_KEYS, SignalValue::Uint8(1));
        settle(1).await;

        bus.inject(APPROACH_KEYS, SignalValue::Uint8(0));
        settle(1).await;

        assert!(
            bus.history()
                .iter()
                .all(|(s, _)| *s != KEY_LOST_WARNING_OUT),
            "no warning expected with ignition OFF"
        );
    }

    /// Successive 0 readings: only the rising-edge transition fires.
    #[tokio::test(start_paused = true)]
    async fn repeated_zero_does_not_re_fire() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(SPEED, SignalValue::Float(30.0));
        bus.inject(APPROACH_KEYS, SignalValue::Uint8(1));
        settle(1).await;

        bus.inject(APPROACH_KEYS, SignalValue::Uint8(0));
        settle(1).await;
        // Now in warning state — clear the history so we can detect
        // any spurious re-publishes.
        bus.clear_history();

        bus.inject(APPROACH_KEYS, SignalValue::Uint8(0));
        settle(1).await;

        let assert_publishes = bus
            .history()
            .iter()
            .filter(|(s, v)| *s == KEY_LOST_WARNING_OUT && *v == SignalValue::Bool(true))
            .count();
        assert_eq!(
            assert_publishes, 0,
            "redundant 0 must not re-fire KeyLostWarning"
        );
    }

    /// User retrieves the key before the 2 s timer expires: warning
    /// is cleared early.
    #[tokio::test(start_paused = true)]
    async fn recovery_before_timeout_clears_early() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(SPEED, SignalValue::Float(30.0));
        bus.inject(APPROACH_KEYS, SignalValue::Uint8(1));
        settle(1).await;

        bus.inject(APPROACH_KEYS, SignalValue::Uint8(0));
        settle(500).await;
        assert_eq!(
            bus.latest_value(KEY_LOST_WARNING_OUT),
            Some(SignalValue::Bool(true)),
        );

        // Key reappears (driver found their fob).
        bus.inject(APPROACH_KEYS, SignalValue::Uint8(1));
        settle(1).await;

        assert_eq!(
            bus.latest_value(KEY_LOST_WARNING_OUT),
            Some(SignalValue::Bool(false)),
            "recovery edge must clear the warning before the timer expires"
        );
        assert_eq!(
            bus.latest_value(CHIME),
            Some(SignalValue::Bool(false)),
            "chime cleared on recovery"
        );
    }

    /// Ignition off mid-warning: warning is cleared (parked-with-no-key
    /// is not actionable).
    #[tokio::test(start_paused = true)]
    async fn ignition_off_mid_warning_clears() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(SPEED, SignalValue::Float(30.0));
        bus.inject(APPROACH_KEYS, SignalValue::Uint8(1));
        settle(1).await;

        bus.inject(APPROACH_KEYS, SignalValue::Uint8(0));
        settle(500).await;

        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        settle(1).await;

        assert_eq!(
            bus.latest_value(KEY_LOST_WARNING_OUT),
            Some(SignalValue::Bool(false)),
            "ignition off must clear an in-flight warning"
        );
    }
}
