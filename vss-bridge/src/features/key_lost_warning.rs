//! Key-Lost Warning — short chime + cluster flag when the cabin is
//! sealed up under power with no paired key inside.
//!
//! # Trigger
//!
//! Fires on the *rising edge* of all four gating conditions being
//! true simultaneously:
//!
//! 1. `Vehicle.LowVoltageSystemState` is `ON` or `START` — the
//!    driver has taken (or is about to take) responsibility for
//!    the vehicle; without ignition this is just a parked car
//!    and the lost-key state is undefined.
//! 2. Every front and rear door (`Row1.{Left,Right}.IsOpen`,
//!    `Row2.{Left,Right}.IsOpen`) is closed.
//! 3. The rear trunk (`Body.Trunk.Rear.IsOpen`) is closed.
//! 4. `ApproachKeys` is exactly 0 — no paired key is in any
//!    interior or approach zone.
//!
//! No speed threshold.  The driver-error this catches is "you got
//! in, closed up, turned the key, and there's no fob in the
//! vehicle" — equally bad standing still as moving.
//!
//! # Action
//!
//! - Publish `Vehicle.Controller.Starting.KeyLostWarning = true`
//!   (cluster polls this to render its "key not detected" icon).
//! - Publish `Vehicle.Controller.Body.Chime.IsActive = true` for
//!   `WARNING_DURATION` (2 s), then publish `false`.
//!
//! # Auto-clear (whichever happens first)
//!
//! - `WARNING_DURATION` (2 s) timer expires.
//! - `ApproachKeys` returns to ≥ 1 (the user produced a paired
//!   fob — found in a pocket, retrieved from the cabin floor).
//! - Any door or the trunk opens (the user is going to look for
//!   the key — keep the chime from re-firing every second).
//! - Ignition drops to anything other than ON / START.
//!
//! # Re-arming
//!
//! Once cleared, the next trigger requires the gating condition to
//! drop to false (at least one of: ignition off, a door open,
//! trunk open, key present) and then rise back to all-true.  A
//! redundant "still no key, still all closed" tick does not re-fire.
//!
//! # Notes on the chime channel
//!
//! The chime path today is a shared `Vehicle.Controller.Body.Chime.IsActive`
//! Bool with no arbiter — `LockFeedback`, `PerimeterAlarm`, and now
//! this feature all publish directly.  When chime usages collide,
//! the plant model's `IsSounding` follows the latest writer.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;
use tokio::time::{sleep_until, Duration, Instant};

use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

// ── Signal constants ───────────────────────────────────────────────────────

const APPROACH_KEYS: VssPath = "Vehicle.Controller.Body.PEPS.ApproachKeys";
const POWER_STATE: VssPath = "Vehicle.LowVoltageSystemState";
const TRUNK_OPEN: VssPath = "Vehicle.Body.Trunk.Rear.IsOpen";
const CHIME: VssPath = "Vehicle.Controller.Body.Chime.IsActive";
const KEY_LOST_WARNING_OUT: VssPath = "Vehicle.Controller.Starting.KeyLostWarning";

/// Per-door `IsOpen` signals.  Index order matches the existing
/// arbiter / plant-model convention (Row1.Left, Row1.Right,
/// Row2.Left, Row2.Right).  Physical paths — see
/// `plant_models::side` for the orientation-aware discussion;
/// this fan-out is genuinely "any of the four doors open" so it
/// stays physical.
const DOOR_OPEN_SIGNALS: [VssPath; 4] = [
    "Vehicle.Cabin.Door.Row1.Left.IsOpen",
    "Vehicle.Cabin.Door.Row1.Right.IsOpen",
    "Vehicle.Cabin.Door.Row2.Left.IsOpen",
    "Vehicle.Cabin.Door.Row2.Right.IsOpen",
];

// ── Tunables ───────────────────────────────────────────────────────────────

/// How long the chime + cluster flag stay active after the trigger
/// edge.  Long enough for the driver to hear and react, short
/// enough not to annoy.
pub const WARNING_DURATION: Duration = Duration::from_secs(2);

// ── Helpers ────────────────────────────────────────────────────────────────

fn is_power_on(val: &SignalValue) -> bool {
    matches!(val, SignalValue::String(s) if s == "ON" || s == "START")
}

fn approach_keys_count(val: &SignalValue) -> Option<u8> {
    match val {
        SignalValue::Uint8(v) => Some(*v),
        _ => None,
    }
}

fn is_open(val: &SignalValue) -> Option<bool> {
    match val {
        SignalValue::Bool(b) => Some(*b),
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
            duration_s = WARNING_DURATION.as_secs(),
            "KeyLostWarning feature started"
        );

        let mut approach_rx = self.bus.subscribe(APPROACH_KEYS).await;
        let mut power_rx = self.bus.subscribe(POWER_STATE).await;
        let mut trunk_rx = self.bus.subscribe(TRUNK_OPEN).await;
        // Explicit per-door subscriptions — same rationale as
        // SlamLock / DoorTrimButton: tokio::select! over a
        // futures::select_all is not cancel-safe and drops edges.
        let mut row1l_rx = self.bus.subscribe(DOOR_OPEN_SIGNALS[0]).await;
        let mut row1r_rx = self.bus.subscribe(DOOR_OPEN_SIGNALS[1]).await;
        let mut row2l_rx = self.bus.subscribe(DOOR_OPEN_SIGNALS[2]).await;
        let mut row2r_rx = self.bus.subscribe(DOOR_OPEN_SIGNALS[3]).await;

        // Cached state.  Default doors + trunk to "closed" and
        // approach_keys to 0 (matches a freshly-booted parked
        // vehicle); the first subscription replay overwrites these
        // with the bus's current truth before any edge is evaluated.
        let mut power_on = false;
        let mut approach_keys: u8 = 0;
        let mut trunk_open = false;
        let mut door_open = [false; 4];
        let mut warning_deadline: Option<Instant> = None;
        // Latched: was the gating condition true after the previous
        // edge?  Suppresses repeat triggers as long as the situation
        // doesn't change.  Cleared either by an auto-clear path or by
        // the gating condition dropping to false.
        let mut latched_active = false;

        loop {
            let warning_expiry = async {
                match warning_deadline {
                    Some(dl) => sleep_until(dl).await,
                    None => std::future::pending().await,
                }
            };

            select! {
                Some(val) = approach_rx.next() => {
                    if let Some(c) = approach_keys_count(&val) {
                        approach_keys = c;
                    }
                }
                Some(val) = power_rx.next() => {
                    power_on = is_power_on(&val);
                }
                Some(val) = trunk_rx.next() => {
                    if let Some(b) = is_open(&val) { trunk_open = b; }
                }
                Some(val) = row1l_rx.next() => {
                    if let Some(b) = is_open(&val) { door_open[0] = b; }
                }
                Some(val) = row1r_rx.next() => {
                    if let Some(b) = is_open(&val) { door_open[1] = b; }
                }
                Some(val) = row2l_rx.next() => {
                    if let Some(b) = is_open(&val) { door_open[2] = b; }
                }
                Some(val) = row2r_rx.next() => {
                    if let Some(b) = is_open(&val) { door_open[3] = b; }
                }
                _ = warning_expiry => {
                    warning_deadline = None;
                    self.clear_warning().await;
                    // Note: latched_active stays true — re-arming
                    // requires the gating condition to drop and rise.
                    continue;
                }
                else => break,
            }

            // Re-evaluate the gating condition after every input edge.
            self.evaluate_gating(
                power_on,
                approach_keys,
                trunk_open,
                &door_open,
                &mut warning_deadline,
                &mut latched_active,
            )
            .await;
        }

        tracing::info!("KeyLostWarning feature stopped");
    }

    /// Decide whether the current input snapshot warrants firing,
    /// clearing, or doing nothing.  Called after every subscription
    /// edge in `run`.
    async fn evaluate_gating(
        &self,
        power_on: bool,
        approach_keys: u8,
        trunk_open: bool,
        door_open: &[bool; 4],
        warning_deadline: &mut Option<Instant>,
        latched_active: &mut bool,
    ) {
        let all_closed = !trunk_open && door_open.iter().all(|&b| !b);
        let gating_true = power_on && all_closed && approach_keys == 0;

        if gating_true {
            if !*latched_active {
                // Rising edge — fire.
                *latched_active = true;
                *warning_deadline = Some(Instant::now() + WARNING_DURATION);
                tracing::info!(
                    "KeyLostWarning: vehicle sealed under power with no paired key \
                     in cabin — chime + cluster warning"
                );
                self.assert_warning().await;
            }
            // Else: still in the same all-closed-no-key state —
            // suppress; the warning already fired or is timing out.
        } else if *latched_active {
            // Gating dropped — clear the latch so the next rising
            // edge can fire again, and tear down any in-flight
            // chime + flag.
            *latched_active = false;
            if warning_deadline.take().is_some() {
                self.clear_warning().await;
            }
        }
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

    /// Build a fresh bus + feature with all gating signals seeded to
    /// their "vehicle parked, doors open, no key, ignition off" boot
    /// state.  Tests then inject the specific edge they care about.
    async fn setup() -> (Arc<MockBus>, tokio::task::JoinHandle<()>) {
        let bus = Arc::new(MockBus::new());
        // Seed before spawning so the run loop's subscription replay
        // observes a coherent starting point.
        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        bus.inject(APPROACH_KEYS, SignalValue::Uint8(0));
        bus.inject(TRUNK_OPEN, SignalValue::Bool(false));
        for s in DOOR_OPEN_SIGNALS {
            // Start with doors *open* so the test setup's "close
            // everything" sequence below is itself the rising edge.
            bus.inject(s, SignalValue::Bool(true));
        }
        let feature = KeyLostWarning::new(Arc::clone(&bus));
        let handle = tokio::spawn(feature.run());
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

    /// Close all doors and trunk while ignition is ON and no paired
    /// key is in the cabin — chime + cluster flag fire on the
    /// last-closed edge.
    #[tokio::test(start_paused = true)]
    async fn close_up_with_no_key_and_ignition_on_fires() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle(1).await;
        // Close three doors — gating still false because Row2.Right is
        // still open.
        for s in &DOOR_OPEN_SIGNALS[..3] {
            bus.inject(s, SignalValue::Bool(false));
        }
        settle(1).await;
        assert_eq!(
            bus.latest_value(KEY_LOST_WARNING_OUT),
            None,
            "must not fire while a door is still open"
        );

        // Close the last door — the all-closed edge fires the warning.
        bus.inject(DOOR_OPEN_SIGNALS[3], SignalValue::Bool(false));
        settle(1).await;

        assert_eq!(
            bus.latest_value(KEY_LOST_WARNING_OUT),
            Some(SignalValue::Bool(true)),
        );
        assert_eq!(bus.latest_value(CHIME), Some(SignalValue::Bool(true)));

        // After WARNING_DURATION the chime + flag auto-clear.
        settle(WARNING_DURATION.as_millis() as u64 + 50).await;
        assert_eq!(
            bus.latest_value(KEY_LOST_WARNING_OUT),
            Some(SignalValue::Bool(false)),
        );
        assert_eq!(bus.latest_value(CHIME), Some(SignalValue::Bool(false)));
    }

    /// Ignition OFF: closing everything up must NOT fire.
    #[tokio::test(start_paused = true)]
    async fn close_up_with_ignition_off_does_not_fire() {
        let (bus, _h) = setup().await;

        // Power stays OFF.
        for s in DOOR_OPEN_SIGNALS {
            bus.inject(s, SignalValue::Bool(false));
        }
        settle(1).await;

        assert!(
            bus.history()
                .iter()
                .all(|(s, _)| *s != KEY_LOST_WARNING_OUT),
            "no warning expected with ignition OFF"
        );
    }

    /// Trunk left open: even with all doors closed + ignition ON +
    /// no key, the warning must not fire until the trunk also closes.
    #[tokio::test(start_paused = true)]
    async fn trunk_open_inhibits_trigger() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(TRUNK_OPEN, SignalValue::Bool(true));
        for s in DOOR_OPEN_SIGNALS {
            bus.inject(s, SignalValue::Bool(false));
        }
        settle(1).await;

        assert!(
            bus.history()
                .iter()
                .all(|(s, _)| *s != KEY_LOST_WARNING_OUT),
            "trunk still open — warning must not fire"
        );

        // Close the trunk: now the all-sealed edge triggers.
        bus.inject(TRUNK_OPEN, SignalValue::Bool(false));
        settle(1).await;
        assert_eq!(
            bus.latest_value(KEY_LOST_WARNING_OUT),
            Some(SignalValue::Bool(true)),
        );
    }

    /// Key in the cabin: closing everything up must NOT fire.
    #[tokio::test(start_paused = true)]
    async fn key_present_inhibits_trigger() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(APPROACH_KEYS, SignalValue::Uint8(1));
        for s in DOOR_OPEN_SIGNALS {
            bus.inject(s, SignalValue::Bool(false));
        }
        settle(1).await;

        assert!(
            bus.history()
                .iter()
                .all(|(s, _)| *s != KEY_LOST_WARNING_OUT),
            "key present — warning must not fire"
        );
    }

    /// User retrieves the key while the warning is active: warning
    /// clears early.
    #[tokio::test(start_paused = true)]
    async fn recovery_before_timeout_clears_early() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        for s in DOOR_OPEN_SIGNALS {
            bus.inject(s, SignalValue::Bool(false));
        }
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
        );
        assert_eq!(bus.latest_value(CHIME), Some(SignalValue::Bool(false)));
    }

    /// Driver opens a door mid-warning to look for the key — chime
    /// clears immediately (don't keep nagging while they hunt).
    #[tokio::test(start_paused = true)]
    async fn door_opens_mid_warning_clears() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        for s in DOOR_OPEN_SIGNALS {
            bus.inject(s, SignalValue::Bool(false));
        }
        settle(500).await;
        assert_eq!(
            bus.latest_value(KEY_LOST_WARNING_OUT),
            Some(SignalValue::Bool(true)),
        );

        // Driver opens their door.
        bus.inject(DOOR_OPEN_SIGNALS[0], SignalValue::Bool(true));
        settle(1).await;

        assert_eq!(
            bus.latest_value(KEY_LOST_WARNING_OUT),
            Some(SignalValue::Bool(false)),
        );
    }

    /// Ignition off mid-warning: warning clears (parked car with no
    /// key isn't an actionable warning).
    #[tokio::test(start_paused = true)]
    async fn ignition_off_mid_warning_clears() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        for s in DOOR_OPEN_SIGNALS {
            bus.inject(s, SignalValue::Bool(false));
        }
        settle(500).await;
        assert_eq!(
            bus.latest_value(KEY_LOST_WARNING_OUT),
            Some(SignalValue::Bool(true)),
        );

        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        settle(1).await;

        assert_eq!(
            bus.latest_value(KEY_LOST_WARNING_OUT),
            Some(SignalValue::Bool(false)),
        );
    }

    /// Re-arming: after a cleared warning, the gating must drop and
    /// rise again to re-fire.  A redundant tick while still in the
    /// all-closed-no-key state does NOT re-fire.
    #[tokio::test(start_paused = true)]
    async fn no_re_fire_while_latched() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        for s in DOOR_OPEN_SIGNALS {
            bus.inject(s, SignalValue::Bool(false));
        }
        settle(1).await;
        // Wait past the 2 s auto-clear so the chime/flag are False
        // but the latch is still held.
        settle(WARNING_DURATION.as_millis() as u64 + 50).await;
        bus.clear_history();

        // Inject a redundant "still 0" key reading.  Must not re-fire.
        bus.inject(APPROACH_KEYS, SignalValue::Uint8(0));
        settle(1).await;

        assert!(
            bus.history()
                .iter()
                .filter(|(s, v)| *s == KEY_LOST_WARNING_OUT && *v == SignalValue::Bool(true))
                .count()
                == 0,
            "must not re-fire while gating stays continuously true"
        );

        // Now drop the gating (open a door) and rise again (close it):
        // the rising edge re-fires.
        bus.inject(DOOR_OPEN_SIGNALS[0], SignalValue::Bool(true));
        settle(1).await;
        bus.inject(DOOR_OPEN_SIGNALS[0], SignalValue::Bool(false));
        settle(1).await;

        assert_eq!(
            bus.latest_value(KEY_LOST_WARNING_OUT),
            Some(SignalValue::Bool(true)),
            "re-fire after the gating dropped and rose again"
        );
    }
}
