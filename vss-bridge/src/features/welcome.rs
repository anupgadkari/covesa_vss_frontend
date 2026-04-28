//! Welcome — courtesy lighting when an authenticated PEPS device
//! enters the vehicle's LF coverage.
//!
//! # Behaviour
//!
//! When any paired key fob or BLE phone transitions from a "no-LF"
//! zone (`OutOfRange` or `RfRange`) into any LF-coverage zone
//! (`Approach` or any proximity zone), the feature claims the
//! exterior puddle lamps via the **Courtesy** arbiter for
//! `WELCOME_HOLD_SECS` (default 30 s).  Same pattern as a real OEM:
//! "I see you walking up; here's some light to find your door."
//!
//! Outputs claimed at MEDIUM priority via the courtesy arbiter:
//! - `Body.Lights.Puddle.Left.IsOn`
//! - `Body.Lights.Puddle.Right.IsOn`
//! - `Cabin.Lights.IsDomeOn`
//!
//! # Release conditions
//!
//! The hold is released early when any of:
//! 1. Timer expires (default 30 s).
//! 2. Any door opens — the user has entered the vehicle (or
//!    a door was opened externally); the cabin lights take over
//!    from this point and the puddle is no longer useful.
//! 3. Ignition transitions to ON / START — driver is in the seat,
//!    courtesy lighting is no longer useful.
//! 4. All paired devices leave the LF coverage entirely (back to
//!    `OutOfRange` or `RfRange`).
//!
//! # Idempotence
//!
//! Multiple devices entering serially do **not** stack the timer or
//! re-arm it — the first arrival latches a deadline; later arrivals
//! within that window are no-ops.  This prevents two people walking
//! up sequentially from doubling the courtesy duration.
//!
//! # Why a separate arbiter?
//!
//! Puddle / dome are *shared courtesy outputs* — Welcome, Farewell,
//! and a future PerimeterAlarm all want to claim them under different
//! conditions.  Putting them on a dedicated `courtesy_arbiter` keeps
//! the arbitration explicit (allow-list per feature) and prevents
//! these features from stepping on each other.

use std::sync::Arc;
use std::time::Duration;

use futures::StreamExt;
use tokio::select;
use tokio::time::{sleep, Instant};

use crate::arbiter::{ActuatorRequest, DomainArbiter};
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::plant_models::peps::signals as peps_signals;
use crate::plant_models::peps::zone::Zone;
use crate::signal_bus::{SignalBus, VssPath};

const FEATURE_ID: FeatureId = FeatureId::Welcome;

const PUDDLE_LEFT: VssPath = "Body.Lights.Puddle.Left.IsOn";
const PUDDLE_RIGHT: VssPath = "Body.Lights.Puddle.Right.IsOn";
const DOME: VssPath = "Cabin.Lights.IsDomeOn";

const POWER_STATE: VssPath = "Vehicle.LowVoltageSystemState";

// Note: the mirror-folded suppression for puddle lamps is enforced
// at the *arbiter* layer (see `puddle_arbiter` in `arbiter.rs` —
// PhysicalGate bound to `Body.Mirror.{Left,Right}.IsFolded`).
// Welcome therefore claims both puddles unconditionally; the arbiter
// silently drops the side whose mirror is folded.  This keeps the
// hardware constraint in one place so Farewell, PerimeterAlarm, and
// any future puddle claimant inherit the same behaviour.

const PAIRED_ZONE_SIGNALS: [VssPath; 6] = [
    "Body.PEPS.Plant.KeyFob.1.Zone",
    "Body.PEPS.Plant.KeyFob.2.Zone",
    "Body.PEPS.Plant.KeyFob.3.Zone",
    "Body.PEPS.Plant.KeyFob.4.Zone",
    peps_signals::PHONE_1_ZONE,
    peps_signals::PHONE_2_ZONE,
];

/// Door-open signals — used to release the courtesy lights early
/// once the user has actually entered the vehicle (or any door has
/// opened externally).  No point illuminating the puddle while the
/// door is already open — the cabin lights take over.
const DOOR_OPEN_SIGNALS: [VssPath; 4] = [
    "Body.Doors.Row1.Left.IsOpen",
    "Body.Doors.Row1.Right.IsOpen",
    "Body.Doors.Row2.Left.IsOpen",
    "Body.Doors.Row2.Right.IsOpen",
];

/// Default hold duration for the welcome courtesy lights.  30 s is
/// the typical OEM value — long enough for the user to walk up to
/// the vehicle and pull a door handle.
pub const WELCOME_HOLD_SECS: u64 = 30;

/// True when `zone` represents *any* LF coverage (proximity zones +
/// Approach).  Used for the entry-detection edge.
fn has_lf(zone: Zone) -> bool {
    matches!(
        zone,
        Zone::DriverDoor
            | Zone::PassengerDoor
            | Zone::Hood
            | Zone::Trunk
            | Zone::TrunkInside
            | Zone::Cabin
            | Zone::Approach
    )
}

/// True when `LowVoltageSystemState` is in a state that means
/// "vehicle is operating" — Welcome should release.
fn is_powered_on(val: &SignalValue) -> bool {
    matches!(val, SignalValue::String(s) if s == "ON" || s == "START")
}

pub struct Welcome<B: SignalBus> {
    bus: Arc<B>,
    /// Arbiter for the interior dome light (and any future shared
    /// interior courtesy actuators).
    courtesy_arb: Arc<DomainArbiter>,
    /// Dedicated arbiter for the exterior puddle lamps — separate
    /// surface because Farewell / PerimeterAlarm / DoorOpenAssist
    /// are all expected to claim them under different conditions
    /// and priorities.
    puddle_arb: Arc<DomainArbiter>,
    hold: Duration,
}

impl<B: SignalBus + Send + Sync + 'static> Welcome<B> {
    pub fn new(
        bus: Arc<B>,
        courtesy_arb: Arc<DomainArbiter>,
        puddle_arb: Arc<DomainArbiter>,
    ) -> Self {
        Self {
            bus,
            courtesy_arb,
            puddle_arb,
            hold: Duration::from_secs(WELCOME_HOLD_SECS),
        }
    }

    /// Override the default 30 s hold (for unit tests with virtual time).
    pub fn with_hold(mut self, hold: Duration) -> Self {
        self.hold = hold;
        self
    }

    pub async fn run(self) {
        tracing::info!(hold_secs = self.hold.as_secs(), "Welcome feature started");

        let mut zone_streams: Vec<futures::stream::BoxStream<'static, SignalValue>> =
            Vec::with_capacity(PAIRED_ZONE_SIGNALS.len());
        for &sig in PAIRED_ZONE_SIGNALS.iter() {
            zone_streams.push(self.bus.subscribe(sig).await);
        }
        let mut device_zones: Vec<Zone> = vec![Zone::OutOfRange; PAIRED_ZONE_SIGNALS.len()];

        let mut power_rx = self.bus.subscribe(POWER_STATE).await;

        // Door-open subscriptions — release the courtesy lights as
        // soon as any door opens.
        let mut door_streams: Vec<futures::stream::BoxStream<'static, SignalValue>> =
            Vec::with_capacity(DOOR_OPEN_SIGNALS.len());
        for &sig in DOOR_OPEN_SIGNALS.iter() {
            door_streams.push(self.bus.subscribe(sig).await);
        }

        // None = idle; Some(deadline) = courtesy lights latched until
        // this Instant (or until released early by ignition / no
        // devices in LF / etc.).
        let mut deadline: Option<Instant> = None;

        loop {
            let zone_event = futures::future::select_all(
                zone_streams
                    .iter_mut()
                    .enumerate()
                    .map(|(i, s)| Box::pin(async move { (i, s.next().await) })),
            );
            let door_event = futures::future::select_all(
                door_streams
                    .iter_mut()
                    .enumerate()
                    .map(|(i, s)| Box::pin(async move { (i, s.next().await) })),
            );

            // If a deadline is set, sleep until it; otherwise sleep
            // for an effectively infinite duration (only zone /
            // power events will wake us).
            let timer_sleep = match deadline {
                Some(d) => d.saturating_duration_since(Instant::now()),
                None => Duration::from_secs(3600),
            };

            select! {
                ((slot, opt), _, _) = zone_event => {
                    let new_zone = match opt {
                        Some(SignalValue::String(s)) => {
                            Zone::from_str_value(&s).unwrap_or(Zone::OutOfRange)
                        }
                        _ => continue,
                    };
                    let old_zone = device_zones[slot];
                    device_zones[slot] = new_zone;

                    // Entry edge: was-not-LF → now-LF (the canonical
                    // "device just walked into LF coverage" event).
                    let entry_edge = !has_lf(old_zone) && has_lf(new_zone);

                    if entry_edge && deadline.is_none() {
                        // First device into LF — arm courtesy lights.
                        // Both puddles + dome are claimed here; the
                        // puddle arbiter silently drops a side whose
                        // mirror is folded (PhysicalGate).
                        tracing::info!(
                            slot, old = ?old_zone, new = ?new_zone,
                            "Welcome: entry edge — arming courtesy lights"
                        );
                        self.claim_all(true).await;
                        deadline = Some(Instant::now() + self.hold);
                    } else if entry_edge {
                        // Already armed — multiple devices entering
                        // serially do NOT extend the hold.  No-op.
                        tracing::debug!(slot, "Welcome: entry edge but already armed — no extend");
                    }

                    // If, after this update, NO paired device is in LF,
                    // release courtesy lights early.  Matches OEM
                    // behaviour: if you walk away before the hold
                    // expires, the lights go off.
                    if deadline.is_some() && !device_zones.iter().copied().any(has_lf) {
                        tracing::info!("Welcome: all devices left LF — releasing");
                        self.release_all().await;
                        deadline = None;
                    }
                }
                Some(val) = power_rx.next() => {
                    if deadline.is_some() && is_powered_on(&val) {
                        tracing::info!("Welcome: ignition ON — releasing courtesy lights");
                        self.release_all().await;
                        deadline = None;
                    }
                }
                ((door_idx, opt), _, _) = door_event => {
                    if deadline.is_some()
                        && matches!(opt, Some(SignalValue::Bool(true)))
                    {
                        tracing::info!(
                            door = DOOR_OPEN_SIGNALS[door_idx],
                            "Welcome: door opened — releasing courtesy lights"
                        );
                        self.release_all().await;
                        deadline = None;
                    }
                }
                _ = sleep(timer_sleep) => {
                    if deadline.is_some() {
                        tracing::info!("Welcome: hold expired — releasing");
                        self.release_all().await;
                        deadline = None;
                    }
                }
                else => break,
            }
        }
    }

    /// Arm or release courtesy outputs as a group.  Both puddles are
    /// claimed unconditionally; the puddle arbiter's `PhysicalGate`
    /// drops a side whose mirror is folded.  Dome runs through the
    /// courtesy arbiter independently.
    async fn claim_all(&self, on: bool) {
        for &sig in &[PUDDLE_LEFT, PUDDLE_RIGHT] {
            let _ = self
                .puddle_arb
                .request(ActuatorRequest {
                    signal: sig,
                    value: SignalValue::Bool(on),
                    priority: Priority::Medium,
                    feature_id: FEATURE_ID,
                })
                .await;
        }
        let _ = self
            .courtesy_arb
            .request(ActuatorRequest {
                signal: DOME,
                value: SignalValue::Bool(on),
                priority: Priority::Medium,
                feature_id: FEATURE_ID,
            })
            .await;
    }

    async fn release_all(&self) {
        for &sig in &[PUDDLE_LEFT, PUDDLE_RIGHT] {
            let _ = self.puddle_arb.release(sig, FEATURE_ID).await;
        }
        let _ = self.courtesy_arb.release(DOME, FEATURE_ID).await;
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::{courtesy_arbiter, puddle_arbiter};
    use tokio::time::advance;

    /// Build the bus, courtesy arbiter, and a Welcome feature with a
    /// short 100 ms hold so tests don't have to advance virtual time
    /// by 30 s for the timer-expiry case.
    async fn setup_with_hold(hold: Duration) -> (Arc<MockBus>, tokio::task::JoinHandle<()>) {
        let bus = Arc::new(MockBus::new());
        let (carb, cfut) = courtesy_arbiter(Arc::clone(&bus));
        let (parb, pfut) = puddle_arbiter(Arc::clone(&bus));
        tokio::spawn(cfut);
        tokio::spawn(pfut);
        let carb = Arc::new(carb);
        let parb = Arc::new(parb);
        let feature = Welcome::new(Arc::clone(&bus), carb, parb).with_hold(hold);
        let h = tokio::spawn(feature.run());
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        (bus, h)
    }

    async fn settle() {
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        tokio::time::sleep(Duration::from_millis(2)).await;
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    fn count_published(bus: &MockBus, sig: VssPath, val: bool) -> usize {
        bus.history()
            .into_iter()
            .filter(|(s, v)| *s == sig && *v == SignalValue::Bool(val))
            .count()
    }

    /// Fob transitions OutOfRange → Approach → courtesy lights claimed.
    #[tokio::test(start_paused = true)]
    async fn fob_entry_into_approach_arms_courtesy() {
        let (bus, _h) = setup_with_hold(Duration::from_secs(30)).await;

        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.Zone",
            SignalValue::String("Approach".into()),
        );
        settle().await;

        assert_eq!(
            bus.latest_value(PUDDLE_LEFT),
            Some(SignalValue::Bool(true)),
            "puddle left expected ON after fob entry"
        );
        assert_eq!(
            bus.latest_value(PUDDLE_RIGHT),
            Some(SignalValue::Bool(true))
        );
        assert_eq!(bus.latest_value(DOME), Some(SignalValue::Bool(true)));
    }

    /// Lights release after the hold expires.
    #[tokio::test(start_paused = true)]
    async fn lights_release_after_hold() {
        let (bus, _h) = setup_with_hold(Duration::from_millis(100)).await;
        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.Zone",
            SignalValue::String("Approach".into()),
        );
        settle().await;
        assert_eq!(bus.latest_value(PUDDLE_LEFT), Some(SignalValue::Bool(true)));

        advance(Duration::from_millis(120)).await;
        settle().await;

        assert_eq!(
            bus.latest_value(PUDDLE_LEFT),
            Some(SignalValue::Bool(false)),
            "puddle should default-off after hold expires"
        );
    }

    /// Ignition ON releases lights early.
    #[tokio::test(start_paused = true)]
    async fn ignition_on_releases_lights_early() {
        let (bus, _h) = setup_with_hold(Duration::from_secs(30)).await;
        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.Zone",
            SignalValue::String("Approach".into()),
        );
        settle().await;
        assert_eq!(bus.latest_value(PUDDLE_LEFT), Some(SignalValue::Bool(true)));

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle().await;

        assert_eq!(
            bus.latest_value(PUDDLE_LEFT),
            Some(SignalValue::Bool(false)),
            "ignition ON should release courtesy lights"
        );
    }

    /// Two devices entering serially do not extend the hold (single
    /// arm-and-release).
    #[tokio::test(start_paused = true)]
    async fn second_device_entry_does_not_extend_hold() {
        let (bus, _h) = setup_with_hold(Duration::from_millis(100)).await;

        // Device 1 enters at t=0.
        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.Zone",
            SignalValue::String("Approach".into()),
        );
        settle().await;

        // Device 2 enters at t=50 — half-way through the hold.
        advance(Duration::from_millis(50)).await;
        settle().await;
        bus.inject(
            "Body.PEPS.Plant.KeyFob.2.Zone",
            SignalValue::String("Approach".into()),
        );
        settle().await;

        bus.clear_history();
        // Total elapsed since first arm = ~50 ms + drain.  Advance
        // another 70 ms — by now we should be past the original
        // 100 ms deadline (NOT 100 ms past the second device's arrival).
        advance(Duration::from_millis(70)).await;
        settle().await;

        assert_eq!(
            count_published(&bus, PUDDLE_LEFT, false),
            1,
            "lights should release at the original deadline; second device must not extend"
        );
    }

    /// Fob in `Approach` then back to `OutOfRange` → lights release
    /// (no devices in LF anymore).
    #[tokio::test(start_paused = true)]
    async fn all_devices_leaving_lf_releases_lights() {
        let (bus, _h) = setup_with_hold(Duration::from_secs(30)).await;
        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.Zone",
            SignalValue::String("Approach".into()),
        );
        settle().await;
        assert_eq!(bus.latest_value(PUDDLE_LEFT), Some(SignalValue::Bool(true)));

        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.Zone",
            SignalValue::String("OutOfRange".into()),
        );
        settle().await;

        assert_eq!(
            bus.latest_value(PUDDLE_LEFT),
            Some(SignalValue::Bool(false)),
            "lights should release when last paired device leaves LF"
        );
    }

    /// Fob transitioning OutOfRange → RfRange (NOT into LF coverage)
    /// must not arm Welcome.
    #[tokio::test(start_paused = true)]
    async fn rf_range_only_does_not_arm_welcome() {
        let (bus, _h) = setup_with_hold(Duration::from_secs(30)).await;
        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.Zone",
            SignalValue::String("RfRange".into()),
        );
        settle().await;

        assert_eq!(
            bus.latest_value(PUDDLE_LEFT),
            None,
            "RfRange has no LF coverage → Welcome should not arm"
        );
    }

    /// Any door opening releases the courtesy lights early.
    #[tokio::test(start_paused = true)]
    async fn door_open_releases_lights_early() {
        let (bus, _h) = setup_with_hold(Duration::from_secs(30)).await;
        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.Zone",
            SignalValue::String("Approach".into()),
        );
        settle().await;
        assert_eq!(bus.latest_value(PUDDLE_LEFT), Some(SignalValue::Bool(true)));

        // Driver opens the door (via PassiveEntry, kick handle, etc.).
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle().await;

        assert_eq!(
            bus.latest_value(PUDDLE_LEFT),
            Some(SignalValue::Bool(false)),
            "any door open should release courtesy lights"
        );
    }

    /// Verify ALL four doors trigger the release, not just Row1.Left.
    #[tokio::test(start_paused = true)]
    async fn rear_door_open_also_releases_lights() {
        let (bus, _h) = setup_with_hold(Duration::from_secs(30)).await;
        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.Zone",
            SignalValue::String("Approach".into()),
        );
        settle().await;
        assert_eq!(bus.latest_value(PUDDLE_LEFT), Some(SignalValue::Bool(true)));

        // Passenger rear door opens.
        bus.inject("Body.Doors.Row2.Right.IsOpen", SignalValue::Bool(true));
        settle().await;

        assert_eq!(
            bus.latest_value(PUDDLE_LEFT),
            Some(SignalValue::Bool(false)),
            "Row2.Right open should also release"
        );
    }

    /// A `door open` event arriving while NO hold is in progress is a
    /// no-op — must not push spurious release publishes onto the bus.
    #[tokio::test(start_paused = true)]
    async fn door_open_when_idle_is_noop() {
        let (bus, _h) = setup_with_hold(Duration::from_secs(30)).await;

        bus.clear_history();
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle().await;

        // No claims and no releases on the courtesy / puddle arbiters
        // because Welcome was never armed.
        assert_eq!(
            count_published(&bus, PUDDLE_LEFT, false),
            0,
            "door open while idle must not produce a release"
        );
        assert_eq!(
            count_published(&bus, PUDDLE_LEFT, true),
            0,
            "door open while idle must not produce a claim either"
        );
    }

    // Mirror-fold suppression of puddle lamps is verified directly
    // against the puddle arbiter in `arbiter::tests` (it's an
    // arbiter-level concern, not Welcome's).  Welcome simply claims
    // both puddles; the arbiter applies the PhysicalGate.
}
