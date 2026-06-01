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
//! - `Vehicle.Controller.Body.Lights.Puddle.Left.IsOn`
//! - `Vehicle.Controller.Body.Lights.Puddle.Right.IsOn`
//! - `Vehicle.Cabin.Light.IsDomeOn`
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
use tokio::time::{sleep, sleep_until, Instant};

use crate::arbiter::{ActuatorRequest, DomainArbiter};
use crate::features::key_search_arbiter::{
    AntennaSet, Coalescing, KeySearchArbiterHandle, SearchMode,
};
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::plant_models::peps::signals as peps_signals;
use crate::plant_models::peps::zone::Zone;
use crate::signal_bus::{SignalBus, VssPath};

const FEATURE_ID: FeatureId = FeatureId::Welcome;

const PUDDLE_LEFT: VssPath = "Vehicle.Controller.Body.Lights.Puddle.Left.IsOn";
const PUDDLE_RIGHT: VssPath = "Vehicle.Controller.Body.Lights.Puddle.Right.IsOn";
const DOME: VssPath = "Vehicle.Cabin.Light.IsDomeOn";

const POWER_STATE: VssPath = "Vehicle.LowVoltageSystemState";

// ── Approach-poll publishes (formerly owned by KeySearchArbiter) ──────────
//
// Welcome runs the periodic AllApproach / Presence poll itself
// (backlog item #24 / PR #50) and publishes these three aggregate
// signals from each tick.  HMI consumers see the same names + types
// as before; only the writer changed.

const APPROACH_STATE_OUT: VssPath = "Vehicle.Controller.Body.PEPS.ApproachState";
const APPROACH_KEYS_OUT: VssPath = "Vehicle.Controller.Body.PEPS.ApproachKeys";
const APPROACH_POLL_INTERVAL_OUT: VssPath = "Vehicle.Controller.Body.PEPS.ApproachPollInterval";

/// Approach-poll cadence when no key is currently in approach —
/// scan briskly so we detect arrivals quickly.
pub const APPROACH_POLL_FAST: Duration = Duration::from_millis(700);

/// Approach-poll cadence when a key is already in approach —
/// confirm presence less often (saves both vehicle and fob battery).
pub const APPROACH_POLL_SLOW: Duration = Duration::from_secs(10);

// Note: the mirror-folded suppression for puddle lamps is enforced
// at the *arbiter* layer (see `puddle_arbiter` in `arbiter.rs` —
// PhysicalGate bound to `Body.Mirror.{Left,Right}.IsFolded`).
// Welcome therefore claims both puddles unconditionally; the arbiter
// silently drops the side whose mirror is folded.  This keeps the
// hardware constraint in one place so Farewell, PerimeterAlarm, and
// any future puddle claimant inherit the same behaviour.

// Welcome subscribes to per-device LastObservedZone (item #14a of
// the post-PEPS backlog) instead of the legacy `.Zone` mirror.  The
// KeySearchArbiter publishes LastObservedZone after each periodic
// approach poll; Welcome reacts to fobs transitioning into approach
// coverage just as it did before, but driven by what the antennas
// actually saw rather than HMI ground truth.
const PAIRED_ZONE_SIGNALS: [VssPath; 6] = [
    "Vehicle.Simulation.KeyFob.1.LastObservedZone",
    "Vehicle.Simulation.KeyFob.2.LastObservedZone",
    "Vehicle.Simulation.KeyFob.3.LastObservedZone",
    "Vehicle.Simulation.KeyFob.4.LastObservedZone",
    peps_signals::PHONE_1_LAST_OBSERVED_ZONE,
    peps_signals::PHONE_2_LAST_OBSERVED_ZONE,
];

/// Door-open signals — used to release the courtesy lights early
/// once the user has actually entered the vehicle (or any door has
/// opened externally).  No point illuminating the puddle while the
/// door is already open — the cabin lights take over.
const DOOR_OPEN_SIGNALS: [VssPath; 4] = [
    "Vehicle.Cabin.Door.Row1.Left.IsOpen",
    "Vehicle.Cabin.Door.Row1.Right.IsOpen",
    "Vehicle.Cabin.Door.Row2.Left.IsOpen",
    "Vehicle.Cabin.Door.Row2.Right.IsOpen",
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
        Zone::LeftFront
            | Zone::RightFront
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
    /// Handle to the KeySearchArbiter.  Welcome owns the periodic
    /// AllApproach / Presence poll — every other feature submits
    /// scans event-driven, and Welcome is the only consumer of the
    /// approach poll's result, so the cadence lives here too.
    key_search: KeySearchArbiterHandle,
    hold: Duration,
    fast_cadence: Duration,
    slow_cadence: Duration,
}

impl<B: SignalBus + Send + Sync + 'static> Welcome<B> {
    pub fn new(
        bus: Arc<B>,
        courtesy_arb: Arc<DomainArbiter>,
        puddle_arb: Arc<DomainArbiter>,
        key_search: KeySearchArbiterHandle,
    ) -> Self {
        Self {
            bus,
            courtesy_arb,
            puddle_arb,
            key_search,
            hold: Duration::from_secs(WELCOME_HOLD_SECS),
            fast_cadence: APPROACH_POLL_FAST,
            slow_cadence: APPROACH_POLL_SLOW,
        }
    }

    /// Override the default 30 s hold (for unit tests with virtual time).
    pub fn with_hold(mut self, hold: Duration) -> Self {
        self.hold = hold;
        self
    }

    /// Override the default approach-poll cadences (700 ms / 10 s).
    /// Tests use much shorter durations so virtual time doesn't have
    /// to advance by seconds to exercise cadence flips.
    pub fn with_cadence(mut self, fast: Duration, slow: Duration) -> Self {
        self.fast_cadence = fast;
        self.slow_cadence = slow;
        self
    }

    pub async fn run(self) {
        tracing::info!(
            hold_secs = self.hold.as_secs(),
            fast_ms = self.fast_cadence.as_millis() as u64,
            slow_ms = self.slow_cadence.as_millis() as u64,
            "Welcome feature started"
        );

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

        // ── Approach-poll state ──────────────────────────────────────
        //
        // Welcome owns the periodic AllApproach / Presence scan
        // (formerly the arbiter's internal task — see PR #50 / backlog
        // #24).  Adaptive cadence: fast when nothing in approach,
        // slow once a key is detected.  Suspended while ignition is
        // in ACC / ON / START (driving — fob is in the cabin anyway,
        // no need to burn LF airtime on approach detection).
        let mut approach_state: bool = false;
        let mut approach_keys: u8 = 0;
        let mut poll_suspended: bool = false;
        let mut poll_deadline: Instant = Instant::now() + self.fast_cadence;

        // Seed the aggregate signals so HMI snapshots see defined values
        // before the first scan completes.
        let _ = self
            .bus
            .publish(APPROACH_STATE_OUT, SignalValue::Bool(false))
            .await;
        let _ = self
            .bus
            .publish(APPROACH_KEYS_OUT, SignalValue::Uint8(0))
            .await;
        let _ = self
            .bus
            .publish(
                APPROACH_POLL_INTERVAL_OUT,
                SignalValue::Uint16(self.fast_cadence.as_millis() as u16),
            )
            .await;

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

            // While suspended, push the poll deadline far out so the
            // poll branch never wins; only zone / power / door events
            // wake the loop.
            let next_poll_deadline = if poll_suspended {
                Instant::now() + Duration::from_secs(3600)
            } else {
                poll_deadline
            };

            select! {
                // `biased` so the existing zone / power / door / hold
                // arms (which arbitrate courtesy lighting) take
                // precedence over the periodic approach poll.  Without
                // this, under paused virtual time the poll arm and the
                // hold-timer arm can both be simultaneously ready and
                // the random selection occasionally picks the poll
                // first, starving the hold-expiry release.
                biased;

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
                    // Approach-poll suspension follows the same
                    // ACC / ON / START rule the arbiter used — keep
                    // the legacy semantic so HMI cadence behaviour
                    // matches `main` byte-for-byte.
                    let should_suspend = matches!(
                        &val,
                        SignalValue::String(s) if s == "ACC" || s == "ON" || s == "START"
                    );
                    if should_suspend != poll_suspended {
                        poll_suspended = should_suspend;
                        tracing::info!(
                            suspended = poll_suspended,
                            "Welcome: approach poll suspension changed"
                        );
                        if poll_suspended {
                            // Going suspended — clear aggregate state
                            // and publish the legacy "paused" markers
                            // (interval = 0 was the arbiter's
                            // suspended-marker; preserved here).
                            approach_state = false;
                            approach_keys = 0;
                            let _ = self
                                .bus
                                .publish(APPROACH_STATE_OUT, SignalValue::Bool(false))
                                .await;
                            let _ = self
                                .bus
                                .publish(APPROACH_KEYS_OUT, SignalValue::Uint8(0))
                                .await;
                            let _ = self
                                .bus
                                .publish(APPROACH_POLL_INTERVAL_OUT, SignalValue::Uint16(0))
                                .await;
                        } else {
                            // Resuming — kick an immediate poll so
                            // ApproachState catches up without waiting
                            // a full fast_cadence.
                            poll_deadline = Instant::now();
                        }
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
                // Periodic AllApproach / Presence poll — last in the
                // biased order so courtesy-lighting decisions (zone,
                // power, door, hold) always take precedence.
                _ = sleep_until(next_poll_deadline), if !poll_suspended => {
                    let started = Instant::now();
                    // Coalescing::Allowed — concurrent in-flight
                    // approach scans (e.g. a feature that submits its
                    // own AllApproach Presence query) can share the
                    // result.  Periodic polls are inherently
                    // refresh-tolerant.
                    let result = self
                        .key_search
                        .submit(
                            "Welcome",
                            AntennaSet::AllApproach,
                            SearchMode::Presence,
                            Coalescing::Allowed,
                        )
                        .await;

                    if let Some(result) = result {
                        let now_any = !result.keys_found.is_empty();
                        let now_count = result.keys_found.len() as u8;

                        if now_any != approach_state || now_count != approach_keys {
                            approach_state = now_any;
                            approach_keys = now_count;
                            let next_interval = if now_any {
                                self.slow_cadence
                            } else {
                                self.fast_cadence
                            };
                            let _ = self
                                .bus
                                .publish(APPROACH_STATE_OUT, SignalValue::Bool(now_any))
                                .await;
                            let _ = self
                                .bus
                                .publish(APPROACH_KEYS_OUT, SignalValue::Uint8(now_count))
                                .await;
                            let _ = self
                                .bus
                                .publish(
                                    APPROACH_POLL_INTERVAL_OUT,
                                    SignalValue::Uint16(next_interval.as_millis() as u16),
                                )
                                .await;
                            tracing::debug!(
                                approach_state, approach_keys, ?next_interval,
                                "Welcome: approach state changed"
                            );
                        }
                    } else {
                        tracing::warn!("Welcome: approach poll returned no result");
                    }

                    // Schedule next poll regardless of result
                    // (a transport error shouldn't kill the cadence).
                    let next_interval = if approach_state {
                        self.slow_cadence
                    } else {
                        self.fast_cadence
                    };
                    poll_deadline = started + next_interval;
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

    /// Build the bus, courtesy + puddle arbiters, the KeySearch
    /// arbiter (Welcome owns the approach-poll now, so the
    /// KeySearchArbiterHandle is required at construction), and a
    /// Welcome feature with a short 100 ms hold so tests don't have
    /// to advance virtual time by 30 s for the timer-expiry case.
    /// The approach-poll cadence is set to a tiny value too so any
    /// cadence-flip-driven side effects fire promptly.
    async fn setup_with_hold(hold: Duration) -> (Arc<MockBus>, tokio::task::JoinHandle<()>) {
        let bus = Arc::new(MockBus::new());
        let (carb, cfut) = courtesy_arbiter(Arc::clone(&bus));
        let (parb, pfut) = puddle_arbiter(Arc::clone(&bus));
        tokio::spawn(cfut);
        tokio::spawn(pfut);
        let carb = Arc::new(carb);
        let parb = Arc::new(parb);
        let (ksa, ksa_handle, ksa_rx) =
            crate::features::key_search_arbiter::KeySearchArbiter::new_with_rx(Arc::clone(&bus));
        tokio::spawn(ksa.run(ksa_rx));
        let feature = Welcome::new(Arc::clone(&bus), carb, parb, ksa_handle)
            .with_hold(hold)
            .with_cadence(Duration::from_millis(20), Duration::from_millis(200));
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
            "Vehicle.Simulation.KeyFob.1.LastObservedZone",
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
            "Vehicle.Simulation.KeyFob.1.LastObservedZone",
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
            "Vehicle.Simulation.KeyFob.1.LastObservedZone",
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
            "Vehicle.Simulation.KeyFob.1.LastObservedZone",
            SignalValue::String("Approach".into()),
        );
        settle().await;

        // Device 2 enters at t=50 — half-way through the hold.
        advance(Duration::from_millis(50)).await;
        settle().await;
        bus.inject(
            "Vehicle.Simulation.KeyFob.2.LastObservedZone",
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
            "Vehicle.Simulation.KeyFob.1.LastObservedZone",
            SignalValue::String("Approach".into()),
        );
        settle().await;
        assert_eq!(bus.latest_value(PUDDLE_LEFT), Some(SignalValue::Bool(true)));

        bus.inject(
            "Vehicle.Simulation.KeyFob.1.LastObservedZone",
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
            "Vehicle.Simulation.KeyFob.1.LastObservedZone",
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
            "Vehicle.Simulation.KeyFob.1.LastObservedZone",
            SignalValue::String("Approach".into()),
        );
        settle().await;
        assert_eq!(bus.latest_value(PUDDLE_LEFT), Some(SignalValue::Bool(true)));

        // Driver opens the door (via PassiveEntry, kick handle, etc.).
        bus.inject(
            "Vehicle.Cabin.Door.Row1.Left.IsOpen",
            SignalValue::Bool(true),
        );
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
            "Vehicle.Simulation.KeyFob.1.LastObservedZone",
            SignalValue::String("Approach".into()),
        );
        settle().await;
        assert_eq!(bus.latest_value(PUDDLE_LEFT), Some(SignalValue::Bool(true)));

        // Passenger rear door opens.
        bus.inject(
            "Vehicle.Cabin.Door.Row2.Right.IsOpen",
            SignalValue::Bool(true),
        );
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
        bus.inject(
            "Vehicle.Cabin.Door.Row1.Left.IsOpen",
            SignalValue::Bool(true),
        );
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

    // ── Approach-poll cadence + suspension ──────────────────────────
    //
    // Migrated from `key_search_arbiter::tests` along with the poll
    // itself.  Welcome now owns the periodic AllApproach / Presence
    // scan; these tests assert the published `ApproachState` /
    // `ApproachKeys` / `ApproachPollInterval` signals behave exactly
    // as the arbiter's loop used to make them behave (same cadence
    // flip, same suspension semantics, same suspended-marker
    // interval = 0).  See backlog item #24 / PR #50.

    fn approach_state(bus: &MockBus) -> Option<bool> {
        match bus.latest_value(APPROACH_STATE_OUT) {
            Some(SignalValue::Bool(b)) => Some(b),
            _ => None,
        }
    }
    fn approach_keys(bus: &MockBus) -> Option<u8> {
        match bus.latest_value(APPROACH_KEYS_OUT) {
            Some(SignalValue::Uint8(v)) => Some(v),
            _ => None,
        }
    }
    fn approach_interval(bus: &MockBus) -> Option<u16> {
        match bus.latest_value(APPROACH_POLL_INTERVAL_OUT) {
            Some(SignalValue::Uint16(v)) => Some(v),
            _ => None,
        }
    }

    /// Yield + a tiny sleep in real time so spawned subscribers
    /// process injected signals before we assert.  Used in tests
    /// that don't pause virtual time (the cadence-flip ones).
    async fn settle_real() {
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
    }

    /// Place a fob's `PlacedZone` + `Paired` so the arbiter's
    /// `AllApproach / Presence` poll surfaces it.  Same shape the
    /// smart_unlock / smart_trunk_pop / key_lost_warning tests use.
    fn place(bus: &MockBus, slot: u8, zone: Zone) {
        let zone_path: &'static str = match slot {
            0 => "Vehicle.Simulation.KeyFob.1.PlacedZone",
            1 => "Vehicle.Simulation.KeyFob.2.PlacedZone",
            2 => "Vehicle.Simulation.KeyFob.3.PlacedZone",
            _ => panic!("unknown slot"),
        };
        let paired_path: &'static str = match slot {
            0 => "Vehicle.Simulation.KeyFob.1.Paired",
            1 => "Vehicle.Simulation.KeyFob.2.Paired",
            2 => "Vehicle.Simulation.KeyFob.3.Paired",
            _ => unreachable!(),
        };
        bus.inject(zone_path, SignalValue::String(zone.as_str().into()));
        bus.inject(paired_path, SignalValue::Bool(true));
    }

    /// Spawn a Welcome with very short cadences so we can exercise
    /// the poll loop in real time without making the test suite slow.
    /// Fast = 20 ms, slow = 200 ms.  Adds the 50 ms AllApproach
    /// Presence scan latency on top.
    async fn setup_short_cadence() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let (carb, cfut) = courtesy_arbiter(Arc::clone(&bus));
        let (parb, pfut) = puddle_arbiter(Arc::clone(&bus));
        tokio::spawn(cfut);
        tokio::spawn(pfut);
        let (ksa, ksa_handle, ksa_rx) =
            crate::features::key_search_arbiter::KeySearchArbiter::new_with_rx(Arc::clone(&bus));
        tokio::spawn(ksa.run(ksa_rx));
        let feature = Welcome::new(Arc::clone(&bus), Arc::new(carb), Arc::new(parb), ksa_handle)
            .with_cadence(Duration::from_millis(20), Duration::from_millis(200));
        tokio::spawn(feature.run());
        settle_real().await;
        bus
    }

    #[tokio::test]
    async fn approach_state_starts_false_with_no_keys() {
        // Spawn at production cadence — only the initial seeded
        // publishes are exercised here, no need to wait a tick.
        let bus = Arc::new(MockBus::new());
        let (carb, cfut) = courtesy_arbiter(Arc::clone(&bus));
        let (parb, pfut) = puddle_arbiter(Arc::clone(&bus));
        tokio::spawn(cfut);
        tokio::spawn(pfut);
        let (ksa, ksa_handle, ksa_rx) =
            crate::features::key_search_arbiter::KeySearchArbiter::new_with_rx(Arc::clone(&bus));
        tokio::spawn(ksa.run(ksa_rx));
        tokio::spawn(
            Welcome::new(Arc::clone(&bus), Arc::new(carb), Arc::new(parb), ksa_handle).run(),
        );
        settle_real().await;
        assert_eq!(approach_state(&bus), Some(false));
        assert_eq!(approach_keys(&bus), Some(0));
        // Initial cadence published is fast (no key detected).
        assert_eq!(approach_interval(&bus), Some(700));
    }

    #[tokio::test]
    async fn approach_state_flips_to_true_when_key_enters_approach() {
        let bus = setup_short_cadence().await;
        place(&bus, 0, Zone::Approach);
        // One fast cadence (20) + scan latency (50) = ~70 ms; slack.
        tokio::time::sleep(Duration::from_millis(200)).await;
        assert_eq!(approach_state(&bus), Some(true));
        assert_eq!(approach_keys(&bus), Some(1));
        assert_eq!(approach_interval(&bus), Some(200)); // slow cadence
    }

    #[tokio::test]
    async fn approach_state_flips_back_when_key_leaves() {
        let bus = setup_short_cadence().await;
        place(&bus, 0, Zone::Approach);
        tokio::time::sleep(Duration::from_millis(200)).await;
        assert_eq!(approach_state(&bus), Some(true));

        // Move the fob out — should flip back after one slow cycle.
        place(&bus, 0, Zone::OutOfRange);
        tokio::time::sleep(Duration::from_millis(400)).await;
        assert_eq!(approach_state(&bus), Some(false));
        assert_eq!(approach_keys(&bus), Some(0));
        assert_eq!(approach_interval(&bus), Some(20)); // fast cadence
    }

    #[tokio::test]
    async fn poll_suspended_on_ignition_on() {
        let bus = setup_short_cadence().await;
        // Place a fob in Approach and immediately turn ignition ON.
        place(&bus, 0, Zone::Approach);
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        // Wait well past what would be a poll cycle.
        tokio::time::sleep(Duration::from_millis(400)).await;
        // Suspension forces ApproachState=false and interval=0
        // (the legacy "paused" marker).
        assert_eq!(approach_state(&bus), Some(false));
        assert_eq!(approach_keys(&bus), Some(0));
        assert_eq!(approach_interval(&bus), Some(0));
    }

    #[tokio::test]
    async fn poll_resumes_when_ignition_returns_to_off() {
        let bus = setup_short_cadence().await;
        // Suspend first.
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        tokio::time::sleep(Duration::from_millis(50)).await;
        place(&bus, 0, Zone::Approach);
        tokio::time::sleep(Duration::from_millis(200)).await;
        assert_eq!(approach_state(&bus), Some(false), "suspended");

        // Resume — kick is immediate, plus the scan latency.
        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        tokio::time::sleep(Duration::from_millis(150)).await;
        assert_eq!(approach_state(&bus), Some(true));
        assert_eq!(approach_interval(&bus), Some(200));
    }
}
