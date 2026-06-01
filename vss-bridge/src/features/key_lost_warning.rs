//! Key-Lost Warning — short chime + cluster flag when the cabin is
//! sealed up under power with no paired key inside.
//!
//! # Architecture
//!
//! This feature **owns its own cabin scans** — it does not subscribe
//! to the arbiter-published `ApproachKeys` aggregate.  Two design
//! reasons:
//!
//! 1. **Locality of authority.**  Each feature that needs PEPS / phone
//!    presence information requests it directly from the
//!    `KeySearchArbiter` with the antenna set, mode, and coalescing
//!    policy that fit its own decision.  Approach Lighting scans the
//!    approach zone unauthenticated; Passive Entry scans the touched
//!    handle's exterior zone authenticated; KeyLostWarning scans only
//!    the cabin authenticated.  Centralising the schedule on the
//!    arbiter would force every feature to share one cadence and one
//!    auth mode, which is wrong for at least two of the three.
//! 2. **Latency-correct triggers.**  We need a fresh scan in response
//!    to a *specific physical event* (cabin sealed under power), not
//!    on an arbitrary polling cadence the arbiter happened to choose.
//!
//! # Trigger conditions for a scan
//!
//! Submit a `Cabin / Authenticated / Disallowed-coalesce` request when
//! all of the following hold:
//!
//! 1. `Vehicle.LowVoltageSystemState` is `ON` or `START`.
//! 2. Every Row1 / Row2 door (`IsOpen = false`) and the rear trunk
//!    (`IsOpen = false`) are closed — the cabin is sealed.
//!
//! The scan is submitted on:
//!
//! - The *closing edge* that completes the all-sealed state (last
//!   door / trunk to close).
//! - The *ignition-on edge* when the cabin is already sealed (the
//!   user got in, closed up, and only then turned the key).
//! - A 1-minute periodic tick while ignition is on (catches the case
//!   where the cabin stayed sealed and the user moved a fob out of
//!   range without opening a door — e.g. via window, or a paired
//!   phone went into the trunk and its battery died).
//!
//! # Warning behaviour
//!
//! When a scan result returns with `keys_found.is_empty()` AND the
//! gating condition is still true AND no warning is already latched:
//! publish `Vehicle.Controller.Body.PEPS.LostKeyWarning = true` and
//! claim the chime for `WARNING_DURATION` (2 s).  After the timer
//! expires we publish both signals `false` and *keep the latch held*
//! so subsequent periodic scans don't chime every minute.
//!
//! # Clearing the latch (whichever fires first)
//!
//! - A scan returns with `keys_found.len() >= 1` (the user found
//!   their fob — even periodic scans pick this up).
//! - A door or the trunk opens (the user is looking — drop the latch
//!   immediately, no scan latency).
//! - Ignition drops to anything other than ON / START.
//!
//! # Notes on the chime channel
//!
//! The chime path today is a shared `Vehicle.Controller.Body.Chime.IsActive`
//! Bool with no arbiter — `LockFeedback`, `PerimeterAlarm`, and now
//! this feature all publish directly.

use std::sync::Arc;
use std::time::Duration;

use futures::StreamExt;
use tokio::select;
use tokio::sync::mpsc;
use tokio::time::{interval, sleep_until, Instant};

use crate::features::key_search_arbiter::{
    AntennaSet, Coalescing, KeySearchArbiterHandle, KeySearchResult, SearchMode,
};
use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

// ── Signal constants ───────────────────────────────────────────────────────

const POWER_STATE: VssPath = "Vehicle.LowVoltageSystemState";
const TRUNK_OPEN: VssPath = "Vehicle.Body.Trunk.Rear.IsOpen";
const CHIME: VssPath = "Vehicle.Controller.Body.Chime.IsActive";
// Cluster-facing "no paired key on board" flag.  Lives under
// `Body.PEPS.*` rather than `Starting.*` because semantically it's
// the PEPS subsystem reporting a detection, not an ignition state.
// This is the same signal the deleted LostPkScan feature used to
// publish — KeyLostWarning is its strict successor, and any HMI
// subscriber that was already wired up needs no change.
const LOST_KEY_WARNING_OUT: VssPath = "Vehicle.Controller.Body.PEPS.LostKeyWarning";

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

/// Cadence of the "still sealed, still keyless?" periodic check
/// while ignition is on.  1 minute matches the typical OEM cluster-
/// nag interval and is far longer than the arbiter's own approach-
/// poll cadence (which we don't piggyback on here — see module
/// header).
pub const PERIODIC_INTERVAL: Duration = Duration::from_secs(60);

/// Identity passed to the arbiter for tracing and per-feature
/// coalescing policy.
const REQUESTER: &str = "KeyLostWarning";

// ── Helpers ────────────────────────────────────────────────────────────────

fn is_power_on(val: &SignalValue) -> bool {
    matches!(val, SignalValue::String(s) if s == "ON" || s == "START")
}

fn is_open(val: &SignalValue) -> Option<bool> {
    match val {
        SignalValue::Bool(b) => Some(*b),
        _ => None,
    }
}

fn all_sealed(door_open: &[bool; 4], trunk_open: bool) -> bool {
    !trunk_open && door_open.iter().all(|&b| !b)
}

/// Spawn the actual arbiter `submit` as a fire-and-forget task that
/// posts the result back to the run loop via `tx`.  Keeps the run
/// loop's `select!` responsive to door / trunk / ignition edges
/// while the scan (~100 ms of simulated LF airtime) is in flight.
fn spawn_cabin_scan(
    handle: &KeySearchArbiterHandle,
    tx: &mpsc::Sender<KeySearchResult>,
    reason: &'static str,
) {
    let handle = handle.clone();
    let tx = tx.clone();
    tracing::debug!(reason, "KeyLostWarning: submitting cabin scan");
    tokio::spawn(async move {
        if let Some(result) = handle
            .submit(
                REQUESTER,
                AntennaSet::Cabin,
                SearchMode::Authenticated,
                Coalescing::Disallowed,
            )
            .await
        {
            let _ = tx.send(result).await;
        } else {
            tracing::warn!("KeyLostWarning: arbiter dropped the request");
        }
    });
}

// ── Feature struct ─────────────────────────────────────────────────────────

pub struct KeyLostWarning<B: SignalBus> {
    bus: Arc<B>,
    key_search: KeySearchArbiterHandle,
}

impl<B: SignalBus + Send + Sync + 'static> KeyLostWarning<B> {
    pub fn new(bus: Arc<B>, key_search: KeySearchArbiterHandle) -> Self {
        Self { bus, key_search }
    }

    pub async fn run(self) {
        tracing::info!(
            duration_s = WARNING_DURATION.as_secs(),
            periodic_s = PERIODIC_INTERVAL.as_secs(),
            "KeyLostWarning feature started"
        );

        let mut power_rx = self.bus.subscribe(POWER_STATE).await;
        let mut trunk_rx = self.bus.subscribe(TRUNK_OPEN).await;
        let mut row1l_rx = self.bus.subscribe(DOOR_OPEN_SIGNALS[0]).await;
        let mut row1r_rx = self.bus.subscribe(DOOR_OPEN_SIGNALS[1]).await;
        let mut row2l_rx = self.bus.subscribe(DOOR_OPEN_SIGNALS[2]).await;
        let mut row2r_rx = self.bus.subscribe(DOOR_OPEN_SIGNALS[3]).await;

        let (scan_tx, mut scan_rx) = mpsc::channel::<KeySearchResult>(4);

        // Periodic ticker — paused unless ignition is on.  `interval`
        // fires immediately on first poll; we consume that throwaway
        // tick at start so the first *real* periodic check happens
        // PERIODIC_INTERVAL into ignition-on.
        let mut periodic = interval(PERIODIC_INTERVAL);
        periodic.tick().await; // consume the immediate one

        let mut power_on = false;
        let mut trunk_open = false;
        let mut door_open = [false; 4];
        let mut warning_deadline: Option<Instant> = None;
        let mut latched = false;

        loop {
            let warning_expiry = async {
                match warning_deadline {
                    Some(dl) => sleep_until(dl).await,
                    None => std::future::pending().await,
                }
            };
            let periodic_tick = async {
                if power_on {
                    periodic.tick().await;
                } else {
                    std::future::pending::<Instant>().await;
                }
            };

            select! {
                Some(val) = power_rx.next() => {
                    let was_on = power_on;
                    power_on = is_power_on(&val);
                    if was_on && !power_on {
                        // Ignition just dropped — tear down everything.
                        self.clear_state(
                            &mut latched, &mut warning_deadline,
                        ).await;
                    } else if !was_on && power_on
                        && all_sealed(&door_open, trunk_open)
                    {
                        // Ignition came on with cabin already sealed —
                        // the user got in, closed up, then turned the
                        // key.  Run a scan now rather than waiting for
                        // the 1-minute periodic to come around.
                        spawn_cabin_scan(
                            &self.key_search, &scan_tx, "ignition_on_while_sealed",
                        );
                    }
                }
                Some(val) = trunk_rx.next() => {
                    let was_open = trunk_open;
                    if let Some(b) = is_open(&val) { trunk_open = b; }
                    self.handle_seal_edge(
                        was_open, trunk_open, &door_open, power_on,
                        &scan_tx, &mut latched, &mut warning_deadline,
                    ).await;
                }
                Some(val) = row1l_rx.next() => {
                    let was_open = door_open[0];
                    if let Some(b) = is_open(&val) { door_open[0] = b; }
                    self.handle_door_edge(
                        was_open, door_open[0], &door_open, trunk_open, power_on,
                        &scan_tx, &mut latched, &mut warning_deadline,
                    ).await;
                }
                Some(val) = row1r_rx.next() => {
                    let was_open = door_open[1];
                    if let Some(b) = is_open(&val) { door_open[1] = b; }
                    self.handle_door_edge(
                        was_open, door_open[1], &door_open, trunk_open, power_on,
                        &scan_tx, &mut latched, &mut warning_deadline,
                    ).await;
                }
                Some(val) = row2l_rx.next() => {
                    let was_open = door_open[2];
                    if let Some(b) = is_open(&val) { door_open[2] = b; }
                    self.handle_door_edge(
                        was_open, door_open[2], &door_open, trunk_open, power_on,
                        &scan_tx, &mut latched, &mut warning_deadline,
                    ).await;
                }
                Some(val) = row2r_rx.next() => {
                    let was_open = door_open[3];
                    if let Some(b) = is_open(&val) { door_open[3] = b; }
                    self.handle_door_edge(
                        was_open, door_open[3], &door_open, trunk_open, power_on,
                        &scan_tx, &mut latched, &mut warning_deadline,
                    ).await;
                }
                _ = periodic_tick => {
                    if power_on && all_sealed(&door_open, trunk_open) {
                        spawn_cabin_scan(
                            &self.key_search, &scan_tx, "periodic_tick",
                        );
                    }
                }
                Some(result) = scan_rx.recv() => {
                    let still_gated = power_on && all_sealed(&door_open, trunk_open);
                    if result.keys_found.is_empty() {
                        if still_gated && !latched {
                            latched = true;
                            warning_deadline = Some(Instant::now() + WARNING_DURATION);
                            tracing::info!(
                                took_ms = result.took.as_millis() as u64,
                                "KeyLostWarning: cabin scan returned 0 paired keys \
                                 with cabin sealed under power — firing warning"
                            );
                            self.assert_warning().await;
                        }
                    } else if latched {
                        tracing::info!(
                            keys = result.keys_found.len(),
                            "KeyLostWarning: cabin scan found a paired key — \
                             clearing latch"
                        );
                        latched = false;
                        if warning_deadline.take().is_some() {
                            self.clear_warning().await;
                        }
                    }
                }
                _ = warning_expiry => {
                    warning_deadline = None;
                    self.clear_warning().await;
                    // Latch stays — re-arm requires the gating condition
                    // to drop and rise again (door open / trunk open /
                    // ignition off, then back to sealed-under-power),
                    // OR a scan that finds a key.
                }
                else => break,
            }
        }

        tracing::info!("KeyLostWarning feature stopped");
    }

    /// Trunk and door arms have slightly different signatures — both
    /// converge here once their cached `*_open` value is updated.
    /// Handles the close-edge-triggers-scan and open-edge-clears-
    /// warning paths in one place.
    #[allow(clippy::too_many_arguments)]
    async fn handle_seal_edge(
        &self,
        was_open: bool,
        now_open: bool,
        door_open: &[bool; 4],
        power_on: bool,
        scan_tx: &mpsc::Sender<KeySearchResult>,
        latched: &mut bool,
        warning_deadline: &mut Option<Instant>,
    ) {
        if !was_open && now_open {
            // An opening edge (trunk just opened) — gating dropped.
            if *latched {
                *latched = false;
                if warning_deadline.take().is_some() {
                    self.clear_warning().await;
                }
            }
        } else if was_open && !now_open && power_on && all_sealed(door_open, false)
        // ^ caller has already updated trunk_open in the cache; this
        // helper sees the post-edge state — so passing `false` for the
        // "ignore trunk in all_sealed check" version is fine because
        // we only got here when `now_open` is false (i.e. trunk is
        // closed).
        {
            spawn_cabin_scan(&self.key_search, scan_tx, "trunk_close");
        }
    }

    #[allow(clippy::too_many_arguments)]
    async fn handle_door_edge(
        &self,
        was_open: bool,
        now_open: bool,
        door_open: &[bool; 4],
        trunk_open: bool,
        power_on: bool,
        scan_tx: &mpsc::Sender<KeySearchResult>,
        latched: &mut bool,
        warning_deadline: &mut Option<Instant>,
    ) {
        if !was_open && now_open {
            if *latched {
                *latched = false;
                if warning_deadline.take().is_some() {
                    self.clear_warning().await;
                }
            }
        } else if was_open && !now_open && power_on && all_sealed(door_open, trunk_open) {
            spawn_cabin_scan(&self.key_search, scan_tx, "door_close");
        }
    }

    /// Reset everything to "idle" — used when ignition drops.
    async fn clear_state(&self, latched: &mut bool, warning_deadline: &mut Option<Instant>) {
        if *latched {
            *latched = false;
            if warning_deadline.take().is_some() {
                self.clear_warning().await;
            }
        }
    }

    async fn assert_warning(&self) {
        let _ = self
            .bus
            .publish(LOST_KEY_WARNING_OUT, SignalValue::Bool(true))
            .await;
        let _ = self.bus.publish(CHIME, SignalValue::Bool(true)).await;
    }

    async fn clear_warning(&self) {
        let _ = self.bus.publish(CHIME, SignalValue::Bool(false)).await;
        let _ = self
            .bus
            .publish(LOST_KEY_WARNING_OUT, SignalValue::Bool(false))
            .await;
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::features::key_search_arbiter::KeySearchArbiter;
    use crate::plant_models::peps::zone::Zone;
    use tokio::time::advance;

    /// Build a fresh bus + arbiter + feature with all gating signals
    /// seeded to "vehicle parked, doors open, no key, ignition off".
    /// The real `KeySearchArbiter` runs alongside the feature; tests
    /// place fobs via PEPS plant signals like the SmartUnlock tests
    /// do, and the arbiter's scans return real `KeySearchResult`s.
    async fn setup() -> (Arc<MockBus>, tokio::task::JoinHandle<()>) {
        let bus = Arc::new(MockBus::new());

        // Seed input state BEFORE spawning the feature so the
        // subscription replay observes a coherent starting point.
        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        bus.inject(TRUNK_OPEN, SignalValue::Bool(false));
        for s in DOOR_OPEN_SIGNALS {
            // Doors *open* initially so the test's close sequence is
            // itself the rising edge that triggers the scan.
            bus.inject(s, SignalValue::Bool(true));
        }

        // Wire up the real arbiter.
        let (ksa, handle, rx) = KeySearchArbiter::new_with_rx(Arc::clone(&bus));
        tokio::spawn(ksa.run(rx));

        let feature = KeyLostWarning::new(Arc::clone(&bus), handle);
        let handle_jh = tokio::spawn(feature.run());

        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
        (bus, handle_jh)
    }

    async fn settle(ms: u64) {
        // Advance time in small steps so timer-driven sleeps inside
        // the arbiter (run_scan sleeps ~100ms) wake up and the spawned
        // submit task can progress while we still own the runtime.
        // Plain `advance(ms)` fast-forwards the clock but doesn't
        // yield enough to drain multi-stage mpsc + oneshot chains
        // (scan_task → arbiter → response → scan_rx).
        for _ in 0..(ms.max(1)) {
            advance(Duration::from_millis(1)).await;
            for _ in 0..4 {
                tokio::task::yield_now().await;
            }
        }
    }

    /// Place a paired key fob in a zone — the arbiter's `Authenticated`
    /// search filters out unpaired fobs, so we also set the `Paired`
    /// flag.  Slots 1-4 are fobs in the PEPS plant.
    fn place(bus: &MockBus, slot: u8, zone: Zone) {
        let zone_path: &'static str = match slot {
            1 => "Vehicle.Simulation.KeyFob.1.PlacedZone",
            2 => "Vehicle.Simulation.KeyFob.2.PlacedZone",
            3 => "Vehicle.Simulation.KeyFob.3.PlacedZone",
            _ => panic!("unknown slot"),
        };
        let paired_path: &'static str = match slot {
            1 => "Vehicle.Simulation.KeyFob.1.Paired",
            2 => "Vehicle.Simulation.KeyFob.2.Paired",
            3 => "Vehicle.Simulation.KeyFob.3.Paired",
            _ => unreachable!(),
        };
        bus.inject(zone_path, SignalValue::String(zone.as_str().into()));
        bus.inject(paired_path, SignalValue::Bool(true));
    }

    /// Close every door + trunk in a single batch (no settle between).
    fn close_everything(bus: &MockBus) {
        for s in DOOR_OPEN_SIGNALS {
            bus.inject(s, SignalValue::Bool(false));
        }
    }

    /// Driver gets in, closes everything up, ignition is on, no key.
    /// The all-sealed-under-power edge triggers a cabin scan; the
    /// scan returns empty (no paired fob placed); warning fires.
    #[tokio::test(start_paused = true)]
    async fn close_up_with_no_key_fires() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle(10).await;
        close_everything(&bus);
        // Give the arbiter time to run the scan (simulated 100 ms).
        settle(200).await;

        assert_eq!(
            bus.latest_value(LOST_KEY_WARNING_OUT),
            Some(SignalValue::Bool(true)),
        );
        assert_eq!(bus.latest_value(CHIME), Some(SignalValue::Bool(true)));

        // Both auto-clear after WARNING_DURATION.
        settle(WARNING_DURATION.as_millis() as u64 + 50).await;
        assert_eq!(
            bus.latest_value(LOST_KEY_WARNING_OUT),
            Some(SignalValue::Bool(false)),
        );
        assert_eq!(bus.latest_value(CHIME), Some(SignalValue::Bool(false)));
    }

    /// Ignition OFF + close-up: no scan should fire, no warning.
    #[tokio::test(start_paused = true)]
    async fn ignition_off_close_up_does_not_fire() {
        let (bus, _h) = setup().await;

        close_everything(&bus);
        settle(300).await;

        assert!(
            bus.history()
                .iter()
                .all(|(s, _)| *s != LOST_KEY_WARNING_OUT),
            "no warning expected with ignition OFF"
        );
    }

    /// Paired key in the cabin: scan returns a finding, no warning.
    #[tokio::test(start_paused = true)]
    async fn key_in_cabin_inhibits_trigger() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        place(&bus, 1, Zone::Cabin);
        settle(10).await;
        close_everything(&bus);
        settle(300).await;

        assert!(
            bus.history()
                .iter()
                .all(|(s, _)| *s != LOST_KEY_WARNING_OUT),
            "paired key in cabin — warning must not fire"
        );
    }

    /// Trunk left open: closing all doors + ignition ON does not
    /// trigger because the seal isn't complete.
    #[tokio::test(start_paused = true)]
    async fn trunk_open_inhibits_trigger() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(TRUNK_OPEN, SignalValue::Bool(true));
        for s in DOOR_OPEN_SIGNALS {
            bus.inject(s, SignalValue::Bool(false));
        }
        settle(300).await;

        assert!(
            bus.history()
                .iter()
                .all(|(s, _)| *s != LOST_KEY_WARNING_OUT),
            "trunk still open — warning must not fire"
        );

        // Now close the trunk — the seal edge triggers the scan.
        bus.inject(TRUNK_OPEN, SignalValue::Bool(false));
        settle(300).await;
        assert_eq!(
            bus.latest_value(LOST_KEY_WARNING_OUT),
            Some(SignalValue::Bool(true)),
        );
    }

    /// Driver opens a door mid-warning to look for the key — clears
    /// the chime + flag immediately, no scan needed.
    #[tokio::test(start_paused = true)]
    async fn door_open_mid_warning_clears() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle(10).await;
        close_everything(&bus);
        settle(300).await;
        assert_eq!(
            bus.latest_value(LOST_KEY_WARNING_OUT),
            Some(SignalValue::Bool(true)),
        );

        bus.inject(DOOR_OPEN_SIGNALS[0], SignalValue::Bool(true));
        settle(50).await;

        assert_eq!(
            bus.latest_value(LOST_KEY_WARNING_OUT),
            Some(SignalValue::Bool(false)),
        );
    }

    /// Ignition off mid-warning: warning clears.
    #[tokio::test(start_paused = true)]
    async fn ignition_off_mid_warning_clears() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle(10).await;
        close_everything(&bus);
        settle(300).await;
        assert_eq!(
            bus.latest_value(LOST_KEY_WARNING_OUT),
            Some(SignalValue::Bool(true)),
        );

        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        settle(50).await;

        assert_eq!(
            bus.latest_value(LOST_KEY_WARNING_OUT),
            Some(SignalValue::Bool(false)),
        );
    }

    /// Periodic re-scan path: 1 minute after the first fire (which
    /// auto-cleared at 2 s), the periodic tick runs another cabin
    /// scan.  Still empty → must NOT re-chime (latch held).
    #[tokio::test(start_paused = true)]
    async fn periodic_with_still_no_key_does_not_re_fire() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle(10).await;
        close_everything(&bus);
        settle(300).await;
        // First fire happened; warning_duration expires.
        settle(WARNING_DURATION.as_millis() as u64 + 50).await;
        bus.clear_history();

        // 1 minute later, the periodic tick runs a fresh scan.  Still
        // no key in cabin (we never placed one) → latch is held → no
        // chime.
        settle(PERIODIC_INTERVAL.as_millis() as u64 + 300).await;

        let trues = bus
            .history()
            .iter()
            .filter(|(s, v)| *s == LOST_KEY_WARNING_OUT && *v == SignalValue::Bool(true))
            .count();
        assert_eq!(
            trues, 0,
            "periodic scan while latched must not re-fire the warning"
        );
    }

    /// Periodic re-scan finds a key (driver retrieved their fob
    /// without opening anything — e.g. through the window): latch
    /// clears.
    #[tokio::test(start_paused = true)]
    async fn periodic_finding_key_clears_latch() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle(10).await;
        close_everything(&bus);
        settle(300).await;
        // Fire + auto-clear so the chime is False but latch is true.
        settle(WARNING_DURATION.as_millis() as u64 + 50).await;
        bus.clear_history();
        assert_eq!(
            bus.latest_value(LOST_KEY_WARNING_OUT),
            Some(SignalValue::Bool(false)),
        );

        // Driver hands a fob through the window.  Place it in the
        // cabin BEFORE the next periodic tick.
        place(&bus, 1, Zone::Cabin);

        // 1 minute later the periodic tick scans, finds the key,
        // clears the latch (publishes the OUT signal false again,
        // which is a no-op value-wise but proves the latch is
        // unlatched — the test that follows is the rising edge).
        settle(PERIODIC_INTERVAL.as_millis() as u64 + 300).await;

        // To prove the latch dropped, take the key away again and
        // run a new sealed-under-power cycle (open + close a door).
        // The scan will return empty and warning re-fires.
        bus.inject(
            "Vehicle.Simulation.KeyFob.1.PlacedZone",
            SignalValue::String(Zone::OutOfRange.as_str().into()),
        );
        bus.inject(DOOR_OPEN_SIGNALS[0], SignalValue::Bool(true));
        settle(50).await;
        bus.inject(DOOR_OPEN_SIGNALS[0], SignalValue::Bool(false));
        settle(300).await;

        assert_eq!(
            bus.latest_value(LOST_KEY_WARNING_OUT),
            Some(SignalValue::Bool(true)),
            "after the periodic-found-key cleared the latch, a new \
             rising edge should re-fire the warning"
        );
    }

    /// Re-arming via door open/close: latch dropped on door open,
    /// next close triggers a fresh scan; if still no key → re-fire.
    #[tokio::test(start_paused = true)]
    async fn re_arm_after_door_open_close() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle(10).await;
        close_everything(&bus);
        settle(300).await;
        // First fire.  Wait past the warning duration; latch is held.
        settle(WARNING_DURATION.as_millis() as u64 + 50).await;

        // Driver opens a door (looks around, doesn't find anything)
        // and closes it again.  Should re-fire.
        bus.inject(DOOR_OPEN_SIGNALS[0], SignalValue::Bool(true));
        settle(50).await;
        bus.inject(DOOR_OPEN_SIGNALS[0], SignalValue::Bool(false));
        settle(300).await;

        assert_eq!(
            bus.latest_value(LOST_KEY_WARNING_OUT),
            Some(SignalValue::Bool(true)),
        );
    }

    /// Ignition turned on AFTER the cabin is already sealed (user
    /// got in, closed up, then twisted the key).  The ignition-on
    /// edge triggers a scan even though no door / trunk edge
    /// arrived afterwards.
    #[tokio::test(start_paused = true)]
    async fn ignition_on_while_already_sealed_fires() {
        let (bus, _h) = setup().await;

        // Close everything BEFORE turning the key.
        close_everything(&bus);
        settle(10).await;
        // Confirm nothing fired yet (ignition still off).
        assert!(bus
            .history()
            .iter()
            .all(|(s, _)| *s != LOST_KEY_WARNING_OUT));

        // Now turn the key.
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle(300).await;

        assert_eq!(
            bus.latest_value(LOST_KEY_WARNING_OUT),
            Some(SignalValue::Bool(true)),
        );
    }
}
