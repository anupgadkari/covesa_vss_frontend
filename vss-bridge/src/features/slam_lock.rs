//! SlamLock — slam-lock-protect inversion (EU-side smart unlock).
//!
//! This feature only fires on EU-style vehicle lines
//! (`vehicle_line.slam_lock_protect = true`).  Its job is to defend
//! against the user accidentally locking the cabin while a door is
//! still open — typically because they hit the trim Lock button
//! reflexively while reaching for something on the seat next to them,
//! or a child in the back fiddles with their door's switch.
//!
//! # Behaviour
//!
//! Subscribes to:
//!   - `Body.Switches.DoorTrim.Row1.{Left,Right}.LockButton`
//!   - `Body.Doors.Row*.IsOpen` × 4
//!
//! On a trim Lock-button rising edge, when ANY door is open AND the
//! `slam_lock_protect` cal is `true`, the feature dispatches the
//! corresponding **unlock** via `DoorLockArbiter`:
//!
//! - **Driver-side trim** (Row1.Left under LHD, Row1.Right under RHD):
//!   * `two_stage_unlock = true`  → `UnlockDriver` (stage-1).
//!   * `two_stage_unlock = false` → `UnlockAll`.
//!
//! - **Passenger-side trim** (the other Row 1 door): always `UnlockAll`,
//!   regardless of `two_stage_unlock`.  Mirrors PassiveEntry's
//!   passenger-side bypass logic — pressing lock from the passenger
//!   seat with a door open is unambiguously "unlock everything."
//!
//! `DoorTrimButton` independently dispatches the original `LockAll`
//! as `DoorTrimButton`.  The arbiter serialises the two commands, so
//! the bus shows a deterministic two-event sequence:
//!
//! ```text
//! EventNum N    LOCKED                      requestor=DoorTrimButton
//! EventNum N+1  UNLOCKED | DRIVER_UNLOCKED  requestor=SlamLock
//! ```
//!
//! # Trust class
//!
//! `FeatureId::SlamLock` is in PerimeterAlarm's `EXTERNAL_LOCK_REQUESTORS`
//! list (so the US slam-lock-allowed lock-with-door-open path arms the
//! alarm), but is intentionally NOT in `EXTERNAL_AUTH_SOURCES` or
//! `INTERNAL_UNLOCK_SOURCES`.  This means the EU inversion's UNLOCK
//! event is neutral to the perimeter alarm — a thief who hits the trim
//! lock button during an active chime cannot use the SlamLock-driven
//! unlock to silently disarm the alarm.  Defended by
//! `slam_lock_intruder_chime_survives_eu_slamlock_protection` and the
//! US analogue in `perimeter_alarm.rs` tests.

use std::sync::Arc;
use std::time::Duration;

use futures::stream::StreamExt;
use tokio::time::sleep;

use crate::arbiter::{DoorLockArbiter, DoorLockRequest, LockCommand, FEEDBACK_REQUEST};
use crate::config::{DriverDoorSide, PlatformConfig};
use crate::ipc_message::{FeatureId, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

/// How long SlamLock waits between observing a lock-with-door-open
/// event and dispatching the corresponding unlock.  Tuned so the
/// user sees the lock flash play out ("the system acknowledged my
/// lock") before the unlock flash ("the system reverted because a
/// door is open") — the two together read as a single longer
/// feedback animation rather than a confusing race.  LockFeedback's
/// lock pattern is 100 ms OFF + 500 ms ON ≈ 600 ms total; 500 ms
/// lets the lock pulse mostly complete before the unlock arrives
/// and preempts cleanly.
const INVERSION_DELAY_MS: u64 = 500;

/// Mislock audible feedback — single horn honk that fires on every
/// slam-lock-protect inversion.  Audible cue that the lock was
/// rejected because of the open door.  Always fires when an
/// inversion happens, regardless of `dealer.horn_chirp_on_lock`:
/// it's an error indicator, not a confirmation, so a dealer cal that
/// silences the cheerful "lock confirmed" chirp shouldn't silence
/// this safety beep.
const MISLOCK_CHIRPS: u8 = 1;
/// Horn ON duration per mislock chirp (ms).  Slightly longer than
/// the rejected chirp-on-lock confirmation (300 ms) so the user
/// distinguishes it as a deliberate "denied" tone rather than a
/// success cue.
const MISLOCK_CHIRP_ON_MS: u64 = 350;
/// Horn OFF gap between mislock chirps (ms).  Unused with a single
/// chirp but retained so the loop reads cleanly if we ever raise
/// `MISLOCK_CHIRPS` again.
const MISLOCK_CHIRP_OFF_MS: u64 = 120;

const HORN: VssPath = "Body.Horn.IsActive";

const FEATURE_ID: FeatureId = FeatureId::SlamLock;

const LEFT_LOCK_BUTTON: VssPath = "Body.Switches.DoorTrim.Row1.Left.LockButton";
const RIGHT_LOCK_BUTTON: VssPath = "Body.Switches.DoorTrim.Row1.Right.LockButton";

const LOCK_STATUS: VssPath = "Cabin.LockStatus";
const LAST_REQUESTOR: VssPath = "Cabin.LockStatus.LastRequestor";
const LOCK_EVENT_NUM: VssPath = "Cabin.LockStatus.EventNum";

const DOOR_OPEN_SIGNALS: [VssPath; 4] = [
    "Body.Doors.Row1.Left.IsOpen",
    "Body.Doors.Row1.Right.IsOpen",
    "Body.Doors.Row2.Left.IsOpen",
    "Body.Doors.Row2.Right.IsOpen",
];

/// Requestors that, when they emit a LOCK event with a door open under
/// `slam_lock_protect=true`, are inverted by the SlamLock feature to an
/// `UnlockAll`.  These are all *exterior auth* lock sources where the
/// user is standing outside the vehicle pressing lock — pressing with
/// a door still open is almost always "I forgot to close the door,"
/// not "I want to leave the door open and lock anyway."
///
/// Excluded:
///   * `DoorTrimButton` — handled by the trim-button path below (it
///     subscribes to the button edges directly so the driver /
///     passenger side info is preserved for the two-stage routing).
///   * `SlamLock` itself — would feedback-loop our own inversion.
///   * `AutoLock`, `AutoRelock`, `WalkAwayLock` — by design these
///     fire only when the cabin is fully closed (auto-lock waits for
///     drive-off, walk-away fires after the user departs, auto-relock
///     follows an unlock with no door open).
///   * `CrashUnlock` / `DoubleLockRelease` — neither emits a LOCK.
const EXTERNAL_LOCK_INVERSION_REQUESTORS: &[&str] = &[
    "KeyfobRke",
    "KeyfobPeps",
    "ThumbPadLock",
    "PhoneApp",
    "PhoneBle",
    "NfcCard",
    "NfcPhone",
];

/// True if `status` represents a freshly armed-able lock state.
fn is_armable_lock_state(status: &str) -> bool {
    matches!(status, "LOCKED" | "DOUBLE_LOCKED")
}

/// Which physical side of Row 1 is the driver door under the current
/// cal.  Returned as a label ("Left" / "Right") so the dispatch site
/// can compare against the button it just observed.  Reads from the
/// dealer cal because that's where `driver_door_side` lives today —
/// see the migration note in `config.rs`.
fn driver_side(cfg: &PlatformConfig) -> DriverDoorSide {
    cfg.dealer_config().driver_door_side
}

pub struct SlamLock<B: SignalBus> {
    bus: Arc<B>,
    arbiter: Arc<DoorLockArbiter>,
    cfg: Arc<PlatformConfig>,
}

impl<B: SignalBus + Send + Sync + 'static> SlamLock<B> {
    pub fn new(bus: Arc<B>, arbiter: Arc<DoorLockArbiter>, cfg: Arc<PlatformConfig>) -> Self {
        Self { bus, arbiter, cfg }
    }

    pub async fn run(self) {
        tracing::info!("SlamLock feature started");

        let mut left_lock_rx = self.bus.subscribe(LEFT_LOCK_BUTTON).await;
        let mut right_lock_rx = self.bus.subscribe(RIGHT_LOCK_BUTTON).await;

        // Lock-event tuple subscriptions.  Mirrors PerimeterAlarm's
        // pattern: status / requestor are pure cache updaters; the
        // event_num bump is the "tuple is now coherent" tripwire that
        // drives the inversion decision.
        let mut status_rx = self.bus.subscribe(LOCK_STATUS).await;
        let mut requestor_rx = self.bus.subscribe(LAST_REQUESTOR).await;
        let mut event_num_rx = self.bus.subscribe(LOCK_EVENT_NUM).await;

        // Per-door IsOpen subscriptions — explicit, one branch each.
        //
        // We deliberately avoid `futures::future::select_all(...)` here
        // because `tokio::select!` over a `select_all` future is NOT
        // cancel-safe in our context: when `select_all` resolves
        // internally (one child's `stream.next()` returned a value),
        // that value is captured inside the `select_all` future.  If
        // tokio::select! happens to pick a different ready branch in
        // the same poll, the `select_all` future — and its captured
        // value — get dropped, silently losing the door-open edge
        // from the broadcast channel.  Manifested as intermittent
        // missed slam-lock-protect inversions where the cache thought
        // every door was closed despite the bus showing one open.
        let mut row1l_rx = self.bus.subscribe(DOOR_OPEN_SIGNALS[0]).await;
        let mut row1r_rx = self.bus.subscribe(DOOR_OPEN_SIGNALS[1]).await;
        let mut row2l_rx = self.bus.subscribe(DOOR_OPEN_SIGNALS[2]).await;
        let mut row2r_rx = self.bus.subscribe(DOOR_OPEN_SIGNALS[3]).await;
        let mut door_open = [false; 4];

        let mut lock_status: String = "UNLOCKED".into();
        let mut last_requestor: String = String::new();

        loop {
            tokio::select! {
                // `biased;` polls branches in declaration order rather
                // than randomly.  Critical for the lock-event tuple
                // here: when the arbiter publishes status → requestor
                // → event_num in quick succession, all three streams
                // can be Ready in the same select! poll.  Without
                // biased ordering, `event_num_rx` could win first and
                // the inversion-check would run against a stale
                // (status, requestor) cache — re-triggering on every
                // SlamLock-dispatched unlock and producing extra
                // mislock chirps.  Listing status_rx and requestor_rx
                // first guarantees the cache is fresh by the time we
                // evaluate event_num_rx.
                biased;
                Some(val) = status_rx.next() => {
                    if let SignalValue::String(s) = val { lock_status = s; }
                }
                Some(val) = requestor_rx.next() => {
                    if let SignalValue::String(s) = val { last_requestor = s; }
                }
                Some(val) = left_lock_rx.next() => {
                    if matches!(val, SignalValue::Bool(true)) {
                        self.maybe_invert(SideLabel::Left, &door_open).await;
                    }
                }
                Some(val) = right_lock_rx.next() => {
                    if matches!(val, SignalValue::Bool(true)) {
                        self.maybe_invert(SideLabel::Right, &door_open).await;
                    }
                }
                Some(_) = event_num_rx.next() => {
                    // External-lock inversion path.  An RKE / phone /
                    // NFC / PEPS / exterior-thumbpad lock arriving while
                    // a door is open is almost always "I forgot to close
                    // the door."  Invert to UnlockAll so the user can
                    // close the door cleanly and try again.  No side
                    // info from these sources, so UnlockAll regardless
                    // of two_stage_unlock — see EXTERNAL_LOCK_INVERSION_REQUESTORS.
                    if self.cfg.vehicle_line.slam_lock_protect
                        && is_armable_lock_state(&lock_status)
                        && door_open.iter().any(|&b| b)
                        && EXTERNAL_LOCK_INVERSION_REQUESTORS.contains(&last_requestor.as_str())
                    {
                        self.dispatch_inversion(
                            LockCommand::UnlockAll,
                            &format!("external-lock ({}) with door open", last_requestor),
                        )
                        .await;
                    }
                }
                Some(val) = row1l_rx.next() => {
                    if let SignalValue::Bool(b) = val { door_open[0] = b; }
                }
                Some(val) = row1r_rx.next() => {
                    if let SignalValue::Bool(b) = val { door_open[1] = b; }
                }
                Some(val) = row2l_rx.next() => {
                    if let SignalValue::Bool(b) = val { door_open[2] = b; }
                }
                Some(val) = row2r_rx.next() => {
                    if let SignalValue::Bool(b) = val { door_open[3] = b; }
                }
                else => {
                    tracing::warn!("SlamLock: a stream closed, exiting");
                    return;
                }
            }
        }
    }

    /// Trim-button path: decide whether to fire the inversion and
    /// which unlock command to use, then dispatch.  The cal gate
    /// (`slam_lock_protect`) is checked here so the same struct is a
    /// no-op on US lines without an explicit US-only build target.
    async fn maybe_invert(&self, side: SideLabel, door_open: &[bool; 4]) {
        if !self.cfg.vehicle_line.slam_lock_protect {
            return; // US line: DoorTrimButton handles slam-lock locking with FeatureId::SlamLock.
        }
        if !door_open.iter().any(|&b| b) {
            return; // All doors closed — normal interior lock, no inversion.
        }
        // Determine driver vs passenger side for stage-1 routing.  RHD
        // swaps which physical row1 door is the driver side.
        let driver = driver_side(&self.cfg);
        let pressed_side_is_driver = matches!(
            (side, driver),
            (SideLabel::Left, DriverDoorSide::Left) | (SideLabel::Right, DriverDoorSide::Right),
        );
        // Driver-side trim respects two-stage; passenger-side trim is
        // always full UnlockAll (passenger-bypass — same rule as
        // PassiveEntry's REQ-PE-009 / REQ-PE-011).
        let command = if pressed_side_is_driver && self.cfg.dealer_config().two_stage_unlock {
            LockCommand::UnlockDriver
        } else {
            LockCommand::UnlockAll
        };
        self.dispatch_inversion(
            command,
            &format!("trim lock {:?} side (driver={:?})", side, driver),
        )
        .await;
    }

    /// Shared dispatch path used by both the trim-button and external-
    /// lock inversion flows.  Sends the unlock command via the
    /// DoorLockArbiter tagged as `FeatureId::SlamLock` and publishes
    /// `FeedbackRequest = "unlock"` so `LockFeedback` plays its 2-flash
    /// pattern.  The dispatch is delayed by `INVERSION_DELAY_MS` and
    /// spawned so the SlamLock select! loop keeps processing other
    /// events while the timer runs — letting the user see the lock
    /// flash before the unlock arrives.
    async fn dispatch_inversion(&self, command: LockCommand, reason: &str) {
        tracing::info!(
            ?command,
            reason,
            delay_ms = INVERSION_DELAY_MS,
            "SlamLock: slam_lock_protect inversion — scheduling delayed unlock"
        );

        // Fire the mislock chirp pattern in parallel with the delayed
        // unlock.  Direct publish to Body.Horn.IsActive (single writer
        // for this short window — no concurrent PerimeterAlarm /
        // PanicAlarm in flight while a user is actively pressing lock).
        // If a second writer wants the horn here later, route both
        // through the existing horn arbiter at appropriate priorities.
        let chirp_bus = Arc::clone(&self.bus);
        tokio::spawn(async move {
            for _ in 0..MISLOCK_CHIRPS {
                let _ = chirp_bus.publish(HORN, SignalValue::Bool(true)).await;
                sleep(Duration::from_millis(MISLOCK_CHIRP_ON_MS)).await;
                let _ = chirp_bus.publish(HORN, SignalValue::Bool(false)).await;
                sleep(Duration::from_millis(MISLOCK_CHIRP_OFF_MS)).await;
            }
        });

        let bus = Arc::clone(&self.bus);
        let arbiter = Arc::clone(&self.arbiter);
        let reason = reason.to_string();
        tokio::spawn(async move {
            sleep(Duration::from_millis(INVERSION_DELAY_MS)).await;
            tracing::info!(?command, reason, "SlamLock: firing delayed inversion");
            if let Err(e) = arbiter
                .request(DoorLockRequest {
                    command,
                    feature_id: FEATURE_ID,
                })
                .await
            {
                tracing::error!(error = %e, "SlamLock: arbiter error");
                return;
            }
            let _ = bus
                .publish(FEEDBACK_REQUEST, SignalValue::String("unlock".into()))
                .await;
        });
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SideLabel {
    Left,
    Right,
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::door_lock_arbiter;
    use crate::config::VehicleLineCal;

    /// Advance the paused tokio clock past the inversion delay and
    /// yield enough times for the dispatch + arbiter + plant-model
    /// chain to settle on the bus.  Tests use `start_paused = true` so
    /// the simulated 500 ms wait doesn't cost real wall-clock time.
    async fn settle() {
        // Slightly more than the inversion delay so the spawned
        // dispatch task definitely fires before assertions run.
        tokio::time::sleep(Duration::from_millis(INVERSION_DELAY_MS + 100)).await;
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
    }

    /// Setup helper.  Spawns DoorLockArbiter and SlamLock with the
    /// supplied vehicle-line cal, plus the matching dealer-cal flips
    /// (`two_stage_unlock`, `driver_door_side`).
    async fn setup(vl: VehicleLineCal, two_stage: bool, driver_side_right: bool) -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let (arb, _ack_tx, loop_fut) = door_lock_arbiter(Arc::clone(&bus));
        tokio::spawn(loop_fut);
        let arb = Arc::new(arb);
        let cfg = PlatformConfig::with_vehicle_line(vl);
        // Apply dealer-cal flips for two-stage and RHD/LHD.
        let mut dc = cfg.dealer_config();
        dc.two_stage_unlock = two_stage;
        dc.driver_door_side = if driver_side_right {
            DriverDoorSide::Right
        } else {
            DriverDoorSide::Left
        };
        cfg.update_dealer_config(dc);

        let feature = SlamLock::new(Arc::clone(&bus), arb, cfg);
        tokio::spawn(feature.run());
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        bus
    }

    fn vl_eu() -> VehicleLineCal {
        VehicleLineCal {
            slam_lock_protect: true,
            ..VehicleLineCal::default()
        }
    }

    fn vl_us() -> VehicleLineCal {
        VehicleLineCal {
            slam_lock_protect: false,
            ..VehicleLineCal::default()
        }
    }

    /// Asserts the latest lock command on the bus matches `expected`.
    fn assert_last_lock_cmd(bus: &Arc<MockBus>, expected: &str) {
        assert_eq!(
            bus.latest_value("Body.Doors.CentralLock.Command"),
            Some(SignalValue::String(expected.into())),
            "expected last lock command to be {}",
            expected
        );
    }

    #[tokio::test(start_paused = true)]
    async fn lhd_driver_side_two_stage_unlocks_driver_only() {
        // EU + LHD + driver-side trim + two-stage on → UnlockDriver.
        let bus = setup(vl_eu(), true, false).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
        bus.inject(LEFT_LOCK_BUTTON, SignalValue::Bool(true));
        settle().await;

        assert_last_lock_cmd(&bus, "unlock_driver");
        assert_eq!(
            bus.latest_value("Cabin.LockStatus.LastRequestor"),
            Some(SignalValue::String("SlamLock".into()))
        );
    }

    #[tokio::test(start_paused = true)]
    async fn lhd_driver_side_two_stage_off_unlocks_all() {
        let bus = setup(vl_eu(), false, false).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
        bus.inject(LEFT_LOCK_BUTTON, SignalValue::Bool(true));
        settle().await;

        assert_last_lock_cmd(&bus, "unlock_all");
    }

    #[tokio::test(start_paused = true)]
    async fn lhd_passenger_side_always_unlocks_all() {
        // Passenger-side bypass — UnlockAll regardless of two-stage.
        for two_stage in [true, false] {
            let bus = setup(vl_eu(), two_stage, false).await;
            bus.inject("Body.Doors.Row1.Right.IsOpen", SignalValue::Bool(true));
            for _ in 0..8 {
                tokio::task::yield_now().await;
            }
            bus.inject(RIGHT_LOCK_BUTTON, SignalValue::Bool(true));
            settle().await;

            assert_last_lock_cmd(&bus, "unlock_all");
        }
    }

    #[tokio::test(start_paused = true)]
    async fn rhd_swaps_driver_passenger_sides() {
        // RHD: Row1.Right is driver → respects two-stage; Row1.Left is
        // passenger → always UnlockAll.
        let bus = setup(vl_eu(), true, true).await;
        bus.inject("Body.Doors.Row1.Right.IsOpen", SignalValue::Bool(true));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
        bus.inject(RIGHT_LOCK_BUTTON, SignalValue::Bool(true));
        settle().await;
        assert_last_lock_cmd(&bus, "unlock_driver");

        // New bus for the passenger-side check.
        let bus = setup(vl_eu(), true, true).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
        bus.inject(LEFT_LOCK_BUTTON, SignalValue::Bool(true));
        settle().await;
        assert_last_lock_cmd(&bus, "unlock_all");
    }

    #[tokio::test(start_paused = true)]
    async fn all_doors_closed_is_a_noop() {
        // Doors closed: SlamLock must NOT fire.  No lock command on bus.
        let bus = setup(vl_eu(), true, false).await;
        bus.inject(LEFT_LOCK_BUTTON, SignalValue::Bool(true));
        settle().await;

        assert_eq!(
            bus.latest_value("Body.Doors.CentralLock.Command"),
            None,
            "all doors closed: SlamLock must not dispatch any unlock"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn cal_off_is_a_noop() {
        // US line: SlamLock feature must not fire even with door open
        // (DoorTrimButton handles US slam-lock with the SlamLock requestor).
        let bus = setup(vl_us(), true, false).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
        bus.inject(LEFT_LOCK_BUTTON, SignalValue::Bool(true));
        settle().await;

        assert_eq!(
            bus.latest_value("Body.Doors.CentralLock.Command"),
            None,
            "US cal: SlamLock feature must be inert; DoorTrimButton handles US slam-lock"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn release_edge_is_a_noop() {
        let bus = setup(vl_eu(), true, false).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
        // Lock-button FALSE (release): must not dispatch anything.
        bus.inject(LEFT_LOCK_BUTTON, SignalValue::Bool(false));
        settle().await;

        assert_eq!(bus.latest_value("Body.Doors.CentralLock.Command"), None);
    }

    #[tokio::test(start_paused = true)]
    async fn feedback_request_is_unlock() {
        let bus = setup(vl_eu(), true, false).await;
        bus.inject("Body.Doors.Row1.Right.IsOpen", SignalValue::Bool(true));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
        bus.inject(RIGHT_LOCK_BUTTON, SignalValue::Bool(true));
        settle().await;

        // SlamLock publishes "unlock" feedback (not "lock") so
        // LockFeedback plays the standard 2-flash unlock pattern.
        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == FEEDBACK_REQUEST && *v == SignalValue::String("unlock".into())),
            "expected unlock FeedbackRequest from SlamLock inversion: {:?}",
            h
        );
    }

    // ── External-lock inversion path (RKE / PEPS / phone / NFC / thumb-pad) ──

    /// Helper: inject the full (status, requestor, event_num) tuple the
    /// arbiter would publish on an accepted external lock command.
    /// Mirrors PerimeterAlarm's `inject_lock` test helper but with the
    /// yields between each publish so SlamLock's caches stay coherent.
    async fn inject_lock_event(bus: &Arc<MockBus>, requestor: &str, event_num: u16) {
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        for _ in 0..4 {
            tokio::task::yield_now().await;
        }
        bus.inject(LAST_REQUESTOR, SignalValue::String(requestor.into()));
        for _ in 0..4 {
            tokio::task::yield_now().await;
        }
        bus.inject(LOCK_EVENT_NUM, SignalValue::Uint16(event_num));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
    }

    #[tokio::test(start_paused = true)]
    async fn rke_lock_with_door_open_under_eu_inverts_to_unlock_all() {
        // The user's request: replicate the trim-lock slam-lock-protect
        // logic for RKE.  EU cal + door open + KeyfobRke lock event →
        // SlamLock dispatches UnlockAll.
        let bus = setup(vl_eu(), true, false).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
        inject_lock_event(&bus, "KeyfobRke", 1).await;
        settle().await;

        assert_last_lock_cmd(&bus, "unlock_all");
        assert_eq!(
            bus.latest_value("Cabin.LockStatus.LastRequestor"),
            Some(SignalValue::String("SlamLock".into())),
            "the inversion's requestor is SlamLock"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn rke_lock_with_all_doors_closed_does_not_invert() {
        // Doors closed: normal RKE lock, no inversion regardless of cal.
        let bus = setup(vl_eu(), true, false).await;
        inject_lock_event(&bus, "KeyfobRke", 1).await;
        settle().await;

        // No new command — the test helper only injects the status
        // tuple onto the bus; SlamLock must NOT dispatch its own
        // unlock_all.
        assert_eq!(
            bus.latest_value("Body.Doors.CentralLock.Command"),
            None,
            "RKE lock with all doors closed: SlamLock must stay quiet"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn rke_lock_with_door_open_under_us_does_not_invert() {
        // US cal: SlamLock feature is inert.  The RKE lock event with
        // door open lands, cabin pre-arms via KeyfobRke (handled by
        // PerimeterAlarm), and nothing in SlamLock fires.
        let bus = setup(vl_us(), true, false).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
        inject_lock_event(&bus, "KeyfobRke", 1).await;
        settle().await;

        assert_eq!(
            bus.latest_value("Body.Doors.CentralLock.Command"),
            None,
            "US cal: external-lock inversion path must not fire"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn keyfob_peps_lock_with_door_open_inverts() {
        let bus = setup(vl_eu(), true, false).await;
        bus.inject("Body.Doors.Row1.Right.IsOpen", SignalValue::Bool(true));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
        inject_lock_event(&bus, "KeyfobPeps", 1).await;
        settle().await;
        assert_last_lock_cmd(&bus, "unlock_all");
    }

    #[tokio::test(start_paused = true)]
    async fn thumb_pad_lock_with_door_open_inverts() {
        // Exterior thumb-pad has its own keys-in-vehicle gate, but if
        // a door is also open the SlamLock inversion still fires under EU.
        let bus = setup(vl_eu(), true, false).await;
        bus.inject("Body.Doors.Row2.Left.IsOpen", SignalValue::Bool(true));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
        inject_lock_event(&bus, "ThumbPadLock", 1).await;
        settle().await;
        assert_last_lock_cmd(&bus, "unlock_all");
    }

    #[tokio::test(start_paused = true)]
    async fn auto_relock_with_door_open_does_not_invert() {
        // Defensive: AutoRelock by construction fires only when no
        // door is open, but if it did somehow appear with a door open
        // we should NOT invert — auto-driven locks aren't "the user
        // forgot to close the door."  Excluded from
        // EXTERNAL_LOCK_INVERSION_REQUESTORS.
        let bus = setup(vl_eu(), true, false).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
        inject_lock_event(&bus, "AutoRelock", 1).await;
        settle().await;
        assert_eq!(
            bus.latest_value("Body.Doors.CentralLock.Command"),
            None,
            "AutoRelock must not trigger the external-lock inversion"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn inversion_fires_mislock_honk() {
        // Audible mislock feedback: every inversion publishes exactly
        // MISLOCK_CHIRPS Body.Horn.IsActive=true pulses (one honk by
        // default), each followed by a Body.Horn.IsActive=false edge.
        let bus = setup(vl_eu(), true, false).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
        inject_lock_event(&bus, "KeyfobRke", 1).await;
        settle().await;

        let true_chirps = bus
            .history()
            .iter()
            .filter(|(s, v)| *s == HORN && *v == SignalValue::Bool(true))
            .count();
        let false_chirps = bus
            .history()
            .iter()
            .filter(|(s, v)| *s == HORN && *v == SignalValue::Bool(false))
            .count();
        assert_eq!(
            true_chirps, MISLOCK_CHIRPS as usize,
            "expected {} horn-ON edge(s) per inversion, saw {}",
            MISLOCK_CHIRPS, true_chirps
        );
        assert_eq!(
            false_chirps, MISLOCK_CHIRPS as usize,
            "expected {} horn-OFF edge(s) per inversion, saw {}",
            MISLOCK_CHIRPS, false_chirps
        );
    }

    #[tokio::test(start_paused = true)]
    async fn no_inversion_no_honk() {
        // No inversion → no mislock honk.  Verifies the honk is gated
        // on the inversion firing, not on the lock event itself.
        let bus = setup(vl_us(), true, false).await; // US cal: external-lock inversion is a no-op
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
        inject_lock_event(&bus, "KeyfobRke", 1).await;
        settle().await;

        assert!(
            bus.history().iter().all(|(s, _)| *s != HORN),
            "US cal: no inversion → no mislock honk"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn slam_lock_self_event_does_not_feedback_loop() {
        // The arbiter publishes events for every dispatch, including
        // SlamLock's own UnlockAll inversion.  Make sure SlamLock
        // doesn't re-invert its own unlock event.  (UnlockAll isn't
        // armable anyway, but defensive — a LOCKED+SlamLock event
        // from somewhere else must still be ignored.)
        let bus = setup(vl_eu(), true, false).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
        inject_lock_event(&bus, "SlamLock", 1).await;
        settle().await;
        assert_eq!(
            bus.latest_value("Body.Doors.CentralLock.Command"),
            None,
            "SlamLock must not re-invert a SlamLock-tagged event"
        );
    }
}
