//! Door-trim Lock / Unlock buttons — interior lock-source on the
//! driver and front-passenger door panels.
//!
//! # Behaviour
//!
//! Pressing **any** trim Lock button on Row 1 (driver / front passenger)
//! dispatches `LockAll` via the `DoorLockArbiter`.  Pressing a trim
//! Unlock button respects the same routing rules as PassiveEntry and
//! SlamLock's inversion:
//!
//! - **Driver-side** trim Unlock (`Row1.Left` under LHD,
//!   `Row1.Right` under RHD):
//!   * `dealer.two_stage_unlock = true`  → `UnlockDriver` (stage-1)
//!   * `dealer.two_stage_unlock = false` → `UnlockAll`
//!
//! - **Passenger-side** trim Unlock: always `UnlockAll`
//!   (passenger-side bypass — pressing unlock from the passenger seat
//!   is unambiguous "unlock everything," regardless of two-stage).
//!
//! Two additional state-driven rules apply to BOTH sides:
//!
//! - If the cabin is already in `DRIVER_UNLOCKED`, the next trim
//!   Unlock press dispatches `UnlockAll` (stage-2 escalation — don't
//!   pointlessly re-issue `UnlockDriver`).
//! - If the cabin is `DOUBLE_LOCKED` (super-lock), **all trim
//!   button presses — Lock and Unlock — are suppressed**.  Super-lock
//!   physically disconnects the interior linkage in the real vehicle,
//!   so the trim panel shall not move the doors in either direction.
//!   The user must release super-lock via an authenticated external
//!   source (RKE / PEPS / phone / NFC) first.
//!
//! Both lock and unlock actions happen with NO authentication — the
//! user is already inside the cabin, so the threat model is "occupant
//! operates the vehicle," not "stranger tries to defeat the lock from
//! outside."
//!
//! Egress safety:  the unlock path always succeeds even when the
//! perimeter alarm is armed.  A passenger trapped in a locked vehicle
//! can ALWAYS get out — we do not gate egress on alarm state.  Instead
//! `PerimeterAlarm` watches the resulting unlock event (LastRequestor =
//! `DoorTrimButton` while the cabin was armed) and escalates the alarm
//! sequence; that hand-off lives in `perimeter_alarm.rs`, not here.
//!
//! # Slam-lock behaviour (lock-button with door open)
//!
//! Pressing a trim Lock button while *any* door is open is a special
//! case driven by `vehicle_line.slam_lock_protect`:
//!
//! - **`slam_lock_protect = false`** (US "slam-lock allowed"): the
//!   lock command is dispatched with `FeatureId::SlamLock` instead of
//!   `DoorTrimButton`.  This puts the lock event in PerimeterAlarm's
//!   external-arming class — when the user closes the door, the cabin
//!   is already locked and the alarm arms after the 20 s pre-arm
//!   window.  Models the user-walks-away-hands-full case.
//!
//! - **`slam_lock_protect = true`** (EU "slam-lock protect"): the
//!   lock command is dispatched as `DoorTrimButton` (the usual
//!   interior identity).  A separate `SlamLock` feature watches for
//!   trim-press × door-open × cal=true and dispatches the
//!   corresponding unlock as `FeatureId::SlamLock`, undoing the
//!   accidental lock.  The two-event pair `(LOCKED, DoorTrimButton, N)`
//!   then `(UNLOCKED|DRIVER_UNLOCKED, SlamLock, N+1)` appears on the
//!   bus.  PerimeterAlarm sees neither event in its arming /
//!   disarming classification — the cabin's perimeter state is
//!   undisturbed.
//!
//! # Sources
//!
//! Subscribes to (FALSE→TRUE edges only):
//! ```text
//! Body.Switches.DoorTrim.Row1.Left.LockButton
//! Body.Switches.DoorTrim.Row1.Right.LockButton
//! Body.Switches.DoorTrim.Row1.Left.UnlockButton
//! Body.Switches.DoorTrim.Row1.Right.UnlockButton
//! ```
//!
//! Row 2 LockButton signals exist in the bus catalogue for
//! completeness but are NOT consumed here — physical Row 2 trim
//! buttons are uncommon and the user-facing requirement only calls
//! for driver / front-passenger control.  Easy to extend later if a
//! vehicle line wants Row 2 support.
//!
//! # Feedback
//!
//! Each successful arbiter request is followed by a
//! `Body.Doors.CentralLock.FeedbackRequest` publish (`"lock"` /
//! `"unlock"`) so `LockFeedback` plays its standard flash pattern.
//! No PEPS-presence gate (cf. `ThumbPadLock`) — this is an interior
//! source where keys-in-vehicle isn't possible (you're sitting on
//! them).

use std::sync::Arc;

use futures::stream::{select_all, StreamExt};

use crate::arbiter::{DoorLockArbiter, DoorLockRequest, LockCommand, FEEDBACK_REQUEST};
use crate::config::{DriverDoorSide, PlatformConfig};
use crate::ipc_message::{FeatureId, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

const LOCK_BUTTONS: [VssPath; 2] = [
    "Body.Switches.DoorTrim.Row1.Left.LockButton",
    "Body.Switches.DoorTrim.Row1.Right.LockButton",
];

const LEFT_UNLOCK_BUTTON: VssPath = "Body.Switches.DoorTrim.Row1.Left.UnlockButton";
const RIGHT_UNLOCK_BUTTON: VssPath = "Body.Switches.DoorTrim.Row1.Right.UnlockButton";
const LOCK_STATUS: VssPath = "Cabin.LockStatus";

const DOOR_OPEN_SIGNALS: [VssPath; 4] = [
    "Body.Doors.Row1.Left.IsOpen",
    "Body.Doors.Row1.Right.IsOpen",
    "Body.Doors.Row2.Left.IsOpen",
    "Body.Doors.Row2.Right.IsOpen",
];

pub struct DoorTrimButton<B: SignalBus> {
    bus: Arc<B>,
    arbiter: Arc<DoorLockArbiter>,
    cfg: Arc<PlatformConfig>,
}

impl<B: SignalBus + Send + Sync + 'static> DoorTrimButton<B> {
    pub fn new(bus: Arc<B>, arbiter: Arc<DoorLockArbiter>, cfg: Arc<PlatformConfig>) -> Self {
        Self { bus, arbiter, cfg }
    }

    pub async fn run(self) {
        tracing::info!("DoorTrimButton feature started");

        // Merge all four switch streams into two single streams (one
        // per command) so an edge from any door is treated identically.
        let lock_streams =
            futures::future::join_all(LOCK_BUTTONS.iter().map(|&s| self.bus.subscribe(s))).await;
        let mut lock_stream = select_all(lock_streams);

        // Per-side trim Unlock subscriptions.  We keep the two sides
        // separate (rather than merging like the Lock side) so we can
        // route stage-1 (driver door) vs passenger-bypass (all doors)
        // based on which physical button was pressed and the current
        // `dealer.two_stage_unlock` cal — matching PassiveEntry and
        // SlamLock's inversion semantics.
        let mut left_unlock_rx = self.bus.subscribe(LEFT_UNLOCK_BUTTON).await;
        let mut right_unlock_rx = self.bus.subscribe(RIGHT_UNLOCK_BUTTON).await;

        // Cached cabin lock status — drives the stage-2 escalation
        // rule below: once the cabin is in `DRIVER_UNLOCKED`, the
        // next driver-side trim Unlock press is a stage-2 "unlock the
        // rest of the doors" rather than another redundant
        // `UnlockDriver`.  Mirrors the PassiveEntry / RKE flow.
        let mut status_rx = self.bus.subscribe(LOCK_STATUS).await;
        let mut lock_status: String = "UNLOCKED".into();

        // Per-door open caches.  Updated on subscription replay + every
        // edge thereafter, then queried at trim-press time to decide
        // between the normal and slam-lock paths.
        //
        // Explicit per-door branches below — see the comment in
        // `slam_lock.rs` for the rationale.  Using
        // `futures::future::select_all(Box::pin(s.next().await))` here
        // is not cancel-safe in a `tokio::select!` arm: when
        // select_all internally resolves but tokio::select! picks a
        // different ready branch in the same poll, the consumed
        // value is dropped and the IsOpen edge is silently lost.
        let mut row1l_rx = self.bus.subscribe(DOOR_OPEN_SIGNALS[0]).await;
        let mut row1r_rx = self.bus.subscribe(DOOR_OPEN_SIGNALS[1]).await;
        let mut row2l_rx = self.bus.subscribe(DOOR_OPEN_SIGNALS[2]).await;
        let mut row2r_rx = self.bus.subscribe(DOOR_OPEN_SIGNALS[3]).await;
        let mut door_open = [false; 4];

        loop {
            tokio::select! {
                Some(val) = lock_stream.next() => {
                    if !matches!(val, SignalValue::Bool(true)) {
                        continue;
                    }
                    // Super-lock gate — same rule as Unlock below.
                    // Trim Lock under DOUBLE_LOCKED is a no-op because
                    // the interior linkage is physically disconnected
                    // in the real vehicle.  Locking-already-locked
                    // would also be semantically pointless.
                    if lock_status == "DOUBLE_LOCKED" {
                        tracing::info!(
                            lock_status = %lock_status,
                            "DoorTrimButton: lock suppressed (super-lock)"
                        );
                        continue;
                    }
                    let any_open = door_open.iter().any(|&b| b);
                    let protect = self.cfg.vehicle_line.slam_lock_protect;
                    // Three-way branch:
                    //   * all closed         → DoorTrimButton lock
                    //   * open + protect on  → DoorTrimButton lock (SlamLock will undo it)
                    //   * open + protect off → SlamLock lock (US slam-lock-allowed)
                    let requestor = if any_open && !protect {
                        FeatureId::SlamLock
                    } else {
                        FeatureId::DoorTrimButton
                    };
                    self.dispatch(LockCommand::LockAll, requestor, "lock").await;
                }
                Some(val) = left_unlock_rx.next() => {
                    if matches!(val, SignalValue::Bool(true)) {
                        if let Some(cmd) = self.unlock_command_for(SideLabel::Left, &lock_status) {
                            self.dispatch(cmd, FeatureId::DoorTrimButton, "unlock").await;
                        } else {
                            tracing::info!(
                                lock_status = %lock_status,
                                "DoorTrimButton: left unlock suppressed (super-lock)"
                            );
                        }
                    }
                }
                Some(val) = right_unlock_rx.next() => {
                    if matches!(val, SignalValue::Bool(true)) {
                        if let Some(cmd) = self.unlock_command_for(SideLabel::Right, &lock_status) {
                            self.dispatch(cmd, FeatureId::DoorTrimButton, "unlock").await;
                        } else {
                            tracing::info!(
                                lock_status = %lock_status,
                                "DoorTrimButton: right unlock suppressed (super-lock)"
                            );
                        }
                    }
                }
                Some(val) = status_rx.next() => {
                    if let SignalValue::String(s) = val { lock_status = s; }
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
                    tracing::warn!("DoorTrimButton: a stream closed, exiting");
                    return;
                }
            }
        }
    }

    /// Map a trim Unlock-button press on `side` to the right
    /// LockCommand under the current dealer cal and cabin state.
    /// Mirrors PassiveEntry and SlamLock's inversion semantics, plus
    /// a stage-2 escalation when the cabin is already `DRIVER_UNLOCKED`:
    ///
    ///   * cabin is `DOUBLE_LOCKED` (super-lock)        → `None`
    ///     (super-lock physically disconnects the interior linkage;
    ///     pressing trim Unlock from inside SHALL NOT unlock the
    ///     cabin — the user must release super-lock via an external
    ///     auth source first).
    ///   * cabin is `DRIVER_UNLOCKED` already           → `UnlockAll`
    ///     (any further trim press is stage-2 "unlock the rest" —
    ///     don't pointlessly re-issue UnlockDriver).
    ///   * driver-side trim + `two_stage_unlock=true`   → `UnlockDriver` (stage-1)
    ///   * driver-side trim + `two_stage_unlock=false`  → `UnlockAll`
    ///   * passenger-side trim (either cal)             → `UnlockAll`
    ///     (passenger-side bypass — pressing unlock from the passenger
    ///     seat is unambiguous "unlock everything").
    ///
    /// RHD respects `dealer.driver_door_side`: Row1.Right is the
    /// driver door on RHD, so the side meanings flip.
    fn unlock_command_for(&self, side: SideLabel, lock_status: &str) -> Option<LockCommand> {
        // Super-lock gate: trim Unlock is suppressed entirely.
        if lock_status == "DOUBLE_LOCKED" {
            return None;
        }
        // Stage-2 escalation: if driver door is already unlocked,
        // ANY further trim Unlock press dispatches UnlockAll.
        if lock_status == "DRIVER_UNLOCKED" {
            return Some(LockCommand::UnlockAll);
        }
        let driver = self.cfg.dealer_config().driver_door_side;
        let pressed_side_is_driver = matches!(
            (side, driver),
            (SideLabel::Left, DriverDoorSide::Left) | (SideLabel::Right, DriverDoorSide::Right),
        );
        if pressed_side_is_driver && self.cfg.dealer_config().two_stage_unlock {
            Some(LockCommand::UnlockDriver)
        } else {
            Some(LockCommand::UnlockAll)
        }
    }

    async fn dispatch(&self, command: LockCommand, feature_id: FeatureId, feedback: &'static str) {
        tracing::info!(
            ?command,
            ?feature_id,
            "DoorTrimButton: dispatching interior trim command"
        );
        if let Err(e) = self
            .arbiter
            .request(DoorLockRequest {
                command,
                feature_id,
            })
            .await
        {
            tracing::error!(error = %e, "DoorTrimButton: arbiter error");
            return;
        }
        let _ = self
            .bus
            .publish(FEEDBACK_REQUEST, SignalValue::String(feedback.into()))
            .await;
    }
}

/// Which physical Row 1 trim Unlock button the user pressed.  The
/// driver-vs-passenger interpretation depends on
/// `dealer.driver_door_side` and is resolved by `unlock_command_for`.
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

    async fn setup() -> Arc<MockBus> {
        // Default cal: slam_lock_protect = true (EU defensive default).
        setup_with_cal(VehicleLineCal::default()).await
    }

    async fn setup_with_cal(vl: VehicleLineCal) -> Arc<MockBus> {
        // Default dealer cal: two_stage_unlock=true, LHD.
        setup_with_cals(vl, true, DriverDoorSide::Left).await
    }

    async fn setup_with_cals(
        vl: VehicleLineCal,
        two_stage_unlock: bool,
        driver_door_side: DriverDoorSide,
    ) -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let (arb, _ack_tx, loop_fut) = door_lock_arbiter(Arc::clone(&bus));
        tokio::spawn(loop_fut);
        let arb = Arc::new(arb);
        let cfg = PlatformConfig::with_vehicle_line(vl);
        let mut dc = cfg.dealer_config();
        dc.two_stage_unlock = two_stage_unlock;
        dc.driver_door_side = driver_door_side;
        cfg.update_dealer_config(dc);
        let feature = DoorTrimButton::new(Arc::clone(&bus), arb, cfg);
        tokio::spawn(feature.run());
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        bus
    }

    async fn settle() {
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
    }

    #[tokio::test]
    async fn driver_trim_lock_button_locks_all() {
        let bus = setup().await;
        bus.inject(
            "Body.Switches.DoorTrim.Row1.Left.LockButton",
            SignalValue::Bool(true),
        );
        settle().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("lock_all".into())),
            "expected lock_all from driver trim lock, history: {:?}",
            h
        );
        assert!(
            h.iter()
                .any(|(s, v)| *s == FEEDBACK_REQUEST && *v == SignalValue::String("lock".into())),
            "expected lock feedback, history: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn passenger_trim_lock_button_locks_all() {
        let bus = setup().await;
        bus.inject(
            "Body.Switches.DoorTrim.Row1.Right.LockButton",
            SignalValue::Bool(true),
        );
        settle().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("lock_all".into())),
            "expected lock_all from passenger trim lock"
        );
    }

    #[tokio::test]
    async fn lhd_driver_trim_unlock_two_stage_on_dispatches_unlock_driver() {
        // LHD + two_stage_unlock=true: pressing the driver-side
        // (Row1.Left) trim Unlock issues stage-1 UnlockDriver, not
        // UnlockAll.  Matches PassiveEntry / SlamLock routing.
        let bus = setup_with_cals(VehicleLineCal::default(), true, DriverDoorSide::Left).await;
        bus.inject(
            "Body.Switches.DoorTrim.Row1.Left.UnlockButton",
            SignalValue::Bool(true),
        );
        settle().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("unlock_driver".into())),
            "expected unlock_driver (stage-1) from driver trim unlock under two_stage_unlock=true, history: {:?}",
            h
        );
        assert!(
            h.iter()
                .any(|(s, v)| *s == FEEDBACK_REQUEST && *v == SignalValue::String("unlock".into())),
            "expected unlock feedback, history: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn lhd_driver_trim_unlock_two_stage_off_dispatches_unlock_all() {
        // LHD + two_stage_unlock=false: driver-side trim Unlock falls
        // back to UnlockAll (no stage-1 routing on this vehicle line).
        let bus = setup_with_cals(VehicleLineCal::default(), false, DriverDoorSide::Left).await;
        bus.inject(
            "Body.Switches.DoorTrim.Row1.Left.UnlockButton",
            SignalValue::Bool(true),
        );
        settle().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("unlock_all".into())),
            "expected unlock_all (no two-stage) from driver trim unlock, history: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn lhd_passenger_trim_unlock_always_unlock_all() {
        // Passenger-side bypass — regardless of two_stage_unlock,
        // pressing the passenger trim Unlock dispatches UnlockAll.
        for two_stage in [true, false] {
            let bus =
                setup_with_cals(VehicleLineCal::default(), two_stage, DriverDoorSide::Left).await;
            bus.inject(
                "Body.Switches.DoorTrim.Row1.Right.UnlockButton",
                SignalValue::Bool(true),
            );
            settle().await;

            let h = bus.history();
            assert!(
                h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                    && *v == SignalValue::String("unlock_all".into())),
                "expected unlock_all from passenger trim unlock (two_stage={}): {:?}",
                two_stage,
                h
            );
        }
    }

    #[tokio::test]
    async fn rhd_driver_trim_unlock_two_stage_on_dispatches_unlock_driver() {
        // RHD: Row1.Right is the driver door, so it's the side that
        // respects two-stage.  Row1.Left becomes the passenger side
        // (covered by rhd_passenger_trim_unlock_always_unlock_all).
        let bus = setup_with_cals(VehicleLineCal::default(), true, DriverDoorSide::Right).await;
        bus.inject(
            "Body.Switches.DoorTrim.Row1.Right.UnlockButton",
            SignalValue::Bool(true),
        );
        settle().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("unlock_driver".into())),
            "RHD: Row1.Right is driver — expected unlock_driver, history: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn rhd_passenger_trim_unlock_always_unlock_all() {
        // RHD: Row1.Left is passenger → always UnlockAll regardless
        // of two_stage_unlock.
        let bus = setup_with_cals(VehicleLineCal::default(), true, DriverDoorSide::Right).await;
        bus.inject(
            "Body.Switches.DoorTrim.Row1.Left.UnlockButton",
            SignalValue::Bool(true),
        );
        settle().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("unlock_all".into())),
            "RHD: Row1.Left is passenger — expected unlock_all bypass, history: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn driver_trim_unlock_in_driver_unlocked_state_escalates_to_unlock_all() {
        // Stage-2 escalation: cabin is already DRIVER_UNLOCKED from a
        // prior stage-1, so pressing the driver-side trim Unlock again
        // dispatches UnlockAll (rather than repeating UnlockDriver).
        let bus = setup_with_cals(VehicleLineCal::default(), true, DriverDoorSide::Left).await;

        // Simulate the cabin coming up in DRIVER_UNLOCKED before the
        // press — the trim feature subscribes to Cabin.LockStatus,
        // injects replay this directly into the cache.
        bus.inject(
            "Cabin.LockStatus",
            SignalValue::String("DRIVER_UNLOCKED".into()),
        );
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }

        bus.inject(
            "Body.Switches.DoorTrim.Row1.Left.UnlockButton",
            SignalValue::Bool(true),
        );
        settle().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("unlock_all".into())),
            "expected unlock_all (stage-2) from driver trim unlock when cabin is DRIVER_UNLOCKED, history: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn driver_trim_unlock_first_press_still_stage_one_in_locked_state() {
        // Sanity / regression: when the cabin is LOCKED (or any state
        // other than DRIVER_UNLOCKED), the driver trim Unlock still
        // routes through stage-1.
        let bus = setup_with_cals(VehicleLineCal::default(), true, DriverDoorSide::Left).await;
        bus.inject("Cabin.LockStatus", SignalValue::String("LOCKED".into()));
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        bus.inject(
            "Body.Switches.DoorTrim.Row1.Left.UnlockButton",
            SignalValue::Bool(true),
        );
        settle().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("unlock_driver".into())),
            "expected unlock_driver (stage-1) when cabin is LOCKED, history: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn passenger_trim_unlock_in_driver_unlocked_state_still_unlock_all() {
        // Passenger trim Unlock always unlocks all — the new stage-2
        // escalation rule for the driver side doesn't change this.
        let bus = setup_with_cals(VehicleLineCal::default(), true, DriverDoorSide::Left).await;
        bus.inject(
            "Cabin.LockStatus",
            SignalValue::String("DRIVER_UNLOCKED".into()),
        );
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        bus.inject(
            "Body.Switches.DoorTrim.Row1.Right.UnlockButton",
            SignalValue::Bool(true),
        );
        settle().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("unlock_all".into())),
            "passenger trim Unlock must dispatch unlock_all in DRIVER_UNLOCKED state: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn trim_presses_suppressed_under_double_lock() {
        // Super-lock physically disconnects the interior linkage —
        // BOTH Lock and Unlock trim presses must be ignored.  Verified
        // across all four physical buttons × both DriverDoorSide cals
        // so the rule applies uniformly regardless of which trim
        // button maps to "driver."
        for driver_side in [DriverDoorSide::Left, DriverDoorSide::Right] {
            for side_signal in [
                "Body.Switches.DoorTrim.Row1.Left.LockButton",
                "Body.Switches.DoorTrim.Row1.Right.LockButton",
                "Body.Switches.DoorTrim.Row1.Left.UnlockButton",
                "Body.Switches.DoorTrim.Row1.Right.UnlockButton",
            ] {
                let bus = setup_with_cals(VehicleLineCal::default(), true, driver_side).await;
                bus.inject(
                    "Cabin.LockStatus",
                    SignalValue::String("DOUBLE_LOCKED".into()),
                );
                for _ in 0..16 {
                    tokio::task::yield_now().await;
                }
                bus.inject(side_signal, SignalValue::Bool(true));
                settle().await;

                assert!(
                    !bus.history()
                        .iter()
                        .any(|(s, _)| *s == "Body.Doors.CentralLock.Command"),
                    "DOUBLE_LOCKED + {} (driver_side={:?}): trim press must NOT dispatch any command",
                    side_signal,
                    driver_side,
                );
                assert!(
                    !bus.history()
                        .iter()
                        .any(|(s, _)| *s == FEEDBACK_REQUEST),
                    "DOUBLE_LOCKED + {} (driver_side={:?}): trim press must NOT publish any feedback",
                    side_signal,
                    driver_side,
                );
            }
        }
    }

    #[tokio::test]
    async fn release_edge_is_a_noop() {
        let bus = setup().await;
        bus.inject(
            "Body.Switches.DoorTrim.Row1.Left.LockButton",
            SignalValue::Bool(false),
        );
        settle().await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, _)| *s == "Body.Doors.CentralLock.Command"),
            "release edge must not dispatch a lock command"
        );
    }

    #[tokio::test]
    async fn last_requestor_is_door_trim_button() {
        let bus = setup().await;
        bus.inject(
            "Body.Switches.DoorTrim.Row1.Left.LockButton",
            SignalValue::Bool(true),
        );
        settle().await;

        assert_eq!(
            bus.latest_value("Cabin.LockStatus.LastRequestor"),
            Some(SignalValue::String("DoorTrimButton".into())),
            "DoorTrimButton must be the last requestor recorded by the arbiter"
        );
    }

    // ── Slam-lock three-way branch (lock-button × door-state × cal) ────

    #[tokio::test]
    async fn slam_lock_off_open_door_dispatches_as_slam_lock() {
        // US-style cal: slam-lock allowed.  Trim lock with any door
        // open dispatches with FeatureId::SlamLock so PerimeterAlarm
        // sees a lock event from a recognised external requestor and
        // arms when the door physically closes.
        let vl = VehicleLineCal {
            slam_lock_protect: false,
            ..VehicleLineCal::default()
        };
        let bus = setup_with_cal(vl).await;

        // A door is open at the time of trim press.
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }

        bus.inject(
            "Body.Switches.DoorTrim.Row1.Left.LockButton",
            SignalValue::Bool(true),
        );
        settle().await;

        assert_eq!(
            bus.latest_value("Cabin.LockStatus.LastRequestor"),
            Some(SignalValue::String("SlamLock".into())),
            "US slam-lock must publish SlamLock as the requestor"
        );
        assert_eq!(
            bus.latest_value("Cabin.LockStatus"),
            Some(SignalValue::String("LOCKED".into()))
        );
    }

    #[tokio::test]
    async fn slam_lock_on_open_door_dispatches_as_door_trim_button() {
        // EU-style cal: slam-lock-protect on.  DoorTrimButton itself
        // dispatches the lock as DoorTrimButton (the SlamLock feature
        // is responsible for the corresponding inversion unlock; that
        // logic is exercised in the slam_lock module's own tests).
        let vl = VehicleLineCal {
            slam_lock_protect: true,
            ..VehicleLineCal::default()
        };
        let bus = setup_with_cal(vl).await;

        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }

        bus.inject(
            "Body.Switches.DoorTrim.Row1.Left.LockButton",
            SignalValue::Bool(true),
        );
        settle().await;

        assert_eq!(
            bus.latest_value("Cabin.LockStatus.LastRequestor"),
            Some(SignalValue::String("DoorTrimButton".into())),
            "EU slam-lock-protect: DoorTrimButton stays the lock requestor; SlamLock undoes it later"
        );
    }

    #[tokio::test]
    async fn closed_doors_always_dispatch_as_door_trim_button() {
        // Doors closed: cal value doesn't matter.  Lock dispatches as
        // DoorTrimButton regardless — it's a regular interior lock, not
        // a slam-lock.
        for protect in [false, true] {
            let vl = VehicleLineCal {
                slam_lock_protect: protect,
                ..VehicleLineCal::default()
            };
            let bus = setup_with_cal(vl).await;
            // Doors stay closed (no inject).
            bus.inject(
                "Body.Switches.DoorTrim.Row1.Left.LockButton",
                SignalValue::Bool(true),
            );
            settle().await;

            assert_eq!(
                bus.latest_value("Cabin.LockStatus.LastRequestor"),
                Some(SignalValue::String("DoorTrimButton".into())),
                "doors closed must always dispatch as DoorTrimButton (cal protect={})",
                protect
            );
        }
    }
}
