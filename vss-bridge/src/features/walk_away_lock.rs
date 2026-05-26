//! Walk-Away Lock — locks the vehicle when all PEPS devices leave the approach zone.
//!
//! Monitors BLE phone zone signals and key fob zone signals. When at least one
//! device has been detected in the approach zone (or closer) and then ALL tracked
//! devices subsequently leave the approach zone (transition to RfRange or OutOfRange),
//! the feature issues a `LockAll` command and publishes a `"lock"` FeedbackRequest.
//!
//! # Zone hierarchy
//! ```text
//! OutOfRange → RfRange → Approach → LeftFront / RightFront / Hood / Trunk / …
//! ```
//! "In approach" means zone is `Approach`, `LeftFront`, `RightFront`, `Hood`,
//! `Trunk`, `TrunkInside`, or `Cabin` (i.e., any zone closer than `RfRange`).
//!
//! # Armed state
//! The feature is *armed* per-device when that device enters the approach zone.
//! It fires when every *currently-armed* device is back outside the approach zone.
//! After firing, the armed set is cleared and the feature waits for the next entry.
//!
//! # Closure gate
//! Before WAL will dispatch a lock, **every cabin door, the trunk,
//! and the hood must be closed at the moment of dispatch**.  Any open
//! opening means the driver is still actively using the vehicle
//! (loading bags, kid still climbing out, mechanic under the hood)
//! and is overwhelmingly likely to come back — auto-locking now is
//! hostile UX, and in the unloading case it also raises the risk of
//! a fob getting set down on a seat or in the cargo bay during the
//! flurry of activity.  Closing every opening is a deliberate "I am
//! done with this car" gesture; only after that gesture is the
//! walk-away signal trustworthy.  Closure is checked **at dispatch
//! time**, not at arm time, so a door re-opened mid-walkaway (kid
//! comes back for forgotten phone) still suppresses the lock.
//!
//! # Interior-key guard
//! After the closure gate passes, WAL submits a `Cabin + TrunkInside`
//! authenticated key-search.  If any paired fob is still inside the
//! vehicle, the lock is held off — locking would leave the only
//! paired key sealed inside, which is exactly the lockout this
//! feature must not cause.  The armed set is preserved across the
//! hold-off so the next zone event can re-evaluate.  The closure
//! gate catches the common cases at the source; the interior-key
//! guard is the belt-and-suspenders for the rare "driver closed
//! every door but walked away with no fob" edge.
//!
//! # Scope
//! Tracks 4 keyfobs and 2 BLE phones (the full device set in the simulation).
//! Walk-away lock does NOT apply to NFC cards (held at the handle — users are
//! still at the vehicle when NFC reads).

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::arbiter::{DoorLockArbiter, DoorLockRequest, LockCommand, FEEDBACK_REQUEST};
use crate::features::key_search_arbiter::{
    AntennaSet, Coalescing, KeySearchArbiterHandle, SearchMode,
};
use crate::ipc_message::{FeatureId, SignalValue};
use crate::plant_models::peps::zone::Zone;
use crate::signal_bus::SignalBus;

// Walk-away tracks per-device LastObservedZone (item #14a of the
// post-PEPS backlog) instead of the legacy `.Zone` ground-truth
// mirror.  The KeySearchArbiter publishes LastObservedZone after
// each periodic approach poll: positive values for fobs found in
// coverage, OutOfRange for fobs that have left coverage since the
// previous scan.  Walk-away therefore experiences the same
// partial-information world a real PEPS feature does — it can only
// react to what the antennas actually saw, not to HMI ground truth.
const FOB_ZONE_SIGNALS: [&str; 4] = [
    "Body.PEPS.Plant.KeyFob.1.LastObservedZone",
    "Body.PEPS.Plant.KeyFob.2.LastObservedZone",
    "Body.PEPS.Plant.KeyFob.3.LastObservedZone",
    "Body.PEPS.Plant.KeyFob.4.LastObservedZone",
];

const PHONE_ZONE_SIGNALS: [&str; 2] = [
    "Body.PEPS.Plant.BlePhone.1.LastObservedZone",
    "Body.PEPS.Plant.BlePhone.2.LastObservedZone",
];

const NUM_FOBS: usize = 4;
const NUM_PHONES: usize = 2;
const NUM_DEVICES: usize = NUM_FOBS + NUM_PHONES;

/// Cabin door IsOpen signals.  WAL holds off until every cabin door
/// is closed — a door still open means the driver is mid-activity
/// (loading bags, kid getting out, etc.), not actually walking away.
const DOOR_OPEN_SIGNALS: [&str; 4] = [
    "Body.Doors.Row1.Left.IsOpen",
    "Body.Doors.Row1.Right.IsOpen",
    "Body.Doors.Row2.Left.IsOpen",
    "Body.Doors.Row2.Right.IsOpen",
];

/// Trunk + hood openings.  Same rationale as the door gate: trunk
/// open = loading; hood open = mechanic / pre-trip inspection.
/// Neither is "I'm done with the car, lock it behind me."
const TRUNK_IS_OPEN: &str = "Body.Trunk.IsOpen";
const HOOD_IS_OPEN: &str = "Body.Hood.IsOpen";

/// True if a zone string value represents "in approach zone or closer".
fn zone_is_in_approach(val: &SignalValue) -> bool {
    matches!(
        val,
        SignalValue::String(s) if matches!(
            s.as_str(),
            "Approach" | "LeftFront" | "RightFront" | "Hood" | "Trunk" | "TrunkInside" | "Cabin"
        )
    )
}

/// True if a zone string value represents "outside approach zone".
fn zone_is_outside_approach(val: &SignalValue) -> bool {
    matches!(
        val,
        SignalValue::String(s) if matches!(s.as_str(), "OutOfRange" | "RfRange")
    )
}

pub struct WalkAwayLock<B: SignalBus> {
    bus: Arc<B>,
    arbiter: Arc<DoorLockArbiter>,
    key_search: KeySearchArbiterHandle,
}

impl<B: SignalBus + Send + Sync + 'static> WalkAwayLock<B> {
    pub fn new(
        bus: Arc<B>,
        arbiter: Arc<DoorLockArbiter>,
        key_search: KeySearchArbiterHandle,
    ) -> Self {
        Self {
            bus,
            arbiter,
            key_search,
        }
    }

    pub async fn run(self) {
        // Subscribe to all fob and phone zone signals.
        let fob_streams =
            futures::future::join_all(FOB_ZONE_SIGNALS.iter().map(|&sig| self.bus.subscribe(sig)))
                .await;
        let phone_streams = futures::future::join_all(
            PHONE_ZONE_SIGNALS
                .iter()
                .map(|&sig| self.bus.subscribe(sig)),
        )
        .await;

        let mut fob_zones = futures::stream::select_all(
            fob_streams
                .into_iter()
                .enumerate()
                .map(|(i, s)| futures::stream::StreamExt::map(s, move |v| (i, v))),
        );
        let mut phone_zones = futures::stream::select_all(
            phone_streams
                .into_iter()
                .enumerate()
                .map(|(i, s)| futures::stream::StreamExt::map(s, move |v| (NUM_FOBS + i, v))),
        );

        // Closure-state streams.  Used only to maintain caches —
        // never trigger the lock decision on their own.  A door
        // *closing* with armed devices already outside still has to
        // wait for the next zone event to drive the decision; that's
        // intentional because in practice the "all closed" edge
        // arrives well before the last fob hits RfRange (driver shuts
        // door at the car, then walks).
        let mut door_streams = futures::stream::select_all(
            futures::future::join_all(
                DOOR_OPEN_SIGNALS.iter().map(|&sig| self.bus.subscribe(sig)),
            )
            .await
            .into_iter()
            .enumerate()
            .map(|(i, s)| futures::stream::StreamExt::map(s, move |v| (i, v))),
        );
        let mut trunk_open_rx = self.bus.subscribe(TRUNK_IS_OPEN).await;
        let mut hood_open_rx = self.bus.subscribe(HOOD_IS_OPEN).await;

        // Per-device: true = device is currently in approach zone or closer.
        let mut in_approach = [false; NUM_DEVICES];
        // Per-device: true = device has entered approach zone since last lock.
        let mut was_armed = [false; NUM_DEVICES];
        // Closure caches.  Default-closed (matches typical boot state
        // of an idle vehicle); the first published value updates the
        // cache before any walkaway event can be evaluated against it.
        let mut door_open = [false; DOOR_OPEN_SIGNALS.len()];
        let mut trunk_open = false;
        let mut hood_open = false;

        tracing::info!("WalkAwayLock feature started");

        loop {
            let (device_idx, zone_val) = select! {
                Some(pair) = fob_zones.next() => pair,
                Some(pair) = phone_zones.next() => pair,
                // Closure-state updates — pure cache writes, then
                // continue the loop without evaluating the lock gate.
                Some((idx, val)) = door_streams.next() => {
                    if let SignalValue::Bool(b) = val {
                        door_open[idx] = b;
                    }
                    continue;
                }
                Some(val) = trunk_open_rx.next() => {
                    if let SignalValue::Bool(b) = val {
                        trunk_open = b;
                    }
                    continue;
                }
                Some(val) = hood_open_rx.next() => {
                    if let SignalValue::Bool(b) = val {
                        hood_open = b;
                    }
                    continue;
                }
                else => break,
            };

            let prev_in = in_approach[device_idx];
            let now_in = zone_is_in_approach(&zone_val);
            let now_out = zone_is_outside_approach(&zone_val);

            in_approach[device_idx] = now_in;

            if now_in {
                // Device entered approach — arm it.
                was_armed[device_idx] = true;
            }

            if now_out && prev_in {
                // Device just left the approach zone — check if all armed devices are now out.
                let any_armed = was_armed.iter().any(|&a| a);
                let all_armed_outside = was_armed
                    .iter()
                    .zip(in_approach.iter())
                    .all(|(&armed, &in_ap)| !armed || !in_ap);

                if any_armed && all_armed_outside {
                    // Closure gate.  Every cabin door, the trunk, and
                    // the hood must be closed *right now* (not "were
                    // closed when the fob left").  An opening still
                    // open means the driver is mid-activity — loading
                    // bags, kid getting out, mechanic under the hood —
                    // and they're coming back.  Holding off here also
                    // means we never reach the interior-key scan in
                    // the common loading case, saving the LF airtime.
                    let any_opening_open =
                        door_open.iter().any(|&b| b) || trunk_open || hood_open;
                    if any_opening_open {
                        tracing::info!(
                            door_open = ?door_open,
                            trunk_open,
                            hood_open,
                            "WalkAwayLock: armed devices outside but a door / trunk / \
                             hood is still open — holding off (driver still active)"
                        );
                        // Preserve `was_armed` so the next zone event
                        // re-evaluates with the closure state at that
                        // moment.  No interior scan, no lock.
                        continue;
                    }

                    tracing::info!(
                        device = device_idx,
                        "WalkAwayLock: all armed devices left approach + cabin sealed — \
                         running interior key check"
                    );

                    // Interior-key guard.  All tracked devices are out
                    // of approach, but that only covers exterior LF
                    // zones; a paired fob sitting on the cabin seat or
                    // in the trunk-inside cargo bay never enters/leaves
                    // approach via the periodic poll, so the FSM above
                    // can't see it.  Submit a Cabin + TrunkInside
                    // authenticated scan and hold off if any paired
                    // key is still inside.  Without this guard the
                    // driver could walk away with no fob on them and
                    // we'd cheerfully lock the only paired key inside.
                    let interior = self
                        .key_search
                        .submit(
                            "WalkAwayLock",
                            AntennaSet::Sequence(vec![
                                (AntennaSet::Cabin, SearchMode::Authenticated),
                                (AntennaSet::TrunkInside, SearchMode::Authenticated),
                            ]),
                            SearchMode::Authenticated,
                            Coalescing::Disallowed,
                        )
                        .await;
                    let interior_key_present = interior
                        .as_ref()
                        .map(|r| {
                            r.keys_found
                                .iter()
                                .any(|k| matches!(k.zone, Zone::Cabin | Zone::TrunkInside))
                        })
                        .unwrap_or(false);

                    if interior_key_present {
                        tracing::warn!(
                            "WalkAwayLock: paired key still in cabin / trunk-inside — \
                             holding off LockAll to prevent locking driver out"
                        );
                        // Leave `was_armed` set so the next zone event
                        // can re-evaluate (e.g. the interior fob gets
                        // picked up and walks out of approach).
                    } else {
                        tracing::info!("WalkAwayLock: no interior paired key — locking");

                        if let Err(e) = self
                            .arbiter
                            .request(DoorLockRequest {
                                command: LockCommand::LockAll,
                                feature_id: FeatureId::WalkAwayLock,
                            })
                            .await
                        {
                            tracing::error!(error = %e, "WalkAwayLock: arbiter error");
                        }
                        let _ = self
                            .bus
                            .publish(FEEDBACK_REQUEST, SignalValue::String("lock".into()))
                            .await;

                        // Reset armed state — wait for next approach entry.
                        was_armed = [false; NUM_DEVICES];
                    }
                }
            }
        }

        tracing::info!("WalkAwayLock feature stopped");
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::door_lock_arbiter;
    use crate::features::key_search_arbiter::KeySearchArbiter;
    use tokio::time::{sleep, Duration};

    /// Wait long enough for the interior-key scan sequence
    /// (Cabin Auth ~100ms + TrunkInside Auth ~100ms) to complete and
    /// for the spawned tasks to settle.
    async fn wait_for_interior_scan() {
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
        sleep(Duration::from_millis(300)).await;
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
    }

    async fn setup() -> (Arc<MockBus>, tokio::task::JoinHandle<()>) {
        let bus = Arc::new(MockBus::new());
        let (arb, _ack_tx, loop_fut) = door_lock_arbiter(Arc::clone(&bus));
        tokio::spawn(loop_fut);
        let arb = Arc::new(arb);
        let (ksa, handle, rx) = KeySearchArbiter::new_with_rx(Arc::clone(&bus));
        tokio::spawn(ksa.run(rx));
        let feature = WalkAwayLock::new(Arc::clone(&bus), arb, handle);
        let h = tokio::spawn(feature.run());
        tokio::task::yield_now().await;
        (bus, h)
    }

    #[tokio::test]
    async fn fob_approach_then_leave_triggers_lock() {
        let (bus, _h) = setup().await;

        // Fob 1 enters approach zone
        bus.inject(FOB_ZONE_SIGNALS[0], SignalValue::String("Approach".into()));
        tokio::task::yield_now().await;

        bus.clear_history();

        // Fob 1 leaves approach zone → all armed devices are outside
        bus.inject(
            FOB_ZONE_SIGNALS[0],
            SignalValue::String("OutOfRange".into()),
        );
        tokio::task::yield_now().await;
        wait_for_interior_scan().await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("lock_all".into())),
            "expected lock_all command, history: {:?}",
            h
        );
        assert!(
            h.iter()
                .any(|(s, v)| *s == FEEDBACK_REQUEST && *v == SignalValue::String("lock".into())),
            "expected lock FeedbackRequest, history: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn no_lock_when_second_device_still_in_approach() {
        let (bus, _h) = setup().await;

        // Two fobs enter approach
        bus.inject(FOB_ZONE_SIGNALS[0], SignalValue::String("Approach".into()));
        bus.inject(FOB_ZONE_SIGNALS[1], SignalValue::String("Approach".into()));
        tokio::task::yield_now().await;

        bus.clear_history();

        // Only fob 1 leaves — fob 2 still in approach
        bus.inject(
            FOB_ZONE_SIGNALS[0],
            SignalValue::String("OutOfRange".into()),
        );
        tokio::task::yield_now().await;
        wait_for_interior_scan().await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, _)| *s == "Body.Doors.CentralLock.Command"),
            "should NOT lock while fob 2 still in approach, history: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn never_armed_device_does_not_prevent_lock() {
        let (bus, _h) = setup().await;

        // Only fob 1 ever enters approach (fobs 2-4 and phones stay out)
        bus.inject(FOB_ZONE_SIGNALS[0], SignalValue::String("Approach".into()));
        tokio::task::yield_now().await;
        bus.clear_history();

        // Fob 1 leaves — only it was armed, and it's now out
        bus.inject(FOB_ZONE_SIGNALS[0], SignalValue::String("RfRange".into()));
        tokio::task::yield_now().await;
        wait_for_interior_scan().await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("lock_all".into())),
            "only armed device leaving should trigger lock, history: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn arbiter_drives_walk_away_end_to_end() {
        // End-to-end proof that the new signal chain works:
        //   HMI PlacedZone → KeySearchArbiter approach poll →
        //   LastObservedZone publish → WalkAwayLock → LockAll.
        //
        // Spawn the arbiter (with a brisk test cadence) alongside
        // walk-away.  Place fob 1 in the Approach zone via
        // PlacedZone, wait for a poll cycle, then move it
        // OutOfRange and verify walk-away dispatches LockAll.
        use crate::features::key_search_arbiter::KeySearchArbiter;
        use std::time::Duration;

        let bus = Arc::new(MockBus::new());
        let (arb, _ack_tx, loop_fut) = door_lock_arbiter(Arc::clone(&bus));
        tokio::spawn(loop_fut);
        let arb = Arc::new(arb);

        let (ksa, ksa_handle, rx) = KeySearchArbiter::new_with_rx(Arc::clone(&bus));
        tokio::spawn(
            ksa.with_cadence(Duration::from_millis(20), Duration::from_millis(40))
                .run(rx),
        );

        tokio::spawn(WalkAwayLock::new(Arc::clone(&bus), arb, ksa_handle).run());
        tokio::task::yield_now().await;

        // Place fob 1 in approach (via PlacedZone — the HMI's write
        // target).  The arbiter's PlacedZone subscription updates
        // its cache; the next poll publishes LastObservedZone=Approach;
        // walk-away arms.
        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.PlacedZone",
            SignalValue::String("Approach".into()),
        );
        sleep(Duration::from_millis(80)).await; // let a couple of polls run
        bus.clear_history();

        // Move fob OutOfRange.  Next poll: arbiter's diff detects
        // "fob was in approach coverage, not found now" and publishes
        // LastObservedZone=OutOfRange.  Walk-away sees the transition
        // and fires.
        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.PlacedZone",
            SignalValue::String("OutOfRange".into()),
        );
        // Allow time for: the next approach poll to publish OutOfRange,
        // walk-away to fire the interior scan, and the scan to complete.
        sleep(Duration::from_millis(400)).await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("lock_all".into())),
            "expected lock_all command from end-to-end arbiter→walk-away chain, history: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn paired_fob_in_cabin_holds_off_lock() {
        // Driver walks away with no fob; a paired fob is sitting on
        // the cabin seat.  The exterior FSM is satisfied (no fob in
        // approach), but the interior scan must catch the cabin fob
        // and suppress the LockAll.  Without this guard the only
        // paired key would be sealed inside the locked car.
        let (bus, _h) = setup().await;

        // Mark the cabin fob as paired so the Authenticated scan
        // returns it, and place it in Cabin via PlacedZone (the
        // arbiter's ground truth).
        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.Paired",
            SignalValue::Bool(true),
        );
        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.PlacedZone",
            SignalValue::String("Cabin".into()),
        );

        // Driver's fob (slot 2) enters approach then leaves.
        bus.inject(FOB_ZONE_SIGNALS[1], SignalValue::String("Approach".into()));
        tokio::task::yield_now().await;
        bus.clear_history();

        bus.inject(
            FOB_ZONE_SIGNALS[1],
            SignalValue::String("OutOfRange".into()),
        );
        wait_for_interior_scan().await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                    && *v == SignalValue::String("lock_all".into())),
            "interior paired key must block WAL; history: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn unpaired_fob_in_cabin_does_not_block_lock() {
        // Belt-and-suspenders: an unpaired stranger's fob sitting in
        // the cabin must NOT block walk-away.  The Authenticated scan
        // filters unpaired fobs out, so the interior check sees
        // zero paired keys and the lock proceeds.
        let (bus, _h) = setup().await;

        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.Paired",
            SignalValue::Bool(false),
        );
        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.PlacedZone",
            SignalValue::String("Cabin".into()),
        );

        bus.inject(FOB_ZONE_SIGNALS[1], SignalValue::String("Approach".into()));
        tokio::task::yield_now().await;
        bus.clear_history();

        bus.inject(
            FOB_ZONE_SIGNALS[1],
            SignalValue::String("OutOfRange".into()),
        );
        wait_for_interior_scan().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("lock_all".into())),
            "unpaired interior fob should not block WAL; history: {:?}",
            h
        );
    }

    /// Helper for the closure-gate tests: arm WAL by entering then
    /// leaving the approach zone with one fob.  Returns the bus so
    /// the caller can inspect history and inject more events.
    async fn walk_in_and_out(bus: &MockBus, slot: usize) {
        bus.inject(FOB_ZONE_SIGNALS[slot], SignalValue::String("Approach".into()));
        tokio::task::yield_now().await;
        bus.clear_history();
        bus.inject(
            FOB_ZONE_SIGNALS[slot],
            SignalValue::String("OutOfRange".into()),
        );
        wait_for_interior_scan().await;
    }

    fn lock_all_dispatched(bus: &MockBus) -> bool {
        bus.history().iter().any(|(s, v)| {
            *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("lock_all".into())
        })
    }

    #[tokio::test]
    async fn open_cabin_door_blocks_lock() {
        let (bus, _h) = setup().await;
        // Driver door still open at the moment the fob leaves approach.
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        tokio::task::yield_now().await;

        walk_in_and_out(&bus, 0).await;

        assert!(
            !lock_all_dispatched(&bus),
            "cabin door open must block WAL; history: {:?}",
            bus.history()
        );
    }

    #[tokio::test]
    async fn open_trunk_blocks_lock() {
        let (bus, _h) = setup().await;
        bus.inject(TRUNK_IS_OPEN, SignalValue::Bool(true));
        tokio::task::yield_now().await;

        walk_in_and_out(&bus, 0).await;

        assert!(!lock_all_dispatched(&bus));
    }

    #[tokio::test]
    async fn open_hood_blocks_lock() {
        let (bus, _h) = setup().await;
        bus.inject(HOOD_IS_OPEN, SignalValue::Bool(true));
        tokio::task::yield_now().await;

        walk_in_and_out(&bus, 0).await;

        assert!(!lock_all_dispatched(&bus));
    }

    #[tokio::test]
    async fn closure_gate_re_evaluates_on_next_zone_tick() {
        // Loading-groceries-then-walking-away sequence:
        //   1. Driver opens passenger door (loading).
        //   2. Driver's fob enters approach, then leaves approach
        //      while the door is still open → WAL holds off.
        //   3. Driver closes the door.
        //   4. Fob does another approach→leave cycle (driver walked
        //      back to the car for one more bag, then left again).
        //   5. WAL fires this time.
        let (bus, _h) = setup().await;

        bus.inject("Body.Doors.Row1.Right.IsOpen", SignalValue::Bool(true));
        walk_in_and_out(&bus, 0).await;
        assert!(
            !lock_all_dispatched(&bus),
            "first pass with door open must not lock"
        );

        // Door closes; fob does another in/out cycle.
        bus.inject("Body.Doors.Row1.Right.IsOpen", SignalValue::Bool(false));
        bus.clear_history();
        walk_in_and_out(&bus, 0).await;

        assert!(
            lock_all_dispatched(&bus),
            "after closing the door, next walk-away cycle should lock; history: {:?}",
            bus.history()
        );
    }

    #[tokio::test]
    async fn door_reopened_after_arm_blocks_dispatch() {
        // Closure is checked at dispatch time, not arm time.  Fob
        // enters approach with everything closed (arm); a kid then
        // opens the rear door (forgotten phone); fob leaves approach.
        // The fact that the closure state was good at arm time must
        // not save us — re-opening before the lock dispatch suppresses.
        let (bus, _h) = setup().await;

        bus.inject(FOB_ZONE_SIGNALS[0], SignalValue::String("Approach".into()));
        tokio::task::yield_now().await;

        // Mid-walkaway: a rear door opens.
        bus.inject("Body.Doors.Row2.Left.IsOpen", SignalValue::Bool(true));
        tokio::task::yield_now().await;
        bus.clear_history();

        bus.inject(
            FOB_ZONE_SIGNALS[0],
            SignalValue::String("OutOfRange".into()),
        );
        wait_for_interior_scan().await;

        assert!(
            !lock_all_dispatched(&bus),
            "door opened mid-walkaway must block; history: {:?}",
            bus.history()
        );
    }

    #[tokio::test]
    async fn device_never_entered_approach_no_lock_on_leave() {
        let (bus, _h) = setup().await;

        // Fob 1 goes directly from (implicit initial) OutOfRange to RfRange
        // without ever having been in approach — no armed device, no lock.
        bus.inject(
            FOB_ZONE_SIGNALS[0],
            SignalValue::String("OutOfRange".into()),
        );
        tokio::task::yield_now().await;
        wait_for_interior_scan().await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, _)| *s == "Body.Doors.CentralLock.Command"),
            "device never in approach should NOT trigger lock, history: {:?}",
            h
        );
    }
}
