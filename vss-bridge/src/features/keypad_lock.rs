//! Keypad Lock — lock from outside door handle capacitive pads (Row 1 only).
//!
//! Row 1 (driver and front passenger) outside door handles have a capacitive thumb
//! pad on the trailing edge. Pressing and *holding* the pad for **500 ms** locks
//! all doors. This provides a convenient walk-up lock without needing the key fob.
//!
//! # Design notes
//! - Only Row 1 Left and Row 1 Right have keypads — Row 2 has no capacitive area.
//! - Debounce is 500 ms: the lock fires at exactly 500 ms of continuous press, not
//!   on release. A release before 500 ms cancels the pending lock.
//! - A new press while debouncing resets the 500 ms window (anti-spam guard).
//! - Each pad is independent: either pad alone is sufficient to lock.
//! - Publishes `FeedbackRequest = "lock"` alongside the `LockAll` command
//!   (external trigger — user is outside the vehicle).
//!
//! # PEPS-presence gate (REQ-PL-002)
//!
//! The lock fires only when **at least one paired PEPS device is in a
//! zone outside the cabin** (LeftFront / RightFront / Hood / Trunk /
//! Approach).  This is the canonical "keys-in-vehicle" guard: a child
//! inside the cabin can't accidentally lock the keys in the vehicle by
//! pressing the keypad through the open door, because the only
//! paired devices in range are inside (Cabin / TrunkInside) and the
//! lock command is denied.
//!
//! ## Fresh-scan policy
//!
//! The gate is evaluated against a **fresh authenticated scan**
//! submitted at debounce-complete, **not** against the cached
//! `LastObservedZone` subscription.  The arbiter's approach poll
//! switches to a 10 s cadence as soon as a fob is detected in
//! approach, so the cached zone for that fob can be up to 10 s
//! stale.  Without the fresh scan, a driver who got out, leaned
//! back in to drop their fob on the seat, and then tapped the thumb
//! pad within that window would have the cache say "fob outside" →
//! lock fires → fob sealed inside.  Running a `Cabin + AllApproach`
//! scan at the moment of decision closes the window.  Coalescing is
//! disallowed for this exact reason — a result from a poll moments
//! ago is precisely the thing we're trying to avoid trusting.
//!
//! When the gate denies a lock attempt, `FeedbackRequest = "lock_denied"`
//! is published — distinct from `"lock"` so LockFeedback (or future
//! HMI alert) can show a different cue if/when wired up.  Today
//! LockFeedback ignores unknown kinds, so the publish is a hint
//! visible in the bus history / HMI signal log.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;
use tokio::time::{sleep, Duration, Instant};

use crate::arbiter::{DoorLockArbiter, DoorLockRequest, LockCommand, FEEDBACK_REQUEST};
use crate::features::key_search_arbiter::{
    AntennaSet, Coalescing, KeySearchArbiterHandle, SearchMode,
};
use crate::ipc_message::{FeatureId, SignalValue};
use crate::plant_models::peps::zone::Zone;
use crate::signal_bus::SignalBus;

const LEFT_PAD: &str = "Body.Doors.Row1.Left.Handle.Outside.LockPad.IsPressed";
const RIGHT_PAD: &str = "Body.Doors.Row1.Right.Handle.Outside.LockPad.IsPressed";

/// Debounce duration before the lock fires.
const DEBOUNCE: Duration = Duration::from_millis(500);

/// Returns true if `zone` represents "outside the cabin" — i.e. the
/// device is somewhere a person who's exiting the vehicle would have
/// it (LeftFront / RightFront / Hood / Trunk / Approach).  Inside-
/// cabin zones (Cabin, TrunkInside) and beyond-range zones (RfRange,
/// OutOfRange) all return false.
fn is_outside_cabin(zone: Zone) -> bool {
    matches!(
        zone,
        Zone::LeftFront | Zone::RightFront | Zone::Hood | Zone::Trunk | Zone::Approach
    )
}

pub struct KeypadLock<B: SignalBus> {
    bus: Arc<B>,
    arbiter: Arc<DoorLockArbiter>,
    key_search: KeySearchArbiterHandle,
}

impl<B: SignalBus + Send + Sync + 'static> KeypadLock<B> {
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
        let mut left_rx = self.bus.subscribe(LEFT_PAD).await;
        let mut right_rx = self.bus.subscribe(RIGHT_PAD).await;

        // Track per-pad: when did the current press start (None = not pressed).
        let mut left_pressed_at: Option<Instant> = None;
        let mut right_pressed_at: Option<Instant> = None;

        tracing::info!("KeypadLock feature started");

        loop {
            // Compute the next debounce deadline (minimum over active pads).
            let left_remaining = left_pressed_at.map(|t| DEBOUNCE.saturating_sub(t.elapsed()));
            let right_remaining = right_pressed_at.map(|t| DEBOUNCE.saturating_sub(t.elapsed()));

            let debounce_sleep = [left_remaining, right_remaining]
                .into_iter()
                .flatten()
                .min()
                .unwrap_or(Duration::from_secs(3600));

            select! {
                Some(val) = left_rx.next() => {
                    match val {
                        SignalValue::Bool(true) => {
                            left_pressed_at = Some(Instant::now());
                        }
                        _ => {
                            left_pressed_at = None;
                        }
                    }
                }
                Some(val) = right_rx.next() => {
                    match val {
                        SignalValue::Bool(true) => {
                            right_pressed_at = Some(Instant::now());
                        }
                        _ => {
                            right_pressed_at = None;
                        }
                    }
                }
                _ = sleep(debounce_sleep) => {
                    // Check which pad(s) completed the debounce
                    let now = Instant::now();
                    let left_done = left_pressed_at
                        .map(|t| now.duration_since(t) >= DEBOUNCE)
                        .unwrap_or(false);
                    let right_done = right_pressed_at
                        .map(|t| now.duration_since(t) >= DEBOUNCE)
                        .unwrap_or(false);

                    if left_done || right_done {
                        // PEPS-presence gate (REQ-PL-002): require at
                        // least one paired device in a zone OUTSIDE
                        // the cabin.  We run a FRESH authenticated
                        // scan here rather than trusting the cached
                        // `device_zones`, because the approach poll's
                        // slow cadence (10 s) means the cache can be
                        // wildly out of date for fobs in approach —
                        // see file header.  Cabin first to short-
                        // circuit cleanly, then AllApproach for the
                        // "any device outside" leg.
                        let scan = self
                            .key_search
                            .submit(
                                "KeypadLock",
                                AntennaSet::Sequence(vec![
                                    (AntennaSet::Cabin, SearchMode::Authenticated),
                                    (AntennaSet::AllApproach, SearchMode::Authenticated),
                                ]),
                                SearchMode::Authenticated,
                                Coalescing::Disallowed,
                            )
                            .await;
                        let device_outside = scan
                            .as_ref()
                            .map(|r| r.keys_found.iter().any(|k| is_outside_cabin(k.zone)))
                            .unwrap_or(false);
                        if !device_outside {
                            tracing::warn!(
                                scan_keys = ?scan.as_ref().map(|r| r.keys_found.len()),
                                "KeypadLock: debounce complete but fresh scan found NO paired device outside cabin — lock denied (keys-in-vehicle guard)"
                            );
                            let _ = self
                                .bus
                                .publish(
                                    FEEDBACK_REQUEST,
                                    SignalValue::String("lock_denied".into()),
                                )
                                .await;
                            // Clear pads so a fresh press is needed to retry.
                            if left_done { left_pressed_at = None; }
                            if right_done { right_pressed_at = None; }
                            continue;
                        }

                        tracing::info!(
                            left = left_done,
                            right = right_done,
                            "KeypadLock: debounce complete — locking"
                        );

                        if let Err(e) = self
                            .arbiter
                            .request(DoorLockRequest {
                                command: LockCommand::LockAll,
                                feature_id: FeatureId::KeypadLock,
                            })
                            .await
                        {
                            tracing::error!(error = %e, "KeypadLock: arbiter error");
                        }
                        let _ = self
                            .bus
                            .publish(FEEDBACK_REQUEST, SignalValue::String("lock".into()))
                            .await;

                        // Clear fired pad(s) — require a new press to fire again.
                        if left_done { left_pressed_at = None; }
                        if right_done { right_pressed_at = None; }
                    }
                }
                else => break,
            }
        }

        tracing::info!("KeypadLock feature stopped");
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::door_lock_arbiter;
    use crate::features::key_search_arbiter::KeySearchArbiter;

    /// Combined debounce + fresh-scan latency.  DEBOUNCE = 500 ms
    /// triggers the gate evaluation; the gate's
    /// `Sequence(Cabin Auth ~100 ms + AllApproach Auth ~150 ms)` scan
    /// then needs to complete before LockAll dispatches.  Round up
    /// for select-loop scheduling slack.
    const TICK_FOR_LOCK: Duration = Duration::from_millis(900);

    /// Default test setup: bus, both arbiters, KeypadLock running,
    /// and **paired fob 1 placed in `Approach`** so the keys-in-
    /// vehicle gate passes for the happy-path tests.  Tests that
    /// need to verify the gate denial path use
    /// `setup_no_paired_device_outside`.
    async fn setup() -> (Arc<MockBus>, tokio::task::JoinHandle<()>) {
        let (bus, h) = setup_no_paired_device_outside().await;
        // The KSA reads `PlacedZone` as the HMI ground truth and
        // honors the `Paired` flag in Authenticated scans.  Set both.
        place_paired(&bus, 1, "Approach");
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;
        (bus, h)
    }

    /// Variant of `setup` that does NOT place any paired device.
    /// Used for tests that exercise the keys-in-vehicle denial path.
    async fn setup_no_paired_device_outside() -> (Arc<MockBus>, tokio::task::JoinHandle<()>) {
        let bus = Arc::new(MockBus::new());
        let (arb, _ack_tx, loop_fut) = door_lock_arbiter(Arc::clone(&bus));
        tokio::spawn(loop_fut);
        let arb = Arc::new(arb);
        let (ksa, key_search, ksa_rx) = KeySearchArbiter::new_with_rx(Arc::clone(&bus));
        tokio::spawn(ksa.run(ksa_rx));
        let feature = KeypadLock::new(Arc::clone(&bus), arb, key_search);
        let handle = tokio::spawn(feature.run());
        tokio::task::yield_now().await;
        (bus, handle)
    }

    /// Position a paired fob via PlacedZone (HMI ground truth) AND
    /// mark it Paired so the Authenticated scan returns it.
    fn place_paired(bus: &MockBus, fob_slot: u8, zone: &str) {
        let zone_path = match fob_slot {
            1 => "Body.PEPS.Plant.KeyFob.1.PlacedZone",
            2 => "Body.PEPS.Plant.KeyFob.2.PlacedZone",
            3 => "Body.PEPS.Plant.KeyFob.3.PlacedZone",
            4 => "Body.PEPS.Plant.KeyFob.4.PlacedZone",
            _ => panic!("unknown fob slot {fob_slot}"),
        };
        let paired_path = match fob_slot {
            1 => "Body.PEPS.Plant.KeyFob.1.Paired",
            2 => "Body.PEPS.Plant.KeyFob.2.Paired",
            3 => "Body.PEPS.Plant.KeyFob.3.Paired",
            4 => "Body.PEPS.Plant.KeyFob.4.Paired",
            _ => unreachable!(),
        };
        bus.inject(zone_path, SignalValue::String(zone.into()));
        bus.inject(paired_path, SignalValue::Bool(true));
    }

    /// Same for the BLE phone slots (5 + 6 in arbiter indexing).
    fn place_paired_phone(bus: &MockBus, phone_slot: u8, zone: &str) {
        let zone_path = match phone_slot {
            1 => "Body.PEPS.Plant.BlePhone.1.PlacedZone",
            2 => "Body.PEPS.Plant.BlePhone.2.PlacedZone",
            _ => panic!("unknown phone slot {phone_slot}"),
        };
        let paired_path = match phone_slot {
            1 => "Body.PEPS.Plant.BlePhone.1.Paired",
            2 => "Body.PEPS.Plant.BlePhone.2.Paired",
            _ => unreachable!(),
        };
        bus.inject(zone_path, SignalValue::String(zone.into()));
        bus.inject(paired_path, SignalValue::Bool(true));
    }

    #[tokio::test]
    async fn left_pad_held_500ms_locks() {
        let (bus, _h) = setup().await;

        bus.inject(LEFT_PAD, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        bus.clear_history();

        sleep(TICK_FOR_LOCK).await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("lock_all".into())),
            "expected lock_all after 500ms debounce, history: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn right_pad_held_500ms_locks() {
        let (bus, _h) = setup().await;

        bus.inject(RIGHT_PAD, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        bus.clear_history();

        sleep(TICK_FOR_LOCK).await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("lock_all".into())),
            "expected lock_all from right pad, history: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn release_before_debounce_cancels_lock() {
        let (bus, _h) = setup().await;

        bus.inject(LEFT_PAD, SignalValue::Bool(true));
        tokio::task::yield_now().await;

        // Release before 500 ms
        tokio::time::sleep(Duration::from_millis(200)).await;
        tokio::task::yield_now().await;
        bus.inject(LEFT_PAD, SignalValue::Bool(false));
        tokio::task::yield_now().await;

        // Advance well past 500 ms — no lock should fire
        tokio::time::sleep(Duration::from_millis(600)).await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, _)| *s == "Body.Doors.CentralLock.Command"),
            "release before debounce should cancel lock, history: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn lock_feedback_published_with_lock() {
        let (bus, _h) = setup().await;

        bus.inject(LEFT_PAD, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        bus.clear_history();

        sleep(TICK_FOR_LOCK).await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == FEEDBACK_REQUEST && *v == SignalValue::String("lock".into())),
            "expected lock FeedbackRequest alongside lock_all, history: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn does_not_refire_without_new_press() {
        let (bus, _h) = setup().await;

        bus.inject(LEFT_PAD, SignalValue::Bool(true));
        tokio::task::yield_now().await;

        sleep(TICK_FOR_LOCK).await;
        tokio::task::yield_now().await;

        let count_after_first = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == "Body.Doors.CentralLock.Command")
            .count();

        // Advance more — should not fire again without a new press
        tokio::time::sleep(Duration::from_millis(1000)).await;
        tokio::task::yield_now().await;

        let count_after_second = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == "Body.Doors.CentralLock.Command")
            .count();

        assert_eq!(
            count_after_first, count_after_second,
            "should not re-fire without a new press"
        );
    }

    // ── PEPS-presence gate (REQ-PL-002) ─────────────────────────────────

    /// 500 ms hold with NO paired device anywhere → lock denied,
    /// `lock_denied` feedback published instead of `LockAll`.
    #[tokio::test]
    async fn no_device_anywhere_denies_lock() {
        let (bus, _h) = setup_no_paired_device_outside().await;

        bus.inject(LEFT_PAD, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        bus.clear_history();

        sleep(TICK_FOR_LOCK).await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, _)| *s == "Body.Doors.CentralLock.Command"),
            "expected NO lock command when no paired device is outside, got {h:?}"
        );
        assert!(
            h.iter()
                .any(|(s, v)| *s == FEEDBACK_REQUEST
                    && *v == SignalValue::String("lock_denied".into())),
            "expected lock_denied feedback, got {h:?}"
        );
    }

    /// 500 ms hold with paired fob in `Cabin` (inside the vehicle) →
    /// keys-in-vehicle guard denies the lock.
    #[tokio::test]
    async fn fob_in_cabin_only_denies_lock() {
        let (bus, _h) = setup_no_paired_device_outside().await;
        place_paired(&bus, 1, "Cabin");
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.inject(LEFT_PAD, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        bus.clear_history();

        sleep(TICK_FOR_LOCK).await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, _)| *s == "Body.Doors.CentralLock.Command"),
            "lock must be denied when only paired device is inside the cabin: {h:?}"
        );
    }

    /// One fob in `Cabin` AND one fob in `Approach` → lock fires
    /// (someone outside the vehicle has a paired key).
    #[tokio::test]
    async fn fob_split_cabin_and_approach_locks() {
        let (bus, _h) = setup_no_paired_device_outside().await;
        place_paired(&bus, 1, "Cabin");
        place_paired(&bus, 2, "Approach");
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.inject(LEFT_PAD, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        bus.clear_history();

        sleep(TICK_FOR_LOCK).await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("lock_all".into())),
            "expected lock_all when at least one paired device is outside (split case): {h:?}"
        );
    }

    /// Paired phone in `LeftFront` (proximity zone) → lock fires.
    /// Phones go through the same gate as fobs.
    #[tokio::test]
    async fn phone_in_driver_door_zone_passes_gate() {
        let (bus, _h) = setup_no_paired_device_outside().await;
        place_paired_phone(&bus, 1, "LeftFront");
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.inject(RIGHT_PAD, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        bus.clear_history();

        sleep(TICK_FOR_LOCK).await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("lock_all".into())),
            "expected lock_all with phone in LeftFront zone: {h:?}"
        );
    }

    /// Staleness regression: prove the gate uses a FRESH scan, not
    /// the cached `LastObservedZone`.  Set up a stale "fob outside"
    /// LastObservedZone via a direct publish (simulating an old
    /// approach-poll result), but PlacedZone says the fob is actually
    /// in Cabin (HMI ground truth — what the next scan will report).
    /// Naive cache-trusting logic would permit the lock; the fresh-
    /// scan logic must deny it.
    #[tokio::test]
    async fn stale_last_observed_zone_does_not_fool_gate() {
        let (bus, _h) = setup_no_paired_device_outside().await;
        // Stale "fob outside" reading from a poll ~10 s ago.
        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.LastObservedZone",
            SignalValue::String("LeftFront".into()),
        );
        // But the fob is actually inside the cabin right now.
        place_paired(&bus, 1, "Cabin");
        tokio::task::yield_now().await;

        bus.inject(LEFT_PAD, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        bus.clear_history();

        sleep(TICK_FOR_LOCK).await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, _)| *s == "Body.Doors.CentralLock.Command"),
            "fresh scan must override stale LastObservedZone — lock should be denied: {h:?}"
        );
        assert!(
            h.iter()
                .any(|(s, v)| *s == FEEDBACK_REQUEST
                    && *v == SignalValue::String("lock_denied".into())),
            "expected lock_denied feedback: {h:?}"
        );
    }
}
