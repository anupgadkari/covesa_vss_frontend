//! Thumb-Pad Lock — lock from outside door handle capacitive pads (Row 1 only).
//!
//! Row 1 (driver and front passenger) outside door handles have a capacitive thumb
//! pad on the trailing edge. Pressing and *holding* the pad for **500 ms** locks
//! all doors. This provides a convenient walk-up lock without needing the key fob.
//!
//! # Design notes
//! - Only Row 1 Left and Row 1 Right have thumb pads — Row 2 has no capacitive area.
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
//! zone outside the cabin** (DriverDoor / PassengerDoor / Hood / Trunk /
//! Approach).  This is the canonical "keys-in-vehicle" guard: a child
//! inside the cabin can't accidentally lock the keys in the vehicle by
//! pressing the thumb pad through the open door, because the only
//! paired devices in range are inside (Cabin / TrunkInside) and the
//! lock command is denied.
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
use crate::ipc_message::{FeatureId, SignalValue};
use crate::plant_models::peps::signals as peps_signals;
use crate::plant_models::peps::zone::Zone;
use crate::signal_bus::{SignalBus, VssPath};

const LEFT_PAD: &str = "Body.Doors.Row1.Left.Handle.Outside.LockPad.IsPressed";
const RIGHT_PAD: &str = "Body.Doors.Row1.Right.Handle.Outside.LockPad.IsPressed";

/// All paired-device zone signals tracked for the keys-in-vehicle gate.
/// Slot order matches the PEPS plant model: 4 paired fobs + 2 phones.
const PAIRED_ZONE_SIGNALS: [VssPath; 6] = [
    "Body.PEPS.Plant.KeyFob.1.Zone",
    "Body.PEPS.Plant.KeyFob.2.Zone",
    "Body.PEPS.Plant.KeyFob.3.Zone",
    "Body.PEPS.Plant.KeyFob.4.Zone",
    peps_signals::PHONE_1_ZONE,
    peps_signals::PHONE_2_ZONE,
];

/// Debounce duration before the lock fires.
const DEBOUNCE: Duration = Duration::from_millis(500);

/// Returns true if `zone` represents "outside the cabin" — i.e. the
/// device is somewhere a person who's exiting the vehicle would have
/// it (DriverDoor / PassengerDoor / Hood / Trunk / Approach).  Inside-
/// cabin zones (Cabin, TrunkInside) and beyond-range zones (RfRange,
/// OutOfRange) all return false.
fn is_outside_cabin(zone: Zone) -> bool {
    matches!(
        zone,
        Zone::DriverDoor | Zone::PassengerDoor | Zone::Hood | Zone::Trunk | Zone::Approach
    )
}

pub struct ThumbPadLock<B: SignalBus> {
    bus: Arc<B>,
    arbiter: Arc<DoorLockArbiter>,
}

impl<B: SignalBus + Send + Sync + 'static> ThumbPadLock<B> {
    pub fn new(bus: Arc<B>, arbiter: Arc<DoorLockArbiter>) -> Self {
        Self { bus, arbiter }
    }

    pub async fn run(self) {
        let mut left_rx = self.bus.subscribe(LEFT_PAD).await;
        let mut right_rx = self.bus.subscribe(RIGHT_PAD).await;

        // Subscribe to every paired-device zone — keep a per-slot
        // `Zone` cache in memory for the keys-in-vehicle gate.
        let mut zone_streams: Vec<futures::stream::BoxStream<'static, SignalValue>> =
            Vec::with_capacity(PAIRED_ZONE_SIGNALS.len());
        for &sig in PAIRED_ZONE_SIGNALS.iter() {
            zone_streams.push(self.bus.subscribe(sig).await);
        }
        let mut device_zones: Vec<Zone> = vec![Zone::OutOfRange; PAIRED_ZONE_SIGNALS.len()];

        // Track per-pad: when did the current press start (None = not pressed).
        let mut left_pressed_at: Option<Instant> = None;
        let mut right_pressed_at: Option<Instant> = None;

        tracing::info!("ThumbPadLock feature started");

        loop {
            // Compute the next debounce deadline (minimum over active pads).
            let left_remaining = left_pressed_at.map(|t| DEBOUNCE.saturating_sub(t.elapsed()));
            let right_remaining = right_pressed_at.map(|t| DEBOUNCE.saturating_sub(t.elapsed()));

            let debounce_sleep = [left_remaining, right_remaining]
                .into_iter()
                .flatten()
                .min()
                .unwrap_or(Duration::from_secs(3600));

            // Manually merge zone streams into the select! since we
            // can't use a Vec<_> directly inside the macro.
            let zone_event = futures::future::select_all(
                zone_streams
                    .iter_mut()
                    .enumerate()
                    .map(|(i, s)| Box::pin(async move { (i, s.next().await) })),
            );

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
                ((slot, opt), _, _) = zone_event => {
                    // Update in-memory zone cache for the gate check below.
                    if let Some(SignalValue::String(s)) = opt {
                        if let Some(z) = Zone::from_str_value(&s) {
                            device_zones[slot] = z;
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
                        // PEPS-presence gate (REQ-PL-002): require at least
                        // one paired device in a zone OUTSIDE the cabin.
                        let device_outside =
                            device_zones.iter().copied().any(is_outside_cabin);
                        if !device_outside {
                            tracing::warn!(
                                zones = ?device_zones,
                                "ThumbPadLock: debounce complete but NO paired device outside cabin — lock denied (keys-in-vehicle guard)"
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
                            "ThumbPadLock: debounce complete — locking"
                        );

                        if let Err(e) = self
                            .arbiter
                            .request(DoorLockRequest {
                                command: LockCommand::LockAll,
                                feature_id: FeatureId::ThumbPadLock,
                            })
                            .await
                        {
                            tracing::error!(error = %e, "ThumbPadLock: arbiter error");
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

        tracing::info!("ThumbPadLock feature stopped");
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::door_lock_arbiter;
    use tokio::time::advance;

    /// Default test setup: bus, arbiter, ThumbPadLock running, and
    /// **fob 1 placed in `Approach`** so the keys-in-vehicle gate
    /// passes for the happy-path tests.  Tests that need to verify
    /// the gate denial path use `setup_no_paired_device_outside`.
    async fn setup() -> (Arc<MockBus>, tokio::task::JoinHandle<()>) {
        let (bus, h) = setup_no_paired_device_outside().await;
        // Place fob 1 in Approach (canonical "user is walking up to
        // the car holding the fob" zone).  The feature reads the
        // initial cached value via subscribe-replay during its first
        // poll.
        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.Zone",
            SignalValue::String("Approach".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;
        (bus, h)
    }

    /// Variant of `setup` that does NOT place any paired device in a
    /// zone outside the cabin.  Used for tests that exercise the
    /// keys-in-vehicle denial path.
    async fn setup_no_paired_device_outside() -> (Arc<MockBus>, tokio::task::JoinHandle<()>) {
        let bus = Arc::new(MockBus::new());
        let (arb, _ack_tx, loop_fut) = door_lock_arbiter(Arc::clone(&bus));
        tokio::spawn(loop_fut);
        let arb = Arc::new(arb);
        let feature = ThumbPadLock::new(Arc::clone(&bus), arb);
        let handle = tokio::spawn(feature.run());
        tokio::task::yield_now().await;
        (bus, handle)
    }

    #[tokio::test(start_paused = true)]
    async fn left_pad_held_500ms_locks() {
        let (bus, _h) = setup().await;

        bus.inject(LEFT_PAD, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        bus.clear_history();

        advance(Duration::from_millis(500)).await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("lock_all".into())),
            "expected lock_all after 500ms debounce, history: {:?}",
            h
        );
    }

    #[tokio::test(start_paused = true)]
    async fn right_pad_held_500ms_locks() {
        let (bus, _h) = setup().await;

        bus.inject(RIGHT_PAD, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        bus.clear_history();

        advance(Duration::from_millis(500)).await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("lock_all".into())),
            "expected lock_all from right pad, history: {:?}",
            h
        );
    }

    #[tokio::test(start_paused = true)]
    async fn release_before_debounce_cancels_lock() {
        let (bus, _h) = setup().await;

        bus.inject(LEFT_PAD, SignalValue::Bool(true));
        tokio::task::yield_now().await;

        // Release before 500 ms
        advance(Duration::from_millis(200)).await;
        tokio::task::yield_now().await;
        bus.inject(LEFT_PAD, SignalValue::Bool(false));
        tokio::task::yield_now().await;

        // Advance well past 500 ms — no lock should fire
        advance(Duration::from_millis(600)).await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, _)| *s == "Body.Doors.CentralLock.Command"),
            "release before debounce should cancel lock, history: {:?}",
            h
        );
    }

    #[tokio::test(start_paused = true)]
    async fn lock_feedback_published_with_lock() {
        let (bus, _h) = setup().await;

        bus.inject(LEFT_PAD, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        bus.clear_history();

        advance(Duration::from_millis(500)).await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == FEEDBACK_REQUEST && *v == SignalValue::String("lock".into())),
            "expected lock FeedbackRequest alongside lock_all, history: {:?}",
            h
        );
    }

    #[tokio::test(start_paused = true)]
    async fn does_not_refire_without_new_press() {
        let (bus, _h) = setup().await;

        bus.inject(LEFT_PAD, SignalValue::Bool(true));
        tokio::task::yield_now().await;

        advance(Duration::from_millis(500)).await;
        tokio::task::yield_now().await;

        let count_after_first = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == "Body.Doors.CentralLock.Command")
            .count();

        // Advance more — should not fire again without a new press
        advance(Duration::from_millis(1000)).await;
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
    #[tokio::test(start_paused = true)]
    async fn no_device_anywhere_denies_lock() {
        let (bus, _h) = setup_no_paired_device_outside().await;

        bus.inject(LEFT_PAD, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        bus.clear_history();

        advance(Duration::from_millis(500)).await;
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
    #[tokio::test(start_paused = true)]
    async fn fob_in_cabin_only_denies_lock() {
        let (bus, _h) = setup_no_paired_device_outside().await;
        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.Zone",
            SignalValue::String("Cabin".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.inject(LEFT_PAD, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        bus.clear_history();

        advance(Duration::from_millis(500)).await;
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
    #[tokio::test(start_paused = true)]
    async fn fob_split_cabin_and_approach_locks() {
        let (bus, _h) = setup_no_paired_device_outside().await;
        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.Zone",
            SignalValue::String("Cabin".into()),
        );
        bus.inject(
            "Body.PEPS.Plant.KeyFob.2.Zone",
            SignalValue::String("Approach".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.inject(LEFT_PAD, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        bus.clear_history();

        advance(Duration::from_millis(500)).await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("lock_all".into())),
            "expected lock_all when at least one paired device is outside (split case): {h:?}"
        );
    }

    /// Paired phone in `DriverDoor` (proximity zone) → lock fires.
    /// Phones go through the same gate as fobs.
    #[tokio::test(start_paused = true)]
    async fn phone_in_driver_door_zone_passes_gate() {
        let (bus, _h) = setup_no_paired_device_outside().await;
        bus.inject(
            "Body.PEPS.Plant.BlePhone.1.Zone",
            SignalValue::String("DriverDoor".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.inject(RIGHT_PAD, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        bus.clear_history();

        advance(Duration::from_millis(500)).await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("lock_all".into())),
            "expected lock_all with phone in DriverDoor zone: {h:?}"
        );
    }
}
