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

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;
use tokio::time::{sleep, Duration, Instant};

use crate::arbiter::{DoorLockArbiter, DoorLockRequest, LockCommand, FEEDBACK_REQUEST};
use crate::ipc_message::{FeatureId, SignalValue};
use crate::signal_bus::SignalBus;

const LEFT_PAD: &str = "Body.Doors.Row1.Left.Handle.Outside.LockPad.IsPressed";
const RIGHT_PAD: &str = "Body.Doors.Row1.Right.Handle.Outside.LockPad.IsPressed";

/// Debounce duration before the lock fires.
const DEBOUNCE: Duration = Duration::from_millis(500);

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

    async fn setup() -> (Arc<MockBus>, tokio::task::JoinHandle<()>) {
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
}
