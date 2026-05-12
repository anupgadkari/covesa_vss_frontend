//! Child-lock feature — latches per-rear-door child-lock state from
//! the driver master pushes.
//!
//! Inputs (momentary bools, driver-master panel):
//! - `Body.Switches.ChildLock.Row2.Left.IsPressed`
//! - `Body.Switches.ChildLock.Row2.Right.IsPressed`
//!
//! Outputs (latched bools — toggled on each rising edge of the
//! matching input):
//! - `Body.Doors.Row2.Left.IsChildLockActive`
//! - `Body.Doors.Row2.Right.IsChildLockActive`
//!
//! # Boot
//!
//! Publishes `false` on both outputs at start-up so HMI snapshots
//! land defined initial states.
//!
//! # Feedback gap
//!
//! In production the door reports the actual mechanical-latch state
//! back (e.g. `Body.Doors.Row2.*.IsChildLockFeedback`); the body
//! controller would then reconcile commanded vs. actual.  Those
//! feedback signals are intentionally out of scope here — for now
//! the feature is open-loop and the output IS the commanded state.
//!
//! # Single writer
//!
//! Only writer of the two `IsChildLockActive` outputs.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

const PRESS_L: VssPath = "Body.Switches.ChildLock.Row2.Left.IsPressed";
const PRESS_R: VssPath = "Body.Switches.ChildLock.Row2.Right.IsPressed";
const OUT_L: VssPath = "Body.Doors.Row2.Left.IsChildLockActive";
const OUT_R: VssPath = "Body.Doors.Row2.Right.IsChildLockActive";

pub struct ChildLock<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> ChildLock<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    pub async fn run(self) {
        tracing::info!("ChildLock feature started");

        let mut press_l_rx = self.bus.subscribe(PRESS_L).await;
        let mut press_r_rx = self.bus.subscribe(PRESS_R).await;

        // Deterministic boot — both rear doors child-lock off.
        let _ = self.bus.publish(OUT_L, SignalValue::Bool(false)).await;
        let _ = self.bus.publish(OUT_R, SignalValue::Bool(false)).await;

        let mut latched_l: bool = false;
        let mut latched_r: bool = false;
        let mut last_l: bool = false;
        let mut last_r: bool = false;

        loop {
            select! {
                Some(v) = press_l_rx.next() => {
                    let now = matches!(v, SignalValue::Bool(true));
                    if now && !last_l {
                        latched_l = !latched_l;
                        tracing::info!(active = latched_l, door = "Row2.Left", "ChildLock: toggled");
                        let _ = self.bus.publish(OUT_L, SignalValue::Bool(latched_l)).await;
                    }
                    last_l = now;
                }
                Some(v) = press_r_rx.next() => {
                    let now = matches!(v, SignalValue::Bool(true));
                    if now && !last_r {
                        latched_r = !latched_r;
                        tracing::info!(active = latched_r, door = "Row2.Right", "ChildLock: toggled");
                        let _ = self.bus.publish(OUT_R, SignalValue::Bool(latched_r)).await;
                    }
                    last_r = now;
                }
                else => break,
            }
        }

        tracing::warn!("ChildLock: press streams closed, exiting");
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;

    async fn settle() {
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        tokio::time::sleep(std::time::Duration::from_millis(2)).await;
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    async fn setup() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let f = ChildLock::new(Arc::clone(&bus));
        tokio::spawn(f.run());
        settle().await;
        bus
    }

    fn out(bus: &MockBus, sig: VssPath) -> Option<bool> {
        match bus.latest_value(sig) {
            Some(SignalValue::Bool(b)) => Some(b),
            _ => None,
        }
    }

    #[tokio::test]
    async fn boots_off_both_sides() {
        let bus = setup().await;
        assert_eq!(out(&bus, OUT_L), Some(false));
        assert_eq!(out(&bus, OUT_R), Some(false));
    }

    #[tokio::test]
    async fn left_press_toggles_left_only() {
        let bus = setup().await;
        bus.inject(PRESS_L, SignalValue::Bool(true));
        settle().await;
        assert_eq!(out(&bus, OUT_L), Some(true));
        assert_eq!(out(&bus, OUT_R), Some(false));
    }

    #[tokio::test]
    async fn right_press_toggles_right_only() {
        let bus = setup().await;
        bus.inject(PRESS_R, SignalValue::Bool(true));
        settle().await;
        assert_eq!(out(&bus, OUT_L), Some(false));
        assert_eq!(out(&bus, OUT_R), Some(true));
    }

    #[tokio::test]
    async fn double_press_toggles_back() {
        let bus = setup().await;
        bus.inject(PRESS_L, SignalValue::Bool(true));
        bus.inject(PRESS_L, SignalValue::Bool(false));
        bus.inject(PRESS_L, SignalValue::Bool(true));
        settle().await;
        assert_eq!(out(&bus, OUT_L), Some(false));
    }

    #[tokio::test]
    async fn held_press_does_not_re_toggle() {
        let bus = setup().await;
        bus.inject(PRESS_L, SignalValue::Bool(true));
        bus.inject(PRESS_L, SignalValue::Bool(true));
        bus.inject(PRESS_L, SignalValue::Bool(true));
        settle().await;
        assert_eq!(out(&bus, OUT_L), Some(true));
    }
}
