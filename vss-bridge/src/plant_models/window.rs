//! Window plant — models the per-window DC motor + glass position.
//!
//! Each of the 4 doors runs one task that subscribes to its commanded
//! motor direction (`Body.Doors.Row{1,2}.{Left,Right}.Window.MotorDirection`,
//! a String enum `UP` / `DOWN` / `STOPPED` published by the window
//! arbiter) and integrates the position
//! (`Body.Doors.Row{1,2}.{Left,Right}.Window.Position`) over time.
//!
//! Travel rate: **10 % per second**, so a full close or open takes
//! 10 s.  This matches the user-spec demo timing — slow enough to
//! see motion, fast enough to be usable.  The plant clamps at the
//! 0 % (fully open) and 100 % (fully closed) end-stops; any further
//! command at the stop simply holds.
//!
//! # Boot
//!
//! Publishes `Position = 0` (fully open) for each window so HMI
//! snapshots land a defined value.
//!
//! # Single writer
//!
//! Only writer of the four `Window.Position` signals.
//!
//! # Future extensions
//!
//! - End-stop reaction (auto-stop the motor when reaching 0 / 100
//!   instead of leaving the arbiter publishing UP/DOWN against a
//!   clamped position).  Needs a back-channel from plant to feature
//!   or arbiter — out of scope for this phase.
//! - Anti-pinch detection: simulate a finger/obstacle by injecting a
//!   "load spike" on a configurable window position; the future
//!   `WindowAntiPinch` feature consumes the load and pre-empts the
//!   motor.  Plant stays open-loop for now.

use std::sync::Arc;
use std::time::Duration;

use futures::StreamExt;
use tokio::select;
use tokio::time::interval;

use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

/// One row per window: (MotorDirection, Position).
const WINDOWS: [(VssPath, VssPath); 4] = [
    (
        "Body.Doors.Row1.Left.Window.MotorDirection",
        "Body.Doors.Row1.Left.Window.Position",
    ),
    (
        "Body.Doors.Row1.Right.Window.MotorDirection",
        "Body.Doors.Row1.Right.Window.Position",
    ),
    (
        "Body.Doors.Row2.Left.Window.MotorDirection",
        "Body.Doors.Row2.Left.Window.Position",
    ),
    (
        "Body.Doors.Row2.Right.Window.MotorDirection",
        "Body.Doors.Row2.Right.Window.Position",
    ),
];

/// Integration tick — 100 ms (10 Hz).  At 10 %/s that's a 1.0-pp step
/// per tick; rounded to u8 it gives a smooth visible ramp.
const TICK: Duration = Duration::from_millis(100);
/// Position step per tick at 10 %/s.
const STEP_PER_TICK: f32 = 1.0;

pub struct WindowPlant<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> WindowPlant<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    pub async fn run(self) {
        tracing::info!("WindowPlant started");

        // Spawn one task per window and join them all so the outer
        // future (held by the caller's JoinSet) only completes when
        // every per-window task exits.
        let mut handles = Vec::with_capacity(4);
        for (idx, (motor, pos)) in WINDOWS.iter().enumerate() {
            let bus = Arc::clone(&self.bus);
            handles.push(tokio::spawn(Self::run_window(idx, motor, pos, bus)));
        }
        for h in handles {
            let _ = h.await;
        }
    }

    async fn run_window(idx: usize, motor: VssPath, pos: VssPath, bus: Arc<B>) {
        let mut motor_rx = bus.subscribe(motor).await;

        // Boot the position at 0 (fully open) so the HMI shows a
        // defined value before any motor command lands.
        let _ = bus.publish(pos, SignalValue::Uint8(0)).await;

        let mut position: f32 = 0.0;
        let mut direction: i8 = 0; // -1 = DOWN, 0 = STOPPED, +1 = UP.
        // Default Burst behaviour — after a tokio time-advance the
        // queued missed ticks fire one per select! iteration so the
        // position integrates correctly under both real and paused
        // time.  Burn the immediate tick at construction time so the
        // loop only starts counting once a real motor command lands.
        let mut tick = interval(TICK);
        tick.tick().await;

        loop {
            select! {
                Some(val) = motor_rx.next() => {
                    let new_dir = match val {
                        SignalValue::String(ref s) => match s.as_str() {
                            "UP" => 1,
                            "DOWN" => -1,
                            "STOPPED" => 0,
                            other => {
                                tracing::warn!(window = idx, motor = ?other, "WindowPlant: ignoring unknown motor enum");
                                continue;
                            }
                        },
                        _ => continue,
                    };
                    if direction != new_dir {
                        direction = new_dir;
                        tracing::debug!(window = idx, direction, "WindowPlant: motor change");
                    }
                }
                _ = tick.tick() => {
                    if direction == 0 {
                        continue;
                    }
                    let next = (position + STEP_PER_TICK * direction as f32).clamp(0.0, 100.0);
                    if (next - position).abs() < f32::EPSILON {
                        continue; // at end-stop — no publish.
                    }
                    position = next;
                    let _ = bus
                        .publish(pos, SignalValue::Uint8(position.round() as u8))
                        .await;
                }
                else => break,
            }
        }
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use tokio::time::advance;

    /// Yield many times so the plant's select! loop drains all
    /// queued interval ticks (one per yield) after a paused-time
    /// advance.  200 yields covers ~200 ticks which is well above
    /// the longest 10 s (100-tick) run we exercise.
    async fn settle() {
        for _ in 0..200 {
            tokio::task::yield_now().await;
        }
    }

    async fn setup() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        tokio::spawn(WindowPlant::new(Arc::clone(&bus)).run());
        settle().await;
        bus
    }

    fn pos(bus: &MockBus, idx: usize) -> Option<u8> {
        let (_, p) = WINDOWS[idx];
        match bus.latest_value(p) {
            Some(SignalValue::Uint8(v)) => Some(v),
            _ => None,
        }
    }

    #[tokio::test(start_paused = true)]
    async fn boots_at_zero_for_all_windows() {
        let bus = setup().await;
        for i in 0..4 {
            assert_eq!(pos(&bus, i), Some(0), "window {i} must boot at 0");
        }
    }

    #[tokio::test(start_paused = true)]
    async fn up_command_ramps_position_at_10_percent_per_second() {
        let bus = setup().await;
        bus.inject(WINDOWS[0].0, SignalValue::String("UP".into()));
        settle().await;
        // After 5 seconds the window should be at ~50%.
        advance(Duration::from_secs(5)).await;
        settle().await;
        let p = pos(&bus, 0).unwrap();
        assert!((45..=55).contains(&p), "expected ~50%, got {p}");
    }

    #[tokio::test(start_paused = true)]
    async fn fully_closes_in_about_10_seconds() {
        let bus = setup().await;
        bus.inject(WINDOWS[0].0, SignalValue::String("UP".into()));
        settle().await;
        advance(Duration::from_millis(10_100)).await;
        settle().await;
        assert_eq!(pos(&bus, 0), Some(100));
    }

    #[tokio::test(start_paused = true)]
    async fn stopped_holds_position() {
        let bus = setup().await;
        bus.inject(WINDOWS[0].0, SignalValue::String("UP".into()));
        settle().await;
        advance(Duration::from_secs(3)).await;
        settle().await;
        let held = pos(&bus, 0).unwrap();
        bus.inject(WINDOWS[0].0, SignalValue::String("STOPPED".into()));
        settle().await;
        advance(Duration::from_secs(5)).await;
        settle().await;
        assert_eq!(pos(&bus, 0), Some(held), "stopped must hold position");
    }

    #[tokio::test(start_paused = true)]
    async fn down_reverses_then_clamps_at_zero() {
        let bus = setup().await;
        bus.inject(WINDOWS[0].0, SignalValue::String("UP".into()));
        settle().await;
        advance(Duration::from_secs(5)).await;
        settle().await;
        bus.inject(WINDOWS[0].0, SignalValue::String("DOWN".into()));
        settle().await;
        // Run more than enough to drive back to 0.
        advance(Duration::from_secs(20)).await;
        settle().await;
        assert_eq!(pos(&bus, 0), Some(0));
    }

    #[tokio::test(start_paused = true)]
    async fn windows_are_independent() {
        let bus = setup().await;
        bus.inject(WINDOWS[0].0, SignalValue::String("UP".into()));
        bus.inject(WINDOWS[2].0, SignalValue::String("UP".into()));
        settle().await;
        advance(Duration::from_secs(3)).await;
        settle().await;
        assert!(pos(&bus, 0).unwrap() > 0);
        assert!(pos(&bus, 2).unwrap() > 0);
        assert_eq!(pos(&bus, 1), Some(0));
        assert_eq!(pos(&bus, 3), Some(0));
    }
}
