//! Mirror-adjust motor plant model.
//!
//! Simulates the per-side tilt + yaw motors that move the mirror
//! glass.  One plant per side; they integrate a directional command
//! into the corresponding axis position.
//!
//! ```text
//!  MirrorAdjust feature
//!      │  Body.Mirror.{Left,Right}.AdjustCmd  (string: NONE|UP|DOWN|LEFT|RIGHT)
//!      ▼
//!  MirrorAdjustPlantModel          ← this module
//!      │  publishes Body.Mirror.{Left,Right}.{Tilt,Yaw}  (i8, -100..100)
//!      ▼
//!  SignalBus → WsBridge → HMI
//! ```
//!
//! # Movement rate
//!
//! The motor sweeps at `RATE_PERCENT_PER_SEC` (50%/sec) — i.e. holding
//! a direction for 2 s sweeps from -100 to +100.  Real production
//! mirrors are slower (30-40%/sec); 50% is a deliberate test-friendly
//! default (cuts CI test wall-time).  Override via
//! [`MirrorAdjustPlantModel::with_rate`].
//!
//! # Sign convention (VSS v6.0)
//!
//! - `Tilt`: +100 = fully UP, -100 = fully DOWN, 0 = centre.
//! - `Yaw`:  +100 = clockwise around vehicle Z-axis (right-hand
//!   rule).  Per VSS v6.0 (which deprecated `Pan` with reversed
//!   direction): positive yaw means a *better view of objects on the
//!   right side of the vehicle*.  So:
//!   - `LEFT` direction → yaw decreases (counter-clockwise, view
//!     swings left)
//!   - `RIGHT` direction → yaw increases (clockwise, view swings
//!     right)
//!
//! # No NVM
//!
//! Tilt/Yaw positions are not persisted across boots in this plant
//! model — that's the responsibility of a future "Mirror Memory"
//! feature (per-driver presets) which would own its own NVM file.
//! Cold boot = both axes at 0.

use std::sync::Arc;
use std::time::Duration;

use futures::stream::BoxStream;
use futures::StreamExt;
use tokio::select;
use tokio::time::{sleep, Instant};

use crate::ipc_message::SignalValue;
use crate::signal_bus::SignalBus;

const ADJUST_CMD_LEFT: &str = "Body.Mirror.Left.AdjustCmd";
const ADJUST_CMD_RIGHT: &str = "Body.Mirror.Right.AdjustCmd";
const TILT_LEFT: &str = "Body.Mirror.Left.Tilt";
const YAW_LEFT: &str = "Body.Mirror.Left.Yaw";
const TILT_RIGHT: &str = "Body.Mirror.Right.Tilt";
const YAW_RIGHT: &str = "Body.Mirror.Right.Yaw";

/// Default movement rate — full -100..+100 sweep in 4 s.
pub const RATE_PERCENT_PER_SEC: f32 = 50.0;

/// How often the integrator publishes intermediate positions while a
/// direction is held.  50 ms is fast enough that the HMI reads as
/// smooth motion (~20 fps) and slow enough to not spam the bus.
const TICK: Duration = Duration::from_millis(50);

/// One of the five command strings.  Anything else maps to `Idle`.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Direction {
    Idle,
    Up,
    Down,
    Left,
    Right,
}

impl Direction {
    fn from_str_value(s: &str) -> Self {
        match s {
            "UP" => Self::Up,
            "DOWN" => Self::Down,
            "LEFT" => Self::Left,
            "RIGHT" => Self::Right,
            // 'NONE' or anything else → idle.
            _ => Self::Idle,
        }
    }
}

/// Per-side state.
#[derive(Debug, Clone, Copy)]
struct SideState {
    /// Current tilt position, integrated as f32 internally (-100..100).
    tilt: f32,
    /// Current yaw position (-100..100).
    yaw: f32,
    /// Currently active direction; `Idle` means the motor is not
    /// moving and the next event will only come from the cmd stream.
    direction: Direction,
    /// Most recent values published — used to suppress duplicate Int16
    /// publishes on bus when the integrated f32 didn't cross a
    /// quantisation boundary.
    last_published_tilt: i8,
    last_published_yaw: i8,
}

impl SideState {
    fn new() -> Self {
        Self {
            tilt: 0.0,
            yaw: 0.0,
            direction: Direction::Idle,
            last_published_tilt: 0,
            last_published_yaw: 0,
        }
    }
}

pub struct MirrorAdjustPlantModel<B: SignalBus> {
    bus: Arc<B>,
    state: [SideState; 2], // [left, right]
    rate: f32,             // percent per second
    tick: Duration,
}

impl<B: SignalBus + Send + Sync + 'static> MirrorAdjustPlantModel<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self {
            bus,
            state: [SideState::new(), SideState::new()],
            rate: RATE_PERCENT_PER_SEC,
            tick: TICK,
        }
    }

    /// Override the integration rate (tests; real motors are slower).
    pub fn with_rate(mut self, percent_per_sec: f32) -> Self {
        self.rate = percent_per_sec;
        self
    }

    /// Override the tick interval (tests; faster ticks shorten the
    /// virtual-time advance needed to observe motion).
    pub fn with_tick(mut self, tick: Duration) -> Self {
        self.tick = tick;
        self
    }

    /// Publish initial positions (both axes = 0) so subscribers see a
    /// concrete value at boot rather than waiting for the first move.
    async fn publish_initial(&self) {
        for &(sig, val) in &[
            (TILT_LEFT, 0i16),
            (YAW_LEFT, 0i16),
            (TILT_RIGHT, 0i16),
            (YAW_RIGHT, 0i16),
        ] {
            let _ = self.bus.publish(sig, SignalValue::Int16(val)).await;
        }
    }

    pub async fn run(mut self) {
        tracing::info!(rate = self.rate, "MirrorAdjustPlantModel started");

        let mut cmd_streams: [BoxStream<'static, SignalValue>; 2] = [
            self.bus.subscribe(ADJUST_CMD_LEFT).await,
            self.bus.subscribe(ADJUST_CMD_RIGHT).await,
        ];

        self.publish_initial().await;

        let mut last_tick: Option<Instant> = None;

        loop {
            // Sleep duration: if any side is moving, tick at `self.tick`;
            // otherwise sleep effectively forever (only command events
            // wake us).
            let any_moving = self
                .state
                .iter()
                .any(|s| !matches!(s.direction, Direction::Idle));
            let sleep_for = if any_moving {
                self.tick
            } else {
                Duration::from_secs(3600)
            };

            let cmd_event = futures::future::select_all(
                cmd_streams
                    .iter_mut()
                    .enumerate()
                    .map(|(i, s)| Box::pin(async move { (i, s.next().await) })),
            );

            select! {
                ((side, opt), _, _) = cmd_event => {
                    let dir = match opt {
                        Some(SignalValue::String(s)) => Direction::from_str_value(&s),
                        _ => continue,
                    };
                    let prev = self.state[side].direction;
                    self.state[side].direction = dir;
                    if prev != dir {
                        tracing::info!(side, ?dir, "MirrorAdjust plant: direction changed");
                        // Reset the tick clock so the first integration
                        // step after a direction change measures from
                        // *now*, not from the previous tick.
                        last_tick = Some(Instant::now());
                    }
                }
                _ = sleep(sleep_for) => {
                    let now = Instant::now();
                    let dt = match last_tick {
                        Some(prev) => now.saturating_duration_since(prev).as_secs_f32(),
                        None => self.tick.as_secs_f32(),
                    };
                    last_tick = Some(now);
                    self.integrate_and_publish(dt).await;
                }
            }
        }
    }

    /// Advance both sides by `dt` seconds at the configured rate.
    async fn integrate_and_publish(&mut self, dt: f32) {
        let step = self.rate * dt;
        for side in 0..2 {
            let dir = self.state[side].direction;
            if dir == Direction::Idle {
                continue;
            }
            // Apply the step to the matching axis; clamp ±100.
            match dir {
                Direction::Up => self.state[side].tilt = (self.state[side].tilt + step).min(100.0),
                Direction::Down => {
                    self.state[side].tilt = (self.state[side].tilt - step).max(-100.0)
                }
                // VSS v6.0 yaw convention: positive = clockwise around
                // Z-axis = "view of right side of vehicle".  So LEFT
                // direction decreases yaw and RIGHT increases it.
                Direction::Left => self.state[side].yaw = (self.state[side].yaw - step).max(-100.0),
                Direction::Right => self.state[side].yaw = (self.state[side].yaw + step).min(100.0),
                Direction::Idle => unreachable!(),
            }

            // Quantise + suppress duplicate publishes.
            let q_tilt = self.state[side].tilt.round() as i8;
            let q_yaw = self.state[side].yaw.round() as i8;
            let (tilt_sig, yaw_sig) = if side == 0 {
                (TILT_LEFT, YAW_LEFT)
            } else {
                (TILT_RIGHT, YAW_RIGHT)
            };
            if q_tilt != self.state[side].last_published_tilt {
                let _ = self
                    .bus
                    .publish(tilt_sig, SignalValue::Int16(q_tilt as i16))
                    .await;
                self.state[side].last_published_tilt = q_tilt;
            }
            if q_yaw != self.state[side].last_published_yaw {
                let _ = self
                    .bus
                    .publish(yaw_sig, SignalValue::Int16(q_yaw as i16))
                    .await;
                self.state[side].last_published_yaw = q_yaw;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use tokio::time::advance;

    async fn settle() {
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    fn latest_i8(bus: &MockBus, sig: &'static str) -> Option<i8> {
        match bus.latest_value(sig)? {
            SignalValue::Int16(v) => Some(v as i8),
            _ => None,
        }
    }

    #[tokio::test(start_paused = true)]
    async fn idle_publishes_initial_zero() {
        let bus = Arc::new(MockBus::new());
        let plant = MirrorAdjustPlantModel::new(Arc::clone(&bus));
        tokio::spawn(plant.run());
        settle().await;
        assert_eq!(latest_i8(&bus, TILT_LEFT), Some(0));
        assert_eq!(latest_i8(&bus, YAW_LEFT), Some(0));
        assert_eq!(latest_i8(&bus, TILT_RIGHT), Some(0));
        assert_eq!(latest_i8(&bus, YAW_RIGHT), Some(0));
    }

    #[tokio::test(start_paused = true)]
    async fn up_increments_tilt_at_rate() {
        let bus = Arc::new(MockBus::new());
        // 100%/sec for fast tests — full sweep in 1 s.
        let plant = MirrorAdjustPlantModel::new(Arc::clone(&bus))
            .with_rate(100.0)
            .with_tick(Duration::from_millis(50));
        tokio::spawn(plant.run());
        settle().await;

        bus.inject(ADJUST_CMD_LEFT, SignalValue::String("UP".into()));
        settle().await;

        // After ~500 ms at 100%/s, tilt should be ~50.
        advance(Duration::from_millis(520)).await;
        settle().await;
        let v = latest_i8(&bus, TILT_LEFT).unwrap();
        assert!((45..=60).contains(&v), "expected ~50, got {v}");
    }

    #[tokio::test(start_paused = true)]
    async fn left_decreases_yaw() {
        // VSS v6.0 sign convention: LEFT direction → yaw decreases.
        let bus = Arc::new(MockBus::new());
        let plant = MirrorAdjustPlantModel::new(Arc::clone(&bus))
            .with_rate(100.0)
            .with_tick(Duration::from_millis(50));
        tokio::spawn(plant.run());
        settle().await;

        bus.inject(ADJUST_CMD_LEFT, SignalValue::String("LEFT".into()));
        settle().await;
        advance(Duration::from_millis(520)).await;
        settle().await;
        let v = latest_i8(&bus, YAW_LEFT).unwrap();
        assert!((-60..=-45).contains(&v), "expected ~-50, got {v}");
    }

    #[tokio::test(start_paused = true)]
    async fn right_increases_yaw() {
        let bus = Arc::new(MockBus::new());
        let plant = MirrorAdjustPlantModel::new(Arc::clone(&bus))
            .with_rate(100.0)
            .with_tick(Duration::from_millis(50));
        tokio::spawn(plant.run());
        settle().await;

        bus.inject(ADJUST_CMD_LEFT, SignalValue::String("RIGHT".into()));
        settle().await;
        advance(Duration::from_millis(520)).await;
        settle().await;
        let v = latest_i8(&bus, YAW_LEFT).unwrap();
        assert!((45..=60).contains(&v), "expected ~+50, got {v}");
    }

    #[tokio::test(start_paused = true)]
    async fn motion_clamps_at_100() {
        let bus = Arc::new(MockBus::new());
        let plant = MirrorAdjustPlantModel::new(Arc::clone(&bus))
            .with_rate(200.0)
            .with_tick(Duration::from_millis(20));
        tokio::spawn(plant.run());
        settle().await;

        bus.inject(ADJUST_CMD_LEFT, SignalValue::String("UP".into()));
        settle().await;
        // 2 s at 200%/s would be 400 — clamped to 100.
        advance(Duration::from_millis(2_000)).await;
        settle().await;
        assert_eq!(latest_i8(&bus, TILT_LEFT), Some(100));
    }

    #[tokio::test(start_paused = true)]
    async fn none_stops_motion() {
        let bus = Arc::new(MockBus::new());
        let plant = MirrorAdjustPlantModel::new(Arc::clone(&bus))
            .with_rate(100.0)
            .with_tick(Duration::from_millis(50));
        tokio::spawn(plant.run());
        settle().await;

        bus.inject(ADJUST_CMD_LEFT, SignalValue::String("UP".into()));
        settle().await;
        advance(Duration::from_millis(520)).await;
        settle().await;
        let mid = latest_i8(&bus, TILT_LEFT).unwrap();
        assert!(mid > 0);

        // Stop.
        bus.inject(ADJUST_CMD_LEFT, SignalValue::String("NONE".into()));
        settle().await;

        advance(Duration::from_millis(2_000)).await;
        settle().await;
        // Should not have moved further.
        let later = latest_i8(&bus, TILT_LEFT).unwrap();
        assert_eq!(mid, later, "motion must stop when direction = NONE");
    }

    #[tokio::test(start_paused = true)]
    async fn sides_independent() {
        let bus = Arc::new(MockBus::new());
        let plant = MirrorAdjustPlantModel::new(Arc::clone(&bus))
            .with_rate(100.0)
            .with_tick(Duration::from_millis(50));
        tokio::spawn(plant.run());
        settle().await;

        bus.inject(ADJUST_CMD_LEFT, SignalValue::String("UP".into()));
        bus.inject(ADJUST_CMD_RIGHT, SignalValue::String("DOWN".into()));
        settle().await;
        advance(Duration::from_millis(520)).await;
        settle().await;

        let l = latest_i8(&bus, TILT_LEFT).unwrap();
        let r = latest_i8(&bus, TILT_RIGHT).unwrap();
        assert!(l > 30, "left tilted UP, got {l}");
        assert!(r < -30, "right tilted DOWN, got {r}");
    }
}
