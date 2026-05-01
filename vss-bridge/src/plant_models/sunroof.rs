//! Sunroof plant model — power-actuated glass panel + retractable shade.
//!
//! Two independent motors share the same model:
//!
//! ```text
//!  HMI sunroof open/close hold buttons
//!      │  Body.Sunroof.MoveCmd          ("OPEN" | "CLOSE" | "STOP")
//!      │  Body.Sunroof.Shade.MoveCmd    (same enum)
//!      ▼
//!  SunroofPlantModel       ← this module
//!      │  integrates Position at TRAVEL_RATE_PCT_PER_SEC,
//!      │  clamps to [0, 100], publishes:
//!      │    Body.Sunroof.Position
//!      │    Body.Sunroof.Shade.Position
//!      ▼
//!  SignalBus → WsBridge → HMI
//! ```
//!
//! # Motor model
//!
//! `TRAVEL_RATE_PCT_PER_SEC = 20` → 5 s full travel (0 → 100), matches
//! typical OEM sunroof timing.  When `MoveCmd = OPEN` the position
//! integrates upward at this rate; `CLOSE` integrates downward; `STOP`
//! halts at the current position.  Reaching 0 or 100 latches that end
//! and stops further drive (so a held-open press doesn't burn motor
//! current at the limit).
//!
//! # NVM persistence
//!
//! Final settled positions are persisted whenever the motor stops
//! (either at a limit or on STOP).  Boot republishes the persisted
//! values so a fresh HMI client sees the actual state without flash.
//!
//! # Why a string enum, not separate booleans?
//!
//! A single `MoveCmd` string makes the press-and-hold UX trivial: the
//! HMI sends `OPEN` on mousedown of the open button and `STOP` on
//! mouseup.  Two booleans (`IsOpening`, `IsClosing`) would need
//! mutual exclusion logic on the consumer; one string says exactly
//! what the user wants.

use std::sync::Arc;
use std::time::Duration;

use futures::StreamExt;
use tokio::select;
use tokio::time::interval;

use crate::ipc_message::SignalValue;
use crate::nvm::{NvmStore, SunroofState};
use crate::signal_bus::SignalBus;

const ROOF_CMD: &str = "Body.Sunroof.MoveCmd";
const SHADE_CMD: &str = "Body.Sunroof.Shade.MoveCmd";
const ROOF_POS: &str = "Body.Sunroof.Position";
const SHADE_POS: &str = "Body.Sunroof.Shade.Position";

/// Motor speed.  20 % / s = 5 seconds full travel, typical for an
/// OEM moonroof.
const TRAVEL_RATE_PCT_PER_SEC: f32 = 20.0;

/// Position update tick — small enough for smooth HMI animation,
/// large enough not to flood the bus.  At 20 ms × 20 %/s = 0.4 % /
/// tick → 250 ticks across the full 5 s travel.
const TICK_MS: u64 = 20;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MoveDir {
    Open,
    Close,
    Stop,
}

impl MoveDir {
    fn parse(val: &SignalValue) -> Option<Self> {
        match val {
            SignalValue::String(s) => match s.as_str() {
                "OPEN" => Some(Self::Open),
                "CLOSE" => Some(Self::Close),
                "STOP" => Some(Self::Stop),
                _ => None,
            },
            _ => None,
        }
    }
}

/// Per-surface motor state.
struct Motor {
    /// 0..=100 percent open.  Stored as f32 for smooth sub-percent
    /// integration; published as u8.
    pos: f32,
    dir: MoveDir,
}

impl Motor {
    fn new(initial_pct: u8) -> Self {
        Self {
            pos: initial_pct as f32,
            dir: MoveDir::Stop,
        }
    }

    /// Advance the motor by `dt_secs` seconds; returns true if `pos`
    /// (rounded to u8) changed since the last call.  Latches to a
    /// stop direction when the limit is reached so subsequent ticks
    /// don't burn cycles.
    fn tick(&mut self, dt_secs: f32, last_published: u8) -> bool {
        match self.dir {
            MoveDir::Open => {
                self.pos += TRAVEL_RATE_PCT_PER_SEC * dt_secs;
                if self.pos >= 100.0 {
                    self.pos = 100.0;
                    self.dir = MoveDir::Stop;
                }
            }
            MoveDir::Close => {
                self.pos -= TRAVEL_RATE_PCT_PER_SEC * dt_secs;
                if self.pos <= 0.0 {
                    self.pos = 0.0;
                    self.dir = MoveDir::Stop;
                }
            }
            MoveDir::Stop => return false,
        }
        self.pos.round() as u8 != last_published
    }

    fn is_settled(&self) -> bool {
        self.dir == MoveDir::Stop
    }

    fn pct(&self) -> u8 {
        self.pos.round() as u8
    }
}

pub struct SunroofPlantModel<B: SignalBus> {
    bus: Arc<B>,
    roof: Motor,
    shade: Motor,
    nvm: Option<NvmStore>,
}

impl<B: SignalBus + Send + Sync + 'static> SunroofPlantModel<B> {
    /// Create in volatile mode — no NVM, boots fully closed.
    pub fn new(bus: Arc<B>) -> Self {
        Self {
            bus,
            roof: Motor::new(0),
            shade: Motor::new(0),
            nvm: None,
        }
    }

    /// Production constructor — boots from NVM and persists settled
    /// positions on every motor stop.
    pub fn with_nvm(bus: Arc<B>, nvm: NvmStore) -> Self {
        let persisted = nvm.load_sunroof();
        tracing::info!(
            position = persisted.position,
            shade_position = persisted.shade_position,
            "SunroofPlantModel: booted from NVM"
        );
        Self {
            bus,
            roof: Motor::new(persisted.position.min(100)),
            shade: Motor::new(persisted.shade_position.min(100)),
            nvm: None,
        }
        .with_nvm_set(nvm)
    }

    fn with_nvm_set(mut self, nvm: NvmStore) -> Self {
        self.nvm = Some(nvm);
        self
    }

    fn save_to_nvm(&self) {
        if let Some(nvm) = &self.nvm {
            nvm.save_sunroof(&SunroofState {
                position: self.roof.pct(),
                shade_position: self.shade.pct(),
            });
        }
    }

    pub async fn run(mut self) {
        let mut roof_rx = self.bus.subscribe(ROOF_CMD).await;
        let mut shade_rx = self.bus.subscribe(SHADE_CMD).await;

        // Publish boot positions.
        let mut last_roof_pct = self.roof.pct();
        let mut last_shade_pct = self.shade.pct();
        let _ = self
            .bus
            .publish(ROOF_POS, SignalValue::Uint8(last_roof_pct))
            .await;
        let _ = self
            .bus
            .publish(SHADE_POS, SignalValue::Uint8(last_shade_pct))
            .await;
        tracing::info!(
            position = last_roof_pct,
            shade_position = last_shade_pct,
            "SunroofPlantModel started"
        );

        let dt_secs = TICK_MS as f32 / 1000.0;
        let mut ticker = interval(Duration::from_millis(TICK_MS));
        // First tick fires immediately; skip it so we don't double-step
        // a freshly-arrived MoveCmd.
        ticker.tick().await;

        loop {
            select! {
                Some(val) = roof_rx.next() => {
                    if let Some(dir) = MoveDir::parse(&val) {
                        self.roof.dir = dir;
                        if dir == MoveDir::Stop {
                            self.save_to_nvm();
                        }
                    }
                }
                Some(val) = shade_rx.next() => {
                    if let Some(dir) = MoveDir::parse(&val) {
                        self.shade.dir = dir;
                        if dir == MoveDir::Stop {
                            self.save_to_nvm();
                        }
                    }
                }
                _ = ticker.tick() => {
                    // Integrate both motors and publish on change.
                    let roof_was_moving = !self.roof.is_settled();
                    if self.roof.tick(dt_secs, last_roof_pct) {
                        last_roof_pct = self.roof.pct();
                        let _ = self.bus.publish(ROOF_POS, SignalValue::Uint8(last_roof_pct)).await;
                    }
                    // If the motor latched to Stop on this tick (hit a
                    // limit), persist the new resting position.
                    if roof_was_moving && self.roof.is_settled() {
                        self.save_to_nvm();
                    }

                    let shade_was_moving = !self.shade.is_settled();
                    if self.shade.tick(dt_secs, last_shade_pct) {
                        last_shade_pct = self.shade.pct();
                        let _ = self.bus.publish(SHADE_POS, SignalValue::Uint8(last_shade_pct)).await;
                    }
                    if shade_was_moving && self.shade.is_settled() {
                        self.save_to_nvm();
                    }
                }
            }
        }
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use tempfile::TempDir;
    use tokio::time::sleep;

    async fn settle() {
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    #[tokio::test]
    async fn open_then_stop_settles_partway() {
        let bus = Arc::new(MockBus::new());
        let plant = SunroofPlantModel::new(Arc::clone(&bus));
        tokio::spawn(plant.run());
        settle().await;

        // Hold OPEN for ~1 second → ~20 % position.
        bus.inject(ROOF_CMD, SignalValue::String("OPEN".into()));
        sleep(Duration::from_millis(1000)).await;
        bus.inject(ROOF_CMD, SignalValue::String("STOP".into()));
        settle().await;

        let pos = match bus.latest_value(ROOF_POS) {
            Some(SignalValue::Uint8(p)) => p,
            _ => panic!("no position published"),
        };
        // Allow ±5 % tolerance for scheduler jitter.
        assert!(
            (15..=25).contains(&pos),
            "expected ~20 % after 1 s open, got {pos}"
        );
    }

    #[tokio::test]
    async fn full_open_latches_at_100() {
        let bus = Arc::new(MockBus::new());
        let plant = SunroofPlantModel::new(Arc::clone(&bus));
        tokio::spawn(plant.run());
        settle().await;

        // 6 s should be more than enough for a 5 s travel.
        bus.inject(ROOF_CMD, SignalValue::String("OPEN".into()));
        sleep(Duration::from_millis(6000)).await;
        settle().await;

        assert_eq!(bus.latest_value(ROOF_POS), Some(SignalValue::Uint8(100)));
    }

    #[tokio::test]
    async fn close_from_open_returns_to_zero() {
        let bus = Arc::new(MockBus::new());
        let plant = SunroofPlantModel::new(Arc::clone(&bus));
        tokio::spawn(plant.run());
        settle().await;

        bus.inject(ROOF_CMD, SignalValue::String("OPEN".into()));
        sleep(Duration::from_millis(6000)).await;
        bus.inject(ROOF_CMD, SignalValue::String("CLOSE".into()));
        sleep(Duration::from_millis(6000)).await;

        assert_eq!(bus.latest_value(ROOF_POS), Some(SignalValue::Uint8(0)));
    }

    #[tokio::test]
    async fn mid_flight_reverse_tracks_correctly() {
        // Open partway, then immediately close — position should
        // decrease back toward 0 from wherever it was.
        let bus = Arc::new(MockBus::new());
        let plant = SunroofPlantModel::new(Arc::clone(&bus));
        tokio::spawn(plant.run());
        settle().await;

        bus.inject(ROOF_CMD, SignalValue::String("OPEN".into()));
        sleep(Duration::from_millis(1500)).await; // ~30 %
        bus.inject(ROOF_CMD, SignalValue::String("CLOSE".into()));
        sleep(Duration::from_millis(1500)).await; // back toward 0
        settle().await;

        let pos = match bus.latest_value(ROOF_POS) {
            Some(SignalValue::Uint8(p)) => p,
            _ => panic!("no position published"),
        };
        // Started at 0, opened to ~30, closed for the same duration —
        // should be at or near 0.  Allow a small overshoot tolerance.
        assert!(pos <= 5, "expected ~0 after symmetric reverse, got {pos}");
    }

    #[tokio::test]
    async fn shade_motor_independent_of_roof() {
        let bus = Arc::new(MockBus::new());
        let plant = SunroofPlantModel::new(Arc::clone(&bus));
        tokio::spawn(plant.run());
        settle().await;

        // Drive shade only.
        bus.inject(SHADE_CMD, SignalValue::String("OPEN".into()));
        sleep(Duration::from_millis(6000)).await;
        bus.inject(SHADE_CMD, SignalValue::String("STOP".into()));
        settle().await;

        assert_eq!(bus.latest_value(SHADE_POS), Some(SignalValue::Uint8(100)));
        assert_eq!(
            bus.latest_value(ROOF_POS),
            Some(SignalValue::Uint8(0)),
            "roof must not move when only shade is commanded"
        );
    }

    #[tokio::test]
    async fn nvm_round_trip() {
        let dir = TempDir::new().unwrap();
        let nvm = NvmStore::with_path(dir.path());
        let bus = Arc::new(MockBus::new());

        let plant = SunroofPlantModel::with_nvm(Arc::clone(&bus), nvm.clone());
        let h = tokio::spawn(plant.run());
        settle().await;

        bus.inject(ROOF_CMD, SignalValue::String("OPEN".into()));
        sleep(Duration::from_millis(6000)).await;
        // Latched at 100 → save_to_nvm fires inside the tick.
        settle().await;
        h.abort();

        // Fresh boot — same NVM, new bus.
        let bus2 = Arc::new(MockBus::new());
        let plant2 = SunroofPlantModel::with_nvm(Arc::clone(&bus2), nvm);
        tokio::spawn(plant2.run());
        settle().await;
        assert_eq!(bus2.latest_value(ROOF_POS), Some(SignalValue::Uint8(100)));
    }
}
