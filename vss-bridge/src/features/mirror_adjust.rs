//! MirrorAdjust — routes the driver's joystick input to the selected
//! mirror's adjust motors.
//!
//! # Inputs
//!
//! - **`Body.Switches.Mirror.Select`** — string enum: `'NONE','LEFT','RIGHT'`.
//!   Picks which mirror responds to subsequent Direction commands.
//!   `NONE` means no mirror is being adjusted.
//! - **`Body.Switches.Mirror.Direction`** — string enum:
//!   `'NONE','UP','DOWN','LEFT','RIGHT'`.  Stateful — held non-NONE
//!   while the user is pressing the joystick.
//!
//! # Outputs
//!
//! - **`Body.Mirror.{Left,Right}.AdjustCmd`** — string enum forwarded
//!   to [`crate::plant_models::mirror_adjust::MirrorAdjustPlantModel`].
//!
//! # Routing rules
//!
//! - When `Select == NONE`, both per-side `AdjustCmd` outputs stay at
//!   `NONE` regardless of `Direction`.
//! - When `Select` flips between LEFT and RIGHT mid-press, the
//!   previously-active side is released to `NONE` and the new side
//!   takes over the current `Direction`.
//! - When `Direction == NONE` (joystick released), both sides go to
//!   `NONE`.

use std::sync::Arc;

use futures::stream::BoxStream;
use futures::StreamExt;
use tokio::select;

use crate::ipc_message::{FeatureId, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

#[allow(dead_code)]
const FEATURE_ID: FeatureId = FeatureId::MirrorAdjust;

const SWITCH_SELECT: VssPath = "Body.Switches.Mirror.Select";
const SWITCH_DIRECTION: VssPath = "Body.Switches.Mirror.Direction";
const CMD_LEFT: VssPath = "Body.Mirror.Left.AdjustCmd";
const CMD_RIGHT: VssPath = "Body.Mirror.Right.AdjustCmd";

#[derive(Debug, Clone, Copy, PartialEq)]
enum Side {
    None,
    Left,
    Right,
}

impl Side {
    fn from_str_value(s: &str) -> Self {
        match s {
            "LEFT" => Self::Left,
            "RIGHT" => Self::Right,
            _ => Self::None,
        }
    }
}

pub struct MirrorAdjust<B: SignalBus> {
    bus: Arc<B>,
    select: Side,
    direction: String, // verbatim NONE/UP/DOWN/LEFT/RIGHT
}

impl<B: SignalBus + Send + Sync + 'static> MirrorAdjust<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self {
            bus,
            select: Side::None,
            direction: "NONE".into(),
        }
    }

    /// Current per-side AdjustCmd values implied by `select` + `direction`.
    fn resolve(&self) -> (&'static str, &'static str) {
        let dir: &'static str = match self.direction.as_str() {
            "UP" => "UP",
            "DOWN" => "DOWN",
            "LEFT" => "LEFT",
            "RIGHT" => "RIGHT",
            _ => "NONE",
        };
        match self.select {
            Side::Left => (dir, "NONE"),
            Side::Right => ("NONE", dir),
            // Select=NONE means feature is inert regardless of Direction.
            Side::None => ("NONE", "NONE"),
        }
    }

    /// Re-publish AdjustCmd for both sides.  Idempotent — the plant
    /// model deduplicates on its end if the value didn't change.
    async fn publish(&self) {
        let (left, right) = self.resolve();
        let _ = self
            .bus
            .publish(CMD_LEFT, SignalValue::String(left.into()))
            .await;
        let _ = self
            .bus
            .publish(CMD_RIGHT, SignalValue::String(right.into()))
            .await;
    }

    pub async fn run(mut self) {
        tracing::info!("MirrorAdjust feature started");

        let mut select_rx: BoxStream<'static, SignalValue> =
            self.bus.subscribe(SWITCH_SELECT).await;
        let mut dir_rx: BoxStream<'static, SignalValue> =
            self.bus.subscribe(SWITCH_DIRECTION).await;

        // Initial state: both AdjustCmds at NONE.  Publish so the plant
        // model can start its idle loop with concrete values.
        self.publish().await;

        loop {
            select! {
                Some(v) = select_rx.next() => {
                    if let SignalValue::String(s) = v {
                        let new_select = Side::from_str_value(&s);
                        if new_select != self.select {
                            tracing::info!(
                                old = ?self.select,
                                new = ?new_select,
                                direction = %self.direction,
                                "MirrorAdjust: Select changed"
                            );
                            self.select = new_select;
                            self.publish().await;
                        }
                    }
                }
                Some(v) = dir_rx.next() => {
                    if let SignalValue::String(s) = v {
                        if s != self.direction {
                            tracing::info!(
                                old = %self.direction,
                                new = %s,
                                select = ?self.select,
                                "MirrorAdjust: Direction changed"
                            );
                            self.direction = s;
                            self.publish().await;
                        }
                    }
                }
                else => break,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;

    async fn settle() {
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    fn latest_str(bus: &MockBus, sig: &'static str) -> Option<String> {
        match bus.latest_value(sig)? {
            SignalValue::String(s) => Some(s),
            _ => None,
        }
    }

    async fn setup() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let feat = MirrorAdjust::new(Arc::clone(&bus));
        tokio::spawn(feat.run());
        settle().await;
        bus
    }

    #[tokio::test]
    async fn select_none_blocks_direction() {
        let bus = setup().await;
        bus.inject(SWITCH_DIRECTION, SignalValue::String("UP".into()));
        settle().await;
        // Both per-side cmds should still be NONE.
        assert_eq!(latest_str(&bus, CMD_LEFT), Some("NONE".into()));
        assert_eq!(latest_str(&bus, CMD_RIGHT), Some("NONE".into()));
    }

    #[tokio::test]
    async fn select_left_routes_direction_to_left_only() {
        let bus = setup().await;
        bus.inject(SWITCH_SELECT, SignalValue::String("LEFT".into()));
        bus.inject(SWITCH_DIRECTION, SignalValue::String("UP".into()));
        settle().await;
        assert_eq!(latest_str(&bus, CMD_LEFT), Some("UP".into()));
        assert_eq!(latest_str(&bus, CMD_RIGHT), Some("NONE".into()));
    }

    #[tokio::test]
    async fn select_right_routes_direction_to_right_only() {
        let bus = setup().await;
        bus.inject(SWITCH_SELECT, SignalValue::String("RIGHT".into()));
        bus.inject(SWITCH_DIRECTION, SignalValue::String("RIGHT".into()));
        settle().await;
        assert_eq!(latest_str(&bus, CMD_LEFT), Some("NONE".into()));
        assert_eq!(latest_str(&bus, CMD_RIGHT), Some("RIGHT".into()));
    }

    #[tokio::test]
    async fn flipping_select_mid_press_swaps_active_side() {
        let bus = setup().await;
        bus.inject(SWITCH_SELECT, SignalValue::String("LEFT".into()));
        bus.inject(SWITCH_DIRECTION, SignalValue::String("UP".into()));
        settle().await;
        assert_eq!(latest_str(&bus, CMD_LEFT), Some("UP".into()));

        bus.inject(SWITCH_SELECT, SignalValue::String("RIGHT".into()));
        settle().await;
        assert_eq!(latest_str(&bus, CMD_LEFT), Some("NONE".into()));
        assert_eq!(latest_str(&bus, CMD_RIGHT), Some("UP".into()));
    }

    #[tokio::test]
    async fn direction_none_releases_both_sides() {
        let bus = setup().await;
        bus.inject(SWITCH_SELECT, SignalValue::String("LEFT".into()));
        bus.inject(SWITCH_DIRECTION, SignalValue::String("DOWN".into()));
        settle().await;
        assert_eq!(latest_str(&bus, CMD_LEFT), Some("DOWN".into()));

        bus.inject(SWITCH_DIRECTION, SignalValue::String("NONE".into()));
        settle().await;
        assert_eq!(latest_str(&bus, CMD_LEFT), Some("NONE".into()));
        assert_eq!(latest_str(&bus, CMD_RIGHT), Some("NONE".into()));
    }
}
