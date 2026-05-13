//! Brake plant model — derives `Chassis.Brake.IsApplied` (bool) from
//! the raw `Chassis.Brake.PedalPosition` (Uint8, 0–100 %).
//!
//! In production this signal would come from one of several places —
//! the ABS / ESC ECU on the chassis CAN bus, the powertrain control
//! module (especially on regen-brake powertrains), or a discrete brake
//! light switch hard-wired to the body controller.  Regardless of the
//! source the body controller subscribes to a debounced "pedal pressed"
//! boolean rather than the raw analog channel.  This plant model
//! simulates that derivation on the dev host so feature business logic
//! (VehicleStartingControl, future stop-start, etc.) can consume the
//! clean boolean without each feature re-implementing thresholding.
//!
//! # Hysteresis
//!
//! Threshold-debounced with two trip points so a pedal hovering near
//! the press threshold can't chatter the output:
//!
//!   * Press   when PedalPosition ≥ 5 %
//!   * Release when PedalPosition ≤ 2 %
//!   * Between 3..4 %: hold the current state
//!
//! Both thresholds are small enough that a deliberate brake press
//! always trips IsApplied true; large enough that sensor noise around
//! zero (typical for unpressed analog pedals) doesn't.
//!
//! # Boot behaviour
//!
//! Publishes `false` at startup so late subscribers (HMI snapshot,
//! VehicleStartingControl, etc.) always see a deterministic value
//! rather than `None`.  No-op on redundant edges.
//!
//! # Single writer
//!
//! Single writer of `Chassis.Brake.IsApplied`.

use std::sync::Arc;

use futures::StreamExt;

use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

const PEDAL_IN: VssPath = "Chassis.Brake.PedalPosition";
const APPLIED_OUT: VssPath = "Chassis.Brake.IsApplied";

const PRESS_THRESHOLD_PCT: u8 = 5;
const RELEASE_THRESHOLD_PCT: u8 = 2;

pub struct BrakePlant<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> BrakePlant<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    pub async fn run(self) {
        tracing::info!("BrakePlant started");

        let mut pedal_rx = self.bus.subscribe(PEDAL_IN).await;

        // Deterministic boot value.
        let _ = self
            .bus
            .publish(APPLIED_OUT, SignalValue::Bool(false))
            .await;

        let mut applied = false;

        while let Some(val) = pedal_rx.next().await {
            let pct = pedal_pct(&val);
            let next = next_state(applied, pct);
            if next == applied {
                continue; // hysteresis hold or redundant edge
            }
            applied = next;
            tracing::info!(applied, pct, "BrakePlant: edge");
            if let Err(e) = self
                .bus
                .publish(APPLIED_OUT, SignalValue::Bool(applied))
                .await
            {
                tracing::error!(error = %e, "BrakePlant: publish failed");
            }
        }

        tracing::warn!("BrakePlant: pedal stream closed, exiting");
    }
}

/// Coerce the incoming pedal-position value into a percentage 0..=100.
/// Accepts the canonical Uint8 form and tolerates Int16 / floats in
/// case a sloppy producer hands us a slightly different numeric type.
/// Anything non-numeric reads as 0 % (released).
fn pedal_pct(val: &SignalValue) -> u8 {
    match val {
        SignalValue::Uint8(p) => *p,
        SignalValue::Int16(p) => (*p).clamp(0, 100) as u8,
        SignalValue::Float(f) => f.clamp(0.0, 100.0) as u8,
        _ => 0,
    }
}

/// Hysteretic state transition.  See module docs for the trip points.
fn next_state(current: bool, pct: u8) -> bool {
    if current {
        // Currently applied — only release if we fall to/below the release threshold.
        pct > RELEASE_THRESHOLD_PCT
    } else {
        // Currently released — only press if we reach/exceed the press threshold.
        pct >= PRESS_THRESHOLD_PCT
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
    }

    async fn setup() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let plant = BrakePlant::new(Arc::clone(&bus));
        tokio::spawn(plant.run());
        settle().await;
        bus
    }

    #[tokio::test]
    async fn boots_to_released() {
        let bus = setup().await;
        assert_eq!(
            bus.latest_value(APPLIED_OUT),
            Some(SignalValue::Bool(false)),
            "plant must publish a deterministic released state at boot"
        );
    }

    #[tokio::test]
    async fn press_above_threshold_trips_applied() {
        let bus = setup().await;
        bus.inject(PEDAL_IN, SignalValue::Uint8(40));
        settle().await;
        assert_eq!(bus.latest_value(APPLIED_OUT), Some(SignalValue::Bool(true)));
    }

    #[tokio::test]
    async fn press_at_exact_press_threshold_trips_applied() {
        let bus = setup().await;
        bus.inject(PEDAL_IN, SignalValue::Uint8(PRESS_THRESHOLD_PCT));
        settle().await;
        assert_eq!(bus.latest_value(APPLIED_OUT), Some(SignalValue::Bool(true)));
    }

    #[tokio::test]
    async fn light_brush_below_press_threshold_stays_released() {
        let bus = setup().await;
        // 4 % is below the 5 % press threshold — hysteresis must hold.
        bus.inject(PEDAL_IN, SignalValue::Uint8(4));
        settle().await;
        assert_eq!(
            bus.latest_value(APPLIED_OUT),
            Some(SignalValue::Bool(false))
        );
    }

    #[tokio::test]
    async fn release_below_threshold_clears_applied() {
        let bus = setup().await;
        bus.inject(PEDAL_IN, SignalValue::Uint8(60));
        settle().await;
        bus.inject(PEDAL_IN, SignalValue::Uint8(0));
        settle().await;
        assert_eq!(
            bus.latest_value(APPLIED_OUT),
            Some(SignalValue::Bool(false))
        );
    }

    #[tokio::test]
    async fn hysteresis_band_holds_applied_state() {
        // Press, then drift down into 3..4 % band — must stay applied.
        let bus = setup().await;
        bus.inject(PEDAL_IN, SignalValue::Uint8(40));
        settle().await;
        bus.inject(PEDAL_IN, SignalValue::Uint8(3));
        settle().await;
        assert_eq!(
            bus.latest_value(APPLIED_OUT),
            Some(SignalValue::Bool(true)),
            "3 % is in the hysteresis band — must hold applied"
        );
        bus.inject(PEDAL_IN, SignalValue::Uint8(4));
        settle().await;
        assert_eq!(
            bus.latest_value(APPLIED_OUT),
            Some(SignalValue::Bool(true)),
            "4 % is in the hysteresis band — must still hold applied"
        );
    }

    #[tokio::test]
    async fn redundant_edges_do_not_republish() {
        let bus = setup().await;
        bus.inject(PEDAL_IN, SignalValue::Uint8(50));
        settle().await;
        bus.clear_history();

        bus.inject(PEDAL_IN, SignalValue::Uint8(60));
        bus.inject(PEDAL_IN, SignalValue::Uint8(70));
        settle().await;

        let republishes = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == APPLIED_OUT)
            .count();
        assert_eq!(
            republishes, 0,
            "already-applied additional presses must not republish"
        );
    }

    #[tokio::test]
    async fn round_trip_pulse_train() {
        let bus = setup().await;
        // Boot publishes one `false`; clear so we measure edges only.
        bus.clear_history();
        for pct in [40u8, 0, 50, 0, 60].iter() {
            bus.inject(PEDAL_IN, SignalValue::Uint8(*pct));
            settle().await;
        }
        let states: Vec<bool> = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == APPLIED_OUT)
            .filter_map(|(_, v)| match v {
                SignalValue::Bool(b) => Some(*b),
                _ => None,
            })
            .collect();
        assert_eq!(states, vec![true, false, true, false, true]);
    }

    #[tokio::test]
    async fn accepts_int16_pedal_value() {
        let bus = setup().await;
        bus.inject(PEDAL_IN, SignalValue::Int16(30));
        settle().await;
        assert_eq!(bus.latest_value(APPLIED_OUT), Some(SignalValue::Bool(true)));
    }

    #[test]
    fn next_state_press_threshold_exact() {
        assert!(next_state(false, PRESS_THRESHOLD_PCT));
        assert!(!next_state(false, PRESS_THRESHOLD_PCT - 1));
    }

    #[test]
    fn next_state_release_threshold_exact() {
        assert!(!next_state(true, RELEASE_THRESHOLD_PCT));
        assert!(next_state(true, RELEASE_THRESHOLD_PCT + 1));
    }
}
