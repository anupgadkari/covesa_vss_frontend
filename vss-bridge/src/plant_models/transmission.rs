//! Transmission plant — models the gearbox's response to the driver's
//! selector position.
//!
//! In production this is the transmission controller (TCU) on the
//! powertrain side: it watches the driver's PRND/S selector
//! (`Powertrain.Transmission.SelectedGear`) and, depending on
//! interlocks (brake pedal, vehicle speed) and shift logic, engages
//! the actual gear and reports it back as
//! `Powertrain.Transmission.CurrentGear`.
//!
//! This plant model stands in for the TCU on dev hosts.  Today it
//! simply mirrors `SelectedGear` → `CurrentGear` so the rest of the
//! bus (and the HMI) closes the loop deterministically.  Future
//! extensions are the natural home for:
//!
//!   * Brake-pedal interlock for selecting D from P.
//!   * Speed-based forward-gear progression for automatic mode
//!     (`D` resolving to 1, 2, 3, … as vehicle speed climbs).
//!   * Shift-time delay so the cluster reads `N` momentarily while
//!     the actual change settles.
//!
//! # Boot behaviour
//!
//! Publishes `CurrentGear = 126` (Park) so HMI snapshots land a
//! deterministic value before the driver touches the selector.
//!
//! # Single writer
//!
//! Only writer of `Powertrain.Transmission.CurrentGear`.  If anything
//! else ever wants to drive the actual gear (manual-mode override,
//! valet/limp-home pin), it should claim through an arbiter rather
//! than racing the plant.

use std::sync::Arc;

use futures::StreamExt;

use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

const SELECTED: VssPath = "Powertrain.Transmission.SelectedGear";
const CURRENT: VssPath = "Powertrain.Transmission.CurrentGear";

const PARK: i16 = 126;

pub struct TransmissionPlant<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> TransmissionPlant<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    pub async fn run(self) {
        tracing::info!("TransmissionPlant started");

        let mut sel_rx = self.bus.subscribe(SELECTED).await;

        // Deterministic boot — publish Park so late subscribers (HMI
        // snapshot) always see a defined gear.
        let _ = self
            .bus
            .publish(CURRENT, SignalValue::Int16(PARK))
            .await;

        let mut current: i16 = PARK;

        while let Some(val) = sel_rx.next().await {
            let want = match val {
                SignalValue::Int16(v) => v,
                _ => continue,
            };
            if want == current {
                continue; // idempotent — no shift edge
            }
            tracing::info!(
                from = current,
                to = want,
                "TransmissionPlant: shift"
            );
            current = want;
            if let Err(e) = self
                .bus
                .publish(CURRENT, SignalValue::Int16(current))
                .await
            {
                tracing::error!(error = %e, "TransmissionPlant: publish failed");
            }
        }

        tracing::warn!("TransmissionPlant: selector stream closed, exiting");
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
        let plant = TransmissionPlant::new(Arc::clone(&bus));
        tokio::spawn(plant.run());
        settle().await;
        bus
    }

    fn current(bus: &MockBus) -> Option<i16> {
        match bus.latest_value(CURRENT) {
            Some(SignalValue::Int16(v)) => Some(v),
            _ => None,
        }
    }

    #[tokio::test]
    async fn boots_to_park() {
        let bus = setup().await;
        assert_eq!(current(&bus), Some(126));
    }

    #[tokio::test]
    async fn mirrors_drive() {
        let bus = setup().await;
        bus.inject(SELECTED, SignalValue::Int16(127));
        settle().await;
        assert_eq!(current(&bus), Some(127));
    }

    #[tokio::test]
    async fn mirrors_reverse_and_neutral() {
        let bus = setup().await;
        bus.inject(SELECTED, SignalValue::Int16(-1));
        settle().await;
        assert_eq!(current(&bus), Some(-1));
        bus.inject(SELECTED, SignalValue::Int16(0));
        settle().await;
        assert_eq!(current(&bus), Some(0));
    }

    #[tokio::test]
    async fn redundant_selection_is_idempotent() {
        let bus = setup().await;
        bus.inject(SELECTED, SignalValue::Int16(127));
        settle().await;
        bus.clear_history();
        bus.inject(SELECTED, SignalValue::Int16(127));
        bus.inject(SELECTED, SignalValue::Int16(127));
        settle().await;
        let republishes = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == CURRENT)
            .count();
        assert_eq!(republishes, 0);
    }

    #[tokio::test]
    async fn manual_gears_pass_through() {
        let bus = setup().await;
        for g in [1, 2, 3, 4, 5, 6] {
            bus.inject(SELECTED, SignalValue::Int16(g));
            settle().await;
            assert_eq!(current(&bus), Some(g));
        }
    }
}
