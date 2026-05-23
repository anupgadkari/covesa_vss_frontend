//! Start/Stop button LED plant — translates the ECU's published
//! PWM duty cycle into perceived LED intensity.
//!
//! # Why a plant model
//!
//! In production the body controller drives an actual PWM signal at
//! the published duty cycle and the LED's RC filtering + phosphor
//! response integrates the square wave into a perceived brightness
//! (essentially low-pass filtering with some gamma curve).  On the
//! simulator bus we publish the duty cycle directly, but we keep
//! the plant boundary in place so:
//!
//!   * Features see a clean intensity signal (`BacklightIntensity`)
//!     and don't have to know the duty-cycle waveform.
//!   * Tests / HMI can poll a single value instead of integrating
//!     ticks.
//!   * A future iteration can add gamma correction, ambient
//!     compensation, or a smoothing filter without touching VSC.
//!
//! Today the plant is a pure passthrough: `Intensity = DutyCycle`.
//!
//! # Single writer
//!
//! Sole writer of `Body.Switches.StartStop.BacklightIntensity`.

use std::sync::Arc;

use futures::StreamExt;

use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

const DUTY_IN: VssPath = "Body.Switches.StartStop.BacklightDutyCycle";
const INTENSITY_OUT: VssPath = "Body.Switches.StartStop.BacklightIntensity";

pub struct StartStopLedPlant<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> StartStopLedPlant<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    pub async fn run(self) {
        tracing::info!("StartStopLedPlant started");

        let mut duty_rx = self.bus.subscribe(DUTY_IN).await;

        // Boot value — emit 0 immediately so HMI snapshots before
        // VSC publishes its first tick still see a defined value.
        let _ = self.bus.publish(INTENSITY_OUT, SignalValue::Uint8(0)).await;

        let mut last_intensity: u8 = 0;

        while let Some(val) = duty_rx.next().await {
            let duty = match val {
                SignalValue::Uint8(v) => v,
                SignalValue::Uint16(v) => v.min(100) as u8,
                SignalValue::Int16(v) => v.clamp(0, 100) as u8,
                _ => continue,
            };
            // Passthrough today.  Future: gamma curve, RC filter,
            // ambient-light gain.
            let intensity = duty.min(100);
            if intensity == last_intensity {
                continue;
            }
            last_intensity = intensity;
            if let Err(e) = self
                .bus
                .publish(INTENSITY_OUT, SignalValue::Uint8(intensity))
                .await
            {
                tracing::error!(error = %e, "StartStopLedPlant: publish failed");
            }
        }

        tracing::warn!("StartStopLedPlant: duty-cycle stream closed, exiting");
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
        let plant = StartStopLedPlant::new(Arc::clone(&bus));
        tokio::spawn(plant.run());
        settle().await;
        bus
    }

    fn intensity(bus: &MockBus) -> Option<u8> {
        match bus.latest_value(INTENSITY_OUT) {
            Some(SignalValue::Uint8(v)) => Some(v),
            _ => None,
        }
    }

    #[tokio::test]
    async fn boots_to_zero() {
        let bus = setup().await;
        assert_eq!(intensity(&bus), Some(0));
    }

    #[tokio::test]
    async fn passthrough_tracks_duty() {
        let bus = setup().await;
        for d in [10u8, 50, 100, 0, 75] {
            bus.inject(DUTY_IN, SignalValue::Uint8(d));
            settle().await;
            assert_eq!(intensity(&bus), Some(d), "intensity must follow duty {d}");
        }
    }

    #[tokio::test]
    async fn clamps_overrange_inputs() {
        let bus = setup().await;
        bus.inject(DUTY_IN, SignalValue::Uint16(200));
        settle().await;
        assert_eq!(intensity(&bus), Some(100));
        bus.inject(DUTY_IN, SignalValue::Int16(-5));
        settle().await;
        assert_eq!(intensity(&bus), Some(0));
    }

    #[tokio::test]
    async fn redundant_duty_no_republish() {
        let bus = setup().await;
        bus.inject(DUTY_IN, SignalValue::Uint8(40));
        settle().await;
        bus.clear_history();
        bus.inject(DUTY_IN, SignalValue::Uint8(40));
        bus.inject(DUTY_IN, SignalValue::Uint8(40));
        settle().await;
        let republishes = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == INTENSITY_OUT)
            .count();
        assert_eq!(republishes, 0);
    }
}
