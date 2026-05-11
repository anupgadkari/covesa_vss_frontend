//! Day / Night mode plant model — models the body controller's HMI
//! display-mode decision.
//!
//! In production, a real vehicle decides "DAY vs NIGHT" for the
//! instrument-cluster backlight and HMI render style by fusing several
//! inputs:
//!
//!   * Low-beam headlamps on (today: the only input we model)
//!   * Ambient light sensor (`Body.Lights.AmbientLightSensor.Illuminance`)
//!     dropping below a dusk threshold
//!   * GPS time + location-based sunset/sunrise table
//!   * Forward camera / rain sensor detecting a tunnel or heavy overcast
//!
//! This plant model captures that decision and publishes the result on
//! the standard COVESA VSS v4.0 signal
//! `Vehicle.Cabin.Infotainment.HMI.DayNightMode` (String enum:
//! `"DAY"` or `"NIGHT"`).  Today it watches only the low-beam state and
//! mirrors it — beams ON ⇒ NIGHT, beams OFF ⇒ DAY.  The plant model is
//! the right place to extend with the sensor-fusion inputs above as
//! they come online; the HMI just consumes the resolved enum.
//!
//! # Boot behaviour
//!
//! Publishes `"DAY"` on startup so late subscribers (HMI snapshot)
//! always see a deterministic value rather than `None`.
//!
//! # Single writer
//!
//! Single writer of `Vehicle.Cabin.Infotainment.HMI.DayNightMode`.  If
//! a second producer ever wants to override the mode (e.g. driver
//! manually forcing NIGHT via the touchscreen for theatre mode), the
//! design should add a higher-priority arbiter rather than letting two
//! plant models race on the bus.

use std::sync::Arc;

use futures::StreamExt;

use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

const LOW_BEAM: VssPath = "Body.Lights.Beam.Low.IsOn";
const DAY_NIGHT_MODE: VssPath = "Vehicle.Cabin.Infotainment.HMI.DayNightMode";

const DAY: &str = "DAY";
const NIGHT: &str = "NIGHT";

pub struct DayNightModePlant<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> DayNightModePlant<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    pub async fn run(self) {
        tracing::info!("DayNightModePlant started");

        let mut low_beam_rx = self.bus.subscribe(LOW_BEAM).await;

        // Publish the initial "DAY" so the HMI snapshot lands a
        // deterministic value before any low-beam edge arrives.
        let _ = self
            .bus
            .publish(DAY_NIGHT_MODE, SignalValue::String(DAY.into()))
            .await;

        let mut current = DAY;

        while let Some(val) = low_beam_rx.next().await {
            let want = if matches!(val, SignalValue::Bool(true)) {
                NIGHT
            } else {
                DAY
            };
            if want == current {
                continue; // Idempotent — no edge.
            }
            current = want;
            tracing::info!(mode = current, "DayNightModePlant: mode change");
            if let Err(e) = self
                .bus
                .publish(DAY_NIGHT_MODE, SignalValue::String(current.into()))
                .await
            {
                tracing::error!(error = %e, "DayNightModePlant: publish failed");
            }
        }

        tracing::warn!("DayNightModePlant: low-beam stream closed, exiting");
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
        let plant = DayNightModePlant::new(Arc::clone(&bus));
        tokio::spawn(plant.run());
        settle().await;
        bus
    }

    #[tokio::test]
    async fn boots_to_day() {
        let bus = setup().await;
        assert_eq!(
            bus.latest_value(DAY_NIGHT_MODE),
            Some(SignalValue::String("DAY".into())),
            "plant must publish a deterministic DAY at boot"
        );
    }

    #[tokio::test]
    async fn low_beam_on_switches_to_night() {
        let bus = setup().await;
        bus.inject(LOW_BEAM, SignalValue::Bool(true));
        settle().await;
        assert_eq!(
            bus.latest_value(DAY_NIGHT_MODE),
            Some(SignalValue::String("NIGHT".into()))
        );
    }

    #[tokio::test]
    async fn low_beam_off_returns_to_day() {
        let bus = setup().await;
        bus.inject(LOW_BEAM, SignalValue::Bool(true));
        settle().await;
        bus.inject(LOW_BEAM, SignalValue::Bool(false));
        settle().await;
        assert_eq!(
            bus.latest_value(DAY_NIGHT_MODE),
            Some(SignalValue::String("DAY".into()))
        );
    }

    #[tokio::test]
    async fn redundant_edges_are_idempotent() {
        let bus = setup().await;
        bus.inject(LOW_BEAM, SignalValue::Bool(true));
        settle().await;
        bus.clear_history();

        // Same value twice — must not republish.
        bus.inject(LOW_BEAM, SignalValue::Bool(true));
        bus.inject(LOW_BEAM, SignalValue::Bool(true));
        settle().await;

        let republishes = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == DAY_NIGHT_MODE)
            .count();
        assert_eq!(republishes, 0, "redundant edges must not republish");
    }

    #[tokio::test]
    async fn round_trip_pulse_train() {
        // Simulates a light show / blink — multiple toggles should
        // produce matching DAY/NIGHT publishes.
        let bus = setup().await;
        for v in [true, false, true, false].iter() {
            bus.inject(LOW_BEAM, SignalValue::Bool(*v));
            settle().await;
        }
        let modes: Vec<String> = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == DAY_NIGHT_MODE)
            .filter_map(|(_, v)| match v {
                SignalValue::String(s) => Some(s.clone()),
                _ => None,
            })
            .collect();
        // Initial boot-DAY + 4 edge transitions.
        assert_eq!(modes, vec!["DAY", "NIGHT", "DAY", "NIGHT", "DAY"]);
    }
}
