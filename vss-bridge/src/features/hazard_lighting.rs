//! Hazard Lighting — activates both direction indicators when the
//! driver engages the physical hazard switch.
//!
//! Subscribes to:
//!   - Body.Switches.Hazard.IsEngaged (overlay sensor — physical switch)
//!
//! Outputs (via Lighting arbiter):
//!   - Body.Lights.DirectionIndicator.Left.IsSignaling  @ HIGH (3)
//!   - Body.Lights.DirectionIndicator.Right.IsSignaling @ HIGH (3)
//!
//! This feature does NOT control blink timing. The 1–2 Hz UN R48-
//! compliant cadence is the responsibility of the LED driver IC or
//! body ECU firmware. This feature only publishes the IsSignaling
//! boolean intent flag.
//!
//! Unlike TurnIndicator, hazard lights operate regardless of ignition
//! state (OFF, ACC, ON, START). This is a safety requirement — hazards
//! must always be available.

use std::sync::Arc;

use futures::StreamExt;

use crate::arbiter::{ActuatorRequest, DomainArbiter};
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

/// Physical hazard switch input (overlay signal).
const HAZARD_SWITCH: VssPath = "Body.Switches.Hazard.IsEngaged";

/// Actuator outputs — both direction indicators.
const LEFT_INDICATOR: VssPath = "Body.Lights.DirectionIndicator.Left.IsSignaling";
const RIGHT_INDICATOR: VssPath = "Body.Lights.DirectionIndicator.Right.IsSignaling";

pub struct HazardLighting<B: SignalBus> {
    arbiter: Arc<DomainArbiter>,
    bus: Arc<B>,
}

impl<B: SignalBus> HazardLighting<B> {
    pub fn new(arbiter: Arc<DomainArbiter>, bus: Arc<B>) -> Self {
        Self { arbiter, bus }
    }

    /// Main run loop. Call via `tokio::spawn(hazard.run())`.
    pub async fn run(self) {
        tracing::info!("HazardLighting feature started");

        let mut switch_stream = self.bus.subscribe(HAZARD_SWITCH).await;

        while let Some(val) = switch_stream.next().await {
            let engaged = val == SignalValue::Bool(true);

            tracing::debug!(engaged, "hazard switch changed");

            // When engaged, claim both indicators at HIGH priority.
            // When disengaged, release the claims so lower-priority
            // claims (e.g. an active turn signal) can resume.
            for &signal in &[LEFT_INDICATOR, RIGHT_INDICATOR] {
                let result = if engaged {
                    self.arbiter
                        .request(ActuatorRequest {
                            signal,
                            value: SignalValue::Bool(true),
                            priority: Priority::High,
                            feature_id: FeatureId::Hazard,
                        })
                        .await
                } else {
                    self.arbiter.release(signal, FeatureId::Hazard).await
                };

                if let Err(e) = result {
                    tracing::error!(error = %e, "HazardLighting: arbiter op failed");
                }
            }
        }

        tracing::warn!("HazardLighting: switch stream closed, exiting");
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::lighting_arbiter;
    use std::time::Duration;
    use tokio::time::sleep;

    async fn setup() -> (Arc<MockBus>, Arc<DomainArbiter>, tokio::task::JoinHandle<()>) {
        let bus = Arc::new(MockBus::new());
        let (arbiter, arbiter_fut) = lighting_arbiter(Arc::clone(&bus));
        tokio::spawn(arbiter_fut);
        let arbiter = Arc::new(arbiter);

        let feature = HazardLighting::new(Arc::clone(&arbiter), Arc::clone(&bus));
        let handle = tokio::spawn(feature.run());
        tokio::task::yield_now().await;

        (bus, arbiter, handle)
    }

    #[tokio::test]
    async fn hazard_engaged_activates_both_indicators() {
        let (bus, _arb, _handle) = setup().await;

        bus.inject(HAZARD_SWITCH, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        let history = bus.history();
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == LEFT_INDICATOR && *val == SignalValue::Bool(true)
            }),
            "left indicator should be TRUE, history: {:?}",
            history
        );
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == RIGHT_INDICATOR && *val == SignalValue::Bool(true)
            }),
            "right indicator should be TRUE, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn hazard_disengaged_deactivates_both_indicators() {
        let (bus, _arb, _handle) = setup().await;

        // Engage then disengage
        bus.inject(HAZARD_SWITCH, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        bus.clear_history();
        bus.inject(HAZARD_SWITCH, SignalValue::Bool(false));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        let history = bus.history();
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == LEFT_INDICATOR && *val == SignalValue::Bool(false)
            }),
            "left indicator should be FALSE after disengage, history: {:?}",
            history
        );
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == RIGHT_INDICATOR && *val == SignalValue::Bool(false)
            }),
            "right indicator should be FALSE after disengage, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn hazard_publishes_once_per_transition() {
        let (bus, _arb, _handle) = setup().await;

        bus.inject(HAZARD_SWITCH, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        let history = bus.history();
        // Should be exactly one TRUE for each indicator (no periodic toggles)
        let left_true_count = history
            .iter()
            .filter(|(sig, val)| *sig == LEFT_INDICATOR && *val == SignalValue::Bool(true))
            .count();
        let right_true_count = history
            .iter()
            .filter(|(sig, val)| *sig == RIGHT_INDICATOR && *val == SignalValue::Bool(true))
            .count();
        assert_eq!(left_true_count, 1, "should publish exactly once per indicator");
        assert_eq!(right_true_count, 1, "should publish exactly once per indicator");
    }

    #[tokio::test]
    async fn feature_stays_alive() {
        let (bus, _arb, handle) = setup().await;

        bus.inject(HAZARD_SWITCH, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        bus.inject(HAZARD_SWITCH, SignalValue::Bool(false));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        assert!(!handle.is_finished(), "feature should still be running after toggle");
    }

    // -----------------------------------------------------------------------
    // Ignition-independent operation (REQ-HAZ-007)
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn hazard_works_with_ignition_off() {
        let (bus, _arb, _handle) = setup().await;

        // Set ignition to OFF — hazard should still work
        bus.inject(
            "Vehicle.LowVoltageSystemState",
            SignalValue::String("OFF".to_string()),
        );
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(20)).await;

        bus.clear_history();
        bus.inject(HAZARD_SWITCH, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        let history = bus.history();
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == LEFT_INDICATOR && *val == SignalValue::Bool(true)
            }),
            "hazard should work with ignition OFF, history: {:?}",
            history
        );
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == RIGHT_INDICATOR && *val == SignalValue::Bool(true)
            }),
            "hazard should work with ignition OFF, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn hazard_survives_ignition_state_change() {
        let (bus, _arb, _handle) = setup().await;

        // Engage hazard
        bus.inject(HAZARD_SWITCH, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        // Change ignition from ON to OFF
        bus.inject(
            "Vehicle.LowVoltageSystemState",
            SignalValue::String("OFF".to_string()),
        );
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        // Hazard should still be engaged — no deactivation published.
        // The last arbiter state for both indicators is still TRUE.
        let history = bus.history();
        let left_events: Vec<_> = history
            .iter()
            .filter(|(sig, _)| *sig == LEFT_INDICATOR)
            .collect();
        // Only the initial TRUE, no FALSE from ignition change
        assert!(
            left_events.last().map(|(_, v)| v) == Some(&SignalValue::Bool(true)),
            "hazard should remain active through ignition change, history: {:?}",
            left_events
        );
    }
}
