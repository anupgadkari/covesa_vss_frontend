//! Turn Indicator — activates the correct direction indicator when
//! the driver moves the turn signal stalk.
//!
//! Subscribes to:
//!   - Body.Switches.TurnIndicator.Direction (overlay sensor — stalk)
//!     Values: "OFF", "LEFT", "RIGHT"
//!
//! Outputs (via Lighting arbiter):
//!   - Body.Lights.DirectionIndicator.Left.IsSignaling  @ MEDIUM (2)
//!   - Body.Lights.DirectionIndicator.Right.IsSignaling @ MEDIUM (2)
//!
//! Like HazardLighting, this feature does NOT control blink timing.
//! The stalk's self-cancelling behavior (via steering angle feedback)
//! is handled by the body ECU firmware — this feature just tracks the
//! stalk position signal.

use std::sync::Arc;

use futures::StreamExt;

use crate::arbiter::{ActuatorRequest, DomainArbiter};
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

/// Physical turn signal stalk input (overlay signal).
const TURN_STALK: VssPath = "Body.Switches.TurnIndicator.Direction";

/// Actuator outputs — direction indicators.
const LEFT_INDICATOR: VssPath = "Body.Lights.DirectionIndicator.Left.IsSignaling";
const RIGHT_INDICATOR: VssPath = "Body.Lights.DirectionIndicator.Right.IsSignaling";

pub struct TurnIndicator<B: SignalBus> {
    arbiter: Arc<DomainArbiter>,
    bus: Arc<B>,
}

impl<B: SignalBus> TurnIndicator<B> {
    pub fn new(arbiter: Arc<DomainArbiter>, bus: Arc<B>) -> Self {
        Self { arbiter, bus }
    }

    /// Main run loop. Call via `tokio::spawn(turn.run())`.
    pub async fn run(self) {
        tracing::info!("TurnIndicator feature started");

        let mut stalk_stream = self.bus.subscribe(TURN_STALK).await;

        while let Some(val) = stalk_stream.next().await {
            let (left, right) = match &val {
                SignalValue::String(s) => match s.as_str() {
                    "LEFT" => (true, false),
                    "RIGHT" => (false, true),
                    _ => (false, false), // "OFF" or any unknown value
                },
                _ => {
                    tracing::warn!(value = ?val, "TurnIndicator: unexpected non-string value");
                    continue;
                }
            };

            tracing::debug!(left, right, "turn stalk changed");

            // Request both indicators — one on, one off (or both off for OFF).
            for (signal, active) in [(LEFT_INDICATOR, left), (RIGHT_INDICATOR, right)] {
                if let Err(e) = self
                    .arbiter
                    .request(ActuatorRequest {
                        signal,
                        value: SignalValue::Bool(active),
                        priority: Priority::Medium,
                        feature_id: FeatureId::TurnIndicator,
                    })
                    .await
                {
                    tracing::error!(error = %e, "TurnIndicator: arbiter request failed");
                }
            }
        }

        tracing::warn!("TurnIndicator: stalk stream closed, exiting");
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

        let feature = TurnIndicator::new(Arc::clone(&arbiter), Arc::clone(&bus));
        let handle = tokio::spawn(feature.run());
        tokio::task::yield_now().await;

        (bus, arbiter, handle)
    }

    fn s(val: &str) -> SignalValue {
        SignalValue::String(val.to_string())
    }

    #[tokio::test]
    async fn stalk_left_activates_left_indicator() {
        let (bus, _arb, _handle) = setup().await;

        bus.inject(TURN_STALK, s("LEFT"));
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
                *sig == RIGHT_INDICATOR && *val == SignalValue::Bool(false)
            }),
            "right indicator should be FALSE, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn stalk_right_activates_right_indicator() {
        let (bus, _arb, _handle) = setup().await;

        bus.inject(TURN_STALK, s("RIGHT"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        let history = bus.history();
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == RIGHT_INDICATOR && *val == SignalValue::Bool(true)
            }),
            "right indicator should be TRUE, history: {:?}",
            history
        );
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == LEFT_INDICATOR && *val == SignalValue::Bool(false)
            }),
            "left indicator should be FALSE, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn stalk_off_deactivates_both() {
        let (bus, _arb, _handle) = setup().await;

        // Activate left, then go to OFF
        bus.inject(TURN_STALK, s("LEFT"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        bus.clear_history();
        bus.inject(TURN_STALK, s("OFF"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        let history = bus.history();
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == LEFT_INDICATOR && *val == SignalValue::Bool(false)
            }),
            "left should be FALSE after OFF, history: {:?}",
            history
        );
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == RIGHT_INDICATOR && *val == SignalValue::Bool(false)
            }),
            "right should be FALSE after OFF, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn left_to_right_switches_indicators() {
        let (bus, _arb, _handle) = setup().await;

        bus.inject(TURN_STALK, s("LEFT"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        bus.clear_history();
        bus.inject(TURN_STALK, s("RIGHT"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        let history = bus.history();
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == LEFT_INDICATOR && *val == SignalValue::Bool(false)
            }),
            "left should turn OFF when switching to RIGHT, history: {:?}",
            history
        );
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == RIGHT_INDICATOR && *val == SignalValue::Bool(true)
            }),
            "right should turn ON when switching to RIGHT, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn hazard_overrides_turn_signal() {
        let (bus, arb, _handle) = setup().await;

        // Turn LEFT is active
        bus.inject(TURN_STALK, s("LEFT"));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        // Hazard engages at HIGH — overrides the MEDIUM turn request
        use crate::features::hazard_lighting::HazardLighting;
        let hazard = HazardLighting::new(Arc::clone(&arb), Arc::clone(&bus));
        tokio::spawn(hazard.run());
        tokio::task::yield_now().await;

        bus.clear_history();
        bus.inject("Body.Switches.Hazard.IsEngaged", SignalValue::Bool(true));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        let history = bus.history();
        // Both indicators should be TRUE (hazard HIGH wins over turn MEDIUM)
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == RIGHT_INDICATOR && *val == SignalValue::Bool(true)
            }),
            "hazard should override: right should be TRUE, history: {:?}",
            history
        );
    }
}
