//! Auto High Beam (AHB) — ADAS-camera-driven high-beam suppression.
//!
//! # Purpose
//!
//! The driver may leave the high-beam stalk engaged indefinitely.
//! The forward camera continuously evaluates oncoming traffic.
//! When an oncoming vehicle is detected AHB forces the high beam OFF,
//! regardless of the manual stalk position, by submitting a HIGH-priority
//! `Bool(false)` claim to the LowBeam arbiter.
//!
//! # Arbitration
//!
//! | Feature | Signal | Value | Priority |
//! |---|---|---|---|
//! | ManualLighting (FeatureId::HighBeam) | Beam.High.IsOn | Bool(true) | Medium |
//! | AutoHighBeam (FeatureId::AutoHighBeam) | Beam.High.IsOn | Bool(false) | **High** |
//!
//! When both claims are active the arbiter resolves to the **highest priority** —
//! AHB's `High` beats ManualLighting's `Medium`, so the lamp is OFF.
//! When AHB releases (oncoming clears), ManualLighting's Medium claim resumes
//! naturally — the driver regains full high beam without touching the stalk.
//!
//! # Signal
//!
//! - **Input**: `Vehicle.ADAS.HighBeam.OncomingVehicleDetected` — `Bool(true)` when
//!   the ADAS camera reports a risk of glare; `Bool(false)` when path is clear.
//!
//! # Hysteresis
//!
//! The ADAS camera is expected to implement its own debounce / hysteresis.
//! AHB reacts to every rising/falling edge without additional filtering.

use std::sync::Arc;

use futures::StreamExt;

use crate::arbiter::{ActuatorRequest, DomainArbiter};
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::SignalBus;

// ── Signal constants ───────────────────────────────────────────────────────

const ADAS_ONCOMING: &str = "Vehicle.ADAS.HighBeam.OncomingVehicleDetected";
const HIGH_BEAM_OUT: &str = "Body.Lights.Beam.High.IsOn";

// ── Feature struct ─────────────────────────────────────────────────────────

pub struct AutoHighBeam<B: SignalBus> {
    arbiter: Arc<DomainArbiter>,
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> AutoHighBeam<B> {
    pub fn new(arbiter: Arc<DomainArbiter>, bus: Arc<B>) -> Self {
        Self { arbiter, bus }
    }

    pub async fn run(self) {
        let mut adas_rx = self.bus.subscribe(ADAS_ONCOMING).await;
        let mut suppressing = false;

        tracing::info!("AutoHighBeam feature started");

        while let Some(val) = adas_rx.next().await {
            let oncoming = matches!(val, SignalValue::Bool(true));

            if oncoming && !suppressing {
                // Oncoming vehicle detected — claim HIGH priority Bool(false)
                // to override whatever ManualLighting is doing.
                suppressing = true;
                tracing::info!("AutoHighBeam: oncoming vehicle detected — suppressing high beam");
                let result = self
                    .arbiter
                    .request(ActuatorRequest {
                        signal: HIGH_BEAM_OUT,
                        value: SignalValue::Bool(false),
                        priority: Priority::High,
                        feature_id: FeatureId::AutoHighBeam,
                    })
                    .await;
                if let Err(e) = result {
                    tracing::error!(error = %e, "AutoHighBeam: arbiter claim failed");
                }
            } else if !oncoming && suppressing {
                // Oncoming cleared — release the suppression claim.
                // ManualLighting's Medium claim resumes automatically.
                suppressing = false;
                tracing::info!("AutoHighBeam: path clear — releasing high beam suppression");
                let result = self
                    .arbiter
                    .release(HIGH_BEAM_OUT, FeatureId::AutoHighBeam)
                    .await;
                if let Err(e) = result {
                    tracing::error!(error = %e, "AutoHighBeam: arbiter release failed");
                }
            }
        }

        tracing::info!("AutoHighBeam feature stopped");
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::{low_beam_arbiter, ActuatorRequest};
    use crate::ipc_message::{FeatureId, Priority, SignalValue};
    use std::sync::Arc;
    use tokio::time::Duration;

    const LOW_BEAM: &str = "Body.Lights.Beam.Low.IsOn";

    /// Set up the LowBeam arbiter + AHB feature. Returns (bus, arb).
    /// Callers can submit their own medium-priority claims via `arb` to simulate
    /// ManualLighting, making it possible to observe the arbitration flip when
    /// AHB releases its High-priority suppression claim.
    async fn setup() -> (Arc<MockBus>, Arc<DomainArbiter>) {
        let bus = Arc::new(MockBus::new());
        let (arb, loop_fut) = low_beam_arbiter(Arc::clone(&bus));
        tokio::spawn(loop_fut);
        let arb = Arc::new(arb);
        let feature = AutoHighBeam::new(Arc::clone(&arb), Arc::clone(&bus));
        tokio::spawn(feature.run());
        tokio::task::yield_now().await;
        (bus, arb)
    }

    async fn drain() {
        tokio::task::yield_now().await;
        tokio::time::sleep(Duration::from_millis(5)).await;
        tokio::task::yield_now().await;
    }

    /// Submit a ManualLighting-style Medium-priority Bool(true) claim for high beam.
    async fn claim_manual_high(arb: &DomainArbiter) {
        arb.request(ActuatorRequest {
            signal: HIGH_BEAM_OUT,
            value: SignalValue::Bool(true),
            priority: Priority::Medium,
            feature_id: FeatureId::HighBeam,
        })
        .await
        .expect("manual high beam claim failed");
    }

    /// Release the ManualLighting-style Medium-priority claim for high beam.
    async fn release_manual_high(arb: &DomainArbiter) {
        arb.release(HIGH_BEAM_OUT, FeatureId::HighBeam)
            .await
            .expect("manual high beam release failed");
    }

    #[tokio::test]
    async fn suppresses_high_beam_when_oncoming_detected() {
        let (bus, arb) = setup().await;

        // Simulate ManualLighting enabling high beam at Medium priority.
        claim_manual_high(&arb).await;
        drain().await;
        bus.clear_history();

        // Oncoming vehicle — AHB must override with Bool(false) at High.
        bus.inject(ADAS_ONCOMING, SignalValue::Bool(true));
        drain().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == HIGH_BEAM_OUT && *v == SignalValue::Bool(false)),
            "oncoming detected: expected high beam suppressed to false, got: {h:?}",
        );
    }

    #[tokio::test]
    async fn releases_suppression_when_oncoming_clears() {
        let (bus, arb) = setup().await;

        // ManualLighting holding high beam on at Medium.
        claim_manual_high(&arb).await;
        drain().await;

        // Oncoming: AHB suppresses → Bool(false) published.
        bus.inject(ADAS_ONCOMING, SignalValue::Bool(true));
        drain().await;
        bus.clear_history();

        // Clear: AHB releases its High claim → ManualLighting's Medium Bool(true) wins.
        bus.inject(ADAS_ONCOMING, SignalValue::Bool(false));
        drain().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == HIGH_BEAM_OUT && *v == SignalValue::Bool(true)),
            "clear path: ManualLighting should regain high beam (true), got: {h:?}",
        );

        release_manual_high(&arb).await;
    }

    #[tokio::test]
    async fn no_duplicate_claim_on_repeated_oncoming_signal() {
        let (bus, arb) = setup().await;

        // ManualLighting holding high beam on.
        claim_manual_high(&arb).await;
        drain().await;

        bus.inject(ADAS_ONCOMING, SignalValue::Bool(true));
        drain().await;
        bus.clear_history();

        // Second oncoming while already suppressing — no additional bus activity.
        bus.inject(ADAS_ONCOMING, SignalValue::Bool(true));
        drain().await;

        let h = bus.history();
        for (s, v) in &h {
            if *s == HIGH_BEAM_OUT {
                assert_eq!(
                    *v,
                    SignalValue::Bool(false),
                    "AHB must not flip high beam to true while suppressing, got: {h:?}"
                );
            }
        }

        release_manual_high(&arb).await;
    }

    #[tokio::test]
    async fn no_action_when_already_clear() {
        let (bus, _arb) = setup().await;
        bus.clear_history();

        // Clear signal with no active suppression — must not publish Bool(true).
        bus.inject(ADAS_ONCOMING, SignalValue::Bool(false));
        drain().await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, v)| *s == HIGH_BEAM_OUT && *v == SignalValue::Bool(true)),
            "AHB clear-when-idle must not activate high beam, got: {h:?}",
        );
    }

    #[tokio::test]
    async fn suppress_then_clear_then_suppress_again() {
        let (bus, arb) = setup().await;

        // ManualLighting holding high beam on throughout.
        claim_manual_high(&arb).await;
        drain().await;

        // Cycle 1 — suppress
        bus.inject(ADAS_ONCOMING, SignalValue::Bool(true));
        drain().await;
        // Cycle 1 — clear (ManualLighting's Bool(true) resumes)
        bus.inject(ADAS_ONCOMING, SignalValue::Bool(false));
        drain().await;
        bus.clear_history();

        // Cycle 2 — AHB must re-arm and suppress again
        bus.inject(ADAS_ONCOMING, SignalValue::Bool(true));
        drain().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == HIGH_BEAM_OUT && *v == SignalValue::Bool(false)),
            "second suppression cycle: high beam must be false, got: {h:?}",
        );

        release_manual_high(&arb).await;
    }

    #[tokio::test]
    async fn ahb_does_not_affect_low_beam() {
        let (bus, _arb) = setup().await;
        bus.clear_history();

        bus.inject(ADAS_ONCOMING, SignalValue::Bool(true));
        drain().await;

        let h = bus.history();
        assert!(
            !h.iter().any(|(s, _)| *s == LOW_BEAM),
            "AHB must not touch low beam, got: {h:?}",
        );
    }
}
