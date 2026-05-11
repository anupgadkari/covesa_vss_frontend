//! Chime — plant model for the interior buzzer / piezo.
//!
//! Subscribes to the actuator-intent signal `Body.Chime.IsActive`
//! (the bool a feature publishes when it wants the chime to make
//! noise) and publishes `Body.Chime.IsSounding` — the physical
//! buzzer state.
//!
//! In production this would model real piezo behaviour:
//!   * onset / decay envelope (a few ms ramp so the tone doesn't click),
//!   * volume profile from the cabin amplifier,
//!   * suppression while a higher-priority audio stream is active.
//!
//! For the dev host it's a near-instantaneous mirror — the input
//! edge is republished as the output edge with no transformation.
//! The split is still useful because:
//!   * features write `IsActive` (intent), HMI / telematics observe
//!     `IsSounding` (state).  Keeps responsibilities clean if we add
//!     suppression / ducking later.
//!   * matches the horn / chime symmetry: features request, plant
//!     model represents the actuator, HMI renders the actuator state.
//!
//! Single writer of `Body.Chime.IsActive` today is `PerimeterAlarm`
//! (12 s warning chime); add an arbiter when a second feature wants
//! the same actuator (door-ajar reminder, seatbelt chime, …).

use std::sync::Arc;

use futures::StreamExt;

use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

const CHIME_INTENT: VssPath = "Body.Chime.IsActive";
const CHIME_SOUNDING: VssPath = "Body.Chime.IsSounding";

pub struct ChimePlantModel<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> ChimePlantModel<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    pub async fn run(self) {
        tracing::info!("ChimePlantModel started");

        let mut intent_rx = self.bus.subscribe(CHIME_INTENT).await;

        // Initial state — buzzer silent.  Publish so late subscribers
        // (HMI snapshot) get a deterministic value rather than `None`.
        let _ = self
            .bus
            .publish(CHIME_SOUNDING, SignalValue::Bool(false))
            .await;

        let mut sounding = false;

        while let Some(val) = intent_rx.next().await {
            let want = matches!(val, SignalValue::Bool(true));
            if want == sounding {
                continue; // Idempotent — no edge.
            }
            sounding = want;
            if let Err(e) = self
                .bus
                .publish(CHIME_SOUNDING, SignalValue::Bool(sounding))
                .await
            {
                tracing::error!(error = %e, "ChimePlantModel: publish failed");
            }
        }

        tracing::warn!("ChimePlantModel: intent stream closed, exiting");
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
        let plant = ChimePlantModel::new(Arc::clone(&bus));
        tokio::spawn(plant.run());
        settle().await;
        bus
    }

    #[tokio::test]
    async fn initial_state_published_false() {
        let bus = setup().await;
        assert_eq!(
            bus.latest_value(CHIME_SOUNDING),
            Some(SignalValue::Bool(false)),
            "plant model must publish a deterministic initial value"
        );
    }

    #[tokio::test]
    async fn intent_true_publishes_sounding_true() {
        let bus = setup().await;
        bus.inject(CHIME_INTENT, SignalValue::Bool(true));
        settle().await;

        assert_eq!(
            bus.latest_value(CHIME_SOUNDING),
            Some(SignalValue::Bool(true))
        );
    }

    #[tokio::test]
    async fn intent_false_after_true_publishes_sounding_false() {
        let bus = setup().await;
        bus.inject(CHIME_INTENT, SignalValue::Bool(true));
        settle().await;
        bus.inject(CHIME_INTENT, SignalValue::Bool(false));
        settle().await;

        assert_eq!(
            bus.latest_value(CHIME_SOUNDING),
            Some(SignalValue::Bool(false))
        );
    }

    #[tokio::test]
    async fn redundant_edges_are_idempotent() {
        let bus = setup().await;
        bus.inject(CHIME_INTENT, SignalValue::Bool(true));
        settle().await;
        bus.clear_history();

        // Re-issue the same intent value twice — must NOT re-publish.
        bus.inject(CHIME_INTENT, SignalValue::Bool(true));
        bus.inject(CHIME_INTENT, SignalValue::Bool(true));
        settle().await;

        let republishes = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == CHIME_SOUNDING)
            .count();
        assert_eq!(republishes, 0, "redundant intent must not republish state");
    }

    #[tokio::test]
    async fn pulse_train_round_trips_each_edge() {
        // Mirrors what PerimeterAlarm produces during the chime phase:
        // alternating true/false edges every ~1 s.  Each edge must
        // produce a corresponding IsSounding publish.
        let bus = setup().await;

        for v in [true, false, true, false, true].iter() {
            bus.inject(CHIME_INTENT, SignalValue::Bool(*v));
            settle().await;
        }

        let states: Vec<bool> = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == CHIME_SOUNDING)
            .filter_map(|(_, v)| match v {
                SignalValue::Bool(b) => Some(*b),
                _ => None,
            })
            .collect();
        // Initial false (boot) + 5 edges.
        assert_eq!(states, vec![false, true, false, true, false, true]);
    }
}
