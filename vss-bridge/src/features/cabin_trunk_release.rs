//! CabinTrunkRelease — push-button or pull-handle inside the cabin
//! that opens the trunk.  Typical placement: low on the dash for
//! sedans / SUVs, or in the driver footwell for older cars.
//!
//! # Behaviour
//!
//! On a rising edge of `Body.Switches.Trunk.Release.IsPressed`,
//! pulse `Body.Trunk.OpenCmd` through the **trunk arbiter**.  No
//! lock-state or auth gate — the user is already inside the cabin,
//! which is the trust boundary.  Valet mode is enforced at the
//! arbiter's `ValetGate` `PhysicalGate`, the same chokepoint that
//! catches RKE TrunkRelease and the ExteriorTrunkButton.
//!
//! # Why route through the arbiter?
//!
//! Direct-publishing `Body.Trunk.OpenCmd` would bypass the valet
//! gate — a stolen-fob valet attack could bridge through the
//! infotainment to fake a press.  Going through the arbiter gives
//! us valet suppression for free and keeps the trunk-open writers
//! (RKE, exterior, cabin, future phone app, future kick sensor)
//! converging on a single policy chokepoint.
//!
//! # No state, no NVM
//!
//! Stateless reader of one signal.  Edge-triggered.

use std::sync::Arc;

use futures::StreamExt;

use crate::arbiter::{ActuatorRequest, DomainArbiter, TRUNK_OPEN_CMD};
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

const FEATURE_ID: FeatureId = FeatureId::CabinTrunkRelease;
const SWITCH: VssPath = "Body.Switches.Trunk.Release.IsPressed";

pub struct CabinTrunkRelease<B: SignalBus> {
    bus: Arc<B>,
    trunk_arb: Arc<DomainArbiter>,
}

impl<B: SignalBus + Send + Sync + 'static> CabinTrunkRelease<B> {
    pub fn new(bus: Arc<B>, trunk_arb: Arc<DomainArbiter>) -> Self {
        Self { bus, trunk_arb }
    }

    pub async fn run(self) {
        tracing::info!("CabinTrunkRelease feature started");

        let mut switch_rx = self.bus.subscribe(SWITCH).await;
        while let Some(val) = switch_rx.next().await {
            if !matches!(val, SignalValue::Bool(true)) {
                continue;
            }
            tracing::info!("CabinTrunkRelease: switch pressed — pulsing trunk arbiter");
            // Pulse OpenCmd: request true → release.  The arbiter
            // publishes true→false; the plant model only cares about
            // the rising edge.  Valet mode → ValetGate suppresses
            // the publish entirely.
            let _ = self
                .trunk_arb
                .request(ActuatorRequest {
                    signal: TRUNK_OPEN_CMD,
                    value: SignalValue::Bool(true),
                    priority: Priority::Medium,
                    feature_id: FEATURE_ID,
                })
                .await;
            let _ = self.trunk_arb.release(TRUNK_OPEN_CMD, FEATURE_ID).await;

            // No `FEEDBACK_REQUEST` publish here — the hazard two-flash
            // is only fired by EXTERNAL trunk-open paths (RKE,
            // ExteriorTrunkButton, future phone app) where the visual
            // confirms the action to someone outside the vehicle.
            // The cabin button is an interior control: the user is
            // sitting in the vehicle and doesn't need an exterior
            // light show to know they pressed a button.
        }
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::trunk_arbiter;

    async fn settle() {
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    async fn setup() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let (tarb, tarb_fut) = trunk_arbiter(Arc::clone(&bus));
        tokio::spawn(tarb_fut);
        let tarb = Arc::new(tarb);
        tokio::spawn(CabinTrunkRelease::new(Arc::clone(&bus), tarb).run());
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        bus
    }

    fn trunk_was_pulsed(bus: &MockBus) -> bool {
        bus.history()
            .into_iter()
            .any(|(s, v)| s == "Body.Trunk.OpenCmd" && v == SignalValue::Bool(true))
    }

    #[tokio::test]
    async fn press_pulses_trunk_open() {
        let bus = setup().await;
        bus.inject(SWITCH, SignalValue::Bool(true));
        settle().await;
        assert!(trunk_was_pulsed(&bus));
    }

    #[tokio::test]
    async fn falling_edge_is_ignored() {
        let bus = setup().await;
        bus.inject(SWITCH, SignalValue::Bool(false));
        settle().await;
        assert!(!trunk_was_pulsed(&bus));
    }

    #[tokio::test]
    async fn valet_active_suppresses_open() {
        // Valet gate sits on the arbiter, not on this feature.  The
        // feature still issues the request; the gate forces the
        // publish to false → plant model sees no rising edge.
        let bus = setup().await;
        bus.inject("Cabin.ValetMode.IsActive", SignalValue::Bool(true));
        settle().await;
        bus.clear_history();

        bus.inject(SWITCH, SignalValue::Bool(true));
        settle().await;

        assert!(
            !trunk_was_pulsed(&bus),
            "valet active: arbiter must suppress every OpenCmd publish"
        );
    }

    #[tokio::test]
    async fn press_does_not_fire_external_flash_feedback() {
        // Regression: cabin trunk release is an INTERIOR control —
        // it must NOT publish "trunk_unlock" to the lock-feedback
        // pipeline.  External-origin paths (RKE, exterior button)
        // own the hazard two-flash; firing it from the cabin would
        // surprise the user with an exterior light show every time
        // they pop the trunk from inside.
        let bus = setup().await;
        bus.clear_history();

        bus.inject(SWITCH, SignalValue::Bool(true));
        settle().await;

        let fired_flash = bus.history().into_iter().any(|(s, v)| {
            s == "Body.Doors.CentralLock.FeedbackRequest"
                && v == SignalValue::String("trunk_unlock".into())
        });
        assert!(
            !fired_flash,
            "cabin trunk release must not request the exterior trunk-unlock flash"
        );
    }

    #[tokio::test]
    async fn no_lock_state_check() {
        // Cabin trunk release is a privileged interior control —
        // works regardless of cabin lock state.  Verifying we don't
        // mistakenly add a lock-state gate later.
        let bus = setup().await;
        bus.inject("Cabin.LockStatus", SignalValue::String("LOCKED".into()));
        settle().await;
        bus.clear_history();

        bus.inject(SWITCH, SignalValue::Bool(true));
        settle().await;

        assert!(
            trunk_was_pulsed(&bus),
            "cabin trunk release must work even when cabin is LOCKED"
        );
    }
}
