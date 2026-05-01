//! ManualHorn — steering-wheel horn pad driver.
//!
//! Subscribes to `Body.Switches.Horn.IsPressed` and claims
//! `Body.Horn.IsActive` at `Medium` priority while pressed.  Releases
//! on release.  PanicAlarm at `High` preempts when the alarm is
//! engaged; the arbiter handles the priority resolution.
//!
//! # Edge handling
//!
//! Press = `Bool(true)` → claim.  Release = `Bool(false)` → release.
//! Anything else (parse failure, missing value) is treated as a
//! release for safety — a stuck claim is the worst outcome
//! (continuous horn) so we err on the side of releasing.

use std::sync::Arc;

use futures::StreamExt;

use crate::arbiter::{ActuatorRequest, DomainArbiter};
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

const FEATURE_ID: FeatureId = FeatureId::ManualHorn;
const SWITCH: VssPath = "Body.Switches.Horn.IsPressed";
const HORN: VssPath = "Body.Horn.IsActive";

pub struct ManualHorn<B: SignalBus> {
    bus: Arc<B>,
    horn_arb: Arc<DomainArbiter>,
}

impl<B: SignalBus + Send + Sync + 'static> ManualHorn<B> {
    pub fn new(bus: Arc<B>, horn_arb: Arc<DomainArbiter>) -> Self {
        Self { bus, horn_arb }
    }

    pub async fn run(self) {
        tracing::info!("ManualHorn feature started");

        let mut switch_rx = self.bus.subscribe(SWITCH).await;
        let mut claimed = false;

        while let Some(val) = switch_rx.next().await {
            let want_claim = matches!(val, SignalValue::Bool(true));
            if want_claim && !claimed {
                let _ = self
                    .horn_arb
                    .request(ActuatorRequest {
                        signal: HORN,
                        value: SignalValue::Bool(true),
                        priority: Priority::Medium,
                        feature_id: FEATURE_ID,
                    })
                    .await;
                claimed = true;
            } else if !want_claim && claimed {
                let _ = self.horn_arb.release(HORN, FEATURE_ID).await;
                claimed = false;
            }
        }
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::horn_arbiter;

    async fn settle() {
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    async fn setup() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let (harb, harb_fut) = horn_arbiter(Arc::clone(&bus));
        tokio::spawn(harb_fut);
        let harb = Arc::new(harb);
        tokio::spawn(ManualHorn::new(Arc::clone(&bus), harb).run());
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        bus
    }

    #[tokio::test]
    async fn press_engages_horn() {
        let bus = setup().await;
        bus.inject(SWITCH, SignalValue::Bool(true));
        settle().await;
        assert_eq!(bus.latest_value(HORN), Some(SignalValue::Bool(true)));
    }

    #[tokio::test]
    async fn release_clears_horn() {
        let bus = setup().await;
        bus.inject(SWITCH, SignalValue::Bool(true));
        settle().await;
        assert_eq!(bus.latest_value(HORN), Some(SignalValue::Bool(true)));

        bus.inject(SWITCH, SignalValue::Bool(false));
        settle().await;
        assert_eq!(bus.latest_value(HORN), Some(SignalValue::Bool(false)));
    }

    #[tokio::test]
    async fn double_press_is_idempotent() {
        // A repeated `true` while already claiming should not
        // generate spurious bus traffic.
        let bus = setup().await;
        bus.inject(SWITCH, SignalValue::Bool(true));
        settle().await;
        let count_after_first = bus
            .history()
            .into_iter()
            .filter(|(s, _)| *s == HORN)
            .count();

        bus.inject(SWITCH, SignalValue::Bool(true));
        settle().await;
        let count_after_second = bus
            .history()
            .into_iter()
            .filter(|(s, _)| *s == HORN)
            .count();

        assert_eq!(count_after_first, count_after_second);
    }
}
