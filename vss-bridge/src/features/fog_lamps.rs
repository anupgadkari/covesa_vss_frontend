//! Fog Lamps — front and rear fog lamps driven by dedicated switches.
//!
//! # Logic
//!
//! Both lamps follow the same simple rule:
//! - **Front fog** (`Body.Lights.Fog.Front.IsOn`): ON when
//!   `Body.Switches.Fog.Front.IsEngaged` is `true` **and** ignition is `ON` or `START`.
//! - **Rear fog** (`Body.Lights.Fog.Rear.IsOn`): ON when
//!   `Body.Switches.Fog.Rear.IsEngaged` is `true` **and** ignition is `ON` or `START`.
//!
//! # Ignition gate
//!
//! Both lamps are extinguished on ignition `OFF` / `ACC` regardless of switch state.
//! This matches typical body-controller behaviour (fog lamps require the vehicle
//! to be running or in key-on state).
//!
//! # No arbiter
//!
//! Fog lamps are not contested by any other feature, so this module publishes
//! directly to the signal bus without going through a domain arbiter.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::ipc_message::SignalValue;
use crate::signal_bus::SignalBus;

// ── Signal constants ───────────────────────────────────────────────────────

const POWER_STATE: &str = "Vehicle.LowVoltageSystemState";
const FRONT_SWITCH: &str = "Body.Switches.Fog.Front.IsEngaged";
const REAR_SWITCH: &str = "Body.Switches.Fog.Rear.IsEngaged";
const FRONT_OUT: &str = "Body.Lights.Fog.Front.IsOn";
const REAR_OUT: &str = "Body.Lights.Fog.Rear.IsOn";

// ── Helpers ────────────────────────────────────────────────────────────────

fn is_power_on(val: &SignalValue) -> bool {
    matches!(val, SignalValue::String(s) if s == "ON" || s == "START")
}

fn is_engaged(val: &SignalValue) -> bool {
    matches!(val, SignalValue::Bool(true))
}

// ── Feature struct ─────────────────────────────────────────────────────────

pub struct FogLamps<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> FogLamps<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    pub async fn run(self) {
        let mut power_rx = self.bus.subscribe(POWER_STATE).await;
        let mut front_rx = self.bus.subscribe(FRONT_SWITCH).await;
        let mut rear_rx = self.bus.subscribe(REAR_SWITCH).await;

        let mut ignition_on = false;
        let mut front_sw = false;
        let mut rear_sw = false;

        tracing::info!("FogLamps feature started");

        loop {
            select! {
                Some(val) = power_rx.next() => {
                    ignition_on = is_power_on(&val);
                    self.apply(ignition_on, front_sw, rear_sw).await;
                }
                Some(val) = front_rx.next() => {
                    front_sw = is_engaged(&val);
                    self.apply(ignition_on, front_sw, rear_sw).await;
                }
                Some(val) = rear_rx.next() => {
                    rear_sw = is_engaged(&val);
                    self.apply(ignition_on, front_sw, rear_sw).await;
                }
                else => break,
            }
        }

        tracing::info!("FogLamps feature stopped");
    }

    async fn apply(&self, ignition_on: bool, front_sw: bool, rear_sw: bool) {
        let _ = self
            .bus
            .publish(FRONT_OUT, SignalValue::Bool(ignition_on && front_sw))
            .await;
        let _ = self
            .bus
            .publish(REAR_OUT, SignalValue::Bool(ignition_on && rear_sw))
            .await;
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use tokio::time::{sleep, Duration};

    async fn setup() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        tokio::spawn(FogLamps::new(Arc::clone(&bus)).run());
        tokio::task::yield_now().await;
        bus
    }

    async fn drain() {
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(5)).await;
        tokio::task::yield_now().await;
    }

    #[tokio::test]
    async fn front_fog_on_with_ignition() {
        let bus = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(FRONT_SWITCH, SignalValue::Bool(true));
        drain().await;
        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == FRONT_OUT && *v == SignalValue::Bool(true)),
            "front fog should be ON with ignition ON + switch ON, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn rear_fog_on_with_ignition() {
        let bus = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(REAR_SWITCH, SignalValue::Bool(true));
        drain().await;
        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == REAR_OUT && *v == SignalValue::Bool(true)),
            "rear fog should be ON with ignition ON + switch ON, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn front_fog_off_without_ignition() {
        let bus = setup().await;
        // Ignition stays OFF
        bus.inject(FRONT_SWITCH, SignalValue::Bool(true));
        drain().await;
        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, v)| *s == FRONT_OUT && *v == SignalValue::Bool(true)),
            "front fog should NOT turn on without ignition, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn rear_fog_off_without_ignition() {
        let bus = setup().await;
        bus.inject(REAR_SWITCH, SignalValue::Bool(true));
        drain().await;
        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, v)| *s == REAR_OUT && *v == SignalValue::Bool(true)),
            "rear fog should NOT turn on without ignition, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn ignition_off_extinguishes_fog_lamps() {
        let bus = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(FRONT_SWITCH, SignalValue::Bool(true));
        bus.inject(REAR_SWITCH, SignalValue::Bool(true));
        drain().await;
        bus.clear_history();

        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        drain().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == FRONT_OUT && *v == SignalValue::Bool(false)),
            "ignition OFF should extinguish front fog, got: {:?}",
            h
        );
        assert!(
            h.iter()
                .any(|(s, v)| *s == REAR_OUT && *v == SignalValue::Bool(false)),
            "ignition OFF should extinguish rear fog, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn switch_off_extinguishes_front_fog() {
        let bus = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(FRONT_SWITCH, SignalValue::Bool(true));
        drain().await;
        bus.clear_history();

        bus.inject(FRONT_SWITCH, SignalValue::Bool(false));
        drain().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == FRONT_OUT && *v == SignalValue::Bool(false)),
            "front fog should turn off when switch is OFF, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn front_and_rear_independent() {
        let bus = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(FRONT_SWITCH, SignalValue::Bool(true));
        bus.inject(REAR_SWITCH, SignalValue::Bool(false));
        drain().await;
        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == FRONT_OUT && *v == SignalValue::Bool(true)),
            "front fog should be ON, got: {:?}",
            h
        );
        assert!(
            !h.iter()
                .any(|(s, v)| *s == REAR_OUT && *v == SignalValue::Bool(true)),
            "rear fog should remain OFF, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn acc_state_extinguishes_fog_lamps() {
        let bus = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(FRONT_SWITCH, SignalValue::Bool(true));
        bus.inject(REAR_SWITCH, SignalValue::Bool(true));
        drain().await;
        bus.clear_history();

        bus.inject(POWER_STATE, SignalValue::String("ACC".into()));
        drain().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == FRONT_OUT && *v == SignalValue::Bool(false)),
            "ACC state should extinguish front fog, got: {:?}",
            h
        );
        assert!(
            h.iter()
                .any(|(s, v)| *s == REAR_OUT && *v == SignalValue::Bool(false)),
            "ACC state should extinguish rear fog, got: {:?}",
            h
        );
    }
}
