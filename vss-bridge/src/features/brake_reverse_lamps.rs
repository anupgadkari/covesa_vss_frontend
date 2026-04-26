//! Brake and Reverse Lamps — pedal-driven stop lights and gear-driven backup lights.
//!
//! # Brake lights (`Body.Lights.Brake.IsActive`)
//!
//! Activated whenever `Chassis.Brake.PedalPosition` (Uint8, 0–100 %) is greater
//! than zero.  No ignition gate — brake lights are a safety output and operate
//! in any power state.
//!
//! # Reverse lights (`Body.Lights.Backup.IsActive`)
//!
//! Activated when `Powertrain.Transmission.CurrentGear` (Int16) is negative
//! **and** ignition is `ON` or `START`.  Reverse lights are suppressed on
//! ignition `OFF` / `ACC` to match typical body-controller behaviour.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::ipc_message::SignalValue;
use crate::signal_bus::SignalBus;

// ── Signal constants ───────────────────────────────────────────────────────

const POWER_STATE: &str = "Vehicle.LowVoltageSystemState";
const BRAKE_PEDAL: &str = "Chassis.Brake.PedalPosition";
const GEAR: &str = "Powertrain.Transmission.CurrentGear";
const BRAKE_OUT: &str = "Body.Lights.Brake.IsActive";
const REVERSE_OUT: &str = "Body.Lights.Backup.IsActive";

// ── Helpers ────────────────────────────────────────────────────────────────

fn is_power_on(val: &SignalValue) -> bool {
    matches!(val, SignalValue::String(s) if s == "ON" || s == "START")
}

/// Brake pedal is considered pressed for any position > 0 %.
fn pedal_pressed(val: &SignalValue) -> bool {
    matches!(val, SignalValue::Uint8(p) if *p > 0)
}

/// Negative gear value = reverse.
/// Neutral (0) and forward gears (≥ 1) arrive as Uint8; reverse gears
/// arrive as Int16 (negative) from ws_bridge's json_to_signal_value.
/// Also handles String("-1") / String("-2") defensively in case a caller
/// sends the value as a quoted number (e.g. legacy JSON clients).
fn is_reverse(val: &SignalValue) -> bool {
    match val {
        SignalValue::Int16(g) => *g < 0,
        SignalValue::String(s) => s.parse::<i64>().map(|n| n < 0).unwrap_or(false),
        _ => false,
    }
}

// ── Feature struct ─────────────────────────────────────────────────────────

pub struct BrakeReverseLamps<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> BrakeReverseLamps<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    pub async fn run(self) {
        let mut power_rx = self.bus.subscribe(POWER_STATE).await;
        let mut pedal_rx = self.bus.subscribe(BRAKE_PEDAL).await;
        let mut gear_rx = self.bus.subscribe(GEAR).await;

        let mut ignition_on = false;
        let mut brake_active = false;
        let mut in_reverse = false;

        tracing::info!("BrakeReverseLamps feature started");

        loop {
            select! {
                Some(val) = power_rx.next() => {
                    ignition_on = is_power_on(&val);
                    self.apply(ignition_on, brake_active, in_reverse).await;
                }
                Some(val) = pedal_rx.next() => {
                    brake_active = pedal_pressed(&val);
                    self.apply(ignition_on, brake_active, in_reverse).await;
                }
                Some(val) = gear_rx.next() => {
                    in_reverse = is_reverse(&val);
                    self.apply(ignition_on, brake_active, in_reverse).await;
                }
                else => break,
            }
        }

        tracing::info!("BrakeReverseLamps feature stopped");
    }

    async fn apply(&self, ignition_on: bool, brake_active: bool, in_reverse: bool) {
        // Brake lights: no ignition gate (safety requirement).
        let brake_on = brake_active;
        // Reverse lights: ignition-gated.
        let reverse_on = ignition_on && in_reverse;
        let _ = self
            .bus
            .publish(BRAKE_OUT, SignalValue::Bool(brake_on))
            .await;
        let _ = self
            .bus
            .publish(REVERSE_OUT, SignalValue::Bool(reverse_on))
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
        tokio::spawn(BrakeReverseLamps::new(Arc::clone(&bus)).run());
        tokio::task::yield_now().await;
        bus
    }

    async fn drain() {
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(5)).await;
        tokio::task::yield_now().await;
    }

    // ── Brake lights ──────────────────────────────────────────────────

    #[tokio::test]
    async fn brake_pedal_pressed_activates_brake_lights() {
        let bus = setup().await;
        bus.inject(BRAKE_PEDAL, SignalValue::Uint8(30));
        drain().await;
        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == BRAKE_OUT && *v == SignalValue::Bool(true)),
            "brake pedal pressed should activate brake lights, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn brake_pedal_released_deactivates_brake_lights() {
        let bus = setup().await;
        bus.inject(BRAKE_PEDAL, SignalValue::Uint8(50));
        drain().await;
        bus.clear_history();

        bus.inject(BRAKE_PEDAL, SignalValue::Uint8(0));
        drain().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == BRAKE_OUT && *v == SignalValue::Bool(false)),
            "brake pedal released should deactivate brake lights, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn brake_lights_active_without_ignition() {
        let bus = setup().await;
        // Ignition stays OFF (never set)
        bus.inject(BRAKE_PEDAL, SignalValue::Uint8(20));
        drain().await;
        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == BRAKE_OUT && *v == SignalValue::Bool(true)),
            "brake lights should work regardless of ignition state, got: {:?}",
            h
        );
    }

    // ── Reverse lights ────────────────────────────────────────────────

    #[tokio::test]
    async fn reverse_gear_with_ignition_activates_backup_lights() {
        let bus = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(GEAR, SignalValue::Int16(-1));
        drain().await;
        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == REVERSE_OUT && *v == SignalValue::Bool(true)),
            "reverse gear + ignition ON should activate backup lights, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn reverse_gear_without_ignition_no_backup_lights() {
        let bus = setup().await;
        // Ignition stays OFF
        bus.inject(GEAR, SignalValue::Int16(-1));
        drain().await;
        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, v)| *s == REVERSE_OUT && *v == SignalValue::Bool(true)),
            "reverse gear without ignition should NOT activate backup lights, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn forward_gear_no_backup_lights() {
        let bus = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(GEAR, SignalValue::Uint8(1));
        drain().await;
        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, v)| *s == REVERSE_OUT && *v == SignalValue::Bool(true)),
            "forward gear should NOT activate backup lights, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn neutral_gear_no_backup_lights() {
        let bus = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(GEAR, SignalValue::Uint8(0));
        drain().await;
        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, v)| *s == REVERSE_OUT && *v == SignalValue::Bool(true)),
            "neutral gear should NOT activate backup lights, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn ignition_off_suppresses_backup_lights() {
        let bus = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(GEAR, SignalValue::Int16(-1));
        drain().await;
        bus.clear_history();

        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        drain().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == REVERSE_OUT && *v == SignalValue::Bool(false)),
            "ignition OFF should suppress backup lights, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn brake_and_reverse_can_be_active_simultaneously() {
        let bus = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(GEAR, SignalValue::Int16(-1));
        bus.inject(BRAKE_PEDAL, SignalValue::Uint8(80));
        drain().await;
        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == BRAKE_OUT && *v == SignalValue::Bool(true)),
            "brake should be active, got: {:?}",
            h
        );
        assert!(
            h.iter()
                .any(|(s, v)| *s == REVERSE_OUT && *v == SignalValue::Bool(true)),
            "reverse should be active simultaneously, got: {:?}",
            h
        );
    }
}
