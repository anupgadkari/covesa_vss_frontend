//! Manual Lighting — low beam and high beam stalk control.
//!
//! # Light switch positions (`Body.Lights.LightSwitch`)
//!
//! | Value      | Low beam          | High beam              |
//! |------------|-------------------|------------------------|
//! | `"OFF"`    | off               | off                    |
//! | `"POSITION"` | off             | off (parking lights future) |
//! | `"DRL"`    | off               | off (DRL feature future) |
//! | `"AUTO"`   | on (ignition gate) | follows high-beam stalk |
//! | `"BEAM"`   | on (ignition gate) | follows high-beam stalk |
//!
//! # High beam
//!
//! `Body.Switches.HighBeam.IsEngaged` (Bool) — latched stalk toggle.
//! High beam only activates when low beam is currently on (interlock).
//!
//! # Ignition gate
//!
//! LOW beam (modes `AUTO` and `BEAM`) requires ignition `ON` or `START`.
//! On ignition `OFF` or `ACC` both beams are forced off regardless of switch
//! position.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::ipc_message::SignalValue;
use crate::signal_bus::SignalBus;

// ── Signal constants ───────────────────────────────────────────────────────

const LIGHT_SWITCH: &str = "Body.Lights.LightSwitch";
const HIGH_BEAM_SWITCH: &str = "Body.Switches.HighBeam.IsEngaged";
const POWER_STATE: &str = "Vehicle.LowVoltageSystemState";
const LOW_BEAM_OUT: &str = "Body.Lights.Beam.Low.IsOn";
const HIGH_BEAM_OUT: &str = "Body.Lights.Beam.High.IsOn";

// ── Helpers ────────────────────────────────────────────────────────────────

fn is_power_on(val: &SignalValue) -> bool {
    matches!(val, SignalValue::String(s) if s == "ON" || s == "START")
}

/// Returns true when the light switch position requests low beam.
fn switch_enables_beam(val: &SignalValue) -> bool {
    matches!(val, SignalValue::String(s) if s == "BEAM" || s == "AUTO")
}

// ── Feature struct ─────────────────────────────────────────────────────────

pub struct ManualLighting<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> ManualLighting<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    pub async fn run(self) {
        let mut power_rx = self.bus.subscribe(POWER_STATE).await;
        let mut switch_rx = self.bus.subscribe(LIGHT_SWITCH).await;
        let mut hb_rx = self.bus.subscribe(HIGH_BEAM_SWITCH).await;

        let mut ignition_on = false;
        let mut beam_switch_on = false;
        let mut high_beam_engaged = false;

        tracing::info!("ManualLighting feature started");

        loop {
            select! {
                Some(val) = power_rx.next() => {
                    ignition_on = is_power_on(&val);
                    self.apply(ignition_on, beam_switch_on, high_beam_engaged).await;
                }
                Some(val) = switch_rx.next() => {
                    beam_switch_on = switch_enables_beam(&val);
                    self.apply(ignition_on, beam_switch_on, high_beam_engaged).await;
                }
                Some(val) = hb_rx.next() => {
                    high_beam_engaged = val == SignalValue::Bool(true);
                    self.apply(ignition_on, beam_switch_on, high_beam_engaged).await;
                }
                else => break,
            }
        }

        tracing::info!("ManualLighting feature stopped");
    }

    async fn apply(&self, ignition_on: bool, beam_switch_on: bool, high_beam_engaged: bool) {
        let low_on = ignition_on && beam_switch_on;
        // High beam interlock: only active when low beam is on.
        let high_on = low_on && high_beam_engaged;
        let _ = self
            .bus
            .publish(LOW_BEAM_OUT, SignalValue::Bool(low_on))
            .await;
        let _ = self
            .bus
            .publish(HIGH_BEAM_OUT, SignalValue::Bool(high_on))
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
        let feature = ManualLighting::new(Arc::clone(&bus));
        tokio::spawn(feature.run());
        tokio::task::yield_now().await;
        bus
    }

    async fn drain(_bus: &MockBus) {
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(5)).await;
        tokio::task::yield_now().await;
    }

    #[tokio::test]
    async fn beam_switch_with_ignition_on_enables_low_beam() {
        let bus = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        drain(&bus).await;
        bus.inject(LIGHT_SWITCH, SignalValue::String("BEAM".into()));
        drain(&bus).await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == LOW_BEAM_OUT && *v == SignalValue::Bool(true)),
            "expected Low.IsOn = true, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn beam_switch_off_no_low_beam() {
        let bus = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        drain(&bus).await;
        bus.inject(LIGHT_SWITCH, SignalValue::String("OFF".into()));
        drain(&bus).await;

        let h = bus.history();
        // No true for low beam
        assert!(
            !h.iter()
                .any(|(s, v)| *s == LOW_BEAM_OUT && *v == SignalValue::Bool(true)),
            "expected Low.IsOn = false when switch OFF, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn ignition_off_forces_both_beams_off() {
        let bus = setup().await;

        // Beams on first
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(LIGHT_SWITCH, SignalValue::String("BEAM".into()));
        bus.inject(HIGH_BEAM_SWITCH, SignalValue::Bool(true));
        drain(&bus).await;
        bus.clear_history();

        // Ignition off
        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        drain(&bus).await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == LOW_BEAM_OUT && *v == SignalValue::Bool(false)),
            "expected Low off after ignition OFF, got: {:?}",
            h
        );
        assert!(
            h.iter()
                .any(|(s, v)| *s == HIGH_BEAM_OUT && *v == SignalValue::Bool(false)),
            "expected High off after ignition OFF, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn high_beam_requires_low_beam_on() {
        let bus = setup().await;

        // Ignition ON but switch OFF — high beam engaged anyway
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(LIGHT_SWITCH, SignalValue::String("OFF".into()));
        bus.inject(HIGH_BEAM_SWITCH, SignalValue::Bool(true));
        drain(&bus).await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, v)| *s == HIGH_BEAM_OUT && *v == SignalValue::Bool(true)),
            "high beam must not fire without low beam, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn high_beam_active_when_low_beam_on() {
        let bus = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(LIGHT_SWITCH, SignalValue::String("BEAM".into()));
        bus.inject(HIGH_BEAM_SWITCH, SignalValue::Bool(true));
        drain(&bus).await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == HIGH_BEAM_OUT && *v == SignalValue::Bool(true)),
            "expected High.IsOn = true with low beam on, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn auto_mode_enables_low_beam() {
        let bus = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(LIGHT_SWITCH, SignalValue::String("AUTO".into()));
        drain(&bus).await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == LOW_BEAM_OUT && *v == SignalValue::Bool(true)),
            "AUTO mode should enable low beam, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn position_mode_does_not_enable_beams() {
        let bus = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(LIGHT_SWITCH, SignalValue::String("POSITION".into()));
        drain(&bus).await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, v)| *s == LOW_BEAM_OUT && *v == SignalValue::Bool(true)),
            "POSITION mode should not enable low beam, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn switch_off_clears_high_beam() {
        let bus = setup().await;

        // Both beams on
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(LIGHT_SWITCH, SignalValue::String("BEAM".into()));
        bus.inject(HIGH_BEAM_SWITCH, SignalValue::Bool(true));
        drain(&bus).await;
        bus.clear_history();

        // Turn switch off
        bus.inject(LIGHT_SWITCH, SignalValue::String("OFF".into()));
        drain(&bus).await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == HIGH_BEAM_OUT && *v == SignalValue::Bool(false)),
            "high beam must clear when light switch goes OFF, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn acc_state_forces_beams_off() {
        let bus = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(LIGHT_SWITCH, SignalValue::String("BEAM".into()));
        drain(&bus).await;
        bus.clear_history();

        bus.inject(POWER_STATE, SignalValue::String("ACC".into()));
        drain(&bus).await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == LOW_BEAM_OUT && *v == SignalValue::Bool(false)),
            "ACC state should force low beam off, got: {:?}",
            h
        );
    }
}
