//! Manual Lighting — low beam and high beam stalk control.
//!
//! # Light switch positions (`Body.Lights.LightSwitch`)
//!
//! | Value        | Low beam                            | High beam              |
//! |--------------|-------------------------------------|------------------------|
//! | `"OFF"`      | off                                 | off                    |
//! | `"POSITION"` | off (parking lights future)         | off                    |
//! | `"DRL"`      | off (DRL feature future)            | off                    |
//! | `"AUTO"`     | on when illuminance < threshold     | follows high-beam stalk |
//! | `"BEAM"`     | on (ignition gate only)             | follows high-beam stalk |
//!
//! # AUTO mode — ambient light threshold
//!
//! In `"AUTO"` mode the feature subscribes to
//! `Body.Lights.AmbientLightSensor.Illuminance` (Uint16, lux).
//! Low beam activates when the reading falls below
//! `auto_headlamp_lux_threshold` (calibrated in `VehicleLineCal`,
//! default 200 lux — aligned with ECE R48 §6.1 dusk/dawn threshold).
//! High beam follows `Body.Switches.HighBeam.IsEngaged` while low beam is on.
//!
//! The ambient sensor starts at u16::MAX (full daylight) so that AUTO mode
//! does not switch the headlamps on at cold boot before any sensor reading
//! has arrived.
//!
//! # High beam interlock
//!
//! `Body.Switches.HighBeam.IsEngaged` (Bool) — latched stalk toggle.
//! High beam only activates when low beam is currently on.
//!
//! # Ignition gate
//!
//! Both modes require ignition `ON` or `START`.
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
const ILLUMINANCE: &str = "Body.Lights.AmbientLightSensor.Illuminance";
const LOW_BEAM_OUT: &str = "Body.Lights.Beam.Low.IsOn";
const HIGH_BEAM_OUT: &str = "Body.Lights.Beam.High.IsOn";

// ── Switch position ────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum SwitchPos {
    #[default]
    Off,
    Position,
    Drl,
    Auto,
    Beam,
}

impl SwitchPos {
    fn from_signal(val: &SignalValue) -> Self {
        match val {
            SignalValue::String(s) => match s.as_str() {
                "POSITION" => Self::Position,
                "DRL" => Self::Drl,
                "AUTO" => Self::Auto,
                "BEAM" => Self::Beam,
                _ => Self::Off,
            },
            _ => Self::Off,
        }
    }
}

// ── Helpers ────────────────────────────────────────────────────────────────

fn is_power_on(val: &SignalValue) -> bool {
    matches!(val, SignalValue::String(s) if s == "ON" || s == "START")
}

// ── Feature struct ─────────────────────────────────────────────────────────

pub struct ManualLighting<B: SignalBus> {
    bus: Arc<B>,
    /// Illuminance threshold (lux) below which AUTO mode activates low beam.
    auto_lux_threshold: u16,
}

impl<B: SignalBus + Send + Sync + 'static> ManualLighting<B> {
    pub fn new(bus: Arc<B>, auto_lux_threshold: u16) -> Self {
        Self {
            bus,
            auto_lux_threshold,
        }
    }

    pub async fn run(self) {
        let mut power_rx = self.bus.subscribe(POWER_STATE).await;
        let mut switch_rx = self.bus.subscribe(LIGHT_SWITCH).await;
        let mut hb_rx = self.bus.subscribe(HIGH_BEAM_SWITCH).await;
        let mut lux_rx = self.bus.subscribe(ILLUMINANCE).await;

        let mut ignition_on = false;
        let mut switch_pos = SwitchPos::Off;
        let mut high_beam_engaged = false;
        // Start at maximum lux — no headlamps at boot before first sensor reading.
        let mut ambient_lux: u16 = u16::MAX;

        tracing::info!(
            threshold_lux = self.auto_lux_threshold,
            "ManualLighting feature started"
        );

        loop {
            select! {
                Some(val) = power_rx.next() => {
                    ignition_on = is_power_on(&val);
                    self.apply(ignition_on, switch_pos, high_beam_engaged, ambient_lux).await;
                }
                Some(val) = switch_rx.next() => {
                    switch_pos = SwitchPos::from_signal(&val);
                    self.apply(ignition_on, switch_pos, high_beam_engaged, ambient_lux).await;
                }
                Some(val) = hb_rx.next() => {
                    high_beam_engaged = val == SignalValue::Bool(true);
                    self.apply(ignition_on, switch_pos, high_beam_engaged, ambient_lux).await;
                }
                Some(val) = lux_rx.next() => {
                    if let SignalValue::Uint16(lux) = val {
                        ambient_lux = lux;
                        tracing::debug!(lux, threshold = self.auto_lux_threshold, "ambient illuminance update");
                        self.apply(ignition_on, switch_pos, high_beam_engaged, ambient_lux).await;
                    }
                }
                else => break,
            }
        }

        tracing::info!("ManualLighting feature stopped");
    }

    async fn apply(
        &self,
        ignition_on: bool,
        switch_pos: SwitchPos,
        high_beam_engaged: bool,
        ambient_lux: u16,
    ) {
        let low_on = ignition_on
            && match switch_pos {
                SwitchPos::Beam => true,
                SwitchPos::Auto => ambient_lux < self.auto_lux_threshold,
                _ => false,
            };
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

    const THRESHOLD: u16 = 200;

    async fn setup() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let feature = ManualLighting::new(Arc::clone(&bus), THRESHOLD);
        tokio::spawn(feature.run());
        tokio::task::yield_now().await;
        bus
    }

    async fn drain() {
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(5)).await;
        tokio::task::yield_now().await;
    }

    #[tokio::test]
    async fn beam_switch_with_ignition_on_enables_low_beam() {
        let bus = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(LIGHT_SWITCH, SignalValue::String("BEAM".into()));
        drain().await;
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
        bus.inject(LIGHT_SWITCH, SignalValue::String("OFF".into()));
        drain().await;
        let h = bus.history();
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
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(LIGHT_SWITCH, SignalValue::String("BEAM".into()));
        bus.inject(HIGH_BEAM_SWITCH, SignalValue::Bool(true));
        drain().await;
        bus.clear_history();

        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        drain().await;

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
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(LIGHT_SWITCH, SignalValue::String("OFF".into()));
        bus.inject(HIGH_BEAM_SWITCH, SignalValue::Bool(true));
        drain().await;
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
        drain().await;
        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == HIGH_BEAM_OUT && *v == SignalValue::Bool(true)),
            "expected High.IsOn = true with low beam on, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn auto_mode_below_threshold_enables_low_beam() {
        let bus = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(LIGHT_SWITCH, SignalValue::String("AUTO".into()));
        // Inject lux below threshold (THRESHOLD - 1 = 199)
        bus.inject(ILLUMINANCE, SignalValue::Uint16(THRESHOLD - 1));
        drain().await;
        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == LOW_BEAM_OUT && *v == SignalValue::Bool(true)),
            "AUTO + lux below threshold should enable low beam, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn auto_mode_above_threshold_no_low_beam() {
        let bus = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(LIGHT_SWITCH, SignalValue::String("AUTO".into()));
        // Inject lux above threshold (THRESHOLD + 1 = 201)
        bus.inject(ILLUMINANCE, SignalValue::Uint16(THRESHOLD + 1));
        drain().await;
        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, v)| *s == LOW_BEAM_OUT && *v == SignalValue::Bool(true)),
            "AUTO + lux above threshold should NOT enable low beam, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn auto_mode_no_sensor_reading_no_low_beam() {
        let bus = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(LIGHT_SWITCH, SignalValue::String("AUTO".into()));
        // No illuminance injected — should default to max lux (daylight, no beam)
        drain().await;
        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, v)| *s == LOW_BEAM_OUT && *v == SignalValue::Bool(true)),
            "AUTO with no sensor reading should NOT enable low beam (safe default), got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn auto_mode_lux_rises_above_threshold_turns_beam_off() {
        let bus = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(LIGHT_SWITCH, SignalValue::String("AUTO".into()));
        bus.inject(ILLUMINANCE, SignalValue::Uint16(50)); // dark
        drain().await;
        bus.clear_history();

        // Sun comes up
        bus.inject(ILLUMINANCE, SignalValue::Uint16(5000));
        drain().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == LOW_BEAM_OUT && *v == SignalValue::Bool(false)),
            "LOW beam should turn off when lux rises above threshold, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn position_mode_does_not_enable_beams() {
        let bus = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(LIGHT_SWITCH, SignalValue::String("POSITION".into()));
        drain().await;
        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, v)| *s == LOW_BEAM_OUT && *v == SignalValue::Bool(true)),
            "POSITION mode should not enable low beam, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn acc_state_forces_beams_off() {
        let bus = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(LIGHT_SWITCH, SignalValue::String("BEAM".into()));
        drain().await;
        bus.clear_history();

        bus.inject(POWER_STATE, SignalValue::String("ACC".into()));
        drain().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == LOW_BEAM_OUT && *v == SignalValue::Bool(false)),
            "ACC state should force low beam off, got: {:?}",
            h
        );
    }
}
