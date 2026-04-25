//! Follow-Me-Home — activates low beam briefly after the driver exits in the dark.
//!
//! # Trigger
//!
//! When all three conditions are true simultaneously:
//! 1. Ignition is `OFF` or `ACC` (feature is **armed**).
//! 2. The driver door (`Body.Doors.Row1.Left.IsOpen`) transitions closed → ajar
//!    (rising edge only — a closing door does not trigger).
//! 3. Ambient illuminance (`Body.Lights.AmbientLightSensor.Illuminance`) is below
//!    `lux_threshold` at the moment of the door-open event (dark outside).
//!
//! # Active period
//!
//! While active the feature holds HIGH-priority claims on:
//! - `Body.Lights.Beam.Low.IsOn`
//! - `Body.Lights.Parking.IsOn`
//! - `Body.Lights.LicensePlate.IsOn`
//!
//! Claims are submitted via the `LowBeam` domain arbiter so ManualLighting's
//! simultaneous MEDIUM-priority releases cannot extinguish them.
//!
//! # Cancellation
//!
//! The timer (45 s by default, `FMH_DURATION_SECS`) is cancelled early if
//! ignition returns to `ON` or `START` before it expires.
//!
//! # Daylight suppression
//!
//! FMH does not trigger when lux ≥ threshold. The lux check uses the same
//! threshold as ManualLighting AUTO mode (`auto_headlamp_lux_threshold` from
//! `VehicleLineCal`, default 200 lux / ECE R48 §6.1).

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;
use tokio::time::{sleep_until, Duration, Instant};

use crate::arbiter::{ActuatorRequest, DomainArbiter};
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

// ── Signal constants ───────────────────────────────────────────────────────

const POWER_STATE: &str = "Vehicle.LowVoltageSystemState";
const ILLUMINANCE: &str = "Body.Lights.AmbientLightSensor.Illuminance";
/// Driver door (LHD — Row1.Left). RHD variants would use Row1.Right.
const DRIVER_DOOR: &str = "Body.Doors.Row1.Left.IsOpen";

const LOW_BEAM_OUT: VssPath = "Body.Lights.Beam.Low.IsOn";
const PARKING_OUT: VssPath = "Body.Lights.Parking.IsOn";
const LICENSE_PLATE_OUT: VssPath = "Body.Lights.LicensePlate.IsOn";

/// Signals claimed by FMH at HIGH priority while the timer is active.
const FMH_SIGNALS: &[VssPath] = &[LOW_BEAM_OUT, PARKING_OUT, LICENSE_PLATE_OUT];

/// Follow-Me-Home active duration.
pub const FMH_DURATION_SECS: u64 = 45;

// ── Helpers ────────────────────────────────────────────────────────────────

fn is_power_on(val: &SignalValue) -> bool {
    matches!(val, SignalValue::String(s) if s == "ON" || s == "START")
}

// ── Feature struct ─────────────────────────────────────────────────────────

pub struct FollowMeHome<B: SignalBus> {
    arbiter: Arc<DomainArbiter>,
    bus: Arc<B>,
    /// Same threshold used by ManualLighting AUTO mode.
    lux_threshold: u16,
}

impl<B: SignalBus + Send + Sync + 'static> FollowMeHome<B> {
    pub fn new(arbiter: Arc<DomainArbiter>, bus: Arc<B>, lux_threshold: u16) -> Self {
        Self {
            arbiter,
            bus,
            lux_threshold,
        }
    }

    pub async fn run(self) {
        let mut power_rx = self.bus.subscribe(POWER_STATE).await;
        let mut door_rx = self.bus.subscribe(DRIVER_DOOR).await;
        let mut lux_rx = self.bus.subscribe(ILLUMINANCE).await;

        let mut fmh_armed = false; // true once ignition goes off
        let mut door_ajar = false;
        let mut ambient_lux: u16 = u16::MAX; // safe default — daylight until first reading
        let mut fmh_deadline: Option<Instant> = None;

        tracing::info!(
            lux_threshold = self.lux_threshold,
            duration_s = FMH_DURATION_SECS,
            "FollowMeHome feature started"
        );

        loop {
            let fmh_expiry = async {
                match fmh_deadline {
                    Some(dl) => sleep_until(dl).await,
                    None => std::future::pending().await,
                }
            };

            select! {
                Some(val) = power_rx.next() => {
                    if is_power_on(&val) {
                        fmh_armed = false;
                        if fmh_deadline.take().is_some() {
                            tracing::info!("FollowMeHome cancelled — ignition ON");
                            self.release_all().await;
                        }
                    } else {
                        fmh_armed = true;
                    }
                }
                Some(val) = door_rx.next() => {
                    let was_ajar = door_ajar;
                    door_ajar = matches!(val, SignalValue::Bool(true));
                    // Trigger on rising edge (closed → ajar) while armed and dark.
                    if !was_ajar && door_ajar && fmh_armed && ambient_lux < self.lux_threshold {
                        let deadline = Instant::now() + Duration::from_secs(FMH_DURATION_SECS);
                        fmh_deadline = Some(deadline);
                        tracing::info!(
                            duration_s = FMH_DURATION_SECS,
                            lux = ambient_lux,
                            "FollowMeHome activated"
                        );
                        self.claim_all().await;
                    }
                }
                Some(val) = lux_rx.next() => {
                    if let SignalValue::Uint16(lux) = val {
                        ambient_lux = lux;
                    }
                }
                _ = fmh_expiry => {
                    fmh_deadline = None;
                    tracing::info!("FollowMeHome timer expired — releasing claims");
                    self.release_all().await;
                }
                else => break,
            }
        }

        tracing::info!("FollowMeHome feature stopped");
    }

    async fn claim_all(&self) {
        for &signal in FMH_SIGNALS {
            let result = self
                .arbiter
                .request(ActuatorRequest {
                    signal,
                    value: SignalValue::Bool(true),
                    priority: Priority::High,
                    feature_id: FeatureId::FollowMeHome,
                })
                .await;
            if let Err(e) = result {
                tracing::error!(signal, error = %e, "FollowMeHome: arbiter claim failed");
            }
        }
    }

    async fn release_all(&self) {
        for &signal in FMH_SIGNALS {
            let result = self.arbiter.release(signal, FeatureId::FollowMeHome).await;
            if let Err(e) = result {
                tracing::error!(signal, error = %e, "FollowMeHome: arbiter release failed");
            }
        }
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::low_beam_arbiter;

    const THRESHOLD: u16 = 200;

    async fn setup() -> (Arc<MockBus>, Arc<DomainArbiter>) {
        let bus = Arc::new(MockBus::new());
        let (arb, loop_fut) = low_beam_arbiter(Arc::clone(&bus));
        tokio::spawn(loop_fut);
        let arb = Arc::new(arb);
        let feature = FollowMeHome::new(Arc::clone(&arb), Arc::clone(&bus), THRESHOLD);
        tokio::spawn(feature.run());
        tokio::task::yield_now().await;
        (bus, arb)
    }

    async fn drain() {
        tokio::task::yield_now().await;
        tokio::time::sleep(Duration::from_millis(5)).await;
        tokio::task::yield_now().await;
    }

    async fn drain_yields() {
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
    }

    #[tokio::test]
    async fn fmh_triggers_when_door_opens_after_ignition_off_in_dark() {
        let (bus, _arb) = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        bus.inject(ILLUMINANCE, SignalValue::Uint16(THRESHOLD - 1));
        drain().await;
        bus.clear_history();

        bus.inject(DRIVER_DOOR, SignalValue::Bool(true));
        drain().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == LOW_BEAM_OUT && *v == SignalValue::Bool(true)),
            "FMH should activate low beam when dark and door opens after ignition off, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn fmh_also_activates_parking_and_license_plate() {
        let (bus, _arb) = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        bus.inject(ILLUMINANCE, SignalValue::Uint16(THRESHOLD - 1));
        drain().await;
        bus.clear_history();

        bus.inject(DRIVER_DOOR, SignalValue::Bool(true));
        drain().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == PARKING_OUT && *v == SignalValue::Bool(true)),
            "FMH should activate parking lights, got: {:?}",
            h
        );
        assert!(
            h.iter()
                .any(|(s, v)| *s == LICENSE_PLATE_OUT && *v == SignalValue::Bool(true)),
            "FMH should activate license plate lamp, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn fmh_does_not_trigger_in_daylight() {
        let (bus, _arb) = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        bus.inject(ILLUMINANCE, SignalValue::Uint16(THRESHOLD + 1));
        drain().await;
        bus.clear_history();

        bus.inject(DRIVER_DOOR, SignalValue::Bool(true));
        drain().await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, v)| *s == LOW_BEAM_OUT && *v == SignalValue::Bool(true)),
            "FMH should NOT trigger in daylight, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn fmh_does_not_trigger_with_ignition_on() {
        let (bus, _arb) = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        bus.inject(ILLUMINANCE, SignalValue::Uint16(THRESHOLD - 1));
        bus.inject(DRIVER_DOOR, SignalValue::Bool(true));
        drain().await;
        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, v)| *s == LOW_BEAM_OUT && *v == SignalValue::Bool(true)),
            "FMH should not trigger while ignition is ON, got: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn fmh_does_not_trigger_on_door_close() {
        let (bus, _arb) = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        bus.inject(ILLUMINANCE, SignalValue::Uint16(THRESHOLD - 1));
        bus.inject(DRIVER_DOOR, SignalValue::Bool(true));
        drain().await;
        bus.clear_history();

        bus.inject(DRIVER_DOOR, SignalValue::Bool(false));
        drain().await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, v)| *s == LOW_BEAM_OUT && *v == SignalValue::Bool(true)),
            "FMH must not trigger on door close (falling edge), got: {:?}",
            h
        );
    }

    #[tokio::test(start_paused = true)]
    async fn fmh_turns_off_after_45_seconds() {
        let (bus, _arb) = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        bus.inject(ILLUMINANCE, SignalValue::Uint16(50));
        drain_yields().await;
        bus.inject(DRIVER_DOOR, SignalValue::Bool(true));
        drain_yields().await;
        bus.clear_history();

        tokio::time::advance(Duration::from_secs(FMH_DURATION_SECS + 1)).await;
        drain_yields().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == LOW_BEAM_OUT && *v == SignalValue::Bool(false)),
            "FMH should turn low beam off after {} s, got: {:?}",
            FMH_DURATION_SECS,
            h
        );
    }

    #[tokio::test(start_paused = true)]
    async fn ignition_on_cancels_fmh_early() {
        let (bus, _arb) = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        bus.inject(ILLUMINANCE, SignalValue::Uint16(50));
        drain_yields().await;
        bus.inject(DRIVER_DOOR, SignalValue::Bool(true));
        drain_yields().await;
        tokio::time::advance(Duration::from_secs(5)).await;
        drain_yields().await;
        bus.clear_history();

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        drain_yields().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == LOW_BEAM_OUT && *v == SignalValue::Bool(false)),
            "Ignition ON should cancel FMH immediately, got: {:?}",
            h
        );
    }
}
