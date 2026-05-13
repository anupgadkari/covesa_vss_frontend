//! VehicleStartingControl — owns `Vehicle.LowVoltageSystemState` and
//! `Vehicle.Starting.ImmobilizerStatus`.  Implements both authentication
//! flavours via the `key_source_cfg` line-cal switch:
//!
//! - [`KeySource::Peps`] — push-button start.  On a `StartStop.IsPressed`
//!   rising edge, the feature submits an `AntennaSet::Cabin` +
//!   `SearchMode::Authenticated` search to the [`KeySearchArbiter`].  If
//!   at least one paired key authenticates, the feature acts on the
//!   request; otherwise the request is ignored and the immobilizer
//!   reports `FAILED`.
//!
//! - [`KeySource::KeyCylinder`] — physical rotary cylinder.  On the
//!   first non-`LOCK` cylinder position after boot or after a return to
//!   `LOCK`, the feature submits an `AntennaSet::Cylinder` +
//!   `SearchMode::Authenticated` search.  If authentication passes,
//!   subsequent cylinder rotations within the same session are accepted
//!   without re-authentication.  If it fails, the power state stays at
//!   `OFF` and `ImmobilizerStatus` goes to `FAILED`.
//!
//! # Brake jump-to-RUN (PEPS only)
//!
//! When the driver presses the start button with the brake applied,
//! the feature jumps straight to `ON` (engine running) regardless of
//! the previous power state.  When the brake is not applied, the
//! feature cycles `OFF → ACC → ON → OFF` one step per press.
//!
//! # Published signals
//!
//! - `Vehicle.LowVoltageSystemState` (String enum: `OFF | ACC | ON |
//!   START`).  This feature is the sole writer.  Boot value: `OFF`.
//! - `Vehicle.Starting.ImmobilizerStatus` (String enum: `LOCKED |
//!   AUTHENTICATING | AUTHENTICATED | FAILED`).  Sole writer.  Boot
//!   value: `LOCKED`.
//!
//! # Subscriptions
//!
//! - `Body.Switches.StartStop.IsPressed` (Bool, PEPS only)
//! - `Body.Switches.IgnitionCylinder.Position` (String, KeyCylinder only)
//! - `Chassis.Brake.IsApplied` (Bool)
//!
//! # Out of scope (deferred)
//!
//! - START momentary flash to simulate crank — kept simple for now.
//! - Powertrain integration — engine speed / RPM signals.
//! - Cylinder antenna as the PEPS-mode low-battery fallback path.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::config::{KeySource, PlatformConfig};
use crate::features::key_search_arbiter::{
    AntennaSet, Coalescing, KeySearchArbiterHandle, SearchMode,
};
use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

// ── Signal constants ───────────────────────────────────────────────────────

const START_STOP_IN: VssPath = "Body.Switches.StartStop.IsPressed";
const CYLINDER_IN: VssPath = "Body.Switches.IgnitionCylinder.Position";
const BRAKE_IN: VssPath = "Chassis.Brake.IsApplied";

const POWER_STATE_OUT: VssPath = "Vehicle.LowVoltageSystemState";
const IMMOBILIZER_OUT: VssPath = "Vehicle.Starting.ImmobilizerStatus";

// ── Enums ──────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PowerState {
    Off,
    Acc,
    On,
    /// `START` is published transiently in real vehicles for the crank
    /// duration.  Today we do not flash through it — kept here so a
    /// future iteration can use it without enum reshuffling.
    #[allow(dead_code)]
    Start,
}

impl PowerState {
    fn as_str(self) -> &'static str {
        match self {
            PowerState::Off => "OFF",
            PowerState::Acc => "ACC",
            PowerState::On => "ON",
            PowerState::Start => "START",
        }
    }

    /// Next state on a brake-not-applied button press.
    fn cycle_next(self) -> Self {
        match self {
            PowerState::Off => PowerState::Acc,
            PowerState::Acc => PowerState::On,
            // ON loops back to OFF; START also collapses to OFF.
            PowerState::On | PowerState::Start => PowerState::Off,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Immobilizer {
    Locked,
    Authenticating,
    Authenticated,
    Failed,
}

impl Immobilizer {
    fn as_str(self) -> &'static str {
        match self {
            Immobilizer::Locked => "LOCKED",
            Immobilizer::Authenticating => "AUTHENTICATING",
            Immobilizer::Authenticated => "AUTHENTICATED",
            Immobilizer::Failed => "FAILED",
        }
    }
}

/// Cylinder rotary position parsed from the HMI signal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CylinderPos {
    Lock,
    Acc,
    On,
    Start,
}

impl CylinderPos {
    fn from_str_value(s: &str) -> Option<Self> {
        match s {
            "LOCK" => Some(Self::Lock),
            "ACC" => Some(Self::Acc),
            "ON" => Some(Self::On),
            "START" => Some(Self::Start),
            _ => None,
        }
    }

    fn to_power(self) -> PowerState {
        match self {
            CylinderPos::Lock => PowerState::Off,
            CylinderPos::Acc => PowerState::Acc,
            CylinderPos::On => PowerState::On,
            CylinderPos::Start => PowerState::Start,
        }
    }
}

// ── Feature struct ─────────────────────────────────────────────────────────

pub struct VehicleStartingControl<B: SignalBus> {
    bus: Arc<B>,
    cfg: Arc<PlatformConfig>,
    key_search: KeySearchArbiterHandle,
}

impl<B: SignalBus + Send + Sync + 'static> VehicleStartingControl<B> {
    pub fn new(
        bus: Arc<B>,
        cfg: Arc<PlatformConfig>,
        key_search: KeySearchArbiterHandle,
    ) -> Self {
        Self {
            bus,
            cfg,
            key_search,
        }
    }

    fn key_source(&self) -> KeySource {
        self.cfg.vehicle_line.key_source_cfg
    }

    pub async fn run(self) {
        tracing::info!(
            mode = ?self.key_source(),
            "VehicleStartingControl started"
        );

        // Subscriptions — set up before publishing seeds so we don't
        // miss our own boot edges in tests.
        let mut start_stop_rx = self.bus.subscribe(START_STOP_IN).await;
        let mut cylinder_rx = self.bus.subscribe(CYLINDER_IN).await;
        let mut brake_rx = self.bus.subscribe(BRAKE_IN).await;

        // Deterministic boot publishes — late subscribers always see a
        // defined value rather than `None`.
        let mut power = PowerState::Off;
        let mut immobilizer = Immobilizer::Locked;
        self.publish_power(power).await;
        self.publish_immobilizer(immobilizer).await;

        let mut brake_applied = false;
        // True after a successful cylinder-mode authentication — survives
        // until the user returns the cylinder to LOCK.
        let mut cylinder_session_authed = false;
        // Track previous start-stop value so we only act on rising edges.
        let mut prev_start_stop = false;

        loop {
            select! {
                Some(val) = brake_rx.next() => {
                    if let SignalValue::Bool(b) = val {
                        brake_applied = b;
                    }
                }

                Some(val) = start_stop_rx.next(),
                    if self.key_source() == KeySource::Peps =>
                {
                    let pressed = matches!(val, SignalValue::Bool(true));
                    let rising = pressed && !prev_start_stop;
                    prev_start_stop = pressed;
                    if !rising {
                        continue;
                    }
                    self.handle_peps_press(
                        &mut power,
                        &mut immobilizer,
                        brake_applied,
                    ).await;
                }

                Some(val) = cylinder_rx.next(),
                    if self.key_source() == KeySource::KeyCylinder =>
                {
                    let pos = match &val {
                        SignalValue::String(s) => CylinderPos::from_str_value(s),
                        _ => None,
                    };
                    let Some(pos) = pos else { continue; };
                    self.handle_cylinder_change(
                        pos,
                        &mut power,
                        &mut immobilizer,
                        &mut cylinder_session_authed,
                    ).await;
                }

                else => break,
            }
        }

        tracing::warn!("VehicleStartingControl: input streams closed, exiting");
    }

    /// PEPS rising-edge press handler.  Authenticates against the cabin
    /// antennas; on success either jumps to `ON` (brake applied) or
    /// cycles the next power state (brake not applied).
    async fn handle_peps_press(
        &self,
        power: &mut PowerState,
        immobilizer: &mut Immobilizer,
        brake_applied: bool,
    ) {
        *immobilizer = Immobilizer::Authenticating;
        self.publish_immobilizer(*immobilizer).await;

        let authed = self.authenticate(AntennaSet::Cabin).await;

        if !authed {
            *immobilizer = Immobilizer::Failed;
            self.publish_immobilizer(*immobilizer).await;
            tracing::info!("VehicleStartingControl: PEPS press rejected (no key in cabin)");
            return;
        }

        *immobilizer = Immobilizer::Authenticated;
        self.publish_immobilizer(*immobilizer).await;

        // Brake jump-to-RUN only applies when the vehicle is currently
        // *off* (OFF or ACC).  Once the engine is running (ON / START),
        // a press always cycles back to OFF regardless of brake state —
        // otherwise a driver sitting at a stoplight (foot on brake,
        // engine running) couldn't shut the car off with a single
        // press.
        let is_off_state = matches!(*power, PowerState::Off | PowerState::Acc);
        let next = if brake_applied && is_off_state {
            PowerState::On
        } else {
            power.cycle_next()
        };
        if next != *power {
            *power = next;
            self.publish_power(*power).await;
        }
        tracing::info!(
            state = power.as_str(),
            brake_applied,
            "VehicleStartingControl: PEPS press accepted"
        );
    }

    /// Cylinder rotation handler.  First non-LOCK rotation in a session
    /// re-authenticates; LOCK clears the session flag.
    async fn handle_cylinder_change(
        &self,
        pos: CylinderPos,
        power: &mut PowerState,
        immobilizer: &mut Immobilizer,
        session_authed: &mut bool,
    ) {
        if pos == CylinderPos::Lock {
            // Returning to LOCK clears the session and forces re-auth
            // on the next rotation away.
            *session_authed = false;
            *immobilizer = Immobilizer::Locked;
            self.publish_immobilizer(*immobilizer).await;
            if *power != PowerState::Off {
                *power = PowerState::Off;
                self.publish_power(*power).await;
            }
            return;
        }

        if !*session_authed {
            *immobilizer = Immobilizer::Authenticating;
            self.publish_immobilizer(*immobilizer).await;

            let authed = self.authenticate(AntennaSet::Cylinder).await;
            if !authed {
                *immobilizer = Immobilizer::Failed;
                self.publish_immobilizer(*immobilizer).await;
                tracing::info!(
                    "VehicleStartingControl: cylinder rotation rejected (no key at cylinder)"
                );
                return;
            }
            *session_authed = true;
            *immobilizer = Immobilizer::Authenticated;
            self.publish_immobilizer(*immobilizer).await;
        }

        let next = pos.to_power();
        if next != *power {
            *power = next;
            self.publish_power(*power).await;
        }
        tracing::info!(
            state = power.as_str(),
            "VehicleStartingControl: cylinder accepted"
        );
    }

    /// Run an authenticated key search via the arbiter; returns true
    /// iff at least one paired key was found.
    async fn authenticate(&self, antennas: AntennaSet) -> bool {
        let res = self
            .key_search
            .submit(
                "VehicleStartingControl",
                antennas,
                SearchMode::Authenticated,
                Coalescing::Disallowed,
            )
            .await;
        match res {
            Some(r) => !r.keys_found.is_empty(),
            None => false,
        }
    }

    async fn publish_power(&self, p: PowerState) {
        let _ = self
            .bus
            .publish(POWER_STATE_OUT, SignalValue::String(p.as_str().into()))
            .await;
    }

    async fn publish_immobilizer(&self, i: Immobilizer) {
        let _ = self
            .bus
            .publish(IMMOBILIZER_OUT, SignalValue::String(i.as_str().into()))
            .await;
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::features::key_search_arbiter::KeySearchArbiter;
    use std::time::Duration;

    async fn settle() {
        // The arbiter simulates LF airtime with a real sleep
        // (100 ms per cabin / cylinder Authenticated scan), so we
        // need to wait long enough for those to complete in real
        // time.  ~250 ms is comfortably above the worst-case path
        // (cabin Authenticated = 100 ms) without dragging the suite.
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        tokio::time::sleep(Duration::from_millis(250)).await;
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    fn cfg_with_key_source(source: KeySource) -> Arc<PlatformConfig> {
        // PlatformConfig holds an Arc<Self> with non-Clone interior
        // RwLock + watch channels, so we can't tweak after the fact —
        // build it from scratch by mirroring `PlatformConfig::defaults`
        // with the line-cal override.
        use crate::config::{DealerConfig, VariantCal, VehicleLineCal};
        use tokio::sync::watch;
        let vl = VehicleLineCal {
            key_source_cfg: source,
            ..VehicleLineCal::default()
        };
        let (tx, rx) = watch::channel(DealerConfig::default());
        // SAFETY-equivalent: PlatformConfig fields used by VSC are
        // public (`vehicle_line`); construct via a transparent struct
        // literal in this same crate.
        Arc::new(PlatformConfig::test_construct(vl, VariantCal::default(), tx, rx))
    }

    fn cfg_peps() -> Arc<PlatformConfig> {
        cfg_with_key_source(KeySource::Peps)
    }

    fn cfg_cylinder() -> Arc<PlatformConfig> {
        cfg_with_key_source(KeySource::KeyCylinder)
    }

    /// Wire up bus, arbiter and feature.  Returns (bus, handle) so each
    /// test can manipulate keys + inputs and inspect publishes.
    async fn setup(cfg: Arc<PlatformConfig>) -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let (arb, handle, rx) = KeySearchArbiter::new_with_rx(Arc::clone(&bus));
        // Very short cadences so the poll loop doesn't drag the test;
        // the start-control feature doesn't depend on the poll loop.
        tokio::spawn(
            arb.with_cadence(Duration::from_millis(20), Duration::from_millis(200))
                .run(rx),
        );
        tokio::spawn(VehicleStartingControl::new(Arc::clone(&bus), cfg, handle).run());
        settle().await;
        bus
    }

    fn latest_power(bus: &MockBus) -> Option<String> {
        match bus.latest_value(POWER_STATE_OUT) {
            Some(SignalValue::String(s)) => Some(s),
            _ => None,
        }
    }
    fn latest_immo(bus: &MockBus) -> Option<String> {
        match bus.latest_value(IMMOBILIZER_OUT) {
            Some(SignalValue::String(s)) => Some(s),
            _ => None,
        }
    }

    fn place_fob_in_cabin(bus: &MockBus, slot: u8) {
        bus.inject(
            match slot {
                1 => "Body.PEPS.Plant.KeyFob.1.Zone",
                2 => "Body.PEPS.Plant.KeyFob.2.Zone",
                3 => "Body.PEPS.Plant.KeyFob.3.Zone",
                4 => "Body.PEPS.Plant.KeyFob.4.Zone",
                _ => panic!("unknown slot"),
            },
            SignalValue::String("Cabin".into()),
        );
    }
    fn place_fob_in_cylinder(bus: &MockBus, slot: u8) {
        bus.inject(
            match slot {
                1 => "Body.PEPS.Plant.KeyFob.1.Zone",
                2 => "Body.PEPS.Plant.KeyFob.2.Zone",
                3 => "Body.PEPS.Plant.KeyFob.3.Zone",
                4 => "Body.PEPS.Plant.KeyFob.4.Zone",
                _ => panic!("unknown slot"),
            },
            SignalValue::String("KeyCylinder".into()),
        );
    }

    // ── PEPS mode ─────────────────────────────────────────────────────

    #[tokio::test]
    async fn peps_boots_to_off_locked() {
        let bus = setup(cfg_peps()).await;
        assert_eq!(latest_power(&bus).as_deref(), Some("OFF"));
        assert_eq!(latest_immo(&bus).as_deref(), Some("LOCKED"));
    }

    #[tokio::test]
    async fn peps_press_without_key_fails_no_state_change() {
        let bus = setup(cfg_peps()).await;
        bus.inject(START_STOP_IN, SignalValue::Bool(true));
        settle().await;
        assert_eq!(latest_power(&bus).as_deref(), Some("OFF"));
        assert_eq!(latest_immo(&bus).as_deref(), Some("FAILED"));
    }

    #[tokio::test]
    async fn peps_press_with_key_in_cabin_cycles_to_acc() {
        let bus = setup(cfg_peps()).await;
        place_fob_in_cabin(&bus, 1);
        settle().await;
        bus.inject(START_STOP_IN, SignalValue::Bool(true));
        settle().await;
        assert_eq!(latest_power(&bus).as_deref(), Some("ACC"));
        assert_eq!(latest_immo(&bus).as_deref(), Some("AUTHENTICATED"));
    }

    #[tokio::test]
    async fn peps_brake_press_jumps_to_on() {
        let bus = setup(cfg_peps()).await;
        place_fob_in_cabin(&bus, 1);
        bus.inject(BRAKE_IN, SignalValue::Bool(true));
        settle().await;
        bus.inject(START_STOP_IN, SignalValue::Bool(true));
        settle().await;
        assert_eq!(latest_power(&bus).as_deref(), Some("ON"));
    }

    #[tokio::test]
    async fn peps_cycle_off_acc_on_off() {
        let bus = setup(cfg_peps()).await;
        place_fob_in_cabin(&bus, 1);
        settle().await;
        // press 1: OFF → ACC
        bus.inject(START_STOP_IN, SignalValue::Bool(true));
        settle().await;
        bus.inject(START_STOP_IN, SignalValue::Bool(false));
        settle().await;
        assert_eq!(latest_power(&bus).as_deref(), Some("ACC"));
        // press 2: ACC → ON
        bus.inject(START_STOP_IN, SignalValue::Bool(true));
        settle().await;
        bus.inject(START_STOP_IN, SignalValue::Bool(false));
        settle().await;
        assert_eq!(latest_power(&bus).as_deref(), Some("ON"));
        // press 3: ON → OFF
        bus.inject(START_STOP_IN, SignalValue::Bool(true));
        settle().await;
        assert_eq!(latest_power(&bus).as_deref(), Some("OFF"));
    }

    #[tokio::test]
    async fn peps_press_with_brake_at_on_state_turns_off() {
        // Real-world scenario: engine running, driver sitting at a
        // stoplight with foot on the brake.  Pressing the start
        // button should turn the engine OFF — not "jump to RUN" and
        // hold at ON.
        let bus = setup(cfg_peps()).await;
        place_fob_in_cabin(&bus, 1);
        bus.inject(BRAKE_IN, SignalValue::Bool(true));
        settle().await;
        // Get to ON first.
        bus.inject(START_STOP_IN, SignalValue::Bool(true));
        settle().await;
        bus.inject(START_STOP_IN, SignalValue::Bool(false));
        settle().await;
        assert_eq!(latest_power(&bus).as_deref(), Some("ON"));
        // Press again with brake still held — must go OFF.
        bus.inject(START_STOP_IN, SignalValue::Bool(true));
        settle().await;
        assert_eq!(latest_power(&bus).as_deref(), Some("OFF"));
    }

    #[tokio::test]
    async fn peps_only_rising_edges_count() {
        let bus = setup(cfg_peps()).await;
        place_fob_in_cabin(&bus, 1);
        settle().await;
        // Two falling-only updates shouldn't advance.
        bus.inject(START_STOP_IN, SignalValue::Bool(false));
        bus.inject(START_STOP_IN, SignalValue::Bool(false));
        settle().await;
        assert_eq!(latest_power(&bus).as_deref(), Some("OFF"));
    }

    // ── KeyCylinder mode ─────────────────────────────────────────────

    #[tokio::test]
    async fn cylinder_boots_to_off_locked() {
        let bus = setup(cfg_cylinder()).await;
        assert_eq!(latest_power(&bus).as_deref(), Some("OFF"));
        assert_eq!(latest_immo(&bus).as_deref(), Some("LOCKED"));
    }

    #[tokio::test]
    async fn cylinder_rotation_without_key_fails() {
        let bus = setup(cfg_cylinder()).await;
        bus.inject(CYLINDER_IN, SignalValue::String("ACC".into()));
        settle().await;
        assert_eq!(latest_power(&bus).as_deref(), Some("OFF"));
        assert_eq!(latest_immo(&bus).as_deref(), Some("FAILED"));
    }

    #[tokio::test]
    async fn cylinder_rotation_with_key_authenticates_and_mirrors() {
        let bus = setup(cfg_cylinder()).await;
        place_fob_in_cylinder(&bus, 1);
        settle().await;
        bus.inject(CYLINDER_IN, SignalValue::String("ACC".into()));
        settle().await;
        assert_eq!(latest_power(&bus).as_deref(), Some("ACC"));
        assert_eq!(latest_immo(&bus).as_deref(), Some("AUTHENTICATED"));
    }

    #[tokio::test]
    async fn cylinder_session_persists_until_lock() {
        let bus = setup(cfg_cylinder()).await;
        place_fob_in_cylinder(&bus, 1);
        settle().await;
        bus.inject(CYLINDER_IN, SignalValue::String("ACC".into()));
        settle().await;
        // Remove fob, then rotate to ON.  Session flag should still
        // accept the rotation because we already authenticated.
        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.Zone",
            SignalValue::String("OutOfRange".into()),
        );
        bus.inject(CYLINDER_IN, SignalValue::String("ON".into()));
        settle().await;
        assert_eq!(latest_power(&bus).as_deref(), Some("ON"));
    }

    #[tokio::test]
    async fn cylinder_return_to_lock_clears_session_and_forces_reauth() {
        let bus = setup(cfg_cylinder()).await;
        place_fob_in_cylinder(&bus, 1);
        settle().await;
        bus.inject(CYLINDER_IN, SignalValue::String("ACC".into()));
        settle().await;
        // Return to LOCK.
        bus.inject(CYLINDER_IN, SignalValue::String("LOCK".into()));
        settle().await;
        assert_eq!(latest_power(&bus).as_deref(), Some("OFF"));
        assert_eq!(latest_immo(&bus).as_deref(), Some("LOCKED"));
        // Remove fob; rotation back to ACC should now fail.
        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.Zone",
            SignalValue::String("OutOfRange".into()),
        );
        bus.inject(CYLINDER_IN, SignalValue::String("ACC".into()));
        settle().await;
        assert_eq!(latest_power(&bus).as_deref(), Some("OFF"));
        assert_eq!(latest_immo(&bus).as_deref(), Some("FAILED"));
    }

    #[tokio::test]
    async fn cylinder_mode_ignores_start_stop_button() {
        // KeyCylinder builds physically lack the push-button, but a
        // stray HMI write must not affect state.
        let bus = setup(cfg_cylinder()).await;
        place_fob_in_cabin(&bus, 1);
        bus.inject(BRAKE_IN, SignalValue::Bool(true));
        settle().await;
        bus.inject(START_STOP_IN, SignalValue::Bool(true));
        settle().await;
        assert_eq!(latest_power(&bus).as_deref(), Some("OFF"));
        assert_eq!(latest_immo(&bus).as_deref(), Some("LOCKED"));
    }

    #[tokio::test]
    async fn peps_mode_ignores_cylinder_signal() {
        let bus = setup(cfg_peps()).await;
        place_fob_in_cylinder(&bus, 1);
        settle().await;
        bus.inject(CYLINDER_IN, SignalValue::String("ON".into()));
        settle().await;
        assert_eq!(latest_power(&bus).as_deref(), Some("OFF"));
    }
}
