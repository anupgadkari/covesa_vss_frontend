//! WebSocket bridge — connects the Web HMI (L6) to the SignalBus (L5).
//!
//! Listens on `0.0.0.0:8080` for WebSocket connections from the HMI.
//!
//! Protocol (JSON):
//!   HMI → bridge:  {"type":"sensor","path":"Body.Switches.Hazard.IsEngaged","value":true}
//!   HMI → bridge:  {"type":"config_set","key":"dealer.two_stage_unlock","value":true}
//!   bridge → HMI:  {"state":{"Body.Lights.DirectionIndicator.Left.IsSignaling":true,...}}
//!   bridge → HMI:  {"config":{"dealer":{...},"variant":{...},"vehicle_line":{...}}}
//!
//! The bridge injects sensor values into the SignalBus (simulating physical
//! switch inputs). It subscribes to output signals (actuator results from the
//! arbiter) and pushes state snapshots back to the HMI.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::Arc;

use futures::{SinkExt, StreamExt};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::{broadcast, Mutex, Notify};
use tokio::time::{sleep, Duration};
use tokio_tungstenite::tungstenite::Message;

use crate::config::PlatformConfig;
use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

/// Signals the HMI can write (sensor inputs — physical switches/stalks).
const INPUT_SIGNALS: &[VssPath] = &[
    "Vehicle.LowVoltageSystemState",
    "Body.Switches.Hazard.IsEngaged",
    "Body.Switches.TurnIndicator.Direction",
    "Body.Switches.HighBeam.IsEngaged",
    "Body.Switches.Fog.Front.IsEngaged",
    "Body.Switches.Fog.Rear.IsEngaged",
    "Chassis.ParkingBrake.IsEngaged",
    "Chassis.Brake.PedalPosition",
    "Powertrain.Transmission.CurrentGear",
    "Body.Lights.LightSwitch",
    "Body.PEPS.KeyPresent",
    "Body.Switches.Keyfob.LockButton",
    "Body.Switches.DoorTrim.Row1.Left.LockButton",
    "Body.Switches.DoorTrim.Row1.Right.LockButton",
    "Body.Switches.DoorTrim.Row2.Left.LockButton",
    "Body.Switches.DoorTrim.Row2.Right.LockButton",
    "Body.Switches.DoorTrim.Row1.Left.UnlockButton",
    "Body.Switches.DoorTrim.Row1.Right.UnlockButton",
    "Body.Connectivity.RemoteLock",
    "Body.Connectivity.BleLock",
    "Body.Connectivity.NfcCardPresent",
    "Body.Connectivity.NfcPhonePresent",
    "Vehicle.Safety.CrashDetected",
    "Body.Lights.AmbientLightSensor.Illuminance",
    // ADAS camera input — HMI toggle simulates oncoming vehicle detection.
    "Vehicle.ADAS.HighBeam.OncomingVehicleDetected",
    // Bulb defect fault-injection (HMI toggles to simulate failed lamp).
    // Three physical lamps per side: Front, Side (mirror repeater), Rear.
    "Body.Lights.DirectionIndicator.Left.Lamp.Front.IsDefect",
    "Body.Lights.DirectionIndicator.Left.Lamp.Side.IsDefect",
    "Body.Lights.DirectionIndicator.Left.Lamp.Rear.IsDefect",
    "Body.Lights.DirectionIndicator.Right.Lamp.Front.IsDefect",
    "Body.Lights.DirectionIndicator.Right.Lamp.Side.IsDefect",
    "Body.Lights.DirectionIndicator.Right.Lamp.Rear.IsDefect",
    // PEPS plant model inputs — HMI positions devices and presses fob buttons.
    "Body.PEPS.Plant.KeyFob.1.Zone",
    "Body.PEPS.Plant.KeyFob.2.Zone",
    "Body.PEPS.Plant.KeyFob.3.Zone",
    "Body.PEPS.Plant.KeyFob.4.Zone",
    "Body.PEPS.Plant.KeyFob.5.Zone",
    "Body.PEPS.Plant.KeyFob.6.Zone",
    "Body.PEPS.Plant.KeyFob.1.ButtonPress",
    "Body.PEPS.Plant.KeyFob.2.ButtonPress",
    "Body.PEPS.Plant.KeyFob.3.ButtonPress",
    "Body.PEPS.Plant.KeyFob.4.ButtonPress",
    "Body.PEPS.Plant.BlePhone.1.Zone",
    "Body.PEPS.Plant.BlePhone.2.Zone",
    "Body.PEPS.Plant.NfcCard.1.Position",
    "Body.PEPS.Plant.NfcCard.2.Position",
    // Door handle plant model inputs — HMI top-view physical interactions.
    "Body.Doors.Row1.Left.Handle.Inside.IsPulled",
    "Body.Doors.Row1.Right.Handle.Inside.IsPulled",
    "Body.Doors.Row2.Left.Handle.Inside.IsPulled",
    "Body.Doors.Row2.Right.Handle.Inside.IsPulled",
    "Body.Doors.Row1.Left.Handle.Outside.IsPulled",
    "Body.Doors.Row1.Right.Handle.Outside.IsPulled",
    "Body.Doors.Row2.Left.Handle.Outside.IsPulled",
    "Body.Doors.Row2.Right.Handle.Outside.IsPulled",
    // Soldier (interior lock knob) — per-door manual lock override.
    "Body.Doors.Row1.Left.Soldier.IsUnlocked",
    "Body.Doors.Row1.Right.Soldier.IsUnlocked",
    "Body.Doors.Row2.Left.Soldier.IsUnlocked",
    "Body.Doors.Row2.Right.Soldier.IsUnlocked",
    // Close command — sent when user clicks an ajar door in the top view.
    "Body.Doors.Row1.Left.CloseCmd",
    "Body.Doors.Row1.Right.CloseCmd",
    "Body.Doors.Row2.Left.CloseCmd",
    "Body.Doors.Row2.Right.CloseCmd",
    // Trunk close command — sent when user taps the open trunk in the HMI.
    "Body.Trunk.CloseCmd",
    // Exterior trunk-release button — capacitive press above the licence
    // plate.  Consumed by the ExteriorTrunkButton + PassiveEntry features.
    "Body.Trunk.ExteriorButton.IsPressed",
    // Valet mode toggle — published by infotainment in production.
    // Gates the trunk arbiter's ValetGate `PhysicalGate`.
    "Cabin.ValetMode.IsActive",
    // Hood commands — HoodPlantModel runs a tri-state latch FSM.
    //   • Body.Switches.Hood.Release.IsPulled — dash release lever
    //     (momentary, double-pull within 3 s pops to HALF_LATCHED).
    //   • Body.Hood.OpenCmd  — top-view click HALF_LATCHED → OPEN.
    //   • Body.Hood.CloseCmd — top-view click OPEN → LATCHED.
    "Body.Switches.Hood.Release.IsPulled",
    "Body.Hood.OpenCmd",
    "Body.Hood.CloseCmd",
    // Cabin trunk release switch — interior push-button / pull-handle.
    // CabinTrunkRelease feature pulses Body.Trunk.OpenCmd through the
    // trunk arbiter (which applies the valet gate).
    "Body.Switches.Trunk.Release.IsPressed",
    // Steering-wheel horn pad — momentary press.  ManualHorn feature
    // claims Body.Horn.IsActive at Medium priority while held.
    "Body.Switches.Horn.IsPressed",
    // Sunroof commands — string enum "OPEN" | "CLOSE" | "STOP".
    // SunroofPlantModel runs a 5 s full-travel motor model and
    // NVM-persists settled positions.
    "Body.Sunroof.MoveCmd",
    "Body.Sunroof.Shade.MoveCmd",
    // Diagnostic overrides (DoorCard direct-write).
    "Body.Doors.Row1.Left.IsOpen",
    "Body.Doors.Row1.Right.IsOpen",
    "Body.Doors.Row2.Left.IsOpen",
    "Body.Doors.Row2.Right.IsOpen",
    // Direct trunk open/close override (control panel and sensor page).
    "Body.Trunk.IsOpen",
    // Central lock command — HMI DblLock toggle sends "lock_double"/"release_double"
    // so the plant model's internal double_locked[] state stays in sync.
    // IsDoubleLocked is an *output* (plant model owns it) and must NOT be in
    // INPUT_SIGNALS — direct writes bypass the plant model and break DoubleLockRelease.
    "Body.Doors.CentralLock.Command",
    // Diagnostic override for the cabin lock status — used by
    // ws_integration tests to simulate "vehicle is locked" without
    // generating a full fob LOCK press.  In production the door-lock
    // arbiter is the only writer; HMI side-channel is for testing.
    "Cabin.LockStatus",
    // Thumb-pad lock inputs — Row 1 outside handle lock areas (HMI top-view).
    "Body.Doors.Row1.Left.Handle.Outside.LockPad.IsPressed",
    "Body.Doors.Row1.Right.Handle.Outside.LockPad.IsPressed",
    // Mirror control switches — momentary press (false→true rising edge).
    // - Fold: toggles intended fold state via the MirrorFold feature.
    // - Select / Direction: defined now so the namespace is reserved;
    //   the Mirror-Adjust feature that consumes these is not implemented
    //   in this PR.
    "Body.Switches.Mirror.Fold",
    "Body.Switches.Mirror.Select",
    "Body.Switches.Mirror.Direction",
    // 3-position interior dome-light switch (String enum OFF/DOOR/ON).
    // Driven by the cockpit HMI; consumed by the DomeSwitch feature.
    "Cabin.Lights.Dome.SwitchPosition",
];

/// Signals the bridge pushes back to the HMI (actuator outputs from arbiters).
const OUTPUT_SIGNALS: &[VssPath] = &[
    "Body.Lights.DirectionIndicator.Left.IsSignaling",
    "Body.Lights.DirectionIndicator.Right.IsSignaling",
    "Body.Lights.Hazard.IsSignaling",
    "Body.Lights.Beam.Low.IsOn",
    "Body.Lights.Beam.High.IsOn",
    "Body.Lights.Parking.IsOn",
    "Body.Lights.Running.IsOn",
    "Body.Lights.Fog.Front.IsOn",
    "Body.Lights.Fog.Rear.IsOn",
    "Body.Lights.Brake.IsActive",
    "Body.Lights.Backup.IsActive",
    "Body.Lights.LicensePlate.IsOn",
    // Exterior puddle lamps — Welcome / Farewell courtesy outputs.
    "Body.Lights.Puddle.Left.IsOn",
    "Body.Lights.Puddle.Right.IsOn",
    // Interior dome / courtesy light — claimed by Welcome via the
    // courtesy_arbiter so the HMI shows the cabin illuminated on
    // approach.
    "Cabin.Lights.IsDomeOn",
    // Mirror fold feedback (per side) — owned by the MirrorFoldPlantModel,
    // published 1 s after each fold/unfold command settles.  HMI reads this
    // for visualization; puddle arbiter's PhysicalGate also reads it.
    "Body.Mirror.Left.IsFolded",
    "Body.Mirror.Right.IsFolded",
    // Mirror tilt / yaw position feedback (i8 -100..100) — owned by
    // MirrorAdjustPlantModel.  HMI shows these as read-only display.
    "Body.Mirror.Left.Tilt",
    "Body.Mirror.Left.Yaw",
    "Body.Mirror.Right.Tilt",
    "Body.Mirror.Right.Yaw",
    // Vehicle-level central lock status (string enum: UNLOCKED |
    // DRIVER_UNLOCKED | LOCKED | DOUBLE_LOCKED) — published by the
    // door-lock arbiter on every accepted command, NVM-persisted.
    "Cabin.LockStatus",
    // Companion signals to Cabin.LockStatus — published by the
    // door-lock arbiter on every accepted command.  AutoRelock
    // subscribes; HMI displays them as a small "last lock event"
    // chip useful for forensics / debugging.
    "Cabin.LockStatus.LastRequestor",
    "Cabin.LockStatus.EventNum",
    "Body.Doors.Row1.Left.IsLocked",
    "Body.Doors.Row1.Right.IsLocked",
    "Body.Doors.Row2.Left.IsLocked",
    "Body.Doors.Row2.Right.IsLocked",
    "Body.Doors.Row1.Left.IsDoubleLocked",
    "Body.Doors.Row1.Right.IsDoubleLocked",
    "Body.Doors.Row2.Left.IsDoubleLocked",
    "Body.Doors.Row2.Right.IsDoubleLocked",
    // Soldier knob state — mirrors central lock actuator (published by DoorLockPlantModel).
    "Body.Doors.Row1.Left.Soldier.IsUnlocked",
    "Body.Doors.Row1.Right.Soldier.IsUnlocked",
    "Body.Doors.Row2.Left.Soldier.IsUnlocked",
    "Body.Doors.Row2.Right.Soldier.IsUnlocked",
    // Door handle plant model outputs — ajar switch and latch state.
    "Body.Doors.Row1.Left.IsOpen",
    "Body.Doors.Row1.Right.IsOpen",
    "Body.Doors.Row2.Left.IsOpen",
    "Body.Doors.Row2.Right.IsOpen",
    "Body.Doors.Row1.Left.Latch.IsLatched",
    "Body.Doors.Row1.Right.Latch.IsLatched",
    "Body.Doors.Row2.Left.Latch.IsLatched",
    "Body.Doors.Row2.Right.Latch.IsLatched",
    // Plant model outputs — actual lamp state from BlinkRelay.
    // Three physical lamps per side: Front, Side (mirror repeater), Rear.
    "Body.Lights.DirectionIndicator.Left.Lamp.Front.IsOn",
    "Body.Lights.DirectionIndicator.Left.Lamp.Side.IsOn",
    "Body.Lights.DirectionIndicator.Left.Lamp.Rear.IsOn",
    "Body.Lights.DirectionIndicator.Right.Lamp.Front.IsOn",
    "Body.Lights.DirectionIndicator.Right.Lamp.Side.IsOn",
    "Body.Lights.DirectionIndicator.Right.Lamp.Rear.IsOn",
    // PEPS plant model outputs — RSSI, challenge responses, RF messages.
    "Body.PEPS.Plant.KeyFob.1.RssiResponse",
    "Body.PEPS.Plant.KeyFob.2.RssiResponse",
    "Body.PEPS.Plant.KeyFob.3.RssiResponse",
    "Body.PEPS.Plant.KeyFob.4.RssiResponse",
    "Body.PEPS.Plant.KeyFob.5.RssiResponse",
    "Body.PEPS.Plant.KeyFob.6.RssiResponse",
    "Body.PEPS.Plant.KeyFob.1.RfMessage",
    "Body.PEPS.Plant.KeyFob.2.RfMessage",
    "Body.PEPS.Plant.KeyFob.3.RfMessage",
    "Body.PEPS.Plant.KeyFob.4.RfMessage",
    "Body.PEPS.Plant.BlePhone.1.RssiResponse",
    "Body.PEPS.Plant.BlePhone.2.RssiResponse",
    // Trunk plant model output — open/close state driven by RKE or CloseCmd.
    "Body.Trunk.IsOpen",
    // Hood plant model outputs — tri-state latch + IsOpen companion.
    "Body.Hood.IsOpen",
    "Body.Hood.LatchState",
    // Sunroof plant model outputs — motor positions, NVM-backed.
    "Body.Sunroof.Position",
    "Body.Sunroof.Shade.Position",
    // Horn arbiter output — drives the HMI horn-pulse visualisation.
    "Body.Horn.IsActive",
    // Interior chime — pulsed by PerimeterAlarm during the 12 s
    // pre-alarm warning phase so the HMI can render a distinct
    // "warning chime" indicator vs the main intrusion-alarm horn.
    "Body.Chime.IsActive",
    // ChimePlantModel output: physical buzzer state.  Mirrors
    // IsActive (intent) but lives on its own signal so the HMI can
    // visualise the actuator with horn-style ripples while features
    // continue writing the intent flag.
    "Body.Chime.IsSounding",
    // Anti-theft alarm status (PanicAlarm direct publish).
    "Vehicle.Body.Alarm.IsActive",
    // Authoritative PerimeterAlarm state enum: "DISARMED" |
    // "PRE_ARMED" | "ARMED" | "ACTIVATED".  HMI subscribes for the
    // status pill + countdown banners.
    "Vehicle.Body.Alarm.State",
    // VSS v4.0 HMI display mode — "DAY" or "NIGHT".  Published by the
    // DayNightModePlant; drives the cockpit view's night-backlit style.
    "Vehicle.Cabin.Infotainment.HMI.DayNightMode",
    // VSS v4.0 per-wheel TPMS low-pressure flags.  Default false at
    // boot — no producer yet (a real TPMS feature will set these).
    // Cockpit aggregates these into a single TPMS warning lamp.
    "Vehicle.Chassis.Axle.Row1.Wheel.Left.Tire.IsPressureLow",
    "Vehicle.Chassis.Axle.Row1.Wheel.Right.Tire.IsPressureLow",
    "Vehicle.Chassis.Axle.Row2.Wheel.Left.Tire.IsPressureLow",
    "Vehicle.Chassis.Axle.Row2.Wheel.Right.Tire.IsPressureLow",
    // Panic switch — set by RKE on PANIC press, cleared by PanicAlarm
    // on cancel-via-unlock.  HMI consumes this to keep its own alarm
    // toggle state in sync.
    "Body.Switches.Panic.IsEngaged",
    // AutoRelock status — drives the HMI countdown banner.  IsArmed
    // is published TRUE when the timer starts, FALSE on every exit.
    // TimeoutSeconds is published once per arm so the HMI can render
    // the matching client-side countdown.
    "Body.Doors.AutoRelock.IsArmed",
    "Body.Doors.AutoRelock.TimeoutSeconds",
];

/// Subset of `OUTPUT_SIGNALS` whose authoritative boot value comes from
/// a plant model.  The bridge waits until each of these has been
/// observed in `output_state` at least once before serving the first
/// snapshot to a freshly-connected HMI client — this guarantees the
/// "first WS message is always a complete, consistent snapshot"
/// contract documented in
/// `docs/signal-ownership-and-state-hydration.md` §5.
///
/// Signals that are normally OFF / inactive at boot (e.g. brake lamp,
/// indicator lamps, horn) are intentionally NOT in this list — they
/// would never tick at boot and would deadlock the gate.  Only owned
/// state that the plant model proactively publishes goes here.
const ESSENTIAL_BOOT_SIGNALS: &[VssPath] = &[
    // Door lock state — DoorLockPlantModel.publish_all() publishes 12
    // values (4×IsLocked + 4×IsDoubleLocked + 4×Soldier.IsUnlocked).
    "Body.Doors.Row1.Left.IsLocked",
    "Body.Doors.Row1.Right.IsLocked",
    "Body.Doors.Row2.Left.IsLocked",
    "Body.Doors.Row2.Right.IsLocked",
    "Body.Doors.Row1.Left.IsDoubleLocked",
    "Body.Doors.Row1.Right.IsDoubleLocked",
    "Body.Doors.Row2.Left.IsDoubleLocked",
    "Body.Doors.Row2.Right.IsDoubleLocked",
    "Body.Doors.Row1.Left.Soldier.IsUnlocked",
    "Body.Doors.Row1.Right.Soldier.IsUnlocked",
    "Body.Doors.Row2.Left.Soldier.IsUnlocked",
    "Body.Doors.Row2.Right.Soldier.IsUnlocked",
    // Trunk plant model — boots from NVM and publishes IsOpen on startup.
    "Body.Trunk.IsOpen",
    // Hood plant model — boots from NVM and publishes both
    // IsOpen + LatchState on startup.
    "Body.Hood.IsOpen",
    "Body.Hood.LatchState",
    // Sunroof plant model — boots from NVM and publishes both Positions on startup.
    "Body.Sunroof.Position",
    "Body.Sunroof.Shade.Position",
    // Status flags published once at boot from main.rs.
    "Body.Switches.Panic.IsEngaged",
    "Vehicle.Body.Alarm.IsActive",
    "Body.Doors.AutoRelock.IsArmed",
];

/// Shared state snapshot sent to HMI clients.
type StateSnapshot = HashMap<&'static str, serde_json::Value>;

pub struct WsBridge<B: SignalBus> {
    bus: Arc<B>,
    addr: SocketAddr,
    platform_config: Arc<PlatformConfig>,
    /// Bumped on `{"type":"reboot"}` from the HMI — `main`'s boot loop
    /// awaits a change and rebuilds the entire simulation stack.
    reboot_tx: tokio::sync::watch::Sender<u64>,
}

impl<B: SignalBus> WsBridge<B> {
    pub fn new(
        addr: SocketAddr,
        bus: Arc<B>,
        platform_config: Arc<PlatformConfig>,
        reboot_tx: tokio::sync::watch::Sender<u64>,
    ) -> Self {
        Self {
            bus,
            addr,
            platform_config,
            reboot_tx,
        }
    }

    /// Run the WebSocket server. Spawns a background task that listens
    /// for output signal changes and broadcasts to all connected HMI clients.
    pub async fn run(self) -> anyhow::Result<()> {
        let listener = TcpListener::bind(self.addr).await?;
        tracing::info!(addr = %self.addr, "WebSocket bridge listening");

        // Shared output state + broadcast channel for pushing updates to clients.
        let output_state: Arc<Mutex<StateSnapshot>> = Arc::new(Mutex::new(HashMap::new()));
        let (update_tx, _) = broadcast::channel::<String>(256);

        // Boot-readiness gate.  Flips TRUE the first time `output_state`
        // contains every signal in ESSENTIAL_BOOT_SIGNALS — guaranteeing
        // that any client that connects after that moment receives a
        // complete, consistent first snapshot.  See
        // docs/signal-ownership-and-state-hydration.md §5.
        let (ready_tx, ready_rx) = tokio::sync::watch::channel::<bool>(false);
        let ready_tx = Arc::new(ready_tx);

        // Coalesce per-signal updates into a single 10 ms batch so that
        // multi-signal publications (e.g. BlinkRelay toggling three lamps
        // on one side in the same tick) arrive at the HMI as a single
        // snapshot. This prevents the "lamps flip one-by-one" visual
        // stagger caused by sending one websocket message per signal.
        let dirty = Arc::new(Notify::new());

        // Subscriber tasks — update shared state and mark dirty; the
        // broadcaster task handles the debounced snapshot send.
        //
        // Track BOTH OUTPUT_SIGNALS (bridge → HMI) and INPUT_SIGNALS
        // (HMI → bridge) so a fresh HMI tab — or one that just
        // reconnected — receives the full current state of everything
        // the user has configured (ignition state, switch positions,
        // fob/phone zones, brake pedal …).  Without this, every reload
        // would reset the HMI to INIT_STATE while the bridge bus still
        // holds the live values.
        //
        // Some signals appear in both lists (e.g. IsOpen, Soldier.IsUnlocked):
        // dedup with a HashSet so we only spawn one subscriber per signal.
        use std::collections::HashSet;
        let mut tracked: HashSet<VssPath> = HashSet::new();
        tracked.extend(OUTPUT_SIGNALS.iter().copied());
        tracked.extend(INPUT_SIGNALS.iter().copied());

        for signal in tracked {
            let bus = Arc::clone(&self.bus);
            let state = Arc::clone(&output_state);
            let dirty = Arc::clone(&dirty);

            tokio::spawn(async move {
                let mut stream = bus.subscribe(signal).await;
                while let Some(val) = stream.next().await {
                    let json_val = signal_value_to_json(&val);
                    {
                        let mut s = state.lock().await;
                        s.insert(signal, json_val);
                    }
                    dirty.notify_one();
                }
            });
        }

        // Broadcaster: waits for a dirty notification, sleeps 10 ms to
        // collect further updates, then sends one coalesced snapshot.
        // Also flips the boot-readiness gate once every essential
        // boot signal has been observed at least once.
        {
            let state = Arc::clone(&output_state);
            let tx = update_tx.clone();
            let dirty = Arc::clone(&dirty);
            let ready_tx = Arc::clone(&ready_tx);
            tokio::spawn(async move {
                const BATCH_WINDOW: Duration = Duration::from_millis(10);
                loop {
                    dirty.notified().await;
                    // Collect any further notifications that arrive
                    // within the batch window.
                    sleep(BATCH_WINDOW).await;
                    let snapshot = {
                        let s = state.lock().await;
                        s.clone()
                    };
                    // Boot-readiness check — only flips once, watch
                    // dedup makes a redundant `send(true)` a no-op.
                    if !*ready_tx.borrow() {
                        let all_essential_present = ESSENTIAL_BOOT_SIGNALS
                            .iter()
                            .all(|s| snapshot.contains_key(s));
                        if all_essential_present {
                            tracing::info!(
                                essentials = ESSENTIAL_BOOT_SIGNALS.len(),
                                "WS bridge: boot-ready (all essential signals present)"
                            );
                            let _ = ready_tx.send(true);
                        }
                    }
                    let msg = serde_json::json!({ "state": snapshot });
                    let _ = tx.send(msg.to_string());
                }
            });
        }

        // Config broadcast channel — separate from signal-state so config HMI
        // can subscribe without receiving every signal update.
        let (config_tx, _) = broadcast::channel::<String>(32);

        // Accept connections
        loop {
            let (stream, peer) = listener.accept().await?;
            tracing::info!(%peer, "HMI client connecting");

            let bus = Arc::clone(&self.bus);
            let output_state = Arc::clone(&output_state);
            let update_rx = update_tx.subscribe();
            let config_rx = config_tx.subscribe();
            let platform_config = Arc::clone(&self.platform_config);
            let config_tx2 = config_tx.clone();
            let ready_rx = ready_rx.clone();
            let reboot_tx = self.reboot_tx.clone();

            tokio::spawn(async move {
                if let Err(e) = handle_connection(
                    stream,
                    bus,
                    output_state,
                    update_rx,
                    config_rx,
                    config_tx2,
                    platform_config,
                    peer,
                    ready_rx,
                    reboot_tx,
                )
                .await
                {
                    tracing::warn!(%peer, error = %e, "HMI client disconnected");
                }
            });
        }
    }
}

#[allow(clippy::too_many_arguments)]
async fn handle_connection<B: SignalBus>(
    stream: TcpStream,
    bus: Arc<B>,
    output_state: Arc<Mutex<StateSnapshot>>,
    mut update_rx: broadcast::Receiver<String>,
    mut config_rx: broadcast::Receiver<String>,
    config_tx: broadcast::Sender<String>,
    platform_config: Arc<PlatformConfig>,
    peer: SocketAddr,
    mut ready_rx: tokio::sync::watch::Receiver<bool>,
    reboot_tx: tokio::sync::watch::Sender<u64>,
) -> anyhow::Result<()> {
    // Each connection subscribes to the reboot watch channel so it can
    // self-close cleanly when `main`'s boot loop tears down the
    // simulation stack.  Per-connection handlers are spawned outside
    // the JoinSet that holds the rest of the stack, so without this
    // explicit cooperation they would survive the reboot, hold stale
    // Bus / PlatformConfig references, and prevent the HMI from
    // re-handshaking against the new boot's listener.
    let mut reboot_watch = reboot_tx.subscribe();
    // Mark the value we joined on as "seen" so .changed() only fires
    // on subsequent bumps.
    let _ = reboot_watch.borrow_and_update();
    let ws_stream = tokio_tungstenite::accept_async(stream).await?;
    let (mut ws_tx, mut ws_rx) = ws_stream.split();

    tracing::info!(%peer, "HMI client connected");

    // Wait for boot-readiness before sending the first snapshot.  This
    // guarantees that every signal in ESSENTIAL_BOOT_SIGNALS is already
    // populated in `output_state` — the HMI's first message is then
    // always complete and consistent.  See
    // docs/signal-ownership-and-state-hydration.md §5.
    if !*ready_rx.borrow() {
        tracing::debug!(%peer, "HMI client waiting for bridge boot-readiness");
        // Watch::changed() resolves when the value changes since last
        // observation; combined with the `*borrow() == false` check
        // above it skips the wait when we're already ready.
        let _ = ready_rx.changed().await;
    }

    // Send current signal state immediately on connect.
    {
        let snapshot = output_state.lock().await.clone();
        if !snapshot.is_empty() {
            let msg = serde_json::json!({ "state": snapshot });
            ws_tx.send(Message::Text(msg.to_string().into())).await?;
        }
    }

    // Send current config state immediately on connect.
    {
        let cfg_msg = build_config_msg(&platform_config);
        ws_tx.send(Message::Text(cfg_msg.into())).await?;
    }

    loop {
        tokio::select! {
            // Reboot signalled — close this connection so `main`'s
            // boot-loop teardown is observed by the HMI as a clean
            // disconnect.  The HMI's onclose reconnect timer fires
            // and lands on the next iteration's listener.
            Ok(_) = reboot_watch.changed() => {
                tracing::info!(%peer, "reboot signalled — closing client connection");
                break;
            }
            // HMI → bridge: sensor or config input
            msg = ws_rx.next() => {
                match msg {
                    Some(Ok(Message::Text(text))) => {
                        let parsed: serde_json::Value = match serde_json::from_str(&text) {
                            Ok(v) => v,
                            Err(_) => { continue; }
                        };
                        match parsed.get("type").and_then(|v| v.as_str()) {
                            Some("config_set") => {
                                if handle_config_set(&parsed, &platform_config) {
                                    // Broadcast updated config to all connected HMIs.
                                    let cfg_msg = build_config_msg(&platform_config);
                                    let _ = config_tx.send(cfg_msg);
                                }
                            }
                            Some("reboot") => {
                                // Persist any staged vehicle_line / variant
                                // edits to disk, then bump the reboot signal
                                // so `main`'s boot loop tears down the
                                // simulation stack and rebuilds against the
                                // freshly-loaded config.  NVM persists; all
                                // RAM state is dropped — mimics a real ECU
                                // power cycle (battery disconnect/reconnect).
                                handle_reboot(&parsed, &reboot_tx, peer).await;
                                // Don't break here — the JoinSet abort that
                                // follows will drop this task naturally.
                            }
                            _ => {
                                handle_hmi_message(&text, &bus).await;
                            }
                        }
                    }
                    Some(Ok(Message::Close(_))) | None => {
                        tracing::info!(%peer, "HMI client disconnected");
                        break;
                    }
                    Some(Err(e)) => {
                        tracing::warn!(%peer, error = %e, "WebSocket read error");
                        break;
                    }
                    _ => {}
                }
            }
            // bridge → HMI: signal state update
            Ok(json_str) = update_rx.recv() => {
                if ws_tx.send(Message::Text(json_str.into())).await.is_err() {
                    break;
                }
            }
            // bridge → HMI: config update (triggered by another client or M7)
            Ok(cfg_str) = config_rx.recv() => {
                if ws_tx.send(Message::Text(cfg_str.into())).await.is_err() {
                    break;
                }
            }
        }
    }

    Ok(())
}

/// Serialize current platform config into a `{"config":{...}}` JSON string.
fn build_config_msg(cfg: &PlatformConfig) -> String {
    let dealer = cfg.dealer_config();
    let variant = cfg.variant_cal();
    let vl = &cfg.vehicle_line;

    let msg = serde_json::json!({
        "config": {
            "dealer": {
                "auto_relock_enabled":        dealer.auto_relock_enabled,
                "horn_chirp_on_lock":         dealer.horn_chirp_on_lock,
                "courtesy_light_timeout_secs":dealer.courtesy_light_timeout_secs,
                "remote_start_max_minutes":   dealer.remote_start_max_minutes,
                "two_stage_unlock":           dealer.two_stage_unlock,
                "driver_door_side":           format!("{:?}", dealer.driver_door_side),
                "mirror_fold_mode":           format!("{:?}", dealer.mirror_fold_mode).to_uppercase(),
                "farewell_hold_secs":         dealer.farewell_hold_secs,
            },
            "variant": {
                "double_lock_enabled":  variant.double_lock_enabled,
                "nfc_enabled":          variant.nfc_enabled,
                "ble_key_enabled":      variant.ble_key_enabled,
                "remote_lock_enabled":  variant.remote_lock_enabled,
                "auto_lock_speed_kmh":  variant.auto_lock_speed_kmh,
                "welcome_light_pattern":format!("{:?}", variant.welcome_light_pattern),
                "doors_row2_left":      variant.doors.row2_left,
                "doors_row2_right":     variant.doors.row2_right,
                "doors_removable":      variant.doors.removable,
            },
            "vehicle_line": {
                "auto_relock_timeout_secs":    vl.auto_relock_timeout_secs,
                "lock_feedback_blink_count":   vl.lock_feedback_blink_count,
                "lock_feedback_blink_period_ms":vl.lock_feedback_blink_period_ms,
                "welcome_light_duration_secs": vl.welcome_light_duration_secs,
                "lane_change_flash_count":     vl.lane_change_flash_count,
                "shutdown_grace_secs":         vl.shutdown_grace_secs,
                "peps_rear_capacitive_handles":vl.peps_rear_capacitive_handles,
                "panic_press_mode":           format!("{:?}", vl.panic_press_mode).to_uppercase(),
                "slam_lock_protect":          vl.slam_lock_protect,
            }
        }
    });
    msg.to_string()
}

/// Apply a `config_set` message to PlatformConfig.
/// Returns `true` if the config was changed (triggers broadcast).
fn handle_config_set(msg: &serde_json::Value, cfg: &PlatformConfig) -> bool {
    use crate::config::DriverDoorSide;

    let key = msg.get("key").and_then(|v| v.as_str()).unwrap_or("");
    let value = match msg.get("value") {
        Some(v) => v,
        None => return false,
    };

    tracing::debug!(key, "config_set received");

    // ── Dealer config ─────────────────────────────────────────────────────
    let mut dealer = cfg.dealer_config();
    let dealer_changed = match key {
        "dealer.auto_relock_enabled" => {
            if let Some(b) = value.as_bool() {
                dealer.auto_relock_enabled = b;
                true
            } else {
                false
            }
        }
        "dealer.horn_chirp_on_lock" => {
            if let Some(b) = value.as_bool() {
                dealer.horn_chirp_on_lock = b;
                true
            } else {
                false
            }
        }
        "dealer.two_stage_unlock" => {
            if let Some(b) = value.as_bool() {
                dealer.two_stage_unlock = b;
                true
            } else {
                false
            }
        }
        "dealer.courtesy_light_timeout_secs" => {
            if let Some(n) = value.as_u64() {
                dealer.courtesy_light_timeout_secs = n;
                true
            } else {
                false
            }
        }
        "dealer.remote_start_max_minutes" => {
            if let Some(n) = value.as_u64() {
                dealer.remote_start_max_minutes = n;
                true
            } else {
                false
            }
        }
        "dealer.driver_door_side" => {
            dealer.driver_door_side = match value.as_str() {
                Some("Right") => DriverDoorSide::Right,
                _ => DriverDoorSide::Left,
            };
            true
        }
        "dealer.mirror_fold_mode" => {
            use crate::config::MirrorFoldMode;
            dealer.mirror_fold_mode = match value.as_str() {
                Some("AUTO") => MirrorFoldMode::Auto,
                Some("OFF") => MirrorFoldMode::Off,
                _ => MirrorFoldMode::Manual,
            };
            true
        }
        _ => false,
    };
    if dealer_changed {
        cfg.update_dealer_config(dealer);
        return true;
    }

    // ── Variant config ────────────────────────────────────────────────────
    let mut variant = cfg.variant_cal();
    let variant_changed = match key {
        "variant.double_lock_enabled" => {
            if let Some(b) = value.as_bool() {
                variant.double_lock_enabled = b;
                true
            } else {
                false
            }
        }
        "variant.nfc_enabled" => {
            if let Some(b) = value.as_bool() {
                variant.nfc_enabled = b;
                true
            } else {
                false
            }
        }
        "variant.ble_key_enabled" => {
            if let Some(b) = value.as_bool() {
                variant.ble_key_enabled = b;
                true
            } else {
                false
            }
        }
        "variant.remote_lock_enabled" => {
            if let Some(b) = value.as_bool() {
                variant.remote_lock_enabled = b;
                true
            } else {
                false
            }
        }
        "variant.auto_lock_speed_kmh" => {
            if let Some(n) = value.as_u64() {
                variant.auto_lock_speed_kmh = n as u16;
                true
            } else {
                false
            }
        }
        "variant.doors_row2_left" => {
            if let Some(b) = value.as_bool() {
                variant.doors.row2_left = b;
                true
            } else {
                false
            }
        }
        "variant.doors_row2_right" => {
            if let Some(b) = value.as_bool() {
                variant.doors.row2_right = b;
                true
            } else {
                false
            }
        }
        "variant.doors_removable" => {
            if let Some(b) = value.as_bool() {
                variant.doors.removable = b;
                true
            } else {
                false
            }
        }
        "variant.welcome_light_pattern" => {
            use crate::config::WelcomeLightPattern;
            variant.welcome_light_pattern = match value.as_str() {
                Some("Sequential") => WelcomeLightPattern::Sequential,
                Some("Disabled") => WelcomeLightPattern::Disabled,
                _ => WelcomeLightPattern::Simple,
            };
            true
        }
        _ => false,
    };
    if variant_changed {
        cfg.update_variant_cal(variant);
        return true;
    }

    tracing::warn!(key, "config_set: unknown key");
    false
}

/// Handle a `{"type":"reboot", "vehicle_line": {...}, "variant": {...}}`
/// message.  Persists any included vehicle_line / variant cal edits
/// to disk, then bumps the reboot watch channel which `main`'s boot
/// loop is waiting on.  The boot loop tears down the entire
/// simulation stack (this connection included) and rebuilds against
/// the freshly-loaded config.
///
/// Both `vehicle_line` and `variant` are optional — the HMI sends
/// only the staged sections.  Missing or malformed entries log a
/// warning but do not block the reboot itself; the user's intent
/// to "cycle power" is honoured even if a config write fails (so a
/// stuck on-disk file doesn't prevent recovery).
async fn handle_reboot(
    msg: &serde_json::Value,
    reboot_tx: &tokio::sync::watch::Sender<u64>,
    peer: SocketAddr,
) {
    if let Some(vl) = msg.get("vehicle_line") {
        match serde_json::from_value::<crate::config::VehicleLineCal>(vl.clone()) {
            Ok(parsed) => match PlatformConfig::save_vehicle_line(&parsed) {
                Ok(path) => tracing::info!(%peer, path = %path.display(),
                    "reboot: persisted vehicle_line.json"),
                Err(e) => tracing::warn!(%peer, error = %e,
                    "reboot: failed to persist vehicle_line.json — continuing anyway"),
            },
            Err(e) => tracing::warn!(%peer, error = %e,
                "reboot: vehicle_line payload did not parse — skipping"),
        }
    }
    if let Some(v) = msg.get("variant") {
        match serde_json::from_value::<crate::config::VariantCal>(v.clone()) {
            Ok(parsed) => match PlatformConfig::save_variant(&parsed) {
                Ok(path) => tracing::info!(%peer, path = %path.display(),
                    "reboot: persisted variant.json"),
                Err(e) => tracing::warn!(%peer, error = %e,
                    "reboot: failed to persist variant.json — continuing anyway"),
            },
            Err(e) => tracing::warn!(%peer, error = %e,
                "reboot: variant payload did not parse — skipping"),
        }
    }

    // Bump the reboot counter — the boot loop in `main` is awaiting
    // a change.  After this returns, the JoinSet abort that follows
    // will drop this connection along with everything else.
    let new = reboot_tx.borrow().wrapping_add(1);
    tracing::info!(%peer, new_boot_id = new, "reboot: signalling main boot loop");
    let _ = reboot_tx.send(new);
}

/// Parse an HMI sensor message and inject into the SignalBus.
async fn handle_hmi_message<B: SignalBus>(text: &str, bus: &Arc<B>) {
    let parsed: serde_json::Value = match serde_json::from_str(text) {
        Ok(v) => v,
        Err(e) => {
            tracing::warn!(error = %e, "HMI: invalid JSON");
            return;
        }
    };

    // Expected: {"type":"sensor","path":"...","value":...}
    let msg_type = parsed.get("type").and_then(|v| v.as_str());
    if msg_type != Some("sensor") {
        tracing::debug!(?msg_type, "HMI: ignoring non-sensor message");
        return;
    }

    let path = match parsed.get("path").and_then(|v| v.as_str()) {
        Some(p) => p,
        None => {
            tracing::warn!("HMI: missing 'path' field");
            return;
        }
    };

    let value_json = match parsed.get("value") {
        Some(v) => v,
        None => {
            tracing::warn!("HMI: missing 'value' field");
            return;
        }
    };

    // Validate this is a known input signal.
    let static_path: VssPath = match INPUT_SIGNALS.iter().find(|&&s| s == path) {
        Some(&p) => p,
        None => {
            tracing::warn!(path, "HMI: unknown input signal, ignoring");
            return;
        }
    };

    let signal_value = json_to_signal_value(value_json);

    tracing::debug!(path = static_path, value = ?signal_value, "HMI → bus");

    // Inject into the bus (simulates a physical sensor input).
    // For MockBus, use inject(). For real buses, use publish().
    // Since we use MockBus's inject semantics through publish:
    if let Err(e) = bus.publish(static_path, signal_value).await {
        tracing::error!(path = static_path, error = %e, "HMI: failed to inject signal");
    }
}

/// Convert a SignalValue to a serde_json::Value.
fn signal_value_to_json(val: &SignalValue) -> serde_json::Value {
    match val {
        SignalValue::Bool(b) => serde_json::Value::Bool(*b),
        SignalValue::Uint8(n) => serde_json::json!(*n),
        SignalValue::Int16(n) => serde_json::json!(*n),
        SignalValue::Uint16(n) => serde_json::json!(*n),
        SignalValue::Float(f) => serde_json::json!(*f),
        SignalValue::String(s) => serde_json::Value::String(s.clone()),
    }
}

/// Convert a serde_json::Value to a SignalValue (best-effort type inference).
fn json_to_signal_value(val: &serde_json::Value) -> SignalValue {
    match val {
        serde_json::Value::Bool(b) => SignalValue::Bool(*b),
        serde_json::Value::String(s) => SignalValue::String(s.clone()),
        serde_json::Value::Number(n) => {
            if let Some(f) = n.as_f64() {
                if f.fract() == 0.0 && (0.0..=255.0).contains(&f) {
                    SignalValue::Uint8(f as u8)
                } else if f.fract() == 0.0 && (256.0..=65535.0).contains(&f) {
                    SignalValue::Uint16(f as u16)
                } else if f.fract() == 0.0 {
                    SignalValue::Int16(f.clamp(-32768.0, 32767.0) as i16)
                } else {
                    SignalValue::Float(f as f32)
                }
            } else {
                SignalValue::Uint8(0)
            }
        }
        _ => SignalValue::Bool(false),
    }
}
