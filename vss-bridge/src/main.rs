//! vss-bridge — COVESA VSS body controller bridge (L5 application layer).
//!
//! This binary runs on the A53 cluster under Android Automotive OS.
//! It bridges the Safety Monitor (M7, RPmsg) with kuksa.val (gRPC)
//! and the Web HMI (WebSocket).
//!
//! # Boot loop / in-process reboot
//!
//! `main` is a *boot loop*.  Each iteration constructs a fresh
//! `MockBus`, loads `PlatformConfig` from disk, and spawns every
//! arbiter, feature, plant model, and the WebSocket bridge into a
//! single `JoinSet`.  When the HMI sends `{"type":"reboot"}` (after
//! persisting any edited cals), the bridge bumps a `tokio::sync::watch`
//! counter; the boot loop sees the change, aborts the entire JoinSet,
//! drops the bus, and starts the next iteration with the freshly
//! re-loaded config.  NVM persists on disk across the cycle, mimicking
//! a real ECU power cycle.
//!
//! Things that DO survive a reboot:
//!   • the HTTP file server (so `http://localhost:3000/vss-hmi.html`
//!     stays loadable mid-reboot)
//!   • on-disk state — NVM (`door_lock.json`, `trunk.json`, …) and
//!     the calibration files themselves
//!
//! Things that DO NOT survive:
//!   • everything in RAM — bus latest-cache, history, every feature's
//!     internal state machines, every arbiter's claim table
//!   • all WebSocket client connections (HMI auto-reconnects on the
//!     next iteration's listener)

use std::sync::Arc;
use tokio::task::JoinSet;
use tracing_subscriber::EnvFilter;

use axum::Router;
use tower_http::services::ServeDir;
use tower_http::set_header::SetResponseHeaderLayer;

use vss_bridge::adapters::mock::MockBus;
use vss_bridge::arbiter;
use vss_bridge::config::PlatformConfig;
use vss_bridge::features::auto_high_beam::AutoHighBeam;
use vss_bridge::features::auto_relock::AutoRelock;
use vss_bridge::features::brake_reverse_lamps::BrakeReverseLamps;
use vss_bridge::features::cabin_trunk_release::CabinTrunkRelease;
use vss_bridge::features::dome_switch::DomeSwitch;
use vss_bridge::features::door_open_assist::DoorOpenAssist;
use vss_bridge::features::door_trim_button::DoorTrimButton;
use vss_bridge::features::double_lock_release::DoubleLockRelease;
use vss_bridge::features::exterior_trunk_button::ExteriorTrunkButton;
use vss_bridge::features::farewell::Farewell;
use vss_bridge::features::fog_lamps::FogLamps;
use vss_bridge::features::follow_me_home::FollowMeHome;
use vss_bridge::features::hazard_lighting::HazardLighting;
use vss_bridge::features::lock_feedback::LockFeedback;
use vss_bridge::features::manual_horn::ManualHorn;
use vss_bridge::features::manual_lighting::ManualLighting;
use vss_bridge::features::mirror_adjust::MirrorAdjust;
use vss_bridge::features::mirror_fold::MirrorFold;
use vss_bridge::features::panic_alarm::PanicAlarm;
use vss_bridge::features::passive_entry::{DeviceKind, PairedDevice, PassiveEntry};
use vss_bridge::features::perimeter_alarm::PerimeterAlarm;
use vss_bridge::features::power_child_lock::PowerChildLock;
use vss_bridge::features::power_window::PowerWindow;
use vss_bridge::features::rke::{PairedFob, RkeFeature};
use vss_bridge::features::slam_lock::SlamLock;
use vss_bridge::features::sunroof_control::SunroofControl;
use vss_bridge::features::thumb_pad_lock::ThumbPadLock;
use vss_bridge::features::turn_indicator::TurnIndicator;
use vss_bridge::features::walk_away_lock::WalkAwayLock;
use vss_bridge::features::welcome::Welcome;
use vss_bridge::ipc_message::SignalValue;
use vss_bridge::kuksa_sync;
use vss_bridge::nvm::NvmStore;
use vss_bridge::plant_models::blink_relay::BlinkRelay;
use vss_bridge::plant_models::chime::ChimePlantModel;
use vss_bridge::plant_models::day_night_mode::DayNightModePlant;
use vss_bridge::plant_models::door_handle::DoorHandlePlantModel;
use vss_bridge::plant_models::door_lock::DoorLockPlantModel;
use vss_bridge::plant_models::hood::HoodPlantModel;
use vss_bridge::plant_models::mirror_adjust::MirrorAdjustPlantModel;
use vss_bridge::plant_models::mirror_fold::MirrorFoldPlantModel;
use vss_bridge::plant_models::peps::PepsPlantModel;
use vss_bridge::plant_models::sunroof::SunroofPlantModel;
use vss_bridge::plant_models::transmission::TransmissionPlant;
use vss_bridge::plant_models::trunk::TrunkPlantModel;
use vss_bridge::plant_models::window::WindowPlant;
use vss_bridge::signal_bus::SignalBus;
use vss_bridge::signal_ids;
use vss_bridge::ws_bridge::WsBridge;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();

    // ── CLI flags ──────────────────────────────────────────────────
    let args: Vec<String> = std::env::args().collect();
    let reset_nvm = args.iter().any(|a| a == "--reset-nvm");

    // ── Dev-default config dir ─────────────────────────────────────
    // Production builds resolve `VSS_BRIDGE_CONFIG_PATH` from the env
    // (set by the systemd unit / yocto image to `/etc/vss-bridge`).
    // For `cargo run` on a developer machine the env var is usually
    // unset, and `/etc/vss-bridge` is not writable without root — so
    // reboot-driven cal edits silently fail with "Permission denied"
    // and the next boot loads defaults instead of the user's staged
    // values.  Fall back to the in-repo `config/` directory so the
    // HMI's Apply & Reboot round-trips cleanly out of the box.
    if std::env::var("VSS_BRIDGE_CONFIG_PATH").is_err() {
        let dev_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("config");
        // Unsafe in multi-threaded contexts on some platforms; this
        // runs before tokio spawns any workers so it's fine here.
        unsafe {
            std::env::set_var("VSS_BRIDGE_CONFIG_PATH", &dev_path);
        }
        tracing::info!(
            path = %dev_path.display(),
            "VSS_BRIDGE_CONFIG_PATH unset — defaulting to in-repo config/"
        );
    }

    tracing::info!(reset_nvm, "vss-bridge starting");

    // ── NVM store (survives reboots) ───────────────────────────────
    let nvm = NvmStore::from_env();
    if reset_nvm {
        nvm.reset();
    }

    // ── Reboot signal — incremented by ws_bridge on `{type:reboot}` ─
    // The boot loop awaits a change to this counter and rebuilds the
    // entire simulation stack against fresh config + a fresh bus.
    let (reboot_tx, mut reboot_rx) = tokio::sync::watch::channel(0u64);

    // ── HTTP file server (kept alive across reboots) ───────────────
    let http_port: u16 = std::env::var("VSS_BRIDGE_HTTP_PORT")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(3000);
    let hmi_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("repo root")
        .to_path_buf();
    let http_app = Router::new()
        .nest_service("/", ServeDir::new(hmi_root))
        .layer(SetResponseHeaderLayer::overriding(
            axum::http::header::CACHE_CONTROL,
            axum::http::HeaderValue::from_static("no-store"),
        ));
    let http_listener = tokio::net::TcpListener::bind(format!("0.0.0.0:{http_port}")).await?;
    tokio::spawn(async move {
        if let Err(e) = axum::serve(http_listener, http_app).await {
            tracing::error!(error = %e, "HTTP file server failed");
        }
    });

    // ── ctrl_c trigger — gracefully exit the boot loop ─────────────
    let (shutdown_tx, mut shutdown_rx) = tokio::sync::watch::channel(false);
    tokio::spawn(async move {
        if tokio::signal::ctrl_c().await.is_ok() {
            tracing::info!("ctrl_c received — exiting boot loop");
            let _ = shutdown_tx.send(true);
        }
    });

    // ── Boot loop ──────────────────────────────────────────────────
    let mut boot_id: u64 = 0;
    loop {
        boot_id += 1;
        tracing::info!(boot_id, "vss-bridge boot");

        let cfg = PlatformConfig::load();
        let bus = Arc::new(MockBus::new());

        tracing::info!(
            signals = signal_ids::ALL_SIGNALS.len(),
            "signal catalog loaded"
        );

        let mut stack = boot_simulation_stack(
            Arc::clone(&bus),
            nvm.clone(),
            Arc::clone(&cfg),
            reboot_tx.clone(),
        )
        .await?;

        if boot_id == 1 {
            tracing::info!("vss-bridge ready");
            tracing::info!("  HMI:             http://localhost:{http_port}/vss-hmi.html");
            tracing::info!("  WebSocket:       ws://localhost:8080");
            tracing::info!("  Press Ctrl+C to stop");
        }

        // Wait until either:
        //   (a) reboot_rx ticks past `boot_id` (HMI requested reboot)
        //   (b) shutdown_rx flips true (ctrl_c)
        //   (c) the JoinSet drains (catastrophic — every task ended)
        loop {
            tokio::select! {
                Ok(_) = reboot_rx.changed() => {
                    let new = *reboot_rx.borrow();
                    if new >= boot_id {
                        tracing::info!(boot_id, new, "reboot signal received — tearing down stack");
                        break;
                    }
                }
                Ok(_) = shutdown_rx.changed() => {
                    if *shutdown_rx.borrow() {
                        tracing::info!("shutdown received — tearing down stack");
                        stack.abort_all();
                        while stack.join_next().await.is_some() {}
                        return Ok(());
                    }
                }
                Some(res) = stack.join_next() => {
                    if let Err(e) = res {
                        if !e.is_cancelled() {
                            tracing::warn!(error = %e, "task in simulation stack ended unexpectedly");
                        }
                    }
                }
                else => break,
            }
        }

        // Tear down the current stack.
        stack.abort_all();
        while stack.join_next().await.is_some() {}
        bus.reset();
        tracing::info!(boot_id, "stack torn down — looping for fresh boot");
    }
}

/// Build and spawn the entire simulation stack (arbiters, features,
/// plant models, WebSocket bridge) into a single `JoinSet`.  Returns
/// the populated set; the caller drops/aborts it to tear everything
/// down for an in-process reboot.
async fn boot_simulation_stack(
    bus: Arc<MockBus>,
    nvm: NvmStore,
    cfg: Arc<PlatformConfig>,
    reboot_tx: tokio::sync::watch::Sender<u64>,
) -> anyhow::Result<JoinSet<()>> {
    let mut set: JoinSet<()> = JoinSet::new();

    // ── Domain Arbiters ─────────────────────────────────────────────
    let (lighting_arb, lighting_fut) = arbiter::lighting_arbiter(Arc::clone(&bus));
    let (low_beam_arb, low_beam_fut) = arbiter::low_beam_arbiter(Arc::clone(&bus));
    let (door_lock_arb, door_lock_ack_tx, door_lock_fut) =
        arbiter::door_lock_arbiter_with_nvm(Arc::clone(&bus), nvm.clone());
    let (horn_arb, horn_fut) = arbiter::horn_arbiter(Arc::clone(&bus));
    let (_comfort_arb, comfort_fut) = arbiter::comfort_arbiter(Arc::clone(&bus));
    let (courtesy_arb, courtesy_fut) = arbiter::courtesy_arbiter(Arc::clone(&bus));
    let (puddle_arb, puddle_fut) = arbiter::puddle_arbiter(Arc::clone(&bus));
    let (trunk_arb, trunk_fut) = arbiter::trunk_arbiter(Arc::clone(&bus));
    let (window_arb, window_fut) = arbiter::window_arbiter(Arc::clone(&bus));

    set.spawn(lighting_fut);
    set.spawn(low_beam_fut);
    set.spawn(door_lock_fut);
    set.spawn(horn_fut);
    set.spawn(comfort_fut);
    set.spawn(courtesy_fut);
    set.spawn(puddle_fut);
    set.spawn(trunk_fut);
    set.spawn(window_fut);

    let lighting_arb = Arc::new(lighting_arb);
    let low_beam_arb = Arc::new(low_beam_arb);
    let door_lock_arb = Arc::new(door_lock_arb);
    let horn_arb = Arc::new(horn_arb);
    let courtesy_arb = Arc::new(courtesy_arb);
    let puddle_arb = Arc::new(puddle_arb);
    let trunk_arb = Arc::new(trunk_arb);
    let window_arb = Arc::new(window_arb);

    // ── Feature Business Logic ──────────────────────────────────────
    let lux_threshold = cfg.vehicle_line.auto_headlamp_lux_threshold;

    set.spawn(
        ManualLighting::new(Arc::clone(&low_beam_arb), Arc::clone(&bus), lux_threshold).run(),
    );
    set.spawn(FollowMeHome::new(Arc::clone(&low_beam_arb), Arc::clone(&bus), lux_threshold).run());
    set.spawn(HazardLighting::new(Arc::clone(&lighting_arb), Arc::clone(&bus)).run());
    set.spawn(
        TurnIndicator::with_config(
            Arc::clone(&lighting_arb),
            Arc::clone(&bus),
            Arc::clone(&cfg),
        )
        .run(),
    );

    // RKE — paired fobs match the PEPS plant model's default secrets.
    let rke_fobs: Vec<PairedFob> = (1u8..=4)
        .map(|i| {
            const DEVICE_TYPE: u8 = b'F';
            let mut key = [0u8; 16];
            key[0] = DEVICE_TYPE;
            key[1] = i;
            for (k, byte) in key.iter_mut().enumerate().skip(2) {
                *byte = (DEVICE_TYPE.wrapping_mul(17))
                    .wrapping_add(i.wrapping_mul(31).wrapping_add(k as u8));
            }
            PairedFob::new(i as u32, key)
        })
        .collect();

    set.spawn(
        RkeFeature::new(
            Arc::clone(&bus),
            Arc::clone(&door_lock_arb),
            Arc::clone(&trunk_arb),
            Arc::clone(&cfg),
            rke_fobs,
        )
        .run(),
    );
    set.spawn(
        LockFeedback::new(Arc::clone(&bus), Arc::clone(&lighting_arb))
            .with_cfg(Arc::clone(&cfg))
            .run(),
    );
    set.spawn(DoubleLockRelease::new(Arc::clone(&bus), Arc::clone(&door_lock_arb)).run());
    set.spawn(WalkAwayLock::new(Arc::clone(&bus), Arc::clone(&door_lock_arb)).run());
    set.spawn(ThumbPadLock::new(Arc::clone(&bus), Arc::clone(&door_lock_arb)).run());
    // Interior trim Lock / Unlock buttons on Row 1 doors.  No auth —
    // occupant-operated; unlock works even with the alarm armed (egress
    // safety) but PerimeterAlarm escalates on the resulting unlock
    // event when LastRequestor = DoorTrimButton during the armed window.
    set.spawn(
        DoorTrimButton::new(
            Arc::clone(&bus),
            Arc::clone(&door_lock_arb),
            Arc::clone(&cfg),
        )
        .run(),
    );
    // SlamLock — slam-lock-protect inversion for EU vehicle lines.
    // No-op on US lines (cfg.vehicle_line.slam_lock_protect = false).
    set.spawn(
        SlamLock::new(
            Arc::clone(&bus),
            Arc::clone(&door_lock_arb),
            Arc::clone(&cfg),
        )
        .run(),
    );
    set.spawn(AutoHighBeam::new(Arc::clone(&low_beam_arb), Arc::clone(&bus)).run());
    set.spawn(BrakeReverseLamps::new(Arc::clone(&bus)).run());
    set.spawn(FogLamps::new(Arc::clone(&bus)).run());
    set.spawn(
        PanicAlarm::new(
            Arc::clone(&lighting_arb),
            Arc::clone(&horn_arb),
            Arc::clone(&bus),
        )
        .run(),
    );
    // PerimeterAlarm — anti-intrusion alarm.  Arms when any door
    // opens while the cabin is LOCKED / DOUBLE_LOCKED.  Disarms on
    // an authenticated unlock (fob/PEPS/phone/NFC) or panic-button
    // press.  Pulses horn for 30 s, lights/dome/puddle for 5 min.
    set.spawn(
        PerimeterAlarm::new(
            Arc::clone(&bus),
            Arc::clone(&lighting_arb),
            Arc::clone(&horn_arb),
            Arc::clone(&courtesy_arb),
            Arc::clone(&puddle_arb),
        )
        .run(),
    );
    set.spawn(AutoRelock::from_config(Arc::clone(&door_lock_arb), Arc::clone(&bus), &cfg).run());

    // DomeSwitch — 3-position interior dome-light switch (OFF / DOOR /
    // ON).  Owns the Low-priority default claim on
    // Cabin.Lights.IsDomeOn; Welcome / Farewell / PerimeterAlarm
    // pre-empt cleanly via the courtesy arbiter.
    set.spawn(DomeSwitch::new(Arc::clone(&bus), Arc::clone(&courtesy_arb)).run());

    // TransmissionPlant — mirrors driver's SelectedGear into the
    // actual engaged CurrentGear.  Stands in for the TCU on dev hosts;
    // future extensions will add brake interlock + speed-based shift
    // logic.  Single writer of CurrentGear.
    set.spawn(TransmissionPlant::new(Arc::clone(&bus)).run());

    // PowerChildLock — single momentary push toggles the master
    // child-lock state and fans out to both rear-door
    // IsChildLockActive outputs.  Door-handle plant + PowerWindow
    // observe these to gate inside pulls and local rear window
    // switches respectively.  Door-side mechanical feedback is TBD.
    set.spawn(PowerChildLock::new(Arc::clone(&bus)).run());

    // PowerWindow — combined driver-master + local 5-detent rocker
    // controller for all 4 windows.  Handles cross-source conflicts
    // internally (both active → motor STOPPED, both must re-press)
    // and runs a 5 s stuck-switch watchdog per source per window.
    // Claims the window arbiter at Medium for the single resolved
    // motor direction.  Future anti-pinch (Critical) and security
    // override (High) can pre-empt via the arbiter.
    set.spawn(PowerWindow::new(Arc::clone(&bus), Arc::clone(&window_arb)).run());

    // SunroofControl — overhead-console rocker → coordinated roof +
    // shade motors.  Sequencing: shade opens first then roof; roof
    // closes first then shade.  Auto-mode latching with cancel-on-press.
    set.spawn(SunroofControl::new(Arc::clone(&bus)).run());

    // WindowPlant — 4 per-window motor → position ramps at 10 %/s.
    // Reads MotorDirection (window arbiter output) and integrates
    // Window.Position.
    set.spawn(WindowPlant::new(Arc::clone(&bus)).run());

    // PassiveEntry — handle-pull authenticated unlock.
    let pe_devices: Vec<PairedDevice> = {
        fn secret(device_type: u8, index: u8) -> [u8; 16] {
            let mut key = [0u8; 16];
            key[0] = device_type;
            key[1] = index;
            for (k, byte) in key.iter_mut().enumerate().skip(2) {
                *byte = (device_type.wrapping_mul(17))
                    .wrapping_add(index.wrapping_mul(31).wrapping_add(k as u8));
            }
            key
        }
        let mut v = Vec::new();
        for i in 1u8..=4 {
            v.push(PairedDevice {
                kind: DeviceKind::Fob,
                slot: (i - 1) as usize,
                secret: secret(b'F', i),
            });
        }
        for i in 1u8..=2 {
            v.push(PairedDevice {
                kind: DeviceKind::Phone,
                slot: (i - 1) as usize,
                secret: secret(b'P', i),
            });
        }
        v
    };
    set.spawn(
        PassiveEntry::new(
            Arc::clone(&bus),
            Arc::clone(&door_lock_arb),
            Arc::clone(&trunk_arb),
            Arc::clone(&cfg),
            pe_devices,
        )
        .run(),
    );
    set.spawn(
        Welcome::new(
            Arc::clone(&bus),
            Arc::clone(&courtesy_arb),
            Arc::clone(&puddle_arb),
        )
        .run(),
    );
    set.spawn(MirrorFold::with_nvm(Arc::clone(&bus), Arc::clone(&cfg), nvm.clone()).run());
    set.spawn(MirrorAdjust::new(Arc::clone(&bus)).run());
    {
        let hold = std::time::Duration::from_secs(cfg.dealer_config().farewell_hold_secs);
        set.spawn(
            Farewell::new(
                Arc::clone(&bus),
                Arc::clone(&courtesy_arb),
                Arc::clone(&puddle_arb),
            )
            .with_hold(hold)
            .run(),
        );
    }
    set.spawn(DoorOpenAssist::new(Arc::clone(&bus), Arc::clone(&puddle_arb), &cfg).run());
    set.spawn(ManualHorn::new(Arc::clone(&bus), Arc::clone(&horn_arb)).run());
    set.spawn(CabinTrunkRelease::new(Arc::clone(&bus), Arc::clone(&trunk_arb)).run());
    set.spawn(ExteriorTrunkButton::new(Arc::clone(&bus), Arc::clone(&trunk_arb)).run());

    tracing::info!("features spawned: ManualLighting, FollowMeHome, AutoHighBeam, BrakeReverseLamps, FogLamps, HazardLighting, TurnIndicator, RKE, LockFeedback, DoubleLockRelease, WalkAwayLock, ThumbPadLock, PanicAlarm, AutoRelock, PassiveEntry, Welcome, MirrorFold, MirrorAdjust, Farewell, DoorOpenAssist, ExteriorTrunkButton, CabinTrunkRelease, ManualHorn, PerimeterAlarm");

    // ── Plant Models ────────────────────────────────────────────────
    set.spawn(BlinkRelay::new(Arc::clone(&bus)).run());
    // Chime piezo: subscribes to Body.Chime.IsActive (intent), publishes
    // Body.Chime.IsSounding (actuator state).  HMI watches IsSounding
    // for the ripple visualisation.
    set.spawn(ChimePlantModel::new(Arc::clone(&bus)).run());
    // Day/Night HMI mode: subscribes to Body.Lights.Beam.Low.IsOn,
    // publishes Vehicle.Cabin.Infotainment.HMI.DayNightMode (VSS v4.0).
    // Drives the cockpit view's night-backlit rendering style.  Future
    // extensions: ambient light sensor, GPS sunset, tunnel detection.
    set.spawn(DayNightModePlant::new(Arc::clone(&bus)).run());
    set.spawn(
        DoorLockPlantModel::with_ack_and_nvm(Arc::clone(&bus), door_lock_ack_tx, nvm.clone())
            .with_cfg(Arc::clone(&cfg))
            .run(),
    );
    set.spawn(DoorHandlePlantModel::new(Arc::clone(&bus)).run());
    set.spawn(TrunkPlantModel::with_nvm(Arc::clone(&bus), nvm.clone()).run());
    set.spawn(HoodPlantModel::with_nvm(Arc::clone(&bus), nvm.clone()).run());
    set.spawn(SunroofPlantModel::with_nvm(Arc::clone(&bus), nvm.clone()).run());
    set.spawn(MirrorFoldPlantModel::with_nvm(Arc::clone(&bus), nvm.clone()).run());
    set.spawn(MirrorAdjustPlantModel::new(Arc::clone(&bus)).run());
    set.spawn(
        PepsPlantModel::new(Arc::clone(&bus))
            .with_response_stagger_ms(vss_bridge::plant_models::peps::PRODUCTION_STAGGER_MS)
            .run(),
    );
    tracing::info!("plant models spawned: BlinkRelay, ChimePlantModel, DayNightModePlant, DoorLockPlantModel, DoorHandlePlantModel, TrunkPlantModel, HoodPlantModel, SunroofPlantModel, PepsPlantModel");

    // ── WebSocket bridge ────────────────────────────────────────────
    let ws_port: u16 = std::env::var("VSS_BRIDGE_WS_PORT")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(8080);
    let ws_addr = format!("0.0.0.0:{ws_port}").parse()?;
    let ws_bridge = WsBridge::new(
        ws_addr,
        Arc::clone(&bus),
        Arc::clone(&cfg),
        reboot_tx.clone(),
    );
    set.spawn(async move {
        if let Err(e) = ws_bridge.run().await {
            tracing::error!(error = %e, "WebSocket bridge failed");
        }
    });

    // ── Initial signal state ────────────────────────────────────────
    bus.publish(
        "Vehicle.LowVoltageSystemState",
        SignalValue::String("OFF".to_string()),
    )
    .await?;
    bus.publish("Body.Switches.Panic.IsEngaged", SignalValue::Bool(false))
        .await?;
    bus.publish("Vehicle.Body.Alarm.IsActive", SignalValue::Bool(false))
        .await?;
    bus.publish("Body.Doors.AutoRelock.IsArmed", SignalValue::Bool(false))
        .await?;

    // ── kuksa.val sync (best-effort, retries forever) ───────────────
    let kuksa_endpoint =
        std::env::var("KUKSA_ENDPOINT").unwrap_or_else(|_| "http://localhost:55555".to_string());
    let kuksa = kuksa_sync::KuksaSync::new(&kuksa_endpoint, Arc::clone(&bus));
    set.spawn(async move { kuksa.run().await });

    Ok(set)
}
