//! vss-bridge — COVESA VSS body controller bridge (L5 application layer).
//!
//! This binary runs on the A53 cluster under Android Automotive OS.
//! It bridges the Safety Monitor (M7, RPmsg) with kuksa.val (gRPC)
//! and the Web HMI (WebSocket).

use std::sync::Arc;
use tracing_subscriber::EnvFilter;

use axum::Router;
use tower_http::services::ServeDir;
use tower_http::set_header::SetResponseHeaderLayer;

use vss_bridge::adapters::mock::MockBus;
use vss_bridge::arbiter;
use vss_bridge::config;
use vss_bridge::features::auto_high_beam::AutoHighBeam;
use vss_bridge::features::auto_relock::AutoRelock;
use vss_bridge::features::brake_reverse_lamps::BrakeReverseLamps;
use vss_bridge::features::double_lock_release::DoubleLockRelease;
use vss_bridge::features::fog_lamps::FogLamps;
use vss_bridge::features::follow_me_home::FollowMeHome;
use vss_bridge::features::hazard_lighting::HazardLighting;
use vss_bridge::features::lock_feedback::LockFeedback;
use vss_bridge::features::manual_lighting::ManualLighting;
use vss_bridge::features::panic_alarm::PanicAlarm;
use vss_bridge::features::passive_entry::{DeviceKind, PairedDevice, PassiveEntry};
use vss_bridge::features::rke::{PairedFob, RkeFeature};
use vss_bridge::features::thumb_pad_lock::ThumbPadLock;
use vss_bridge::features::turn_indicator::TurnIndicator;
use vss_bridge::features::walk_away_lock::WalkAwayLock;
use vss_bridge::features::mirror_fold::MirrorFold;
use vss_bridge::features::welcome::Welcome;
use vss_bridge::ipc_message::SignalValue;
use vss_bridge::kuksa_sync;
use vss_bridge::nvm::NvmStore;
use vss_bridge::plant_models::blink_relay::BlinkRelay;
use vss_bridge::plant_models::door_handle::DoorHandlePlantModel;
use vss_bridge::plant_models::door_lock::DoorLockPlantModel;
use vss_bridge::plant_models::peps::PepsPlantModel;
use vss_bridge::plant_models::mirror_fold::MirrorFoldPlantModel;
use vss_bridge::plant_models::trunk::TrunkPlantModel;
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
    // `--reset-nvm` wipes persisted state on boot — used to simulate a
    // factory-new vehicle for cold-boot test scenarios.  Anything more
    // sophisticated should grow into clap; today we only have one flag.
    let args: Vec<String> = std::env::args().collect();
    let reset_nvm = args.iter().any(|a| a == "--reset-nvm");

    tracing::info!(reset_nvm, "vss-bridge starting");

    // ── NVM store ──────────────────────────────────────────────────
    // Path defaults to ./nvm/ (overridable via VSS_BRIDGE_NVM_PATH).
    // Plant models that own persistent state load from / save to here.
    // See docs/signal-ownership-and-state-hydration.md §3.
    let nvm = NvmStore::from_env();
    if reset_nvm {
        nvm.reset();
    }

    // Platform configuration — four-tier system:
    //   Tier 1: compile-time constants (config::IPC_MAGIC, etc.)
    //   Tier 2: vehicle-line calibration (/etc/vss-bridge/vehicle_line.json)
    //   Tier 3: variant/trim calibration (/etc/vss-bridge/variant.json)
    //   Tier 4: dealer config (pushed by M7 via RPmsg at boot + runtime)
    let _platform_config = config::PlatformConfig::load();

    // Transport adapter — swap this line to change transport:
    //   let bus = Arc::new(RpmsgBus::new("/dev/rpmsg0", "/dev/rpmsg1").await?);
    let bus = Arc::new(MockBus::new());

    tracing::info!(
        signals = signal_ids::ALL_SIGNALS.len(),
        "signal catalog loaded"
    );

    // ── Domain Arbiters ─────────────────────────────────────────────
    let (lighting_arb, lighting_fut) = arbiter::lighting_arbiter(Arc::clone(&bus));
    let (low_beam_arb, low_beam_fut) = arbiter::low_beam_arbiter(Arc::clone(&bus));
    let (door_lock_arb, door_lock_ack_tx, door_lock_fut) =
        arbiter::door_lock_arbiter_with_nvm(Arc::clone(&bus), nvm.clone());
    let (horn_arb, horn_fut) = arbiter::horn_arbiter(Arc::clone(&bus));
    let (_comfort_arb, comfort_fut) = arbiter::comfort_arbiter(Arc::clone(&bus));
    let (courtesy_arb, courtesy_fut) = arbiter::courtesy_arbiter(Arc::clone(&bus));
    let (puddle_arb, puddle_fut) = arbiter::puddle_arbiter(Arc::clone(&bus));

    tokio::spawn(lighting_fut);
    tokio::spawn(low_beam_fut);
    tokio::spawn(door_lock_fut);
    tokio::spawn(horn_fut);
    tokio::spawn(comfort_fut);
    tokio::spawn(courtesy_fut);
    tokio::spawn(puddle_fut);

    let lighting_arb = Arc::new(lighting_arb);
    let low_beam_arb = Arc::new(low_beam_arb);
    let door_lock_arb = Arc::new(door_lock_arb);
    let horn_arb = Arc::new(horn_arb);
    let courtesy_arb = Arc::new(courtesy_arb);
    let puddle_arb = Arc::new(puddle_arb);

    // ── Feature Business Logic ──────────────────────────────────────
    let lux_threshold = _platform_config.vehicle_line.auto_headlamp_lux_threshold;

    // ManualLighting — switch-driven low/high/DRL/parking/license outputs via LowBeam arbiter.
    tokio::spawn(
        ManualLighting::new(Arc::clone(&low_beam_arb), Arc::clone(&bus), lux_threshold).run(),
    );

    // FollowMeHome — activates low beam 45 s after ignition-off door open (dark only).
    tokio::spawn(
        FollowMeHome::new(Arc::clone(&low_beam_arb), Arc::clone(&bus), lux_threshold).run(),
    );

    // HazardLighting — no ignition gate, works in any power state
    tokio::spawn(HazardLighting::new(Arc::clone(&lighting_arb), Arc::clone(&bus)).run());

    // TurnIndicator — ignition-gated (ON/START only), with comfort blink
    tokio::spawn(
        TurnIndicator::with_config(
            Arc::clone(&lighting_arb),
            Arc::clone(&bus),
            Arc::clone(&_platform_config),
        )
        .run(),
    );

    // RKE — Remote Keyless Entry.
    // Provisioned fob secrets must match the PEPS plant model's default_secret().
    // In production these come from key provisioning (not hard-coded here).
    let rke_fobs: Vec<PairedFob> = (1u8..=4)
        .map(|i| {
            // Must match plant model's default_secret(b'F', index).
            // Formula: key[0]=device_type, key[1]=index,
            //          key[k] = device_type*17 + index*31 + k  (for k >= 2)
            const DEVICE_TYPE: u8 = b'F'; // b'F' = 70, matches PepsPlantModel keyfob secrets
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

    tokio::spawn(
        RkeFeature::new(
            Arc::clone(&bus),
            Arc::clone(&door_lock_arb),
            Arc::clone(&_platform_config),
            rke_fobs,
        )
        .run(),
    );

    // LockFeedback — plays direction-indicator flash patterns for external lock/unlock events.
    tokio::spawn(LockFeedback::new(Arc::clone(&bus), Arc::clone(&lighting_arb)).run());

    // DoubleLockRelease — clears superlock on ignition ON (no feedback, internal trigger).
    tokio::spawn(DoubleLockRelease::new(Arc::clone(&bus), Arc::clone(&door_lock_arb)).run());

    // WalkAwayLock — locks when all PEPS devices leave the approach zone.
    tokio::spawn(WalkAwayLock::new(Arc::clone(&bus), Arc::clone(&door_lock_arb)).run());

    // ThumbPadLock — Row 1 outside handle thumb pad, 500 ms debounce.
    tokio::spawn(ThumbPadLock::new(Arc::clone(&bus), Arc::clone(&door_lock_arb)).run());

    // AutoHighBeam — ADAS camera suppresses high beam when oncoming vehicle detected.
    // Claims Beam.High.IsOn at High priority with Bool(false), overriding ManualLighting's
    // Medium-priority claim. Releases when path is clear so manual high beam resumes.
    tokio::spawn(AutoHighBeam::new(Arc::clone(&low_beam_arb), Arc::clone(&bus)).run());

    // BrakeReverseLamps — pedal-driven stop lights, gear-driven backup lights.
    tokio::spawn(BrakeReverseLamps::new(Arc::clone(&bus)).run());

    // FogLamps — front and rear fog lamps, ignition-gated switch pass-through.
    tokio::spawn(FogLamps::new(Arc::clone(&bus)).run());

    // PanicAlarm — flashes both indicators + chirps horn while
    // Body.Switches.Panic.IsEngaged is TRUE.  Triggered by RKE on a paired-
    // keyfob PANIC press; ignition-independent (security feature).
    tokio::spawn(
        PanicAlarm::new(
            Arc::clone(&lighting_arb),
            Arc::clone(&horn_arb),
            Arc::clone(&bus),
        )
        .run(),
    );

    // AutoRelock — re-locks the vehicle ${auto_relock_timeout_secs} seconds
    // after an unlock event if no door has been opened.  Cancelled if a
    // door opens, the vehicle is re-locked manually, or a crash is
    // detected (and stays disabled until full power-cycle in that case).
    tokio::spawn(
        AutoRelock::from_config(
            Arc::clone(&door_lock_arb),
            Arc::clone(&bus),
            &_platform_config,
        )
        .run(),
    );

    // PassiveEntry — handle-pull authenticated unlock for the four
    // outside doors.  Subscribes to handle pulls + paired-device zone
    // signals; on a pull, issues an LF + BLE challenge, verifies the
    // first response, dispatches UnlockDriver / UnlockAll via the
    // door-lock arbiter (two-stage cal shared with RKE).
    let pe_devices: Vec<PairedDevice> = {
        // Same secret-derivation formula as the RKE wiring above + the
        // PEPS plant model's default_secret().  Four paired fobs and
        // two paired phones.
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
    tokio::spawn(
        PassiveEntry::new(
            Arc::clone(&bus),
            Arc::clone(&door_lock_arb),
            Arc::clone(&_platform_config),
            pe_devices,
        )
        .run(),
    );
    // Welcome — exterior puddle + dome courtesy lights when any
    // paired PEPS device enters LF coverage (Approach or proximity).
    // 30 s hold by default; releases early on ignition ON or when
    // all devices leave LF.
    tokio::spawn(
        Welcome::new(
            Arc::clone(&bus),
            Arc::clone(&courtesy_arb),
            Arc::clone(&puddle_arb),
        )
        .run(),
    );

    // MirrorFold — handles `Body.Switches.Mirror.Fold` momentary press
    // and (when dealer cal `mirror_fold_mode = AUTO`) auto-folds on
    // central-lock state edges.  Publishes per-side `FoldCmd` to the
    // MirrorFoldPlantModel.  Persists `last_fold_cmd` in NVM so the
    // toggle direction stays consistent across power cycles.
    tokio::spawn(
        MirrorFold::with_nvm(Arc::clone(&bus), Arc::clone(&_platform_config), nvm.clone()).run(),
    );

    tracing::info!("features spawned: ManualLighting, FollowMeHome, AutoHighBeam, BrakeReverseLamps, FogLamps, HazardLighting, TurnIndicator, RKE, LockFeedback, DoubleLockRelease, WalkAwayLock, ThumbPadLock, PanicAlarm, AutoRelock, PassiveEntry, Welcome, MirrorFold");

    // ── Plant Models ────────────────────────────────────────────────
    // Simulate physical lamp behavior the M7 / smart actuator firmware
    // would normally provide. Plant models bypass the arbiter and
    // publish feedback signals (lamp on/off, defects) directly.
    tokio::spawn(BlinkRelay::new(Arc::clone(&bus)).run());
    tokio::spawn(
        DoorLockPlantModel::with_ack_and_nvm(Arc::clone(&bus), door_lock_ack_tx, nvm.clone()).run(),
    );
    tokio::spawn(DoorHandlePlantModel::new(Arc::clone(&bus)).run());
    tokio::spawn(TrunkPlantModel::with_nvm(Arc::clone(&bus), nvm.clone()).run());
    tokio::spawn(MirrorFoldPlantModel::with_nvm(Arc::clone(&bus), nvm.clone()).run());
    tokio::spawn(
        PepsPlantModel::new(Arc::clone(&bus))
            // 10 ms × slot index — fob 1 = 10 ms, fob 6 = 60 ms.
            // Stops concurrent same-zone devices from all responding
            // on the same scheduler tick and lets PassiveEntry pick
            // a deterministic "first responder wins".
            .with_response_stagger_ms(vss_bridge::plant_models::peps::PRODUCTION_STAGGER_MS)
            .run(),
    );
    tracing::info!("plant models spawned: BlinkRelay, DoorLockPlantModel, DoorHandlePlantModel, TrunkPlantModel, PepsPlantModel");

    // ── WebSocket bridge for L6 HMI ─────────────────────────────────
    // Port is overridable via VSS_BRIDGE_WS_PORT for integration tests
    // (each test picks a free ephemeral port so it never collides with a
    // developer's running bridge on the default 8080).
    let ws_port: u16 = std::env::var("VSS_BRIDGE_WS_PORT")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(8080);
    let ws_addr = format!("0.0.0.0:{ws_port}").parse()?;
    let ws_bridge = WsBridge::new(ws_addr, Arc::clone(&bus), Arc::clone(&_platform_config));
    tokio::spawn(async move {
        if let Err(e) = ws_bridge.run().await {
            tracing::error!(error = %e, "WebSocket bridge failed");
        }
    });

    // ── Set initial vehicle state ───────────────────────────────────
    // Default ignition OFF — HMI user can switch to ON to enable turn signals.
    bus.publish(
        "Vehicle.LowVoltageSystemState",
        SignalValue::String("OFF".to_string()),
    )
    .await?;

    // Panic alarm starts disengaged — keep the PanicAlarm feature's switch
    // subscription primed so it sees the first FALSE→TRUE transition.
    bus.publish("Body.Switches.Panic.IsEngaged", SignalValue::Bool(false))
        .await?;
    bus.publish("Vehicle.Body.Alarm.IsActive", SignalValue::Bool(false))
        .await?;
    bus.publish("Body.Doors.AutoRelock.IsArmed", SignalValue::Bool(false))
        .await?;

    // gRPC client for kuksa.val databroker at L4 (optional — fails gracefully)
    let kuksa_endpoint =
        std::env::var("KUKSA_ENDPOINT").unwrap_or_else(|_| "http://localhost:55555".to_string());
    let kuksa = kuksa_sync::KuksaSync::new(&kuksa_endpoint, Arc::clone(&bus));
    tokio::spawn(async move { kuksa.run().await });

    // ── Static HTTP server for HMI files ───────────────────────────
    // Serves the repo root (HTML files) on port 3000 so the browser
    // can open http://localhost:3000/vss-hmi-body-sensors.html instead
    // of a file:// URL (which has CORS and caching quirks).
    // Port is overridable via VSS_BRIDGE_HTTP_PORT.
    let http_port: u16 = std::env::var("VSS_BRIDGE_HTTP_PORT")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(3000);
    // Serve the directory one level above the vss-bridge crate (the repo root).
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

    tracing::info!("vss-bridge ready");
    tracing::info!("  HMI:             http://localhost:{http_port}/vss-hmi.html");
    tracing::info!("  WebSocket:       ws://localhost:8080");
    tracing::info!("  Press Ctrl+C to stop");

    tokio::signal::ctrl_c().await?;
    tracing::info!("vss-bridge shutting down");

    Ok(())
}
