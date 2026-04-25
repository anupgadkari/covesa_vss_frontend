//! vss-bridge — COVESA VSS body controller bridge (L5 application layer).
//!
//! This binary runs on the A53 cluster under Android Automotive OS.
//! It bridges the Safety Monitor (M7, RPmsg) with kuksa.val (gRPC)
//! and the Web HMI (WebSocket).

use std::sync::Arc;
use tracing_subscriber::EnvFilter;

use vss_bridge::adapters::mock::MockBus;
use vss_bridge::arbiter;
use vss_bridge::config;
use vss_bridge::features::double_lock_release::DoubleLockRelease;
use vss_bridge::features::hazard_lighting::HazardLighting;
use vss_bridge::features::lock_feedback::LockFeedback;
use vss_bridge::features::rke::{PairedFob, RkeFeature};
use vss_bridge::features::thumb_pad_lock::ThumbPadLock;
use vss_bridge::features::turn_indicator::TurnIndicator;
use vss_bridge::features::walk_away_lock::WalkAwayLock;
use vss_bridge::ipc_message::SignalValue;
use vss_bridge::kuksa_sync;
use vss_bridge::plant_models::blink_relay::BlinkRelay;
use vss_bridge::plant_models::door_handle::DoorHandlePlantModel;
use vss_bridge::plant_models::door_lock::DoorLockPlantModel;
use vss_bridge::plant_models::peps::PepsPlantModel;
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

    tracing::info!("vss-bridge starting");

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
    let (door_lock_arb, door_lock_ack_tx, door_lock_fut) =
        arbiter::door_lock_arbiter(Arc::clone(&bus));
    let (_horn_arb, horn_fut) = arbiter::horn_arbiter(Arc::clone(&bus));
    let (_comfort_arb, comfort_fut) = arbiter::comfort_arbiter(Arc::clone(&bus));

    tokio::spawn(lighting_fut);
    tokio::spawn(door_lock_fut);
    tokio::spawn(horn_fut);
    tokio::spawn(comfort_fut);

    let lighting_arb = Arc::new(lighting_arb);
    let door_lock_arb = Arc::new(door_lock_arb);

    // ── Feature Business Logic ──────────────────────────────────────
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

    // TODO: remaining features
    // tokio::spawn(AutoRelock::from_config(Arc::clone(&door_lock_arb), Arc::clone(&bus), &_platform_config).run());

    tracing::info!("features spawned: HazardLighting, TurnIndicator, RKE, LockFeedback, DoubleLockRelease, WalkAwayLock, ThumbPadLock");

    // ── Plant Models ────────────────────────────────────────────────
    // Simulate physical lamp behavior the M7 / smart actuator firmware
    // would normally provide. Plant models bypass the arbiter and
    // publish feedback signals (lamp on/off, defects) directly.
    tokio::spawn(BlinkRelay::new(Arc::clone(&bus)).run());
    tokio::spawn(DoorLockPlantModel::with_ack_tx(Arc::clone(&bus), door_lock_ack_tx).run());
    tokio::spawn(DoorHandlePlantModel::new(Arc::clone(&bus)).run());
    tokio::spawn(TrunkPlantModel::new(Arc::clone(&bus)).run());
    tokio::spawn(PepsPlantModel::new(Arc::clone(&bus)).run());
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

    // gRPC client for kuksa.val databroker at L4 (optional — fails gracefully)
    let kuksa_endpoint =
        std::env::var("KUKSA_ENDPOINT").unwrap_or_else(|_| "http://localhost:55555".to_string());
    let kuksa = kuksa_sync::KuksaSync::new(&kuksa_endpoint, Arc::clone(&bus));
    tokio::spawn(async move { kuksa.run().await });

    tracing::info!("vss-bridge ready — open vss-hmi-body-sensors.html in a browser");
    tracing::info!("  WebSocket: ws://localhost:8080");
    tracing::info!("  Press Ctrl+C to stop");

    tokio::signal::ctrl_c().await?;
    tracing::info!("vss-bridge shutting down");

    Ok(())
}
