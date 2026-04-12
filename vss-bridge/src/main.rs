//! vss-bridge — COVESA VSS body controller bridge (L5 application layer).
//!
//! This binary runs on the A53 cluster under Android Automotive OS.
//! It bridges the Safety Monitor (M7, RPmsg) with kuksa.val (gRPC)
//! and the Web HMI (WebSocket).

pub mod config;
pub mod ipc_message;
pub mod signal_bus;
pub mod signal_ids;
pub mod arbiter;
pub mod adapters;
pub mod features;
pub mod kuksa_sync;
pub mod sleep_inhibit;
pub mod ws_bridge;

use std::sync::Arc;
use tracing_subscriber::EnvFilter;

use adapters::mock::MockBus;
use features::hazard_lighting::HazardLighting;
use features::turn_indicator::TurnIndicator;
use ipc_message::SignalValue;
use signal_bus::SignalBus;
use ws_bridge::WsBridge;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| EnvFilter::new("info")),
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
    let (_door_lock_arb, _door_lock_ack_tx, door_lock_fut) =
        arbiter::door_lock_arbiter(Arc::clone(&bus));
    let (_horn_arb, horn_fut) = arbiter::horn_arbiter(Arc::clone(&bus));
    let (_comfort_arb, comfort_fut) = arbiter::comfort_arbiter(Arc::clone(&bus));

    tokio::spawn(lighting_fut);
    tokio::spawn(door_lock_fut);
    tokio::spawn(horn_fut);
    tokio::spawn(comfort_fut);

    let lighting_arb = Arc::new(lighting_arb);

    // ── Feature Business Logic ──────────────────────────────────────
    // HazardLighting — no ignition gate, works in any power state
    tokio::spawn(
        HazardLighting::new(Arc::clone(&lighting_arb), Arc::clone(&bus)).run(),
    );

    // TurnIndicator — ignition-gated (ON/START only)
    tokio::spawn(
        TurnIndicator::new(Arc::clone(&lighting_arb), Arc::clone(&bus)).run(),
    );

    // TODO: remaining features
    // tokio::spawn(AutoRelock::from_config(Arc::clone(&_door_lock_arb), Arc::clone(&bus), &_platform_config).run());

    tracing::info!("features spawned: HazardLighting, TurnIndicator");

    // ── WebSocket bridge for L6 HMI ─────────────────────────────────
    let ws_addr = "0.0.0.0:8080".parse()?;
    let ws_bridge = WsBridge::new(ws_addr, Arc::clone(&bus));
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
    let kuksa_endpoint = std::env::var("KUKSA_ENDPOINT")
        .unwrap_or_else(|_| "http://localhost:55555".to_string());
    let kuksa = kuksa_sync::KuksaSync::new(&kuksa_endpoint, Arc::clone(&bus));
    tokio::spawn(async move { kuksa.run().await });

    tracing::info!("vss-bridge ready — open vss-hmi-body-sensors.html in a browser");
    tracing::info!("  WebSocket: ws://localhost:8080");
    tracing::info!("  Press Ctrl+C to stop");

    tokio::signal::ctrl_c().await?;
    tracing::info!("vss-bridge shutting down");

    Ok(())
}
