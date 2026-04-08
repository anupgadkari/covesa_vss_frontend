//! vss-bridge — COVESA VSS body controller bridge (L5 application layer).
//!
//! This binary runs on the A53 cluster under Android Automotive OS.
//! It bridges the Safety Monitor (M7, RPmsg) with kuksa.val (gRPC)
//! and the Web HMI (WebSocket).

pub mod ipc_message;
pub mod signal_bus;
pub mod signal_ids;
pub mod arbiter;
pub mod adapters;
pub mod features;
pub mod kuksa_sync;
pub mod sleep_inhibit;

use std::sync::Arc;
use tracing_subscriber::EnvFilter;

use adapters::mock::MockBus;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .init();

    tracing::info!("vss-bridge starting");

    // Transport adapter — swap this line to change transport:
    //   let bus = Arc::new(RpmsgBus::new("/dev/rpmsg0", "/dev/rpmsg1").await?);
    let bus = Arc::new(MockBus::new());

    tracing::info!(
        signals = signal_ids::ALL_SIGNALS.len(),
        "signal catalog loaded"
    );

    // Domain Arbiters — one per actuator domain
    let (lighting_arb, lighting_fut) = arbiter::lighting_arbiter(Arc::clone(&bus));
    let (door_lock_arb, _door_lock_ack_tx, door_lock_fut) =
        arbiter::door_lock_arbiter(Arc::clone(&bus));
    let (horn_arb, horn_fut) = arbiter::horn_arbiter(Arc::clone(&bus));
    let (comfort_arb, comfort_fut) = arbiter::comfort_arbiter(Arc::clone(&bus));

    tokio::spawn(lighting_fut);
    tokio::spawn(door_lock_fut);
    tokio::spawn(horn_fut);
    tokio::spawn(comfort_fut);

    let _lighting_arb = Arc::new(lighting_arb);
    let _door_lock_arb = Arc::new(door_lock_arb);
    let _horn_arb = Arc::new(horn_arb);
    let _comfort_arb = Arc::new(comfort_arb);

    // TODO: Feature Business Logic
    // tokio::spawn(HazardFsm::new(Arc::clone(&_lighting_arb), Arc::clone(&bus)).run());
    // tokio::spawn(TurnFsm::new(Arc::clone(&_lighting_arb), Arc::clone(&bus)).run());
    // tokio::spawn(PepsFsm::new(Arc::clone(&_door_lock_arb), Arc::clone(&bus)).run());
    // ...

    // TODO: WebSocket server for L6 HMI
    // let ws_server = WsServer::new("0.0.0.0:8080", Arc::clone(&bus));
    // tokio::spawn(ws_server.run());

    // gRPC client for kuksa.val databroker at L4
    let kuksa_endpoint = std::env::var("KUKSA_ENDPOINT")
        .unwrap_or_else(|_| "http://localhost:55555".to_string());
    let kuksa = kuksa_sync::KuksaSync::new(&kuksa_endpoint, Arc::clone(&bus));
    tokio::spawn(async move { kuksa.run().await });

    let _ = bus; // suppress unused warning until FSMs are wired

    tracing::info!("vss-bridge ready — waiting for shutdown signal");
    tokio::signal::ctrl_c().await?;
    tracing::info!("vss-bridge shutting down");

    Ok(())
}
