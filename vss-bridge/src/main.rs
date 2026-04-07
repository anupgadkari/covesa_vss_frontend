//! vss-bridge — COVESA VSS body controller bridge (L5 application layer).
//!
//! This binary runs on the A53 cluster under Android Automotive OS.
//! It bridges the Safety Monitor (M7, RPmsg) with kuksa.val (gRPC)
//! and the Web HMI (WebSocket).

pub mod ipc_message;
pub mod signal_bus;
pub mod signal_ids;
pub mod adapters;
pub mod features;

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

    // TODO: Signal Arbiter
    // let (arbiter, arb_rx) = SignalArbiter::new(Arc::clone(&bus));
    // let arbiter = Arc::new(arbiter);
    // tokio::spawn(arbiter_loop(Arc::clone(&bus), arb_rx));

    // TODO: Feature FSMs
    // tokio::spawn(HazardFsm::new(...).run());
    // tokio::spawn(TurnFsm::new(...).run());
    // ...

    // TODO: WebSocket server for L6 HMI
    // let ws_server = WsServer::new("0.0.0.0:8080", Arc::clone(&bus));
    // tokio::spawn(ws_server.run());

    // TODO: gRPC client for kuksa.val at L4
    // let kuksa = KuksaClient::connect("http://localhost:55555").await?;
    // tokio::spawn(kuksa.sync_loop(Arc::clone(&bus)));

    let _ = bus; // suppress unused warning until components are wired

    tracing::info!("vss-bridge ready — waiting for shutdown signal");
    tokio::signal::ctrl_c().await?;
    tracing::info!("vss-bridge shutting down");

    Ok(())
}
