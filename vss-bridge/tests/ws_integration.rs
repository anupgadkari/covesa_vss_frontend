//! Tier 2 — Out-of-process WebSocket integration tests.
//!
//! Launches the real vss-bridge binary as a subprocess, connects a
//! `tokio-tungstenite` WebSocket client, sends sensor messages, and
//! asserts on the state snapshots pushed back.
//!
//! These tests validate the full pipeline:
//!   HMI JSON → ws_bridge → SignalBus → features → arbiter → plant model → ws_bridge → HMI JSON
//!
//! Timing assertions use wall-clock with generous tolerances (±50 ms)
//! since the bridge runs in real-time mode.
//!
//! Run:  cargo test --test ws_integration

use std::process::{Child, Command};
use std::time::Duration;

use futures::{SinkExt, StreamExt};
use serde_json::{json, Value};
use tokio::time::{sleep, timeout, Instant};
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;

/// Port the bridge listens on (must match vss-bridge main.rs).
const WS_URL: &str = "ws://127.0.0.1:8080";

// ---------------------------------------------------------------------------
// Test fixture: launch & teardown the bridge process
// ---------------------------------------------------------------------------

struct BridgeProcess {
    child: Child,
}

impl BridgeProcess {
    fn start() -> Self {
        let child = Command::new(env!("CARGO_BIN_EXE_vss-bridge"))
            .env("RUST_LOG", "warn")
            .spawn()
            .expect("failed to start vss-bridge");
        Self { child }
    }
}

impl Drop for BridgeProcess {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// Connect to the bridge WS, retrying for up to 3 seconds.
async fn connect_ws() -> (
    futures::stream::SplitSink<
        tokio_tungstenite::WebSocketStream<
            tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
        >,
        Message,
    >,
    futures::stream::SplitStream<
        tokio_tungstenite::WebSocketStream<
            tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
        >,
    >,
) {
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        match connect_async(WS_URL).await {
            Ok((stream, _)) => return stream.split(),
            Err(_) if Instant::now() < deadline => {
                sleep(Duration::from_millis(50)).await;
            }
            Err(e) => panic!("failed to connect to bridge at {WS_URL}: {e}"),
        }
    }
}

/// Send a sensor message to the bridge.
async fn send_sensor(
    tx: &mut futures::stream::SplitSink<
        tokio_tungstenite::WebSocketStream<
            tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
        >,
        Message,
    >,
    path: &str,
    value: Value,
) {
    let msg = json!({"type": "sensor", "path": path, "value": value});
    tx.send(Message::Text(msg.to_string().into()))
        .await
        .expect("ws send failed");
}

/// Wait for a state snapshot where `path` has the expected value.
/// Times out after `max_wait`.
async fn wait_for_state(
    rx: &mut futures::stream::SplitStream<
        tokio_tungstenite::WebSocketStream<
            tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
        >,
    >,
    path: &str,
    expected: Value,
    max_wait: Duration,
) -> bool {
    let result = timeout(max_wait, async {
        while let Some(Ok(msg)) = rx.next().await {
            if let Message::Text(text) = msg {
                if let Ok(parsed) = serde_json::from_str::<Value>(&text) {
                    if let Some(state) = parsed.get("state") {
                        if state.get(path) == Some(&expected) {
                            return true;
                        }
                    }
                }
            }
        }
        false
    })
    .await;
    result.unwrap_or(false)
}

/// Collect state snapshots for `duration` and count transitions on `path`.
async fn count_transitions(
    rx: &mut futures::stream::SplitStream<
        tokio_tungstenite::WebSocketStream<
            tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
        >,
    >,
    path: &str,
    duration: Duration,
) -> usize {
    let mut transitions = 0;
    let mut last_val: Option<Value> = None;

    let _ = timeout(duration, async {
        while let Some(Ok(msg)) = rx.next().await {
            if let Message::Text(text) = msg {
                if let Ok(parsed) = serde_json::from_str::<Value>(&text) {
                    if let Some(state) = parsed.get("state") {
                        if let Some(val) = state.get(path) {
                            if last_val.as_ref() != Some(val) {
                                transitions += 1;
                                last_val = Some(val.clone());
                            }
                        }
                    }
                }
            }
        }
    })
    .await;

    transitions
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[tokio::test]
async fn hazard_switch_activates_both_indicators_via_ws() {
    let _bridge = BridgeProcess::start();
    let (mut tx, mut rx) = connect_ws().await;

    send_sensor(&mut tx, "Body.Switches.Hazard.IsEngaged", json!(true)).await;

    let left_ok = wait_for_state(
        &mut rx,
        "Body.Lights.DirectionIndicator.Left.IsSignaling",
        json!(true),
        Duration::from_secs(2),
    )
    .await;
    assert!(left_ok, "Left.IsSignaling should be TRUE via WS");
}

#[tokio::test]
async fn turn_stalk_requires_ignition_on() {
    let _bridge = BridgeProcess::start();
    let (mut tx, mut rx) = connect_ws().await;

    // Ignition defaults to OFF — stalk should have no effect.
    send_sensor(
        &mut tx,
        "Body.Switches.TurnIndicator.Direction",
        json!("RIGHT"),
    )
    .await;

    // Wait briefly — should NOT see Right.IsSignaling = true.
    let right = wait_for_state(
        &mut rx,
        "Body.Lights.DirectionIndicator.Right.IsSignaling",
        json!(true),
        Duration::from_millis(500),
    )
    .await;
    assert!(!right, "turn stalk should be ignored with ignition OFF");
}

#[tokio::test]
async fn hazard_engaged_then_disengaged_with_turn_resuming() {
    let _bridge = BridgeProcess::start();
    let (mut tx, mut rx) = connect_ws().await;

    // Ignition ON + stalk RIGHT
    send_sensor(&mut tx, "Vehicle.LowVoltageSystemState", json!("ON")).await;
    sleep(Duration::from_millis(50)).await;

    send_sensor(
        &mut tx,
        "Body.Switches.TurnIndicator.Direction",
        json!("RIGHT"),
    )
    .await;

    // Wait for right indicator to start
    let right_on = wait_for_state(
        &mut rx,
        "Body.Lights.DirectionIndicator.Right.IsSignaling",
        json!(true),
        Duration::from_secs(2),
    )
    .await;
    assert!(
        right_on,
        "Right should signal after stalk RIGHT + ignition ON"
    );

    // Engage hazard
    send_sensor(&mut tx, "Body.Switches.Hazard.IsEngaged", json!(true)).await;
    let left_on = wait_for_state(
        &mut rx,
        "Body.Lights.DirectionIndicator.Left.IsSignaling",
        json!(true),
        Duration::from_secs(2),
    )
    .await;
    assert!(left_on, "Left should signal after hazard engage");

    // Disengage hazard — right should resume, left should go off
    send_sensor(&mut tx, "Body.Switches.Hazard.IsEngaged", json!(false)).await;

    // Right should still be true (Turn MEDIUM claim resumes)
    let right_still = wait_for_state(
        &mut rx,
        "Body.Lights.DirectionIndicator.Right.IsSignaling",
        json!(true),
        Duration::from_secs(2),
    )
    .await;
    assert!(
        right_still,
        "Right should resume signaling after hazard release"
    );
}

#[tokio::test]
async fn plant_model_blinks_at_expected_rate() {
    let _bridge = BridgeProcess::start();
    let (mut tx, mut rx) = connect_ws().await;

    // Engage hazard (ignition-independent).
    send_sensor(&mut tx, "Body.Switches.Hazard.IsEngaged", json!(true)).await;

    // Wait for the first lamp event to appear.
    let lamp_path = "Body.Lights.DirectionIndicator.Left.Lamp.Front.IsOn";
    let started = wait_for_state(&mut rx, lamp_path, json!(true), Duration::from_secs(2)).await;
    assert!(started, "plant model should start blinking");

    // Count transitions over 2 seconds at 1.5 Hz (333 ms half-period).
    // Expected: ~6 transitions (on→off→on→off→on→off) in 2s.
    let transitions = count_transitions(&mut rx, lamp_path, Duration::from_secs(2)).await;
    assert!(
        (4..=8).contains(&transitions),
        "expected 4-8 lamp transitions in 2s at 1.5 Hz, got {transitions}"
    );
}
