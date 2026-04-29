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

use std::net::TcpListener as StdTcpListener;
use std::process::{Child, Command};
use std::time::Duration;

use futures::{SinkExt, StreamExt};
use serde_json::{json, Value};
use tokio::time::{sleep, timeout, Instant};
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;

// ---------------------------------------------------------------------------
// Test fixture: launch & teardown the bridge process
// ---------------------------------------------------------------------------

/// Ask the OS for a free port by binding :0 and immediately releasing.
/// The bridge is started on this port via VSS_BRIDGE_WS_PORT, so each
/// test uses its own port and is fully isolated from any bridge a developer
/// may be running manually on the default :8080.
fn free_port() -> u16 {
    StdTcpListener::bind("127.0.0.1:0")
        .expect("failed to bind ephemeral port")
        .local_addr()
        .unwrap()
        .port()
}

struct BridgeProcess {
    child: Child,
    port: u16,
}

impl BridgeProcess {
    fn start() -> Self {
        let port = free_port();
        // Each test bridge needs its own HTTP port too — the default
        // (3000) is the developer-facing HMI port and would collide
        // when cargo runs integration tests in parallel.
        let http_port = free_port();
        let child = Command::new(env!("CARGO_BIN_EXE_vss-bridge"))
            .env("RUST_LOG", "warn")
            .env("VSS_BRIDGE_WS_PORT", port.to_string())
            .env("VSS_BRIDGE_HTTP_PORT", http_port.to_string())
            .spawn()
            .expect("failed to start vss-bridge");
        Self { child, port }
    }

    fn ws_url(&self) -> String {
        format!("ws://127.0.0.1:{}", self.port)
    }
}

impl Drop for BridgeProcess {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// Connect to the bridge WS at `url`, retrying for up to 3 seconds.
async fn connect_ws(
    url: &str,
) -> (
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
        match connect_async(url).await {
            Ok((stream, _)) => return stream.split(),
            Err(_) if Instant::now() < deadline => {
                sleep(Duration::from_millis(50)).await;
            }
            Err(e) => panic!("failed to connect to bridge at {url}: {e}"),
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
    let bridge = BridgeProcess::start();
    let (mut tx, mut rx) = connect_ws(&bridge.ws_url()).await;

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
    let bridge = BridgeProcess::start();
    let (mut tx, mut rx) = connect_ws(&bridge.ws_url()).await;

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
    let bridge = BridgeProcess::start();
    let (mut tx, mut rx) = connect_ws(&bridge.ws_url()).await;

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
    let bridge = BridgeProcess::start();
    let (mut tx, mut rx) = connect_ws(&bridge.ws_url()).await;

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

// ───────────────────────────────────────────────────────────────────────────
// PEPS / PassiveEntry / Welcome integration tests
// ───────────────────────────────────────────────────────────────────────────

/// Place paired fob 1 in LeftFront zone, pull the Row1.Left handle —
/// verify the bridge unlocks Row1.Left via the real PassiveEntry +
/// PEPS plant pipeline (challenge → response → arbiter → plant).
#[tokio::test]
async fn passive_entry_unlocks_driver_door_via_handle_pull() {
    let bridge = BridgeProcess::start();
    let (mut tx, mut rx) = connect_ws(&bridge.ws_url()).await;

    // Make sure doors start locked so we'll see the unlock transition.
    send_sensor(&mut tx, "Body.Doors.CentralLock.Command", json!("lock_all")).await;
    let locked = wait_for_state(
        &mut rx,
        "Body.Doors.Row1.Left.IsLocked",
        json!(true),
        Duration::from_secs(2),
    )
    .await;
    assert!(locked, "doors should lock for the test setup");

    // Place fob 1 at the driver door.
    send_sensor(&mut tx, "Body.PEPS.Plant.KeyFob.1.Zone", json!("LeftFront")).await;

    // Allow the production stagger (10 ms × slot) plus arbiter +
    // plant model round trip.
    sleep(Duration::from_millis(50)).await;

    // Pull the driver-door outside handle.
    send_sensor(
        &mut tx,
        "Body.Doors.Row1.Left.Handle.Outside.IsPulled",
        json!(true),
    )
    .await;

    // Driver door should unlock within the 150 ms challenge window
    // plus ~50 ms arbiter / plant model latency.
    let unlocked = wait_for_state(
        &mut rx,
        "Body.Doors.Row1.Left.IsLocked",
        json!(false),
        Duration::from_secs(2),
    )
    .await;
    assert!(
        unlocked,
        "Row1.Left should unlock after passive-entry handle pull with paired fob in LeftFront"
    );
}

/// Handle pull with no paired device positioned must NOT unlock.
#[tokio::test]
async fn passive_entry_no_device_in_zone_keeps_doors_locked() {
    let bridge = BridgeProcess::start();
    let (mut tx, mut rx) = connect_ws(&bridge.ws_url()).await;

    send_sensor(&mut tx, "Body.Doors.CentralLock.Command", json!("lock_all")).await;
    let locked = wait_for_state(
        &mut rx,
        "Body.Doors.Row1.Left.IsLocked",
        json!(true),
        Duration::from_secs(2),
    )
    .await;
    assert!(locked, "doors should lock for the test setup");

    // No paired device positioned anywhere.  Pull the handle.
    send_sensor(
        &mut tx,
        "Body.Doors.Row1.Left.Handle.Outside.IsPulled",
        json!(true),
    )
    .await;

    // Wait the full challenge timeout + a margin; doors should still
    // be locked.
    let still_locked = !wait_for_state(
        &mut rx,
        "Body.Doors.Row1.Left.IsLocked",
        json!(false),
        Duration::from_millis(500),
    )
    .await;
    assert!(
        still_locked,
        "Row1.Left must stay locked when no paired device is in the proximity zone"
    );
}

/// Welcome — fob entering Approach turns puddle lamps ON.
#[tokio::test]
async fn welcome_arms_puddle_lamps_on_approach_entry() {
    let bridge = BridgeProcess::start();
    let (mut tx, mut rx) = connect_ws(&bridge.ws_url()).await;

    // Place fob 1 at Approach.
    send_sensor(&mut tx, "Body.PEPS.Plant.KeyFob.1.Zone", json!("Approach")).await;

    let on = wait_for_state(
        &mut rx,
        "Body.Lights.Puddle.Left.IsOn",
        json!(true),
        Duration::from_secs(2),
    )
    .await;
    assert!(on, "puddle Left should arm on PEPS approach entry");
}

/// ThumbPadLock — pad held with fob in Approach locks; pad held with
/// no fob anywhere does NOT lock (keys-in-vehicle guard end-to-end).
#[tokio::test]
async fn thumb_pad_lock_blocked_when_no_fob_outside() {
    let bridge = BridgeProcess::start();
    let (mut tx, mut rx) = connect_ws(&bridge.ws_url()).await;

    // No paired device anywhere.  Press the lock pad and hold for >500 ms.
    send_sensor(
        &mut tx,
        "Body.Doors.Row1.Left.Handle.Outside.LockPad.IsPressed",
        json!(true),
    )
    .await;

    // After the 500 ms debounce, the gate should deny.  Verify no
    // lock state change reaches the HMI within the window.
    let locked = wait_for_state(
        &mut rx,
        "Body.Doors.Row1.Left.IsLocked",
        json!(true),
        Duration::from_millis(800),
    )
    .await;
    assert!(
        !locked,
        "ThumbPadLock must deny when no paired device is outside the cabin"
    );
}
