//! WebSocket bridge — connects the Web HMI (L6) to the SignalBus (L5).
//!
//! Listens on `0.0.0.0:8080` for WebSocket connections from the HMI.
//!
//! Protocol (JSON):
//!   HMI → bridge:  {"type":"sensor","path":"Body.Switches.Hazard.IsEngaged","value":true}
//!   bridge → HMI:  {"state":{"Body.Lights.DirectionIndicator.Left.IsSignaling":true,...}}
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

use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

/// Signals the HMI can write (sensor inputs — physical switches/stalks).
const INPUT_SIGNALS: &[VssPath] = &[
    "Vehicle.LowVoltageSystemState",
    "Body.Switches.Hazard.IsEngaged",
    "Body.Switches.TurnIndicator.Direction",
    "Body.Switches.HighBeam.IsEngaged",
    "Chassis.ParkingBrake.IsEngaged",
    "Body.Lights.LightSwitch",
    "Body.PEPS.KeyPresent",
    "Body.Switches.Keyfob.LockButton",
    "Body.Switches.DoorTrim.Row1.Left.LockButton",
    "Body.Switches.DoorTrim.Row1.Right.LockButton",
    "Body.Switches.DoorTrim.Row2.Left.LockButton",
    "Body.Switches.DoorTrim.Row2.Right.LockButton",
    "Body.Connectivity.RemoteLock",
    "Body.Connectivity.BleLock",
    "Body.Connectivity.NfcCardPresent",
    "Body.Connectivity.NfcPhonePresent",
    "Vehicle.Safety.CrashDetected",
    // Bulb defect fault-injection (HMI toggles to simulate failed lamp).
    // Three physical lamps per side: Front, Side (mirror repeater), Rear.
    "Body.Lights.DirectionIndicator.Left.Lamp.Front.IsDefect",
    "Body.Lights.DirectionIndicator.Left.Lamp.Side.IsDefect",
    "Body.Lights.DirectionIndicator.Left.Lamp.Rear.IsDefect",
    "Body.Lights.DirectionIndicator.Right.Lamp.Front.IsDefect",
    "Body.Lights.DirectionIndicator.Right.Lamp.Side.IsDefect",
    "Body.Lights.DirectionIndicator.Right.Lamp.Rear.IsDefect",
];

/// Signals the bridge pushes back to the HMI (actuator outputs from arbiters).
const OUTPUT_SIGNALS: &[VssPath] = &[
    "Body.Lights.DirectionIndicator.Left.IsSignaling",
    "Body.Lights.DirectionIndicator.Right.IsSignaling",
    "Body.Lights.Hazard.IsSignaling",
    "Body.Lights.Beam.Low.IsOn",
    "Body.Lights.Beam.High.IsOn",
    "Body.Lights.Running.IsOn",
    "Body.Doors.Row1.Left.IsLocked",
    "Body.Doors.Row1.Right.IsLocked",
    "Body.Doors.Row2.Left.IsLocked",
    "Body.Doors.Row2.Right.IsLocked",
    // Plant model outputs — actual lamp state from BlinkRelay.
    // Three physical lamps per side: Front, Side (mirror repeater), Rear.
    "Body.Lights.DirectionIndicator.Left.Lamp.Front.IsOn",
    "Body.Lights.DirectionIndicator.Left.Lamp.Side.IsOn",
    "Body.Lights.DirectionIndicator.Left.Lamp.Rear.IsOn",
    "Body.Lights.DirectionIndicator.Right.Lamp.Front.IsOn",
    "Body.Lights.DirectionIndicator.Right.Lamp.Side.IsOn",
    "Body.Lights.DirectionIndicator.Right.Lamp.Rear.IsOn",
];

/// Shared state snapshot sent to HMI clients.
type StateSnapshot = HashMap<&'static str, serde_json::Value>;

pub struct WsBridge<B: SignalBus> {
    bus: Arc<B>,
    addr: SocketAddr,
}

impl<B: SignalBus> WsBridge<B> {
    pub fn new(addr: SocketAddr, bus: Arc<B>) -> Self {
        Self { bus, addr }
    }

    /// Run the WebSocket server. Spawns a background task that listens
    /// for output signal changes and broadcasts to all connected HMI clients.
    pub async fn run(self) -> anyhow::Result<()> {
        let listener = TcpListener::bind(self.addr).await?;
        tracing::info!(addr = %self.addr, "WebSocket bridge listening");

        // Shared output state + broadcast channel for pushing updates to clients.
        let output_state: Arc<Mutex<StateSnapshot>> = Arc::new(Mutex::new(HashMap::new()));
        let (update_tx, _) = broadcast::channel::<String>(256);

        // Coalesce per-signal updates into a single 10 ms batch so that
        // multi-signal publications (e.g. BlinkRelay toggling three lamps
        // on one side in the same tick) arrive at the HMI as a single
        // snapshot. This prevents the "lamps flip one-by-one" visual
        // stagger caused by sending one websocket message per signal.
        let dirty = Arc::new(Notify::new());

        // Subscriber tasks — update shared state and mark dirty; the
        // broadcaster task handles the debounced snapshot send.
        for &signal in OUTPUT_SIGNALS {
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
        {
            let state = Arc::clone(&output_state);
            let tx = update_tx.clone();
            let dirty = Arc::clone(&dirty);
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
                    let msg = serde_json::json!({ "state": snapshot });
                    let _ = tx.send(msg.to_string());
                }
            });
        }

        // Accept connections
        loop {
            let (stream, peer) = listener.accept().await?;
            tracing::info!(%peer, "HMI client connecting");

            let bus = Arc::clone(&self.bus);
            let output_state = Arc::clone(&output_state);
            let update_rx = update_tx.subscribe();

            tokio::spawn(async move {
                if let Err(e) = handle_connection(stream, bus, output_state, update_rx, peer).await
                {
                    tracing::warn!(%peer, error = %e, "HMI client disconnected");
                }
            });
        }
    }
}

async fn handle_connection<B: SignalBus>(
    stream: TcpStream,
    bus: Arc<B>,
    output_state: Arc<Mutex<StateSnapshot>>,
    mut update_rx: broadcast::Receiver<String>,
    peer: SocketAddr,
) -> anyhow::Result<()> {
    let ws_stream = tokio_tungstenite::accept_async(stream).await?;
    let (mut ws_tx, mut ws_rx) = ws_stream.split();

    tracing::info!(%peer, "HMI client connected");

    // Send current output state immediately on connect.
    {
        let snapshot = output_state.lock().await.clone();
        if !snapshot.is_empty() {
            let msg = serde_json::json!({ "state": snapshot });
            ws_tx.send(Message::Text(msg.to_string().into())).await?;
        }
    }

    loop {
        tokio::select! {
            // HMI → bridge: sensor input
            msg = ws_rx.next() => {
                match msg {
                    Some(Ok(Message::Text(text))) => {
                        handle_hmi_message(&text, &bus).await;
                    }
                    Some(Ok(Message::Close(_))) | None => {
                        tracing::info!(%peer, "HMI client disconnected");
                        break;
                    }
                    Some(Err(e)) => {
                        tracing::warn!(%peer, error = %e, "WebSocket read error");
                        break;
                    }
                    _ => {} // ping/pong/binary — ignore
                }
            }
            // bridge → HMI: output state update
            Ok(json_str) = update_rx.recv() => {
                if ws_tx.send(Message::Text(json_str.into())).await.is_err() {
                    break;
                }
            }
        }
    }

    Ok(())
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
                if f.fract() == 0.0 && f >= 0.0 && f <= 255.0 {
                    SignalValue::Uint8(f as u8)
                } else if f.fract() == 0.0 {
                    SignalValue::Int16(f as i16)
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
