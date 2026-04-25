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
    // Diagnostic overrides (DoorCard direct-write).
    "Body.Doors.Row1.Left.IsOpen",
    "Body.Doors.Row1.Right.IsOpen",
    "Body.Doors.Row2.Left.IsOpen",
    "Body.Doors.Row2.Right.IsOpen",
    // Direct trunk open/close override (control panel and sensor page).
    "Body.Trunk.IsOpen",
    "Body.Doors.Row1.Left.IsDoubleLocked",
    "Body.Doors.Row1.Right.IsDoubleLocked",
    "Body.Doors.Row2.Left.IsDoubleLocked",
    "Body.Doors.Row2.Right.IsDoubleLocked",
    // Thumb-pad lock inputs — Row 1 outside handle lock areas (HMI top-view).
    "Body.Doors.Row1.Left.Handle.Outside.LockPad.IsPressed",
    "Body.Doors.Row1.Right.Handle.Outside.LockPad.IsPressed",
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
];

/// Shared state snapshot sent to HMI clients.
type StateSnapshot = HashMap<&'static str, serde_json::Value>;

pub struct WsBridge<B: SignalBus> {
    bus: Arc<B>,
    addr: SocketAddr,
    platform_config: Arc<PlatformConfig>,
}

impl<B: SignalBus> WsBridge<B> {
    pub fn new(addr: SocketAddr, bus: Arc<B>, platform_config: Arc<PlatformConfig>) -> Self {
        Self {
            bus,
            addr,
            platform_config,
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
) -> anyhow::Result<()> {
    let ws_stream = tokio_tungstenite::accept_async(stream).await?;
    let (mut ws_tx, mut ws_rx) = ws_stream.split();

    tracing::info!(%peer, "HMI client connected");

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
