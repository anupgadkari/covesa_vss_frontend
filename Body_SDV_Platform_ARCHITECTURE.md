# VSS Body Controller — Software Architecture

**Platform:** NXP S32G2 HPC · Android Automotive OS · COVESA VSS v4.0  
**Safety:** ASIL-B (body lighting, passive entry) / QM (HMI, application features)  
**Application language:** Rust (L5 feature layer) · C/AUTOSAR (M7 safety monitor)  
**Container runtime:** Podman · Red Hat Enterprise Linux base image  
**IPC boundary:** RPmsg virtio (M7 ↔ A53) · gRPC localhost (container ↔ container)

---

## Table of Contents

1. [Stack Overview](#1-stack-overview)
2. [Layer Definitions](#2-layer-definitions)
3. [Hardware — L1](#3-hardware--l1)
4. [OS / BSP — L2](#4-os--bsp--l2)
5. [Android VHAL — L3](#5-android-vhal--l3)
6. [VSS Middleware — L4](#6-vss-middleware--l4)
7. [Application Layer — L5 (Rust)](#7-application-layer--l5-rust)
   - 7.1 [SignalBus Trait](#71-signalbus-trait)
   - 7.2 [Signal Arbiter](#72-signal-arbiter)
   - 7.3 [Feature State Machines](#73-feature-state-machines)
   - 7.4 [Transport Adapters](#74-transport-adapters)
8. [Web HMI — L6](#8-web-hmi--l6)
9. [Safety Monitor — ASIL-B](#9-safety-monitor--asil-b)
10. [IPC Message Schema](#10-ipc-message-schema)
11. [VSS Signal Overlay](#11-vss-signal-overlay)
12. [Container Topology](#12-container-topology)
13. [PEPS Timing Budget](#13-peps-timing-budget)
14. [State Ownership & Persistency](#14-state-ownership--persistency)
15. [Portability Strategy](#15-portability-strategy)
16. [Code Generation Prompts](#16-code-generation-prompts)

---

## 1. Stack Overview

```
┌──────────────────────────────────────────────────────────┐
│  L6  Web HMI — React · SVG top/side/cockpit views        │
│        WebSocket client · mock VSS signal store           │
├──────────────────────────────────────────────────────────┤
│        ws://  (WebSocket)                                 │
├──────────────────────────────────────────────────────────┤
│  L5  Application layer — Rust · RHEL container           │
│        tokio-tungstenite WS bridge                        │
│        Signal Arbiter · Feature FSMs                      │
│        Kuksa client crate · Android Auto apps             │
├──────────────────────────────────────────────────────────┤
│        gRPC localhost · port 55555                        │
├──────────────────────────────────────────────────────────┤
│  L4  VSS Middleware — COVESA                              │
│        kuksa.val data broker                              │
│        vss-tools · VSS-to-DBC signal mapper               │
├──────────────────────────────────────────────────────────┤
│        VSS signal paths · Vehicle.Body.*                  │
├──────────────────────────────────────────────────────────┤
│  L3  Android VHAL + CarService                            │
│        vehicle.h HAL implementation                       │
│        VehiclePropertyAccess · HAL property store         │
├──────────────────────────────────────────────────────────┤
│        HIDL / AIDL                                        │
├──────────────────────────────────────────────────────────┤
│  L2  OS / BSP — NXP S32G2                                 │
│        Android Automotive OS (AAOS)                       │
│        Linux kernel · SocketCAN · LIN daemon              │
├──────────────────────────────────────────────────────────┤
│        CAN FD 500 kbps · LIN 19.2 kbps                   │
├──────────────────────────────────────────────────────────┤
│  L1  Hardware — NXP S32G2                                 │
│        4× Cortex-A53 (application domain)                 │
│        3× Cortex-M7 (real-time / AUTOSAR)                 │
│        LLCE CAN FD + LIN engine                           │
│        PFE GbE + TSN forwarding                           │
│        Body ECUs · CAN/LIN I/O nodes                      │
└──────────────────────────────────────────────────────────┘
```

All application feature logic, the Signal Arbiter, and the SignalBus trait live in **L5 Rust**. Every layer below L5 is either a concrete adapter implementing that trait, or unmodified platform software. Swapping the SoC (e.g. NXP → Qualcomm) requires replacing only the transport adapter in L5 and the BSP in L2. No feature code changes.

---

## 2. Layer Definitions

| Layer | Name | Language | Safety | Owner |
|-------|------|----------|--------|-------|
| L6 | Web HMI | HTML/CSS/React | QM | App team |
| L5 | Application / bridge | Rust | QM | Platform team |
| L4 | VSS middleware | C++ / Python | QM | COVESA / OEM |
| L3 | Android VHAL | Java / C++ | QM | Google / OEM |
| L2 | OS / BSP | C | QM | NXP / OEM |
| L1 | Hardware | — | ASIL-B (safety paths) | NXP |
| SM | Safety Monitor | C / AUTOSAR CP | ASIL-B | Safety team |

The **Safety Monitor** (SM) is not a numbered layer — it runs on the M7 cores alongside the AUTOSAR stack and crosses the A53/M7 boundary via RPmsg. It is the sole authority for ASIL-B state.

---

## 3. Hardware — L1

### NXP S32G2 SoC

**Application domain (A53 cluster)**
- 4× ARM Cortex-A53 @ 1 GHz
- Runs AAOS + Podman containers
- All QM workloads: VSS broker, Rust bridge, HMI server, Android Auto

**Real-time domain (M7 cluster)**
- 3× ARM Cortex-M7 @ 400 MHz
- Runs AUTOSAR Classic CP
- Safety Monitor partition (ASIL-B)
- PEPS state machine (ASIL-B)
- CAN/LIN frame processing via LLCE

**LLCE (Low Latency Communication Engine)**
- Dedicated hardware engine for CAN FD and LIN
- Handles frame encoding/decoding independently of A53 and M7
- Exposes frames to M7 via shared memory; to A53 via SocketCAN

**PFE (Packet Forwarding Engine)**
- GbE MAC + TSN forwarding
- Used for SOME/IP Ethernet backbone if domain controller topology is adopted

**Body ECU nodes (dumb CAN/LIN peripherals)**
- Drive all physical I/O: lamps, lock actuators, window motors, mirror motors
- Receive CAN FD frames from LLCE; execute actuator commands
- Return sensor state (latch position, lock state, etc.) as CAN frames
- No application logic — pure I/O mapping

---

## 4. OS / BSP — L2

**Android Automotive OS (AAOS)**
- Runs on A53 cluster
- Hosts CarService, MediaSession, VoiceInteraction, and all Android Auto apps
- Podman runs as a userspace process under AAOS — containers do not replace the OS

**Linux kernel (within AAOS)**
- `PREEMPT_RT` patch for improved latency on the A53
- `SocketCAN` driver exposes LLCE CAN FD frames as standard Linux network interface (`can0`)
- LIN daemon bridges LIN frames to userspace via `/dev/linX`

**RPmsg framework**
- Linux virtio RPmsg driver enables shared-memory message passing between A53 and M7
- One RPmsg channel per direction: `vss-cmd` (A53→M7) and `vss-state` (M7→A53)
- Character device exposed as `/dev/rpmsg0` and `/dev/rpmsg1`
- The Rust transport adapter opens these devices directly via `tokio::fs::File`

**SEMA4 hardware semaphores**
- Used internally by the Safety Monitor for NVM write atomicity
- Never exposed above L2; not part of the SignalBus interface

---

## 5. Android VHAL — L3

**Role**
VHAL is the Android abstraction layer between CarService (Java) and vehicle hardware. It defines a property-based interface where every vehicle datum is a typed property with a numeric ID.

**Key interfaces**
```
android.hardware.automotive.vehicle.IVehicle  (AIDL)
  ├── getValues(prop_ids[]) → VehiclePropValue[]
  ├── setValues(VehiclePropValue[])
  └── subscribe(options[]) → callback stream
```

**Important constraint**: AIDL operates only inside the Android Binder IPC domain. It cannot directly address the M7/AUTOSAR partition or the bare Linux SocketCAN interface. The Rust L5 bridge is the adapter that connects VHAL to Kuksa.val and, via RPmsg, to the Safety Monitor.

**ASIL note**: CarService and VHAL are permanently QM. They cannot issue ASIL-B commands. When CarService calls `setValues()` on a safety-relevant property (e.g. headlamp state), the VHAL implementation routes the request to the Rust bridge → Signal Arbiter → RPmsg → Safety Monitor. The Safety Monitor validates and executes. CarService never touches safety state directly.

**Custom property range**: OEM-specific properties use IDs in the range `0x0F400000–0x0F40FFFF` (VENDOR_EXTENSION). VSS signals without a standard VHAL property ID mapping use this range in the VHAL implementation.

---

## 6. VSS Middleware — L4

**COVESA VSS v4.0**  
The Vehicle Signal Specification defines a hierarchical tree of signal paths. Every signal has a path (e.g. `Vehicle.Body.Lights.Beam.Low.IsOn`), a data type, a unit, and a direction (`sensor` or `actuator`).

**kuksa.val data broker**
- gRPC server (port 55555) running in a Podman container on A53
- Stores the current value of every subscribed VSS signal in memory
- Clients (Rust bridge, Android apps, HMI) subscribe via gRPC streaming
- **Not** the state authority for ASIL-B signals — holds a projection only
- On container restart, the Safety Monitor replays current state via RPmsg within ~200 ms

**Signal ID generation**
Run `vspec --export-id` against the combined catalog (base + overlay) to produce stable 32-bit integer IDs for every signal. These IDs are used in the RPmsg wire format and compiled into both the Rust constants file and the AUTOSAR M7 lookup table.

```bash
vspec export json \
  --vspec spec/VehicleSignalSpecification.vspec \
  --output vss_full.json \
  --uuid   # adds stable UUID/ID per signal
```

**VSS-to-DBC mapper**
A build-time tool that generates a `.dbc` CAN database from the VSS overlay. The DBC maps VSS signal paths to CAN frame IDs, bit positions, and scaling factors. This is the artifact that the AUTOSAR M7 CAN stack consumes.

---

## 7. Application Layer — L5 (Rust)

### Crate structure

```
vss-bridge/
├── Cargo.toml
├── build.rs                  # bindgen: vss_ipc_message.h → Rust bindings
├── include/
│   └── vss_ipc_message.h     # canonical IPC schema (shared with AUTOSAR)
├── src/
│   ├── main.rs               # tokio runtime, dependency injection
│   ├── signal_bus.rs         # trait SignalBus — the portability seam
│   ├── arbiter.rs            # Signal Arbiter — per-actuator priority resolution
│   ├── ipc_message.rs        # encode/parse RPmsg wire format
│   ├── signal_ids.rs         # generated: VSS path → u32 ID constants
│   ├── adapters/
│   │   ├── rpmsg.rs          # RpmsgBus: NXP S32G2 (current)
│   │   ├── glink.rs          # GlinkBus: Qualcomm (future)
│   │   ├── someip.rs         # SomeIpBus: Ethernet, most portable
│   │   └── mock.rs           # MockBus: unit tests / CI
│   └── features/
│       ├── hazard_fsm.rs
│       ├── turn_fsm.rs
│       ├── lock_feedback.rs
│       ├── peps_fsm.rs
│       ├── low_beam.rs
│       ├── high_beam.rs
│       ├── drl.rs
│       └── auto_lock.rs
```

### Key Cargo dependencies

```toml
[dependencies]
tokio           = { version = "1", features = ["full"] }
tokio-tungstenite = "0.21"
tonic           = "0.11"          # gRPC client for kuksa.val
prost           = "0.12"          # protobuf codegen
crc             = "3"             # CRC-16/CCITT-FALSE
thiserror       = "1"
serde           = { version = "1", features = ["derive"] }
serde_json      = "1"
async-trait     = "0.1"
bytes           = "1"

[build-dependencies]
bindgen         = "0.69"
tonic-build     = "0.11"
```

---

### 7.1 SignalBus Trait

The portability seam. Every feature FSM and the Signal Arbiter depend only on this trait. No feature imports any transport type.

```rust
// src/signal_bus.rs

use async_trait::async_trait;
use futures::stream::BoxStream;
use crate::ipc_message::SignalValue;

pub type VssPath = &'static str;

#[async_trait]
pub trait SignalBus: Send + Sync + 'static {
    /// Publish an arbitrated actuator value downstream (toward Safety Monitor).
    async fn publish(&self, signal: VssPath, value: SignalValue) -> anyhow::Result<()>;

    /// Subscribe to incoming state updates (Safety Monitor → A53).
    /// Returns a stream of (VssPath, SignalValue) tuples.
    async fn subscribe(&self, signal: VssPath) -> BoxStream<'static, SignalValue>;

    /// Request-response: publish and await the CMD_ACK from Safety Monitor.
    /// Times out after `timeout_ms` milliseconds.
    async fn publish_await_ack(
        &self,
        signal: VssPath,
        value: SignalValue,
        timeout_ms: u64,
    ) -> anyhow::Result<AckResult>;
}

#[derive(Debug)]
pub enum AckResult {
    Ok,
    Vetoed(String),
    Timeout,
}
```

**Design principle**: `publish_await_ack` is used only by safety-relevant features (PEPS, headlamps) that need confirmation the Safety Monitor executed the command. Purely informational writes (e.g. ambient light colour) use `publish` with fire-and-forget semantics.

---

### 7.2 Signal Arbiter

Resolves per-actuator ownership conflicts between features. Each feature publishes `ActuatorRequest` structs tagged with a priority. The Arbiter holds the current highest-priority request per actuator signal and emits arbitrated commands downstream.

**Priority table (hardcoded)**

| Actuator | Feature | Priority |
|----------|---------|----------|
| `Body.Lights.DirectionIndicator.*.IsSignaling` | HazardFsm | 3 (HIGH) |
| `Body.Lights.DirectionIndicator.*.IsSignaling` | TurnFsm | 2 (MEDIUM) |
| `Body.Lights.DirectionIndicator.*.IsSignaling` | LockFeedback | 1 (LOW) |
| `Body.Doors.*.IsLocked` | PepsFsm | 3 (HIGH) |
| `Body.Doors.*.IsLocked` | AutoLock | 2 (MEDIUM) |
| `Body.Lights.Beam.Low.IsOn` | LowBeamFsm | 2 (MEDIUM) |
| `Body.Lights.Beam.High.IsOn` | HighBeamFsm | 2 (MEDIUM) |

**Key invariant**: a feature is only permitted to claim the priority the table assigns to it for that signal. The Safety Monitor independently validates the claim and returns `ACK_ERR_PRIORITY` if the claim does not match its own copy of the table.

```rust
// src/arbiter.rs  (abbreviated)

pub struct ActuatorRequest {
    pub signal:     VssPath,
    pub value:      SignalValue,
    pub priority:   Priority,
    pub feature_id: FeatureId,
}

pub struct SignalArbiter<B: SignalBus> {
    bus:      Arc<B>,
    tx:       mpsc::Sender<ActuatorRequest>,
}

impl<B: SignalBus> SignalArbiter<B> {
    pub fn new(bus: Arc<B>) -> (Self, mpsc::Receiver<ActuatorRequest>) { ... }

    /// Feature state machines call this — fire and forget.
    pub async fn request(&self, req: ActuatorRequest) {
        self.tx.send(req).await.ok();
    }
}

/// Arbiter resolution loop — runs as a tokio task.
async fn arbiter_loop<B: SignalBus>(
    bus:  Arc<B>,
    mut rx: mpsc::Receiver<ActuatorRequest>,
) {
    // Per-signal map: signal path → current winning request
    let mut winners: HashMap<VssPath, ActuatorRequest> = HashMap::new();

    while let Some(req) = rx.recv().await {
        let should_emit = match winners.get(req.signal) {
            None => true,
            Some(current) => req.priority >= current.priority,
        };
        if should_emit {
            winners.insert(req.signal, req.clone());
            bus.publish(req.signal, req.value).await.ok();
        }
    }
}
```

---

### 7.3 Feature State Machines

Each feature is a self-contained `async` struct that holds a reference to the `SignalArbiter`. Features subscribe to relevant input signals via the `SignalBus` and publish requests when their internal state changes. **No feature imports another feature.**

**Example: HazardFsm**

```rust
// src/features/hazard_fsm.rs

pub struct HazardFsm<B: SignalBus> {
    arbiter: Arc<SignalArbiter<B>>,
    bus:     Arc<B>,
}

impl<B: SignalBus> HazardFsm<B> {
    pub async fn run(self) {
        let mut stream = self.bus
            .subscribe("Body.Lights.Hazard.IsSignaling")
            .await;

        while let Some(value) = stream.next().await {
            let active = matches!(value, SignalValue::Bool(true));
            for side in ["Left", "Right"] {
                let path = if side == "Left" {
                    "Body.Lights.DirectionIndicator.Left.IsSignaling"
                } else {
                    "Body.Lights.DirectionIndicator.Right.IsSignaling"
                };
                self.arbiter.request(ActuatorRequest {
                    signal:     path,
                    value:      SignalValue::Bool(active),
                    priority:   Priority::High,   // 3 — wins over Turn and LockFeedback
                    feature_id: FeatureId::Hazard,
                }).await;
            }
        }
    }
}
```

**LED blink waveform note**: `IsSignaling` is a boolean *intent* flag. UN R48-compliant 1–2 Hz blink cadence is implemented in the LED driver IC or body ECU firmware. Feature FSMs never set timers for blink patterns.

**Feature inventory**

| FSM | Inputs (VSS subscriptions) | Outputs (Arbiter requests) |
|-----|---------------------------|---------------------------|
| `HazardFsm` | `Body.Lights.Hazard.IsSignaling` | Both `DirectionIndicator.*.IsSignaling` @ prio 3 |
| `TurnFsm` | `Body.Lights.DirectionIndicator.{L,R}.IsSignaling` | Same signals @ prio 2 |
| `LockFeedback` | `Body.Doors.*.IsLocked` (state change) | Both indicators @ prio 1 (brief sequence) |
| `PepsFsm` | LF antenna event (RPmsg sensor), key auth result | `Body.Doors.*.IsLocked` @ prio 3 |
| `LowBeamFsm` | `Body.Lights.Beam.Low.IsOn` | Same signal @ prio 2 |
| `HighBeamFsm` | `Body.Lights.Beam.High.IsOn` | Same signal @ prio 2 |
| `DrlFsm` | Ignition state, park brake | `Body.Lights.Running.IsOn` @ prio 2 |
| `AutoLock` | Speed signal (`Vehicle.Speed`) | `Body.Doors.*.IsLocked` @ prio 2 |

---

### 7.4 Transport Adapters

Each adapter implements `SignalBus`. Injected at startup in `main.rs`.

**RpmsgBus** (NXP S32G2 — current implementation)
- Opens `/dev/rpmsg0` (cmd) and `/dev/rpmsg1` (state) via `tokio::fs::File`
- Encodes outbound frames using `ActuatorCmd::encode()` from `ipc_message.rs`
- Parses inbound frames using `parse_inbound()` from `ipc_message.rs`
- Maintains a `SeqCounter` per direction; logs gaps in received sequence numbers
- Drives a `tokio::sync::broadcast` channel to fan out state updates to feature subscribers

**GlinkBus** (Qualcomm — future)
- Implements the same `SignalBus` trait over Qualcomm GLINK IPC
- Different device nodes; same encode/parse logic from `ipc_message.rs`
- Feature code and arbiter unchanged

**SomeIpBus** (any SoC — most portable)
- Implements `SignalBus` over SOME/IP on the GbE interface
- Highest latency (~500 µs vs ~100 µs for RPmsg) but zero SoC-specific code
- Preferred for domain controller topologies where the Safety Monitor runs on a separate ECU

**MockBus** (CI / unit tests)
- In-memory broadcast channel; no hardware
- Enables unit testing of every feature FSM and the Arbiter with no hardware dependency
- Records all published signals and exposed as `MockBus::history()` for assertions

**Dependency injection in main.rs**

```rust
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // Swap this one line to change transport:
    let bus = Arc::new(RpmsgBus::new("/dev/rpmsg0", "/dev/rpmsg1").await?);

    let arbiter = Arc::new(SignalArbiter::new(Arc::clone(&bus)));

    // Spawn all feature FSMs
    tokio::spawn(HazardFsm::new(Arc::clone(&arbiter), Arc::clone(&bus)).run());
    tokio::spawn(TurnFsm::new(Arc::clone(&arbiter), Arc::clone(&bus)).run());
    tokio::spawn(LockFeedback::new(Arc::clone(&arbiter), Arc::clone(&bus)).run());
    tokio::spawn(PepsFsm::new(Arc::clone(&arbiter), Arc::clone(&bus)).run());
    // ... rest of features

    // WebSocket bridge toward L6 HMI
    let ws_server = WsServer::new("0.0.0.0:8080", Arc::clone(&bus));
    tokio::spawn(ws_server.run());

    // gRPC client toward kuksa.val at L4
    let kuksa = KuksaClient::connect("http://localhost:55555").await?;
    tokio::spawn(kuksa.sync_loop(Arc::clone(&bus)));

    tokio::signal::ctrl_c().await?;
    Ok(())
}
```

---

## 8. Web HMI — L6

**Technology**: Single-file HTML/React/SVG, no build step required.

**Three views**
- **Top view (XY plane)** — accurate pickup truck silhouette, all lamps, indicators, door open arcs, mirror fold state, wiper animation, ambient cabin glow
- **Side view (XZ plane)** — profile silhouette, headlamps, wipers, sunroof gap, door open arcs, fuel lid
- **Cockpit view** — instrument cluster with telltales, dual-zone HVAC readout, wiper status, horn indicator

**Signal coverage** (50 VSS signals)

| Domain | Signals |
|--------|---------|
| `Body.Lights` | Low/high beam, DRL, parking, front/rear fog, brake, reverse, license plate, left/right indicator, hazard |
| `Body.Windshield` | Front/rear wiper mode (enum), front intensity, front/rear washer |
| `Body.Mirrors` | Fold, heat, tilt, pan (×2 sides) |
| `Body.Doors` | Open, lock, double-lock, child lock, window position, latch status (×4 doors) |
| `Body` | Hood, trunk open/lock, fuel lid, charge lid, horn |
| `Body.Sunroof` | Position, shade position |
| `Cabin.Lights` | Dome, glovebox, ambient intensity, ambient colour |
| `Cabin.HVAC` | A/C, recirculation, front/rear defroster, dual-zone temp, fan speed, vent distribution |
| `Cabin.Seat` | Row 1 driver/passenger heating, ventilation |

**Control types**
- Toggle switch — boolean actuators
- Slider — numeric actuators (position, temperature, fan speed)
- Enum button row — mode signals (wiper mode, vent distribution, latch status)
- Colour picker — ambient light colour
- Door card — compact per-door grid combining all door sub-signals

**Connection model**: currently drives a mock in-memory state store. Connecting to kuksa.val requires replacing `useState` initialisation with a `useEffect` that opens a WebSocket to `ws://localhost:8080` and maps incoming VSS signal updates to state. The Rust WS bridge in L5 handles translation.

---

## 9. Safety Monitor — ASIL-B

Runs as an AUTOSAR Classic partition on M7 cores. ASIL-B certified per ISO 26262.

**Responsibilities**
1. Receive `ACTUATOR_CMD` messages from A53 via RPmsg
2. Validate: magic, version, CRC, priority claim vs. internal table, vehicle state constraints
3. If valid: write to ASIL-B NVM, forward to AUTOSAR COM stack → LLCE → CAN FD → body ECU
4. If invalid: return `CMD_ACK` with appropriate error code; do not forward
5. On every committed state change: push `STATE_UPDATE` to A53
6. Monitor body ECU sensor frames; push `FAULT_REPORT` on open-circuit, short, or timeout
7. **Hold-last on fault**: if a CAN/LIN node stops responding, hold the last known safe state in NVM rather than defaulting to off

**ASIL-B NVM**
- Dedicated NVM partition, separate from QM AAOS storage
- Atomic writes via SEMA4 hardware semaphore
- Every write includes a CRC32 over the full state record
- Boot sequence reads NVM, validates CRC; if CRC fails, applies safe-state defaults and logs DEM event
- After M7 restores from NVM on boot, it pushes all signals as `STATE_UPDATE` to A53 to re-populate kuksa.val

**PEPS (Passive Entry/Passive Start) path**
- LF antenna interrupt → M7 GPIO ISR (< 1 ms)
- M7 initiates key challenge/response over LF (NXP NCJ29D5 protocol)
- Key authentication complete → M7 drives lock actuator via LLCE/LIN directly
- M7 pushes `STATE_UPDATE` for `Body.Doors.*.IsLocked` and `Body.Doors.*.LatchStatus`
- A53 is notified *after* unlock; not in the critical path
- Total M7-side latency: < 80 ms (well within 200 ms budget)

---

## 10. IPC Message Schema

Defined in `include/vss_ipc_message.h`. Consumed by AUTOSAR C (direct include) and Rust (via `bindgen` at build time).

### Wire format

All messages share a 16-byte header:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 4 B | `magic` | `0xBCC01A00` — identity + version guard |
| 4 | 1 B | `version` | Schema version (currently 1) |
| 5 | 1 B | `msg_type` | `ACTUATOR_CMD` / `STATE_UPDATE` / `CMD_ACK` / `FAULT_REPORT` |
| 6 | 2 B | `seq` | Wrapping u16 per direction; receiver detects gaps |
| 8 | 4 B | `timestamp_ms` | SoC uptime in ms |
| 12 | 4 B | `signal_id` | Stable 32-bit ID from `vspec2id` |

### Message types

**`VssActuatorCmd`** (A53 → M7, 28 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 16 | 1 B | `feature_id` | Which feature is requesting |
| 17 | 1 B | `priority` | 1=LOW, 2=MEDIUM, 3=HIGH |
| 18 | 1 B | `sig_type` | Type tag for the value union |
| 19 | 1 B | `_pad0` | Must be 0 (counted in CRC) |
| 20 | 4 B | `value` | Tagged union (bool/u8/i16/u16/f32) |
| 24 | 2 B | `crc16` | CRC-16/CCITT-FALSE over bytes [0..24) |
| 26 | 2 B | `_pad1` | Must be 0 |

**`VssStateUpdate`** (M7 → A53, 28 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 16 | 1 B | `sig_type` | Type tag |
| 17 | 1 B | `last_feature` | FeatureId of last committer |
| 18 | 2 B | `_pad0` | Must be 0 |
| 20 | 4 B | `value` | Current committed value |
| 24 | 2 B | `crc16` | CRC-16/CCITT-FALSE over bytes [0..24) |
| 26 | 2 B | `_pad1` | Must be 0 |

**`VssCmdAck`** (M7 → A53, 24 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 16 | 2 B | `ack_seq` | seq of the command being acknowledged |
| 18 | 1 B | `status` | `OK` / `ERR_SAFETY` / `ERR_PRIORITY` / `ERR_STATE` / `ERR_CHECKSUM` / `ERR_VERSION` / `ERR_UNKNOWN_SIG` |
| 19 | 1 B | `_pad0` | Must be 0 |
| 20 | 2 B | `crc16` | CRC over bytes [0..20) |
| 22 | 2 B | `_pad1` | Must be 0 |

**`VssFaultReport`** (M7 → A53, 24 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 16 | 1 B | `fault_code` | Lamp open circuit / short / actuator timeout / sensor lost / NVM error |
| 17 | 1 B | `severity` | `WARNING` (0) / `CRITICAL` (1) |
| 18 | 2 B | `_pad0` | Must be 0 |
| 20 | 2 B | `crc16` | CRC over bytes [0..20) |
| 22 | 2 B | `_pad1` | Must be 0 |

### CRC rule
CRC-16/CCITT-FALSE (IBM 3740, initial value 0xFFFF) over all bytes from offset 0 to `sizeof(message) - 4`. The CRC field and trailing pad are the last 4 bytes of every message and are excluded from CRC computation.

---

## 11. VSS Signal Overlay

Standard COVESA VSS v4.0 `SingleDoor.vspec` defines only `IsOpen` and `IsLocked`. The following overlay extends each door instance with signals required for latch state reporting and regulatory compliance.

**File**: `overlay/Body/DoorExtended.vspec`

```yaml
# Latch mechanical position — required for door-ajar logic and cinch confirmation
Vehicle.Cabin.Door.Row1.DriverSide.LatchStatus:
  datatype: string
  type: sensor
  allowed: ['UNLATCHED', 'HALF_LATCHED', 'FULL_LATCHED']
  description: >
    Mechanical position of the door striker latch.
    HALF_LATCHED indicates the door is closed but not fully engaged.
    The Safety Monitor uses FULL_LATCHED to confirm cinch sequence completion.

Vehicle.Cabin.Door.Row1.DriverSide.IsDoubleLocked:
  datatype: boolean
  type: sensor
  description: >
    True when the door is in double-lock (deadlock) state.
    In double-lock, the interior handle is mechanically disabled.
    Required for regulatory reporting in several EU markets.

Vehicle.Cabin.Door.Row1.DriverSide.IsChildLockActive:
  datatype: boolean
  type: actuator
  description: >
    True when the child lock is engaged on the rear interior door release.
    Applies only to Row2 doors; defined on all instances for schema uniformity.
```

Instance the above across `Row[1,2]` × `["DriverSide", "PassengerSide"]` using the VSS overlay mechanism. Run `vspec2id` after including the overlay to generate stable signal IDs for the new signals.

---

## 12. Container Topology

All containers run on the A53 cluster under Podman (rootless, daemonless). AAOS is the host OS; Podman is a userspace process within it.

```
A53 cluster (Cortex-A53 ×4)
└── Android Automotive OS
    └── Podman (rootless)
        ├── kuksa-val          RHEL base  gRPC :55555
        ├── vss-bridge         RHEL base  WS :8080  gRPC client
        └── hmi-server         RHEL base  HTTP :3000  static files

    └── CarService / VHAL      (AAOS process, not containerised)
    └── Android Auto apps      (AAOS process, not containerised)
```

**Inter-container communication**: all on `localhost`. gRPC between `vss-bridge` and `kuksa-val` stays on loopback — sub-millisecond, no TLS required (same host, same trust domain).

**A53 ↔ M7 boundary**: `/dev/rpmsg0` and `/dev/rpmsg1` are bind-mounted into the `vss-bridge` container. No other container touches the RPmsg devices.

**Container images**: built from `registry.redhat.io/ubi9/ubi-minimal` base. Compiled Rust binary is statically linked (musl target) and copied into a minimal image with no runtime dependencies beyond the binary itself.

**OTA update flow**: container image tags are promoted through a CI pipeline (OpenShift Pipelines or Tekton). On the vehicle, Podman pulls the new image, runs a canary validation step (smoke-test the WS and gRPC endpoints), then atomically replaces the running container. The Safety Monitor is not updated via this path — it follows the AUTOSAR SWC update process.

---

## 13. PEPS Timing Budget

Passive Entry target: **< 200 ms** from key LF wakeup to lock actuator energised.

| Step | Owner | Time |
|------|-------|------|
| LF antenna wakeup → M7 GPIO ISR | M7 hardware | < 1 ms |
| Key challenge/response (LF protocol) | M7 AUTOSAR / NXP NCJ29D5 | ~40 ms |
| Lock actuator command via LLCE/LIN | M7 → LLCE → LIN | ~10 ms |
| LIN actuator response / confirmation | Body ECU → LLCE → M7 | ~10 ms |
| **Total M7-side latency** | | **~61 ms** |
| NVM write (ASIL-B state commit) | M7 Safety Monitor | ~5 ms |
| STATE_UPDATE push to A53 (notification) | RPmsg | ~0.2 ms |
| Kuksa.val signal update (HMI indication) | A53 gRPC | ~2 ms |

The A53/AAOS/Kuksa stack is entirely **out of the critical path**. The 200 ms budget is met with ~139 ms of margin. The A53 is notified after the unlock event for state sync and HMI indication only.

---

## 14. State Ownership & Persistency

### Two-tier model

**ASIL-B state (Safety Monitor — M7)**
- Authority for: all lamp states, all door lock states, latch states, horn
- Persisted in ASIL-B NVM: atomic write, CRC32 checksum, validated on every boot
- On boot: NVM restore → validate CRC → push all signals as `STATE_UPDATE` to A53
- On Kuksa.val restart: Safety Monitor receives reconnection event → replays all current state
- **Hold-last on fault**: CAN/LIN node loss does not cause state transition to off

**QM state (kuksa.val — A53)**
- Projection only — not authority for any ASIL signal
- Receives state from Safety Monitor via RPmsg → Rust bridge → gRPC
- HMI and Android apps read from here
- May be restarted, updated, or cleared without affecting physical actuator state

### Why Kuksa.val must not own ASIL state

1. **No certified NVM**: kuksa.val writes to a standard Linux file system with no CRC, no atomic guarantee, no recovery time bound
2. **Restart gap**: container restart takes 200 ms–2 s. Headlamps on a moving vehicle cannot have a state gap
3. **ISO 26262 boundary**: software without an ASIL certification cannot be the authority for an ASIL-B function
4. **Recovery undefined**: there is no standard mechanism for kuksa.val to know what state it should replay after restart without an authoritative source

### Actuator ownership conflicts

Multiple features can request the same physical actuator simultaneously (e.g. both HazardFsm and TurnFsm want to control direction indicator lamps). The Signal Arbiter resolves this by priority. The Safety Monitor validates the priority claim independently. Neither layer modifies the other's priority table — the table is compiled into both sides at build time from a shared JSON source-of-truth.

---

## 15. Portability Strategy

The architecture is designed so that moving from NXP S32G2 to another SoC (Qualcomm SA8775P, Renesas R-Car S4, TI TDA4, etc.) requires changes in exactly two places:

1. **L2 BSP** — kernel drivers, device tree, SocketCAN config, RPmsg channel names
2. **One adapter file in L5** — `src/adapters/rpmsg.rs` replaced by `src/adapters/glink.rs` (Qualcomm) or similar; `main.rs` changes one `Arc::new(...)` line

**Everything above the trait boundary is unchanged:**
- All feature FSMs
- Signal Arbiter and priority table
- IPC message schema (reused verbatim)
- VSS signal IDs and overlay
- kuksa.val broker
- Android VHAL implementation
- Web HMI

**The `SignalBus` trait is the portability contract.** Every ecosystem has an equivalent construct (Java interface, C++ pure virtual, Go interface, AIDL IVehicle). The concept is transport-agnostic by design.

**SOME/IP as the maximally portable option**: if the next SoC does not have heterogeneous cores (i.e. no M7 equivalent), the Safety Monitor can run on a separate MCU connected via Ethernet. `SomeIpBus` implements `SignalBus` over SOME/IP; feature code is unchanged. Latency increases from ~100 µs (RPmsg) to ~500 µs (GbE), which is acceptable for body domain functions outside the PEPS critical path.

---

## 16. Code Generation Prompts

Use these prompts with this document as context to generate each component. Paste the relevant architecture sections alongside the prompt for best results.

---

### Prompt 1 — SignalBus trait and MockBus

```
You are implementing the SignalBus trait for a Rust automotive body controller.

Context:
- The crate is `vss-bridge`, running on a Cortex-A53 under Android Automotive OS
- `SignalBus` is the portability seam between feature logic and transport (RPmsg, GLINK, SOME/IP)
- `VssPath` is `&'static str`
- `SignalValue` is an enum: Bool(bool), Uint8(u8), Int16(i16), Uint16(u16), Float(f32)
- Use `async-trait = "0.1"` and `tokio` runtime
- `subscribe()` returns a `BoxStream<'static, SignalValue>` (from futures crate)
- `publish_await_ack()` must time out after a configurable number of milliseconds

Implement:
1. The full `SignalBus` trait in `src/signal_bus.rs`
2. `MockBus` in `src/adapters/mock.rs`:
   - Uses `tokio::sync::broadcast` internally
   - Records all published (signal, value) pairs in a thread-safe history vec
   - `MockBus::history() -> Vec<(VssPath, SignalValue)>` for test assertions
   - `MockBus::inject(signal, value)` to simulate incoming state updates
   - Implement `publish_await_ack` to return `AckResult::Ok` immediately (no hardware)
3. Unit tests in the same file covering publish, subscribe, and ack behaviour
```

---

### Prompt 2 — Signal Arbiter

```
You are implementing the Signal Arbiter for a Rust automotive body controller.

Context (from architecture doc):
- The arbiter sits between feature FSMs and the SignalBus
- Features publish `ActuatorRequest` structs with { signal, value, priority, feature_id }
- The arbiter holds a per-signal "current winner" map
- A new request replaces the winner if its priority >= current winner's priority
- The arbiter is generic over `B: SignalBus`
- All inter-task communication uses `tokio::sync::mpsc`

Priority enum: Low=1, Medium=2, High=3
FeatureId enum: Peps=0x01, Hazard=0x02, TurnIndicator=0x03, LowBeam=0x04,
  HighBeam=0x05, Drl=0x06, AutoLock=0x07, LockFeedback=0x08, Welcome=0x09

Implement:
1. `ActuatorRequest` struct
2. `SignalArbiter<B: SignalBus>` struct with a `request()` method
3. `arbiter_loop()` async function that runs as a tokio task
4. Unit tests using MockBus covering:
   - High priority wins over low priority for the same signal
   - Two different signals do not interfere
   - Priority tie: latest request wins
   - Releasing a high-priority hold (setting to false) allows lower priority to retake
```

---

### Prompt 3 — RpmsgBus transport adapter

```
You are implementing the RpmsgBus transport adapter for a Rust automotive body controller.

Context:
- Target: NXP S32G2, Linux kernel, RPmsg character devices `/dev/rpmsg0` (cmd out) and `/dev/rpmsg1` (state in)
- `RpmsgBus` implements the `SignalBus` trait
- Outbound: encode `ActuatorCmd` using the schema in `ipc_message.rs`, write 28 bytes to `/dev/rpmsg0`
- Inbound: read from `/dev/rpmsg1` in a background tokio task, parse with `parse_inbound()`, fan out via `tokio::sync::broadcast`
- Sequence counter: one `SeqCounter` for outbound, detect gaps in inbound seq
- `publish_await_ack`: send cmd, await matching `CmdAck` by `ack_seq`, timeout after configurable ms
- Log dropped frames (seq gap > 1) at WARN level using the `tracing` crate
- The RPmsg device is a character device: use `tokio::io::AsyncReadExt` and `AsyncWriteExt`

Implement:
1. `RpmsgBus` struct and `impl SignalBus for RpmsgBus`
2. Background reader task that parses `VssStateUpdate`, `VssCmdAck`, and `VssFaultReport`
3. A `pending_acks: HashMap<u16, oneshot::Sender<AckResult>>` for correlating ack responses
4. Error handling: device not found, read/write errors, CRC failures
5. Integration test skeleton that opens a pair of Unix pipes in place of the RPmsg devices
```

---

### Prompt 4 — Feature FSMs (full set)

```
You are implementing all body feature state machines for a Rust automotive body controller.

Context:
- Each FSM is an async struct holding Arc<SignalArbiter<B>> and Arc<B: SignalBus>
- FSMs subscribe to input signals, compute state transitions, publish ActuatorRequests
- No FSM imports another FSM
- LED blink waveform is NOT the FSM's responsibility — it sets boolean intent only
- Priority assignments (hardcoded, matches Safety Monitor's table):
    HazardFsm          → DirectionIndicator.*.IsSignaling    @ HIGH (3)
    TurnFsm            → DirectionIndicator.{Left,Right}     @ MEDIUM (2)
    LockFeedback       → DirectionIndicator.*.IsSignaling    @ LOW (1)
    PepsFsm            → Doors.*.IsLocked                    @ HIGH (3)
    AutoLock           → Doors.*.IsLocked                    @ MEDIUM (2)
    LowBeamFsm         → Lights.Beam.Low.IsOn                @ MEDIUM (2)
    HighBeamFsm        → Lights.Beam.High.IsOn               @ MEDIUM (2)
    DrlFsm             → Lights.Running.IsOn                 @ MEDIUM (2)

Implement all 8 FSMs, each in its own file under src/features/.
For LockFeedback: on any IsLocked state change event, publish a HIGH request to both indicators
for 500 ms, then release (publish LOW priority false). Use tokio::time::sleep for the duration.
For PepsFsm: subscribe to a synthetic signal `Body.PEPS.KeyPresent` (bool) as a stand-in for
the LF antenna interrupt; unlock all four doors when true.
Include unit tests using MockBus for each FSM covering the primary state transitions.
```

---

### Prompt 5 — Safety Monitor (AUTOSAR C)

```
You are implementing the Safety Monitor for an AUTOSAR Classic CP application on a Cortex-M7.

Context:
- Runs as an ASIL-B certified AUTOSAR SWC partition
- Communicates with A53 via RPmsg character devices (Linux side: /dev/rpmsg0 and /dev/rpmsg1)
- Shared header: vss_ipc_message.h (defines VssActuatorCmd, VssStateUpdate, VssCmdAck, VssFaultReport)
- ASIL-B NVM is accessed via a provided BSP function: Nvm_Write(block_id, data, len) and Nvm_Read(...)
- SEMA4 semaphore for atomic NVM access: Sema4_Lock(id) and Sema4_Unlock(id)

Priority table (must match Rust arbiter):
  FEAT_HAZARD(0x02) → signals 0x1001,0x1002 → priority 3
  FEAT_TURN(0x03)   → signals 0x1001,0x1002 → priority 2
  FEAT_LOCK_FB(0x08)→ signals 0x1001,0x1002 → priority 1
  FEAT_PEPS(0x01)   → signals 0x2001-0x2004  → priority 3
  FEAT_AUTOLOCK(0x07)→ signals 0x2001-0x2004 → priority 2

Implement in C (C99):
1. `safety_monitor_init()` — read NVM, validate CRC32, push all STATE_UPDATE to A53
2. `safety_monitor_rx_task()` — read VssActuatorCmd from RPmsg, validate, execute or veto
3. `validate_cmd()` — check magic, version, CRC-16, priority claim against table, vehicle state constraints
4. `commit_state()` — write to NVM with SEMA4 lock, send STATE_UPDATE to A53
5. `send_ack()` — build and write VssCmdAck with appropriate status
6. `hold_last_on_fault()` — called when CAN node timeout is detected; do not change NVM state
7. CRC-16/CCITT-FALSE implementation (no external library — inline 256-entry lookup table)
Include MISRA-C:2012 compatible code style. No dynamic allocation. No recursion.
```

---

### Prompt 6 — kuksa.val ↔ vss-bridge gRPC sync

```
You are implementing the Kuksa.val synchronisation loop for the Rust vss-bridge service.

Context:
- kuksa.val runs in a sibling container at grpc://localhost:55555
- Use the `kuksa-client` Rust crate (or raw `tonic` with kuksa proto definitions)
- The sync loop has two responsibilities:
    1. INBOUND: subscribe to all Body.* and Cabin.* VSS signals in kuksa.val;
       on change, forward to SignalBus::publish() so features react
    2. OUTBOUND: subscribe to SignalBus state updates from Safety Monitor;
       on receipt, write the new value to kuksa.val via SetValues RPC
- Handle kuksa.val disconnection gracefully: retry with exponential backoff (1s, 2s, 4s, max 30s)
- On reconnect: request a full state snapshot from Safety Monitor and replay to kuksa.val
- The struct is `KuksaSync<B: SignalBus>` with a `run()` async method

Implement:
1. `KuksaSync<B>` struct and `run()` method
2. Inbound subscription loop with reconnect logic
3. Outbound state update forwarding
4. A `signal_path_to_kuksa_id()` helper that maps VSS path strings to kuksa entry IDs
5. Unit test using MockBus and a mock gRPC server (use `tonic` test utilities)
```

---

### Prompt 7 — WebSocket bridge (HMI ↔ L5)

```
You are implementing the WebSocket server in the Rust vss-bridge for the Web HMI client.

Context:
- Listens on ws://0.0.0.0:8080
- Uses tokio-tungstenite = "0.21"
- Message format: JSON  {"path": "Body.Lights.Beam.Low.IsOn", "value": true}
  (same for both directions — HMI → server and server → HMI)
- On connection: send a full state snapshot of all known signals from SignalBus
- On message from HMI: parse JSON, resolve to SignalValue, publish to SignalArbiter
- On SignalBus state update: broadcast JSON message to all connected HMI clients
- Support multiple concurrent HMI clients (e.g. tablet + phone simultaneously)
- The struct is `WsServer<B: SignalBus>` with a `run()` method

Implement:
1. `WsServer<B>` struct
2. Connection handler: send snapshot, then bidirectional relay
3. Broadcast to all connected clients on state update using `tokio::sync::broadcast`
4. JSON serialisation/deserialisation of VSS path + SignalValue pairs
5. Graceful handling of client disconnect
6. Unit test: connect two mock clients, verify both receive a broadcast
```

---

### Prompt 8 — Build system and container setup

```
You are setting up the build and container infrastructure for the vss-bridge Rust service.

Context:
- Target: aarch64-unknown-linux-musl (statically linked, for RHEL UBI minimal container)
- Cross-compilation from x86_64 dev machine using cross-rs
- build.rs must run bindgen on include/vss_ipc_message.h targeting thumbv7em-none-eabihf
  (for consistent C type sizes matching the M7)
- Container base: registry.redhat.io/ubi9/ubi-minimal
- Podman, not Docker

Produce:
1. `Cargo.toml` with all dependencies from the architecture doc
2. `build.rs` using bindgen with allowlist for Vss* types and VSS_* constants
3. `Cross.toml` configuring cross-rs for aarch64-unknown-linux-musl
4. `Containerfile` (Podman):
   - Stage 1: cargo build --release --target aarch64-unknown-linux-musl
   - Stage 2: ubi9-minimal, copy binary, set non-root USER, EXPOSE 8080
   - Label with VSS schema version and git SHA
5. `podman-compose.yml` defining three services: kuksa-val, vss-bridge, hmi-server
   with correct port mappings, volume mounts (/dev/rpmsg0, /dev/rpmsg1 for vss-bridge),
   and restart policies
6. A `justfile` (just task runner) with targets:
   build, test, clippy, container-build, container-run, generate-signal-ids
```

---

*Document version: 1.0 — Architecture as of April 2026*  
*VSS base: COVESA VSS v4.0 + DoorExtended overlay*  
*IPC schema: vss_ipc_message.h v1 (magic 0xBCC01A00)*
