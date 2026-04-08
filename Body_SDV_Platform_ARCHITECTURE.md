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
   - 7.3 [Feature Business Logic](#73-feature-business-logic)
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
│        Signal Arbiter · Feature Business Logic             │
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
| L6 | Web HMI (optional diagnostics) | HTML/CSS/React | QM | OEM app team |
| L5 | Application / bridge | Rust | QM | OEM platform team |
| L4 | VSS middleware | C++ / Python | QM | COVESA / OEM |
| L3 | Android VHAL | Java / C++ | QM | Google / OEM |
| L2 | OS / BSP | C | QM | Platform Provider / OEM |
| L1 | Hardware (schematics, PCB) | — | ASIL-B (safety paths) | Platform Provider |
| SM | Safety Monitor | C / AUTOSAR CP | ASIL-B | Platform Provider (or OEM) |
| ASIL App | AUTOSAR Application SWCs | C / AUTOSAR CP | ASIL-B | OEM (optional) |

The **Safety Monitor** (SM) is not a numbered layer — it runs on the M7 cores alongside the AUTOSAR stack and crosses the A53/M7 boundary via RPmsg. It is the sole authority for ASIL-B state. The **Platform Provider** delivers the Classic AUTOSAR BSW, Linux distribution, hardware schematics/PCB layout, and a reference Safety Monitor. The OEM may optionally develop ASIL-B Application Layer SWCs on the M7 as in-house competency grows.

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
├── build.rs                  # tonic-build: kuksa.val proto → gRPC client stubs
├── proto/
│   └── kuksa/val/v1/
│       ├── val.proto         # kuksa.val gRPC service definition
│       └── types.proto       # kuksa.val data types (DataEntry, Datapoint, etc.)
├── overlay/
│   └── Body/
│       └── SwitchInputs.vspec  # overlay: physical switch/stalk sensor signals
├── src/
│   ├── main.rs               # tokio runtime, dependency injection
│   ├── signal_bus.rs         # trait SignalBus — the portability seam
│   ├── arbiter.rs            # Signal Arbiter — per-actuator priority resolution
│   ├── ipc_message.rs        # pure Rust IPC wire format (replaces C header + bindgen)
│   ├── signal_ids.rs         # VSS path ↔ u32 ID constants (86 signals)
│   ├── kuksa_sync.rs         # bidirectional gRPC sync with kuksa.val databroker
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
prost           = "0.12"          # protobuf runtime
prost-types     = "0.12"          # well-known protobuf types (Timestamp)
crc             = "3"             # CRC-16/CCITT-FALSE
thiserror       = "1"
serde           = { version = "1", features = ["derive"] }
serde_json      = "1"
async-trait     = "0.1"
bytes           = "1"
futures         = "0.3"
tokio-stream    = { version = "0.1", features = ["sync"] }
anyhow          = "1"
tracing         = "0.1"
tracing-subscriber = { version = "0.3", features = ["env-filter"] }

[build-dependencies]
tonic-build     = "0.11"
```

---

### 7.1 SignalBus Trait

The portability seam. Every feature module and the Signal Arbiter depend only on this trait. No feature imports any transport type.

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

### 7.2 Domain Arbiters

Resolves per-actuator ownership conflicts between features. Two arbiter patterns coexist:

1. **DomainArbiter** (Lighting, Horn, Comfort) — instant per-signal priority resolution. A new request replaces the winner if its priority ≥ the current winner's priority.
2. **DoorLockArbiter** — serialized command queue with ACK handshake (see below).

Each domain carries a **static allow-list**. A request is rejected locally if the feature is not in the allow-list — before it reaches the Safety Monitor. The Safety Monitor independently validates the claim.

**Domain: Lighting** (DomainArbiter — instant priority resolution)

| Actuator | Feature | Priority |
|----------|---------|----------|
| `Body.Lights.DirectionIndicator.*.IsSignaling` | Hazard | 3 (HIGH) |
| `Body.Lights.DirectionIndicator.*.IsSignaling` | TurnIndicator | 2 (MEDIUM) |
| `Body.Lights.DirectionIndicator.*.IsSignaling` | LockFeedback | 3 (HIGH, overlay) |
| `Body.Lights.Hazard.IsSignaling` | Hazard | 3 (HIGH) |
| `Body.Lights.Beam.Low.IsOn` | LowBeam | 2 (MEDIUM) |
| `Body.Lights.Beam.High.IsOn` | HighBeam | 2 (MEDIUM) |
| `Body.Lights.Running.IsOn` | Drl | 2 (MEDIUM) |

**Domain: DoorLock** (DoorLockArbiter — serialized queue)

The lock motor takes ~300 ms per operation and cannot accept concurrent commands. Instead of instant priority resolution, the DoorLock arbiter uses a **one-deep command queue** (active + pending) with ACK handshake:

- **Idle → dispatch immediately.** Request becomes the active operation.
- **Active in progress → queue.** New request replaces the pending slot (latest wins).
- **ACK received → promote.** Pending moves to active and dispatches.
- **Crash unlock protection:** A CrashUnlock pending request cannot be replaced. After a CrashUnlock dispatches, a 10-second lockout rejects all new requests (prevents debris/spurious signals from re-locking during a collision).

The Classic AUTOSAR Locking SWC acknowledges completion by publishing an incrementing event number, the last requestor (FeatureId), and per-door lock status. It also persists the last 5 requestor/status entries in NVM for diagnostic readout. The Rust arbiter has no NVM responsibility.

| Feature | Commands |
|---------|----------|
| KeyfobPeps | UNLOCK, LOCK |
| AutoLock | LOCK |
| DoorTrimButton | LOCK, UNLOCK |
| KeyfobRke | LOCK, UNLOCK, DOUBLE_LOCK |
| PhoneApp | LOCK, UNLOCK |
| PhoneBle | LOCK, UNLOCK |
| NfcCard | LOCK, UNLOCK |
| NfcPhone | LOCK, UNLOCK |
| AutoRelock | LOCK (45s timeout after unlock if no door opened) |
| CrashUnlock | UNLOCK (protected, triggers 10s lockout) |

**Domain: Horn** — single feature today, pass-through with allow-list validation.

**Domain: Comfort** — seat heating/ventilation, HVAC, cabin lights, sunroof. No contention today. Adding a second feature to any comfort actuator requires only an allow-list entry.

**Key invariant**: a feature is only permitted to claim the priority the allow-list assigns to it for that signal (Lighting/Horn/Comfort) or be registered in the DoorLock allow-list. Unauthorized requests are rejected silently. This is validated both in the application arbiter and independently in the Safety Monitor.

```rust
// src/arbiter.rs  (abbreviated)

pub struct ActuatorRequest {
    pub signal:     VssPath,
    pub value:      SignalValue,
    pub priority:   Priority,
    pub feature_id: FeatureId,
}

pub struct AllowEntry {
    pub feature_id: FeatureId,
    pub signal:     VssPath,
    pub priority:   Priority,
}

pub struct DomainArbiter {
    pub name: &'static str,
    tx:       mpsc::Sender<ActuatorRequest>,
}

impl DomainArbiter {
    /// Create a new domain arbiter. Returns the handle and a future to spawn.
    pub fn new<B: SignalBus>(
        name: &'static str,
        allow_list: Vec<AllowEntry>,
        bus: Arc<B>,
    ) -> (Self, impl Future<Output = ()>) { ... }

    /// Feature business logic calls this — fire and forget.
    pub async fn request(&self, req: ActuatorRequest) -> anyhow::Result<()> {
        self.tx.send(req).await?;
        Ok(())
    }
}

// Factory functions for each domain:
pub fn lighting_arbiter<B: SignalBus>(bus: Arc<B>)   -> (DomainArbiter, impl Future<Output = ()>);
pub fn door_lock_arbiter<B: SignalBus>(bus: Arc<B>)  -> (DomainArbiter, impl Future<Output = ()>);
pub fn horn_arbiter<B: SignalBus>(bus: Arc<B>)       -> (DomainArbiter, impl Future<Output = ()>);
pub fn comfort_arbiter<B: SignalBus>(bus: Arc<B>)    -> (DomainArbiter, impl Future<Output = ()>);
```

**Wiring in main.rs:**

```rust
let (lighting_arb, lighting_fut) = arbiter::lighting_arbiter(Arc::clone(&bus));
let (door_lock_arb, door_lock_fut) = arbiter::door_lock_arbiter(Arc::clone(&bus));
let (horn_arb, horn_fut) = arbiter::horn_arbiter(Arc::clone(&bus));
let (comfort_arb, comfort_fut) = arbiter::comfort_arbiter(Arc::clone(&bus));

tokio::spawn(lighting_fut);
tokio::spawn(door_lock_fut);
tokio::spawn(horn_fut);
tokio::spawn(comfort_fut);

// Feature modules receive Arc<DomainArbiter> for their domain:
tokio::spawn(HazardFsm::new(Arc::clone(&lighting_arb), Arc::clone(&bus)).run());
tokio::spawn(KeyfobPepsFsm::new(Arc::clone(&door_lock_arb), Arc::clone(&bus)).run());
```

---

### 7.3 Feature Business Logic

Each feature is a self-contained `async` module that holds a reference to the `SignalArbiter`. Features subscribe to relevant input signals via the `SignalBus` and publish requests when their internal state changes. **No feature imports another feature.** Feature implementations may range from simple state machines (e.g., HazardFsm) to complex algorithms — the architecture does not constrain the implementation approach.

**Example: HazardFsm**

```rust
// src/features/hazard_fsm.rs

pub struct HazardFsm<B: SignalBus> {
    arbiter: Arc<SignalArbiter<B>>,
    bus:     Arc<B>,
}

impl<B: SignalBus> HazardFsm<B> {
    pub async fn run(self) {
        // Subscribe to the physical hazard SWITCH input (overlay sensor signal),
        // NOT the actuator output — prevents feedback loops.
        let mut stream = self.bus
            .subscribe("Body.Switches.Hazard.IsEngaged")
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

**LED blink waveform note**: `IsSignaling` is a boolean *intent* flag. UN R48-compliant 1–2 Hz blink cadence is implemented in the LED driver IC or body ECU firmware. Feature business logic never sets timers for blink patterns.

**Feature inventory**

| Feature | Min ECU State | Inputs (VSS subscriptions) | Outputs (Arbiter requests) |
|---------|---------------|---------------------------|---------------------------|
| `HazardFsm` | Locally Awake | `Body.Switches.Hazard.IsEngaged` (overlay) | Both `DirectionIndicator.*.IsSignaling` @ prio 3 |
| `TurnFsm` | Locally Awake | `Body.Switches.TurnIndicator.Direction` (overlay) | `DirectionIndicator.{Left,Right}.IsSignaling` @ prio 2 |
| `LockFeedback` | Locally Awake | `Body.Doors.*.IsLocked` (state change) | Both indicators @ prio 3 (HIGH, overlay — self-releases after pattern) |
| `KeyfobPeps` | Locally Awake | `Body.PEPS.KeyPresent` (synthetic sensor) | DoorLock arbiter: UNLOCK / LOCK |
| `AutoLock` | Fully Awake | `Vehicle.Speed` | DoorLock arbiter: LOCK |
| `DoorTrimButton` | Locally Awake | `Body.Switches.DoorTrim.*.LockButton` (overlay) | DoorLock arbiter: LOCK / UNLOCK |
| `KeyfobRke` | Locally Awake | `Body.Switches.Keyfob.LockButton` (overlay) | DoorLock arbiter: LOCK / UNLOCK / DOUBLE_LOCK |
| `PhoneApp` | Locally Awake | `Body.Connectivity.RemoteLock` (overlay, cloud) | DoorLock arbiter: LOCK / UNLOCK |
| `PhoneBle` | Locally Awake | `Body.Connectivity.BleLock` (overlay, BLE digital key) | DoorLock arbiter: LOCK / UNLOCK |
| `NfcCard` | Locally Awake | `Body.Connectivity.NfcCardPresent` (overlay, physical NFC card) | DoorLock arbiter: LOCK / UNLOCK |
| `NfcPhone` | Locally Awake | `Body.Connectivity.NfcPhonePresent` (overlay, NFC key on phone) | DoorLock arbiter: LOCK / UNLOCK |
| `AutoRelock` | Locally Awake | `Body.Doors.*.IsLocked`, `Body.Doors.*.IsOpen`, `Vehicle.Safety.CrashDetected`, `Vehicle.LowVoltageSystemState` | DoorLock arbiter: LOCK (45s timeout, crash-disables until power cycle) |
| `CrashUnlock` | Locally Awake | `Vehicle.Safety.CrashDetected` (M7 state update) | DoorLock arbiter: UNLOCK (protected) |
| `LowBeamFsm` | Locally Awake | `Body.Lights.LightSwitch` | `Body.Lights.Beam.Low.IsOn` @ prio 2 |
| `HighBeamFsm` | Locally Awake | `Body.Switches.HighBeam.IsEngaged` (overlay) | `Body.Lights.Beam.High.IsOn` @ prio 2 |
| `DrlFsm` | Fully Awake | `Vehicle.LowVoltageSystemState`, `Chassis.ParkingBrake.IsEngaged` (overlay) | `Body.Lights.Running.IsOn` @ prio 2 |

**Input/output separation principle**: FSMs subscribe to *physical switch/stalk inputs* (sensor overlay signals), not to the actuator outputs they control. This prevents feedback loops and correctly models the hardware: a hazard switch is physically separate from the indicator lamps it controls. Overlay signals that are not in standard VSS v4.0 are defined in `overlay/Body/SwitchInputs.vspec`.

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
- Enables unit testing of every feature module and the Arbiter with no hardware dependency
- Records all published signals and exposed as `MockBus::history()` for assertions

**Dependency injection in main.rs**

```rust
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // Swap this one line to change transport:
    let bus = Arc::new(RpmsgBus::new("/dev/rpmsg0", "/dev/rpmsg1").await?);

    let arbiter = Arc::new(SignalArbiter::new(Arc::clone(&bus)));

    // Spawn all feature business logic
    tokio::spawn(HazardFsm::new(Arc::clone(&arbiter), Arc::clone(&bus)).run());
    tokio::spawn(TurnFsm::new(Arc::clone(&arbiter), Arc::clone(&bus)).run());
    tokio::spawn(LockFeedback::new(Arc::clone(&arbiter), Arc::clone(&bus)).run());
    tokio::spawn(KeyfobPepsFsm::new(Arc::clone(&arbiter), Arc::clone(&bus)).run());
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

**PEPS (Passive Entry/Passive Start) wake-up chain**

The PEPS unlock path executes entirely on M7 hardware while the A53 may still be asleep:

1. **Capacitive touch sensor** on door handle detects hand presence (always powered, µA draw — the only always-on component)
2. Capacitive sensor interrupt **wakes the M7** from sleep
3. M7 drives **LF antennas** in door handles (transmits challenge, a few seconds window)
4. Keyfob receives LF, **replies on UHF RF** (315/433 MHz)
5. M7 validates RF response (**crypto challenge-response**, NXP NCJ29D5 protocol)
6. M7 drives **lock actuators** directly via LLCE/LIN
7. M7 pushes `STATE_UPDATE` for `Body.Doors.*.IsLocked` and `Body.Doors.*.LatchStatus`
8. M7 wakes A53, A53 receives `Body.PEPS.KeyPresent = TRUE` (post-facto notification)

The A53 is notified *after* the unlock; it is **not in the critical path**. Steps 1–6 complete in < 80 ms. The A53 may still be booting when the doors are already unlocked. The KeyfobPeps feature on A53 submits an arbiter request only to keep application-layer state consistent.

---

## 10. IPC Message Schema

Defined in `src/ipc_message.rs` (pure Rust implementation). The AUTOSAR C side uses a matching `vss_ipc_message.h` header with identical layout, offsets, and CRC algorithm. Both sides must produce byte-identical wire format.

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

### Switch / Stalk Input Overlay

Standard COVESA VSS v4.0 defines actuator outputs (e.g. `Body.Lights.Hazard.IsSignaling`) but not the physical switch inputs that drive them. Feature business logic must subscribe to inputs, not outputs, to avoid feedback loops. The following overlay defines sensor signals for physical switches and stalks.

**File**: `overlay/Body/SwitchInputs.vspec`

```yaml
Body.Switches.Hazard.IsEngaged:
  datatype: boolean
  type: sensor
  description: >
    Physical hazard switch state. True when the driver has pressed the
    hazard button on the dashboard. HazardFsm subscribes to this signal.

Body.Switches.TurnIndicator.Direction:
  datatype: string
  type: sensor
  allowed: ['OFF', 'LEFT', 'RIGHT']
  description: >
    Turn signal stalk position. TurnFsm subscribes to this signal and
    maps LEFT/RIGHT to the corresponding DirectionIndicator actuator output.

Body.Switches.HighBeam.IsEngaged:
  datatype: boolean
  type: sensor
  description: >
    High beam stalk/switch state. HighBeamFsm subscribes to this signal.

Chassis.ParkingBrake.IsEngaged:
  datatype: boolean
  type: sensor
  description: >
    Parking brake engagement state. Used by DrlFsm as an input condition.
```

### Synthetic Signals

**`Body.PEPS.KeyPresent`** — Boolean sensor signal injected by the vss-bridge when the Safety Monitor reports a successful keyfob proximity authentication. The underlying hardware chain is: capacitive touch sensor on door handle → M7 wakes → M7 transmits LF challenge → keyfob replies on UHF RF → M7 validates crypto response. By the time this signal reaches the A53, the M7 has already driven the lock motors. KeyfobPeps subscribes to this signal to keep application-layer state consistent.

### Power Mode Signals

Features that depend on vehicle power state subscribe to **`Vehicle.LowVoltageSystemState`** (standard VSS v4.0) rather than `Vehicle.Powertrain.Engine.IsRunning`. This is powertrain-agnostic — works for ICE, HEV, and BEV platforms. Values: `undefined`, `lock`, `off`, `acc`, `on`, `start` (lowest → highest; `lock` is steering column lock, an anti-theft state below `off`). **`Vehicle.Powertrain.Type`** (string: `COMBUSTION`, `HYBRID`, `ELECTRIC`) is available for features that need powertrain-specific behaviour.

### ECU Sleep/Wake States

The body controller ECU has its own power management states, independent of the vehicle ignition position. Feature lifecycle is tied to these ECU states, not directly to the ignition key.

| ECU State | M7 | A53 | CAN/LIN | Wake Source | Typical Ignition |
|-----------|-----|------|---------|-------------|-----------------|
| **Deep Sleep** | Not polling; waiting for interrupt | Off | Off | Hardware interrupts only: capacitive touch on door handles, crash sensor, RKE RF receiver (always-on, µA draw) | Long-term parking (days/weeks) |
| **Sleep** | Periodic polling (low duty cycle) | Off | Off | Timer tick, battery voltage threshold, tilt/motion sensor (theft detection) | Parked overnight |
| **Low Power (M7 only)** | Fully active | Off | Active | M7 already awake; processes switch inputs, PEPS, crash signals directly over CAN/LIN. **M7 can wake A53** if it determines application-layer features are needed (see below). | Parked (OFF), driver operating lights/locks without ignition |
| **Locally Awake, Network Off** | Fully active | Active | Local body bus active; vehicle-wide backbone (powertrain, chassis CAN FD) in bus-sleep | A53 boot complete (ignition or M7-initiated wake); no cross-domain signals yet | ACC, M7-initiated A53 wake, or early ON before network management wakes all domains |
| **Fully Awake** | Fully active | Active | All buses active (body + powertrain + chassis) | Network management wake-up complete | ON / START |

**State transitions:**

```
Deep Sleep → (capacitive touch / RKE RF / crash interrupt) → Low Power (M7 only)
Sleep → (polling timer / tilt sensor) → Low Power (M7 only)

Low Power → (ignition ACC or ON) → Locally Awake → (network mgmt) → Fully Awake
Low Power → (M7-initiated A53 wake)  → Locally Awake
                                        ↑ M7 wakes A53 when it determines
                                          application-layer features are needed
                                          (PEPS auth, RKE event, switch input)

Locally Awake → (network management completes) → Fully Awake
Locally Awake → (no ignition, A53 work done, shutdown timer) → Low Power

Fully Awake → (ignition OFF, shutdown timer expires) → Low Power → Sleep → Deep Sleep
```

**M7-initiated A53 wake**: The M7 does not need to wait for an ignition change to wake the A53. After handling a critical-path operation in Low Power state (PEPS unlock, RKE lock/unlock, light switch change), the M7 can decide that application-layer features are needed and boot the A53. Examples:
- **PEPS unlock** → M7 wakes A53 so AutoRelock timer starts, LockFeedback blink fires, state syncs to kuksa.val
- **RKE lock/unlock** → M7 wakes A53 for LockFeedback blink and state sync
- **Light switch** → M7 wakes A53 for state sync and HMI update (if HMI is available)

In these cases the ECU enters **Locally Awake** (A53 up, local body bus active, but vehicle-wide backbone may still be in bus-sleep). If no ignition event follows and the A53 completes its work, a shutdown timer returns the ECU to Low Power. If the driver subsequently turns the ignition to ON, network management brings the ECU to Fully Awake.

**Feature lifecycle by ECU state:**

| ECU State | Features Active on A53 | M7 / Body ECU Handles Directly |
|-----------|----------------------|-------------------------------|
| **Deep Sleep** | None | Capacitive touch wake only; no active processing |
| **Sleep** | None | Battery monitoring, tilt/motion polling |
| **Low Power (M7 only)** | None | PEPS (full wake-up chain), CrashUnlock, light switch → lamp drive, hazard switch → indicator drive, RKE lock/unlock, all via CAN/LIN. Body ECU is the primary controller in this state. M7 wakes A53 when application-layer follow-up is needed. |
| **Locally Awake** | All features **except** AutoLock and DRL | Door lock features (KeyfobPeps, KeyfobRke, DoorTrimButton, AutoRelock, PhoneApp, PhoneBle, NfcCard, NfcPhone, CrashUnlock), lighting (LowBeam, HighBeam, Hazard, Turn, LockFeedback). No cross-domain signals available (Vehicle.Speed not yet on bus). Reached via ignition **or** M7-initiated wake. |
| **Fully Awake** | All features | All features + cross-domain signal coordination |

**Key design principles:**

1. **M7 handles all critical-path operations** while the A53 sleeps. In Low Power state, the body ECU drives lamps and locks from hardwired switch inputs over CAN/LIN — same architectural pattern as PEPS. The A53 feature layer is never in the critical path.

2. **M7-initiated A53 wake**: The M7 can wake the A53 from Low Power independently of ignition state changes. After completing a critical-path operation (PEPS unlock, RKE event, light switch), the M7 determines whether application-layer follow-up is needed (AutoRelock timer, LockFeedback blink, state sync) and boots the A53 if so. This means A53 features run even when the ignition is OFF, as long as there is a reason for them to be active.

3. **A53 state sync on boot**: When the A53 transitions from off to Locally Awake (whether by ignition or M7-initiated wake), the Safety Monitor pushes all current signal values as `STATE_UPDATE` via RPmsg to re-populate the signal bus and kuksa.val datastore. Features start with accurate state, even if the M7 has been handling requests while A53 was asleep.

4. **Features self-gate on power state**: Features that require cross-domain signals (AutoLock needs `Vehicle.Speed`, DRL needs engine-running confirmation) subscribe to `Vehicle.LowVoltageSystemState` and remain idle until the ECU reaches Fully Awake. They do not fail — they simply wait.

5. **Sleep inhibitors**: Features and subsystems that need the A53 to stay awake hold a **sleep inhibit claim**. The ECU cannot transition from Locally Awake to Low Power while any claim is active. Each claim has an associated maximum hold time to prevent a stuck feature from draining the 12V battery indefinitely.

   | Sleep Inhibitor | Held By | Typical Duration | Max Hold |
   |----------------|---------|-----------------|----------|
   | `AutoRelockTimer` | AutoRelock feature | Up to 45 s (relock timeout) | 60 s |
   | `LockFeedbackBlink` | LockFeedback feature | ~2 s (blink pattern) | 5 s |
   | `DoorLockInFlight` | DoorLock arbiter | ~300 ms (motor + ACK) | 2 s |
   | `OtaInProgress` | OTA subsystem | Minutes (download + apply) | 30 min |
   | `StateFlush` | vss-bridge | < 1 s (kuksa.val sync) | 5 s |
   | `ShutdownGrace` | Power manager | Fixed post-ignition-off grace period | 30 s |

   **Shutdown sequence**: When ignition turns OFF (or M7-initiated wake work triggers shutdown), the power manager starts `ShutdownGrace`. Features that still have work in progress hold their own inhibitors. The A53 powers down only when **all** inhibitors are released. If any inhibitor exceeds its max hold time, the power manager force-releases it, logs a DEM event, and proceeds with shutdown.

   **M7 enforcement**: The M7 controls A53 power and is the ultimate authority. If the A53 fails to shut down within a platform-configured hard timeout (e.g., 5 minutes), the M7 force-kills A53 power to protect the 12V battery. This is a safety backstop, not a normal code path.

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

Passive Entry target: **< 200 ms** from driver hand on door handle to lock actuator energised.

| Step | Owner | Time |
|------|-------|------|
| Capacitive touch detection → M7 wake | Door handle sensor + M7 | < 2 ms |
| M7 transmits LF challenge via door handle antennas | M7 AUTOSAR / NXP NCJ29D5 | ~15 ms |
| Keyfob receives LF, replies on UHF RF (315/433 MHz) | Keyfob | ~20 ms |
| M7 validates crypto challenge-response | M7 AUTOSAR / NXP NCJ29D5 | ~5 ms |
| Lock actuator command via LLCE/LIN | M7 → LLCE → LIN | ~10 ms |
| LIN actuator response / confirmation | Body ECU → LLCE → M7 | ~10 ms |
| **Total M7-side latency** | | **~62 ms** |
| NVM write (ASIL-B state commit) | M7 Safety Monitor | ~5 ms |
| STATE_UPDATE push to A53 (notification) | RPmsg | ~0.2 ms |
| Kuksa.val signal update (HMI indication) | A53 gRPC | ~2 ms |

The A53/AAOS/Kuksa stack is entirely **out of the critical path**. The 200 ms budget is met with ~138 ms of margin. The A53 is notified after the unlock event for state sync and HMI indication only. The capacitive touch sensor is the only always-on component (µA draw); the M7, LF antennas, and RF receiver are all dormant until the touch event.

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
- All feature business logic
- Signal Arbiter and priority table
- IPC message schema (reused verbatim)
- VSS signal IDs and overlay
- kuksa.val broker
- Android VHAL implementation
- Web HMI

**The `SignalBus` trait is the portability contract.** Every ecosystem has an equivalent construct (Java interface, C++ pure virtual, Go interface, AIDL IVehicle). The concept is transport-agnostic by design.

**SOME/IP as the maximally portable option**: if the next SoC does not have heterogeneous cores (i.e. no M7 equivalent), the Safety Monitor can run on a separate MCU connected via Ethernet. `SomeIpBus` implements `SignalBus` over SOME/IP; feature business logic is unchanged. Latency increases from ~100 µs (RPmsg) to ~500 µs (GbE), which is acceptable for body domain functions outside the PEPS critical path.

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

### Prompt 2 — Domain Arbiters

```
You are implementing domain-based Signal Arbiters for a Rust automotive body controller.

Context (from architecture doc):
- Two arbiter patterns:
  1. DomainArbiter (Lighting, Horn, Comfort) — instant per-signal priority resolution
  2. DoorLockArbiter — serialized one-deep command queue with ACK handshake

DomainArbiter details:
- Each `DomainArbiter` has a static allow-list of (FeatureId, VssPath, Priority) tuples
- Features publish `ActuatorRequest` structs with { signal, value, priority, feature_id }
- The arbiter validates against the allow-list, then holds a per-signal "current winner" map
- A new request replaces the winner if its priority >= current winner's priority
- Requests not in the allow-list are rejected silently (logged at WARN)
- All inter-task communication uses `tokio::sync::mpsc`
- Factory functions create each domain with its hardcoded allow-list

DoorLockArbiter details:
- Serialized queue: active operation (~300ms motor) + one pending slot
- Features submit `DoorLockRequest { command: LockCommand, feature_id }` — NOT ActuatorRequest
- LockCommand enum: Unlock, Lock, DoubleLock
- ACK from Classic AUTOSAR Locking SWC (via `LockAck` channel) promotes pending to active
- Pending slot: latest request replaces previous (except CrashUnlock)
- CrashUnlock: cannot be replaced in queue; triggers 10s lockout rejecting all new requests
- NVM persistence (last 5 requestors) is AUTOSAR SWC responsibility, not Rust

Priority enum: Low=1, Medium=2, High=3
FeatureId enum: KeyfobPeps=0x01, Hazard=0x02, TurnIndicator=0x03, LowBeam=0x04,
  HighBeam=0x05, Drl=0x06, AutoLock=0x07, LockFeedback=0x08, Welcome=0x09,
  DoorTrimButton=0x0A, KeyfobRke=0x0B, PhoneApp=0x0C, CrashUnlock=0x0D,
  PhoneBle=0x0E, NfcCard=0x0F, NfcPhone=0x10, AutoRelock=0x11

Lighting allow-list: Hazard→DirectionIndicator.*@HIGH, Turn→DirectionIndicator.*@MEDIUM,
  LockFeedback→DirectionIndicator.*@HIGH(overlay), Hazard→Hazard.IsSignaling@HIGH,
  LowBeam→Beam.Low@MEDIUM, HighBeam→Beam.High@MEDIUM, Drl→Running@MEDIUM
DoorLock allow-list: KeyfobPeps, KeyfobRke, AutoLock, AutoRelock, DoorTrimButton, PhoneApp, PhoneBle, NfcCard, NfcPhone, CrashUnlock
Horn/Comfort: empty allow-lists (pass-through with validation, ready for future features)

Implement:
1. `ActuatorRequest` and `AllowEntry` structs (for DomainArbiter)
2. `DomainArbiter` struct with `new()` returning (handle, future) and `request()` method
3. `arbiter_loop()` async function with allow-list validation and priority resolution
4. `DoorLockArbiter` struct with one-deep queue, `DoorLockRequest`, `LockAck` channel
5. `door_lock_loop()` async function with queue management and crash-unlock protection
4. Factory functions: lighting_arbiter, door_lock_arbiter, horn_arbiter, comfort_arbiter
5. Unit tests using MockBus covering:
   - High priority wins over low priority for the same signal
   - Low priority suppressed by existing high priority winner
   - Two different signals do not interfere
   - Priority tie: latest request wins
   - Request rejected if feature/signal/priority not in allow-list
   - Request rejected if feature claims wrong priority
   - Cross-domain: door lock arbiter PEPS wins over AutoLock
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

### Prompt 4 — Feature Business Logic (full set)

```
You are implementing all body feature business logic for a Rust automotive body controller.

Context:
- Each feature is an async struct holding Arc<SignalArbiter<B>> and Arc<B: SignalBus>
- Features subscribe to input signals, compute state transitions or run algorithms, and publish ActuatorRequests
- No feature imports another feature
- Implementations may be simple state machines or complex algorithms — use whatever fits the feature
- LED blink waveform is NOT the FSM's responsibility — it sets boolean intent only
- IMPORTANT: FSMs subscribe to physical switch/stalk INPUTS (overlay sensors),
  not to the actuator outputs they control. This prevents feedback loops.
- Lighting domain priority assignments (hardcoded, matches Safety Monitor's table):
    HazardFsm   input: Body.Switches.Hazard.IsEngaged        → DirectionIndicator.*.IsSignaling    @ HIGH (3)
    TurnFsm     input: Body.Switches.TurnIndicator.Direction  → DirectionIndicator.{Left,Right}     @ MEDIUM (2)
    LockFeedback input: Body.Doors.*.IsLocked (state change)  → DirectionIndicator.*.IsSignaling    @ HIGH (3, overlay — self-releases)
    LowBeamFsm input: Body.Lights.LightSwitch                 → Lights.Beam.Low.IsOn                @ MEDIUM (2)
    HighBeamFsm input: Body.Switches.HighBeam.IsEngaged        → Lights.Beam.High.IsOn               @ MEDIUM (2)
    DrlFsm      input: Vehicle.LowVoltageSystemState,
                       Chassis.ParkingBrake.IsEngaged          → Lights.Running.IsOn                 @ MEDIUM (2)

- DoorLock domain (serialized queue, NOT priority-based):
    KeyfobPepsFsm  input: Body.PEPS.KeyPresent (synthetic)              → DoorLockArbiter: UNLOCK / LOCK
    KeyfobRke      input: Body.Switches.Keyfob.LockButton (overlay)    → DoorLockArbiter: LOCK / UNLOCK / DOUBLE_LOCK
    AutoLock       input: Vehicle.Speed                                 → DoorLockArbiter: LOCK
    DoorTrimButton input: Body.Switches.DoorTrim.*.LockButton (overlay) → DoorLockArbiter: LOCK / UNLOCK
    PhoneApp       input: Body.Connectivity.RemoteLock (overlay, cloud) → DoorLockArbiter: LOCK / UNLOCK
    PhoneBle       input: Body.Connectivity.BleLock (overlay, BLE key)  → DoorLockArbiter: LOCK / UNLOCK
    NfcCard        input: Body.Connectivity.NfcCardPresent (overlay)    → DoorLockArbiter: LOCK / UNLOCK
    NfcPhone       input: Body.Connectivity.NfcPhonePresent (overlay)   → DoorLockArbiter: LOCK / UNLOCK
    AutoRelock     input: Body.Doors.*.IsLocked + *.IsOpen,              → DoorLockArbiter: LOCK (45s timer)
                          Vehicle.Safety.CrashDetected,                   crash → DISABLED until power cycle
                          Vehicle.LowVoltageSystemState                   OFF → ON re-enables after crash
    CrashUnlock    input: Vehicle.Safety.CrashDetected (M7 state update) → DoorLockArbiter: UNLOCK (protected)

  DoorLock arbiter rules:
  - One-deep queue: active operation (~300 ms motor) + one pending
  - Pending replaced by newer request (latest wins), EXCEPT CrashUnlock
  - CrashUnlock in queue cannot be replaced; triggers 10s lockout after dispatch
  - ACK from Classic AUTOSAR Locking SWC promotes pending to active
  - NVM diagnostic persistence (last 5) is AUTOSAR SWC's responsibility

Implement all features, each in its own file under src/features/.
For LockFeedback: on any IsLocked state change event, play a timed pattern on both indicators
at priority HIGH (overlay): lock = one 600ms flash, unlock = two 150ms flashes (100ms gap).
Then release (publish HIGH false). Use tokio::time::sleep for durations.
For door lock features: submit DoorLockRequest to the DoorLockArbiter (not ActuatorRequest).
Include unit tests using MockBus for each feature covering the primary state transitions.
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
- kuksa.val runs in a sibling container at grpc://localhost:55555 (configurable via KUKSA_ENDPOINT env var)
- Use raw `tonic` with bundled kuksa.val proto definitions in `proto/kuksa/val/v1/`
  (val.proto defines the VAL gRPC service; types.proto defines DataEntry, Datapoint, etc.)
- build.rs uses `tonic_build::configure().build_server(false).compile()` to generate the client stubs
- The proto module is re-exported as `kuksa_sync::proto` via `tonic::include_proto!("kuksa.val.v1")`
- The sync loop has two responsibilities:
    1. INBOUND: subscribe to all signals from ALL_SIGNALS in kuksa.val via the Subscribe RPC;
       convert kuksa.val Datapoint values to internal SignalValue; forward to SignalBus::publish()
    2. OUTBOUND: relay Safety Monitor state updates to kuksa.val via the Set RPC
       using `push_to_kuksa()` which converts SignalValue → Datapoint
- Handle kuksa.val disconnection gracefully: retry with exponential backoff (1s, 2s, 4s, max 30s)
- On reconnect: request a full state snapshot from Safety Monitor and replay to kuksa.val
- The struct is `KuksaSync<B: SignalBus>` with a `run()` async method
- On connect, log server info via GetServerInfo RPC

Implement:
1. `KuksaSync<B>` struct and `run()` method with reconnect loop
2. Inbound subscription loop using `ValClient::subscribe()` streaming RPC
3. `datapoint_to_signal_value()` and `signal_value_to_datapoint()` conversion functions
4. `push_to_kuksa()` for outbound Set RPCs
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
- build.rs runs tonic-build to generate gRPC client stubs from bundled kuksa.val proto files
- IPC message types are implemented in pure Rust (no C header, no bindgen)
- Container base: registry.redhat.io/ubi9/ubi-minimal
- Podman, not Docker

Produce:
1. `Cargo.toml` with all dependencies from the architecture doc
2. `build.rs` using tonic-build to compile kuksa.val proto files (build_server=false, client only)
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

*Document version: 1.1 — Architecture as of April 2026*
*VSS base: COVESA VSS v4.0 + DoorExtended overlay + SwitchInputs overlay*
*IPC schema: src/ipc_message.rs v1 (magic 0xBCC01A00) — pure Rust, matching AUTOSAR C header*
