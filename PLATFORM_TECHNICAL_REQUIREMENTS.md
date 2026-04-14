# Body SDV Platform — Technical Requirements

**Document purpose**: Development guide for the hardware and embedded software teams building the body controller platform. Covers what needs to be designed, what needs to be brought up, and what's already implemented in the application layer.

**Last updated**: 2026-04-14

---

## 1. Business Context

This platform is a **product sold by a service company to OEM management**. It is not a one-off project for a single vehicle program. Every design decision must account for:

- **Cross-program reuse**: The same platform ships on sedans, SUVs, coupes, and trucks. Hardware and software must be configurable, not hard-coded for one vehicle.
- **OEM replaceability**: OEMs buy the platform (middleware + infrastructure) but may replace individual feature applications with their own. Features must be self-contained and decoupled.
- **Portability across SoCs**: The first target is NXP S32G2, but the architecture must support Qualcomm SA8775P, Renesas R-Car S4, or TI TDA4 with changes in exactly two places: BSP and one transport adapter file.
- **Safety certification boundary**: OEMs expect ASIL-B certification on the safety-critical paths (lighting, locks, PEPS) without requiring the entire application layer to be certified. The M7 Safety Monitor owns all ASIL-B state. The A53 application layer is QM.

---

## 2. Target Hardware

### 2.1 SoC: NXP S32G2

| Domain | Cores | Clock | Role |
|--------|-------|-------|------|
| Application (A53) | 4x ARM Cortex-A53 | 1 GHz | AAOS, Podman containers, Rust application layer, HMI |
| Real-time (M7) | 3x ARM Cortex-M7 | 400 MHz | AUTOSAR Classic CP, Safety Monitor (ASIL-B), PEPS, CAN/LIN frame processing |

**Key peripherals on the SoC:**

| Peripheral | Purpose |
|-----------|---------|
| **LLCE** (Low Latency Communication Engine) | Dedicated CAN FD + LIN engine. Handles frame encoding/decoding independently of A53 and M7. Exposes frames to M7 via shared memory, to A53 via SocketCAN. |
| **PFE** (Packet Forwarding Engine) | GbE MAC + TSN forwarding. Used for SOME/IP Ethernet backbone if domain controller topology is adopted. |
| **RPmsg** (virtio) | Shared-memory IPC between A53 and M7. Two channels: `vss-cmd` (A53→M7) and `vss-state` (M7→A53). Character devices `/dev/rpmsg0` and `/dev/rpmsg1`. |
| **SEMA4** | Hardware semaphores for atomic NVM access on M7. Not exposed above BSP. |

### 2.1.1 SoC Optionality — S32G vs. S32N vs. Third-Party

The reference design targets NXP S32G2, but the platform must support alternate SoCs. The table below captures current options and their trade-offs.

| Dimension | NXP S32G2/G3 (reference) | NXP S32N | Qualcomm SA8775P | Renesas R-Car S4 |
|---|---|---|---|---|
| **Application cores** | 4× Cortex-A53 @ 1 GHz | Cortex-A72 class, more cores | 4× Cortex-A78AE + GPU | 8× Cortex-A76 |
| **Safety cores** | 3× Cortex-M7 | Multiple safety islands (R52) | Cortex-R52 safety island | Cortex-R52 lock-step |
| **Best fit** | Body domain controller (standalone ECU) | Zonal compute (body + gateway + partial chassis on one chip) | Cockpit-body fusion (IVI + body on one SoC) | Zone/domain controller |
| **I/O integration** | LLCE (CAN FD + LIN), PFE (GbE/TSN) | Higher I/O mux, more CAN/LIN/ETH | Limited automotive I/O (needs companion MCU for CAN/LIN) | CAN FD, Ethernet TSN |
| **AUTOSAR Classic on safety core** | Yes (Cortex-M7) | Yes (R52 island) | Limited (R52 island, smaller ecosystem) | Yes (R52 lock-step) |
| **Production maturity** | Production silicon, multiple OEM programs | Pre-production / early production | Production (cockpit), limited body-domain deployment | Production |
| **BOM cost** | Lower (body-optimized) | Higher (zonal-class SoC) | Higher (cockpit-class SoC) | Comparable to S32G |
| **A53↔M7 IPC** | RPmsg (shared memory, ~100 µs) | RPmsg variant | Qualcomm GLINK | Renesas ICCOM |

**Guidance for SoC selection:**

- **Standalone body controller** (most OEM programs today): NXP S32G2/G3. Purpose-built for this role, lowest BOM, mature BSP and AUTOSAR support. No reason to pay for silicon you don't use.
- **Zonal architecture** (OEM consolidating body + gateway): NXP S32N. Body domain runs as one workload among several. The platform's container topology maps cleanly to this — body containers run alongside gateway containers on the same A-core cluster.
- **Cockpit-body fusion** (OEM merging IVI + body onto one SoC): Qualcomm SA8775P. Body containers share the A-core cluster with Android Automotive IVI. Requires companion MCU for CAN/LIN I/O and safety functions — the platform's `GlinkBus` adapter handles the A-core ↔ MCU boundary.
- **Future option**: Renesas R-Car S4 for OEMs with existing Renesas infrastructure.

**The `SignalBus` trait guarantees portability.** Changing SoC means replacing the transport adapter (`RpmsgBus` → `GlinkBus` or `IccomBus`) and the BSP. Zero feature code changes.

### 2.1.2 Application-Core OS Options

The Rust application layer is OS-agnostic — it compiles as a statically-linked musl binary with no distribution-specific dependencies. The OS choice affects the BSP, container runtime, and support model.

| Dimension | Yocto/Poky (reference) | Android Automotive OS | Red Hat In-Vehicle OS |
|---|---|---|---|
| **License model** | Open source (MIT/GPL). No per-unit cost. | AOSP is open source. Google Automotive Services (GAS) requires license. | Subscription-based per unit. |
| **Image size** | Minimal: 50-100 MB. Fast boot (< 2s to userspace). | Full AAOS: 2-4 GB. Slower boot (5-10s). | 300-500 MB base. Boot ~3-5s. |
| **Container runtime** | Podman, Docker, containerd — all supported | Podman runs under AAOS as userspace process | Podman (native, Red Hat is the upstream maintainer) |
| **OEM familiarity** | Embedded teams know Yocto/BitBake. Standard in body/chassis domain. | IVI teams know AAOS. Body teams may not. | Enterprise IT teams know RHEL. Embedded teams may not. |
| **Long-term support** | Community-maintained. Vendor BSP lifecycle varies (typically 5-7 years). LTS requires OEM or platform provider effort. | Google controls release cadence. OEM has limited influence on lifecycle. | Red Hat commits to 10+ year lifecycle with security patches. Best alignment with automotive production + support timelines. |
| **Security patching** | Platform provider must track CVEs and rebuild images. No upstream SLA. | Google patches Android monthly. Body-specific BSP patches are OEM/provider responsibility. | Red Hat provides CVE response SLA. Dedicated automotive security team. |
| **Certification/compliance** | No pre-certification. UNECE R155/R156 compliance is platform provider's responsibility. | Android CTS provides a baseline. Automotive-specific compliance is additional. | Red Hat pursuing ISO/SAE 21434 and UNECE R155/R156 alignment. |
| **Best fit** | Standalone body controller with cost-sensitive OEMs. Platform provider controls the full stack. | OEMs requiring AAOS on the same SoC (cockpit-body fusion, VHAL integration). | OEMs requiring vendor-backed 10+ year lifecycle with SLA. Programs where procurement requires a named OS vendor. |

**Recommendation**: Yocto as the reference BSP for development and cost-sensitive programs. Red Hat In-Vehicle OS as the supported option for OEMs requiring vendor-backed long-term support (10+ years) and security patching SLA — this aligns with the automotive production + field support timeline. AAOS when the OEM requires Android integration on the same SoC.

**The platform provider (us) absorbs the OS complexity.** The OEM licenses the platform, not the OS. Whether we build on Yocto or Red Hat underneath, the OEM sees the same Rust service layer, the same VSS signal model, the same SOVD API. OS choice is a platform configuration parameter, not an OEM engineering decision.

### 2.1.3 Safety-Core RTOS

The safety core (M7 / R52) runs **AUTOSAR Classic CP**. This is a non-negotiable platform requirement for current and near-term OEM programs.

**Rationale:**
- Classic AUTOSAR provides the full BSW stack (CAN/LIN, UDS diagnostics, NvM, power management, COM) that body controllers depend on. No alternative RTOS provides this out of the box.
- OEM compliance and validation teams expect AUTOSAR Classic on the safety core. Proposing a non-AUTOSAR safety stack adds certification risk and validation cost that OEMs will not accept.
- Tool ecosystem (Vector CANoe, dSPACE, ETAS ISOLAR) integrates with AUTOSAR Classic. These tools are already deployed at every OEM.

The platform's value proposition is that it **reduces** the OEM's Classic AUTOSAR scope: feature business logic moves to Rust on the A-core (QM), leaving only the thin BSW + safety monitor + I/O stack on the M7. This cuts AUTOSAR Classic development effort and seat licenses while keeping the proven stack where it earns its keep — the safety-critical I/O boundary.

### 2.2 What the Hardware Team Needs to Design

#### 2.2.1 ECU Board / PCB

The S32G2 SoC is the central processor. The hardware team must design the body controller ECU board around it:

| Subsystem | Requirements |
|-----------|-------------|
| **Power supply** | 12V battery input with reverse polarity and load dump protection. Regulated supplies for A53 (1.1V core, 1.8V I/O), M7 (1.1V core), and LLCE/PFE. M7 must be able to power-gate the A53 independently (sleep/wake control). |
| **CAN FD transceivers** | Minimum 2x CAN FD channels at 500 kbps (body bus, diagnostic bus). Route to LLCE pins. Termination resistors (switchable for end-of-line). |
| **LIN transceivers** | Minimum 4x LIN channels at 19.2 kbps for door modules, mirror motors, seat motors, ambient lighting. Route to LLCE LIN engine. |
| **Ethernet** | 1x 100BASE-T1 or 1000BASE-T1 for vehicle backbone (SOME/IP, DoIP diagnostics). Route to PFE GbE MAC. |
| **eMMC / flash** | AAOS requires ~8 GB eMMC for the A53 (OS, container images). M7 AUTOSAR needs dedicated NOR flash for ASIL-B NVM (CRC-protected, separate from QM storage). |
| **RAM** | 2-4 GB LPDDR4 for A53 (AAOS + containers). M7 uses on-chip TCM (tightly coupled memory) — no external RAM needed. |
| **PEPS hardware** | See section 2.2.2. |
| **Debug** | JTAG for M7 (AUTOSAR debugging). ADB over USB for A53/AAOS. Serial console (UART) for both domains. |
| **Connectors** | ECU main connector with all CAN/LIN/power/Ethernet pins. Match OEM connector spec (varies by program). |

#### 2.2.2 PEPS (Passive Entry / Passive Start) Hardware

The PEPS unlock path executes entirely on M7 hardware while A53 may be asleep. The hardware team must provide:

| Component | Requirements |
|-----------|-------------|
| **Capacitive touch sensors** | One per exterior door handle. Always-on (µA draw — the only always-on component). Must generate an interrupt to wake M7 from sleep. |
| **LF antennas** | Embedded in each door handle. Driven by M7 to transmit the challenge signal. Frequency: 125 kHz. |
| **RF receiver** | UHF (315 MHz NA / 433 MHz EU). Receives keyfob reply. Route to NXP NCJ29D5 or equivalent crypto IC. |
| **NXP NCJ29D5** (or equivalent) | Crypto challenge-response IC. Connected to M7 via SPI. Handles the keyfob authentication protocol. |

**PEPS timing budget**: The entire chain (touch detect → M7 wake → LF challenge → RF reply → crypto validate → lock actuator) must complete in **< 200 ms**. Current estimate: ~62 ms with 138 ms margin.

| Step | Owner | Time |
|------|-------|------|
| Capacitive touch → M7 wake | HW sensor + M7 | < 2 ms |
| LF challenge transmit | M7 / NCJ29D5 | ~15 ms |
| Keyfob RF reply | Keyfob | ~20 ms |
| Crypto validation | M7 / NCJ29D5 | ~5 ms |
| Lock actuator via LLCE/LIN | M7 → LLCE → LIN | ~10 ms |
| LIN confirmation | Body ECU → LLCE → M7 | ~10 ms |
| **Total** | | **~62 ms** |

#### 2.2.3 Body ECU I/O Nodes (Peripheral ECUs on CAN/LIN)

These are the "dumb" peripheral nodes that drive physical actuators. They receive CAN FD or LIN frames from the S32G2 LLCE and execute commands. They are **not** application-intelligent — pure I/O mapping.

| Node | Bus | Actuators / Sensors |
|------|-----|-------------------|
| **Front lighting module** | CAN FD | Low beam, high beam, DRL, front fog, front direction indicators |
| **Rear lighting module** | CAN FD | Brake lights, tail lights, reverse lights, rear fog, rear direction indicators, license plate lights |
| **Driver door module** | LIN | Lock motor, window motor, mirror motor (tilt/pan/fold/heat), interior handle sensor, exterior handle capacitive touch + LF antenna |
| **Passenger door module** | LIN | Lock motor, window motor, mirror motor, handle sensor |
| **Rear door modules** (x2) | LIN | Lock motor, window motor, child lock solenoid |
| **Trunk/liftgate module** | LIN | Lock motor, latch sensor, cinch motor (if power liftgate) |
| **Interior module** | LIN | Dome light, glovebox light, ambient LED strip (RGB), courtesy lights |
| **Seat modules** (x2-4) | LIN | Heating element, ventilation fan, position motors (if power seat) |
| **HVAC module** | CAN FD | Blower motor, blend doors, A/C compressor relay, recirculation flap |
| **Horn module** | CAN FD | Horn relay |
| **Wiper module** | CAN FD | Front wiper motor (intermittent/low/high), rear wiper motor, washer pump |
| **Sunroof module** | LIN | Sunroof motor, shade motor, position sensor |

Each node must:
- Accept CAN FD / LIN command frames and drive the actuator
- Return sensor state (latch position, lock state, current draw, temperature) as periodic or event-driven frames
- Implement open-circuit and short-circuit detection on actuator outputs (for DTC reporting)

#### 2.2.4 ECU Power Management (Sleep/Wake States)

The ECU has five power states, **independent of ignition position**. The hardware must support the transitions.

| ECU State | M7 | A53 | CAN/LIN | Wake Source |
|-----------|-----|------|---------|-------------|
| **Deep Sleep** | Interrupt-only | Off | Off | Capacitive touch, crash sensor, RKE RF (all always-on, µA draw) |
| **Sleep** | Periodic polling | Off | Off | Timer tick, battery voltage, tilt/motion sensor |
| **Low Power (M7 only)** | Fully active | Off | Active | M7 already awake; can wake A53 if application features needed |
| **Locally Awake** | Fully active | Active | Body bus active; backbone sleeping | Ignition ACC/ON or M7-initiated A53 wake |
| **Fully Awake** | Fully active | Active | All buses active | Network management complete |

**Hardware requirements for power management:**
- M7 must have independent control over A53 power rail (GPIO-controlled regulator or PMIC channel)
- Crash sensor interrupt must be routed to M7 as a non-maskable interrupt (always-on even in Deep Sleep)
- RKE RF receiver must be always-on with µA standby current
- Battery voltage ADC input to M7 for monitoring (low-voltage shutdown protection)
- Tilt/motion sensor (accelerometer) on I2C/SPI to M7 for theft detection wake

---

## 3. Embedded Software Requirements

### 3.1 M7 / AUTOSAR Classic (ASIL-B)

The M7 runs AUTOSAR Classic CP. The embedded SW team must develop or integrate:

#### 3.1.1 Safety Monitor SWC

The central safety authority. All ASIL-B state lives here.

**Responsibilities:**
1. Receive `ACTUATOR_CMD` messages from A53 via RPmsg
2. Validate: magic (`0xBCC01A00`), version, CRC-16/CCITT-FALSE, priority claim vs. internal table, vehicle state constraints
3. If valid: write to ASIL-B NVM, forward to AUTOSAR COM stack → LLCE → CAN FD/LIN → body ECU
4. If invalid: return `CMD_ACK` with error code (`ERR_SAFETY`, `ERR_PRIORITY`, `ERR_STATE`, `ERR_CHECKSUM`, `ERR_VERSION`, `ERR_UNKNOWN_SIG`); do **not** forward
5. On every committed state change: push `STATE_UPDATE` to A53
6. Monitor body ECU sensor frames; push `FAULT_REPORT` on open-circuit, short-circuit, or timeout
7. **Hold-last on fault**: if a CAN/LIN node stops responding, hold the last known safe state in NVM — do not default to off

**ASIL-B NVM requirements:**
- Dedicated NVM partition, separate from QM AAOS storage
- Atomic writes via SEMA4 hardware semaphore
- CRC32 checksum over every state record
- Boot: read NVM → validate CRC → if CRC fails, apply safe-state defaults and log DEM event
- After NVM restore: push all signals as `STATE_UPDATE` to A53

**Priority table** (must match the Rust arbiter — compiled from a shared JSON source):

| Feature ID | Hex | Signals | Priority |
|-----------|-----|---------|----------|
| Hazard | 0x02 | DirectionIndicator.Left/Right.IsSignaling | 3 (HIGH) |
| TurnIndicator | 0x03 | DirectionIndicator.Left/Right.IsSignaling | 2 (MEDIUM) |
| LockFeedback | 0x08 | DirectionIndicator.Left/Right.IsSignaling | 3 (HIGH, overlay) |
| LowBeam | 0x04 | Beam.Low.IsOn | 2 (MEDIUM) |
| HighBeam | 0x05 | Beam.High.IsOn | 2 (MEDIUM) |
| DRL | 0x06 | Running.IsOn | 2 (MEDIUM) |
| CrashUnlock | 0x0D | Door locks | Protected (10s lockout) |

**Code constraints:**
- MISRA-C:2012 compliant
- No dynamic allocation
- No recursion
- C99 standard

#### 3.1.2 PEPS SWC

Runs the PEPS state machine entirely on M7 while A53 sleeps:
1. Capacitive touch interrupt → wake from sleep
2. Drive LF antennas (125 kHz challenge via NCJ29D5)
3. Receive and validate UHF RF keyfob response
4. Drive lock actuators via LLCE/LIN
5. Push `STATE_UPDATE` to A53 (post-facto notification)
6. Optionally wake A53 for application-layer follow-up (AutoRelock timer, LockFeedback blink)

#### 3.1.3 UDS Diagnostic Server

Standard ISO 14229 UDS server on M7:
- **Service 0x22** (ReadDataByIdentifier): read current config values
- **Service 0x2E** (WriteDataByIdentifier): dealer configurable parameters, persisted in NVM
- **Service 0x19** (ReadDTCInformation): fault codes from body ECU monitoring
- **Service 0x14** (ClearDTC): clear stored DTCs
- **Service 0x27** (SecurityAccess): seed-key authentication before 0x2E writes

Dealer-configurable DIDs (Tier 4 config):

| DID | Parameter | Type | Default |
|-----|-----------|------|---------|
| 0xF190 | Auto-relock enabled | bool | true |
| 0xF191 | Horn chirp on lock | bool | false |
| 0xF192 | Courtesy light timeout (seconds) | u8 | 30 |
| 0xF193 | Remote start max duration (minutes) | u8 | 15 |
| 0xF194 | Approach unlock mode | string | "DRIVER_ONLY" |

After a 0x2E write, M7 pushes the updated value to A53 via a `CONFIG_UPDATE` IPC message.

#### 3.1.4 CAN/LIN Stack (AUTOSAR COM + LLCE)

- Configure AUTOSAR COM to route CAN FD and LIN frames through LLCE
- Define the CAN database (.dbc) mapping VSS signal paths to frame IDs, bit positions, and scaling
- The DBC is generated from the VSS overlay at build time using `vss-tools`
- M7 handles all CAN/LIN frame processing; A53 never touches raw frames directly (except SocketCAN for diagnostics if needed)

#### 3.1.5 Power Manager (M7-side)

- Controls A53 power rail
- Implements the 5-state ECU power management (Deep Sleep → Sleep → Low Power → Locally Awake → Fully Awake)
- Decides when to wake A53 (after PEPS, RKE, switch events)
- Respects sleep inhibitors from A53 (communicated via RPmsg)
- **Hard backstop**: if A53 fails to shut down within 5 minutes, M7 force-kills A53 power to protect the 12V battery

### 3.2 A53 / Linux / AAOS

#### 3.2.1 BSP (Board Support Package)

| Component | Requirement |
|-----------|-------------|
| **Kernel** | `PREEMPT_RT` patched Linux kernel for the A53 cluster |
| **SocketCAN** | Driver exposing LLCE CAN FD frames as standard Linux `can0` interface |
| **LIN daemon** | Bridges LIN frames to userspace via `/dev/linX` |
| **RPmsg driver** | Linux virtio RPmsg driver. Character devices `/dev/rpmsg0` (cmd out) and `/dev/rpmsg1` (state in) |
| **Podman** | Rootless, daemonless container runtime running as userspace process under AAOS |

#### 3.2.2 Android VHAL Implementation

- Implements `android.hardware.automotive.vehicle.IVehicle` (AIDL)
- Maps VSS signals to VHAL property IDs (OEM custom range `0x0F400000–0x0F40FFFF` for signals without standard mapping)
- Routes safety-relevant `setValues()` calls through the Rust bridge → arbiter → RPmsg → Safety Monitor
- CarService and VHAL are permanently QM — they cannot issue ASIL-B commands directly

#### 3.2.3 SOVD Gateway

**Standard**: ASAM SOVD V1.0.0 (2023) — Service-Oriented Vehicle Diagnostics. Developed by ASAM with AUTOSAR Adaptive input.

**What it is**: An HTTP/REST gateway running on the A53 that exposes vehicle diagnostics as a standard OpenAPI interface. Diagnostic tools, cloud backends, and mobile apps access vehicle data through REST calls with JSON payloads instead of proprietary UDS tooling.

**Why it matters for this platform**:
- OEMs evaluating HPC-based body controllers in 2026+ will expect SOVD support. BMW and VW Group are early adopters; it is becoming a procurement checkbox.
- Our architecture is exactly what SOVD was designed for: Linux HPC (A53) + Classic AUTOSAR MCU (M7). The SOVD Gateway bridges the two worlds.
- Cloud diagnostics: same REST API works in-workshop (Ethernet) and remote (cellular). OEM cloud can read DTCs, health data, and trigger guided troubleshooting without a proprietary protocol.

**Architecture**:

```
                    ┌─────────────────────────────────────────┐
                    │         External Clients                 │
                    │  Workshop tool · OEM cloud · Tablet app  │
                    └──────────────┬──────────────────────────┘
                                   │ HTTP/REST (JSON)
                    ┌──────────────▼──────────────────────────┐
                    │      SOVD Gateway (A53 container)        │
                    │  OpenAPI server · ASAM SOVD V1.0.0       │
                    │                                          │
                    │  ┌────────────────┐  ┌────────────────┐  │
                    │  │ Native SOVD    │  │ Classic Diag   │  │
                    │  │ endpoints      │  │ Proxy          │  │
                    │  │ (HPC services) │  │ (SOVD→UDS)     │  │
                    │  └───────┬────────┘  └───────┬────────┘  │
                    └──────────┼────────────────────┼──────────┘
                               │                    │
              ┌────────────────▼──┐    ┌────────────▼──────────┐
              │  vss-bridge       │    │  M7 UDS Server         │
              │  kuksa.val        │    │  (ISO 14229)           │
              │  Container health │    │  0x22, 0x2E, 0x19, etc│
              │  OTA status       │    │  NVM-backed DIDs       │
              └───────────────────┘    └────────────────────────┘
```

The SOVD Gateway has two internal paths:

1. **Native SOVD endpoints** — for HPC-resident services that don't exist on the M7:
   - Container health and restart status
   - OTA update status and history
   - kuksa.val signal snapshot (current values of all VSS signals)
   - Feature enable/disable status from the 4-tier config system
   - Application-layer logs and diagnostics

2. **Classic Diagnostic Proxy** (SOVD-to-UDS translation) — for M7-hosted diagnostics:
   - DTC read/clear → translated to UDS 0x19 / 0x14 over DoIP or RPmsg
   - Dealer config read/write → translated to UDS 0x22 / 0x2E with security access (0x27)
   - Calibration reads → translated to UDS 0x22 for Tier 2/3/4 parameters
   - ECU identification → translated to UDS 0x22 (F1xx DIDs)

**SOVD REST API examples** (illustrative, based on ASAM SOVD V1.0.0 patterns):

```
GET  /sovd/v1/components
     → lists all diagnosable components (body-controller-m7, vss-bridge, kuksa-val, etc.)

GET  /sovd/v1/components/body-controller-m7/faults
     → reads all active DTCs from M7 (proxied via UDS 0x19)

DELETE /sovd/v1/components/body-controller-m7/faults
     → clears DTCs (proxied via UDS 0x14)

GET  /sovd/v1/components/body-controller-m7/data/auto-relock-enabled
     → reads dealer config DID 0xF190 (proxied via UDS 0x22)

PUT  /sovd/v1/components/body-controller-m7/data/auto-relock-enabled
     { "value": false }
     → writes dealer config (proxied via UDS 0x2E with 0x27 security access)

GET  /sovd/v1/components/vss-bridge/data/signal-snapshot
     → returns current values of all VSS signals from kuksa.val (native, no UDS)

GET  /sovd/v1/components/vss-bridge/data/container-health
     → returns uptime, memory usage, restart count (native)
```

**Relationship to the existing UDS server on M7**:

SOVD does **not replace** the M7 UDS server. The M7 keeps its full UDS stack (ISO 14229) — it is the NVM owner, the security access authority, and the DTC manager. SOVD is a higher-level API layer on A53 that translates REST calls into UDS requests and forwards them to M7. The M7 does not know or care whether a 0x22 read came from a CAN-connected scan tool or from the SOVD Gateway over RPmsg.

**Implementation approach**:

| Component | Language | Container | Notes |
|-----------|----------|-----------|-------|
| SOVD HTTP server | Rust (axum or actix-web) | `sovd-gateway` (new Podman container) | OpenAPI spec auto-generated from ASAM SOVD schema |
| Classic Diag Proxy | Rust | Same container | Translates REST→UDS, manages security access sessions |
| UDS transport | Rust | Same container | Sends UDS frames to M7 via RPmsg (reuses `ipc_message.rs` framing) or DoIP over Ethernet |
| Native endpoints | Rust | Same container | Reads from kuksa.val (gRPC), config system, container runtime |

**Deliverables**:
1. SOVD Gateway container with OpenAPI server
2. Classic Diagnostic Proxy (SOVD→UDS translation for M7)
3. Native endpoints for HPC-resident services
4. Authentication/authorization (TLS + token-based for cloud access, local-only for workshop)
5. Integration tests: REST call → UDS proxy → M7 mock → response validation

**M7 impact**: None. The M7 UDS server speaks standard UDS and doesn't care who the client is. The SOVD Gateway is an additive component on A53 that wraps the existing UDS interface. The `ipc_message.rs` wire format already carries UDS-compatible DIDs (Tier 4 config). The SOVD proxy reuses this.

---

## 4. IPC Wire Format (A53 ↔ M7)

Both sides must produce **byte-identical** wire format. Defined in Rust (`ipc_message.rs`) and C (`vss_ipc_message.h`).

### 4.1 Common Header (16 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 4 B | `magic` | `0xBCC01A00` |
| 4 | 1 B | `version` | Schema version (currently 1) |
| 5 | 1 B | `msg_type` | ACTUATOR_CMD / STATE_UPDATE / CMD_ACK / FAULT_REPORT |
| 6 | 2 B | `seq` | Wrapping u16 per direction; receiver detects gaps |
| 8 | 4 B | `timestamp_ms` | SoC uptime in ms |
| 12 | 4 B | `signal_id` | Stable 32-bit ID from `vspec2id` |

### 4.2 Message Types

| Message | Direction | Size | Purpose |
|---------|-----------|------|---------|
| `VssActuatorCmd` | A53 → M7 | 28 B | Feature requests actuator change |
| `VssStateUpdate` | M7 → A53 | 28 B | Safety Monitor reports committed state |
| `VssCmdAck` | M7 → A53 | 24 B | Acknowledgement / rejection of command |
| `VssFaultReport` | M7 → A53 | 24 B | Open-circuit, short, actuator timeout, NVM error |

### 4.3 CRC Rule

CRC-16/CCITT-FALSE (initial value 0xFFFF) over all bytes from offset 0 to `sizeof(message) - 4`. The CRC field and trailing pad are the last 4 bytes and are excluded from computation.

### 4.4 IPC Protocol Options — Decision Required

The current wire format is a custom 24-28 byte binary protocol over RPmsg. This is the simplest and lowest-latency option for the NXP S32G2 target, but it locks both sides (Rust and AUTOSAR C) into a bespoke schema. As the platform targets multiple SoCs and OEM customers, we need to decide whether to stay with the custom protocol or adopt a standard middleware protocol.

Three options are on the table:

#### Option A: Custom binary protocol over RPmsg

Fixed-size structs with hand-coded encode/decode on both sides. Optimized for the A53↔M7 shared-memory boundary.

| Argument | For | Against |
|----------|-----|---------|
| **Latency** | ~100 µs round-trip over RPmsg shared memory. Lowest possible latency for A53↔M7. | Only applies to RPmsg on heterogeneous SoCs. Irrelevant when Safety Monitor runs on a separate ECU. |
| **Simplicity** | 28 bytes, no serialization library, no runtime dependencies. M7 C code is a single header file with no external stack. Fits MISRA-C/no-malloc constraint trivially. | Every schema change requires coordinated updates to both the Rust encoder and the C header. No backward/forward compatibility — version bump breaks both sides. |
| **Footprint** | Zero overhead on M7 flash/RAM. No middleware stack to fit on Cortex-M7 TCM. | Not reusable outside the A53↔M7 boundary. If we later need the same signal flow between ECUs over Ethernet, we need a second protocol anyway. |
| **Tooling** | None needed — just a struct definition. | No service discovery, no IDL, no code generation. Schema is documentation, not machine-checkable. Drift between Rust and C is caught only by integration tests. |
| **OEM perception** | N/A | OEM engineering teams may question a proprietary wire format. Standards like SOME/IP and DDS are recognized by AUTOSAR and the wider SDV ecosystem. |

**Recommendation**: Best choice for the A53↔M7 boundary on heterogeneous SoCs where the M7 is resource-constrained. Lowest latency, simplest M7 implementation.

#### Option B: SOME/IP (AUTOSAR-standard service-oriented middleware)

Replace the custom binary protocol with SOME/IP serialization. On RPmsg, SOME/IP messages would be carried as the payload (SOME/IP-over-RPmsg). On Ethernet topologies, SOME/IP runs natively over UDP/TCP.

| Argument | For | Against |
|----------|-----|---------|
| **AUTOSAR alignment** | SOME/IP is the standard AUTOSAR Adaptive and Classic service-oriented communication protocol. M7 AUTOSAR Classic already has a SOME/IP stack (AUTOSAR SWS SomeIpXf). OEMs expect it. | AUTOSAR Classic SOME/IP implementations are heavy — typically 50-200 KB flash, 10-30 KB RAM. May be tight on M7 Cortex-M7 TCM depending on what else runs there. |
| **Portability** | Same protocol works over RPmsg (local), Ethernet (remote ECU), and even intra-A53 (container-to-container). One protocol for all topologies. The `SomeIpBus` adapter already exists in the architecture as a planned transport. | Adds ~300-500 µs latency over Ethernet vs. ~100 µs for raw RPmsg. For body domain functions outside PEPS critical path, this is acceptable. For PEPS, M7 handles the critical path locally regardless. |
| **Service discovery** | SOME/IP-SD enables automatic service discovery. New ECUs or features can announce themselves. Valuable in domain controller topologies with multiple ECUs. | Overkill for a single-SoC body controller with a fixed, known set of features. Service discovery adds boot-time complexity and a failure mode (SD timeout). |
| **Schema evolution** | SOME/IP-over-FIDL or SOME/IP-over-ARXML provides IDL-based code generation. Schema changes are machine-checkable — no Rust/C drift risk. | Requires tooling: `vsomeip` or `AUTOSAR SomeIpXf` code generator. Adds a build dependency. FIDL/ARXML files must be maintained alongside the VSS overlay. |
| **Ecosystem** | `vsomeip` (open-source, Covesa-adjacent) provides a mature Linux implementation. Rust bindings exist (`someip-rs`, `vsomeip-rs`). AUTOSAR Classic has native support. | `vsomeip` is C++ and pulls in Boost. Rust bindings are thin wrappers with FFI overhead. On M7, the AUTOSAR vendor's SOME/IP stack may have licensing costs. |
| **OEM acceptance** | OEMs with existing SOME/IP infrastructure can integrate the body controller into their vehicle network without a protocol bridge. Strong selling point. | OEMs without SOME/IP (older programs, low-cost platforms) gain no benefit and pay the complexity cost. |

**Recommendation**: Best choice for Ethernet-facing interfaces (A53 ↔ other ECUs, diagnostic tools, cloud gateway) and domain controller topologies where the Safety Monitor runs on a separate ECU. Can coexist with custom RPmsg on the local A53↔M7 boundary.

#### Option C: DDS (Data Distribution Service)

Replace the custom protocol with OMG DDS (e.g., Eclipse Cyclone DDS, RTI Connext, eProsima Fast DDS). DDS is a publish-subscribe middleware with QoS policies, automatic discovery, and strong typing via IDL.

| Argument | For | Against |
|----------|-----|---------|
| **Pub-sub native** | DDS is inherently publish-subscribe, which maps directly to the VSS signal model (signals are topics, features are subscribers/publishers). No request-reply shim needed for state updates. | The DoorLock arbiter's request-reply-with-ACK pattern does not map naturally to pub-sub. Requires a DDS request-reply overlay (DDS-RPC) or a separate command channel. |
| **QoS policies** | DDS QoS is extremely rich: reliability, durability (persistent last-value), deadline, liveliness, ownership strength. Ownership strength maps directly to our priority model — the arbiter could theoretically be replaced by DDS ownership QoS. | QoS richness is also complexity. Misconfigured QoS causes silent data loss or unbounded memory growth. Debugging QoS mismatches is notoriously difficult. The team needs DDS expertise. |
| **ROS 2 / SDV alignment** | DDS is the default middleware for ROS 2 and is gaining traction in the SDV (Software-Defined Vehicle) ecosystem. AUTOSAR Adaptive R22-11 added a DDS binding. Forward-looking choice. | AUTOSAR Classic CP (M7) has **no native DDS support**. Running DDS on Cortex-M7 is not practical — DDS implementations require dynamic allocation, threads, and typically 500 KB+ flash. Would need a DDS-to-RPmsg bridge on the A53 side, adding a translation layer. |
| **Discovery** | DDS discovery is automatic and decentralized (no broker). Nodes find each other on the network without configuration. Ideal for dynamic topologies. | Body controller topology is static and known at build time. Automatic discovery adds boot latency (1-5 seconds for participant matching) and a failure mode. We know exactly which features talk to which signals. |
| **Typing / IDL** | DDS IDL provides strong typing and code generation for C, C++, Rust, Python. Schema evolution with `@optional` and `@key` annotations. | Another IDL to maintain alongside VSS overlay and AUTOSAR ARXML. Three schema definitions for the same data is a maintenance burden. |
| **Latency** | Shared-memory transport (e.g., Cyclone DDS `iox` or iceoryx integration) achieves ~10-50 µs for intra-A53. Over Ethernet: ~200-500 µs (comparable to SOME/IP). | Over RPmsg to M7: not available natively. Would require a bridge process on A53 that translates DDS topics to RPmsg binary frames — adding latency and a failure point. |
| **Ecosystem** | Cyclone DDS is open-source (Eclipse), Apache 2.0 licensed. Rust bindings (`cyclonedds-rs`) exist. Large community from ROS 2. | RTI Connext (the most feature-complete DDS) is commercial and expensive. Open-source alternatives (Cyclone, Fast DDS) have fewer QoS features and less automotive validation. No ASIL-certified DDS implementation exists today. |
| **OEM acceptance** | Some OEMs (particularly those investing in ROS 2 or AUTOSAR Adaptive) are exploring DDS. Forward-looking choice for SDV roadmaps. | Most body domain OEMs today use SOME/IP or proprietary protocols. DDS is more common in ADAS/AD domains. Proposing DDS for a body controller may raise questions about maturity and fit. |

**Recommendation**: Best choice for A53-to-A53 and A53-to-network communication when OEM roadmap includes ADAS integration, ROS 2 interop, or AUTOSAR Adaptive. The M7 gap is significant — DDS cannot run on AUTOSAR Classic, so a bridge layer is unavoidable. Not suitable for the safety-critical A53↔M7 boundary.

#### Comparison Summary

| Criterion | Custom RPmsg | SOME/IP | DDS |
|-----------|-------------|---------|-----|
| A53↔M7 latency | ~100 µs | ~100 µs (RPmsg) / ~500 µs (Ethernet) | Not native; requires bridge |
| M7 feasibility | Trivial (28-byte struct) | Possible (AUTOSAR SomeIpXf) but heavy | **Not feasible** on Cortex-M7 |
| Ethernet topology | Needs second protocol | Native — same protocol everywhere | Native — same protocol everywhere |
| AUTOSAR Classic support | N/A (custom) | Yes (standard) | No |
| AUTOSAR Adaptive support | No | Yes | Yes (R22-11) |
| Schema evolution | Manual, error-prone | IDL code-gen (FIDL/ARXML) | IDL code-gen (OMG IDL) |
| OEM familiarity | Low (proprietary) | High (industry standard) | Medium (growing in SDV) |
| Team learning curve | None | Medium (SOME/IP concepts, tooling) | High (QoS, discovery, IDL) |
| Priority/arbitration | Application-layer arbiter | Application-layer arbiter | Could use DDS ownership QoS (but adds complexity) |
| License cost | None | Vendor SOME/IP stack may have cost | Cyclone DDS free; RTI Connext commercial |

#### Recommended Approach

**A53 ↔ M7 boundary**: Custom RPmsg protocol. The M7 Cortex-M7 cannot run SOME/IP or DDS stacks without significant resource pressure. RPmsg shared memory provides the lowest latency (~100 µs) and simplest M7 implementation (28-byte struct, no middleware). This boundary is internal to the SoC and not exposed to OEMs or external tools.

**A53 ↔ other ECUs / external interfaces**: SOME/IP over Ethernet. This is the AUTOSAR-standard service-oriented protocol, works natively over GbE, and is expected by OEM vehicle network architects. The `SomeIpBus` adapter implements the `SignalBus` trait — no feature code changes.

**A53 ↔ ROS 2 / AUTOSAR Adaptive (if required by OEM roadmap)**: DDS adapter (`DdsBus` implementing `SignalBus`). Only applicable when OEM customers require ROS 2 interop or AUTOSAR Adaptive on the A53. DDS does not replace the A53↔M7 boundary — it cannot run on AUTOSAR Classic.

**Multiple transports can coexist.** The `SignalBus` trait is the architectural insurance policy — it guarantees that the protocol decision can be revisited or mixed without rewriting features. A production deployment may use RPmsg for M7, SOME/IP for Ethernet backbone, and DDS for an ADAS gateway simultaneously.

---

## 5. Application Layer (Rust on A53)

The Rust application layer runs in a Podman container on the A53. It is QM software.

### 5.1 Architecture Summary

```
┌─────────────────────────────────────────────────┐
│  Features (Hazard, Turn, AutoRelock, etc.)       │
│  Each feature: Arc<DomainArbiter> + Arc<SignalBus>│
├─────────────────────────────────────────────────┤
│  Domain Arbiters (Lighting, DoorLock, Horn, etc.)│
│  Static allow-lists, priority resolution         │
├─────────────────────────────────────────────────┤
│  SignalBus trait (portability seam)               │
├────────────┬────────────┬───────────┬───────────┤
│ RpmsgBus   │ GlinkBus   │ SomeIpBus │ MockBus   │
│ (NXP S32G2)│ (Qualcomm) │ (Ethernet)│ (CI/test) │
└────────────┴────────────┴───────────┴───────────┘
```

**Key principle**: Every feature and the arbiters depend only on the `SignalBus` trait. Swapping SoC means replacing one adapter file. No feature code changes.

### 5.2 Component Inventory

| Component | Requirement | Implementation Status |
|-----------|-------------|----------------------|
| **SignalBus trait** | Portability seam — all features and arbiters depend only on this trait | Done (`vss-bridge/src/signal_bus.rs`) |
| **MockBus** (test adapter) | In-memory adapter for unit tests and CI with no hardware dependency | Done (`vss-bridge/src/adapters/mock.rs`) |
| **RpmsgBus** transport adapter | NXP S32G2 transport — opens `/dev/rpmsg0` and `/dev/rpmsg1`, encodes/decodes IPC messages | Not started |
| **SomeIpBus** transport adapter | SOME/IP over Ethernet for domain controller topologies and non-NXP SoCs | Not started |
| **GlinkBus** transport adapter | Qualcomm GLINK IPC for SA8775P targets | Not started |
| **DdsBus** transport adapter | DDS middleware for ROS 2 / AUTOSAR Adaptive interop (if required by OEM) | Not started |
| **IPC message schema** | Binary wire format (28 B) with CRC-16, matching Rust and C implementations | Done (`vss-bridge/src/ipc_message.rs`) |
| **Signal ID constants** (86 signals) | Stable 32-bit IDs from `vspec2id`, shared between Rust and AUTOSAR C | Done (`vss-bridge/src/signal_ids.rs`) |
| **Lighting arbiter** (DomainArbiter) | Per-signal priority resolution for direction indicators, beams, DRL | Done (`vss-bridge/src/arbiter.rs`) |
| **DoorLock arbiter** | Serialized command queue with ACK handshake, crash-unlock protection | Done (`vss-bridge/src/arbiter.rs`) |
| **Horn / Comfort arbiters** | Domain arbiters with empty allow-lists, ready for future features | Done (`vss-bridge/src/arbiter.rs`) |
| **HazardLighting** feature | Both indicators at HIGH on hazard switch. No ignition gate. | Done + tests (`vss-bridge/src/features/hazard_lighting.rs`) |
| **TurnIndicator** feature | Stalk-driven indicators at MEDIUM. Ignition-gated (ON/START only). | Done + tests (`vss-bridge/src/features/turn_indicator.rs`) |
| **AutoRelock** feature | 45s relock timer. Crash-disables until full power cycle. | Done + tests (`vss-bridge/src/features/auto_relock.rs`) |
| **LockFeedback** feature | On lock/unlock state change, blink indicators at HIGH (overlay), then self-release | Not started |
| **KeyfobPeps** feature | Subscribes to `Body.PEPS.KeyPresent`, submits DoorLock arbiter request | Not started |
| **KeyfobRke** feature | Keyfob remote lock/unlock/double-lock | Not started |
| **AutoLock** feature | Speed-based auto-lock (requires Fully Awake for `Vehicle.Speed`) | Not started |
| **LowBeam / HighBeam** features | Light switch → beam control | Not started |
| **DRL** feature | Ignition ON + parking brake off → DRL on | Not started |
| **DoorTrimButton** feature | Per-door interior lock/unlock buttons | Not started |
| **CrashUnlock** feature | Safety-critical: unlock all doors on crash detection | Not started |
| **PhoneApp / PhoneBle / NfcCard / NfcPhone** | Connectivity-based lock sources (cloud, BLE digital key, NFC) | Not started |
| **SleepInhibitManager** | RAII-based wake claims with reaper and max-hold enforcement | Done + tests (`vss-bridge/src/sleep_inhibit.rs`) |
| **4-tier config system** | Compile-time, vehicle-line, variant, and dealer-configurable parameters | Done + tests (`vss-bridge/src/config.rs`) |
| **kuksa.val gRPC sync** | Bidirectional sync between SignalBus and kuksa.val data broker | Not started |
| **WebSocket bridge** (L5 → L6 HMI) | Connect web HMI to live signal bus instead of mock store | Not started |
| **SOVD Gateway** | HTTP/REST diagnostic API per ASAM SOVD V1.0.0. Classic Diag Proxy for M7 UDS, native endpoints for HPC services. See section 3.2.3. | Not started |
| **VSS signal overlay** | Switch inputs, door lock inputs, door extensions | Done (`vss-bridge/overlay/Body/`) |
| **Web HMI** (sensor simulator) | SVG vehicle views with toggle/slider controls for all 86 signals | Done (`vss-hmi-body-sensors.html`) |
| **Gherkin feature specs** (10 features) | Requirements and acceptance scenarios for all body features | Done (`features/*.feature`) |
| **Config examples** (6 variants) | JSON calibration files for sedan, truck, coupe, premium, base, offroad | Done (`config/*.json`) |

### 5.4 Four-Tier Configuration System

Parameters are stratified by who sets them and when they change:

| Tier | Name | Set By | When | Storage | Example |
|------|------|--------|------|---------|---------|
| 1 | Compile-time constants | Developer | Build time | Rust `const` | IPC magic, crash lockout (10s), motor timeout (300ms) |
| 2 | Vehicle-line calibration | Vehicle program | Container build | JSON in container image | Auto-relock timeout (45s), PEPS LF window (3s), DRL brightness |
| 3 | Variant/trim calibration | Assembly plant | Flash at BOM | JSON on persistent volume | Auto-lock speed, double-lock enabled, NFC enabled, door count |
| 4 | Dealer configurable | Dealer tech | UDS 0x2E at runtime | M7 NVM (pushed to A53) | Auto-relock enable, horn chirp, courtesy light timeout |

**Variant door configuration**: The platform supports 2-door (coupe) and 4-door layouts, plus removable doors (Bronco/Wrangler). The `DoorConfig` struct generates VSS signal paths only for doors that are physically present.

---

## 6. Feature Requirements Summary

Each feature is an independent async module. No feature imports another feature. Features subscribe to physical switch/stalk inputs (sensor overlay signals), never to actuator outputs — this prevents feedback loops.

### 6.1 Lighting Domain

| Feature | Input Signal | Output | Priority | Ignition Gate | Min ECU State |
|---------|-------------|--------|----------|--------------|---------------|
| **HazardLighting** | `Body.Switches.Hazard.IsEngaged` | Both DirectionIndicator.IsSignaling | HIGH (3) | **None** — works in OFF/ACC/ON/START | Locally Awake |
| **TurnIndicator** | `Body.Switches.TurnIndicator.Direction` + `Vehicle.LowVoltageSystemState` | DirectionIndicator.Left/Right.IsSignaling | MEDIUM (2) | **ON or START only** — deactivates on OFF/ACC | Locally Awake |
| **LockFeedback** | `Body.Doors.*.IsLocked` (state change) | Both DirectionIndicator.IsSignaling | HIGH (3, overlay) | None | Locally Awake |
| **LowBeam** | `Body.Lights.LightSwitch` | `Beam.Low.IsOn` | MEDIUM (2) | None (works in ACC) | Locally Awake |
| **HighBeam** | `Body.Switches.HighBeam.IsEngaged` | `Beam.High.IsOn` | MEDIUM (2) | ON or START | Locally Awake |
| **DRL** | `Vehicle.LowVoltageSystemState` + `Chassis.ParkingBrake.IsEngaged` | `Running.IsOn` | MEDIUM (2) | ON or START, parking brake off | Fully Awake |

**Blink timing**: Features set boolean intent (`IsSignaling = true/false`). The 1-2 Hz UN R48-compliant blink cadence is the LED driver IC or body ECU firmware's responsibility. Features never set timers for blink patterns.

**Priority resolution**: Hazard HIGH (3) overrides Turn MEDIUM (2). LockFeedback uses HIGH to overlay a brief blink pattern on top of active hazard or turn, then self-releases. Equal priority: latest request wins.

### 6.2 Door Lock Domain

Serialized command queue (not priority-based). Lock motor takes ~300 ms — cannot accept concurrent commands.

| Feature | Input Signal | Command(s) | Special Rules |
|---------|-------------|-----------|---------------|
| **KeyfobPeps** | `Body.PEPS.KeyPresent` | UNLOCK, LOCK | Post-facto (M7 already drove actuator) |
| **KeyfobRke** | `Body.Switches.Keyfob.LockButton` | LOCK, UNLOCK, DOUBLE_LOCK | |
| **AutoLock** | `Vehicle.Speed` | LOCK | Requires Fully Awake (cross-domain signal) |
| **DoorTrimButton** | `Body.Switches.DoorTrim.*.LockButton` | LOCK, UNLOCK | Per-door buttons |
| **AutoRelock** | `Body.Doors.*.IsLocked` + `*.IsOpen` + crash + ignition | LOCK (45s timer) | Crash → DISABLED until full power cycle (OFF→ON) |
| **CrashUnlock** | `Vehicle.Safety.CrashDetected` | UNLOCK | **Protected**: cannot be replaced in queue. Triggers 10s lockout after dispatch. |
| **PhoneApp** | `Body.Connectivity.RemoteLock` | LOCK, UNLOCK | Cloud-connected |
| **PhoneBle** | `Body.Connectivity.BleLock` | LOCK, UNLOCK | BLE digital key |
| **NfcCard** | `Body.Connectivity.NfcCardPresent` | LOCK, UNLOCK | Physical NFC card |
| **NfcPhone** | `Body.Connectivity.NfcPhonePresent` | LOCK, UNLOCK | NFC key on phone |

### 6.3 Safety Requirements

| Requirement | Implementation |
|-------------|---------------|
| ASIL-B state authority is always M7 | A53 arbiter validates, but M7 Safety Monitor independently validates and can veto |
| Priority table compiled into both sides | Shared JSON source → Rust constants + C lookup table at build time |
| Crash unlock cannot be overridden | DoorLock arbiter rejects all requests for 10s after CrashUnlock dispatch |
| AutoRelock disables after crash | Stays disabled until full power cycle (LowVoltageSystemState OFF→ON) |
| Hold-last on fault | CAN/LIN node loss does not cause state transition to off |
| NVM integrity | CRC32 over every state record; safe-state defaults on CRC failure |
| A53 cannot block safety | M7 handles critical-path operations (PEPS, crash unlock, light switch) while A53 is asleep or unresponsive |

---

## 7. Container Topology (A53)

```
A53 cluster (Cortex-A53 x4)
└── Android Automotive OS
    └── Podman (rootless)
        ├── kuksa-val          RHEL base  gRPC :55555
        ├── vss-bridge         RHEL base  WS :8080  gRPC client
        ├── hmi-server         RHEL base  HTTP :3000  static files
        └── sovd-gateway       RHEL base  HTTP :8443
                               REST/JSON diagnostic API (ASAM SOVD)
                               Classic Diag Proxy (SOVD→UDS→M7)
                               Native endpoints (HPC services)

    └── CarService / VHAL      (AAOS process, not containerised)
    └── Android Auto apps      (AAOS process, not containerised)
```

- All inter-container communication on localhost (sub-millisecond, no TLS)
- `/dev/rpmsg0` and `/dev/rpmsg1` bind-mounted into `vss-bridge` container only
- Container images built from `registry.redhat.io/ubi9/ubi-minimal`
- Rust binary statically linked (musl target) — no runtime dependencies
- OTA: Podman pulls new image → canary validation → atomic container replacement

---

## 8. Signal Inventory

86 VSS signals are currently defined across the platform. The overlay extends standard COVESA VSS v4.0 with:

- **Switch/stalk inputs** (`overlay/Body/SwitchInputs.vspec`): physical hazard switch, turn stalk, high beam switch, parking brake
- **Door lock inputs** (`overlay/Body/DoorLockInputs.vspec`): keyfob RKE, door trim buttons, phone app, BLE key, NFC card/phone, per-door IsRemoved, crash detected
- **Door extensions** (`overlay/Body/DoorExtended.vspec`): latch status (UNLATCHED/HALF/FULL), double-lock, child lock

Signal IDs are generated by running `vspec --export-id` against the combined catalog. These 32-bit IDs are used in the RPmsg wire format and must match between the Rust constants file and the AUTOSAR M7 lookup table.

---

## 9. Development Workflow

### 9.1 For Hardware Team

1. **Design the ECU board** around S32G2 per section 2.2
2. **Provide the BSP**: Linux kernel with PREEMPT_RT, SocketCAN driver, RPmsg driver, device tree
3. **Provide body ECU node schematics** and CAN/LIN DBC files for each peripheral module
4. **Validate PEPS timing** on bench with actual capacitive touch sensors, LF antennas, and keyfob

### 9.2 For Embedded SW Team (M7 / AUTOSAR)

1. **Safety Monitor SWC**: implement per section 3.1.1 — this is the first critical deliverable
2. **PEPS SWC**: implement per section 3.1.2
3. **UDS server**: implement per section 3.1.3
4. **CAN/LIN stack**: configure AUTOSAR COM + LLCE, generate DBC from VSS overlay
5. **Power Manager**: implement 5-state ECU power management per section 3.1.5
6. **IPC wire format**: implement `vss_ipc_message.h` matching the Rust `ipc_message.rs` byte-for-byte

### 9.3 For Application SW Team (Rust / A53)

1. **RpmsgBus adapter**: enables integration testing with real M7 hardware
2. **kuksa.val gRPC sync**: enables full signal flow from M7 → Rust → kuksa.val → HMI
3. **Feature business logic**: LockFeedback, KeyfobPeps, KeyfobRke, AutoLock, LowBeam, HighBeam, DRL, DoorTrimButton, CrashUnlock, connectivity features
4. **WebSocket bridge**: connect the HMI to live signal bus
5. **SomeIpBus / GlinkBus / DdsBus adapters**: transport adapters for non-NXP SoCs and Ethernet topologies
6. **SOVD Gateway container**: HTTP/REST server (axum or actix-web) implementing ASAM SOVD V1.0.0
7. **Classic Diagnostic Proxy**: SOVD→UDS translation layer for M7 diagnostics (DTC read/clear, dealer config, ECU ID)
8. **Native SOVD endpoints**: container health, OTA status, signal snapshot, feature config status
9. **SOVD authentication**: TLS + token-based auth for cloud/remote access; local-only mode for workshop

### 9.4 Building and Testing

```bash
# Build the Rust crate
cd vss-bridge && cargo build

# Run unit tests (uses MockBus — no hardware needed)
cargo test

# Run a specific feature's tests
cargo test --lib features::turn_indicator
cargo test --lib features::hazard_lighting
cargo test --lib features::auto_relock

# Check compilation without building
cargo check
```

**Note**: `cargo build` requires `protoc` (Protocol Buffers compiler) for the kuksa.val gRPC stubs. Install via: `brew install protobuf` (macOS) or `apt install protobuf-compiler` (Linux).

---

## 10. Key Architectural Decisions

| Decision | Rationale |
|----------|-----------|
| **Rust for application layer** | Memory safety without GC. async/await maps naturally to the event-driven body controller pattern. Cross-compilation to ARM targets. |
| **M7 owns all ASIL-B state** | ISO 26262 boundary. A53 Linux/AAOS cannot be certified ASIL-B. M7 validates independently — A53 crash cannot corrupt safety state. |
| **SignalBus trait as portability seam** | Decouples all feature logic from transport. SoC swap = one adapter file + BSP. |
| **Domain arbiters with static allow-lists** | No runtime registration. Feature cannot claim a priority it wasn't assigned. Both Rust and M7 validate independently. |
| **Centralized config over per-feature config** | Platform product: OEMs replace features but reuse infrastructure. Central config survives feature swaps. |
| **Features subscribe to inputs, not outputs** | Prevents feedback loops. Hazard switch ≠ hazard lamp. Correctly models the hardware separation. |
| **M7 handles critical path while A53 sleeps** | PEPS unlock, crash unlock, light switch all work without A53. A53 is never in the critical path. |
| **ECU states independent of ignition** | M7 can wake A53 without ignition change. Features run when needed, not just when ignition is ON. |
| **No blink timing in features** | UN R48 cadence is LED driver / body ECU firmware responsibility. Features set boolean intent only. |
| **Custom RPmsg for A53↔M7; SOME/IP for Ethernet** | RPmsg is lowest-latency for the on-chip boundary; SOME/IP is the AUTOSAR standard for Ethernet. DDS available for ROS 2/Adaptive interop. SignalBus trait allows all three to coexist — see section 4.4. |
| **SOVD Gateway for diagnostics** | ASAM SOVD V1.0.0 HTTP/REST API on A53. Wraps existing M7 UDS server via Classic Diagnostic Proxy. Enables cloud diagnostics, tablet-based dealer tooling, and OEM diagnostic integration — see section 3.2.3. |
| **SoC optionality (S32G reference, S32N/SA8775P supported)** | S32G2 is the reference for standalone body controllers. S32N for zonal consolidation, SA8775P for cockpit-body fusion. SignalBus trait portability means SoC swap = one adapter file + BSP. See section 2.1.1. |
| **OS optionality (Yocto reference, Red Hat/AAOS supported)** | Yocto for cost-sensitive programs. Red Hat In-Vehicle OS for OEMs requiring vendor-backed 10+ year support SLA. AAOS for cockpit-body fusion. Platform provider absorbs OS complexity — OEM licenses the platform, not the OS. See section 2.1.2. |
| **AUTOSAR Classic on safety core (non-negotiable)** | Full BSW stack (CAN/LIN, UDS, NvM, COM) that body controllers require. Platform reduces AUTOSAR scope by moving feature logic to Rust/QM, cutting development effort and seat licenses. See section 2.1.3. |
