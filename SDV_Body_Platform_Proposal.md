# Software-Defined Body Controller Platform
## A Turnkey SDV Platform for Next-Generation Body Control

**Prepared for:** OEM Senior Leadership, Vehicle Architecture & Engineering
**Prepared by:** [Company Name] — Automotive Software Platform Services
**Date:** April 2026
**Classification:** OEM Confidential

---

## 1. The Problem with Today's Body Controller Architecture

Current body controllers are **frozen at manufacturing**. Every ECU runs monolithic firmware — tightly coupled to the specific SoC vendor, CAN database, and body ECU supplier. This creates compounding costs:

| Pain Point | Business Impact |
|-----------|----------------|
| **Body supplier owns the software integration and build - binary** | OEM cannot add, modify, or remove a body feature without a change request to the supplier. Every CR has NRE cost, lead time, and contractual negotiation. The OEM pays for features and changes it could build in-house. |
| Feature changes require full ECU reflash | 12–18 month lead time for even minor feature updates. Field campaigns for software bugs require dealer visits. |
| Each SoC vendor requires a complete rewrite (Millions of $). Supplier owns the port — OEM has no leverage to accelerate or dual-source. |
| Fuzzy separation between safety-critical and application logic | Full ASIL-B certification scope for *any* software change, even a welcome-light animation. Supplier bills ASIL rates for QM-level work. |
| Body features are hard-coupled to the supplier's software stack | Switching body suppliers means rewriting all feature logic. Multi-year qualification cycle. OEM is locked in for the life of the platform. |
| No post-sale feature delivery | Zero software revenue opportunity after vehicle sale. Competitor OEMs (Tesla, BMW, Mercedes) already monetize post-sale body features. |

**The core issue:** the OEM does not own its own body feature logic. The supplier owns the source code, the SoC-specific integration, the CAN database, and the safety certification scope. Every feature request flows through the supplier's release cycle. The OEM is a customer of its own body controller.

This is not a technology problem — it is an **ownership problem**. We have built a platform that returns feature ownership to the OEM while evolving the traditional body supplier into a **Platform Provider** — focused on what they do best: AUTOSAR BSW, Linux distribution, hardware schematics/PCB layout, CAN/LIN I/O, and ASIL-B certification.

---

## 2. What We are building

A **production-ready, layered, software-defined body controller platform** built on three principles:

1. **Separate what changes from what must not change** — Application features (Rust, QM) run on application cores; safety-critical actuation (C/AUTOSAR, ASIL-B) runs on real-time cores. They communicate over a well-defined IPC boundary.

2. **Standardize the signal interface** — All features speak COVESA VSS (Vehicle Signal Specification), an open industry standard. No proprietary signal dictionaries.

3. **Containerize application software** — Feature logic and the VSS broker run as OCI containers with independent OTA update paths. No full-ECU reflash required. No supplier change request required.

### Architecture at a Glance

```
                    ┌ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┐
                      Optional: diagnostic/eng. HMI
                    │   (WebSocket — not required for   │
                        body feature operation)
                    └ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┘
                                    │ ws://
┌─────────────────────────────────────────────────────────┐
│  Application Layer (Rust containers) — OEM-OWNED        │
│    Feature Business Logic · Signal Arbiter · kuksa sync  │
│    ── runs on Cortex-A53, QM, OTA-updatable ──           │
├═════════════════════════════════════════════════════════╡
│  ▲ RPmsg IPC boundary — 28-byte CRC-protected messages  │
├═════════════════════════════════════════════════════════╡
│  AUTOSAR Classic (M7) — OEM + PLATFORM PROVIDER         │
│    OEM: ASIL-B Application SWCs (optional)               │
│    Platform Provider: BSW, Safety Monitor, NVM, drivers  │
│    ── runs on Cortex-M7, certified, rarely updated ──    │
├─────────────────────────────────────────────────────────┤
│  CAN FD / LIN → Body ECU nodes (lamps, locks, motors)   │
└─────────────────────────────────────────────────────────┘
```

**Key insight:** The IPC boundary is also an **ownership boundary**. Above the line: OEM-owned feature business logic, updatable via OTA without Platform Provider involvement. Below the line: shared between OEM ASIL-B application SWCs (which the OEM may choose to develop in-house) and the Platform Provider's domain — AUTOSAR BSW, Linux distribution, hardware schematics/PCB layout, and CAN/LIN I/O drivers. The Platform Provider delivers the foundation; the OEM builds the differentiation.

---

## 3. What Your Organization Gets

### 3.1 You Own the Feature Logic — Your Supplier Becomes a Platform Provider

Today, your body supplier owns the feature source code. You specify requirements, the supplier implements them, and you pay NRE for every change. This model made sense when body controllers were simple relay logic. It does not make sense when customers expect post-sale feature updates, personalization, and software-defined differentiation.

**Our platform redefines the supplier relationship:**

| Responsibility | Owner | Why |
|---------------|-------|-----|
| Feature business logic (hazard, turn, PEPS, DRL, auto-lock, …) | **You (OEM)** | This is your differentiating IP. You should own what the vehicle *does*. |
| ASIL-B application SWCs (optional — e.g., PEPS state machine) | **You (OEM)**, if desired | You can extend ownership into safety-critical application logic on the M7 as your competency grows. |
| Classic AUTOSAR BSW, Linux distribution, hardware schematics/PCB | **Platform Provider** | This is the Platform Provider's core competency — the reusable foundation you build upon. |
| Safety Monitor (ASIL-B validation, NVM, actuator control) | **Platform Provider** (or you) | Can be delivered by the Platform Provider as a certified component, or developed in-house as you mature. |
| CAN/LIN body ECU nodes, I/O hardware | **Platform Provider** | Physical hardware and low-level drivers — the traditional supplier strength. |
| Signal interface between layers | **Shared (COVESA VSS + IPC schema)** | Open standard. Neither party is locked to the other's proprietary format. |

The Platform Provider delivers a validated hardware platform with AUTOSAR BSW, Linux, and reference Safety Monitor. Your engineering team writes the feature business logic that generates IPC commands. **The Platform Provider does not need to know or care how your feature logic works** — the Safety Monitor only validates that commands are well-formed, CRC-correct, and priority-compliant.

This is not about removing your supplier — it is about **evolving the relationship** from "feature implementer where you are the customer" to "platform provider where you are the application developer."

**What this means for you:**
- You control your own feature roadmap — no CR queue, no NRE per feature change
- Platform Provider scope is well-bounded and reusable across your programs — lower supplier cost per vehicle
- You can switch Platform Providers without rewriting feature logic (the IPC schema is the contract, not a proprietary API)
- You can progressively take ownership of ASIL-B application SWCs as internal competency grows
- Feature development uses your own engineering team with standard tools (Rust, CI/CD, OTA) rather than waiting for supplier release cycles

**Our role:** We deliver this platform ready to integrate, provide ongoing maintenance and updates, and support your engineering team as they build feature business logic on top of it. We handle the plumbing so your engineers focus on differentiation.

### 3.2 SoC Portability — Protect Against Silicon Supplier Lock-In

The entire application layer depends on a single Rust trait (`SignalBus`) — a 10-line interface contract. Migrating from NXP S32G2 to Qualcomm SA8775P, Renesas R-Car S4, or TI TDA4 requires changing **exactly two things**:

1. The Linux BSP / device tree (L2) — standard platform work
2. One transport adapter file in L5 — we swap `rpmsg.rs` for `glink.rs`

**Zero feature code changes. Zero re-testing of your application logic.** All 8 feature modules, the signal arbiter, and the VSS broker — all unchanged. This transforms SoC selection from a multi-year re-architecture into a 2–4 week adapter swap that we deliver.

**What this means for you:** Leverage in SoC supplier negotiations. Ability to dual-source. Protection against supply chain disruption (as experienced industry-wide in 2021–2023). Your feature investment carries forward across silicon generations.

### 3.3 OTA Feature Delivery — Post-Sale Software Revenue

Application containers update independently via standard OCI image pull:

```
Vehicle receives new container image
  → Podman pulls image
  → Canary validation (smoke-test endpoints)
  → Atomic swap (zero-downtime)
  → Safety Monitor is NOT touched
```

Because application features are QM and the Safety Monitor is separately certified ASIL-B, **new body features can ship without re-certifying the safety layer**. This enables:

- Post-sale feature activation (e.g., ambient lighting themes, welcome sequences, gesture-based lock)
- A/B testing of feature behavior across fleet segments
- Rapid bug fixes (days, not months)
- Subscription-based feature upsell (industry trend: BMW, Mercedes, Tesla)

**What this means for you:** New recurring revenue stream. Faster time-to-market. Reduced warranty software campaign costs. Your body controller becomes a software product, not a hardware fixture.

### 3.4 Safety Without Compromise — ASIL-B Where It Matters, QM Everywhere Else

The architecture enforces a hard boundary:

| Domain | Safety Level | Update Path | Scope |
|--------|-------------|-------------|-------|
| Safety Monitor (M7) | ASIL-B | AUTOSAR SWC update (V-model) | Actuator control, NVM, priority validation |
| Application features (A53) | QM | OTA container pull | Feature logic, cloud sync, diagnostics |

**The Safety Monitor is the sole authority for all physical actuator state.** It validates every command from the application layer — checking CRC, priority claims, and vehicle state constraints. A compromised or buggy application container **cannot** override the safety layer.

Key safety properties:
- **Hold-last on fault:** If a CAN/LIN node stops responding, the last known safe state is preserved — headlamps stay on
- **ASIL-B NVM:** Atomic writes with CRC32, validated on every boot. Survives power loss, container restart, and application crashes
- **Priority arbitration validated on both sides:** The application arbiter and the Safety Monitor both enforce the same priority table, compiled from a shared source of truth
- **PEPS critical path excludes the application layer entirely:** Key-to-unlock in ~61 ms, well within the 200 ms regulatory budget. The A53/Android stack is notified *after* unlock — it's never in the critical path

**What this means for you:** Reduced ASIL certification scope for feature changes. Faster homologation. A defensible safety architecture for regulatory review — designed by engineers who understand ISO 26262 body domain requirements.

### 3.5 Open Standards — Avoid Proprietary Dead Ends

| Component | Standard | Governance |
|-----------|----------|-----------|
| Signal specification | COVESA VSS v4.0 | Industry consortium (BMW, JLR, Volvo, Bosch, …) |
| Signal broker | kuksa.val | COVESA / Eclipse Foundation, Apache 2.0 |
| Container runtime | Podman (OCI) | Red Hat / open source |
| Application language | Rust | Mozilla Foundation / open source |
| Safety layer | AUTOSAR Classic CP | AUTOSAR consortium |
| IPC wire format | Custom, fully documented (28 bytes, CRC-protected) | Delivered to OEM, open spec |

No single-vendor dependency in the application stack — including no lock-in to us. VSS paths are the lingua franca — any tool, any team, any supplier can read and write `Body.Lights.Beam.Low.IsOn` without a proprietary translation layer.

**What this means for you:** Supplier flexibility. Industry-standard tooling. Easier recruitment (Rust/gRPC/containers vs. proprietary RTOS APIs). Interoperability with ecosystem partners. And if you ever outgrow our services, the platform runs on open standards — you walk away with everything.

### 3.6 Development Velocity — Ship Features in Weeks, Not Years

The platform enables a modern software development workflow for your engineering team — impossible with monolithic ECU firmware or a supplier CR-based process:

| Capability | How |
|-----------|-----|
| **Unit-testable features** | MockBus adapter lets every feature module run under `cargo test` with no hardware. CI runs 17+ tests in < 1 second today. |
| **Hardware-free development** | Full application stack runs on a laptop with MockBus. Hardware-in-the-loop only needed for transport adapter validation. No bench time, no vehicle prototype. |
| **Independent team velocity** | Your feature teams own individual modules. Safety team owns the M7 partition. No cross-team blocking. No supplier in the loop for feature iteration. |
| **Incremental delivery** | Each feature module is self-contained. Ship one, ship all eight, or ship any subset. Features can be simple state machines or complex algorithms — the platform does not constrain the implementation approach. |
| **Standard CI/CD pipeline** | Same tools the rest of your software organization uses — Git, CI, container registry, OTA. No proprietary supplier toolchain. |

**What this means for you:** 3–5x faster feature iteration vs. supplier CR cycle. Reduced bench/prototype costs. Parallel team execution. Your engineers build automotive features using mainstream software tools — we maintain the platform underneath.

---

## 4. Platform Maturity — What Exists Today

This is not a slide deck. Working, tested code exists today and is ready for demonstration:

| Component | Status | Tests |
|-----------|--------|-------|
| IPC wire format (28-byte messages, CRC-16) | Complete | 8 tests (roundtrip, tamper detection) |
| SignalBus trait + MockBus adapter | Complete | 5 tests |
| Signal ID catalog (86 VSS signals) | Complete | 4 tests (no duplicates, roundtrip) |
| kuksa.val gRPC client (subscribe + set) | Complete | Compiles, reconnect logic |
| VSS switch input overlay (4 signals) | Complete | Defined in vspec |
| Engineering diagnostic HMI (50 signals) | Complete | Browser-based, used for bench validation |
| Architecture document (v1.1) | Complete | — |

**Remaining deliverables (included in engagement):**
1. Signal Arbiter implementation
2. All 8 feature business logic reference modules
3. RpmsgBus transport adapter (requires S32G2 hardware)
4. Container build pipeline + OTA flow
5. Safety Monitor reference C implementation (M7/AUTOSAR)
6. Hardware-in-the-loop validation on S32G2 eval board
7. Knowledge transfer and onboarding for your feature engineering team

---

## 5. Risk Acknowledgement

| Risk | Mitigation |
|------|-----------|
| Supplier pushback on changed role | The Platform Provider retains the highest-value, highest-margin work: AUTOSAR BSW, Linux distribution, hardware design, ASIL-B Safety Monitor, CAN/LIN I/O. Their scope is *better-defined and more reusable*, not eliminated. Many suppliers are actively pursuing this model (Bosch, Continental, Aptiv SDV initiatives). We can facilitate the transition and provide the integration spec. |
| Building internal body feature engineering competency | This is the strategic investment. Start with a small team (3 engineers) on a single platform. We provide knowledge transfer, reference implementations, and ongoing support. The platform's MockBus and CI pipeline mean your team is productive from day one without hardware. As competency grows, you can progressively take ownership of ASIL-B application SWCs on the M7 as well. |
| Rust is less common in automotive than C/C++ | Rust is adopted by Volvo, Volkswagen CARIAD, Ferrocene (ISO 26262 qualified Rust toolchain). Application layer is QM — no safety certification of the Rust compiler required. We provide Rust training and code review as part of the engagement. |
| COVESA VSS may not cover all OEM-specific signals | VSS supports overlays — the platform already includes two (DoorExtended, SwitchInputs). We add OEM-specific signals without modifying the base spec. |
| Heterogeneous SoC (A53+M7) adds integration complexity | This complexity exists today — the platform makes it *explicit and testable* rather than hidden inside monolithic firmware. The IPC boundary is 28 bytes with CRC; integration failures are caught by the wire format, not by a vehicle-level regression. |
| Container overhead on automotive-grade hardware | Podman is daemonless and rootless. Rust binary is statically linked, < 5 MB. Container startup < 2 seconds. Application layer is not in the safety-critical path. |
| Dependency on a service company | The platform is built entirely on open standards (COVESA VSS, AUTOSAR, OCI containers, Rust). There is no proprietary lock-in. All source code is delivered to you. If the engagement ends, your team can maintain and extend the platform independently. |

---

## 6. Engagement Model

We offer a phased engagement designed to minimize risk and demonstrate value early:

### Phase 1 — Proof of Concept (8–12 weeks)
- Deploy the platform on an S32G2 evaluation board with your selected body feature set
- Hardware-in-the-loop demonstration of end-to-end signal flow (switch input → feature logic → Safety Monitor → CAN actuator)
- Your engineers write their first feature module with our guidance
- **Deliverable:** Working PoC, architecture review with your safety and platform teams

### Phase 2 — Pilot Vehicle Integration (3–6 months)
- Integrate the platform alongside the existing body controller on a designated pilot vehicle program
- Complete all 8 reference feature modules, Safety Monitor, and OTA pipeline
- Knowledge transfer: your team owns feature development by end of phase
- **Deliverable:** Production-candidate platform, trained OEM feature engineering team

### Phase 3 — Ongoing Platform Maintenance
- Platform updates: new SoC adapters, VSS spec upgrades, AUTOSAR BSW alignment
- Feature business logic support and code review
- Scaling support as you extend the platform to additional vehicle programs
- **Deliverable:** Maintained platform, SoC migration support, on-demand engineering

### Why now

Every vehicle program you commit to a monolithic, vendor-locked body controller becomes a liability — unable to deliver post-sale features, unable to migrate SoCs, and requiring full ASIL re-certification for any software change. The platform exists today. The first PoC can be running within weeks.

---

## 7. About Us

[Company Name] is an automotive software services company specializing in SDV platform architecture and body domain engineering. We build platforms that OEMs own — not products that lock them in.

**What we bring:**
- Deep expertise in COVESA VSS, kuksa.val, and AUTOSAR Classic integration
- Production experience with Rust on automotive-grade SoCs (NXP S32G, Qualcomm SA8775P)
- ISO 26262 body domain knowledge (ASIL-B lighting, PEPS, door systems)
- A working platform with 17 passing tests, gRPC integration, and a detailed architecture document — ready for your evaluation today

**Our commitment:** All source code is delivered to you. The platform runs on open standards. There is no proprietary lock-in. We succeed when your team can build and ship body features independently.

---

*We welcome the opportunity to schedule a live platform demonstration and architecture deep-dive with your engineering leadership. Contact: [contact information]*
