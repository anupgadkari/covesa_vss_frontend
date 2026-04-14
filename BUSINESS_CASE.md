# Body SDV Platform — Business Case

**Purpose**: Internal document for management. Makes the case for investing in a licensable body controller software platform, with an honest assessment of what we are building, how we make money, what the risks are, and why the rewards justify the investment.

**Last updated**: 2026-04-14

---

## 1. What We Are Building

We are building a **production-grade software platform for automotive body controllers** — the ECU that controls door locks, lighting, PEPS (passive entry/passive start), comfort features, and related body electronics.

This is not a consulting engagement. It is not a staffing arrangement. It is a **product** — a reusable, configurable, licensable software stack that runs on an OEM's body controller hardware and handles the complete body domain: feature business logic, safety arbitration, diagnostics, over-the-air update infrastructure, and the signal interface to the vehicle network.

### 1.1 What the Platform Contains

| Layer | What it is | Why it matters |
|---|---|---|
| **Feature business logic** (Rust, A-core) | Hazard lighting, turn indicators, auto-relock, PEPS sequencing, crash unlock, headlamp control, lock feedback, connectivity-based lock sources (BLE, NFC, phone app) | This is the intelligence of the body controller. OEMs currently build this from scratch in C on AUTOSAR, taking 2-3 years per program. Our platform delivers it ready-to-configure. |
| **Domain arbiters** (Rust, A-core) | Priority-based signal resolution with static allow-lists. Prevents feature conflicts (e.g., hazard overrides turn signal). | Safety-critical arbitration logic, validated once and reused across programs. An OEM building this from scratch must design, implement, and validate it every time. |
| **Safety Monitor** (C, AUTOSAR Classic, M-core) | ASIL-B state authority. Validates every actuator command from the A-core. Cannot be overridden by QM software. | The ISO 26262 compliance boundary. Certified once, reused across OEM programs. This is the most expensive component to develop and validate from scratch. |
| **Signal abstraction** (SignalBus trait) | Transport-agnostic interface. Features work identically on NXP S32G (RPmsg), Qualcomm SA8775P (GLINK), or over Ethernet (SOME/IP). | Portability guarantee. OEM changes SoC vendor — zero feature code rewrites. This is architectural insurance that protects the OEM's investment in the platform. |
| **Diagnostic gateway** (SOVD, A-core) | ASAM SOVD V1.0.0 HTTP/REST API. Cloud diagnostics, dealer tablet tooling, and OTA integration through a standard interface. | OEMs evaluating SDV body controllers expect SOVD. Wraps the existing M7 UDS server — no M7 changes needed. |
| **Configuration system** | Four-tier: compile-time constants, vehicle-line calibration, variant/trim, dealer-configurable via UDS. | One platform serves sedan, SUV, coupe, and truck. OEM configures, not re-engineers. |
| **BSP and OS integration** | Yocto (reference), Red Hat In-Vehicle OS (10+ year support), AAOS (cockpit fusion). Containerized deployment via Podman. | OEM picks the OS that fits their program. We absorb the integration complexity. |

### 1.2 What the Platform is NOT

- **Not ADAS.** We do not touch perception, planning, or autonomous driving. OEMs consider ADAS a differentiator — they will not outsource it. Our platform explicitly excludes it.
- **Not infotainment.** We do not build the IVI, media, or app ecosystem. Same reason — OEMs differentiate on user experience.
- **Not a silicon company.** We do not sell hardware. We provide the reference design and BSP integration, but the OEM's Tier-1 manufactures the ECU. Or the OEM does, if they have in-house hardware capability.
- **Not a one-time project.** We are not building a body controller for one OEM program and walking away. We are building a platform that generates recurring revenue across programs and OEMs.

---

## 2. Why OEMs Need This

### 2.1 The OEM's Problem

Body controllers are unsexy but ubiquitous — every vehicle has one. The typical OEM development cycle for a body controller program:

| Activity | Duration | Cost driver |
|---|---|---|
| Feature requirements gathering | 6 months | Program management, systems engineering |
| Body controller software development (C on AUTOSAR) | 18-24 months | 10-15 AUTOSAR developers @ $150-200K/yr fully loaded |
| ASIL-B safety validation | 6-12 months | Functional safety engineers, ISO 26262 work products |
| Integration and vehicle testing | 12 months | Test benches, prototype vehicles, homologation |
| **Total for one program** | **3-4 years** | **$5-10M software development cost** |

And then the OEM does this **again** for the next vehicle program, because the software is too tightly coupled to the hardware and the vehicle-specific requirements to reuse.

### 2.2 What We Change

| OEM pain point | Platform solution |
|---|---|
| 18-24 month feature development cycle | Feature business logic is delivered, validated, and configurable. OEM integration takes 6-9 months, not 24. |
| 10-15 AUTOSAR developers per program | OEM needs 3-5 engineers for integration, calibration, and vehicle-specific adaptation. Our platform team handles the core. |
| ASIL-B validation from scratch every time | Safety Monitor and arbiters are validated once. OEM inherits the safety case and extends it for vehicle-specific concerns. |
| Software locked to one SoC vendor | SignalBus portability means the OEM can switch from NXP to Qualcomm to Renesas without rewriting features. |
| No diagnostic modernization path | SOVD gateway provides cloud diagnostics and modern dealer tooling out of the box. |
| Each program is a greenfield build | Configuration system means the same platform serves sedan, SUV, coupe, and truck with calibration changes, not redesign. |

### 2.3 What OEMs Actually Expect From a Platform Provider

Based on real OEM procurement patterns for non-differentiating domains:

1. **Proven platform, shared across OEMs.** OEMs are comfortable with a platform that also serves their competitors — this validates the technology and spreads development cost. They expect platform-level features to be common, with OEM differentiation through configuration and optional feature modules.

2. **Separate or unified domain coverage.** Some OEMs want body, chassis, and connectivity as separate platforms from separate suppliers. Others want a single supplier providing a unified body-chassis-connectivity stack. We need to decide where we stand (see section 4.2).

3. **Platform provider keeps it current.** The OEM expects us to maintain the platform for the full production + support lifecycle — typically 10 years after the last vehicle rolls off the line for that program. This means security patches, regulatory compliance updates (new UN ECE regulations), SoC BSP updates, and AUTOSAR version upgrades. This is not optional — it is a contractual expectation.

4. **Platform provider absorbs integration complexity.** The OEM does not want to manage Yocto builds, AUTOSAR Classic toolchain licenses, or SoC BSP updates. They license the platform and expect it to work. Our team (or our subcontracted engineering) handles the BSP, AUTOSAR integration, safety certification updates, and tool chain.

5. **Platform provider shares liability.** The OEM will negotiate liability clauses covering defects in the platform software that reach production vehicles. This is similar to how a Build-To-Print supplier takes liability for manufacturing defects. We will need product liability insurance and rigorous quality processes.

---

## 3. How We Make Money

### 3.1 Revenue Model: Platform Licensing

The business model is **per-program licensing with ongoing maintenance fees**, not FTE staffing.

| Revenue stream | When | Structure | Why it works |
|---|---|---|---|
| **Platform license fee** | At program kick-off (SOP - 24 months) | One-time fee per vehicle program. Priced by feature scope (body-only vs. body+connectivity) and production volume tier. | OEM pays for access to the platform and the right to ship it on their vehicles. This is our upfront development cost recovery. |
| **Annual maintenance fee** | SOP through SOP + 10 years (end of field support) | Annual fee covering security patches, regulatory updates, BSP updates, AUTOSAR version upgrades, and platform enhancement releases. | Recurring, predictable revenue. Aligns with OEM expectation that we keep the platform current. |
| **Per-unit royalty** (optional) | On each vehicle produced | Low per-unit fee ($2-10/vehicle depending on feature scope). May be blended into the license fee for simplicity. | Aligns our revenue with OEM production success. OEMs may prefer a higher upfront license with no per-unit royalty, or vice versa. |
| **Integration services** | During OEM program development (SOP - 24 to SOP - 6 months) | Time and materials or fixed-price for vehicle-specific adaptation, calibration, OEM-specific feature modules, and vehicle integration testing. | Bridges the gap between platform delivery and vehicle launch. Uses our platform expertise, not generic staffing. |
| **Custom feature development** | As needed | Fixed-price per feature module. OEM-specific features built to platform architecture standards, reusable across that OEM's programs. | OEM gets features built correctly the first time, to platform standards. We may (with OEM agreement) generalize custom features into the platform for other customers. |

### 3.2 Why Licensing, Not Staffing

| Staffing model | Licensing model |
|---|---|
| Revenue = headcount × billing rate. Linear. | Revenue = programs × license fee + maintenance annuity. Scalable. |
| OEM owns the IP, we walk away after the contract. | We own the platform IP, OEM licenses usage rights. |
| No recurring revenue after project ends. | Maintenance fees continue for 10+ years per program. |
| Each engagement is a new build, limited reuse. | Each engagement configures and extends the same platform. |
| OEM's risk: we leave, they maintain unfamiliar code. | OEM's benefit: we maintain and evolve the platform continuously. |
| Our risk: OEM cuts headcount, revenue drops to zero. | Our benefit: diversified across multiple OEM programs. |
| Margins: 20-30% (typical services). | Margins: 50-70% on maintenance; 30-40% on licenses (after platform matures). |

### 3.3 Revenue Build-Up (Illustrative)

| Year | Programs | Revenue type | Illustrative annual revenue |
|---|---|---|---|
| Year 1 | 0 live, 2 in development | Integration services only | $2-4M |
| Year 2 | 0 live, 3 in development | License fees + integration | $5-8M |
| Year 3 | 2 at SOP, 2 in development | License + maintenance + integration | $8-12M |
| Year 5 | 5 at SOP, 3 in development | License + maintenance stack | $15-25M |
| Year 8 | 8+ at SOP, 4 in development | Maintenance annuity dominates | $25-40M |

The critical insight: **maintenance revenue compounds**. Every program that reaches SOP adds to the annuity base, and that revenue continues for 10+ years. By year 8, the maintenance stack from earlier programs provides baseline revenue that funds ongoing platform development.

---

## 4. Risks and How We Mitigate Them

### 4.1 Technical Risks

| Risk | Severity | Mitigation |
|---|---|---|
| **ASIL-B Safety Monitor certification cost** | High | Single most expensive development item. Budget $1-2M for initial ISO 26262 work products and certification. Amortized across all OEM programs. Engage a functional safety consultancy (e.g., exida, TUV) early. |
| **AUTOSAR Classic licensing cost** | Medium | We pay per-seat licenses for Vector/EB tools. These are our COGS, not the OEM's. Factor into license pricing. As our team stabilizes, seat count stays flat while program count grows — fixed cost, scaling revenue. |
| **SoC vendor lock-in** | Low | SignalBus trait portability is already proven (MockBus in CI, RPmsg for NXP planned, GLINK for Qualcomm planned). Architecture guarantees portability. |
| **Rust maturity in automotive** | Medium | Rust is gaining automotive traction (Volvo, Volkswagen CARIAD, Ferrocene safety-qualified compiler). Our use is in the QM application layer — not safety-critical. The M7 remains C/AUTOSAR. The mixed approach is pragmatic, not radical. |
| **Platform scope creep** | Medium | Clear boundary: body + connectivity. No ADAS. No IVI. Resist OEM requests to add chassis or powertrain unless we deliberately expand scope (see 4.2). |

### 4.2 Strategic Risk: Scope — Body Only vs. Body + Chassis + Connectivity

This is a decision we must make before signing our first OEM program:

**Option A: Body domain only.** Smallest investment, fastest to market, clearest boundary. Risk: OEMs wanting a unified platform choose a competitor who offers body + chassis + connectivity.

**Option B: Body + connectivity.** Natural extension — BLE digital key, NFC, phone-as-key, cloud-connected lock/unlock are already in our feature set. Connectivity lock sources are body features that happen to have a cloud interface. Low incremental investment.

**Option C: Body + connectivity + chassis (window, sunroof, seat adjustment, wiper).** Larger scope, more features, more compelling platform. Risk: chassis features (wiper control, window anti-pinch) have ASIL-B requirements that expand the Safety Monitor scope.

**Recommendation:** Start with **Option B** (body + connectivity). It is the natural scope of our current architecture and feature set. Evaluate chassis expansion after the first OEM program validates the business model.

### 4.3 Business Risks

| Risk | Severity | Mitigation |
|---|---|---|
| **First OEM is hardest to sign** | Critical | No production track record yet. Mitigation: offer favorable terms on the first program (reduced license fee, co-development model) in exchange for a reference customer. Budget the first program as a loss-leader that validates the platform. |
| **Product liability** | High | Platform defects in production vehicles expose us to liability claims. Mitigation: product liability insurance (automotive-grade), rigorous quality processes (A-SPICE CL2 minimum), clear contractual boundaries on what we warrant vs. what the OEM's integration introduces. |
| **OEM demands source code escrow or ownership** | Medium | OEMs may require source code escrow (protection if we go out of business) or full source access. Mitigation: escrow is standard and acceptable. Full source ownership transfer is not — it destroys the licensing model. License grants OEM usage rights, not IP ownership. |
| **Long sales cycle** | High | Automotive programs have 24-36 month development cycles. First revenue may be 12-18 months after first OEM contact. Mitigation: budget 18-24 months of operating expense before first meaningful revenue. Integration services during development provide bridging revenue. |
| **Maintenance obligation exceeds team capacity** | Medium | As programs accumulate, maintenance burden grows (security patches, BSP updates for 5+ SoC variants, AUTOSAR version upgrades). Mitigation: invest in CI/CD infrastructure that automates build, test, and release across all supported configurations. Platform architecture (SignalBus abstraction, container isolation) limits the blast radius of changes. |
| **OEM replaces us with in-house team** | Low-Medium | After learning from our platform, OEM may attempt to bring body controller development in-house. Mitigation: the maintenance annuity creates switching cost. Replacing us means the OEM takes on BSP updates, safety re-certification, and AUTOSAR toolchain management. The platform must be good enough that replacing it is more expensive than renewing the license. |

### 4.4 Financial Risks

| Risk | Severity | Mitigation |
|---|---|---|
| **High upfront investment before revenue** | Critical | Platform development (Rust application layer, Safety Monitor, BSP integration, certification) requires $3-5M before the first license fee. Mitigation: staged investment — build the application layer first (lower cost, demonstrates value in demos), defer Safety Monitor certification until first OEM is signed. |
| **AUTOSAR Classic toolchain cost** | Medium | Vector/EB toolchain licenses: $50-100K/yr per seat, typically need 3-5 seats. This is a fixed cost regardless of program count. Mitigation: factor into platform pricing. As program count grows, this cost becomes a smaller percentage of revenue. |
| **Per-unit royalty collection** | Low | If we include per-unit royalties, we depend on OEM accurately reporting production volumes. Mitigation: use fixed license + maintenance model as the primary structure. Per-unit royalty as an optional add-on for OEMs who prefer lower upfront cost. |

---

## 5. Why the Rewards Justify the Investment

### 5.1 The Compounding Annuity

The single most important financial characteristic of this business is that **maintenance revenue stacks**. Each OEM program that reaches SOP adds a maintenance annuity that runs for 10+ years. Unlike project-based services where revenue resets to zero after delivery, our revenue base grows with each program and persists for a decade.

By year 5, if we have 5 programs at SOP, we are collecting 5 simultaneous maintenance streams. By year 8, we may have 8+ streams. The early programs are still generating revenue while new programs add to the stack.

### 5.2 Development Cost Leverage

The platform is developed once and configured per program. The marginal cost of the second OEM program is dramatically lower than the first:

| Cost category | First OEM program | Second OEM program | Fifth OEM program |
|---|---|---|---|
| Platform development | $3-5M (full build) | $0 (already exists) | $0 |
| Safety Monitor certification | $1-2M (initial) | $200-400K (delta cert for new SoC/config) | $100-200K |
| Vehicle-specific integration | $500K-1M | $500K-1M | $300-500K (team gets faster) |
| BSP adaptation (if new SoC) | $300-500K | $0 (if same SoC) or $300-500K (new SoC) | Likely $0 |
| **Total** | **$5-8M** | **$700K-1.5M** | **$400K-700K** |

The gross margin on the second program is 70-80%. On the fifth, it is 80-90%. This is the software product economics that management should focus on — it is fundamentally different from staffing economics.

### 5.3 Competitive Moat

Once an OEM integrates the platform into a vehicle program, switching to a competitor or in-house solution requires:

1. Re-developing the Safety Monitor (or re-certifying someone else's)
2. Re-validating all feature business logic against the vehicle
3. Re-integrating the BSP and AUTOSAR Classic layer
4. Re-building the diagnostic gateway (SOVD, UDS proxy)
5. Re-training their team on a different architecture

This is a 12-18 month effort costing $2-5M. As long as our maintenance fee is significantly less than the switching cost, the OEM will renew. The maintenance annuity is protected by the switching cost.

### 5.4 Market Timing

Body controllers are in transition:

- OEMs moving from dedicated body ECUs to zonal/central compute, but the software is the same problem regardless of where it runs
- AUTOSAR Classic application layer is being replaced by Linux + services (exactly our architecture)
- SOVD (diagnostic modernization) is becoming a procurement checkbox
- Digital key (BLE, NFC, UWB) is driving new lock source features that traditional body controller suppliers are slow to deliver
- OEMs want faster iteration cycles — C on AUTOSAR Classic takes 2-3 years per feature set, Rust on Linux takes months

We are building the right product at the right time. The question is whether we move fast enough to capture the window before larger players (Continental, Aptiv, Bosch) build their own SDV body platforms.

---

## 6. What We Need From Management

| Need | Why | Investment |
|---|---|---|
| **18-24 months of runway** | Automotive sales cycles are long. First revenue will not arrive for 12-18 months after first OEM contact. | Operating budget for platform team (6-10 engineers) + AUTOSAR toolchain licenses + safety certification engagement. |
| **Commitment to the licensing model** | Staffing is comfortable but linear. Licensing requires patience but compounds. We cannot do both — mixed signals to OEMs destroy credibility. | Decision to price and sell as a platform product, not an engineering services engagement. |
| **Product liability insurance** | OEMs will require it before signing. Non-negotiable. | Automotive-grade product liability policy. Cost depends on coverage and program count. |
| **A reference customer strategy** | The first OEM is the hardest. We may need to offer favorable terms (reduced license, co-development) to get the first production reference. | Willingness to invest in the first program at reduced margin in exchange for a reference that sells the second and third. |
| **Sales capability** | Selling a platform to OEM management is not the same as selling engineering hours. Requires product positioning, ROI analysis, and executive-level relationships. | One senior automotive business development hire or partnership with an existing Tier-1 that needs this capability (white-label or co-sell arrangement). |

---

## 7. Summary

**What we are building**: A licensable software platform for automotive body controllers, covering feature business logic, safety arbitration, diagnostics, and the full signal path from physical switches to vehicle network.

**How we make money**: Per-program license fees + annual maintenance annuity (10+ years per program) + integration services. Not FTE staffing.

**Why it works**: Maintenance revenue compounds across programs. Development cost leverage means 70-90% gross margin on programs after the first. Switching cost protects the annuity.

**What it costs**: $5-8M for the first OEM program (platform build + certification + integration). $700K-1.5M marginal cost for each additional program.

**What the risks are**: Long sales cycle, product liability, first-customer acquisition, maintenance obligation scaling. All manageable with proper planning and capitalization.

**Why now**: Body controllers are in SDV transition. OEMs need this platform and the window is open. Larger players are still figuring out their SDV body strategy. We can be first to market with a production-grade, licensable body SDV platform.
