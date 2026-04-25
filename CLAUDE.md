# Memory

## Me
Anup Gadkari — works at a **service company** selling the SDV body controller platform to OEM management. Architecture docs and proposals are sales/consulting deliverables, not internal OEM budget requests.

## Terms
| Term | Meaning |
|------|---------|
| **SDV** | Software-Defined Vehicle |
| **VSS** | Vehicle Signal Specification (COVESA standard) |
| **Feature Business Logic** | The correct term for feature logic — may be FSMs or complex algorithms, never just "Feature FSMs" |
| **Platform Provider** | Correct term for the supplier (provides Classic AUTOSAR BSW, Linux distro, schematics, PCB) — never "Tier-1" |
| **ASIL-B** | Automotive Safety Integrity Level B — OEM *can* own this layer (not exclusively Platform Provider scope) |
| **QM** | Quality Managed — the Rust feature-logic layer the OEM typically owns |
| **HMI** | Optional diagnostic/engineering tooling — NOT a core body controller benefit |
| **BSW** | Base Software (Classic AUTOSAR) |
| **SWC** | Software Component (AUTOSAR) |
| **PEPS** | Passive Entry Passive Start |
| **OEM** | Original Equipment Manufacturer — the customer |
→ Full glossary: memory/glossary.md

## Architecture Rules (apply to all docs & proposals)
1. Say **"Feature Business Logic"**, never "Feature FSMs"
2. Say **"Platform Provider"**, never "Tier-1"
3. **HMI is optional tooling** — never list as a top-level architecture benefit; frame as bench validation / factory diagnostics / engineering visualization
4. **OEM can own ASIL-B app layer** — acknowledge OEM can extend into Classic AUTOSAR Application Layer SWCs on M7, not just QM Rust layer
5. Pain point to emphasize: Tier-1 body supplier dependency (not head unit supplier)

## Projects
| Name | What |
|------|------|
| **covesa-vss-frontend** | Frontend visualization/tooling for COVESA VSS signals |
→ Details: memory/projects/

## Preferences
- Proposals are aimed at OEM senior management — keep language accessible, business-focused
- Partnership framing over adversarial supplier-replacement narrative
- Body features (lighting, locks, PEPS, wipers, mirrors) run autonomously — no head unit dependency
