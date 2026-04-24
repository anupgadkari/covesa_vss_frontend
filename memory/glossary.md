# Glossary

Full decoder ring — all terms, acronyms, and internal language for the SDV body platform context.

## Acronyms
| Term | Meaning | Context |
|------|---------|---------|
| SDV | Software-Defined Vehicle | The platform category being sold |
| VSS | Vehicle Signal Specification | COVESA standard for vehicle signal naming |
| OEM | Original Equipment Manufacturer | The customer |
| HMI | Human-Machine Interface | Optional tooling — NOT a core body controller benefit |
| BSW | Base Software | Classic AUTOSAR base software layer |
| SWC | Software Component | AUTOSAR software component |
| PEPS | Passive Entry Passive Start | Body feature — keyless entry/start |
| ASIL | Automotive Safety Integrity Level | Functional safety classification (A/B/C/D) |
| ASIL-B | ASIL Level B | Mid-level functional safety — OEM can own this app layer |
| QM | Quality Managed | Below ASIL — the Rust feature-logic layer OEM typically owns |
| AUTOSAR | AUTomotive Open System ARchitecture | The automotive software standard |
| PCB | Printed Circuit Board | Hardware layer, Platform Provider scope |
| ECU | Electronic Control Unit | Vehicle compute node |
| BCM | Body Control Module | The body controller ECU |
| CAN | Controller Area Network | Vehicle bus protocol |
| SoM | System on Module | Embedded compute module |
| COVESA | Connected Vehicle Systems Alliance | Standards body (VSS, etc.) |

## Architecture Terminology (Enforced)
| Say This | Not This | Why |
|----------|----------|-----|
| Feature Business Logic | Feature FSMs | Features may be FSMs or complex algorithms |
| Platform Provider | Tier-1 | Partnership model, not just a component supplier |
| Optional tooling / diagnostic tool | Core HMI benefit | Body features run without HMI |

## Ownership Model
| Layer | Owner |
|-------|-------|
| Classic AUTOSAR BSW, Linux distro, schematics, PCB | Platform Provider |
| Classic AUTOSAR Application Layer SWCs (M7, ASIL-B) | Platform Provider *or* OEM (OEM can extend here) |
| QM Rust feature-logic layer | OEM (typical) |
| Safety Monitor | Not exclusively Platform Provider scope |

## Body Features (run autonomously, no HMI required)
- Lighting control
- Door locks
- PEPS (Passive Entry Passive Start)
- Wipers
- Mirrors
- Seat / window / sunroof actuation
