# covesa-vss-frontend

**Repo:** covesa_vss_frontend (underscore in filesystem, hyphen in display)
**What it is:** Frontend tooling/visualization for COVESA VSS (Vehicle Signal Specification) signals

## Context
Part of the broader SDV body platform toolchain. VSS provides a standardized, hierarchical naming scheme for vehicle signals (e.g., `Vehicle.Body.Lights.Beam.Low.IsOn`). This frontend likely provides visualization, browsing, or tooling on top of the VSS signal tree.

## Relevant Standards
- **COVESA VSS** — the signal spec this frontend is built around
- Signals follow a dot-path hierarchy (Vehicle → Body/Chassis/Cabin → subsystems → signals)
