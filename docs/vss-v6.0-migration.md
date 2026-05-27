# VSS v6.0 migration — design + path mapping

Companion to backlog item **22**.  This doc is the deliverable of
the first stage of the v4.0 → v6.0 refresh: catalog vendoring,
gap analysis, and the architectural recommendations that gate the
follow-up code-change PRs.  **No source files outside `docs/` and
`vss-bridge/specs/` are touched in the PR that introduces this
document.**

## TL;DR

- We've been pinned at VSS v4.0 (May 2023); latest is **v6.0**
  (Jan 2026).
- Our bridge today uses **project-local path roots** (`Body.*`,
  `Cabin.*`, `Chassis.*`) — not VSS-canonical, never was.  The
  scattered "VSS v4.0" doc comments refer to signal *semantics*
  (enum values, units, names) not path-namespace conformance.
- Of our 265 paths in `signal_ids.rs`, **47 (18 %) have a canonical
  v6.0 equivalent** (8 already-canonical + 39 with a structural
  rename).  **218 paths (82 %)** are project-specific (arbiter
  intents, simulator plant state, HMI diagnostic toggles, body-
  feature aggregates) with no VSS equivalent.
- The single biggest semantic difference: VSS uses
  **`DriverSide` / `PassengerSide`** instead of `Left` / `Right`,
  baking RHD/LHD awareness into the schema.  Adopting that
  eliminates our `dealer.driver_door_side` cal — a desirable
  outcome but a **large architectural refactor** touching every
  feature that reasons about door sides.
- This document proposes:
  1. A two-tier namespace — canonical `Vehicle.*` for everything
     that maps, plus a project-specific `Vehicle.Body.Bridge.*`
     subtree for our extensions.
  2. Adoption of `DriverSide`/`PassengerSide` semantics and
     elimination of `dealer.driver_door_side`.
  3. A staged migration plan delivered as ~6 follow-up PRs.

The complete mapping table lives in
[`docs/vss-v6.0-path-mapping.csv`](./vss-v6.0-path-mapping.csv).

---

## Why we're not currently VSS-conformant

The bridge predates a serious VSS-conformance push.  When the
project started, COVESA VSS was used as a vocabulary reference
(signal names, units, enum values) but not as a strict path
authority.  Two reasons that drift happened:

1. **Brevity.**  `Body.Doors.Row1.Left.IsLocked` is shorter than
   `Vehicle.Cabin.Door.Row1.DriverSide.IsLocked` and easier to
   read in feature code, comments, and Gherkin scenarios.
2. **Project-specific signals don't have a VSS home.**  Roughly
   80 % of our paths are arbiter commands (`Body.Doors.CentralLock.Command`),
   simulator plant state (`Body.PEPS.Plant.KeyFob.1.PlacedZone`),
   or HMI diagnostic toggles (`Body.Connectivity.RemoteLock`).
   VSS doesn't model any of these — they're our extensions.
   Sticking them under `Body.*` parallel to the standard ones felt
   natural at the time.

The cost of staying non-conformant becomes visible when item 19
(Kuksa.val broker) lands: the broker only knows VSS-canonical
paths, so we'd either have to (a) extend the broker's catalog
with our 218 extension signals, or (b) translate at the boundary,
or (c) rebase internally to canonical names.  This document
chooses (c).

## What v6.0 looks like

Released **2026-01-16**.  Catalog vendored at
[`vss-bridge/specs/vss-v6.0.json`](../vss-bridge/specs/vss-v6.0.json).
1,267 leaf signals across `Vehicle.{ADAS, Body, Cabin, Chassis,
Connectivity, ControlUnit, Driver, Exterior, Powertrain, Service,
Trailer, Acceleration, AngularVelocity, …}`.

Key structural choices in v6.0 (and indeed all post-v3 releases)
relevant to us:

1. **Single root.**  Everything is `Vehicle.*`.  No top-level
   `Body.*` / `Cabin.*` / `Chassis.*`.
2. **Side-aware naming.**  `Vehicle.Cabin.Door.Row1.DriverSide.*`
   instead of `Vehicle.Cabin.Door.Row1.Left.*`.  Same for mirrors
   (`Vehicle.Body.Mirrors.DriverSide.*`).  RHD vehicles see the
   same path resolving to the physically-right door.
3. **`Vehicle.Cabin.Door` is singular.**  We model
   `Body.Doors.*`; canonical is `Vehicle.Cabin.Door.*`.
4. **No central-lock command signal.**  VSS models per-door state
   (`IsLocked`) but provides no arbiter-style command channel.
   Our `Body.Doors.CentralLock.Command` is unrepresentable.
5. **No alarm-state / lock-event-num.**  Our perimeter alarm /
   double-lock tracking signals are project-specific.
6. **No PEPS plant model.**  All of `Body.PEPS.Plant.*` is
   ours — VSS exposes `Vehicle.Cabin.Driver.IdentifierType`
   and not much else around key detection.

## Migration strategy

### Two-tier namespace

| Tier | Path root | Used for |
|---|---|---|
| **Canonical** | `Vehicle.*` (per v6.0 schema) | The 47 paths that have a direct VSS equivalent.  Drives bus interop with kuksa.val. |
| **Bridge extension** | `Vehicle.Body.Bridge.*` | The 218 paths that are project-specific.  Sits under `Vehicle.Body.*` so the kuksa.val catalog can be extended with our subtree as an overlay, but namespaced so canonical consumers don't accidentally subscribe to our internals. |

Why `Vehicle.Body.Bridge.*` rather than `Vehicle.Extension.*` or
`Bridge.*`?

- **Stays under `Vehicle.*`** so kuksa.val's tree-walk consumers
  see it.
- **Names the owner** (`.Bridge`) so future engineers know which
  subsystem these signals belong to and that they're not VSS-
  promoted.
- **Easy to overlay** in a VSS overlay file — when item 19 adds
  the broker, the overlay file declares the `Vehicle.Body.Bridge.*`
  subtree explicitly so the broker accepts it.

### `DriverSide` / `PassengerSide` adoption

VSS 6.0 has no `Left` / `Right` door paths — only `DriverSide` and
`PassengerSide`.  Adopting canonical paths therefore forces us to
either:

- **A. Adopt `DriverSide`/`PassengerSide` semantics throughout the
  codebase.**  Every feature that reasons about which door is the
  driver simply uses `DriverSide`.  The `dealer.driver_door_side`
  cal is **eliminated**.  Internally everything still resolves to
  a physical Row+Side at the plant-model layer, but feature logic
  doesn't care.
- **B. Keep `Left`/`Right` internally, add a canonical-translation
  layer at the boundary.**  Bridge-internal paths stay
  `Vehicle.Body.Bridge.Body.Doors.Row1.Left.IsLocked`; we
  publish both that and the canonical
  `Vehicle.Cabin.Door.Row1.DriverSide.IsLocked` from the same
  source value, swapping side based on the cal.  Keeps the existing
  cal but doubles the path count.

**Recommendation: A.**  The architectural simplification is
significant (one fewer dealer cal, one fewer source of
RHD-vs-LHD bug surface), and it's the direction every standards-
aware platform is moving.  Implementation cost is high but
one-time.

### Aliased rename pattern

For paths that change name:

1. **Migration PR**: bus publishes both old and new paths, both
   resolve to the same internal storage and signal ID.  All
   feature code starts using the new path.  Tests are migrated
   one feature at a time.
2. **Cleanup PR**: once every consumer (including HMI) is on the
   new path, remove the old.  Old paths in `signal_ids.rs` get
   deleted; bus stops accepting them.

The alias window is short — single sprint — so consumers don't
get stuck on the old paths.

## Path-mapping summary

From the full table in [`vss-v6.0-path-mapping.csv`](./vss-v6.0-path-mapping.csv):

| Category | Count | Example |
|---|---:|---|
| `canonical-as-is` | 8 | `Vehicle.LowVoltageSystemState` |
| `canonical-with-vehicle-prefix` | 23 | `Body.Hood.IsOpen` → `Vehicle.Body.Hood.IsOpen` |
| `canonical-door-side-rename` | 16 | `Body.Doors.Row1.Left.IsLocked` → `Vehicle.Cabin.Door.Row1.DriverSide.IsLocked` |
| `canonical-restructured` | 23 | `Body.Trunk.IsOpen` → `Vehicle.Body.Trunk.Rear.IsOpen`; `Cabin.HVAC.Station.Row1.Left.FanSpeed` → `Vehicle.Cabin.HVAC.Station.Row1.Driver.FanSpeed` |
| `project-extension-mirroring` | 34 | `Body.Hood.OpenCmd` → `Vehicle.Body.Bridge.Body.Hood.OpenCmd` (VSS has hood Position/Switch state but no command channel) |
| `project-door-side-extension` | 38 | `Body.Doors.Row1.Left.Handle.Outside.IsPulled` → `Vehicle.Cabin.Door.Row1.DriverSide.Handle.Outside.IsPulled` |
| `project-namespace` | 123 | `Body.Doors.CentralLock.Command` → `Vehicle.Body.Bridge.Body.Doors.CentralLock.Command` |
| **Total** | **265** | |

**No remaining `fallback` rows** — sub-PR 2 closed out the long tail
by walking each manually, looking up the v6.0 catalog, and either
finding a restructured canonical (e.g. Trunk.Rear, HVAC Driver vs
Left, Sunroof moved to Cabin) or assigning a project-extension or
project-namespace path with a written justification.

## Category definitions

- **`canonical-as-is`** — path already matches v6.0 exactly, no
  rename needed.  Migration is a no-op for the path string.
- **`canonical-with-vehicle-prefix`** — just prepend `Vehicle.`.
- **`canonical-door-side-rename`** — same canonical except
  `Left`/`Right` → `DriverSide`/`PassengerSide`.
- **`canonical-restructured`** — VSS has the same signal but at a
  different shape: trunk split into `Front`/`Rear`, HVAC station
  side names `Driver`/`Passenger` (not `Left`/`Right`), sunroof
  moved from `Body.*` to `Cabin.*`, mirror taxonomy `Mirrors`
  (plural) with side enum.  These are all canonical reads; the
  rename surface is wider but the destination is real.
- **`project-extension-mirroring`** — the canonical parent path
  exists but our specific leaf doesn't (commonly: VSS exposes
  state, we add a command channel; VSS has one signal per system,
  we add per-lamp / per-position detail).  Goes under
  `Vehicle.Body.Bridge.*` mirroring the canonical hierarchy so
  the relationship is visible.
- **`project-door-side-extension`** — same as above but
  specifically for door-side extensions: VSS doesn't subdivide
  the door handle taxonomy as deeply as we do (LockPad, individual
  handle pull edges), so those go under
  `Vehicle.Cabin.Door.RowN.DriverSide.*` with our custom suffix.
- **`project-namespace`** — fully project-specific concept with
  no canonical parent (arbiter commands, simulator plant state,
  alarm FSM strings, valet mode, immobiliser status, etc.).

## Staged sub-PR plan

Each sub-PR is independently shippable.  All under
`feature/vss-v6.0/<sub-tag>`.

### Sub-PR 1 — Vendor spec + mapping artifact + this design doc

**Status:** this PR.

Vendors `vss-bridge/specs/vss-v6.0.json`, produces
`docs/vss-v6.0-path-mapping.csv` and this design document.  No
code change.  Updates backlog item 22 to reference the design.

### Sub-PR 2 — Manual review of the 71 fallback paths *(done)*

All 71 fallback rows walked manually and re-categorised against
the v6.0 catalog.  Findings:

- 23 had a real canonical home that wasn't a straight rename —
  trunk Front/Rear split, HVAC Driver/Passenger station naming,
  sunroof moving from `Body.*` to `Cabin.*`, mirror `Mirrors`
  (plural) + side enum.  These are now in `canonical-restructured`.
- 34 had a canonical parent path but no leaf at the level we
  need (commonly: VSS exposes state, we model command intents on
  top; VSS aggregates a system signal, we expose per-lamp
  detail).  These are now in `project-extension-mirroring` and
  go under `Vehicle.Body.Bridge.*` mirroring the canonical
  hierarchy.
- 14 were genuinely project-specific (alarm FSM, valet mode,
  immobiliser status, AutoHighBeam oncoming-vehicle input,
  crash-detected, etc.).  These joined `project-namespace`.

Output: updated `vss-v6.0-path-mapping.csv` with zero remaining
`fallback` rows.  Doc-only.

### Sub-PR 3 — Path-canonicalisation layer in `SignalBus`

Add a thin canonicalisation layer so `bus.publish(legacy_path,
…)` and `bus.publish(canonical_path, …)` resolve to the same
broadcast channel for the alias window.  Implementation: a
static table loaded at startup, indexed by both directions; the
bus does the lookup on every publish / subscribe.  Tests verify
the round-trip on every CSV row.

### Sub-PR 4 — Adopt `DriverSide` / `PassengerSide` in feature code

The big one.  Refactor every feature that today reasons about
`Left` / `Right` to use `DriverSide` / `PassengerSide`.  Delete
the `dealer.driver_door_side` cal.  The plant-model layer
internally maps `DriverSide` → physical Row1.Left or Row1.Right
based on a single build-time / boot-time vehicle orientation
constant.  ~6–10 features touched.

### Sub-PR 5 — Migrate canonical paths in `signal_ids.rs` + features

Convert every `canonical-*` row's old path to the new one in
feature code.  Bus still serves both via the alias layer.  Tests
on the new paths land in this PR.

### Sub-PR 6 — Migrate project-namespace paths

Convert every `project-namespace*` row.  Same pattern.  Largest
volume change by line count, but mechanical.

### Sub-PR 7 — Remove alias layer, retire legacy paths

After every internal consumer has moved, drop the alias layer
from `SignalBus`, delete old path arms from `signal_ids.rs`,
fail-fast if any caller still uses a legacy path.  HMI manifest
regenerated.

### Sub-PR 8 — Adopt new VSS v6.0 signals worth exposing

Out of scope for the rebase; covered separately in backlog item
22 step 5.  Likely candidates: expanded child-lock signals,
cabin lighting taxonomy, Driver.IdentifierType for PEPS
integration.

## Doc-comment update

All "VSS v4.0" comments in `signal_ids.rs`, feature files, and
plant models become "VSS v6.0" in sub-PR 5.  Where we explicitly
extend the spec, the comment notes the extension subtree
(`Vehicle.Body.Bridge.*`) to make it clear the path is project-
local.

## Open architectural questions

1. **`Cabin.LockStatus` aggregate.**  VSS models per-door
   `IsLocked`.  Our aggregate (`LOCKED` / `UNLOCKED` /
   `DRIVER_UNLOCKED` / `DOUBLE_LOCKED`) is observably useful but
   has no canonical home.  Keep as `Vehicle.Body.Bridge.*` or
   propose to COVESA upstream?
2. **PEPS plant signals.**  500 ms of every test run touches
   these; they're simulator-only and never reach a real vehicle.
   Should they even be on the bus in a production build, or get
   gated behind a `cfg(feature = "simulator")`?
3. **Project `dealer.*` and HMI `Vehicle.Cabin.Infotainment.HMI.*`
   paths.**  Already canonical under `Vehicle.Cabin.Infotainment.*`.
   Keep canonical placement.

## How this lands

This sub-PR introduces:

- `vss-bridge/specs/vss-v6.0.json` (321 KB, vendored release artifact).
- `docs/vss-v6.0-migration.md` (this file).
- `docs/vss-v6.0-path-mapping.csv` (full 265-row mapping table).
- Updates to backlog item 22 in `docs/post-peps-backlog.md` to
  reference this design.

No code under `vss-bridge/src/` is touched.  Sub-PR 2 is the
immediate follow-up.
