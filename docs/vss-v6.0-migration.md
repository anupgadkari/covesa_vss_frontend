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
  1. A three-namespace split — canonical `Vehicle.*` for everything
     that maps to VSS, **`Vehicle.Controller.*`** for body-
     controller-owned signals (arbiter commands, FSM state,
     processed inputs, derived views), and **`Vehicle.Simulation.*`**
     for PEPS plant / HMI affordances that don't exist on real
     vehicles.  (Initial draft proposed a single `Vehicle.Body.Bridge.*`
     catchall; that was discarded because "Bridge" named the
     software artifact rather than a vehicle subsystem, and
     because simulator signals deserve to be obviously simulator-
     only.)
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

### Three-namespace split

Our 265 paths land in three root namespaces:

| Tier | Path root | Used for | ~Count |
|---|---|---|---:|
| **Canonical** | `Vehicle.*` (per v6.0 schema) | Paths that map directly to VSS, including restructured variants and `DriverSide`/`PassengerSide` renames | 70 |
| **Door-extension under canonical** | `Vehicle.Cabin.Door.RowN.{Driver,Passenger}Side.*` | Project-specific door-handle / lockpad signals that extend the canonical Door schema (VSS has the parent path, we add leaves) | 38 |
| **Controller** | `Vehicle.Controller.*` | Body-controller-owned signals: arbiter command channels, FSM state, processed user-input edges, derived sensor views, per-feature outputs.  These are exactly what an OEM body engineer would call "body controller signals." | 100 |
| **Simulation** | `Vehicle.Simulation.*` | Signals that exist *only* because we model the LF / connectivity subsystems in software, plus HMI affordances that simulate external command inputs.  Gated behind `cfg(feature = "simulator")` for production builds. | 57 |

**Earlier draft of this document proposed a single
`Vehicle.Body.Bridge.*` catchall.**  That was rejected in favour
of the three-namespace split for three reasons:

1. **`Bridge` is a software-artifact name.**  The `vss-bridge`
   binary is the executable; "Bridge" describes our code, not a
   vehicle subsystem.  VSS is supposed to model the vehicle, not
   the codebase that talks to it.  `Vehicle.Controller.*` names
   a vehicle architecture concept — the body controller — which
   any OEM body engineer recognises immediately.
2. **Simulator signals should be obviously simulator-only.**
   `Vehicle.Body.Bridge.Body.PEPS.Plant.KeyFob.1.PlacedZone` is
   indistinguishable from a real controller output by namespace
   alone.  Tagging it `Vehicle.Simulation.PEPS.Plant.…` makes
   the simulator-only status visible at a glance, and the kuksa.val
   catalog for production builds simply omits the
   `Vehicle.Simulation.*` subtree.
3. **HMI affordances ≠ controller outputs.**  The four
   `Body.Connectivity.*` HMI toggles (`RemoteLock`, `BleLock`,
   `NfcCardPresent`, `NfcPhonePresent`) simulate inputs that on a
   real vehicle would arrive over CAN from the telematics / NFC
   modules.  They're not controller-produced data.  Putting them
   under `Vehicle.Simulation.Connectivity.*` correctly reflects
   that.

### What goes where, by signal family

**`Vehicle.Controller.*`** (100 paths):

- Arbiter command channels: `CentralLock.Command`,
  `Trunk.OpenCmd`/`CloseCmd`, `Hood.OpenCmd`/`CloseCmd`,
  `Mirror.{Driver,Passenger}Side.AdjustCmd` / `FoldCmd`,
  `Sunroof.MoveCmd`, `Sunroof.Shade.MoveCmd`.
- FSM state aggregates: `Cabin.LockStatus` (+ `.LastRequestor` +
  `.EventNum`), `Alarm.State` / `.IsActive`,
  `Starting.ImmobilizerStatus`, `Cabin.ValetMode.IsActive`,
  `AutoRelock.IsArmed`, `Power.DelayedAccessory.IsActive`,
  `PowerChildLock.MasterStatus`.
- Per-lamp diagnostics: `DirectionIndicator.*.Lamp.{Front,Side,Rear}.{IsOn,IsDefect}` (12 paths).
- Per-feature outputs: `Puddle.{Driver,Passenger}Side.IsOn`,
  `Chime.IsActive` / `.IsSounding`, `Windshield.{Front,Rear}.Washing.IsActive`,
  `Seat.Row1.{Driver,Passenger}Side.IsHeatingOn` / `.IsVentilationOn`.
- Processed user-input edges: trim switch lock buttons, lock-pad
  presses, exterior trunk button, dome rotary, hood switch, etc.
- Derived sensor views: `Chassis.Brake.IsApplied` (boolean
  computed by the controller from VSS's `Brake.PedalPosition`),
  `Lights.AmbientLightSensor.Illuminance`.
- Controller-owned safety: `Safety.CrashDetected`.

**`Vehicle.Simulation.*`** (57 paths):

- `Simulation.PEPS.Plant.KeyFob.{1..6}.{PlacedZone, LastObservedZone, Paired, ButtonPress, ChallengeResponse, …}` — the LF antenna / fob plant model state.  Doesn't exist on a real vehicle; the LF subsystem doesn't expose "where the fob is" as a signal.
- `Simulation.Connectivity.{RemoteLock, BleLock, NfcCardPresent, NfcPhonePresent}` — HMI affordances simulating telematics / NFC-reader inputs that the controller would receive over CAN in production.
- `Simulation.TestFixtures.DoesNotExist` — deliberate non-existent path used by the signal-id-lookup error-path test.

**`Vehicle.Cabin.Door.RowN.{Driver,Passenger}Side.*`** (38 paths,
under canonical Door schema):

- Handle pull edges: `.Handle.{Inside,Outside}.IsPulled`
- Lock-pad presses: `.Handle.Outside.LockPad.IsPressed`
- Close-cmd actuator intent: `.CloseCmd`

These ride **under the canonical Door taxonomy** rather than
elsewhere because the door is a canonical concept; our extension
just adds leaves where VSS doesn't subdivide far enough.

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
  layer at the boundary.**  Internal paths stay
  `Vehicle.Controller.Body.Doors.Row1.Left.IsLocked`; we publish
  both that and the canonical
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
| `project-door-side-extension` | 38 | `Body.Doors.Row1.Left.Handle.Outside.IsPulled` → `Vehicle.Cabin.Door.Row1.DriverSide.Handle.Outside.IsPulled` (extends canonical Door schema) |
| `controller-extension` | 34 | `Body.Hood.OpenCmd` → `Vehicle.Controller.Body.Hood.OpenCmd` (controller adds command channel where VSS has Position/Switch state only) |
| `controller-namespace` | 66 | `Body.Doors.CentralLock.Command` → `Vehicle.Controller.Body.Doors.CentralLock.Command`; `Cabin.LockStatus` → `Vehicle.Controller.Cabin.LockStatus` |
| `simulation` | 57 | `Body.PEPS.Plant.KeyFob.1.PlacedZone` → `Vehicle.Simulation.PEPS.Plant.KeyFob.1.PlacedZone`; `Body.Connectivity.RemoteLock` → `Vehicle.Simulation.Connectivity.RemoteLock` |
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
- **`project-door-side-extension`** — VSS has the parent
  `Vehicle.Cabin.Door.RowN.{Driver,Passenger}Side.*` but not the
  fine-grained handle / lockpad leaves.  We extend the canonical
  schema with our suffixes — these still ride under the canonical
  Door root, not under `Vehicle.Controller.*`.
- **`controller-extension`** — VSS has a canonical parent (e.g.
  `Vehicle.Body.Hood.*`, `Vehicle.Body.Trunk.Rear.*`) but no
  command-channel or per-lamp leaf.  The body controller's
  actuator-intent and per-lamp diagnostic signals go to
  `Vehicle.Controller.<canonical-shape>` so the mirroring
  relationship is visible.
- **`controller-namespace`** — fully controller-owned concept
  with no canonical parent: `Cabin.LockStatus` aggregate, the
  arbiter command bus (`CentralLock.Command`, `FeedbackRequest`),
  alarm FSM, immobiliser, valet mode, etc.
- **`simulation`** — won't exist on a real vehicle.  PEPS plant
  model state (`Simulation.PEPS.Plant.*`), HMI affordances that
  simulate telematics / NFC inputs (`Simulation.Connectivity.*`),
  and test fixtures.  In production builds this subtree is
  gated behind `cfg(feature = "simulator")` and the kuksa.val
  catalog omits it entirely.

## Staged sub-PR plan

Each sub-PR is independently shippable.  All under
`feature/vss-v6.0/<sub-tag>`.

### Sub-PR 1 — Vendor spec + mapping artifact + this design doc

**Status:** this PR.

Vendors `vss-bridge/specs/vss-v6.0.json`, produces
`docs/vss-v6.0-path-mapping.csv` and this design document.  No
code change.  Updates backlog item 22 to reference the design.

### Sub-PR 2 — Manual review + namespace refactor *(done)*

Two activities ended up bundled into the same PR:

**(a) Walk the 71 fallback rows.**  All re-categorised against
the v6.0 catalog:

- 23 had a real canonical home that wasn't a straight rename —
  trunk Front/Rear split, HVAC Driver/Passenger station naming,
  sunroof moving from `Body.*` to `Cabin.*`, mirror `Mirrors`
  (plural) + side enum.  Now in `canonical-restructured`.
- 34 had a canonical parent path but no leaf at the level we
  need (commonly: VSS exposes state, we model command intents on
  top; VSS aggregates a system signal, we expose per-lamp
  detail).
- 14 were genuinely project-specific (alarm FSM, valet mode,
  immobiliser status, AutoHighBeam oncoming-vehicle input,
  crash-detected, etc.).

**(b) Replace `Vehicle.Body.Bridge.*` with the three-namespace
split.**  The single `Body.Bridge.*` catchall got refactored into
`Vehicle.Controller.*` (100 paths) and `Vehicle.Simulation.*`
(57 paths) — see the "Three-namespace split" section above for
the rationale.  Net: every project-* row in the CSV got its
target re-prefixed, and two new categories (`controller-extension`,
`controller-namespace`, `simulation`) replaced the three
`project-*` categories from sub-PR 1 / 2(a).

Output: updated `vss-v6.0-path-mapping.csv` with zero remaining
`fallback` rows and the three-namespace split applied.  Doc-only.

### Sub-PRs 3 + 5 + 6 + 7 — Big-bang rename *(done — collapsed)*

These four were planned as separate alias-window steps, but the
codebase is a **monorepo with no out-of-process consumers** —
features, plant models, tests, Gherkin, and the HMI all live in
the same repo and move together.  There is nothing to keep alive
during an alias window, so the aliased-rename machinery (a
canonicalisation layer in `SignalBus`, sub-PR 3) was unnecessary.

Instead, a single mechanical rename applied every CSV mapping
across the whole codebase at once:

- 257 path renames (8 were already canonical no-ops).
- 3,084 substitutions across 83 files (Rust `src` + `tests`,
  Gherkin `.feature`, HMI `.html`, the playwright spec).
- Applied via a single-pass regex (longest-first alternation,
  path-boundary guards) to avoid partial-match corruption.
- Two e2e step-definition regexes used escaped-dot path literals
  (`Body\.Switches\.…`) that the literal-dot matcher missed; fixed
  by hand.
- `Left` / `Right` naming **preserved** — the `DriverSide` /
  `PassengerSide` adoption (sub-PR 4) is deferred as a separate
  architectural change.
- "VSS v4.0" doc comments rotated to "VSS v6.0".

Result: clean build, 686 lib + 8 ws_integration + 35 e2e
scenarios green (1 pre-existing WIP skip in `hazard.feature`),
clippy + fmt clean.  No alias layer; legacy paths are simply
gone.

### Sub-PR 4 — Adopt `DriverSide` / `PassengerSide` in feature code *(deferred)*

The remaining architectural change.  Refactor every feature that
today reasons about `Left` / `Right` to use `DriverSide` /
`PassengerSide`.  Delete the `dealer.driver_door_side` cal.  The
plant-model layer internally maps `DriverSide` → physical
Row1.Left or Row1.Right based on a single build-time / boot-time
vehicle orientation constant.  ~6–10 features touched.  Kept
separate because it changes feature *logic*, not just path
strings.

### Sub-PR 8 — Adopt new VSS v6.0 signals worth exposing

Out of scope for the rebase; covered separately in backlog item
22 step 5.  Likely candidates: expanded child-lock signals,
cabin lighting taxonomy, Driver.IdentifierType for PEPS
integration.

## Doc-comment update

All "VSS v4.0" comments in `signal_ids.rs`, feature files, and
plant models become "VSS v6.0" in sub-PR 5.  Where we explicitly
extend the spec, the comment notes the extension subtree
(`Vehicle.Controller.*` or `Vehicle.Simulation.*`) to make it
clear which signals are controller-owned vs. simulator-only.

## Open architectural questions

1. **`Cabin.LockStatus` aggregate.**  VSS models per-door
   `IsLocked`.  Our aggregate (`LOCKED` / `UNLOCKED` /
   `DRIVER_UNLOCKED` / `DOUBLE_LOCKED`) is observably useful but
   has no canonical home.  Lands at
   `Vehicle.Controller.Cabin.LockStatus` for now; worth proposing
   to COVESA upstream — most OEMs need an aggregate signal.
2. **PEPS plant signals.**  500 ms of every test run touches
   these; they're simulator-only and never reach a real vehicle.
   Decision: gated behind `cfg(feature = "simulator")`, surfaced
   under `Vehicle.Simulation.PEPS.Plant.*` so production builds
   simply omit the subtree.
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
