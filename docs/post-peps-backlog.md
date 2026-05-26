# Post-PEPS backlog plan

Tracker for follow-up work after the PEPS / VehicleStartingControl
push (PR #27, merged into `main`).  Each item lists the trigger,
the dependencies the current `main` already provides, the work
needed, the risks, and a rough size.

Order is *suggested* — pick by priority not by listing order.  Each
item is independently shippable as its own sub-branch / PR.

## Status

| # | Item | Size | Status |
|---|------|------|--------|
| 1 | Mirror Adjust feature | M | ✅ Done — in `main` |
| 2 | Farewell feature | M | ✅ Done — PR #18 |
| 3 | DoorOpenAssist feature | S | ✅ Done — PR #18 |
| 4 | PerimeterAlarm puddle pulse | M | ✅ Done — in `main` |
| 5 | PEPS rear-door fence | S | ✅ Resolved via cal default (`peps_rear_capacitive_handles = false`) |
| 6 | Window plant model + NVM | M | ✅ Done — `plant_models/window.rs` + `power_window` feature |
| 7 | Hood / Sunroof NVM persistence | S | ✅ Done — both plant models in `main` |
| 8 | Two-stage-disabled e2e audit | XS | ✅ Done — covered in `features/passive_entry.feature` |
| 9 | SomeIp / Glink transport | L | 🚫 Out of scope (separate track) |
| 10 | KeySearch arbiter + VehicleStartingControl | XL | ✅ Done — PR #27 |
| 11 | BTSI in TransmissionPlant | M | ✅ Done — PR #27 |
| 12 | Key-in-Ignition Inhibit (KeyCylinder mode) | S | ✅ Done — PR #27 |
| 13 | Migrate ExteriorTrunkButton → KeySearch arbiter | M | ⏳ Pending |
| 14 | Stop continuous Zone publishing; `PlacedZone` + `LastObservedZone` | L | ⏳ Pending |
| 14c | Extra scan triggers — brake-press pre-auth | S | ✅ Done |
| 14d | Lost-paired-key scan + cluster warning (formerly door-close snapshot) | S | ✅ Done |
| 15 | Smart Unlock feature (key-locked-in-trunk) | M | ⏳ Pending |
| 16 | NFC Entry feature (card/phone tap → unlock; tap at push-button → start) | M | ⏳ Pending |
| 17 | Key-lost warning chime | XS | ⏳ Pending |
| 18 | ASIL-B on FreeRTOS + Ferrocene-qualified Rust (replaces Classic AUTOSAR M7) | XXL | 📋 Program-level — see plan |
| 19 | Kuksa.val databroker integration (replace MockBus on the SignalBus seam) | L | 📋 Program-level |
| 20 | Feature-completeness + interconnection audit (requirements + Gherkin + tests) | M | 📋 Program-level |
| 21 | Run the test suite on a virtualised target (QEMU / Renode / Avocado) | L | 📋 Program-level |
| 22 | Refresh to the latest COVESA VSS release (currently pinned at v4.0) | M | 📋 Program-level |

Suggested next order: **13 → 16 → 14 → 15 → 17**.  Reasoning:
**13** is a localised migration that exercises the arbiter end-to-end
on an existing feature, cheap regression value.  **16** delivers a
visible new auth path on the existing NFC HMI plumbing.  **14** is
the biggest refactor and benefits from having two arbiter consumers
first.  **15** and **17** are small follow-ups.

---

## 13. Migrate ExteriorTrunkButton onto the KeySearch arbiter  *(M)*

**Trigger.**  Today the `exterior_trunk_button` feature still reads
`Body.PEPS.Plant.KeyFob.{N}.Zone` directly from the continuous-Zone
signal to decide whether a paired fob is at the trunk.  PR #27
introduced the `KeySearchArbiter` as the single owner of LF airtime.
Every authenticated lookup needs to migrate.

**Built on (in `main`).**  `KeySearchArbiterHandle::submit`,
`AntennaSet::TrunkOutside`, `SearchMode::Authenticated`,
`Coalescing::Disallowed`.

**Plan.**
1. Constructor takes `KeySearchArbiterHandle`.  Threaded from
   `main.rs` alongside the existing trunk arbiter handle.
2. On rising edge of `Body.Trunk.ExteriorButton.IsPressed`:
   - If `Cabin.LockStatus == UNLOCKED|DRIVER_UNLOCKED`: claim
     trunk open directly (current direct-path).
   - If LOCKED|DOUBLE_LOCKED: submit
     `AntennaSet::TrunkOutside + Authenticated` via the arbiter.
     On a non-empty `keys_found`, claim trunk open.
3. Remove direct `KeyFob.{N}.Zone` reads.
4. Tests:
   - Unlocked → trunk pops.
   - Locked + no key → no pop.
   - Locked + paired fob at Trunk → pop.
   - Locked + unpaired fob at Trunk → no pop (pairing filter).
5. PassiveEntry will follow as a separate migration (item 14a) —
   keep it on the old path for this PR.

**Risks.**  Behavioural parity — the arbiter's coalescing window
might re-use stale results.  Use `Coalescing::Disallowed` for the
security-critical button press path; allowed for any speculative
checks.

---

## 14. Stop continuous Zone publishing; introduce `PlacedZone` + `LastObservedZone`  *(L)*

**Trigger.**  The PEPS plant currently broadcasts
`Body.PEPS.Plant.KeyFob.{N}.Zone` on every HMI drag.  That worked as
a stand-in for proximity polling pre-arbiter; now it's an unprincipled
ambient signal that multiple features cache.  Phase 9 of the
KeySearch design (`docs/key-search-arbiter-and-ignition.md` §10)
calls for splitting the signal:

- `PlacedZone` — where the HMI drag put the fob.  Plant state only;
  not used directly by features.
- `LastObservedZone` — published only after an arbiter scan
  completes.  Features that need "where was the fob last time we
  looked" subscribe to this instead of the raw Zone.

**Built on (in `main`).**  All features that read `KeyFob.{N}.Zone`
today (`walk_away_lock`, `keypad_lock`, `passive_entry`, etc.)
need migration.  KeySearch arbiter is the central point that
publishes `LastObservedZone` from inside `run_scan`.

**Plan.**
1. New signal IDs:
   - `Body.PEPS.Plant.KeyFob.{N}.PlacedZone` (replaces `.Zone` as
     the HMI write target).
   - `Body.PEPS.Plant.KeyFob.{N}.LastObservedZone` (arbiter writes).
2. `key_search_arbiter::run_scan` publishes
   `LastObservedZone` for every fob in coverage after each scan.
3. PEPS plant uses `PlacedZone` internally; the existing `Zone`
   signal stays as a transitional alias.
4. Migrate each consumer one by one:
   - `walk_away_lock` — subscribes to `LastObservedZone` instead of
     `Zone`.  Triggers a fresh `AllApproach + Presence` scan via
     the arbiter before deciding "all keys away".
   - `keypad_lock` — same.
   - `passive_entry` — sub-item 14a; this is the big one.  Move
     handle-pull → arbiter Authenticated search; remove all direct
     `Zone` reads.
5. After all consumers are migrated, remove the legacy `Zone`
   publish from the plant.
6. HMI: top-view drag now writes `PlacedZone`.  Chip rendering
   reads `PlacedZone` for instant visual feedback (no waiting for
   an arbiter scan to reposition).

**Risks.**  Massive surface area — every PEPS-aware feature
touches.  Stage the migration carefully behind a feature flag if
needed.  The `passive_entry` test corpus is 1.6 kLOC and 48 tests;
schedule a full afternoon for sub-item 14a.

---

## 14c. Extra scan triggers — brake-press pre-auth  *(S)*

**Trigger.**  Today the only events that drive a fresh
authenticated scan are: a Start/Stop press (PEPS), a cylinder
rotation to a live state (KeyCylinder), and an exterior-trunk
button press.  Real OEMs additionally **pre-authenticate the
moment the driver touches the brake pedal** so the subsequent
Start/Stop press feels instantaneous — the cabin Authenticated
scan (~100 ms LF airtime) has already completed by the time the
press fires.

**Built on (in `main`).**  KeySearchArbiter, `Chassis.Brake.IsApplied`
(published by the brake plant), `KeySearchArbiterHandle::submit`
+ coalescing (so the cached result is what the Start press picks
up).

**Plan.**
1. New small handler in `VehicleStartingControl` (or a separate
   pre-auth feature if preferred): subscribe to
   `Chassis.Brake.IsApplied`.
2. On `false → true` edge AND `key_source_cfg == PEPS` AND power
   ∈ {`OFF`, `ACC`}: submit `AntennaSet::Cabin` +
   `SearchMode::Authenticated` with `Coalescing::Allowed`.
3. The Start press, if it comes within the coalesce window
   (50 ms), reuses the cached result.  Outside the window it runs
   a fresh scan as today.
4. Tests: brake edge + delayed Start press uses the pre-auth
   cache; brake edge without a follow-up press is a no-op.

**Risks.**  Low.  The arbiter already serialises scans and the
coalesce window is short; spurious brake presses cost a few ms of
simulated LF airtime each.

---

## 14d. Lost-paired-key scan + cluster warning  *(S)*

**Trigger.**  Driver gets in, closes the door, starts the car —
but somehow no paired fob is with them (left on porch, dropped
on driveway, handed to a passenger who got out, …).  The vehicle
is now running without an authenticated key on board.  We want a
cluster warning popup so the driver doesn't unknowingly drive
away.

The original 14d plan was a generic on-close snapshot that also
fed Smart Unlock (#15).  In practice Smart Unlock's trigger is
the **lock** edge (driver locks the door from outside with a fob
potentially still in the trunk) — a different event entirely —
so Smart Unlock should run its own scan when it needs one.
Conflating the two muddied both features; we narrowed 14d to the
lost-PK case and renamed the feature accordingly.

**Built on (in `main`).**  Per-door `IsOpen` signals,
`Vehicle.LowVoltageSystemState`, KeySearchArbiter with
`AntennaSet::Sequence` (runs AllApproach + TrunkInside + Cabin
in one go).

**Plan.**
1. New feature `features/lost_pk_scan.rs`: subscribe to all four
   `IsOpen` signals + `Vehicle.LowVoltageSystemState`.
2. On the any-open→all-closed edge **AND** ignition ∈
   {`ON`, `START`}: submit a `Sequence` of `AllApproach` +
   `TrunkInside` + `Cabin` Presence scans, `Coalescing::Disallowed`.
3. On zero paired keys found: publish
   `Body.PEPS.LostKeyWarning = true`.  Clears on the next scan
   that finds a key, or on ignition leaving live.
4. HMI cluster subscribes to `LostKeyWarning` and renders a
   "KEY NOT IN VEHICLE" popup.
5. Tests: boot publishes false; ignition-off close = no-op;
   ignition-on close with no key = warning; ignition-on close
   with cabin key = no warning; warning clears on key reappear
   and on ignition off; partial close doesn't fire.

**Risks.**  Low.  Sequence scan latency adds up (~150 ms total)
but runs after the user shuts the door, off any user-perceived
path.

---

## 15. Smart Unlock feature (key-locked-in-trunk)  *(M)*

**Trigger.**  Real OEM convenience feature: when the user double-
locks the vehicle and walks away, if any paired fob is detected
inside the trunk via a follow-up cargo-area scan, the vehicle
unlocks itself + chirps + flashes to alert the user.  Prevents the
classic "locked my keys in the trunk" failure mode.

**Built on (in `main`).**  KeySearchArbiter with `AntennaSet::
TrunkInside` already defined.  `walk_away_lock` event marks the
"freshly locked from outside" moment.  Chime + hazard light
arbiters already accept claims.

**Plan.**
1. New `FeatureId::SmartUnlock = 0x1E` (or next free).
2. Subscribe to `Cabin.LockStatus`, `Cabin.LockStatus.LastRequestor`,
   `Cabin.LockStatus.EventNum`.
3. On a fresh lock event whose `LastRequestor` ∈ {`KeyfobRke`,
   `WalkAwayLock`, `KeypadLock`}:
   - Delay 1.5 s (gives the user time to step away).
   - Submit `TrunkInside + Authenticated` search.
   - If non-empty: claim trunk open through `trunk_arbiter`,
     chirp horn briefly, flash hazards 3×.
   - Publish `Body.SmartUnlock.LastEvent` (String, e.g.
     `"TRIGGERED" | "NO_KEY"`).
4. Dealer cal: `dealer.smart_unlock_enabled` (default `true`).
5. Re-locks: if the user re-locks during the delay window, abort
   the search.
6. Tests:
   - Fob in trunk + lock → triggers.
   - No fob → no-op.
   - Re-lock during delay → aborts.
   - Dealer flag off → never triggers.

**Risks.**  False positives if the user actually wanted to lock a
fob inside (rare).  The 1.5 s delay + audible feedback should make
it obvious; can extend with a "confirm by holding RKE LOCK"
override later.

---

## 16. NFC Entry feature  *(M)*

**Trigger.**  Phase 7d added the HMI plumbing for NFC cards (N1,
N2) and phones with NFC at both `DriverHandle` and `PushButton`
positions, but no bridge feature consumes them.  Currently a card
or phone at the driver-handle does nothing.

**Built on (in `main`).**  Signal paths exist:
- `Body.PEPS.Plant.NfcCard.{N}.Position` (NfcPosition enum).
- `Body.PEPS.Plant.BlePhone.{N}.Zone` (Zone enum — phones tap via
  the existing zone signal at `LeftFront`/`RightFront`/`KeyCylinder`
  with the side-aware mapping already in the HMI).
- `Body.Connectivity.NfcCardPresent`, `Body.Connectivity.NfcPhonePresent`
  (HMI-writable bools, unused today).
- `door_lock_arbiter` accepts new `FeatureId` claims at all
  priorities.

**Plan.**
1. New `FeatureId::NfcEntry = 0x1F` allow-listed on the door-lock
   arbiter at `Priority::Medium`.
2. Subscribe to:
   - `Body.PEPS.Plant.NfcCard.{1,2}.Position`
   - `Body.PEPS.Plant.BlePhone.{1,2}.Zone` (for NFC-equipped phones
     near the driver handle).
3. Rising edge to `DriverHandle` (or phone arrival at the driver-
   side B-pillar Zone) → submit a quick auth check via the
   KeySearch arbiter (`SingleHandle(DriverDoor) + Authenticated`)
   and dispatch `UnlockAll` (or driver-only per two-stage) through
   the door-lock arbiter.
4. Rising edge to `PushButton` → publish a one-shot
   `Body.Switches.StartStop.IsPressed` rising edge so
   `VehicleStartingControl` can use the NFC tap as a start-button
   substitute (PEPS-mode only).
5. Tests:
   - NFC card at DriverHandle → unlock.
   - NFC phone at LeftFront (LHD) → unlock.
   - NFC card at PushButton → ignition press fires.
   - With `dealer.two_stage_unlock = true`: first tap unlocks
     driver only, second tap unlocks all.

**Risks.**  Double-fire if the user holds the card on the reader —
add a debounce.  The phone-via-Zone-mapping requires the HMI's
existing side-aware driver_door_side cal to route correctly.

---

## 17. Key-lost warning chime  *(XS)*

**Trigger.**  `docs/key-search-arbiter-and-ignition.md` §11 lists
this as a trivial add: while the vehicle is moving (`Vehicle.Speed
> threshold`) and `Body.PEPS.ApproachKeys` drops to 0, sound a
short chime + display a cluster warning.  Catches "fob dropped out
of the car" mid-drive.

**Built on (in `main`).**  `ApproachKeys` published by the
KeySearch arbiter.  `Vehicle.Speed` HMI input.  `Body.Chime.IsActive`
plant model.

**Plan.**
1. New tiny feature `features/key_lost_warning.rs`.
2. Subscribe `ApproachKeys`, `Vehicle.Speed`,
   `Vehicle.LowVoltageSystemState`.
3. When `Speed > 5 km/h` AND `ApproachKeys` transitions ≥1 → 0
   AND power ∈ {`ON`, `START`}: claim chime for 2 seconds, publish
   `Vehicle.Starting.KeyLostWarning = true` (new bool).
4. Auto-clear after 2 s or when ApproachKeys ≥ 1 again.
5. Tests: trigger on the drop edge; suppression while parked.

**Risks.**  None significant.

---

## 18. ASIL-B on FreeRTOS + Ferrocene-qualified Rust  *(XXL — program-level)*

**Trigger.**  Today the platform-provider proposal assumes Classic
AUTOSAR on the M7 for the ASIL-B layer (BSW + safety SWCs).  Move
that layer to **FreeRTOS / SafeRTOS + ISO 26262-qualified Rust**
so the OEM owns the M7 application code in the same language they
already own the QM layer in, and we drop a tier of vendor lock-in
and licensing.  Architecturally this collapses "QM Rust on A53" and
"ASIL-B C on M7" into one Rust codebase with two safety classes,
linked only by build-time toolchain selection.

**Built on (in `main`).**  Nothing — this is a parallel hardware /
toolchain track, not a feature.  Existing QM Rust code is the
proof-of-concept that the language fits; the safety-monitor IPC
message types in `ipc_message.rs` and the FaultReport / CmdAck
discipline already define the M7 ↔ A53 wire format.

**Plan.**

*Phase 1 — Toolchain evaluation (M).*
1. Pick a qualified Rust compiler.  **Ferrocene** (Ferrous Systems)
   is the current reference — qualified to ISO 26262 ASIL-D / IEC
   61508 SIL 4 from rustc 1.68+, recent releases track upstream
   within a few months.  Alternatives: **AdaCore GNAT Pro for
   Rust** (similar coverage), or rolling our own qualification kit
   (multi-year, not realistic).
2. Pick the RTOS.  **SafeRTOS** (WHIS) — FreeRTOS-API-compatible
   with a TÜV-certified safety case to IEC 61508 / ISO 26262
   ASIL-D; the de-facto choice when the BSP needs to clear an
   automotive audit.  Vanilla FreeRTOS is fine for dev / CI but
   the cert paperwork lives with SafeRTOS.
3. Stand up a cross-compile target (`thumbv7em-none-eabihf` or
   the vendor MCU's tier-1 triple) under Ferrocene; verify the
   existing `no_std`-compatible parts of `ipc_message.rs` /
   `signal_ids.rs` build clean.

*Phase 2 — MCAL + RTE shim (L).*
4. Replace Classic AUTOSAR MCAL with vendor-provided HAL crates +
   thin Rust wrappers.  Bring up GPIO, CAN-FD, LIN, ADC, DIO,
   watchdog, NVM, crypto accelerator.
5. RTE replacement: define a Rust trait surface that mirrors
   AUTOSAR's port/interface model just enough that ASIL-B
   features port 1:1 from the spec.  No Arxml; trait impls are
   the contract.

*Phase 3 — Pilot ASIL-B feature port (M).*
6. Pick one ASIL-B feature — **CrashUnlock** is the canonical
   choice: small, well-bounded, lives entirely on M7, already
   has its IPC message types defined.  Port to Ferrocene + RTOS
   target.  Run it alongside the existing Classic AUTOSAR
   implementation on a bench rig; compare cycle-accurate
   behaviour over 1000+ injected crash events.
7. Produce the safety-case artifacts: requirements traceability
   (Polarion / DOORS export), software unit specification,
   coverage analysis (MC/DC required at ASIL-B), unit + integration
   test reports, configuration management evidence.

*Phase 4 — Toolchain integration into CI (S).*
8. Add a Ferrocene job to the existing GitHub Actions matrix.
   `cargo +ferrocene build --target thumbv7em-none-eabihf`.
   Cross-compiled artifact gets flashed to a QEMU ARM Cortex-M7
   image (see item 21) for the safety-case regression suite.

*Phase 5 — Full ASIL-B feature port (XL).*
9. Port the remaining ASIL-B safety SWCs one at a time: watchdog
   supervisor, voltage monitor, fault aggregator, safety bus,
   wake-up controller.  Each gets the same artifact set as the
   pilot.

*Phase 6 — Decommission Classic AUTOSAR (M).*
10. Strip the Classic AUTOSAR stack from the build manifest once
    every ASIL-B feature has been ported and signed off.  Free
    the licence seats.

**Risks.**
- **Ferrocene lag.**  Qualified rustc trails upstream by 3–9
  months.  Some `std`-shaped ergonomic features (e.g. const
  generics evolution) may not be in the qualified release for
  another cycle.  Mitigate by writing portable code that compiles
  on both stable and Ferrocene.
- **Cert paperwork.**  ISO 26262 part 6 software unit verification
  is the long pole; an OEM safety auditor signs off the toolchain
  qualification kit + the per-feature evidence.  Budget 6 months
  for the first feature, 1–2 months for each subsequent one.
- **Vendor MCAL ecosystem.**  Most Tier-1 MCU vendors ship Classic
  AUTOSAR MCAL only; a HAL crate may not exist.  Wrapping the C
  HAL in `unsafe` Rust is fine but the FFI boundary becomes the
  weak link in the safety case.  Pick MCU families with
  community-supported HAL crates (STM32, Infineon AURIX has
  embassy support) when the silicon choice is still open.
- **SafeRTOS licence cost.**  WHIS licences per-target-MCU per-
  project.  Build into the BOM, but cheaper than full Classic
  AUTOSAR seats by an order of magnitude.

**Selling angle (memory/preferences.md).**  "Same Rust codebase
across QM (Linux on A53) AND ASIL-B (M7), single toolchain,
single Cargo workspace, single CI matrix.  No Arxml, no model-
generation tools, no vendor BSW seats.  OEM-owned end-to-end."

---

## 19. Kuksa.val databroker integration  *(L)*

**Trigger.**  The bridge's `SignalBus` seam currently has one
implementation: `MockBus` (in-process broadcast channels).  The
production bus is **kuksa.val** databroker — COVESA's reference
gRPC implementation of the VSS data model.  `kuksa_sync.rs`
already attempts a connection on boot but the bridge doesn't
actually round-trip signals through it; today it just retries
forever on a closed socket (visible in any bridge log).  Replace
MockBus with a `KuksaBus` adapter so signals flow through the
real broker.

**Built on (in `main`).**
- `kuksa_sync.rs` — opens a gRPC channel to `localhost:55555`,
  has the retry/back-off loop wired.
- `SignalBus` trait — already abstracts over the bus
  implementation; switching is a `cargo --features kuksa` away in
  principle.
- VSS signal IDs in `signal_ids.rs` — already mapped to VSS-4.0
  paths the broker understands.

**Plan.**
1. **Vendor the broker.**  Ship `kuksa-databroker` as a Docker
   container in the dev environment + a systemd unit on target.
   Optionally embed it directly as a Rust dependency
   (`kuksa-rust` crate) if the binary footprint allows.
2. **`KuksaBus` adapter** implementing `SignalBus`:
   - `subscribe(path) -> Stream<SignalValue>` → kuksa.val
     `Subscribe` RPC, map gRPC `Datapoint` to our `SignalValue`.
   - `publish(path, value)` → kuksa.val `Set` RPC.
   - `latest_value(path)` → kuksa.val `GetValue` RPC.
3. **Cargo feature flag.**  `default = ["mock"]`,
   `kuksa = ["dep:kuksa-rust"]`.  CI builds both.  Unit tests
   keep MockBus; integration suite gains a `kuksa-integration`
   test crate that spins the broker via testcontainers.
4. **Signal-type translation.**  Kuksa.val uses VSS Datapoint
   variants; our `SignalValue` is a smaller enum.  Build a
   bidirectional codec, exhaustive match on both sides.
5. **Authorization.**  Production kuksa.val supports per-client
   ACLs.  Define a vss-bridge client identity with scoped
   read/write per signal family — wire into the systemd unit.
6. **Regression.**  Run the existing lib + e2e suite against
   `KuksaBus` (with the broker running in-process for tests).
   Anything that depended on `MockBus::history()` for
   assertions needs an equivalent query against the broker.

**Risks.**
- **Latency.**  Every publish/subscribe is now a gRPC round-trip
  to localhost (~50 µs vs MockBus's ~µs).  Most features are
  fine; the approach-poll loop in `KeySearchArbiter` and any
  brake-pre-auth path may need budget revisits.
- **Test history.**  Many tests use `bus.history()` to assert
  the sequence of publishes.  Kuksa doesn't keep a publish log
  per-subscriber — we'd need a thin in-test recorder layered on
  the `KuksaBus`, or a `tee` adapter that fans out to both
  Kuksa and an in-memory recorder.

---

## 20. Feature-completeness + interconnection audit  *(M)*

**Trigger.**  We have 35+ features today, ~700 lib tests, ~20
Gherkin features, but no systematic confirmation that every
feature has the full requirements → Gherkin → unit → module → e2e
chain, and that **cross-feature interactions** (which is where
most real-vehicle defects live) have explicit test coverage.

**Built on (in `main`).**
- One source-of-truth file per feature (`src/features/*.rs`).
- BDD scenario files in `features/*.feature`.
- Unit tests embedded in each feature module.
- e2e tests in `tests/e2e/`.
- Integration tests in `tests/ws_integration.rs`.

**Plan.**
1. **Build the matrix.**  Script that walks
   `src/features/*.rs` and for each feature emits a row of:
   - Module name
   - Public requirements doc / Gherkin file (link)
   - Unit tests (count + names from `#[tokio::test]` discovery)
   - Module-level tests (cucumber feature file?)
   - e2e tests (which `.feature` references it)
   - ws_integration coverage
   - **Interconnections**: features that read/write the same
     VSS signals (parse the consts from each module).
2. **Identify gaps.**  Any row missing a column → backlog ticket.
3. **Interconnection contracts.**  For every pair of features
   that share a signal, document the contract — who writes,
   who reads, who arbitrates conflicts.  Already implicit in
   feature comments; surface as a top-level
   `docs/feature-interconnections.md`.  Examples:
   - `Cabin.LockStatus` written by `DoorLockArbiter`, read by
     PerimeterAlarm, AutoRelock, SmartUnlock, KeypadLock,
     PassiveEntry, WalkAwayLock, SlamLock.  Eight readers, one
     writer.  Test that every reader has a "stale-cache /
     missed-event" test (we've discovered three bugs here).
   - `Body.Horn.IsActive` — central question: who has the
     authority?  Today `LockFeedback` (mislock), `PanicAlarm`,
     `PerimeterAlarm`, `ManualHorn`.  Document priority
     ordering, add cross-feature tests for the priority
     boundaries.
4. **Cross-feature scenarios.**  Write Gherkin scenarios that
   span multiple features explicitly:
   - SmartUnlock undoes a PhoneApp lock → mislock honk + 2-flash
     unlock pattern + LockFeedback's chime suppressed.
   - WAL hold-off + driver returns with fob in pocket → WAL
     proceeds on next zone tick.
   - SlamLock inversion during a running PerimeterAlarm chime →
     does the trim-press disarm anything?
5. **CI gate.**  Add a "coverage matrix" job to CI that fails
   the build if a new feature lands without all five rows
   populated.

**Risks.**  Discovery cost — there will be gaps.  Each gap
becomes a sub-ticket.

---

## 21. Run the test suite on a virtualised target  *(L)*

**Trigger.**  Today CI runs `cargo test` natively (Linux x86-64).
The production target is dual-core (Cortex-M7 + Cortex-A53)
ARM.  Bugs in endianness, atomic ordering, alignment, and
cross-core IPC don't surface in native CI.  Run the full test
suite on a virtualised target so we catch them before bench.

**Built on (in `main`).**
- The bridge is a single Rust binary today; cross-compilation
  works (`cargo build --target aarch64-unknown-linux-gnu` is
  green).
- Item **18** brings the M7 cross-compile (Ferrocene + thumbv7em).
- All tests use tokio + `MockBus` — no native-only deps in
  the test harness.

**Plan.**
1. **Pick a virtualiser per side.**
   - **A53 side**: **QEMU `aarch64-softmmu`** with `virt`
     machine + Linux rootfs (Yocto recipe-of-record matches the
     target image).  Mature, fast, full Linux + std.
   - **M7 side**: **Renode** is the strongest fit — open-source,
     scriptable, supports multi-core ARM, has libraries for
     CAN/LIN/SPI peripherals.  QEMU's Cortex-M support exists
     but is thinner.  Alternative: **Avocado-VT** + vendor
     SoC simulators (Infineon Aurix has a free TriCore sim).
2. **Cross-compile + package.**
   - `cargo build --release --target aarch64-unknown-linux-gnu`
     for the A53 bridge binary.
   - `cargo +ferrocene build --release --target thumbv7em-none-eabihf`
     for the M7 safety SWCs (gated on item 18 landing first).
   - Bundle both into a Renode platform script that boots Linux
     on the virtual A53 and the M7 image on the virtual M7,
     wires up the IPC channel between them.
3. **Port the test harness.**
   - `tests/e2e/` runs as-is on Linux-on-QEMU; just the runner
     environment changes.
   - `tests/ws_integration.rs` needs the WS port forwarded out
     of the VM so the host-side test driver can connect.
   - A new test crate `tests/cross_core/` for the IPC
     round-trip tests that only make sense with both cores up.
4. **CI runners.**
   - GitHub Actions: a self-hosted runner with KVM enabled +
     Renode pre-installed.  Or, accept the slower nested-virt
     path on GitHub-hosted Linux runners — Renode boots fast
     enough that nested KVM isn't critical.
   - Job naming: `test-virt-a53`, `test-virt-m7`,
     `test-virt-cross-core`.  Run on PR + nightly.
5. **Failure-injection layer.**  Renode supports scripted fault
   injection (bit flips, missed interrupts, jittered IPC).
   Add a tier of nightly chaos tests that walk the existing
   feature suite under each injection.

**Risks.**
- **Renode platform descriptions** for the exact MCU + peripheral
  set the OEM ships need to be hand-written if the silicon is
  exotic.  Mainstream NXP / Infineon / Renesas families are
  covered in upstream Renode; truly custom silicon may take a
  multi-week port.
- **CI runtime.**  Booting Linux + Renode + running the full
  suite is minutes per PR.  Tier 2 / nightly is fine; gating
  every PR on virt-target is probably too slow — keep the
  current native suite as the PR gate and let virt-target run
  on `main` post-merge + nightly.
- **Cross-core IPC fidelity.**  Renode's CAN/LIN models are
  packet-accurate but not always cycle-accurate.  Pure logical
  regression is reliable; timing-sensitive races still need
  bench validation.

---

## 22. Refresh to the latest COVESA VSS release  *(M)*

**Trigger.**  The bridge currently targets **VSS v4.0** — visible in
the doc comments scattered through `ws_bridge.rs`, `main.rs`, and
the assumed paths in `signal_ids.rs` (e.g.
`Vehicle.LowVoltageSystemState`, `Vehicle.Cabin.Infotainment.HMI.DayNightMode`,
the four `Vehicle.Chassis.Axle.Row{1,2}.Wheel.{Left,Right}.Tire.IsPressureLow`
TPMS signals).  COVESA cuts a new VSS release every few months;
each one adds signals, occasionally renames or reshapes existing
branches, and updates the catalog units / enums.  Staying on v4.0
ships an aging schema and forecloses any new signals the OEM may
want to expose to head units / fleet telematics / OTA tools.

**Built on (in `main`).**
- `signal_ids.rs` — the single chokepoint where every VSS path
  is registered with a stable 32-bit ID.  Any rename in the new
  catalog only touches this file + the const string in each
  feature that references it.
- The MockBus / KuksaBus (item 19) is path-agnostic — it carries
  whatever string we give it.  No bus-layer changes needed.
- VSS-tools (`vspec`, `vss2vss-yaml`, the JSON / Protobuf
  generators) — used to produce the kuksa.val catalog and the
  HMI side's signal manifest.

**Plan.**

1. **Check the latest release.**  Pull
   `https://github.com/COVESA/vehicle_signal_specification/releases`
   and read the changelog for every minor since v4.0.  As of the
   writing of this item: v4.1, v4.2, v5.0 (breaking) shipped;
   v5.x is the likely target.  Note in the PR description which
   specific tag is being adopted.
2. **Pull the spec.**  `git submodule add` the COVESA repo at
   the chosen tag, or vendor the generated YAML catalog into
   `vss-bridge/specs/vss-<version>.yaml`.
3. **Generate the path manifest.**  Run `vspec export json` (or
   the binary Protobuf generator) into a file the HMI and the
   bridge both consume.  Update the HMI's manifest path.
4. **Migrate `signal_ids.rs`.**
   - For every path the bridge currently registers, look up the
     new release's equivalent.  Three outcomes per path:
     - **Unchanged** — no work.
     - **Renamed / reshaped** — update the string; keep the
       32-bit ID stable so on-the-wire compatibility holds.  Log
       the rename in a section of the file's header so
       downstream Tier-2 consumers know what moved.
     - **Deleted** — usually means a signal was promoted into a
       struct or replaced by something more idiomatic.  Map our
       use to the replacement.
5. **New signals worth adopting.**  Sweep the new spec for any
   body-domain signals we don't expose yet but probably should:
   - VSS 4.1+: `Vehicle.Cabin.Door.RowN.SideN.ChildLock.IsActive`
     (we have our own `Body.Doors.RowN.SideN.IsChildLockActive` —
     consider migrating to the canonical path).
   - VSS 5.0+: cabin comfort / interior lighting signals that
     might subsume our `Cabin.Lights.IsDomeOn`.
   - Anything new under `Vehicle.Body.PEPS` — VSS hasn't
     historically modelled PEPS deeply, but the cabin / driver-
     identification area in 5.x is expanding.  Worth a look.
6. **Update all in-code references.**  Doc comments saying "VSS
   v4.0" become "VSS v5.x" (or whatever the chosen tag is).
   `Cargo.toml` `description` field updates to mention the new
   version.
7. **Regenerate the kuksa.val catalog** (gated on item 19
   landing).  The broker only understands signals that exist in
   its loaded VSS catalog; the regenerated catalog is what makes
   the new paths queryable.
8. **HMI sync.**  The web HMI reads the signal manifest at
   runtime; it picks up the new paths automatically once the
   manifest is regenerated.  Test the chip rendering / signal
   log explorer pages against the new catalog.
9. **Tests.**
   - Existing lib + e2e + ws_integration suites should pass
     unchanged on renamed-but-equivalent paths after step 4.
     A failing test is most likely catching a rename we missed.
   - Add one new test per newly-adopted signal that publishes
     and reads it through the bus, asserting the round-trip
     works.
10. **Tier-2 callout.**  If any path the platform's API surface
    exposes to head units / cluster gets renamed, that's a
    breaking change downstream — flag it in the PR description
    and coordinate the rename with whatever consumer owns the
    head-unit / cluster integration.

**Risks.**
- **Breaking renames cascade.**  A renamed signal in the schema
  means every feature consuming it touches; with ~30 features
  and ~300 signal paths the search-and-replace is mechanical but
  needs careful review.  Mitigate by doing the rename in two
  PRs: one that adds aliases (both old and new paths publish to
  the same `latest_value`), then a follow-up that removes the
  old path after every consumer has migrated.
- **Deleted signals.**  Rare but real.  Need a fallback
  signal-or-feature-removal plan per case.
- **HMI manifest drift.**  If the HMI loads a stale manifest
  it'll silently fall back to "unknown signal" rendering.  Wire
  a manifest-version check into the WS handshake so a mismatched
  HMI logs a loud warning.
- **Cadence.**  COVESA ships VSS roughly twice a year.  Once
  we're on a recent tag the rebase cost stays small (~a day per
  release) if we keep up; let it slip 2-3 releases and the next
  bump grows non-linearly.  Worth wiring into the team's
  quarterly cadence after the first catch-up.

---

## How to consume this plan

1. Pick an item from the pending list.  Open a sub-branch off
   `main` named `feature/<item-tag>` (e.g. `feature/etb-arbiter`).
2. Reference this doc in the PR description; tick the item off
   in the status table when merged.
3. If you discover sub-items mid-implementation, add them under
   the parent here as `(13a)`, `(13b)`, etc.  Don't grow this doc
   into a spec — keep it a tracker.

---

## Out-of-band notes

- **Transport adapters (item 9)** remain on a separate track —
  vendor-specific dependencies and CI infra make them unsuitable
  for sub-branching off this plan.
- The original `docs/key-search-arbiter-and-ignition.md` design
  doc is the canonical spec for items **13–17** (Phase 8+ in that
  doc's numbering).  Cross-reference when implementing.
