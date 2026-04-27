# Body-System Validation Plan — Owner's Manual + FMVSS

**Branch:** `validation/body-e2e-from-owner-manual-fmvss`
**Scope:** End-to-end test coverage for the body-controller stack derived from
(a) a North-American SUV owner's manual and (b) the Federal Motor Vehicle
Safety Standards (FMVSS) applicable to the body system.
**Status:** Planning document — no source changes in this commit.

---

## 1. Purpose

The existing `vss-bridge` test pyramid covers feature-internal logic
(unit tests) and wired-up feature interactions (cucumber e2e via
`tests/e2e`).  Both tiers were authored from the engineering side —
*"what does the feature module promise to do?"*.

This plan adds a **third axis**, authored from the customer/regulator side:

* **Owner's manual derived tests** — *"does the vehicle behave the way
  the user-facing documentation claims it does?"*
* **FMVSS derived tests** — *"does the vehicle meet the safety
  requirements that allow it to be sold in the US?"*

Both perspectives produce **black-box scenarios** that the body controller
must satisfy regardless of implementation detail.  These scenarios are the
right material for **acceptance** and **certification readiness** — not
for catching unit bugs.

---

## 2. Sources

### 2.1 Vehicle reference

**2024 Toyota RAV4 — Owner's Manual** (representative NA-region compact SUV).
Vehicle line chosen because:
- Best-selling SUV in North America for multiple years; representative
  feature set for the segment.
- Covers all body-controller features the platform implements: smart
  entry, RKE (lock/unlock/panic/trunk), comfort blink, auto headlamps,
  hands-free liftgate, anti-pinch windows.
- Manual is hosted publicly by Toyota at
  [toyota.com/owners — 2024 RAV4](https://www.toyota.com/owners/warranty-owners-manuals/digital/rav4/2024/)
  and on third-party mirrors (e.g.
  [ownersman.com/manuals/2024-Toyota-Rav4-owners-manual](https://ownersman.com/manuals/2024-Toyota-Rav4-owners-manual)).

> **Implementation note.** When test scenarios reference numeric
> tolerances (e.g. *"3 flashes for lane change"*, *"30 s follow-me-home
> hold"*), the value must be **read from the actual PDF chapter** during
> implementation — this plan documents the *category* of test, not the
> exact number, since some details require fetching the 588-page PDF
> directly.  Unknown numerics are flagged below as `‹from manual§N›`.

### 2.2 Regulatory references

US Federal Motor Vehicle Safety Standards (FMVSS) — published under
49 CFR Part 571.  The body-system relevant standards are:

| FMVSS | Title | Source |
|-------|-------|--------|
| **108** | Lamps, Reflective Devices, and Associated Equipment | [49 CFR § 571.108](https://www.ecfr.gov/current/title-49/subtitle-B/chapter-V/part-571/subpart-B/section-571.108) |
| **111** | Rear Visibility | [49 CFR § 571.111](https://www.ecfr.gov/current/title-49/subtitle-B/chapter-V/part-571/subpart-B/section-571.111) |
| **113** | Hood Latch System | [49 CFR § 571.113](https://www.ecfr.gov/current/title-49/subtitle-B/chapter-V/part-571/subpart-B/section-571.113) |
| **118** | Power-Operated Window, Partition, and Roof Panel Systems | [49 CFR § 571.118](https://www.ecfr.gov/current/title-49/subtitle-B/chapter-V/part-571/subpart-B/section-571.118) |
| **206** | Door Locks and Door Retention Components | [49 CFR § 571.206](https://www.law.cornell.edu/cfr/text/49/571.206) |
| **401** | Interior Trunk Release | [49 CFR § 571.401](https://www.law.cornell.edu/cfr/text/49/571.401) |

Standards excluded from scope (not body-system):
- 124 (Accelerator Control), 135 (Brakes), 138 (TPMS), 208 (Crash
  Protection), 305 (EV battery isolation).

---

## 3. Test architecture

The plan adds a new gherkin layer **on top of** the existing test stack —
no rewrites, just additional `.feature` files and step definitions.

```
tests/e2e/                                 (existing — engineering perspective)
├── steps.rs
├── main.rs                                ← runs all .feature files
└── ../../features/
    ├── turn_indicator.feature
    ├── hazard.feature
    ├── panic_alarm.feature                ← merged from feature/panic-alarm
    └── ...

features/owner_manual/                     (NEW — customer perspective)
├── doors_and_locks.feature
├── exterior_lighting.feature
├── interior_lighting.feature
├── turn_signals_and_hazard.feature
├── horn_and_alarm.feature
├── windows.feature
└── liftgate.feature

features/fmvss/                            (NEW — regulator perspective)
├── fmvss_108_lamps_signaling.feature
├── fmvss_111_mirrors.feature
├── fmvss_113_hood_latch.feature
├── fmvss_118_power_windows.feature
├── fmvss_206_door_locks.feature
└── fmvss_401_trunk_release.feature
```

Step definitions are added to a new `tests/e2e/steps_body.rs` module
imported from `tests/e2e/main.rs`, sharing the existing `VssWorld`.

### 3.1 Three-tier execution

| Tier | Driver | What it proves | Existing? |
|------|--------|----------------|-----------|
| **T1: in-process** | cucumber-rs + MockBus + Tokio paused | Feature business logic + arbitration produces the correct VSS signal value sequences | ✅ existing pattern |
| **T2: bridge integration** | WebSocket client → real `vss-bridge` binary → JSON over `:8080` | The full Rust stack (arbiters + features + plant models) accepts realistic inputs and emits realistic outputs | ✅ pattern exists in `vss-bridge/tests/ws_integration.rs`; needs scenario expansion |
| **T3: HMI end-to-end** | Headless browser (Claude Preview / Playwright) driving `vss-hmi.html` | The cluster icon, button, and ribbon affordances actually fire the right signals — i.e. UX matches plant-model behaviour | partial (manual smoke only today) |

**T1 owns most acceptance scenarios.**  T2 catches IPC/serialization
regressions.  T3 covers the customer-facing UX claims that depend on
HMI wiring (e.g. "click the LOCK button on the RKE panel and the door
lock plant model engages").

---

## 4. Owner's-manual scenarios

Grouped by manual chapter with the corresponding implemented feature.

### 4.1 Doors and locks

| # | Manual claim | Test (gherkin scenario name) | Implemented? |
|---|---|---|---|
| OM-DL-01 | RKE LOCK button locks all doors | `Pressing LOCK on the keyfob locks all four doors` | ✅ `RkeFeature::handle_lock` |
| OM-DL-02 | RKE LOCK twice within 3 s engages double-lock | `Two LOCK presses within 3s engages double-lock when variant supports it` | ✅ `RkeFeature` + `DoubleLockRelease` |
| OM-DL-03 | RKE UNLOCK first press unlocks driver only (two-stage) | `Single UNLOCK in two-stage mode unlocks only Row1.Left` | ✅ |
| OM-DL-04 | RKE UNLOCK second press within 3 s unlocks all | `Second UNLOCK within 3s in two-stage mode unlocks all four doors` | ✅ |
| OM-DL-05 | Walking away locks the car automatically | `WalkAwayLock locks all doors when all PEPS devices leave the approach zone` | ✅ |
| OM-DL-06 | Auto-relock if doors don't open after unlock | `AutoRelock re-locks if no door opened within the configured window` | partial — feature exists but not wired in `main.rs` (TODO) |
| OM-DL-07 | Thumbpad on outside door handle locks | `ThumbPadLock on Row1 outside handle locks all doors after 500 ms hold` | ✅ |
| OM-DL-08 | Child safety locks disable Row 2 inside handles | `Row2 child lock active prevents inside handle release` | partial — signal exists, no feature yet |
| OM-DL-09 | Ignition ON releases double-lock | `DoubleLockRelease clears superlock when ignition transitions to ON` | ✅ |
| OM-DL-10 | Lock chirp / flash on RKE LOCK | `LockFeedback plays 1-flash on lock command` | ✅ |
| OM-DL-11 | Unlock flash on RKE UNLOCK | `LockFeedback plays 2-flash on unlock command` | ✅ |
| OM-DL-12 | Trunk close while cabin unsecured triggers warning flash | `LockFeedback plays unlock flash if trunk closes with cabin unlocked` | ✅ |

### 4.2 Exterior lighting

| # | Manual claim | Test | Implemented? |
|---|---|---|---|
| OM-EL-01 | AUTO position uses ambient light to switch DRL ↔ low beam | `AUTO mode below lux threshold turns on low beams` | ✅ `ManualLighting` |
| OM-EL-02 | High beam stalk pull activates high beam (with ignition ON) | `HighBeam stalk pull with ignition ON activates high beam` | ✅ |
| OM-EL-03 | Auto high beam dips on oncoming vehicle | `AutoHighBeam suppresses high beam when ADAS reports oncoming traffic` | ✅ |
| OM-EL-04 | Follow-me-home holds low beams 30 s after ignition off (dark only) | `FollowMeHome holds low beams 30s post-ignition-off when below lux threshold` | ✅ (45 s in current code — verify against manual: ‹from manual§4-2›) |
| OM-EL-05 | Front fog lamp only with low beam ON | `FogLamps.Front activates only when low beam is on and switch engaged` | ✅ |
| OM-EL-06 | Rear fog lamp ignition-gated | `FogLamps.Rear deactivates on ignition OFF` | ✅ |
| OM-EL-07 | DRL active in any drive-able state | `DRL on with ignition ON or START regardless of light switch` | ✅ |
| OM-EL-08 | License plate lamp follows low beam | `LicensePlate.IsOn matches Beam.Low.IsOn` | ✅ |

### 4.3 Turn signals and hazard

| # | Manual claim | Test | Implemented? |
|---|---|---|---|
| OM-TS-01 | Tap-to-signal: brief stalk tap → 3 flashes | `Comfort blink 3-flash count when stalk tapped briefly` | ✅ `TurnIndicator` |
| OM-TS-02 | Stalk held → continuous flash until released | `Stalk held continuously activates indicator until released` | ✅ |
| OM-TS-03 | Hazards work with ignition OFF | `HazardLighting engages with ignition OFF` | ✅ |
| OM-TS-04 | Hazards override active turn signal | `Hazard HIGH wins over active Turn MEDIUM` | ✅ |
| OM-TS-05 | Hazards survive ignition state changes | `Hazard claim persists across ignition transitions` | ✅ |

### 4.4 Horn and alarm

| # | Manual claim | Test | Implemented? |
|---|---|---|---|
| OM-HA-01 | RKE PANIC button starts alarm: lights flash + horn chirps | `Panic press from paired keyfob engages synchronized blink+chirp` | ✅ (PR #13) |
| OM-HA-02 | Panic press while alarm running cancels it | `Second panic press disengages alarm and releases all claims` | ✅ |
| OM-HA-03 | Alarm operates with ignition off | `PanicAlarm engages with ignition OFF` | ✅ |
| OM-HA-04 | Steady "alarm sounding" status flag for telematics | `Vehicle.Body.Alarm.IsActive published once on engage, not duty-cycled` | ✅ |

### 4.5 Windows (NEW — feature does not yet exist)

| # | Manual claim | Test | Implemented? |
|---|---|---|---|
| OM-WIN-01 | One-touch auto-up to fully closed | `Window auto-up runs to position 0 (closed) on single switch tap` | ❌ no `windows.rs` feature yet |
| OM-WIN-02 | One-touch auto-down to fully open | `Window auto-down runs to position 100 (open) on single switch tap` | ❌ |
| OM-WIN-03 | Anti-pinch reverses on obstacle | `Window auto-up reverses to ≥125 mm open when obstruction simulated` | ❌ |
| OM-WIN-04 | Window switch retain after ignition OFF (~ 45 s) | `Windows operable for retention window post ignition OFF` | ❌ |
| OM-WIN-05 | Driver master lock disables passenger switches | `Master window lock disables Row1.Right + Row2.* switches` | ❌ |

### 4.6 Liftgate (PARTIAL — basic open/close in trunk plant model only)

| # | Manual claim | Test | Implemented? |
|---|---|---|---|
| OM-LG-01 | Smart-power liftgate one-touch open | `Trunk.OpenCmd published, plant model transitions to open` | ✅ |
| OM-LG-02 | Liftgate kick sensor opens hands-free with key in zone | `Kick gesture with PEPS in approach zone opens trunk` | ❌ no kick sensor signal yet |
| OM-LG-03 | Liftgate jam protection reverses on obstruction | `Liftgate close reverses on simulated jam` | ❌ |
| OM-LG-04 | Closing liftgate on locked vehicle re-locks trunk | `Trunk close after RKE-trunk-unlock re-engages trunk lock` | ✅ |

---

## 5. FMVSS scenarios

Each FMVSS scenario lists the **regulatory clause**, the **testable
intent at the body-controller software level**, and a **scenario name**.
Photometric, mechanical, and crash-rig requirements are out of scope —
those are bench/sled tests, not body-controller tests.

### 5.1 FMVSS 108 — Lamps, Reflective Devices

| Req | Clause intent | Software testable? | Scenario |
|---|---|---|---|
| FM108-S6 | Turn signal flash rate **60–120 flashes/min** (1–2 Hz) | ✅ at HMI Lamp.IsOn cadence (BlinkRelay output) | `Turn indicator lamp.IsOn pulses at 1–2 Hz cadence under TurnIndicator request` |
| FM108-S6 | Turn signal continues until released or cancelled by steering return | ✅ via `Body.Switches.TurnIndicator.Direction` | `Turn signal stops when stalk returns to OFF` |
| FM108-S5.5.10 | Hazard flasher **operates with ignition in any state** | ✅ existing | `Hazard flashes regardless of LowVoltageSystemState` |
| FM108-S5.5.10 | Hazard cannot be cancelled by direction stalk | ✅ priority test | `Turn stalk cannot suppress active hazard signaling` |
| FM108-S5.5.11 | Headlamp activation gates DRL OFF (DRL inhibited when low beam on) | ✅ via low-beam arbiter | `DRL released by ManualLighting when LightSwitch=BEAM` |
| FM108-S7.9 (DRL) | DRLs **shall be steady-burning**, not flashing | ✅ duty cycle check | `DRL output never toggles while ignition ON` |
| FM108-S6.1.5.4 | Stop lamp activation **shall be ≤ 100 ms** from brake pedal apply | ✅ time-bound test | `Body.Lights.Brake.IsActive becomes TRUE within 100 ms of Chassis.Brake.PedalPosition > 0` |
| FM108-S6 | Backup lamp activates when transmission in REVERSE, not in any forward gear | ✅ existing `BrakeReverseLamps` | `Backup lamp on with negative gear and ignition ON; off otherwise` |

### 5.2 FMVSS 111 — Rear Visibility

Mirror **photometric/optical** requirements are out of scope.
Software-testable subset:

| Req | Clause intent | Software testable? | Scenario |
|---|---|---|---|
| FM111-S6 | Rear-view image of the rearward viewing area must display when reverse gear engaged | ✅ at signal level (`Body.Cameras.Rear.IsActive` style) | `Rear camera signal asserted within manufacturer's spec when REVERSE selected` |
| FM111-S6.2 | Linger time after gear change to non-REVERSE shall not retain image past safe window | ✅ time-bound | `Rear camera signal de-asserts within reasonable window of leaving REVERSE` |

> Both require a **new `RearVisibility` feature module** + camera signal
> in the catalog.  Currently out of scope of the v1 body controller; flag
> as future work.

### 5.3 FMVSS 113 — Hood Latch

Mostly a mechanical standard.  Software intent at the body controller:

| Req | Clause intent | Software testable? | Scenario |
|---|---|---|---|
| FM113-S4.3 | Hood-open status must be reflected to the cluster while ignition is ON | ✅ via `Body.Hood.IsOpen` | `Body.Hood.IsOpen=TRUE drives cluster warning when ignition ON` |
| (industry-best) | Hood ajar warning suppressed below an interlock speed threshold? **Not** required by FMVSS 113 — implementation choice. | n/a | — |

### 5.4 FMVSS 118 — Power Windows

| Req | Clause intent | Software testable? | Scenario |
|---|---|---|---|
| FM118-S5 | Window pinch force ≤ **100 N** OR auto-reverse before contact | partial — bench test for force, software test for reverse logic | `Window auto-reverse asserted when obstruction signal injected during auto-up` |
| FM118-S5.1 | After auto-reverse, window opens to ≥ 125 mm or to original start position | ✅ with position model | `Reversed window settles at position ≥ original-start or ≥ 125 mm open` |
| FM118-S5(c) | Window not operable when key removed (subject to retain-after-off exemption) | ✅ ignition-state gate | `Window switch input ignored once retain window expires` |

> Requires a new `Windows` feature + plant model tracking position.
> Existing signal `Body.Doors.Row1.Left.Window.Position` is in the
> catalog but no feature drives it.

### 5.5 FMVSS 206 — Door Locks

| Req | Clause intent | Software testable? | Scenario |
|---|---|---|---|
| FM206-S4.1.1.4(a) | When the door is locked, **outside** handle release shall be inoperative | ✅ via `Handle.Outside.IsPulled` while `IsLocked=true` | `Pulling outside handle on a locked door does NOT publish IsOpen=TRUE` |
| FM206-S4.1.1.4(b) (rear side doors) | When **child lock** is engaged, **inside** handle release shall be inoperative | ✅ | `Pulling Row2 inside handle while ChildLockActive does NOT publish IsOpen=TRUE` |
| FM206-S4.1.3 | A **separate action** is required to disengage the lock and to operate the handle | ✅ logic test | `Single inside-handle pull on a locked door must require unlock first` |
| FM206 (industry interpretation) | Crash-detected unlock: doors unlock on airbag deployment / impact event | ✅ existing `CrashUnlock` priority | `Vehicle.Safety.CrashDetected=TRUE unlocks all doors at HIGHEST priority` |

### 5.6 FMVSS 401 — Interior Trunk Release

| Req | Clause intent | Software testable? | Scenario |
|---|---|---|---|
| FM401-S4 | Interior trunk release must **operate independently of door lock state** (a child trapped in the trunk must escape even when vehicle is locked from outside) | ✅ priority test | `Pulling interior trunk release while doors are locked publishes Trunk.OpenCmd=TRUE` |
| FM401-S4 | Interior trunk release **glow** / illumination — for auto-illuminating versions, must light when trunk is closed | ✅ via `Body.Trunk.InteriorRelease.Illuminated` (signal needs adding) | `Trunk interior release lamp on when trunk lid latched` |

> Requires a new signal `Body.Trunk.InteriorRelease.IsPulled` and a tiny
> `InteriorTrunkRelease` feature module that bypasses the door-lock
> arbiter to publish `Trunk.OpenCmd` directly.

---

## 6. Coverage matrix

| Source | Total scenarios | Already covered | New feature work needed | Just need new tests |
|---|---:|---:|---:|---:|
| Owner's manual | ~32 | 22 | 8 (windows, kick sensor, child lock, liftgate jam) | 2 |
| FMVSS 108 | 8 | 6 | 0 | 2 (timing-bound) |
| FMVSS 111 | 2 | 0 | 2 (camera) | 0 |
| FMVSS 113 | 1 | 0 | 0 | 1 |
| FMVSS 118 | 3 | 0 | 3 (windows feature) | 0 |
| FMVSS 206 | 4 | 3 | 1 (child-lock guard) | 0 |
| FMVSS 401 | 2 | 0 | 1 (interior release feature) | 1 |
| **Total** | **~52** | **~31** | **~15** | **~6** |

About 60 % of the planned scenarios can be authored against the existing
codebase as a **pure test-only PR**.  The remaining 40 % drive a small
list of new feature modules that the platform would need anyway for a
production-grade body controller.

---

## 7. Milestones and deliverables

| # | Deliverable | LOC est. | Branch / PR |
|---|---|---:|---|
| **M1** | This plan document (`docs/validation-plan-body-e2e.md`) | ~400 | this branch |
| **M2** | T1 gherkin spec for owner's-manual scenarios that map to existing features (~22 scenarios) + step defs | ~600 (mostly steps) | `validation/owner-manual-existing-tests` |
| **M3** | T1 gherkin spec for FMVSS scenarios that map to existing features (~10 scenarios) | ~400 | `validation/fmvss-existing-tests` |
| **M4** | T2 WebSocket integration test runner + 4 representative scenarios | ~250 | `validation/t2-ws-integration` |
| **M5** | T3 HMI E2E test runner (Playwright or Claude-in-Chrome MCP) + 5 representative scenarios | ~300 | `validation/t3-hmi-playwright` |
| **F1** | Windows feature + plant model + FMVSS 118 + OM-WIN-01..05 | ~800 | `feature/windows` |
| **F2** | Child-lock guard + FMVSS 206 child-lock + OM-DL-08 | ~250 | `feature/child-lock-guard` |
| **F3** | Interior trunk release + FMVSS 401 | ~150 | `feature/interior-trunk-release` |
| **F4** | Liftgate kick sensor + jam protection | ~400 | `feature/liftgate-power` |
| **F5** | Rear camera + FMVSS 111 | ~300 | `feature/rear-camera-stub` |

Sequencing: **M1 → M2 → M3** are the immediate deliverables — they
execute against today's feature set.  **M4 / M5** uplift the test pyramid
to cover the bridge and HMI layers.  **F1..F5** are independent feature
streams that close the remaining FMVSS / owner's-manual gaps; each gets
its own tests piggybacked.

---

## 8. Open questions for the team

1. **Manual numerics.** The flash counts, timeout durations, and lux
   thresholds in §4 must be set from the actual 2024 RAV4 manual or from
   the platform's calibration tables — not from this document.  Action:
   read the PDF chapters `3-2 Doors and locks`, `3-2 Lights`, `3-2
   Power windows`, `3-3 Liftgate`.
2. **FMVSS interpretation.** The "≤ 100 ms brake-lamp delay" in
   FMVSS 108 references an SAE-J586 spec; confirm whether the platform's
   end-to-end latency target includes plant-model and arbiter time, or
   only feature business logic.
3. **Cucumber scenario language.** Stick with English-only or add
   localized variants?  Recommendation: English-only — these are
   engineering acceptance tests, not e2e UX tests.
4. **Bench test handoff.** FMVSS 118 pinch force, 113 hood latch
   strength, 206 inertial latch — purely mechanical and out of scope.
   Plan does not attempt to substitute software tests for them.
5. **Production calibration vs. demo defaults.** Several requirements
   (e.g. follow-me-home duration, two-stage unlock window) are
   variant-configurable.  Tests should run against the *default
   variant* table; per-OEM-customer calibration is verified separately
   during integration.

---

## 9. Why this is worth doing

- **Sales tool.** "Our body controller passes a documented set of
  FMVSS-derived tests on every commit" is a credible answer to OEM
  procurement asking *how* the platform proves itself.  Today the answer
  is the engineering test suite — strong, but not in customer language.
- **Ratchet against feature drift.** Every owner's-manual claim becomes
  a regression-locked behavior.  A single PR can't silently break
  *"hazards work with ignition off"* without showing in the diff.
- **Pre-staging for certification.** OEMs run real FMVSS bench tests
  with hardware lamps and force gauges.  But software-side regression
  catches **intent** breakage long before a vehicle hits the test cell.
- **Documentation by example.** Each `.feature` file doubles as
  human-readable behavior spec.  New engineers / vendor partners can
  read `features/owner_manual/doors_and_locks.feature` and understand
  the door system without reading the Rust code.

---

*Author: Anup Gadkari. Generated 2026-04-26.*
