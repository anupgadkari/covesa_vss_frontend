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
| 13 | Migrate ExteriorTrunkButton → KeySearch arbiter | M | ✅ Done — `exterior_trunk_button` now submits `AntennaSet::TrunkOutside` + `Authenticated` via `KeySearchArbiterHandle`; no direct `.Zone`/`.PlacedZone` reads |
| 14 | Stop continuous Zone publishing; `PlacedZone` + `LastObservedZone` | L | ✅ Done.  Closed out by audit in the wake of #17 / #23 / #24: zero remaining subscribers to legacy `.Zone` signals anywhere in `vss-bridge/src/`, `signal_ids.rs`, `hmi_alias.rs`, `ws_bridge.rs`, or the HMI.  PEPS plant subscribes only to `PlacedZone`; KeySearchArbiter publishes `LastObservedZone` from every scan; Welcome / SmartUnlock / SmartTrunkPop / KeyLostWarning consume `LastObservedZone`.  Originally targeted as a substantial L; reduced to a no-op audit because the migration completed organically as the other items landed. |
| 14c | Extra scan triggers — brake-press pre-auth | S | ✅ Done |
| 14d | Lost-paired-key scan + cluster warning (formerly door-close snapshot) | S | ✅ Done — merged into #17.  The standalone `lost_pk_scan` feature was deleted; `KeyLostWarning` (item 17) is its strict successor and publishes the same `Vehicle.Controller.Body.PEPS.LostKeyWarning` bool on the wire, with corrected gating (cabin sealed under power vs. the original door-close trigger) and a chime claim. |
| 15 | Smart Cabin Unlock (key-locked-in-cabin) | M | ✅ Done — `features/smart_unlock.rs` subscribes to `LockStatus`/`LastRequestor`/`EventNum`, runs `Sequence(Cabin + AllApproach, Authenticated)` scan, dispatches `UnlockAll` on "paired key in cabin only".  **Originally specified as the key-locked-in-trunk case (now item 23); the cabin case was built under the `SmartUnlock` name and is documented here.** |
| 23 | Smart Trunk Pop (key-locked-in-trunk) | M | ✅ Done — `features/smart_trunk_pop.rs` runs `TrunkInside` Authenticated scans on a fresh external lock event while quiescent and pops the trunk via the trunk arbiter when a paired key is found.  Two trigger paths: direct (lock event with trunk already closed) and pending-after-trunk-close (latched on a lock event with trunk still open; consumed on the trunk's open→closed edge — power-tailgate case).  PR #49. |
| 16 | NFC Entry feature (card/phone tap → unlock; tap at push-button → start) | M | ✅ Done — `features/nfc_entry.rs` handles `NfcCard.{1,2}.Position → DriverHandle` and `BlePhone.{1,2}.NfcTap → UnlockAll`; PushButton tap publishes `NfcAuthBypass = true` for start-button override |
| 17 | Key-lost warning chime | XS | ✅ Done — `features/key_lost_warning.rs` owns its own Cabin / Authenticated arbiter scans (last-close edge, ignition-on while sealed, 1-min periodic).  Publishes `Vehicle.Controller.Body.PEPS.LostKeyWarning` (same signal the deleted LostPkScan used) and claims the chime for 2 s.  Latch held across the periodic so still-no-key ticks don't re-chime.  PR #47. |
| 24 | Welcome owns the approach poll (drop arbiter's self-driven loop) | M | ✅ Done — PR #51.  `features::welcome` runs the `AllApproach / Presence / Coalescing::Allowed` periodic scan (fast 700 ms / slow 10 s, suspended ACC/ON/START) and publishes `ApproachState` / `ApproachKeys` / `ApproachPollInterval`.  KeySearchArbiter lost its poll loop, ignition subscription, cadence state, and `with_cadence` constructor — it's now purely a request serialiser.  Completes the "every scan is feature-driven" architectural principle. |
| 18 | ASIL-B on FreeRTOS + Ferrocene-qualified Rust (replaces Classic AUTOSAR M7) | XXL | 📋 Program-level — see plan |
| 19 | Kuksa.val databroker integration (replace MockBus on the SignalBus seam) | L | 📋 Program-level |
| 20 | Feature-completeness + interconnection audit (requirements + Gherkin + tests) | M | 📋 Program-level |
| 21 | Run the test suite on a virtualised target (QEMU / Renode / Avocado) | L | 📋 Program-level |
| 22 | Refresh to the latest COVESA VSS release (v4.0 → v6.0) | XL | 🔄 In progress.  Sub-PRs 1-7 done (rename, DriverSide adoption, cucumber consolidation, canonical bus paths for door_lock + door_handle + window).  Sub-PR 8 (new VSS v6.0 signals worth exposing) reframed broadly — see item **25** below.  Slice **8a** (transmission canonical rename) in PR #59. |
| 25 | Bridge as VSS publisher for domains without separate POSIX services | L | ⏳ In progress.  Reframing of #22 sub-PR 8 after architectural discussion.  In a production deployment, signals come from multiple ECU services (engine, BMS, telematics, …).  Where the platform has no separate publisher service, this bridge takes responsibility.  Detail section below. |
| 26 | Body-platform OBD / DTC service | M | 📋 Future.  Eventually the bridge needs a body-platform OBD-II / DTC surface — diagnostic trouble codes, freeze frames, the standard `Vehicle.OBD.*` subtree.  Tied to having an ASIL-B fault store (likely lives on the M7 in the final platform).  Not started; tracking here so it isn't lost. |

Suggested next order from what genuinely remains: **22 → 25 → 14 (remainder) → 26 (later)**.
The "every key search is feature-driven" cleanup is complete with
#24 — KeyLostWarning (#17), SmartTrunkPop (#23), and Welcome's
approach poll (#24) all own their own scans, and `KeySearchArbiter`
is purely a request serialiser.  Item #14 (stop continuous Zone
publishing) closed out organically.

# 25. Bridge as VSS publisher for non-body domains

In a real production deployment the VSS broker consumes signals from
many publisher services — engine ECU → RPM, BMS → State-of-Charge,
HVAC controller → vent state, telematics modem → GPS, and so on.
On this platform we are deliberately **not** assuming separate
POSIX services per domain.  Where no such service exists, this
bridge is the only thing on the seam between the broker and the
buses (CAN / LIN / Ethernet); it owns the publish responsibility.

## In scope (bridge will publish)

| Slice | Domain | Signals (rough) | Status |
|---|---|---|---|
| 8a | Powertrain — Transmission rename | `Vehicle.Powertrain.Transmission.*` (canonical naming for the 3 existing signals) | In PR #59 |
| 8b | Driver | `Vehicle.Driver.IdentifierType`, `.Identifier.Subject` (sourced from `LockStatus.LastRequestor` + the active fob/phone slot) | Next |
| 8c | Powertrain — engine + fuel stub | `Vehicle.Powertrain.CombustionEngine.Speed` (RPM), `.FuelSystem.RelativeLevel` / `.Range` / `.IsEngineFuelLevelLow` | Planned |
| 8d | Vehicle motion + ambient | `Vehicle.Speed` (with an actual source), `.IsMoving`, `.AverageSpeed`, `.TraveledDistance`, `.AmbientAirTemperature` | Planned |
| 8e | HVAC plant | The existing 10 HVAC signals get a plant model that simulates the Classic AUTOSAR HVAC algorithm.  Bridge already exposes the user-input side (SetTemperature, FanSpeed, Mode, A/C On/Off, Defrost, Recirculation); plant adds actuator outputs (resolved blower speed, compressor active, vent flap positions, cabin-actual temperature) plus a small thermodynamic model | Planned |

## Out of scope (separate POSIX services own these)

| Domain | Owner | Why not the bridge |
|---|---|---|
| ADAS (`Vehicle.ADAS.*`) | dedicated ADAS service | Cameras + radar + perception stack — not a body-controller concern |
| Telematics (`Vehicle.CurrentLocation.*`, NAV) | telematics modem service | GPS + cellular owned by the modem |
| Connectivity (`Vehicle.Connectivity.*`) | NetworkManager / modem service | WiFi / cellular state belongs at the platform networking layer |

The single existing `Vehicle.ADAS.HighBeam.OncomingVehicleDetected`
signal stays (it's an HMI debug input feeding the AutoHighBeam
feature — not a stub publisher).  We won't expand the ADAS surface.

# 26. Body-platform OBD / DTC service

The body controller eventually needs a diagnostic-trouble-code
surface — the standard `Vehicle.OBD.*` subtree, fault freeze frames,
maybe a J1979 PID mapping.  Tied to having an ASIL-B fault store
(likely living on the M7 in the final platform; on this bridge it
would be a Rust plant model that mirrors that).  Not started.
Tracking here so it doesn't get lost.

---

## 13. Migrate ExteriorTrunkButton onto the KeySearch arbiter  *(M)* — ✅ Done

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

## 14. Stop continuous Zone publishing; introduce `PlacedZone` + `LastObservedZone`  *(L)* — ✅ Done

> Closed out by audit after PR #51 landed.  See the status row at
> the top of this doc.  No subscribers to the legacy continuous
> `Vehicle.Simulation.KeyFob.{N}.Zone` / `Body.PEPS.Plant.KeyFob.{N}.Zone`
> remain in `vss-bridge/src/`, `signal_ids.rs`, `hmi_alias.rs`,
> `ws_bridge.rs`, or the HMI.  The migration completed organically
> as #17, #23, and #24 each split their consumers off the
> continuous-Zone path; what's preserved below is the original
> plan for the historical record.

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

## 14d. Lost-paired-key scan + cluster warning  *(S)* — ✅ Done (merged into #17)

> **History.**  This was originally implemented as a standalone
> `features/lost_pk_scan.rs` that fired on the door-close edge with
> ignition live, ran a `Sequence(AllApproach + TrunkInside + Cabin)`
> Presence scan, and published
> `Vehicle.Controller.Body.PEPS.LostKeyWarning` on zero results.
>
> During the #17 (key-lost warning chime) implementation we realised
> the two features were addressing the same scenario — driver in a
> running car with no paired key on board — and the original `lost_pk_scan`
> shape had three issues:
>
> 1. The Sequence scan included `TrunkInside`, but a key in the trunk
>    is the *key-locked-in-trunk* case (now backlog item #23), not
>    the same problem.
> 2. The Sequence scan used `Presence`, not `Authenticated` — so an
>    intruder fob / mechanically-compatible blank in the cabin would
>    suppress the warning.
> 3. The "door close" trigger alone missed two real cases: the user
>    closing up before turning the key (no door edge after ignition-
>    on), and a fob leaving the cabin without any door edge (window,
>    dead phone, etc.).
>
> The merged successor is `features/key_lost_warning.rs`: same
> `LostKeyWarning` signal on the wire (no HMI churn), correct
> Cabin / Authenticated scan, and three trigger sources (last-close
> edge, ignition-on while sealed, 1-minute periodic).  See #17 below
> for the landed shape.

**Built on (in `main`).**  Per-door `IsOpen` signals,
`Vehicle.LowVoltageSystemState`, KeySearchArbiter, the
`Vehicle.Controller.Body.Chime.IsActive` direct-publish path.

---

## 15. Smart Cabin Unlock (key-locked-in-cabin)  *(M)* — ✅ Done

> **Note.**  This item was originally written up as the
> *key-locked-in-trunk* case (trunk-pop on detection).  The
> implementation that landed under `features/smart_unlock.rs` solves
> a sibling problem — *key-locked-in-cabin* (UnlockAll on
> detection).  Both flows make sense; the cabin case was built first
> and got the `SmartUnlock` name.  The trunk-pop case is now item
> **23** below.

**Trigger.**  Real OEM convenience feature: when the user locks the
vehicle from outside (RKE / PEPS / keypad / phone / NFC) while
ignition is quiescent, run a cabin-presence check.  If a paired
fob is detected in the cabin **and** nothing is detected outside,
the vehicle silently un-locks itself with a `mislock` audible cue
+ the standard unlock flash pattern.  Prevents the classic "locked
my keys in the car" failure mode.

**Built on (in `main`).**  KeySearchArbiter with `AntennaSet::Cabin`
+ `AntennaSet::AllApproach`, the standard door-lock arbiter,
`LockStatus`/`LastRequestor`/`EventNum` tuple from the existing
arbiter.

**Plan.**
1. `FeatureId::SmartUnlock = 0x2B`.
2. Subscribe to `Cabin.LockStatus`, `Cabin.LockStatus.LastRequestor`,
   `Cabin.LockStatus.EventNum`.
3. On a fresh lock event whose `LastRequestor` is an external
   source (`KeyfobRke`, `KeyfobPeps`, `KeypadLock`, `PhoneApp`,
   `PhoneBle`, `NfcCard`, `NfcPhone`) **and** ignition ∈ {OFF, ACC,
   LOCK}:
   - Submit `Sequence(Cabin + AllApproach, Authenticated)`.
   - If key in Cabin **and** no key in any outside zone: publish
     `FeedbackRequest = "mislock"` and dispatch `UnlockAll` via the
     door-lock arbiter as `FeatureId::SmartUnlock`.
4. PEPS-only — KeyCylinder builds disable the feature (the legacy
   key cylinder is the primary auth and a fob-in-cabin doesn't
   imply the same failure mode).
5. Tests cover: trigger condition with key in cabin; suppression
   when a key is also outside; no-fire on KeyCylinder builds;
   no-fire on internal-source lock; ignition non-quiescent
   suppression.

**Risks.**  False positives if the driver legitimately locked a
phone or fob inside and then re-locked from the keypad.  The
`mislock` audible cue is distinct from the normal lock chirp, so
the user knows the lock didn't take.

---

## 16. NFC Entry feature  *(M)* — ✅ Done

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

## 17. Key-lost warning chime  *(XS)* — 🔄 In PR #47

> **Scope drifted during review.**  The original plan (mid-drive
> speed-gated `ApproachKeys` edge) was wrong on two counts:
> (a) `ApproachKeys` isn't continuously updated — it's published
> by the arbiter after triggered or periodic scans, so a "drop to
> zero" edge wasn't a coherent triggering signal; and (b) the
> driver-error that's actually worth catching is *getting in a
> running car with no key on board*, not *losing a key mid-drive*.
> Item #14d (LostPkScan) already addressed (b) for the door-close
> case, so #17 absorbed it.  See the in-PR description for the
> landed shape.

**Final trigger.**  Submit a `Cabin / Authenticated / Disallowed-
coalesce` KeySearchArbiter request when:

1. `Vehicle.LowVoltageSystemState` is `ON` or `START`, AND
2. Every Row1 / Row2 door and the rear trunk is closed.

The scan is submitted on:
- The closing edge that completes the all-sealed state.
- The ignition-on edge when the cabin is already sealed.
- A 1-minute periodic tick while ignition is on.

**Action.**  When the scan returns empty AND gating still holds AND
no warning is latched: publish `Vehicle.Controller.Body.PEPS.
LostKeyWarning = true` (the same signal LostPkScan used to publish —
no HMI churn) and claim `Vehicle.Controller.Body.Chime.IsActive` for
2 s.  Auto-clear of the chime + flag at 2 s; latch held so periodic
ticks don't re-chime every minute.  Latch clears on: a scan finding
a paired key, any door / trunk opening, ignition off.

**Built on.**  `KeySearchArbiterHandle::submit` (#13 / item 14 /
SmartUnlock established the per-feature scan-request pattern);
chime path (`Vehicle.Controller.Body.Chime.IsActive`) shared with
LockFeedback + PerimeterAlarm; no arbiter today.

**Deletes.**  `features/lost_pk_scan.rs` (its successor); the
`Vehicle.Controller.Starting.KeyLostWarning` interim signal added
mid-review.

**Risks.**  Chime collision — multiple features publish the shared
chime signal directly with no arbitration.  Mitigated by short
duration (2 s) and the rarity of overlapping cases.

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

**Cost framing — when does this pencil out?**

The decision lives or dies on four cost categories.  Each is
order-of-magnitude reasoned (loaded senior-embedded eng cost
~US$250–350k/yr, US/EU rates; numbers shift in EM markets but the
ratios are stable).

*1. Per-vehicle / per-program licensing (recurring).*  The clear win.

| | Classic AUTOSAR | FreeRTOS + SafeRTOS |
|---|---|---|
| BSW royalty | $0.50–$2 per ECU (Vector, EB, Elektrobit) | $0 |
| RTOS license | bundled | SafeRTOS one-time: ~$50–$200k per product family (WHIS) |
| Tooling seats | DaVinci Configurator, Vector tools: $30–$100k/dev/yr | nil (open toolchain) |

For a body ECU at 100k vehicles/year/program, BSW royalty alone is
**$100–200k/year per program, forever**.  This is the number we put
in front of OEM procurement — it justifies the Platform Provider
licensing model standalone.

*2. Safety qualification (one-time, but the biggest hidden cost).*
The math gets uncomfortable here.  Classic AUTOSAR BSW is
pre-qualified ASIL-B/D and you inherit the certificate — that is
literally what the BSW vendor sells.  SafeRTOS gives you a
pre-certified kernel (SIL3 / ASIL-D) but **everything around the
kernel — CAN/LIN drivers, NVM driver, watchdog manager, application
SWCs — is unqualified**.  We own the full safety case.

Rough first-program safety-case budget (ASIL-B body controller,
greenfield):

| Item | Senior heads × duration | Rough $ |
|---|---|---|
| DIA / safety plan / HSI | 2–3 × 6 mo | $500–900k |
| HARA + safety concept | 1–2 × 4–6 mo | $200–400k |
| HSI/HRA + V&V + TÜV assessment | 2–3 × 6 mo + cert fees | $300–700k |
| Per-subsequent-program delta (HW or major SW change) | | $100–300k |

**First-program safety case: $1–2M extra vs. inheriting AUTOSAR's.**
Ferrocene's qualified rustc removes the toolchain side of the case
(huge), but the driver-layer case remains.

*3. One-time engineering build (HAL + drivers).*  Classic AUTOSAR
MCAL + RTE eliminates ~30–40% of platform plumbing.  Hand-rolling,
ballpark per driver class:

- CAN/LIN drivers + frame timing: 3–6 senior-eng-months
- NVM driver (atomic + wear-leveling): 2–3 months
- RPmsg / IPC adapter: 1–2 months
- Watchdog / clock / power-mode management: 2–3 months
- Reference Safety Monitor + SWC scaffolding in Rust: 4–6 months

Net additional one-time engineering: **$600k–$1.5M**.  Amortized
across every program after the first.

*4. Talent + hiring (ongoing).*  Counter-intuitively leans toward
FreeRTOS/Rust.  Classic AUTOSAR + DaVinci expertise is scarce,
brutal turnover, ~30–50% hiring premium over generic embedded.
FreeRTOS + Rust + embedded safety is also scarce but the pool is
growing fast and the work attracts higher-quality engineers
(Ferrocene-Rust stack is recruiting bait).

**Total bet for the service company.**  Year 1–2 build phase
absorbs roughly **$2.5–4M** extra vs. starting on Classic AUTOSAR
(Δ safety case + Δ engineering).  From Year 3+ each OEM program
carries a structural cost advantage of $100–200k/yr royalty +
$30–100k/dev/yr tooling savings, which we either share with the
OEM or capture as margin.  At a steady state of 3–4 active programs
we recover the $2.5–4M within **18–30 months**.

**Strategic moat.**  Classic AUTOSAR makes us another Vector / EB
integrator competing on procurement relationships.  FreeRTOS /
SafeRTOS + Ferrocene Rust + open VSS makes us a differentiated
stack — same direction Tesla, Rivian, parts of BMW and JLR are
already moving in.  OEM senior management increasingly *expects*
this answer.

**When the math does NOT work.**
- ≤1 OEM program over 3 years: $2.5–4M won't amortize.  Stay on
  Classic AUTOSAR.
- Risk-averse procurement-led OEM as first customer: they will
  discount SafeRTOS + Rust on principle.  Need an early partner
  who values the differentiation enough to co-fund the safety case.
- No in-house ASIL safety engineering: start there before the RTOS
  decision.  The safety case is the cost; the kernel choice is
  downstream.

**Decision posture (service company view).**  Architect the M7 SWC
layer RTOS-portable above the SignalBus / IPC seam from day one.
Build the Classic AUTOSAR reference first to win procurement
credibility, but keep the FreeRTOS + SafeRTOS + Ferrocene track on
the roadmap as the differentiation lever for the second OEM program
(co-funding terms negotiable).  Do not bet the company on FreeRTOS
adoption before the first OEM signs.

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

## 22. Refresh to the latest COVESA VSS release  *(L → XL after scope review)*

> **Design + path mapping artifact lives at
> [`docs/vss-v6.0-migration.md`](./vss-v6.0-migration.md) and
> [`docs/vss-v6.0-path-mapping.csv`](./vss-v6.0-path-mapping.csv).**
> The plan below has been superseded — see the design doc for the
> 8-sub-PR breakdown.  Sub-PR 1 (this artifact) is **complete**.
> Original size estimate (`M`) revised upward to `L`/`XL` after
> discovering the project's path convention is non-canonical (`Body.*`
> top-level) and adopting v6.0 requires a `Vehicle.*` rebase plus
> `DriverSide` / `PassengerSide` semantics that eliminate the
> `dealer.driver_door_side` cal.

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

## 23. Smart Trunk Pop (key-locked-in-trunk)  *(M)*

The sibling case to #15.  When the user locks the vehicle from
outside (RKE / PEPS / keypad / phone / NFC) while ignition is
quiescent, run a **TrunkInside** authenticated scan; if a paired
key is detected in the trunk, **pop the trunk** (not unlock the
doors) and chirp + flash to draw the user's attention.

**Why a separate feature from SmartUnlock (#15).**  The remediation
is different — popping the trunk leaves the doors locked, which
is the correct response for "you put a paired phone in the grocery
bag that's now in the trunk."  Unlocking everything (SmartUnlock)
would invite a passerby to walk into the cabin.  SmartUnlock
explicitly excludes TrunkInside coverage and has a regression test
asserting the trunk case is a no-op for it (`smart_unlock.rs`
line 625, `key_in_trunk_inside_alone_does_not_unlock`).

**Built on (in `main`).**
- `KeySearchArbiter` with `AntennaSet::TrunkInside` already defined.
- Trunk arbiter accepts `TRUNK_OPEN_CMD` claims; ExteriorTrunkButton
  already uses the same path for its locked-cabin auth flow
  (`features/exterior_trunk_button.rs:191`).
- `LockStatus` / `LastRequestor` / `EventNum` tuple, same as #15.
- Chime + hazard arbiters already accept claims.

**Plan.**
1. New `FeatureId::SmartTrunkPop` (next free, ~0x2D).
2. New `features/smart_trunk_pop.rs`.  Constructor takes
   `KeySearchArbiterHandle`, the trunk arbiter, and `PlatformConfig`
   (for the PEPS-vs-KeyCylinder gate).
3. Subscribe to `Cabin.LockStatus`, `Cabin.LockStatus.LastRequestor`,
   `Cabin.LockStatus.EventNum`, `Vehicle.LowVoltageSystemState`,
   `Vehicle.Body.Trunk.Rear.IsOpen`.
4. On a fresh lock event with `LastRequestor` in the same
   external-source list as SmartUnlock and ignition ∈ {OFF, ACC, LOCK}:
   - Skip if trunk is already open (someone's at it).
   - Submit `AntennaSet::TrunkInside, SearchMode::Authenticated,
     Coalescing::Disallowed` to the KeySearchArbiter.
   - On non-empty: pulse `TRUNK_OPEN_CMD` via the trunk arbiter,
     same momentary edge ExteriorTrunkButton uses; publish
     `FeedbackRequest = "mislock_trunk"` (new variant) for the
     audible cue; claim hazard 3-flash via the lighting arbiter.
   - On empty: no-op (the cabin case will be picked up by
     SmartUnlock if applicable).
5. PEPS-only — disabled on KeyCylinder builds.
6. Dealer cal `dealer.smart_trunk_pop_enabled` (default `true`);
   skip the whole flow when false.
7. Tests:
   - Paired key in `TrunkInside` + external lock → trunk pops,
     doors stay locked.
   - Paired key in cabin + trunk → trunk pops; cabin unlock is
     SmartUnlock's call.
   - No paired key in trunk → no-op.
   - Trunk already open at lock time → no-op.
   - KeyCylinder build → no-op.
   - Internal-source lock (`DoorTrimButton`, `WalkAwayLock`) → no-op.

**Risks.**
- *Latency window.*  The trunk pop happens after a real arbiter
  scan (~100 ms LF airtime) plus arbiter request serialization.
  The user's hand could already be off the lock button by the time
  the trunk lid releases — the audible cue is essential to make the
  cause-and-effect clear.
- *Phantom-fob trigger.*  An intruder fob (mechanically compatible
  blank) won't trigger because the scan is `Authenticated` — the
  HMAC challenge filters unpaired fobs out.
- *Auto-relock interaction.*  After the trunk-pop, the doors are
  still locked.  If AutoRelock is enabled and the trunk closes,
  the cycle could repeat; the trunk arbiter's pulse semantics +
  the `trunk_already_open` skip should prevent this, but worth a
  scenario-level test.

**Signals introduced.**
- `Vehicle.Controller.Body.Trunk.SmartPop.LastEvent` (String,
  optional — for telemetry / fault logging).
- `dealer.smart_trunk_pop_enabled` (Bool, default true).
- `FeedbackRequest = "mislock_trunk"` (extends the existing
  `FeedbackRequest` String enum — LockFeedback can play a distinct
  chime sequence to distinguish from `"mislock"`).

---

## 24. Welcome owns the approach poll  *(M)* — ✅ Done (PR #51)

The final step of the "every key search is feature-driven"
principle that #17 (KeyLostWarning) and #23 (SmartTrunkPop)
already applied to their respective scans.  Eliminates the
`KeySearchArbiter`'s internal periodic poll — the arbiter
becomes purely a serialiser of feature-submitted requests.

### Why move it

Today `KeySearchArbiter::run` carries an adaptive periodic
`AllApproach / Presence` poll (700 ms fast when no key in
approach; 10 s slow once a key is detected; suspended while
ignition is in `ACC` / `ON` / `START`).  See
[`key-search-arbiter-and-ignition.md`](./key-search-arbiter-and-ignition.md) §3.3.

The arbiter doesn't *need* the result of that poll for anything
itself.  It publishes `ApproachState` / `ApproachKeys` /
`ApproachPollInterval` as side-effects, and the actual consumer
is the `Welcome` feature (via per-fob `LastObservedZone` signals
that the same poll updates).  Every other PEPS-aware feature
already submits its own scans on demand:

| Feature | Scan trigger |
|---|---|
| PassiveEntry | Outside handle pull edge |
| KeypadLock | Keypad debounce-complete |
| VehicleStartingControl | Brake-press pre-auth + start-button press |
| ExteriorTrunkButton | Trunk-button press while locked |
| SmartUnlock | External-source lock event qualifying |
| SmartTrunkPop | External-source lock event + trunk-close edge |
| KeyLostWarning | Cabin-sealed-under-power + periodic 1 min |
| **Welcome** | **periodic (NEW — currently free-rides the arbiter's poll)** |

Moving the poll into Welcome:

1. Removes the only time-driven scan that isn't tied to a
   feature event.  Arbiter becomes purely request-driven.
2. Lets Welcome adapt the cadence based on its own state (e.g.
   pause once the courtesy lights are latched, not just on
   ignition).
3. Makes the architectural rule uniform across the codebase —
   easier to reason about, easier to teach.

### Built on (in `main`)

- `KeySearchArbiterHandle::submit` — already exposes everything
  Welcome needs.  `Coalescing::Allowed` is the right policy for
  a periodic poll (a concurrent burst should coalesce).
- The PEPS plant's `LastObservedZone` publish path runs after
  every arbiter scan regardless of who submitted, so Welcome's
  per-fob subscriptions don't change.
- The arbiter's `ApproachState` / `ApproachKeys` /
  `ApproachPollInterval` publishes: Welcome takes over publishing
  these from the result of its own scan, same field semantics.

### Plan

1. **New cadence state in Welcome.**  `fast_cadence` /
   `slow_cadence` move from `KeySearchArbiter` to `Welcome` (with
   the same `APPROACH_POLL_FAST` / `APPROACH_POLL_SLOW` constants
   re-exported from Welcome's module).  `Welcome::with_cadence`
   replaces `KeySearchArbiter::with_cadence`.
2. **Periodic submit loop.**  Welcome's run loop grows a periodic
   tick that submits `AntennaSet::AllApproach` / `Presence` /
   `Coalescing::Allowed`.  The interval starts at `fast_cadence`
   and flips to `slow_cadence` once the result reports
   ≥ 1 approach key.  Same suspension logic as today: paused
   while ignition is in `ACC` / `ON` / `START`.
3. **Welcome publishes the aggregate signals.**
   `Vehicle.Controller.Body.PEPS.ApproachState`,
   `.ApproachKeys`, and `.ApproachPollInterval` are published by
   Welcome after each poll instead of by the arbiter.  Signal
   names + types unchanged — HMI consumers are unaffected.
4. **Delete the arbiter's poll loop.**  `KeySearchArbiter::run`
   loses its `poll_deadline` arm, its `ApproachState` /
   `ApproachKeys` / `ApproachPollInterval` writes, and its
   `fast_cadence` / `slow_cadence` / `IGNITION_STATE_SIGNAL`
   subscriptions.  The struct shrinks to just the request mpsc
   receiver and the per-fob zone / paired caches that incoming
   requests need.  `KeySearchArbiter::with_cadence` disappears.
5. **Reuse the existing `LastObservedZone` publish path.**  Every
   feature-submitted scan already updates per-fob
   `LastObservedZone`s in `publish_last_observed`; this stays
   exactly as is, since Welcome's poll is just another submitter.
6. **Tests.**  KeySearchArbiter tests that exercised the
   periodic-poll cadence flip move to Welcome's test module.
   The arbiter's remaining tests (request-submission path,
   coalescing window, priority queue) are unchanged.

### Risks

- **Cadence migration must be careful about lazy starts.**  Today
  the arbiter starts polling immediately on boot.  Welcome
  doesn't currently boot the same way — its run loop blocks on
  the zone streams.  The first periodic tick in Welcome must
  fire promptly (within `fast_cadence`) so ApproachState
  doesn't lag behind the legacy timing.
- **Test parity.**  `key_search_arbiter::tests` has several
  scenarios that exercise the poll cadence flip; they need to be
  re-rooted on a setup that spawns Welcome alongside the arbiter.
  The PEPS plant model's mock-bus + virtual-time scaffolding is
  shared, so the rewrite is mechanical.
- **HMI signal continuity.**  The HMI's PEPS Devices card
  subscribes to `ApproachKeys` and `ApproachPollInterval` for
  rendering.  As long as Welcome publishes them with the same
  cadence-flip semantics, the card behaves identically.  Add a
  manual smoke test (drag a fob into approach, watch the badge
  light + the interval flip) to the PR's test plan.

### Out of scope for this item

- Coalescing the arbiter's per-fob `paired` / `LastObservedZone`
  caches with a more principled design — that's a follow-up if
  the arbiter's remaining footprint feels misshapen after the
  poll loop leaves.
- Splitting `Welcome` itself into "approach detection" +
  "courtesy lighting" sub-features.  Conceptually clean, but
  the two are tightly coupled today and the existing tests treat
  them as one feature — defer.

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
