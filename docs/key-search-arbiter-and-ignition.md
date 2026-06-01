# KeySearch Arbiter, Vehicle Starting Control, Smart Unlock — Design

Status: design + implementation in progress on `feature/key-search-arbiter-and-ignition`.
This document is the source of truth for the multi-phase tear-up of the PEPS
subsystem, the new ignition control feature, the brake plant, and the Smart
Unlock feature.  Each phase ends in a self-contained commit; each commit
should build and pass the full test suite.

---

## 1. Motivation

Today the PEPS plant continuously publishes per-fob `Zone` signals on a
free-running scan.  Every feature that cares about key proximity
subscribes to those signals directly.  This has three problems:

1. **It's not how real cars work.**  Real PEPS controllers fire LF
   antennas in carefully scheduled timeslots, on demand, and pay a
   measurable energy cost per scan (vehicle 12V battery + fob coin
   cell).  Always-on scanning is a poor abstraction.
2. **There's no single owner for LF airtime.**  Multiple features
   would want to fire LF challenges at the same time on a real ECU,
   and the antennas can't be shared.  Today's "everyone subscribes
   to the zone signal" model doesn't expose that contention surface.
3. **Approach polling cadence isn't expressible.**  Real PEPS
   controllers scan briskly (~700 ms) when no key is nearby and back
   off (~10 s) once a key is detected, to save power.  The current
   continuous-zone model has no way to model this.

The redesign introduces a **KeySearch arbiter** that owns LF airtime
and serializes search requests from features.  Features stop
subscribing to per-fob zone signals and instead issue an explicit
search request when they need to know.  Each feature drives its own
scans tied to a specific physical event it owns (a button, an edge,
a periodic check); the arbiter does no time-driven scanning of its
own.

> The original draft of this doc had the arbiter run an adaptive
> approach poll on its own as a fourth "feature."  Subsequent work
> (KeyLostWarning #17, SmartTrunkPop #23) made it clear that
> putting a scan in the arbiter is just the wrong layer — the
> arbiter has no decision to make with the result.  The poll moved
> into the `Welcome` feature in PR #51 (backlog #24); the arbiter
> is now purely a request serialiser.  §3.3 below describes the
> poll as it lives today in `features::welcome`.

---

## 2. Search semantics

### 2.1 Antenna sets

The vehicle has several LF antenna installations:

| Antenna | Coverage zone(s) | Purpose |
|---|---|---|
| Cabin antennas (2-3 of them) | `Cabin` | Cabin-presence — driver-in-seat with key |
| Door-handle antennas | `Approach`, `LeftFront`, `RightFront`, `Hood`, `Trunk` | Approach detection + per-handle passive entry |
| Trunk-outside antenna | `Trunk` | Trunk-exterior-button authentication |
| Trunk-inside antenna | `TrunkInside` | Smart Unlock: key-locked-in-trunk detection |
| Cylinder / registry antenna | `KeyCylinder` | Start-button or backup auth |

The new `AntennaSet` enum lets a feature ask for a specific subset:

```rust
pub enum AntennaSet {
    Cabin,
    Cylinder,
    AllApproach,             // all corner / handle / hood / trunk-outside antennas
    SingleHandle(DoorRef),
    TrunkOutside,
    TrunkInside,
    Sequence(Vec<(AntennaSet, SearchMode)>),   // chained scans for compound requests
}
```

### 2.2 Search modes

```rust
pub enum SearchMode {
    /// LF "ping" — fobs respond with their slot ID only.  No HMAC
    /// challenge / response.  ~50 ms.  Negligible battery cost.
    Presence,

    /// Full LF challenge → RF response → HMAC verify.  100–150 ms.
    /// Used wherever the vehicle has to act on a key's presence
    /// (unlock, start, trunk pop).
    Authenticated,
}
```

### 2.3 Simulated latencies

| Antenna set | Presence | Authenticated |
|---|---|---|
| `AllApproach` | 50 ms | 150 ms |
| `Cabin` | – (not polled in Presence mode) | 100 ms |
| `Cylinder` | – | 50 ms |
| `SingleHandle` | – | 100 ms |
| `TrunkOutside` | – | 100 ms |
| `TrunkInside` | – | 100 ms |

Sequences add segment latencies.  Each search has a timeout equal to
its latency + 50 ms; on timeout the search returns "no key found".

---

## 3. The KeySearch arbiter

### 3.1 Request API

```rust
pub struct KeySearchRequest {
    pub requester: FeatureId,
    pub antennas: AntennaSet,
    pub mode: SearchMode,
    pub coalescing: Coalescing,         // can this share another in-flight result?
    pub response: oneshot::Sender<KeySearchResult>,
}

pub enum Coalescing {
    /// Re-use any in-flight or just-completed (<50 ms ago) result
    /// for the same (antennas, mode).
    Allowed,
    /// Always run a fresh scan.  Used for security-critical paths
    /// (handle pull, start press) where a stale result is unsafe.
    Disallowed,
}

pub struct KeySearchResult {
    pub keys_found: Vec<KeyFinding>,
    pub antennas_fired: AntennaSet,
    pub mode: SearchMode,
    pub took: Duration,
}

pub struct KeyFinding {
    pub slot: KeySlot,
    pub zone: Zone,
    pub rssi: i8,
}
```

### 3.2 Scheduling

- **One LF scan at a time.**  Real antennas can't share airtime cleanly.
- **Priority classes** (request gets queued in its class, FIFO within):
  - Critical: Vehicle Starting Control's start-button request
  - High: Passive Entry, Keypad Lock, Exterior Trunk Button,
    Smart Unlock — all event-driven authenticated searches
  - Normal: Approach poll (arbiter's own internal driver)
- Higher classes don't preempt mid-scan, but they jump the queue.
- **Coalescing window: 50 ms.**  Two `Coalescing::Allowed` requests
  for the same `(antennas, mode)` within 50 ms get one underlying
  scan and both `oneshot::Sender`s receive the same result.

### 3.3 Approach poll loop (now owned by `Welcome`)

> **Architectural correction landed in PR #51 (backlog #24).**
> The periodic approach poll described below originally lived in
> `KeySearchArbiter` as an internal task.  It now lives in the
> [`Welcome` feature](../vss-bridge/src/features/welcome.rs) —
> Welcome is the only consumer of the poll's result, and the rule
> the rest of the codebase landed on (KeyLostWarning #17,
> SmartTrunkPop #23) is that *every key search is triggered by the
> feature that needs the result*, with the arbiter acting purely as
> a serialiser of LF airtime.  Cadences, suspension semantics, and
> published-signal contract are identical to the legacy shape —
> only the writer changed.

Welcome runs the periodic poll as one arm of its `select!` loop
(the last, biased so courtesy-lighting decisions take precedence):

```text
poll @ 700 ms when ApproachState == false
poll @  10 s when ApproachState == true
suspended when ign in ACC/ON/START   (driving — fob is in cabin anyway)
```

The poll submits `AntennaSet::AllApproach` in `SearchMode::Presence`
(cheap, no HMAC) with `Coalescing::Allowed` (concurrent in-flight
approach scans can share the result).  From the response Welcome
derives and publishes:

- `Vehicle.Controller.Body.PEPS.ApproachState` (Bool — any paired key in approach)
- `Vehicle.Controller.Body.PEPS.ApproachKeys` (Uint8 count)
- `Vehicle.Controller.Body.PEPS.ApproachPollInterval` (Uint16 ms — current cadence)

Suspension sets all three to their "paused" markers (`false`, `0`,
`0`) so the HMI's PEPS-status indicator reads correctly during
driving without needing its own ignition subscription.  The
cadence-flip + suspension tests live alongside Welcome's other
tests in [`features/welcome.rs`](../vss-bridge/src/features/welcome.rs).

---

## 4. PEPS plant — the new API

The PEPS plant becomes a passive "answer questions about fob positions"
service:

```rust
async fn run_search(&self, antennas: AntennaSet, mode: SearchMode)
    -> KeySearchResult
```

Internally:

- Plant still tracks each fob's HMI-set position (drag target).
- A separate `PlacedZone` signal per fob is published instantly on
  HMI drag — purely for visualization (`Body.PEPS.Plant.KeyFob.N.PlacedZone`).
- `run_search` simulates the antenna firing (with `tokio::time::sleep`
  for the latency), determines which fobs are in coverage, and for
  `Authenticated` mode runs the existing HMAC challenge/response.
- After each successful search the plant also publishes a per-fob
  `Body.PEPS.Plant.KeyFob.N.LastObservedZone` signal — feedback
  derived from the actual scan, separate from the placed position.

**No more continuous Zone publishing.**  The continuous `Zone` signal
that exists today is removed in phase 9 (after every feature has
migrated).

---

## 5. Vehicle Starting Control

### 5.1 Config

`PlatformConfig` gains `key_source_cfg: KeySource`, where:

```rust
pub enum KeySource {
    Peps,         // default
    KeyCylinder,  // physical rotary cylinder + remote-only fobs
}
```

### 5.2 Signals

HMI inputs (one or the other depending on config):

| Signal | Type | Mode |
|---|---|---|
| `Body.Switches.StartStop.IsPressed` | bool, momentary | PEPS only — dash start button |
| `Body.Switches.IgnitionCylinder.Position` | String enum `LOCK`/`OFF`/`ACC`/`ON`/`START` | KeyCylinder only — rotary, HMI spring-back from `START` |
| `Chassis.Brake.IsApplied` | bool | derived by the new brake plant from `Chassis.Brake.PedalPosition` > 5 |

Feature outputs:

| Signal | Type | Notes |
|---|---|---|
| `Vehicle.LowVoltageSystemState` | String enum | **ownership moves from HMI to feature** |
| `Vehicle.Starting.ImmobilizerStatus` | String enum `UNVERIFIED`/`VERIFIED`/`FAILED` | for HMI debug + cluster lamp |

### 5.3 State machine — PEPS mode

Inputs: `StartStop.IsPressed` (rising edge), `Brake.IsApplied`,
KeySearch result from `Sequence([Cylinder, Cabin], Authenticated)`.

| Current state | Press w/o brake | Press w/ brake | No key found |
|---|---|---|---|
| LOCK / OFF | → ACC | → ON (skip ACC) | stay OFF, set `ImmobilizerStatus = FAILED` |
| ACC | → ON | → ON | stay ACC |
| ON | → OFF | → OFF | – (state change requires no key check; ON→OFF is unconditional) |

### 5.4 State machine — KeyCylinder mode

Input: `IgnitionCylinder.Position`.

| Cylinder transition | Action |
|---|---|
| any → `LOCK` / `OFF` | publish the new state directly |
| → `ACC` from `LOCK`/`OFF` | publish `ACC` (no auth — accessory only) |
| → `ON` from any | request `Cylinder` authenticated search; on `VERIFIED` publish `ON`, on `FAILED` stay at previous state |
| → `START` from `ON` | request `Cylinder` authenticated search; on `VERIFIED` briefly publish `START`, then `ON` after 800 ms; on `FAILED` stay at `ON` |

### 5.5 KeyCylinder zone semantics

New `Zone::KeyCylinder` variant in `peps::zone`.  Range very short
(~5 cm).  Available in both vehicle-line configs.

| Mode | Cabin / Approach / corner zones | KeyCylinder zone |
|---|---|---|
| `PEPS` | active — PEPS plant detects fobs | active — backup auth (low-battery fob) |
| `KeyCylinder` | **dead** — PEPS plant suppresses these zones | active — sole start-auth path |

In `KeyCylinder` mode, fobs degrade to **remote-only** — RKE button
presses still work via the RF channel (independent of LF antennas).

---

## 6. Brake plant

`plant_models/brake_pedal.rs` — single derived-signal plant.

- Subscribes to `Chassis.Brake.PedalPosition` (existing).
- Threshold: `PedalPosition > 5.0` ⇒ `IsApplied = true`.
- Publishes `Chassis.Brake.IsApplied` (bool).
- Idempotent — only republishes on change.

In production this signal would come from the powertrain or ABS
module, or a hardware switch.  We model it as a stand-alone plant
so the body-controller code subscribing to `IsApplied` matches the
real architecture.

---

## 7. Smart Unlock

### 7.1 Activation gates

All must hold for Smart Unlock to consider firing:

1. Trunk-close edge has just fired.
2. `Vehicle.LowVoltageSystemState ∈ {LOCK, OFF}` — driver isn't using
   the vehicle, not even in accessory mode.
3. `Cabin.LockStatus ∈ {LOCKED, DOUBLE_LOCKED}` — vehicle-level lock
   state (subscribed from `lock_feedback`, not per-door `IsLocked`).
4. `Cabin.LockStatus.LastRequestor` is in the **external-source
   allowlist** (not `DoorTrimButton`).  Includes: `KeyfobRke`,
   `PhoneBle`, `PhoneApp`, `NfcCard`, `NfcPhone`, `KeypadLock`,
   `WalkAwayLock`, `AutoLock`, `PassiveEntry`, `SlamLock`.
5. `dealer.smart_unlock_enabled == true` (new config flag, default true).

### 7.2 Flow

```
on trunk-close edge:
  if state == RescuedThisSession        →  return
  if any gate fails                     →  return

  request Sequence([
    (TrunkInside,  Authenticated),
    (TrunkOutside, Authenticated),
  ], Coalescing::Disallowed)

  if inside.has_any && outside.empty:
    state = WaitingToPop
    sleep 1000 ms                       // user-perceived deliberation
    re-check all gates                  // user may have intervened
    if all still valid:
      issue Body.Trunk.OpenCmd via trunk arbiter (new participant slot)
      chime (single 300 ms beep, Medium priority)
      hazard flash x 3 (standard hazard cadence)
      publish Body.SmartUnlock.LastEvent = "RescuedKeyLockedInTrunk"
      state = RescuedThisSession
    else:
      state = Idle
```

### 7.3 Latch reset

`RescuedThisSession` clears to `Idle` on any of:

- `Cabin.LockStatus` transitions to `UNLOCKED` (any source).
- `Vehicle.LowVoltageSystemState` transitions to `ON` or `START`.
- A fresh trunk-close edge with `inside.empty` (problem self-resolved).

### 7.4 Feedback

- Chime: 300 ms single beep via the chime arbiter at Medium priority.
- Hazards: 3 flashes at standard hazard cadence, via the lighting
  arbiter at a new Medium-priority allow-list slot for `SmartUnlock`.
- `Body.SmartUnlock.LastEvent` (String) — `"RescuedKeyLockedInTrunk"`
  / `"Idle"`.  HMI optional banner.

### 7.5 Worst-case timing

| Step | Duration |
|---|---|
| Trunk latch fires close edge | t = 0 |
| `TrunkInside` search | ~100 ms |
| `TrunkOutside` search | ~100 ms |
| Deliberation delay | 1000 ms |
| `OpenCmd` issued, chime + hazard | t ≈ 1.2 s |

User-perceived gap between "I shut the trunk" and "the trunk re-opens"
is ~1.2 s — long enough to read as deliberate, short enough that the
user is still near the back of the car.

---

## 8. Migration plan — phases

Each phase is a single commit.  Order matters; do not skip ahead.

| # | Phase | Deliverable |
|---|---|---|
| 1 | KeySearch arbiter foundation | New `key_search_arbiter` module + `run_search` API on the PEPS plant + `Presence` / `Authenticated` modes + scheduling + coalescing.  No feature uses it yet.  Unit tests on arbiter scheduling, coalescing, both latencies. |
| 2 | Approach poll loop | *Originally* an arbiter-internal periodic task driving `Body.PEPS.ApproachState` / `ApproachKeys` / `ApproachPollInterval`.  Paused on `ACC`/`ON`/`START`.  Backlog #24 / PR #51 moved the poll into the `Welcome` feature; the signal contract is unchanged.  See §3.3. |
| 3 | Migrate Passive Entry | `SingleHandle + Authenticated` requests on handle-pull edges; drop direct `Zone` signal subscriptions.  Existing passive-entry tests rewritten to seed `PlacedZone` and assert via arbiter mock. |
| 4 | Migrate Keypad Lock | Same pattern as Passive Entry. |
| 5 | Migrate Welcome | Subscribe to `Body.PEPS.ApproachState` rather than zone transitions. |
| 6 | Migrate Walk-Away Lock | Subscribe to `all_doors_closed` + `ApproachState`; no own search request.  Lock fires when both align with ignition in `LOCK/OFF`. |
| 7 | Vehicle Starting Control + brake plant + `Zone::KeyCylinder` + `key_source_cfg` config | New ignition feature, brake plant, KeyCylinder zone added, HMI sim-panel switches between START button / cylinder rotary by config. |
| 8 | Migrate Exterior Trunk Button | `TrunkOutside + Authenticated` request. |
| 9 | Stop continuous Zone publishing | Plant emits `PlacedZone` (HMI drag, instant) + `LastObservedZone` (search result feedback) only.  Continuous `Zone` signal removed. |
| 10 | Smart Unlock | New feature, `dealer.smart_unlock_enabled` config, trunk-arbiter participant, chime + hazard feedback. |

Each phase ends with `cargo fmt && cargo clippy --lib --tests -- -D warnings && cargo test --lib` clean + the existing Playwright smoke suite passing.

---

## 9. Signal additions / removals

### Add

- `Body.PEPS.ApproachState` (bool)
- `Body.PEPS.ApproachKeys` (uint)
- `Body.PEPS.ApproachPollInterval` (uint ms)
- `Body.PEPS.LastSearch.Purpose` (String)
- `Body.PEPS.LastSearch.KeysFound` (uint)
- `Body.PEPS.Plant.KeyFob.N.PlacedZone` (String — HMI drag target)
- `Body.PEPS.Plant.KeyFob.N.LastObservedZone` (String — search feedback)
- `Body.Switches.StartStop.IsPressed` (bool)
- `Body.Switches.IgnitionCylinder.Position` (String enum)
- `Chassis.Brake.IsApplied` (bool)
- `Vehicle.Starting.ImmobilizerStatus` (String enum)
- `Body.SmartUnlock.LastEvent` (String)

### Remove (phase 9)

- Continuous publishing of `Body.PEPS.Plant.KeyFob.N.Zone` from the plant.
  The signal continues to exist for backward compatibility with any
  test harnesses, but the plant no longer drives it.  Features must
  go through the KeySearch arbiter.

### Repurpose

- `Vehicle.LowVoltageSystemState` ownership moves from the HMI sim
  panel (current direct EnumR write) to the `VehicleStartingControl`
  feature.  The sim panel becomes a read-only display + adds the
  key-source-specific input controls.

---

## 10. Test strategy

Per phase:

- Phase 1: arbiter unit tests (scheduling, FIFO within class,
  coalescing, both latencies, timeout returns empty).  Plant
  `run_search` unit tests.
- Phase 2: cadence flip tests (poll @ 700 ms with no key; place a
  key, cadence transitions to 10 s; remove key, back to 700 ms).
  Suspension on ignition `ACC/ON/START`.
- Phases 3–6, 8: behaviour-preserving migration — each feature's
  existing test cases must continue to pass after the rewrite.
  Test harness adjusted to drive `PlacedZone` instead of the old
  `Zone` signal, and to mock the arbiter where needed.
- Phase 7: Vehicle Starting Control state machine — all PEPS-mode
  and KeyCylinder-mode transitions; immobilizer pass/fail paths;
  brake-pressed direct-to-ON; no-key path.
- Phase 9: regression check — every feature still works with the
  new event-driven Zone signals.
- Phase 10: Smart Unlock — every gate independently, condition
  matrix (inside-only / outside-only / both / neither), latch
  prevention, dealer flag disables entirely, deliberation delay
  re-check aborts on intervening unlock.

---

## 11. Open items deferred to later iterations

- **Relay-attack mitigation**.  Real PEPS controllers measure
  round-trip time of the LF/RF exchange to detect a relay attack
  (key being remote-amplified).  We can later tune the simulated
  latency + add an explicit "RTT too long" rejection.  Out of scope
  for this redesign.
- **Anti-pinch on power windows + security override** — earlier
  reserved slots in the window arbiter; the same `KeySearch` pattern
  could be applied to them but not part of this work.
- **Key-lost warning** — when an `ApproachKeys` value drops while
  the vehicle is moving (driver dropped a fob).  Trivial to add
  once `ApproachKeys` exists.
