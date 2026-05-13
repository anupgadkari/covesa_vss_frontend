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
| 15 | Smart Unlock feature (key-locked-in-trunk) | M | ⏳ Pending |
| 16 | NFC Entry feature (card/phone tap → unlock; tap at push-button → start) | M | ⏳ Pending |
| 17 | Key-lost warning chime | XS | ⏳ Pending |

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
today (`walk_away_lock`, `thumb_pad_lock`, `passive_entry`, etc.)
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
   - `thumb_pad_lock` — same.
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
   `WalkAwayLock`, `ThumbPadLock`}:
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
