# PEPS Features — Passive Entry, Passive Lock, Welcome

**Status:** Implemented on branch `feature/peps`.
**Related architecture:**
[`signal-ownership-and-state-hydration.md`](signal-ownership-and-state-hydration.md).

---

## 1. Scope

Three features ride on top of the existing PEPS plant model
(`vss-bridge/src/plant_models/peps/`) and the paired-device crypto
machinery first wired up for RKE:

| Feature | What it does | Trigger | Outputs |
|---|---|---|---|
| **PassiveEntry** | Unlock-on-handle-pull | `Body.Doors.Row*.*.Handle.Outside.IsPulled` FALSE→TRUE while a paired device is in the matching proximity zone | `LockCommand::UnlockDriver` / `UnlockAll` via `DoorLockArbiter`; `FEEDBACK_REQUEST = "unlock"` |
| **ThumbPadLock** (PEPS gate added) | Walk-up lock from outside handle thumb pad — gated on a paired device being **outside the cabin** | 500 ms hold of `LockPad.IsPressed` AND ≥1 paired device in DriverDoor / PassengerDoor / Hood / Trunk / Approach | `LockCommand::LockAll`; `FEEDBACK_REQUEST = "lock"` (or `"lock_denied"` if gate fails) |
| **Welcome** | Courtesy puddle / dome / interior lights on approach | Any paired device transitions from no-LF (OutOfRange/RfRange) into ANY LF zone | `Puddle.Left.IsOn`, `Puddle.Right.IsOn`, `Cabin.Lights.IsDomeOn` via courtesy arbiter |

---

## 2. Authentication flow (PassiveEntry)

```text
HMI / human pulls Row1.Left handle
   │  Body.Doors.Row1.Left.Handle.Outside.IsPulled = true
   ▼
PassiveEntry feature
   │  1. Identify door's proximity zone (Row1.Left → DriverDoor)
   │  2. Pick paired devices currently in that zone
   │     (in-memory device_zones cache, kept fresh by zone-watcher
   │      branch in run() — no per-pull bus subscribe race)
   │  3. Generate fresh 16-byte nonce
   │  4. Publish Body.PEPS.LfChallenge  (fobs respond)
   │      AND  Body.PEPS.BleChallenge   (phones respond)
   ▼
PepsPlantModel (per device, staggered by PRODUCTION_STAGGER_MS × slot)
   │  AES-128 encrypts the nonce with its shared secret
   │  Publishes Body.PEPS.Plant.KeyFob.N.ChallengeResponse
   ▼
PassiveEntry races N response streams against a 150 ms timeout.
First response that verifies via aes_cmac (compute_challenge_response)
against the candidate's shared secret wins.
   │
   ▼
DoorLockArbiter ← UnlockDriver (stage 1) | UnlockAll (stage 2)
   │  also FEEDBACK_REQUEST = "unlock"
   ▼
DoorLockPlantModel actuates the latch motor → IsLocked=false
```

### Two-stage unlock

Same dealer cal as RKE (`dealer.two_stage_unlock`):

- **Enabled (default):**
  1. First successful pull on Row1.Left → `UnlockDriver` (only the
     driver door unlocks).  Latches a `pending_stage_two` deadline.
  2. A second pull from the *same* paired device within 3 s → `UnlockAll`.
- **Disabled:** every successful pull → `UnlockAll`.

Stage-2 from a non-driver door is also accepted within the window —
matches typical OEM UX where the driver unlocks first, walks around
to the passenger door, and pulls that handle to let everyone in.

### Why we publish on both LF and BLE channels

Real PEPS hardware uses physically distinct antennas for fobs (LF,
~134 kHz) and BLE phones (2.4 GHz).  In our simulation a single
challenge nonce is published on both signals because the BCM doesn't
know in advance which device type is approaching.  The plant model's
challenge handlers ignore unrelated devices: `handle_lf_challenge`
iterates fobs, `handle_ble_challenge` iterates phones.

---

## 3. Why a separate **courtesy arbiter**?

The puddle lamps and dome are courtesy outputs that *several*
features want to claim under different conditions:

| Feature | Claim conditions |
|---|---|
| **Welcome** (this branch) | Any paired device enters LF coverage (not yet seated) |
| **Farewell** (future) | Driver opens door after ignition OFF — outside lighting on briefly while the user gets out |
| **PerimeterAlarm** (future) | Intrusion attempt — flash dome + puddle as attention-getter |
| **Follow-Me-Home** (existing) | Headlamps + parking — separate concern, stays on `low_beam_arbiter` |

Putting puddle / dome / future shared courtesy outputs on a
dedicated `courtesy_arbiter` keeps arbitration explicit (allow-list
per feature, priority per claim) and prevents the future
PerimeterAlarm from accidentally winning over Welcome by publishing
the signal directly.  All three current Welcome claims are at
MEDIUM priority so a future security feature at HIGH would override.

The arbiter is registered with the `DomainArbiter` infrastructure
in `arbiter.rs::courtesy_arbiter()` — same machinery as Lighting,
Horn, Comfort, etc.

---

## 4. PEPS plant model — challenge response stagger

`PepsPlantModel::with_response_stagger_ms(N)` makes each fob/phone
slot's challenge response sleep `(slot+1) × N` ms before publishing.

Production setting: `PRODUCTION_STAGGER_MS = 10` (set in `main.rs`).
Worst case 6 fobs × 10 ms = 60 ms latency; +20 ms for 2 phones = 80 ms
total.  Well below the 150 ms `CHALLENGE_TIMEOUT_MS` in PassiveEntry.

Test default: `0` (instant) — so the existing 85 PEPS unit tests
continue asserting on response presence without needing virtual-time
advance.

The stagger has two purposes:

1. **Make tests meaningful.** Without stagger, all 6 candidate fobs
   in the same zone would publish on the same scheduler tick, and
   the bus's broadcast ordering would be implementation-defined.
   With stagger the "first-responder wins" behaviour becomes
   deterministic.
2. **Avoid cross-fob interference.** Real LF transmissions are
   half-duplex — only one device can cleanly transmit on the LF
   channel at a time.  Real BCMs use device-specific time slots or
   collision-detection.  Our 10 ms × slot is a simple deterministic
   stand-in.

---

## 5. The keys-in-vehicle guard (ThumbPadLock)

**Rule:** ThumbPadLock fires `LockAll` only if at least one paired
PEPS device is in a zone outside the cabin
(`is_outside_cabin(zone) == true` for DriverDoor, PassengerDoor,
Hood, Trunk, or Approach).

When the gate denies a press:
- No `LockAll` is dispatched.
- `FEEDBACK_REQUEST = "lock_denied"` is published — a new feedback
  kind that LockFeedback currently ignores; it's intended for a
  future "denial" flash pattern.
- Pad latch is cleared so a fresh press is required to retry.

Why this matters: a child inside the cabin pressing the thumb pad
through an open door (intentionally or by accident) must not lock
the keys in the vehicle.  Real PEPS systems implement this in the
BCM at the same level — it's a regulatory + safety expectation, not
a UX nicety.

The gate uses an in-memory `device_zones: Vec<Zone>` cache, kept
current by a zone-watcher branch in the same `select!` loop.  Same
pattern as PassiveEntry — keeps the hot path (the 500 ms debounce
fire) purely in-memory.

---

## 6. Test coverage

| Layer | Tests |
|---|---|
| `passive_entry::tests` | 11 (handle pull happy path, no-device deny, Approach-only deny, two-stage, unpaired ignore, phone path, parser round-trip + fuzz, nonce uniqueness, secret-format invariant, crypto determinism) |
| `thumb_pad_lock::tests` | 9 (5 existing + 4 new gate-denial tests) |
| `welcome::tests` | 6 (entry arms, hold expires, ignition-on releases, second-device-no-extend, all-leave releases, RfRange does NOT arm) |
| `tests/ws_integration` | 4 new (real bridge subprocess; PE unlock, no-device deny, Welcome puddle, ThumbPad gate) |
| Gherkin specs | `features/passive_entry.feature` (8 scenarios), `features/welcome.feature` (6 scenarios) — documentation only; cucumber step defs are a follow-up |

Total impact: **+34 tests** on the branch.  All 323 lib + 33 cucumber +
8 ws_integration = **364 tests pass**.  `clippy --all-targets -D warnings`
clean, `cargo fmt` clean.

---

## 7. Manual test recipe

```bash
# Build + run
cd vss-bridge && cargo build --release
./target/release/vss-bridge

# In the HMI (http://localhost:3000/vss-hmi.html):
# 1. Place fob F1 in the APPROACH zone     → puddle lamps come on (Welcome)
# 2. Drag F1 to the DRIVER DOOR zone       → still on (proximity is also LF)
# 3. Pull the Row1.Left outside handle     → driver door unlocks (PassiveEntry)
# 4. Pull again within 3 s                 → all doors unlock (stage 2)
# 5. Press LOCK PAD on Row1.Left handle    → all doors lock (ThumbPadLock,
#                                            gate passes because F1 is at the
#                                            driver door)
# 6. Drag F1 to CABIN, press LOCK PAD      → does NOT lock (keys-in-vehicle
#                                            guard); look for "lock_denied"
#                                            in the bus log
# 7. Set ignition ON                       → Welcome lights release immediately
```

---

## 8. Open work / known limitations

1. **Cucumber step definitions** for `passive_entry.feature` and
   `welcome.feature` are not yet wired up.  The unit tests + ws_integration
   cover the same requirement set; full gherkin coverage requires a new
   e2e harness that can manipulate dealer config + virtual time + paired
   secrets.  Useful follow-up; not blocking.

2. **Rear-door proximity-zone mapping** (`Row2.*` doors map to
   `Zone::PassengerDoor` for the candidate scan).  In real vehicles
   the rear handles share the cabin LF perimeter rather than having
   their own antennas; this approximation is fine for now but a
   production-grade implementation would gate rear-door auth on a
   prior successful stage-2 unlock from the front.

3. **Welcome cal value**.  30 s hold is hard-coded as
   `WELCOME_HOLD_SECS`.  Real OEMs vary 15–60 s; should become a
   `vehicle_line.welcome_hold_secs` Tier-2 parameter once the calibration
   structure has a place for it.

4. **`lock_denied` feedback** has no consumer yet.  LockFeedback
   should grow a third pattern (e.g. 3 short flashes, or a different
   colour) and a HMI cue.  See `features/welcome.feature` /
   `features/passive_entry.feature` for inspiration on the pattern.

5. **PerimeterAlarm / Farewell** are not implemented.  The courtesy
   arbiter's allow-list intentionally only registers Welcome today;
   adding those features is a one-line allow-entry per signal each.

---

*Author: Anup Gadkari & Claude. Branch `feature/peps`, generated
2026-04-27.*
