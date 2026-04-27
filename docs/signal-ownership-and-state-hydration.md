# Signal Ownership, NVM Persistence, and HMI State Hydration

**Status:** Architecture decision — applies to all stateful signals on the bus.
**Branch where this took effect:** `feature/panic-alarm`.
**Related work:** `docs/validation-plan-body-e2e.md` (covers the testing
implications of these rules).

---

## 1. Why this document exists

While testing the panic-alarm + auto-relock features in the HMI, we hit
two related symptoms:

1. The HMI's `INIT_STATE` had `Body.Doors.Row*.IsLocked = false` *and*
   `Body.Doors.Row*.Soldier.IsUnlocked = false`.  These two values are
   logically inconsistent — `Soldier.IsUnlocked` is the *interior knob
   position*, mechanically linked to the actuator, so it should be the
   inverse of `IsLocked`.  The bug was masked at runtime by the
   WebSocket bridge sending a state snapshot on connect, but a
   millisecond of inconsistent state was visible at page-load.

2. There was no way to test "BCM rebooted while doors were locked" —
   the plant model's constructor hard-codes `[false; 4]`, so the
   simulation always cold-boots in the all-unlocked / factory-new state.

These two symptoms share one root cause: **there is no single source of
truth for boot state**, and three different layers (plant model, HMI
`INIT_STATE`, WebSocket snapshot) each have their own opinion that can
drift out of sync.

This document codifies the rules that prevent this class of bug from
recurring as the platform grows.

---

## 2. The rule: signal ownership

> **Every stateful signal on the bus has exactly one *owner*.**
> The owner is the only authority that can publish that signal's value.
> Everyone else either (a) reads it, or (b) sends a *command* that the
> owner may choose to translate into a state change.

For body-controller signals the owner is almost always either the
**plant model** that simulates the physical actuator, or the **feature
module** that drives a derived state flag.  The HMI is *never* an
owner — it's a presentation layer.

### 2.1 Owner classes

| Owner class | Examples | Persistence requirement |
|---|---|---|
| **Plant model** (state of a physical thing) | `Body.Doors.Row*.IsLocked`, `Body.Doors.Row*.Soldier.IsUnlocked`, `Body.Doors.Row*.IsOpen`, `Body.Trunk.IsOpen`, `Body.Trunk.IsLocked`, `Body.Hood.IsOpen` | NVM-backed (state must survive a power cycle) |
| **Feature module** (computed state flag) | `Vehicle.Body.Alarm.IsActive`, `Body.Doors.AutoRelock.IsArmed`, `Body.Doors.AutoRelock.TimeoutSeconds` | Volatile (rebuilds from inputs each boot) |
| **Arbiter output** (resolved actuator value) | `Body.Lights.Beam.Low.IsOn`, `Body.Horn.IsActive`, `Body.Lights.DirectionIndicator.*.IsSignaling` | Volatile (recomputed from current claims) |
| **HMI input** (sensor / switch position the user sets) | `Body.Switches.Hazard.IsEngaged`, `Body.Switches.Panic.IsEngaged`, `Vehicle.LowVoltageSystemState`, `Body.PEPS.Plant.KeyFob.*.Zone`, `Chassis.Brake.PedalPosition` | Currently volatile; could be made persistent if "remember last user setting" is desired in demo mode |

The HMI **owns** the inputs it presents — those reflect the human's
intent.  Everything else, the HMI **renders** from snapshots received
on the WebSocket.

### 2.2 What "ownership" forbids

If a signal is owned by the plant model, then:

- The HMI **must not** include a default value for it in `INIT_STATE`.
  Until the WebSocket has delivered the first snapshot, the HMI doesn't
  know the value — and it must visually communicate that uncertainty
  rather than silently render a wrong default.

- No other Rust module may `bus.publish(...)` that signal *except as a
  bypass for testing*.  Bypasses must be explicitly audited and the
  plant model must subscribe to its own outputs (the "mirror
  subscription" pattern in `door_lock.rs`) so the dedup guard doesn't
  desynchronise.

If a signal is owned by the HMI (an input), then:

- The plant model and features must subscribe to it but never publish
  it back.  Echoing an input as if it were an output creates loops.

---

## 3. NVM persistence model

### 3.1 What goes in NVM

In a real BCM, the following typically persist across power cycles:

- **Door lock state** (`IsLocked`, `IsDoubleLocked`) — the doors stay
  locked when the car is parked.
- **Trunk lock state** — same reason.
- **Window positions** (when this feature lands) — windows stay where
  they were last left.
- **Mirror tilt/pan, seat memory, cabin climate setpoints** — comfort
  preferences.
- **Crash-detected latch + AutoRelock-disabled latch** — safety
  interlocks must persist so that a power cycle during a crash event
  doesn't accidentally re-enable auto-relock.

In our simulation, only the lock-related state is currently
non-volatile; the rest can be added one at a time using the same
pattern.

### 3.2 Where it lives

A small persistence module:

```
vss-bridge/src/nvm.rs

pub struct NvmStore { path: PathBuf }

impl NvmStore {
    pub fn load_door_lock(&self) -> Option<DoorLockState>
    pub fn save_door_lock(&self, state: &DoorLockState) -> io::Result<()>
}

#[derive(Serialize, Deserialize, Default)]
pub struct DoorLockState {
    pub locked: [bool; 4],
    pub double_locked: [bool; 4],
}
```

- File path defaults to `./nvm/` relative to CWD; overridable via the
  `VSS_BRIDGE_NVM_PATH` env var.
- Files are plain JSON — easy to inspect, easy to seed in tests.
- Plant models load on construction; persist on every state change.
- Failure mode: missing file → factory default (`Default::default()`)
  + log a `tracing::info!` so it's clear in the log.  Corrupt file →
  factory default + log a `tracing::warn!` (don't crash, don't lose
  service).

### 3.3 Test affordances

- `--reset-nvm` CLI flag on the bridge wipes persisted state on start.
  Used for cold-boot tests.
- Plant models keep a `with_initial_state(...)` test-only constructor
  that bypasses NVM entirely.  Cucumber `Given` steps can then say
  *"Given the doors were locked at last shutdown"* and have it actually
  mean something.
- Each plant model documents which NVM keys it owns.

---

## 4. HMI hydration

### 4.1 The contract

> **The first WebSocket message from the bridge to a fresh HMI client
> is always a complete, consistent snapshot of every owned signal.**

This contract is enforced by the bridge — see §5 below.  The HMI's job
is to (a) wait for that snapshot before rendering owned-signal visuals,
and (b) know the difference between an unhydrated value and a real one.

### 4.2 The HMI-side rules

1. **`INIT_STATE` carries values only for HMI-owned inputs**
   (switch positions, fob zones, ignition state).  Bridge-owned signals
   are absent — they're populated by the snapshot.

2. **A `wsHydrated` flag** is set when the first `{state: ...}` message
   arrives.  Until then, `wsHydrated === false`.

3. **Visuals that depend on bridge-owned signals** check `wsHydrated`
   before rendering.  When unhydrated, render a dimmed / "—" / "loading"
   placeholder rather than misrepresenting state.

4. **On WebSocket reconnect**, `wsHydrated` is reset to `false` and
   re-set when the new snapshot arrives.  The dimmed-placeholder
   behaviour kicks back in for the hydration window.

This produces correct behaviour on:

- First page load (briefly dim until snapshot arrives, then live)
- Reload mid-operation (briefly dim, then live with correct state)
- Bridge restart while HMI is open (dim during reconnect, then live)
- Race against plant-model boot (no flash of inconsistent values
  because the bridge's ready-gate ensures the snapshot is complete —
  see §5)

---

## 5. Bridge ready-gate

### 5.1 The problem

Today the WebSocket bridge accepts connections immediately.  If a
client connects before plant models have finished publishing their
initial state, the snapshot is partial — `output_state` only contains
whatever has been published so far.

### 5.2 The fix

Each plant model that owns persisted state signals a `Notify` once it
has published its boot snapshot:

```rust
// In WsBridge::run():
let plant_models_ready = Arc::new(tokio::sync::Notify::new());

// Each plant model that owns NVM-backed state calls notify_one() after
// completing its first publish_all().

// In handle_connection(), before sending the first snapshot:
plant_models_ready.notified().await;
let snapshot = output_state.lock().await.clone();
ws_tx.send(Message::Text(json!({"state": snapshot}).to_string().into())).await?;
```

For multiple plant models we use a counted barrier (or a `JoinSet`) so
the gate only opens after *all* of them have boot-published.

### 5.3 Why this matters

Without the gate, a fast browser tab can race the plant model and
receive an empty snapshot.  The HMI's `wsHydrated` flag would still
flip true, but the rendered state would be wrong.  The gate makes the
contract enforceable.

---

## 6. End-to-end flow (revised)

```
Boot:
    ┌─────────────────────────┐
    │ vss-bridge starts       │
    └───────────┬─────────────┘
                │
                ▼
    ┌─────────────────────────┐    ┌──────────────┐
    │ Plant models load NVM   │◄───┤ ./nvm/*.json │
    │ (door_lock, trunk, …)   │    └──────────────┘
    └───────────┬─────────────┘
                │
                ▼
    ┌─────────────────────────┐
    │ Plant models publish    │
    │ initial state to bus    │  ← arbiter / feature subscribers see this
    └───────────┬─────────────┘
                │
                ▼
    ┌─────────────────────────┐
    │ Plant models notify     │  ← ready-gate flips
    │ "boot publish complete" │
    └───────────┬─────────────┘
                │
                ▼
    ┌─────────────────────────┐
    │ WS bridge accepts new   │
    │ HMI connections         │
    └───────────┬─────────────┘
                │
                ▼ (HMI connects)
    ┌─────────────────────────┐
    │ Bridge sends complete   │  ← HMI flips wsHydrated=true
    │ snapshot                │
    └───────────┬─────────────┘
                │
                ▼
    ┌─────────────────────────┐
    │ HMI renders live state  │  ← no more dimmed placeholders
    └─────────────────────────┘

Runtime change (e.g. user presses RKE LOCK):
    HMI publishes input  →  RKE feature  →  DoorLockArbiter  →
    DoorLockPlantModel  →  publishes IsLocked=true  →  saves to NVM
    →  WS bridge captures + broadcasts  →  HMI re-renders.

Reload (HMI Cmd+Shift+R):
    HMI WS reconnects  →  bridge sends current snapshot  →
    HMI flips wsHydrated=true  →  live state.

Cold boot (--reset-nvm):
    NVM wiped  →  plant models fall back to Default::default()  →
    factory state (all unlocked) on first boot.

Warm boot (NVM survived):
    Plant models read NVM  →  whatever was persisted at last shutdown.
```

---

## 7. Test scenarios this enables

| Scenario | How to set it up | What it proves |
|---|---|---|
| Cold boot (factory new) | Run with `--reset-nvm` | Plant models fall back to `Default::default()` cleanly |
| Warm boot, was locked | Pre-seed `nvm/door_lock.json` with `locked:[true;4]`, restart bridge | Doors stay locked across power cycle |
| Warm boot, was double-locked | Pre-seed with `double_locked:[true;4]` | Superlock survives reset; `DoubleLockRelease` clears it on next ignition ON |
| Corrupt NVM | Write garbage into `nvm/door_lock.json` | Falls back to factory default + emits warn-level log; service stays up |
| HMI reload mid-AutoRelock-countdown | UNLOCK, reload page | Banner re-appears at remaining seconds (or full timeout — see §8 limitation) |
| HMI reload during panic alarm | PANIC, reload page | `Vehicle.Body.Alarm.IsActive=true` and pulse loop both visible after reconnect |
| HMI reload mid-pulse-cycle | PANIC, reload during ON window | No flash of wrong indicator state because of `wsHydrated` gating |
| BCM crash recovery | Trigger crash signal, kill bridge mid-write, restart | NVM either holds last-committed state or factory default — never half-written |

---

## 8. Known limitations / non-goals

1. **AutoRelock countdown is wall-clock client-side.**  If the HMI is
   reloaded mid-countdown, it captures the moment it sees `IsArmed=true`
   and shows a fresh 45 s — not the remaining time on the bridge's
   actual timer.  Acceptable for a demo.  A full fix would require the
   bridge to publish the *deadline timestamp* (or remaining seconds at
   1 Hz) so the HMI can compute true remaining time.

2. **NVM atomic writes are not yet implemented.**  A power cut during a
   write could leave `door_lock.json` truncated.  Mitigation: write to
   `door_lock.json.tmp`, fsync, rename — standard pattern, easy to add
   when needed.

3. **Tier 4 dealer config is separate.**  Calibration values pushed by
   the M7 / dealer tool are managed in `config.rs`, not NVM.  This
   distinction matches real OEM architecture — calibration vs. runtime
   state are two different stores.

4. **HMI input persistence is not in scope.**  If the user sets
   ignition to ON and reloads, the bridge already remembers (because
   ws_bridge subscribes to INPUT_SIGNALS and replays them) — this
   document doesn't change that behaviour.  But the *user's intent* of
   setting the switch is conceptually a side effect of an external
   actor, not vehicle-state-at-rest, so it's reasonable that it can
   reset on a full bridge restart (which represents a physical
   battery disconnect).

---

## 9. Migration plan (this PR)

| Step | Scope | Where |
|---|---|---|
| 1 | Strip owned values from HMI `INIT_STATE` | `vss-hmi.html` |
| 2 | Add `wsHydrated` flag and dim-until-hydrated gating | `vss-hmi.html` |
| 3 | Add `nvm.rs` module + plant-model NVM integration | `vss-bridge/src/nvm.rs`, `door_lock.rs`, `main.rs` |
| 4 | Add bridge ready-gate before first WS snapshot | `ws_bridge.rs` |
| 5 | Apply same pattern to trunk, hood, sunroof state where applicable | `trunk.rs`, future plant models |

Each step is an atomic commit with its own tests.  Step 1 alone closes
the visible bug.  Step 3 unlocks all the boot-scenario testing.
Steps 2 + 4 prevent the bug from coming back when a new owned signal
lands.

---

## 10. Style guide for new owned signals

When adding a new owned signal to the platform, the contributor should
ask in this order:

1. **Who owns this signal?**  If it's persisted physical state, the
   answer is "a plant model".  If it's a derived flag, "a feature
   module".  Document the owner in the signal's doc comment.

2. **Does it need NVM?**  If state must survive a power cycle, yes.
   Add it to the NVM struct for that owner.

3. **Should the HMI render it?**  If yes, add the path to
   `OUTPUT_SIGNALS` in `ws_bridge.rs`.  Do **not** add a default value
   to `INIT_STATE`.  Render visuals via the `wsHydrated` gate.

4. **Should the HMI mutate it directly?**  If it's an input from the
   user (a switch they're pressing), yes — add to `INPUT_SIGNALS`.  If
   it's owned state, no — add a *command* signal that the owner
   subscribes to instead.

Following these four questions every time keeps the layer boundaries
clean and prevents the next "INIT_STATE has wrong default" bug.

---

*Author: Anup Gadkari & Claude. Branch `feature/panic-alarm`,
generated 2026-04-27.*
