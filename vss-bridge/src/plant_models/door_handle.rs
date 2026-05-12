//! Door handle and latch plant model.
//!
//! Simulates the physical door handle mechanism on each door:
//!
//! ```text
//!  HMI (top-view handle button)
//!      │  Handle.Inside.IsPulled / Handle.Outside.IsPulled
//!      ▼
//!  DoorHandlePlantModel          ← this module
//!      │  publishes Latch.IsLatched / IsOpen (ajar switch)
//!      ▼
//!  SignalBus → WsBridge → HMI
//! ```
//!
//! # Behaviour matrix
//!
//! | Lock state    | Inside handle | Outside handle |
//! |---------------|---------------|----------------|
//! | Unlocked      | Latch unlatches → door ajar | Same |
//! | Single-locked | Latch briefly unlatches while held, re-engages on release, no ajar | **Blocked** |
//! | Double-locked | **Blocked** | **Blocked** |
//!
//! The interior "soldier" knob overrides a single door's lock state.
//! Double-lock physically disconnects the interior linkage — soldier input
//! is ignored while `IsDoubleLocked` is true.
//!
//! Closing the door (CloseCmd) re-latches and republishes `IsLocked` so
//! the HMI soldier indicator stays consistent with the door's lock state.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::ipc_message::SignalValue;
use crate::signal_bus::SignalBus;

// ── Signal path arrays (all indexed 0=Row1.Left, 1=Row1.Right,
//                                    2=Row2.Left, 3=Row2.Right) ──────────────

pub const INSIDE_HANDLE_SIGNALS: [&str; 4] = [
    "Body.Doors.Row1.Left.Handle.Inside.IsPulled",
    "Body.Doors.Row1.Right.Handle.Inside.IsPulled",
    "Body.Doors.Row2.Left.Handle.Inside.IsPulled",
    "Body.Doors.Row2.Right.Handle.Inside.IsPulled",
];

pub const OUTSIDE_HANDLE_SIGNALS: [&str; 4] = [
    "Body.Doors.Row1.Left.Handle.Outside.IsPulled",
    "Body.Doors.Row1.Right.Handle.Outside.IsPulled",
    "Body.Doors.Row2.Left.Handle.Outside.IsPulled",
    "Body.Doors.Row2.Right.Handle.Outside.IsPulled",
];

pub const SOLDIER_SIGNALS: [&str; 4] = [
    "Body.Doors.Row1.Left.Soldier.IsUnlocked",
    "Body.Doors.Row1.Right.Soldier.IsUnlocked",
    "Body.Doors.Row2.Left.Soldier.IsUnlocked",
    "Body.Doors.Row2.Right.Soldier.IsUnlocked",
];

pub const CLOSE_CMD_SIGNALS: [&str; 4] = [
    "Body.Doors.Row1.Left.CloseCmd",
    "Body.Doors.Row1.Right.CloseCmd",
    "Body.Doors.Row2.Left.CloseCmd",
    "Body.Doors.Row2.Right.CloseCmd",
];

pub const LATCH_SIGNALS: [&str; 4] = [
    "Body.Doors.Row1.Left.Latch.IsLatched",
    "Body.Doors.Row1.Right.Latch.IsLatched",
    "Body.Doors.Row2.Left.Latch.IsLatched",
    "Body.Doors.Row2.Right.Latch.IsLatched",
];

/// Read-back of IsLocked / IsDoubleLocked (published by DoorLockPlantModel).
const IS_LOCKED_SIGNALS: [&str; 4] = [
    "Body.Doors.Row1.Left.IsLocked",
    "Body.Doors.Row1.Right.IsLocked",
    "Body.Doors.Row2.Left.IsLocked",
    "Body.Doors.Row2.Right.IsLocked",
];

const IS_DOUBLE_LOCKED_SIGNALS: [&str; 4] = [
    "Body.Doors.Row1.Left.IsDoubleLocked",
    "Body.Doors.Row1.Right.IsDoubleLocked",
    "Body.Doors.Row2.Left.IsDoubleLocked",
    "Body.Doors.Row2.Right.IsDoubleLocked",
];

const AJAR_SIGNALS: [&str; 4] = [
    "Body.Doors.Row1.Left.IsOpen",
    "Body.Doors.Row1.Right.IsOpen",
    "Body.Doors.Row2.Left.IsOpen",
    "Body.Doors.Row2.Right.IsOpen",
];

/// Per-door child-lock state (output of the `PowerChildLock` feature).
/// Only Row2 doors are ever child-locked; the Row1 slots are `None`
/// because no signal is subscribed for them.  When the indexed slot is
/// `Some(sig)` and the latest value is `true`, the door plant ignores
/// `Handle.Inside.IsPulled` — kids can't open the door from inside.
const CHILD_LOCK_SIGNALS: [Option<&str>; 4] = [
    None,
    None,
    Some("Body.Doors.Row2.Left.IsChildLockActive"),
    Some("Body.Doors.Row2.Right.IsChildLockActive"),
];

/// Short labels used in tracing.
const DOOR_LABELS: [&str; 4] = ["Row1.Left", "Row1.Right", "Row2.Left", "Row2.Right"];

// ── Plant model ───────────────────────────────────────────────────────────────

/// Door handle / latch / ajar plant model.
///
/// Spawn 4 per-door coroutines via [`DoorHandlePlantModel::run`].
pub struct DoorHandlePlantModel<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> DoorHandlePlantModel<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    /// Drive a single door indefinitely.
    async fn run_door(door: usize, bus: Arc<B>) {
        let label = DOOR_LABELS[door];

        // Subscribe to all relevant signals.
        let mut is_locked_rx = bus.subscribe(IS_LOCKED_SIGNALS[door]).await;
        let mut is_double_locked_rx = bus.subscribe(IS_DOUBLE_LOCKED_SIGNALS[door]).await;
        let mut is_open_rx = bus.subscribe(AJAR_SIGNALS[door]).await;
        let mut inside_rx = bus.subscribe(INSIDE_HANDLE_SIGNALS[door]).await;
        let mut outside_rx = bus.subscribe(OUTSIDE_HANDLE_SIGNALS[door]).await;
        let mut soldier_rx = bus.subscribe(SOLDIER_SIGNALS[door]).await;
        let mut close_rx = bus.subscribe(CLOSE_CMD_SIGNALS[door]).await;
        // Row2 doors track child-lock state from the PowerChildLock
        // feature so they can suppress inside-handle pulls.  Row1 has
        // no child lock — we substitute an empty stream so the
        // `select!` arms stay symmetric.
        let mut child_lock_rx: futures::stream::BoxStream<'static, SignalValue> =
            match CHILD_LOCK_SIGNALS[door] {
                Some(sig) => bus.subscribe(sig).await,
                None => Box::pin(futures::stream::empty()),
            };

        // Internal state — updated by bus events.
        let mut locked = false;
        let mut double_locked = false;
        let mut child_locked = false;
        let mut is_latched = true;
        let mut is_open = false;
        // Currently-held state — true while the user is holding the
        // handle.  Tracked so a lock→unlock transition (e.g. from
        // PassiveEntry's auth) immediately opens the door if the
        // user is still pulling, matching real-vehicle behaviour
        // where you hold the handle and the latch releases under
        // your hand.
        let mut outside_held = false;
        let mut inside_held = false;

        // Publish initial latch state (ajar is already false by default).
        let _ = bus
            .publish(LATCH_SIGNALS[door], SignalValue::Bool(true))
            .await;
        let _ = bus
            .publish(AJAR_SIGNALS[door], SignalValue::Bool(false))
            .await;

        tracing::info!(door = label, "DoorHandle plant: door initialized");

        loop {
            select! {
                // ── Track lock state from DoorLockPlantModel / arbiter ──────
                Some(val) = is_locked_rx.next() => {
                    if let SignalValue::Bool(b) = val {
                        let was_locked = locked;
                        locked = b;
                        tracing::debug!(door = label, locked, "DoorHandle: tracked IsLocked");

                        // Lock → unlock transition while the user is still
                        // holding a handle: release the latch under their hand.
                        // This is the canonical PassiveEntry flow:
                        //   1. user pulls outside handle on a locked door
                        //   2. PassiveEntry runs auth, dispatches Unlock
                        //   3. DoorLockPlantModel publishes IsLocked = false
                        //   4. (us) → if outside_held, open the door now
                        if was_locked && !locked && !double_locked && !is_open
                            && (outside_held || inside_held)
                        {
                            is_latched = false;
                            is_open = true;
                            let _ = bus
                                .publish(LATCH_SIGNALS[door], SignalValue::Bool(false))
                                .await;
                            let _ = bus
                                .publish(AJAR_SIGNALS[door], SignalValue::Bool(true))
                                .await;
                            tracing::info!(
                                door = label,
                                via = if outside_held { "outside" } else { "inside" },
                                "DoorHandle: door opened on lock release while handle held"
                            );
                        }
                    }
                }

                Some(val) = is_double_locked_rx.next() => {
                    if let SignalValue::Bool(b) = val {
                        double_locked = b;
                        tracing::debug!(door = label, double_locked, "DoorHandle: tracked IsDoubleLocked");
                    }
                }

                // ── Track IsOpen even when set externally (e.g. DoorCard) ──
                Some(val) = is_open_rx.next() => {
                    if let SignalValue::Bool(b) = val {
                        let was_open = is_open;
                        is_open = b;
                        if !b && was_open {
                            // Door transitioned from open → closed externally — re-latch.
                            is_latched = true;
                            let _ = bus
                                .publish(LATCH_SIGNALS[door], SignalValue::Bool(true))
                                .await;
                        }
                    }
                }

                // ── Child-lock state (Row2 only — Row1 stream is empty) ──
                Some(val) = child_lock_rx.next() => {
                    if let SignalValue::Bool(b) = val {
                        if child_locked != b {
                            child_locked = b;
                            tracing::debug!(door = label, child_locked,
                                "DoorHandle: tracked IsChildLockActive");
                        }
                    }
                }

                // ── Inside handle ────────────────────────────────────────────
                Some(val) = inside_rx.next() => {
                    if let SignalValue::Bool(pulled) = val {
                        inside_held = pulled;
                        if pulled {
                            if child_locked {
                                // Inside-pull suppressed — kids can't
                                // open the door from inside.  Outside
                                // handle is unaffected.
                                tracing::debug!(door = label,
                                    "DoorHandle: inside handle suppressed (child-locked)");
                            } else if double_locked {
                                // Interior linkage disconnected — completely blocked.
                                tracing::debug!(door = label,
                                    "DoorHandle: inside handle blocked (double-locked)");
                            } else if locked {
                                // Single-locked: latch moves while held, no ajar.
                                if is_latched {
                                    is_latched = false;
                                    let _ = bus
                                        .publish(LATCH_SIGNALS[door], SignalValue::Bool(false))
                                        .await;
                                    tracing::debug!(door = label,
                                        "DoorHandle: inside handle on locked door — latch unlatched");
                                }
                            } else {
                                // Unlocked: door opens.
                                is_latched = false;
                                is_open = true;
                                let _ = bus
                                    .publish(LATCH_SIGNALS[door], SignalValue::Bool(false))
                                    .await;
                                let _ = bus
                                    .publish(AJAR_SIGNALS[door], SignalValue::Bool(true))
                                    .await;
                                tracing::info!(door = label,
                                    "DoorHandle: door opened via inside handle");
                            }
                        } else {
                            // Handle released.
                            if locked && !double_locked && !is_open {
                                // Re-engage latch on single-locked door.
                                is_latched = true;
                                let _ = bus
                                    .publish(LATCH_SIGNALS[door], SignalValue::Bool(true))
                                    .await;
                                tracing::debug!(door = label,
                                    "DoorHandle: inside handle released — latch re-engaged");
                            }
                        }
                    }
                }

                // ── Outside handle ───────────────────────────────────────────
                Some(val) = outside_rx.next() => {
                    if let SignalValue::Bool(pulled) = val {
                        outside_held = pulled;
                        if pulled {
                            if locked {
                                // Locked: don't open now, but the held
                                // state is recorded so a subsequent
                                // unlock-while-pulled (PassiveEntry) opens
                                // the door under the user's hand.
                                tracing::debug!(door = label,
                                    "DoorHandle: outside handle held on locked door (deferred — will open on unlock)");
                            } else {
                                // Unlocked: door opens.
                                is_latched = false;
                                is_open = true;
                                let _ = bus
                                    .publish(LATCH_SIGNALS[door], SignalValue::Bool(false))
                                    .await;
                                let _ = bus
                                    .publish(AJAR_SIGNALS[door], SignalValue::Bool(true))
                                    .await;
                                tracing::info!(door = label,
                                    "DoorHandle: door opened via outside handle");
                            }
                        }
                        // Outside handle release: no re-latch needed (was blocked or door opened).
                    }
                }

                // ── Soldier (interior lock knob) ──────────────────────────────
                Some(val) = soldier_rx.next() => {
                    if let SignalValue::Bool(soldier_unlocked) = val {
                        if double_locked {
                            // Superlock physically disconnects interior linkage.
                            tracing::debug!(door = label,
                                "DoorHandle: soldier movement ignored (double-locked)");
                        } else {
                            let new_locked = !soldier_unlocked;
                            locked = new_locked;
                            let _ = bus
                                .publish(IS_LOCKED_SIGNALS[door], SignalValue::Bool(locked))
                                .await;
                            tracing::info!(door = label, locked,
                                "DoorHandle: soldier moved — lock state updated");
                        }
                    }
                }

                // ── Close door (user clicks ajar door in top view) ────────────
                Some(val) = close_rx.next() => {
                    if let SignalValue::Bool(true) = val {
                        if is_open {
                            is_open = false;
                            is_latched = true;
                            let _ = bus
                                .publish(AJAR_SIGNALS[door], SignalValue::Bool(false))
                                .await;
                            let _ = bus
                                .publish(LATCH_SIGNALS[door], SignalValue::Bool(true))
                                .await;
                            // Republish IsLocked so the soldier indicator on the HMI
                            // reflects the door's current lock state after closing.
                            let _ = bus
                                .publish(IS_LOCKED_SIGNALS[door], SignalValue::Bool(locked))
                                .await;
                            tracing::info!(door = label, locked,
                                "DoorHandle: door closed — latch engaged, soldier refreshed");
                        }
                    }
                }
            }
        }
    }

    /// Spawns all 4 per-door coroutines and drives them concurrently.
    pub async fn run(self) {
        let b = self.bus;
        tokio::join!(
            Self::run_door(0, Arc::clone(&b)),
            Self::run_door(1, Arc::clone(&b)),
            Self::run_door(2, Arc::clone(&b)),
            Self::run_door(3, Arc::clone(&b)),
        );
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use tokio::time::Duration;

    use crate::adapters::mock::MockBus;

    /// Spawn model, let it initialise, then return bus + handle.
    async fn setup() -> (Arc<MockBus>, tokio::task::JoinHandle<()>) {
        let bus = Arc::new(MockBus::new());
        // Pre-seed all doors as unlocked (matches DoorLockPlantModel initial state).
        for sig in &IS_LOCKED_SIGNALS {
            bus.inject(sig, SignalValue::Bool(false));
        }
        for sig in &IS_DOUBLE_LOCKED_SIGNALS {
            bus.inject(sig, SignalValue::Bool(false));
        }
        let model = DoorHandlePlantModel::new(Arc::clone(&bus));
        let handle = tokio::spawn(model.run());
        // Yield enough for all 4 per-door tasks to start and publish initial state.
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
        (bus, handle)
    }

    #[tokio::test]
    async fn initial_state_latched_and_closed() {
        let (bus, handle) = setup().await;
        let h = bus.history();
        let latched: Vec<_> = h
            .iter()
            .filter(|(p, v)| p.contains("Latch.IsLatched") && *v == SignalValue::Bool(true))
            .collect();
        assert_eq!(latched.len(), 4, "all 4 doors should start latched");
        handle.abort();
        let _ = handle.await;
    }

    #[tokio::test]
    async fn inside_handle_opens_unlocked_door() {
        let (bus, handle) = setup().await;
        bus.clear_history();

        bus.inject(INSIDE_HANDLE_SIGNALS[0], SignalValue::Bool(true));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(p, v)| *p == AJAR_SIGNALS[0] && *v == SignalValue::Bool(true)),
            "door should be ajar after inside handle on unlocked door"
        );
        assert!(
            h.iter()
                .any(|(p, v)| *p == LATCH_SIGNALS[0] && *v == SignalValue::Bool(false)),
            "latch should be unlatched"
        );

        handle.abort();
        let _ = handle.await;
    }

    #[tokio::test]
    async fn outside_handle_opens_unlocked_door() {
        let (bus, handle) = setup().await;
        bus.clear_history();

        bus.inject(OUTSIDE_HANDLE_SIGNALS[2], SignalValue::Bool(true));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(p, v)| *p == AJAR_SIGNALS[2] && *v == SignalValue::Bool(true)),
            "RL door should be ajar"
        );

        handle.abort();
        let _ = handle.await;
    }

    #[tokio::test]
    async fn outside_handle_blocked_when_locked() {
        let (bus, handle) = setup().await;

        bus.inject(IS_LOCKED_SIGNALS[1], SignalValue::Bool(true));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;
        bus.clear_history();

        bus.inject(OUTSIDE_HANDLE_SIGNALS[1], SignalValue::Bool(true));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(p, v)| *p == AJAR_SIGNALS[1] && *v == SignalValue::Bool(true)),
            "outside handle should be blocked on locked door"
        );

        handle.abort();
        let _ = handle.await;
    }

    #[tokio::test]
    async fn inside_handle_unlatches_then_reengages_on_locked_door() {
        let (bus, handle) = setup().await;

        bus.inject(IS_LOCKED_SIGNALS[0], SignalValue::Bool(true));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;
        bus.clear_history();

        // Pull handle
        bus.inject(INSIDE_HANDLE_SIGNALS[0], SignalValue::Bool(true));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(p, v)| *p == LATCH_SIGNALS[0] && *v == SignalValue::Bool(false)),
            "latch should unlatch briefly on locked door"
        );
        assert!(
            !h.iter()
                .any(|(p, v)| *p == AJAR_SIGNALS[0] && *v == SignalValue::Bool(true)),
            "door should NOT open on locked door"
        );

        bus.clear_history();

        // Release handle
        bus.inject(INSIDE_HANDLE_SIGNALS[0], SignalValue::Bool(false));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(p, v)| *p == LATCH_SIGNALS[0] && *v == SignalValue::Bool(true)),
            "latch should re-engage after handle released on locked door"
        );

        handle.abort();
        let _ = handle.await;
    }

    #[tokio::test]
    async fn double_lock_blocks_inside_handle() {
        let (bus, handle) = setup().await;

        bus.inject(IS_LOCKED_SIGNALS[0], SignalValue::Bool(true));
        bus.inject(IS_DOUBLE_LOCKED_SIGNALS[0], SignalValue::Bool(true));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;
        bus.clear_history();

        bus.inject(INSIDE_HANDLE_SIGNALS[0], SignalValue::Bool(true));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(p, _)| p.contains("Latch") || p.contains("IsOpen")),
            "double-locked door: inside handle must be completely blocked"
        );

        handle.abort();
        let _ = handle.await;
    }

    #[tokio::test]
    async fn soldier_unlocks_door_and_updates_is_locked() {
        let (bus, handle) = setup().await;

        // Pre-set door as locked
        bus.inject(IS_LOCKED_SIGNALS[3], SignalValue::Bool(true));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;
        bus.clear_history();

        // Soldier moved to unlocked position
        bus.inject(SOLDIER_SIGNALS[3], SignalValue::Bool(true));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(p, v)| *p == IS_LOCKED_SIGNALS[3] && *v == SignalValue::Bool(false)),
            "soldier unlock should publish IsLocked=false"
        );

        handle.abort();
        let _ = handle.await;
    }

    #[tokio::test]
    async fn soldier_blocked_when_double_locked() {
        let (bus, handle) = setup().await;

        bus.inject(IS_LOCKED_SIGNALS[0], SignalValue::Bool(true));
        bus.inject(IS_DOUBLE_LOCKED_SIGNALS[0], SignalValue::Bool(true));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;
        bus.clear_history();

        bus.inject(SOLDIER_SIGNALS[0], SignalValue::Bool(true));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(p, v)| *p == IS_LOCKED_SIGNALS[0] && *v == SignalValue::Bool(false)),
            "soldier should be blocked when double-locked"
        );

        handle.abort();
        let _ = handle.await;
    }

    #[tokio::test]
    async fn close_cmd_resets_ajar_and_relatch_and_refreshes_soldier() {
        let (bus, handle) = setup().await;

        // Open door via inside handle
        bus.inject(INSIDE_HANDLE_SIGNALS[1], SignalValue::Bool(true));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Move soldier to locked
        bus.inject(SOLDIER_SIGNALS[1], SignalValue::Bool(false));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        // Close door
        bus.inject(CLOSE_CMD_SIGNALS[1], SignalValue::Bool(true));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter()
                .any(|(p, v)| *p == AJAR_SIGNALS[1] && *v == SignalValue::Bool(false)),
            "door should close"
        );
        assert!(
            h.iter()
                .any(|(p, v)| *p == LATCH_SIGNALS[1] && *v == SignalValue::Bool(true)),
            "latch should re-engage on close"
        );
        assert!(
            h.iter()
                .any(|(p, v)| *p == IS_LOCKED_SIGNALS[1] && *v == SignalValue::Bool(true)),
            "IsLocked should be refreshed to reflect soldier position (locked)"
        );

        handle.abort();
        let _ = handle.await;
    }

    #[tokio::test]
    async fn doors_are_independent() {
        let (bus, handle) = setup().await;
        bus.clear_history();

        // Open only Row1.Right via inside handle
        bus.inject(INSIDE_HANDLE_SIGNALS[1], SignalValue::Bool(true));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let h = bus.history();
        // Row1.Right open
        assert!(h
            .iter()
            .any(|(p, v)| *p == AJAR_SIGNALS[1] && *v == SignalValue::Bool(true)));
        // Other doors not open
        for &i in &[0usize, 2, 3] {
            assert!(
                !h.iter()
                    .any(|(p, v)| *p == AJAR_SIGNALS[i] && *v == SignalValue::Bool(true)),
                "door {} should not be ajar",
                i
            );
        }

        handle.abort();
        let _ = handle.await;
    }

    /// Hold the outside handle on a LOCKED door, then unlock — door
    /// should open under the user's hand (PassiveEntry flow).
    #[tokio::test]
    async fn outside_handle_held_when_unlocked_opens_door() {
        let (bus, handle) = setup().await;

        // Lock Row1.Left, then start holding the outside handle.
        bus.inject(IS_LOCKED_SIGNALS[0], SignalValue::Bool(true));
        for _ in 0..4 {
            tokio::task::yield_now().await;
        }
        bus.inject(OUTSIDE_HANDLE_SIGNALS[0], SignalValue::Bool(true));
        for _ in 0..4 {
            tokio::task::yield_now().await;
        }
        // Door should NOT be open yet — locked.
        assert!(
            !bus.history()
                .iter()
                .any(|(p, v)| *p == AJAR_SIGNALS[0] && *v == SignalValue::Bool(true)),
            "door must NOT open while locked, even with handle held"
        );

        // Now unlock — PassiveEntry would fire this normally.
        bus.clear_history();
        bus.inject(IS_LOCKED_SIGNALS[0], SignalValue::Bool(false));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }

        // Door should NOW be open — handle was still being held when
        // the unlock arrived.
        let h = bus.history();
        assert!(
            h.iter()
                .any(|(p, v)| *p == AJAR_SIGNALS[0] && *v == SignalValue::Bool(true)),
            "door should open when held outside handle is still pulled at unlock time, history: {:?}",
            h
        );

        handle.abort();
        let _ = handle.await;
    }

    /// Pull-and-release on a locked door, THEN unlock — door must NOT
    /// open (the user already let go).
    #[tokio::test]
    async fn outside_handle_released_before_unlock_does_not_open() {
        let (bus, handle) = setup().await;

        bus.inject(IS_LOCKED_SIGNALS[0], SignalValue::Bool(true));
        for _ in 0..4 {
            tokio::task::yield_now().await;
        }
        // Pull then release.
        bus.inject(OUTSIDE_HANDLE_SIGNALS[0], SignalValue::Bool(true));
        for _ in 0..4 {
            tokio::task::yield_now().await;
        }
        bus.inject(OUTSIDE_HANDLE_SIGNALS[0], SignalValue::Bool(false));
        for _ in 0..4 {
            tokio::task::yield_now().await;
        }

        // Now unlock.
        bus.clear_history();
        bus.inject(IS_LOCKED_SIGNALS[0], SignalValue::Bool(false));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(p, v)| *p == AJAR_SIGNALS[0] && *v == SignalValue::Bool(true)),
            "door must NOT open when handle was released before unlock"
        );

        handle.abort();
        let _ = handle.await;
    }

    #[allow(dead_code)]
    fn _use_duration(_: Duration) {}
}
