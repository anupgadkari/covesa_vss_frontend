//! Door lock plant model — simulates the M7 Classic AUTOSAR door lock SWC.
//!
//! In production, the RKE / PEPS feature logic sends a lock command to the
//! M7 via the DoorLockArbiter. The M7 Classic AUTOSAR application layer SWC
//! actuates the door lock motors and publishes the confirmed door state back
//! to the A53 via a `STATE_UPDATE` IPC message.
//!
//! This plant model fills that M7 role during development:
//!
//! ```text
//!  RKE feature
//!      │  DoorLockArbiter request
//!      ▼
//!  DoorLockArbiter
//!      │  publishes LockCmd / DoubleLockCmd (Bool) per door
//!      ▼
//!  DoorLockPlantModel          ← this module
//!      │  publishes IsLocked / IsDoubleLocked (Bool) per door
//!      ▼
//!  SignalBus → WsBridge → HMI
//! ```
//!
//! # Signals consumed (from arbiter)
//! | Signal | Value | Meaning |
//! |--------|-------|---------|
//! | `Body.Doors.Row*.*.LockCmd`       | Bool(true)  | Lock all doors |
//! | `Body.Doors.Row*.*.LockCmd`       | Bool(false) | Unlock all doors |
//! | `Body.Doors.Row*.*.DoubleLockCmd` | Bool(true)  | Engage superlock |
//!
//! # Signals published (confirmed state)
//! | Signal | Value | Meaning |
//! |--------|-------|---------|
//! | `Body.Doors.Row*.*.IsLocked`       | Bool | Door is locked |
//! | `Body.Doors.Row*.*.IsDoubleLocked` | Bool | Door superlock engaged |

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::arbiter::{DOOR_DOUBLE_LOCK_CMD_SIGNALS, DOOR_LOCK_CMD_SIGNALS};
use crate::ipc_message::SignalValue;
use crate::signal_bus::SignalBus;

/// VSS signal paths for confirmed door lock state (published by this plant model).
const DOOR_LOCKED_SIGNALS: [&str; 4] = [
    "Body.Doors.Row1.Left.IsLocked",
    "Body.Doors.Row1.Right.IsLocked",
    "Body.Doors.Row2.Left.IsLocked",
    "Body.Doors.Row2.Right.IsLocked",
];

/// VSS signal paths for confirmed double-lock state.
const DOOR_DOUBLE_LOCKED_SIGNALS: [&str; 4] = [
    "Body.Doors.Row1.Left.IsDoubleLocked",
    "Body.Doors.Row1.Right.IsDoubleLocked",
    "Body.Doors.Row2.Left.IsDoubleLocked",
    "Body.Doors.Row2.Right.IsDoubleLocked",
];

/// Door index constants (matches signal array ordering above).
const ROW1_LEFT: usize = 0;
const ROW1_RIGHT: usize = 1;
const ROW2_LEFT: usize = 2;
const ROW2_RIGHT: usize = 3;

/// Door lock plant model. Spawn with `.run()`.
pub struct DoorLockPlantModel<B: SignalBus> {
    bus: Arc<B>,
    /// Confirmed locked state per door.
    locked: [bool; 4],
    /// Confirmed double-locked state per door.
    double_locked: [bool; 4],
}

impl<B: SignalBus + Send + Sync + 'static> DoorLockPlantModel<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self {
            bus,
            locked: [false; 4],
            double_locked: [false; 4],
        }
    }

    /// Apply a lock/unlock command to a specific door index and publish
    /// confirmed state. Double-lock also sets `IsLocked` (superlock implies locked).
    async fn apply_lock_cmd(&mut self, door: usize, locked: bool) {
        if self.locked[door] == locked {
            return; // no change
        }
        self.locked[door] = locked;
        if !locked {
            // Unlock also clears double-lock.
            self.double_locked[door] = false;
            let _ = self
                .bus
                .publish(DOOR_DOUBLE_LOCKED_SIGNALS[door], SignalValue::Bool(false))
                .await;
        }
        let _ = self
            .bus
            .publish(DOOR_LOCKED_SIGNALS[door], SignalValue::Bool(locked))
            .await;
        tracing::debug!(
            door = DOOR_LOCKED_SIGNALS[door],
            locked,
            "DoorLock plant: door state updated"
        );
    }

    async fn apply_double_lock_cmd(&mut self, door: usize, double_locked: bool) {
        if self.double_locked[door] == double_locked {
            return;
        }
        self.double_locked[door] = double_locked;
        if double_locked {
            // Superlock also sets IsLocked.
            self.locked[door] = true;
            let _ = self
                .bus
                .publish(DOOR_LOCKED_SIGNALS[door], SignalValue::Bool(true))
                .await;
        }
        let _ = self
            .bus
            .publish(
                DOOR_DOUBLE_LOCKED_SIGNALS[door],
                SignalValue::Bool(double_locked),
            )
            .await;
        tracing::debug!(
            door = DOOR_DOUBLE_LOCKED_SIGNALS[door],
            double_locked,
            "DoorLock plant: double-lock state updated"
        );
    }

    /// Publish current state for all doors (called at startup to initialise the HMI).
    async fn publish_all(&self) {
        for i in 0..4 {
            let _ = self
                .bus
                .publish(DOOR_LOCKED_SIGNALS[i], SignalValue::Bool(self.locked[i]))
                .await;
            let _ = self
                .bus
                .publish(
                    DOOR_DOUBLE_LOCKED_SIGNALS[i],
                    SignalValue::Bool(self.double_locked[i]),
                )
                .await;
        }
    }

    /// Extract a `bool` from a `SignalValue::Bool`, ignoring other types.
    fn to_bool(val: &SignalValue) -> Option<bool> {
        match val {
            SignalValue::Bool(b) => Some(*b),
            _ => None,
        }
    }

    pub async fn run(mut self) {
        // Subscribe to all 4 LockCmd and 4 DoubleLockCmd signals.
        let mut lc0 = self.bus.subscribe(DOOR_LOCK_CMD_SIGNALS[ROW1_LEFT]).await;
        let mut lc1 = self.bus.subscribe(DOOR_LOCK_CMD_SIGNALS[ROW1_RIGHT]).await;
        let mut lc2 = self.bus.subscribe(DOOR_LOCK_CMD_SIGNALS[ROW2_LEFT]).await;
        let mut lc3 = self.bus.subscribe(DOOR_LOCK_CMD_SIGNALS[ROW2_RIGHT]).await;

        let mut dlc0 = self
            .bus
            .subscribe(DOOR_DOUBLE_LOCK_CMD_SIGNALS[ROW1_LEFT])
            .await;
        let mut dlc1 = self
            .bus
            .subscribe(DOOR_DOUBLE_LOCK_CMD_SIGNALS[ROW1_RIGHT])
            .await;
        let mut dlc2 = self
            .bus
            .subscribe(DOOR_DOUBLE_LOCK_CMD_SIGNALS[ROW2_LEFT])
            .await;
        let mut dlc3 = self
            .bus
            .subscribe(DOOR_DOUBLE_LOCK_CMD_SIGNALS[ROW2_RIGHT])
            .await;

        // Publish initial state (all unlocked).
        self.publish_all().await;
        tracing::info!("DoorLock plant model started — all doors unlocked");

        loop {
            select! {
                Some(val) = lc0.next() => {
                    if let Some(b) = Self::to_bool(&val) {
                        self.apply_lock_cmd(ROW1_LEFT, b).await;
                    }
                }
                Some(val) = lc1.next() => {
                    if let Some(b) = Self::to_bool(&val) {
                        self.apply_lock_cmd(ROW1_RIGHT, b).await;
                    }
                }
                Some(val) = lc2.next() => {
                    if let Some(b) = Self::to_bool(&val) {
                        self.apply_lock_cmd(ROW2_LEFT, b).await;
                    }
                }
                Some(val) = lc3.next() => {
                    if let Some(b) = Self::to_bool(&val) {
                        self.apply_lock_cmd(ROW2_RIGHT, b).await;
                    }
                }
                Some(val) = dlc0.next() => {
                    if let Some(b) = Self::to_bool(&val) {
                        self.apply_double_lock_cmd(ROW1_LEFT, b).await;
                    }
                }
                Some(val) = dlc1.next() => {
                    if let Some(b) = Self::to_bool(&val) {
                        self.apply_double_lock_cmd(ROW1_RIGHT, b).await;
                    }
                }
                Some(val) = dlc2.next() => {
                    if let Some(b) = Self::to_bool(&val) {
                        self.apply_double_lock_cmd(ROW2_LEFT, b).await;
                    }
                }
                Some(val) = dlc3.next() => {
                    if let Some(b) = Self::to_bool(&val) {
                        self.apply_double_lock_cmd(ROW2_RIGHT, b).await;
                    }
                }
            }
        }
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;

    async fn run_one_tick(bus: &Arc<MockBus>, model: DoorLockPlantModel<MockBus>) {
        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;
        handle.abort();
        let _ = handle.await;
    }

    #[tokio::test]
    async fn initial_state_all_unlocked() {
        let bus = Arc::new(MockBus::new());
        let model = DoorLockPlantModel::new(Arc::clone(&bus));
        run_one_tick(&bus, model).await;

        let history = bus.history();
        // 4 IsLocked + 4 IsDoubleLocked = 8 initial publishes
        let locked_signals: Vec<_> = history
            .iter()
            .filter(|(p, _)| p.contains("IsLocked"))
            .collect();
        assert_eq!(locked_signals.len(), 4, "should publish 4 IsLocked signals");
        for (_, val) in &locked_signals {
            assert_eq!(*val, SignalValue::Bool(false), "initial state should be unlocked");
        }
    }

    #[tokio::test]
    async fn lock_cmd_locks_all_doors() {
        let bus = Arc::new(MockBus::new());
        let model = DoorLockPlantModel::new(Arc::clone(&bus));
        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        // Simulate arbiter writing lock command for all 4 doors
        for &sig in &DOOR_LOCK_CMD_SIGNALS {
            bus.inject(sig, SignalValue::Bool(true));
        }
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let locked_true: Vec<_> = history
            .iter()
            .filter(|(p, v)| p.contains("IsLocked") && !p.contains("Double") && *v == SignalValue::Bool(true))
            .collect();
        assert_eq!(locked_true.len(), 4, "all 4 doors should report IsLocked=true");

        handle.abort();
        let _ = handle.await;
    }

    #[tokio::test]
    async fn unlock_cmd_unlocks_all_doors() {
        let bus = Arc::new(MockBus::new());
        let model = DoorLockPlantModel::new(Arc::clone(&bus));
        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // First lock all doors
        for &sig in &DOOR_LOCK_CMD_SIGNALS {
            bus.inject(sig, SignalValue::Bool(true));
        }
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        // Now unlock
        for &sig in &DOOR_LOCK_CMD_SIGNALS {
            bus.inject(sig, SignalValue::Bool(false));
        }
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let unlocked: Vec<_> = history
            .iter()
            .filter(|(p, v)| p.contains("IsLocked") && !p.contains("Double") && *v == SignalValue::Bool(false))
            .collect();
        assert_eq!(unlocked.len(), 4, "all 4 doors should report IsLocked=false");

        handle.abort();
        let _ = handle.await;
    }

    #[tokio::test]
    async fn double_lock_cmd_sets_both_signals() {
        let bus = Arc::new(MockBus::new());
        let model = DoorLockPlantModel::new(Arc::clone(&bus));
        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        for &sig in &DOOR_DOUBLE_LOCK_CMD_SIGNALS {
            bus.inject(sig, SignalValue::Bool(true));
        }
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();

        // IsDoubleLocked = true for all 4
        let dl_true: Vec<_> = history
            .iter()
            .filter(|(p, v)| p.contains("IsDoubleLocked") && *v == SignalValue::Bool(true))
            .collect();
        assert_eq!(dl_true.len(), 4, "double-lock should set all 4 IsDoubleLocked=true");

        // IsLocked = true for all 4 (superlock implies locked)
        let locked_true: Vec<_> = history
            .iter()
            .filter(|(p, v)| p.contains("IsLocked") && !p.contains("Double") && *v == SignalValue::Bool(true))
            .collect();
        assert_eq!(locked_true.len(), 4, "double-lock should also set IsLocked=true");

        handle.abort();
        let _ = handle.await;
    }

    #[tokio::test]
    async fn unlock_clears_double_lock() {
        let bus = Arc::new(MockBus::new());
        let model = DoorLockPlantModel::new(Arc::clone(&bus));
        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // First double-lock
        for &sig in &DOOR_DOUBLE_LOCK_CMD_SIGNALS {
            bus.inject(sig, SignalValue::Bool(true));
        }
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        // Unlock should clear double-lock too
        for &sig in &DOOR_LOCK_CMD_SIGNALS {
            bus.inject(sig, SignalValue::Bool(false));
        }
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let dl_cleared: Vec<_> = history
            .iter()
            .filter(|(p, v)| p.contains("IsDoubleLocked") && *v == SignalValue::Bool(false))
            .collect();
        assert_eq!(dl_cleared.len(), 4, "unlock should clear all IsDoubleLocked");

        handle.abort();
        let _ = handle.await;
    }

    #[tokio::test]
    async fn no_duplicate_publish_if_state_unchanged() {
        let bus = Arc::new(MockBus::new());
        let model = DoorLockPlantModel::new(Arc::clone(&bus));
        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // All doors start unlocked; sending unlock cmd again should not re-publish
        bus.clear_history();
        for &sig in &DOOR_LOCK_CMD_SIGNALS {
            bus.inject(sig, SignalValue::Bool(false));
        }
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        assert_eq!(
            history.len(),
            0,
            "no publish expected when state is already correct"
        );

        handle.abort();
        let _ = handle.await;
    }
}
