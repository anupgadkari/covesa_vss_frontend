//! Door lock plant model — simulates the M7 Classic AUTOSAR door lock SWC.
//!
//! In production, the RKE / PEPS feature logic sends a high-level lock command
//! to the M7 via the DoorLockArbiter. The M7 Classic AUTOSAR application layer
//! SWC actuates the door lock motors and publishes the confirmed per-door state
//! back to the A53 via a `STATE_UPDATE` IPC message.
//!
//! This plant model fills that M7 role during development:
//!
//! ```text
//!  RKE feature
//!      │  DoorLockArbiter request
//!      ▼
//!  DoorLockArbiter
//!      │  publishes Body.Doors.CentralLock.Command (String)
//!      │  values: "unlock_driver" | "unlock_all" | "lock_all" | "lock_double"
//!      ▼
//!  DoorLockPlantModel          ← this module
//!      │  publishes IsLocked / IsDoubleLocked / Soldier.IsUnlocked per door
//!      ▼
//!  SignalBus → WsBridge → HMI
//! ```
//!
//! # Signals consumed (from arbiter)
//! | Signal | Value | Meaning |
//! |--------|-------|---------|
//! | `Body.Doors.CentralLock.Command` | `"unlock_driver"` | Unlock driver door only (stage 1) |
//! | `Body.Doors.CentralLock.Command` | `"unlock_all"`    | Unlock all doors |
//! | `Body.Doors.CentralLock.Command` | `"lock_all"`      | Lock all doors |
//! | `Body.Doors.CentralLock.Command` | `"lock_double"`   | Superlock all doors |
//!
//! # Signals published (confirmed state)
//! | Signal | Value | Meaning |
//! |--------|-------|---------|
//! | `Body.Doors.Row*.*.IsLocked`          | Bool | Door is locked |
//! | `Body.Doors.Row*.*.IsDoubleLocked`    | Bool | Door superlock engaged |
//! | `Body.Doors.Row*.*.Soldier.IsUnlocked`| Bool | Interior knob mirrors actuator |

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;
use tokio::sync::mpsc;

use crate::arbiter::{LockAck, CENTRAL_LOCK_CMD};
use crate::ipc_message::SignalValue;
use crate::signal_bus::SignalBus;

/// VSS signal paths for confirmed door lock state (published by this plant model).
const DOOR_LOCKED_SIGNALS: [&str; 4] = [
    "Body.Doors.Row1.Left.IsLocked",
    "Body.Doors.Row1.Right.IsLocked",
    "Body.Doors.Row2.Left.IsLocked",
    "Body.Doors.Row2.Right.IsLocked",
];

/// VSS signal paths for the interior soldier knob (mirrors central lock state).
/// When the actuator locks/unlocks, the knob moves with it.
const DOOR_SOLDIER_SIGNALS: [&str; 4] = [
    "Body.Doors.Row1.Left.Soldier.IsUnlocked",
    "Body.Doors.Row1.Right.Soldier.IsUnlocked",
    "Body.Doors.Row2.Left.Soldier.IsUnlocked",
    "Body.Doors.Row2.Right.Soldier.IsUnlocked",
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

/// All door indices — used for commands that affect every door.
const ALL_DOORS: [usize; 4] = [ROW1_LEFT, ROW1_RIGHT, ROW2_LEFT, ROW2_RIGHT];

/// Driver door index — Row 1 Left by default (dealer-configurable in production).
/// Used for two-stage unlock stage 1: only this door is released on first press.
const DRIVER_DOOR: usize = ROW1_LEFT;

/// The three doors that are NOT the driver door.
/// When a double-locked vehicle receives `unlock_driver`, these doors stay
/// locked but their superlock is downgraded to normal lock — the security
/// perimeter is broken the moment the driver door is opened.
const PASSENGER_DOORS: [usize; 3] = [ROW1_RIGHT, ROW2_LEFT, ROW2_RIGHT];

/// Door lock plant model. Spawn with `.run()`.
///
/// In production the M7 Classic AUTOSAR Locking SWC sends a `LockAck`
/// after the door motors finish actuating (~300 ms). This plant model
/// simulates that: it sends a `LockAck` after processing each command
/// signal so the `DoorLockArbiter` queue doesn't stall.
///
/// Use [`DoorLockPlantModel::new`] in tests (no-ack mode) and
/// [`DoorLockPlantModel::with_ack_tx`] in `main` (wired to the arbiter).
pub struct DoorLockPlantModel<B: SignalBus> {
    bus: Arc<B>,
    /// Optional ACK channel back to the DoorLockArbiter.
    /// If `None`, no ACKs are sent (test / standalone mode).
    ack_tx: Option<mpsc::Sender<LockAck>>,
    /// Confirmed locked state per door.
    locked: [bool; 4],
    /// Confirmed double-locked state per door.
    double_locked: [bool; 4],
    /// Monotonic counter for LockAck event numbers.
    event_number: u32,
}

impl<B: SignalBus + Send + Sync + 'static> DoorLockPlantModel<B> {
    /// Create in standalone / test mode — no ACKs sent to any arbiter.
    pub fn new(bus: Arc<B>) -> Self {
        Self {
            bus,
            ack_tx: None,
            locked: [false; 4],
            double_locked: [false; 4],
            event_number: 0,
        }
    }

    /// Create wired to the `DoorLockArbiter`'s ACK channel.
    ///
    /// Pass the `mpsc::Sender<LockAck>` returned by `door_lock_arbiter()`.
    /// The plant model will send one `LockAck` after processing each command
    /// signal, allowing the arbiter to dequeue and dispatch the next pending
    /// request.
    pub fn with_ack_tx(bus: Arc<B>, ack_tx: mpsc::Sender<LockAck>) -> Self {
        Self {
            bus,
            ack_tx: Some(ack_tx),
            locked: [false; 4],
            double_locked: [false; 4],
            event_number: 0,
        }
    }

    /// Send a `LockAck` to the arbiter (no-op if no ACK channel configured).
    async fn send_ack(&mut self) {
        if let Some(tx) = &self.ack_tx {
            self.event_number = self.event_number.wrapping_add(1);
            let ack = LockAck {
                event_number: self.event_number,
                door_results: [true; 4],
            };
            if tx.send(ack).await.is_err() {
                tracing::warn!("DoorLock plant: ACK channel closed — arbiter may be gone");
            }
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
        // Soldier knob mirrors the central actuator — moves with it.
        let _ = self
            .bus
            .publish(DOOR_SOLDIER_SIGNALS[door], SignalValue::Bool(!locked))
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
                .publish(DOOR_SOLDIER_SIGNALS[i], SignalValue::Bool(!self.locked[i]))
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

    pub async fn run(mut self) {
        // Single command subscription — arbiter writes one high-level token.
        let mut cmd_rx = self.bus.subscribe(CENTRAL_LOCK_CMD).await;

        // Mirror subscriptions — keep self.locked / self.double_locked in sync
        // with whatever is actually on the bus.  This matters when the HMI (or
        // a test harness) manually overrides Body.Doors.*.IsLocked or
        // Soldier.IsUnlocked without going through a command.  Without these
        // subscriptions the dedup guard in apply_lock_cmd would believe the door
        // is still locked and silently skip the re-publish, leaving the manually-
        // flipped soldier in the wrong position after a LOCK command.
        let mut islocked_rx0 = self.bus.subscribe(DOOR_LOCKED_SIGNALS[0]).await;
        let mut islocked_rx1 = self.bus.subscribe(DOOR_LOCKED_SIGNALS[1]).await;
        let mut islocked_rx2 = self.bus.subscribe(DOOR_LOCKED_SIGNALS[2]).await;
        let mut islocked_rx3 = self.bus.subscribe(DOOR_LOCKED_SIGNALS[3]).await;
        let mut isdblocked_rx0 = self.bus.subscribe(DOOR_DOUBLE_LOCKED_SIGNALS[0]).await;
        let mut isdblocked_rx1 = self.bus.subscribe(DOOR_DOUBLE_LOCKED_SIGNALS[1]).await;
        let mut isdblocked_rx2 = self.bus.subscribe(DOOR_DOUBLE_LOCKED_SIGNALS[2]).await;
        let mut isdblocked_rx3 = self.bus.subscribe(DOOR_DOUBLE_LOCKED_SIGNALS[3]).await;

        // Publish initial state (all unlocked).
        self.publish_all().await;
        tracing::info!("DoorLock plant model started — all doors unlocked");

        loop {
            select! {
                Some(val) = cmd_rx.next() => {
                    let token = match &val {
                        SignalValue::String(s) => s.as_str(),
                        _ => continue,
                    };

                    match token {
                        "unlock_driver" => {
                            // Stage-1 two-stage unlock: driver door only.
                            self.apply_lock_cmd(DRIVER_DOOR, false).await;
                            // If the vehicle was double-locked, downgrade the remaining
                            // 3 doors from superlock to normal lock. The security
                            // perimeter is broken once the driver door is released —
                            // keeping superlock on the other doors would block interior
                            // release handles, which is a safety violation.
                            for door in PASSENGER_DOORS {
                                self.apply_double_lock_cmd(door, false).await;
                            }
                            tracing::info!("DoorLock plant: unlock_driver (superlock downgraded on passenger doors)");
                            self.send_ack().await;
                        }
                        "unlock_all" => {
                            for door in ALL_DOORS {
                                self.apply_lock_cmd(door, false).await;
                            }
                            tracing::info!("DoorLock plant: unlock_all");
                            self.send_ack().await;
                        }
                        "lock_all" => {
                            for door in ALL_DOORS {
                                self.apply_lock_cmd(door, true).await;
                            }
                            tracing::info!("DoorLock plant: lock_all");
                            self.send_ack().await;
                        }
                        "lock_double" => {
                            for door in ALL_DOORS {
                                self.apply_double_lock_cmd(door, true).await;
                            }
                            tracing::info!("DoorLock plant: lock_double (superlock)");
                            self.send_ack().await;
                        }
                        "release_double" => {
                            // Ignition-ON downgrade: clear superlock on all doors
                            // while preserving IsLocked. No feedback flash.
                            for door in ALL_DOORS {
                                self.apply_double_lock_cmd(door, false).await;
                            }
                            tracing::info!("DoorLock plant: release_double (ignition ON, superlock cleared)");
                            self.send_ack().await;
                        }
                        other => {
                            tracing::warn!(cmd = other, "DoorLock plant: unknown command — ignored");
                        }
                    }
                }

                // ── Mirror: sync self.locked from bus ──────────────────────
                // When the HMI overrides Body.Doors.*.IsLocked directly (e.g.
                // to simulate a defeated lock), update internal state so the
                // next LOCK command correctly re-asserts the locked position.
                // The feedback of our own publishes is harmless (same value).
                Some(val) = islocked_rx0.next() => {
                    if let SignalValue::Bool(b) = val { self.locked[0] = b; }
                }
                Some(val) = islocked_rx1.next() => {
                    if let SignalValue::Bool(b) = val { self.locked[1] = b; }
                }
                Some(val) = islocked_rx2.next() => {
                    if let SignalValue::Bool(b) = val { self.locked[2] = b; }
                }
                Some(val) = islocked_rx3.next() => {
                    if let SignalValue::Bool(b) = val { self.locked[3] = b; }
                }
                Some(val) = isdblocked_rx0.next() => {
                    if let SignalValue::Bool(b) = val { self.double_locked[0] = b; }
                }
                Some(val) = isdblocked_rx1.next() => {
                    if let SignalValue::Bool(b) = val { self.double_locked[1] = b; }
                }
                Some(val) = isdblocked_rx2.next() => {
                    if let SignalValue::Bool(b) = val { self.double_locked[2] = b; }
                }
                Some(val) = isdblocked_rx3.next() => {
                    if let SignalValue::Bool(b) = val { self.double_locked[3] = b; }
                }

                else => break,
            }
        }
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;

    async fn run_one_tick(_bus: &Arc<MockBus>, model: DoorLockPlantModel<MockBus>) {
        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;
        handle.abort();
        let _ = handle.await;
    }

    fn cmd(bus: &Arc<MockBus>, token: &str) {
        bus.inject(CENTRAL_LOCK_CMD, SignalValue::String(token.into()));
    }

    #[tokio::test]
    async fn initial_state_all_unlocked() {
        let bus = Arc::new(MockBus::new());
        let model = DoorLockPlantModel::new(Arc::clone(&bus));
        run_one_tick(&bus, model).await;

        let history = bus.history();
        // 4 IsLocked + 4 Soldier + 4 IsDoubleLocked = 12 initial publishes
        let locked_signals: Vec<_> = history
            .iter()
            .filter(|(p, _)| p.contains("IsLocked") && !p.contains("Double"))
            .collect();
        assert_eq!(locked_signals.len(), 4, "should publish 4 IsLocked signals");
        for (_, val) in &locked_signals {
            assert_eq!(
                *val,
                SignalValue::Bool(false),
                "initial state should be unlocked"
            );
        }
    }

    #[tokio::test]
    async fn lock_all_locks_all_doors() {
        let bus = Arc::new(MockBus::new());
        let model = DoorLockPlantModel::new(Arc::clone(&bus));
        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();
        cmd(&bus, "lock_all");
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let locked_true: Vec<_> = history
            .iter()
            .filter(|(p, v)| {
                p.contains("IsLocked") && !p.contains("Double") && *v == SignalValue::Bool(true)
            })
            .collect();
        assert_eq!(
            locked_true.len(),
            4,
            "all 4 doors should report IsLocked=true"
        );

        handle.abort();
        let _ = handle.await;
    }

    #[tokio::test]
    async fn unlock_all_unlocks_all_doors() {
        let bus = Arc::new(MockBus::new());
        let model = DoorLockPlantModel::new(Arc::clone(&bus));
        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        cmd(&bus, "lock_all");
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();
        cmd(&bus, "unlock_all");
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let unlocked: Vec<_> = history
            .iter()
            .filter(|(p, v)| {
                p.contains("IsLocked") && !p.contains("Double") && *v == SignalValue::Bool(false)
            })
            .collect();
        assert_eq!(
            unlocked.len(),
            4,
            "all 4 doors should report IsLocked=false"
        );

        handle.abort();
        let _ = handle.await;
    }

    #[tokio::test]
    async fn unlock_driver_only_releases_driver_door() {
        let bus = Arc::new(MockBus::new());
        let model = DoorLockPlantModel::new(Arc::clone(&bus));
        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        cmd(&bus, "lock_all");
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();
        cmd(&bus, "unlock_driver");
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        // Only driver door (Row1.Left) should be unlocked
        let unlocked: Vec<_> = history
            .iter()
            .filter(|(p, v)| {
                p.contains("IsLocked") && !p.contains("Double") && *v == SignalValue::Bool(false)
            })
            .collect();
        assert_eq!(unlocked.len(), 1, "only driver door should be unlocked");
        assert!(
            unlocked[0].0.contains("Row1.Left"),
            "driver door is Row1.Left"
        );

        handle.abort();
        let _ = handle.await;
    }

    #[tokio::test]
    async fn lock_double_sets_both_signals() {
        let bus = Arc::new(MockBus::new());
        let model = DoorLockPlantModel::new(Arc::clone(&bus));
        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();
        cmd(&bus, "lock_double");
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();

        let dl_true: Vec<_> = history
            .iter()
            .filter(|(p, v)| p.contains("IsDoubleLocked") && *v == SignalValue::Bool(true))
            .collect();
        assert_eq!(
            dl_true.len(),
            4,
            "double-lock should set all 4 IsDoubleLocked=true"
        );

        let locked_true: Vec<_> = history
            .iter()
            .filter(|(p, v)| {
                p.contains("IsLocked") && !p.contains("Double") && *v == SignalValue::Bool(true)
            })
            .collect();
        assert_eq!(
            locked_true.len(),
            4,
            "double-lock should also set IsLocked=true"
        );

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

        cmd(&bus, "lock_double");
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();
        cmd(&bus, "unlock_all");
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let dl_cleared: Vec<_> = history
            .iter()
            .filter(|(p, v)| p.contains("IsDoubleLocked") && *v == SignalValue::Bool(false))
            .collect();
        assert_eq!(
            dl_cleared.len(),
            4,
            "unlock should clear all IsDoubleLocked"
        );

        handle.abort();
        let _ = handle.await;
    }

    /// When a double-locked vehicle receives `unlock_driver`:
    /// - Driver door → unlocked (IsLocked=false, IsDoubleLocked=false)
    /// - Passenger doors → stay locked (IsLocked=true) but lose superlock (IsDoubleLocked=false)
    #[tokio::test]
    async fn driver_unlock_on_double_locked_vehicle_downgrades_passenger_superlock() {
        let bus = Arc::new(MockBus::new());
        let model = DoorLockPlantModel::new(Arc::clone(&bus));
        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Put vehicle into double-lock state
        cmd(&bus, "lock_double");
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();
        cmd(&bus, "unlock_driver");
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();

        // Driver door (Row1.Left) must be unlocked
        let driver_unlocked = history.iter().any(|(p, v)| {
            p.contains("Row1.Left")
                && p.contains("IsLocked")
                && !p.contains("Double")
                && *v == SignalValue::Bool(false)
        });
        assert!(driver_unlocked, "driver door (Row1.Left) must be unlocked");

        // Passenger doors must remain locked
        for path in &["Row1.Right", "Row2.Left", "Row2.Right"] {
            let still_locked = !history.iter().any(|(p, v)| {
                p.contains(path)
                    && p.contains("IsLocked")
                    && !p.contains("Double")
                    && *v == SignalValue::Bool(false)
            });
            assert!(
                still_locked,
                "{path} must stay locked (IsLocked must not become false)"
            );
        }

        // All 4 IsDoubleLocked must be cleared
        let dl_cleared: Vec<_> = history
            .iter()
            .filter(|(p, v)| p.contains("IsDoubleLocked") && *v == SignalValue::Bool(false))
            .collect();
        assert_eq!(
            dl_cleared.len(),
            4,
            "all 4 doors should have IsDoubleLocked cleared"
        );

        handle.abort();
        let _ = handle.await;
    }

    /// When the HMI directly overrides IsLocked (e.g. simulating a defeated
    /// lock sensor), the plant model must re-assert the correct state on the
    /// next LOCK command.  Without the mirror subscriptions the dedup guard
    /// would suppress the re-publish and the soldier would stay in the wrong
    /// position.
    #[tokio::test]
    async fn lock_command_reasserts_after_hmi_override() {
        let bus = Arc::new(MockBus::new());
        let model = DoorLockPlantModel::new(Arc::clone(&bus));
        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Put vehicle into locked state.
        cmd(&bus, "lock_all");
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Simulate HMI user flipping Row1.Left.IsLocked to false
        // (and soldier to IsUnlocked=true) — bypassing the arbiter.
        bus.inject("Body.Doors.Row1.Left.IsLocked", SignalValue::Bool(false));
        bus.inject(
            "Body.Doors.Row1.Left.Soldier.IsUnlocked",
            SignalValue::Bool(true),
        );
        tokio::task::yield_now().await;
        tokio::time::sleep(tokio::time::Duration::from_millis(5)).await;
        tokio::task::yield_now().await;

        bus.clear_history();

        // Issue a fresh LOCK command — plant model must re-publish IsLocked=true
        // and Soldier.IsUnlocked=false for the overridden door.
        cmd(&bus, "lock_all");
        tokio::task::yield_now().await;
        tokio::time::sleep(tokio::time::Duration::from_millis(5)).await;
        tokio::task::yield_now().await;

        let history = bus.history();
        assert!(
            history.iter().any(
                |(p, v)| *p == "Body.Doors.Row1.Left.IsLocked" && *v == SignalValue::Bool(true)
            ),
            "LOCK command must re-publish IsLocked=true after HMI override; history: {:?}",
            history
        );
        assert!(
            history
                .iter()
                .any(|(p, v)| *p == "Body.Doors.Row1.Left.Soldier.IsUnlocked"
                    && *v == SignalValue::Bool(false)),
            "LOCK command must reset Soldier.IsUnlocked=false after HMI override; history: {:?}",
            history
        );

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

        // All doors start unlocked; sending unlock_all again should not re-publish
        bus.clear_history();
        cmd(&bus, "unlock_all");
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
