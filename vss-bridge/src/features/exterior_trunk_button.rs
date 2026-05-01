//! ExteriorTrunkButton — capacitive trunk-release button above the
//! rear license plate.
//!
//! # Behaviour
//!
//! On a rising edge of `Body.Trunk.ExteriorButton.IsPressed`, branch
//! on the cached `Cabin.LockStatus` and `Cabin.ValetMode.IsActive`:
//!
//! | Cabin lock state                | Valet | Action                                       |
//! |---------------------------------|:-----:|----------------------------------------------|
//! | `UNLOCKED` / `DRIVER_UNLOCKED`  |  ❌   | Pulse trunk arbiter + `trunk_unlock` flash   |
//! | `UNLOCKED` / `DRIVER_UNLOCKED`  |  ✅   | Silently denied (log only)                   |
//! | `LOCKED` / `DOUBLE_LOCKED`      |   —   | No-op — `PassiveEntry` owns the locked path  |
//!
//! Valet mode also gates the **locked** path even though we don't act
//! on it here: the trunk arbiter's `ValetGate` `PhysicalGate` (see
//! `arbiter::trunk_arbiter`) suppresses any `Body.Trunk.OpenCmd` from
//! any feature when valet is active.  So `PassiveEntry` could
//! authenticate and pulse the arbiter and the gate would still drop
//! it.  This feature's valet check is a *short-circuit* that avoids
//! the wasted publish in the unlocked path.
//!
//! # No state, no NVM
//!
//! Stateless reader of three signals.  Safe boot defaults: lock
//! status `"LOCKED"` (so a missing-signal stance is "don't fire") and
//! valet `false` (don't deny).
//!
//! # Lock-state independence
//!
//! Trunk-only open never publishes `Cabin.LockStatus` or
//! `Cabin.LockStatus.LastRequestor`.  AutoRelock's external-source
//! filter never fires on a trunk press, and the cabin lock indicator
//! stays consistent across a "pop-trunk-while-locked" sequence.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::arbiter::{ActuatorRequest, DomainArbiter, FEEDBACK_REQUEST, TRUNK_OPEN_CMD};
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

const FEATURE_ID: FeatureId = FeatureId::ExteriorTrunkButton;

const BUTTON: VssPath = "Body.Trunk.ExteriorButton.IsPressed";
const LOCK_STATUS: VssPath = "Cabin.LockStatus";
const VALET_MODE: VssPath = "Cabin.ValetMode.IsActive";

pub struct ExteriorTrunkButton<B: SignalBus> {
    bus: Arc<B>,
    trunk_arb: Arc<DomainArbiter>,
}

impl<B: SignalBus + Send + Sync + 'static> ExteriorTrunkButton<B> {
    pub fn new(bus: Arc<B>, trunk_arb: Arc<DomainArbiter>) -> Self {
        Self { bus, trunk_arb }
    }

    pub async fn run(self) {
        tracing::info!("ExteriorTrunkButton feature started");

        let mut button_rx = self.bus.subscribe(BUTTON).await;
        let mut lock_rx = self.bus.subscribe(LOCK_STATUS).await;
        let mut valet_rx = self.bus.subscribe(VALET_MODE).await;

        // Safe defaults — see module docs.  `"LOCKED"` so a missing
        // lock-status signal at boot routes the press to PassiveEntry
        // (where it will be cleanly rejected if no fob is present)
        // rather than firing an unauthenticated trunk open.
        let mut lock_status = "LOCKED".to_string();
        let mut valet_active = false;

        loop {
            select! {
                Some(val) = button_rx.next() => {
                    if !matches!(val, SignalValue::Bool(true)) {
                        continue;
                    }
                    self.on_press(&lock_status, valet_active).await;
                }
                Some(val) = lock_rx.next() => {
                    if let SignalValue::String(s) = val {
                        lock_status = s;
                    }
                }
                Some(val) = valet_rx.next() => {
                    if let SignalValue::Bool(b) = val {
                        valet_active = b;
                    }
                }
                else => break,
            }
        }
    }

    /// Branch on lock state + valet, fire the unlocked path only when
    /// it's clearly safe to do so.
    async fn on_press(&self, lock_status: &str, valet_active: bool) {
        if valet_active {
            tracing::info!(
                lock_status,
                "ExteriorTrunkButton: press denied — valet mode active"
            );
            return;
        }

        match lock_status {
            "UNLOCKED" | "DRIVER_UNLOCKED" => {
                tracing::info!(
                    lock_status,
                    "ExteriorTrunkButton: cabin unlocked — pulsing trunk arbiter"
                );
                self.pulse_trunk_open().await;
            }
            // LOCKED, DOUBLE_LOCKED, or anything unrecognized — let
            // PassiveEntry's auth path handle it (or quietly drop on
            // its own gates if the cabin is in some unknown state).
            _ => {
                tracing::debug!(
                    lock_status,
                    "ExteriorTrunkButton: cabin locked — deferring to PassiveEntry"
                );
            }
        }
    }

    /// Pulse `Body.Trunk.OpenCmd` through the trunk arbiter as a
    /// momentary edge: request true, then immediately release so the
    /// arbiter publishes true → false and a subsequent press can
    /// re-fire.  Also fires the `trunk_unlock` lock-feedback flash so
    /// the user gets the same visual confirmation as the RKE
    /// TrunkRelease path.
    async fn pulse_trunk_open(&self) {
        let _ = self
            .trunk_arb
            .request(ActuatorRequest {
                signal: TRUNK_OPEN_CMD,
                value: SignalValue::Bool(true),
                priority: Priority::Medium,
                feature_id: FEATURE_ID,
            })
            .await;
        let _ = self.trunk_arb.release(TRUNK_OPEN_CMD, FEATURE_ID).await;

        let _ = self
            .bus
            .publish(FEEDBACK_REQUEST, SignalValue::String("trunk_unlock".into()))
            .await;
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::trunk_arbiter;

    async fn settle() {
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
    }

    /// Spin up bus + trunk arbiter + ExteriorTrunkButton.
    async fn setup() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let (tarb, tarb_fut) = trunk_arbiter(Arc::clone(&bus));
        tokio::spawn(tarb_fut);
        let tarb = Arc::new(tarb);

        let feat = ExteriorTrunkButton::new(Arc::clone(&bus), tarb);
        tokio::spawn(feat.run());
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
        bus
    }

    fn trunk_was_pulsed(bus: &MockBus) -> bool {
        bus.history()
            .into_iter()
            .any(|(s, v)| s == "Body.Trunk.OpenCmd" && v == SignalValue::Bool(true))
    }

    fn lock_status_was_published(bus: &MockBus) -> bool {
        bus.history().into_iter().any(|(s, _)| s == LOCK_STATUS)
    }

    #[tokio::test]
    async fn unlocked_press_opens_trunk_directly() {
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("UNLOCKED".into()));
        settle().await;
        bus.clear_history();

        bus.inject(BUTTON, SignalValue::Bool(true));
        settle().await;

        assert!(
            trunk_was_pulsed(&bus),
            "unlocked + valet off: press must pulse Body.Trunk.OpenCmd=true"
        );
        assert!(
            !lock_status_was_published(&bus),
            "trunk-only open must not mutate Cabin.LockStatus"
        );
    }

    #[tokio::test]
    async fn driver_unlocked_press_opens_trunk_directly() {
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("DRIVER_UNLOCKED".into()));
        settle().await;
        bus.clear_history();

        bus.inject(BUTTON, SignalValue::Bool(true));
        settle().await;

        assert!(trunk_was_pulsed(&bus));
    }

    #[tokio::test]
    async fn locked_press_is_no_op_in_this_feature() {
        // The locked path is owned by PassiveEntry — this feature must
        // NOT pulse the arbiter when the cabin is locked (otherwise
        // we'd open the trunk without auth, which is the whole bug
        // we're avoiding).
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        settle().await;
        bus.clear_history();

        bus.inject(BUTTON, SignalValue::Bool(true));
        settle().await;

        assert!(
            !trunk_was_pulsed(&bus),
            "locked: ExteriorTrunkButton must defer to PassiveEntry"
        );
    }

    #[tokio::test]
    async fn double_locked_press_is_no_op_in_this_feature() {
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("DOUBLE_LOCKED".into()));
        settle().await;
        bus.clear_history();

        bus.inject(BUTTON, SignalValue::Bool(true));
        settle().await;

        assert!(!trunk_was_pulsed(&bus));
    }

    #[tokio::test]
    async fn valet_active_unlocked_press_does_not_open_trunk() {
        // Valet must override the unlocked-direct path.
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("UNLOCKED".into()));
        bus.inject(VALET_MODE, SignalValue::Bool(true));
        settle().await;
        bus.clear_history();

        bus.inject(BUTTON, SignalValue::Bool(true));
        settle().await;

        assert!(
            !trunk_was_pulsed(&bus),
            "valet active + unlocked: press must be silently denied"
        );
    }

    #[tokio::test]
    async fn valet_deactivation_restores_unlocked_path() {
        // Valet on → blocked.  Toggle valet off → next press fires.
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("UNLOCKED".into()));
        bus.inject(VALET_MODE, SignalValue::Bool(true));
        settle().await;
        bus.inject(BUTTON, SignalValue::Bool(true));
        settle().await;
        assert!(!trunk_was_pulsed(&bus));

        // Deactivate valet, press again — pulse should fire.
        bus.inject(VALET_MODE, SignalValue::Bool(false));
        settle().await;
        bus.clear_history();
        bus.inject(BUTTON, SignalValue::Bool(true));
        settle().await;

        assert!(
            trunk_was_pulsed(&bus),
            "valet deactivated: subsequent press must fire"
        );
    }

    #[tokio::test]
    async fn falling_edge_is_ignored() {
        // Only rising edges count.  A `false` press value is a
        // button-release event and must not trigger any arbiter call.
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("UNLOCKED".into()));
        settle().await;
        bus.clear_history();

        bus.inject(BUTTON, SignalValue::Bool(false));
        settle().await;

        assert!(!trunk_was_pulsed(&bus));
    }

    #[tokio::test]
    async fn missing_lock_status_at_boot_defers_to_passive_entry() {
        // Boot default is `"LOCKED"`.  A press with no published
        // lock status must NOT fire the unlocked-direct path.
        let bus = setup().await;
        // Don't inject LOCK_STATUS at all.
        settle().await;
        bus.clear_history();

        bus.inject(BUTTON, SignalValue::Bool(true));
        settle().await;

        assert!(
            !trunk_was_pulsed(&bus),
            "boot-default lock_status='LOCKED' must defer to PassiveEntry"
        );
    }
}
