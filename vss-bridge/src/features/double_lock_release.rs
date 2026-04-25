//! Double-lock Release — clears superlock on ignition ON.
//!
//! When the vehicle is double-locked (superlock) and the ignition is turned ON,
//! driving with exterior door handles physically disconnected is a safety hazard.
//! This feature watches `Vehicle.LowVoltageSystemState` and, on a transition from
//! OFF/ACC/LOCK to ON/START, dispatches `ReleaseDouble` via the DoorLockArbiter.
//!
//! # Design notes
//! - `ReleaseDouble` keeps `IsLocked = true` on all doors — only `IsDoubleLocked`
//!   is cleared. The vehicle remains locked; only the mechanical disconnection of
//!   the door linkage is restored.
//! - No `FeedbackRequest` is published — this is an internal automatic trigger,
//!   not a user-initiated command.
//! - The feature detects the OFF→ON transition (not repeated ON states) to avoid
//!   dispatching ReleaseDouble on every ignition signal publication.

use std::sync::Arc;

use futures::StreamExt;

use crate::arbiter::{DoorLockArbiter, DoorLockRequest, LockCommand};
use crate::ipc_message::{FeatureId, SignalValue};
use crate::signal_bus::SignalBus;

const POWER_STATE_SIGNAL: &str = "Vehicle.LowVoltageSystemState";

fn is_power_off(val: &SignalValue) -> bool {
    matches!(val, SignalValue::String(s) if s == "OFF" || s == "ACC" || s == "LOCK")
}

fn is_power_on(val: &SignalValue) -> bool {
    matches!(val, SignalValue::String(s) if s == "ON" || s == "START")
}

pub struct DoubleLockRelease<B: SignalBus> {
    bus: Arc<B>,
    arbiter: Arc<DoorLockArbiter>,
}

impl<B: SignalBus + Send + Sync + 'static> DoubleLockRelease<B> {
    pub fn new(bus: Arc<B>, arbiter: Arc<DoorLockArbiter>) -> Self {
        Self { bus, arbiter }
    }

    pub async fn run(self) {
        let mut power_rx = self.bus.subscribe(POWER_STATE_SIGNAL).await;

        // Assume ignition was off at boot (safe default — avoids spurious release
        // if bus replays an ON value at startup before any OFF is seen).
        let mut last_was_off = true;

        tracing::info!("DoubleLockRelease feature started");

        while let Some(val) = power_rx.next().await {
            if is_power_off(&val) {
                last_was_off = true;
            } else if is_power_on(&val) {
                if last_was_off {
                    // Detected OFF → ON transition: release superlock
                    tracing::info!("DoubleLockRelease: ignition ON — dispatching release_double");
                    if let Err(e) = self
                        .arbiter
                        .request(DoorLockRequest {
                            command: LockCommand::ReleaseDouble,
                            feature_id: FeatureId::DoubleLockRelease,
                        })
                        .await
                    {
                        tracing::error!(error = %e, "DoubleLockRelease: arbiter error");
                    }
                }
                last_was_off = false;
            }
        }

        tracing::info!("DoubleLockRelease feature stopped");
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::door_lock_arbiter;
    use tokio::time::{sleep, Duration};

    async fn setup() -> (Arc<MockBus>, tokio::task::JoinHandle<()>) {
        let bus = Arc::new(MockBus::new());
        let (arb, _ack_tx, loop_fut) = door_lock_arbiter(Arc::clone(&bus));
        tokio::spawn(loop_fut);
        let arb = Arc::new(arb);
        let feature = DoubleLockRelease::new(Arc::clone(&bus), arb);
        let handle = tokio::spawn(feature.run());
        tokio::task::yield_now().await;
        (bus, handle)
    }

    #[tokio::test]
    async fn off_to_on_dispatches_release_double() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE_SIGNAL, SignalValue::String("OFF".to_string()));
        tokio::task::yield_now().await;

        bus.clear_history();

        bus.inject(POWER_STATE_SIGNAL, SignalValue::String("ON".to_string()));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(10)).await;
        tokio::task::yield_now().await;

        let history = bus.history();
        assert!(
            history
                .iter()
                .any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                    && *v == SignalValue::String("release_double".into())),
            "expected release_double command, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn repeated_on_without_off_does_not_re_dispatch() {
        let (bus, _h) = setup().await;

        // Boot with last_was_off = true → first ON triggers release
        bus.inject(POWER_STATE_SIGNAL, SignalValue::String("ON".to_string()));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(10)).await;
        tokio::task::yield_now().await;

        bus.clear_history();

        // Second ON without an OFF in between — should NOT dispatch again
        bus.inject(POWER_STATE_SIGNAL, SignalValue::String("ON".to_string()));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(10)).await;
        tokio::task::yield_now().await;

        let history = bus.history();
        assert!(
            !history
                .iter()
                .any(|(s, _)| *s == "Body.Doors.CentralLock.Command"),
            "second ON without OFF should NOT dispatch release_double, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn acc_between_off_and_on_still_triggers() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE_SIGNAL, SignalValue::String("OFF".to_string()));
        tokio::task::yield_now().await;

        bus.inject(POWER_STATE_SIGNAL, SignalValue::String("ACC".to_string()));
        tokio::task::yield_now().await;

        bus.clear_history();

        bus.inject(POWER_STATE_SIGNAL, SignalValue::String("ON".to_string()));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(10)).await;
        tokio::task::yield_now().await;

        let history = bus.history();
        assert!(
            history
                .iter()
                .any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                    && *v == SignalValue::String("release_double".into())),
            "OFF → ACC → ON should dispatch release_double, history: {:?}",
            history
        );
    }
}
