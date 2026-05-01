//! Hood plant model.
//!
//! Mechanical (manually operated) hood with a sensor-style on/off
//! `IsOpen` signal:
//!
//! ```text
//!  HMI hood toggle / future hood-release lever sensor
//!      │  Body.Hood.OpenCmd / Body.Hood.CloseCmd
//!      ▼
//!  HoodPlantModel          ← this module
//!      │  publishes Body.Hood.IsOpen
//!      ▼
//!  SignalBus → WsBridge → HMI
//! ```
//!
//! Mirrors the Trunk plant-model pattern exactly (momentary commands +
//! NVM-persisted state) so a future power-hood (rare but extant on
//! some prestige SUVs) is a plant-model swap with no feature-side
//! changes.
//!
//! # NVM persistence (optional)
//!
//! When constructed via [`HoodPlantModel::with_nvm`], the hood's
//! open/closed state is persisted on every change and re-read at boot.
//! "Vehicle parked with hood up, restart bridge, hood is still up" is
//! reproducible end-to-end.  See
//! `docs/signal-ownership-and-state-hydration.md`.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::ipc_message::SignalValue;
use crate::nvm::{HoodState, NvmStore};
use crate::signal_bus::SignalBus;

const OPEN_CMD: &str = "Body.Hood.OpenCmd";
const CLOSE_CMD: &str = "Body.Hood.CloseCmd";
const IS_OPEN: &str = "Body.Hood.IsOpen";

pub struct HoodPlantModel<B: SignalBus> {
    bus: Arc<B>,
    /// Current open state — kept in sync with NVM when configured.
    is_open: bool,
    /// Optional NVM store; when set, every change is persisted.
    nvm: Option<NvmStore>,
}

impl<B: SignalBus + Send + Sync + 'static> HoodPlantModel<B> {
    /// Create in volatile mode — no NVM, boots closed.
    pub fn new(bus: Arc<B>) -> Self {
        Self {
            bus,
            is_open: false,
            nvm: None,
        }
    }

    /// Production constructor — boots from NVM and persists on every
    /// change.
    pub fn with_nvm(bus: Arc<B>, nvm: NvmStore) -> Self {
        let persisted = nvm.load_hood();
        tracing::info!(
            is_open = persisted.is_open,
            "HoodPlantModel: booted from NVM"
        );
        Self {
            bus,
            is_open: persisted.is_open,
            nvm: Some(nvm),
        }
    }

    fn save_to_nvm(&self) {
        if let Some(nvm) = &self.nvm {
            nvm.save_hood(&HoodState {
                is_open: self.is_open,
            });
        }
    }

    pub async fn run(mut self) {
        let mut open_rx = self.bus.subscribe(OPEN_CMD).await;
        let mut close_rx = self.bus.subscribe(CLOSE_CMD).await;

        // Publish boot state — fresh HMI clients see the right value
        // (factory default OR NVM-persisted) without flicker.
        let _ = self
            .bus
            .publish(IS_OPEN, SignalValue::Bool(self.is_open))
            .await;
        tracing::info!(is_open = self.is_open, "HoodPlantModel started");

        loop {
            select! {
                Some(val) = open_rx.next() => {
                    if matches!(val, SignalValue::Bool(true)) && !self.is_open {
                        tracing::info!("Hood: OpenCmd received — hood open");
                        self.is_open = true;
                        let _ = self.bus.publish(IS_OPEN, SignalValue::Bool(true)).await;
                        self.save_to_nvm();
                    }
                }
                Some(val) = close_rx.next() => {
                    if matches!(val, SignalValue::Bool(true)) && self.is_open {
                        tracing::info!("Hood: CloseCmd received — hood closed");
                        self.is_open = false;
                        let _ = self.bus.publish(IS_OPEN, SignalValue::Bool(false)).await;
                        self.save_to_nvm();
                    }
                }
            }
        }
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use tempfile::TempDir;

    async fn settle() {
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    #[tokio::test]
    async fn open_cmd_opens_hood() {
        let bus = Arc::new(MockBus::new());
        let plant = HoodPlantModel::new(Arc::clone(&bus));
        tokio::spawn(plant.run());
        settle().await;

        bus.inject(OPEN_CMD, SignalValue::Bool(true));
        settle().await;

        assert_eq!(bus.latest_value(IS_OPEN), Some(SignalValue::Bool(true)));
    }

    #[tokio::test]
    async fn close_cmd_closes_hood() {
        let bus = Arc::new(MockBus::new());
        let plant = HoodPlantModel::new(Arc::clone(&bus));
        tokio::spawn(plant.run());
        settle().await;

        bus.inject(OPEN_CMD, SignalValue::Bool(true));
        settle().await;
        assert_eq!(bus.latest_value(IS_OPEN), Some(SignalValue::Bool(true)));

        bus.inject(CLOSE_CMD, SignalValue::Bool(true));
        settle().await;
        assert_eq!(bus.latest_value(IS_OPEN), Some(SignalValue::Bool(false)));
    }

    #[tokio::test]
    async fn open_cmd_when_already_open_is_idempotent() {
        let bus = Arc::new(MockBus::new());
        let plant = HoodPlantModel::new(Arc::clone(&bus));
        tokio::spawn(plant.run());
        settle().await;

        bus.inject(OPEN_CMD, SignalValue::Bool(true));
        settle().await;
        let baseline = bus
            .history()
            .into_iter()
            .filter(|(s, _)| *s == IS_OPEN)
            .count();

        // Repeat — should not re-publish.
        bus.inject(OPEN_CMD, SignalValue::Bool(true));
        settle().await;
        let after = bus
            .history()
            .into_iter()
            .filter(|(s, _)| *s == IS_OPEN)
            .count();
        assert_eq!(baseline, after, "duplicate OpenCmd must not re-publish");
    }

    #[tokio::test]
    async fn nvm_round_trip() {
        let dir = TempDir::new().unwrap();
        let nvm = NvmStore::with_path(dir.path());
        let bus = Arc::new(MockBus::new());

        // First lifecycle: boot, open, persist.
        let plant = HoodPlantModel::with_nvm(Arc::clone(&bus), nvm.clone());
        let h = tokio::spawn(plant.run());
        settle().await;
        bus.inject(OPEN_CMD, SignalValue::Bool(true));
        settle().await;
        h.abort();

        // Second lifecycle: same NVM, fresh bus.  Boot must reflect persisted "open".
        let bus2 = Arc::new(MockBus::new());
        let plant2 = HoodPlantModel::with_nvm(Arc::clone(&bus2), nvm);
        tokio::spawn(plant2.run());
        settle().await;
        assert_eq!(bus2.latest_value(IS_OPEN), Some(SignalValue::Bool(true)));
    }
}
