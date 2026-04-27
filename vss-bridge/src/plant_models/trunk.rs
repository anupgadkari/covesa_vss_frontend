//! Trunk latch plant model.
//!
//! Simulates the power-latch actuator on the trunk/tailgate:
//!
//! ```text
//!  RKE feature (double TRUNK_RELEASE press)
//!      │  Body.Trunk.OpenCmd
//!      ▼
//!  TrunkPlantModel          ← this module
//!      │  publishes Body.Trunk.IsOpen
//!      ▼
//!  SignalBus → WsBridge → HMI
//! ```
//!
//! The trunk is independent of the cabin door lock domain — opening the
//! trunk does not affect door lock state.  Closing is manual (or via HMI
//! `Body.Trunk.CloseCmd`).
//!
//! # NVM persistence (optional)
//!
//! When constructed via [`TrunkPlantModel::with_nvm`], the trunk's
//! open/closed state is persisted on every change and re-read at boot.
//! This makes "vehicle parked with trunk open, restart bridge, trunk is
//! still open" reproducible end-to-end and keeps the plant-model state
//! consistent across HMI reloads.  See
//! `docs/signal-ownership-and-state-hydration.md`.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::ipc_message::SignalValue;
use crate::nvm::{NvmStore, TrunkState};
use crate::signal_bus::SignalBus;

const OPEN_CMD: &str = "Body.Trunk.OpenCmd";
const CLOSE_CMD: &str = "Body.Trunk.CloseCmd";
const IS_OPEN: &str = "Body.Trunk.IsOpen";

pub struct TrunkPlantModel<B: SignalBus> {
    bus: Arc<B>,
    /// Current open state — kept in sync with NVM when configured.
    is_open: bool,
    /// Optional NVM store; when set, every change is persisted.
    nvm: Option<NvmStore>,
}

impl<B: SignalBus + Send + Sync + 'static> TrunkPlantModel<B> {
    /// Create in volatile mode — no NVM, boots closed.
    pub fn new(bus: Arc<B>) -> Self {
        Self {
            bus,
            is_open: false,
            nvm: None,
        }
    }

    /// Production constructor — boots from NVM and persists on every
    /// change.  Used by `main.rs`.
    pub fn with_nvm(bus: Arc<B>, nvm: NvmStore) -> Self {
        let persisted = nvm.load_trunk();
        tracing::info!(
            is_open = persisted.is_open,
            "TrunkPlantModel: booted from NVM"
        );
        Self {
            bus,
            is_open: persisted.is_open,
            nvm: Some(nvm),
        }
    }

    fn save_to_nvm(&self) {
        if let Some(nvm) = &self.nvm {
            nvm.save_trunk(&TrunkState {
                is_open: self.is_open,
            });
        }
    }

    pub async fn run(mut self) {
        let mut open_rx = self.bus.subscribe(OPEN_CMD).await;
        let mut close_rx = self.bus.subscribe(CLOSE_CMD).await;

        // Publish boot state.  This satisfies the bridge's ESSENTIAL_BOOT
        // gate for `Body.Trunk.IsOpen` so a fresh HMI client sees the
        // right value (factory default OR NVM-persisted) without flash.
        let _ = self
            .bus
            .publish(IS_OPEN, SignalValue::Bool(self.is_open))
            .await;
        tracing::info!(is_open = self.is_open, "TrunkPlantModel started");

        loop {
            select! {
                Some(val) = open_rx.next() => {
                    if matches!(val, SignalValue::Bool(true)) && !self.is_open {
                        tracing::info!("Trunk: OpenCmd received — trunk open");
                        self.is_open = true;
                        let _ = self.bus.publish(IS_OPEN, SignalValue::Bool(true)).await;
                        self.save_to_nvm();
                    }
                }
                Some(val) = close_rx.next() => {
                    if matches!(val, SignalValue::Bool(true)) && self.is_open {
                        tracing::info!("Trunk: CloseCmd received — trunk closed");
                        self.is_open = false;
                        let _ = self.bus.publish(IS_OPEN, SignalValue::Bool(false)).await;
                        self.save_to_nvm();
                    }
                }
            }
        }
    }
}
