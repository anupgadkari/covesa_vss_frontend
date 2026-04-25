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

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::ipc_message::SignalValue;
use crate::signal_bus::SignalBus;

const OPEN_CMD: &str = "Body.Trunk.OpenCmd";
const CLOSE_CMD: &str = "Body.Trunk.CloseCmd";
const IS_OPEN: &str = "Body.Trunk.IsOpen";

pub struct TrunkPlantModel<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> TrunkPlantModel<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    pub async fn run(self) {
        let mut open_rx = self.bus.subscribe(OPEN_CMD).await;
        let mut close_rx = self.bus.subscribe(CLOSE_CMD).await;

        let _ = self.bus.publish(IS_OPEN, SignalValue::Bool(false)).await;
        tracing::info!("TrunkPlantModel started");

        loop {
            select! {
                Some(val) = open_rx.next() => {
                    if matches!(val, SignalValue::Bool(true)) {
                        tracing::info!("Trunk: OpenCmd received — trunk open");
                        let _ = self.bus.publish(IS_OPEN, SignalValue::Bool(true)).await;
                    }
                }
                Some(val) = close_rx.next() => {
                    if matches!(val, SignalValue::Bool(true)) {
                        tracing::info!("Trunk: CloseCmd received — trunk closed");
                        let _ = self.bus.publish(IS_OPEN, SignalValue::Bool(false)).await;
                    }
                }
            }
        }
    }
}
