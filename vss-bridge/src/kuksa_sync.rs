//! KuksaSync — bidirectional bridge between the internal SignalBus and
//! the kuksa.val databroker (gRPC).
//!
//! INBOUND:  Subscribe to all Body.*/Cabin.*/Vehicle.* signals in kuksa.val
//!           and forward value changes to the local SignalBus.
//! OUTBOUND: Relay actuator state updates from the SignalBus to kuksa.val
//!           via Set RPCs.
//!
//! Reconnects with exponential backoff (1 s → 2 s → 4 s → … → 30 s cap).

use std::sync::Arc;
use std::time::Duration;

use futures::StreamExt;
use tokio::time::sleep;
use tonic::transport::Channel;

use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};
use crate::signal_ids::ALL_SIGNALS;

/// Re-export the generated kuksa.val gRPC types.
pub mod proto {
    tonic::include_proto!("kuksa.val.v1");
}

use proto::val_client::ValClient;
use proto::{datapoint, DataEntry, Datapoint, EntryUpdate, Field, SubscribeEntry, SubscribeRequest, View};

const INITIAL_BACKOFF: Duration = Duration::from_secs(1);
const MAX_BACKOFF: Duration = Duration::from_secs(30);

/// Manages the connection to the kuksa.val databroker.
pub struct KuksaSync<B: SignalBus> {
    endpoint: String,
    bus: Arc<B>,
}

impl<B: SignalBus> KuksaSync<B> {
    pub fn new(endpoint: &str, bus: Arc<B>) -> Self {
        Self {
            endpoint: endpoint.to_owned(),
            bus,
        }
    }

    /// Main loop: connect, subscribe, and relay. Reconnects on failure.
    pub async fn run(&self) {
        let mut backoff = INITIAL_BACKOFF;

        loop {
            tracing::info!(endpoint = %self.endpoint, "connecting to kuksa.val databroker");

            match self.connect_and_sync().await {
                Ok(()) => {
                    tracing::info!("kuksa.val stream ended normally");
                    backoff = INITIAL_BACKOFF; // reset on clean close
                }
                Err(e) => {
                    tracing::warn!(
                        error = %e,
                        retry_in = ?backoff,
                        "kuksa.val connection failed"
                    );
                }
            }

            sleep(backoff).await;
            backoff = (backoff * 2).min(MAX_BACKOFF);
        }
    }

    async fn connect_and_sync(&self) -> anyhow::Result<()> {
        let mut client = ValClient::connect(self.endpoint.clone()).await?;
        tracing::info!("connected to kuksa.val databroker");

        // Log server info
        if let Ok(resp) = client.get_server_info(proto::GetServerInfoRequest {}).await {
            let info = resp.into_inner();
            tracing::info!(
                name = %info.name,
                version = %info.version,
                "kuksa.val server info"
            );
        }

        // INBOUND: subscribe to all signals from our catalog
        let entries: Vec<SubscribeEntry> = ALL_SIGNALS
            .iter()
            .map(|(path, _)| SubscribeEntry {
                path: path.to_string(),
                view: View::CurrentValue.into(),
                fields: vec![Field::Value.into()],
            })
            .collect();

        let req = SubscribeRequest { entries };
        let mut stream = client.subscribe(req).await?.into_inner();

        // Process incoming updates and forward to SignalBus
        while let Some(msg) = stream.next().await {
            let resp = msg?;
            for update in resp.updates {
                if let Some(entry) = update.entry {
                    self.handle_inbound_entry(&entry).await;
                }
            }
        }

        Ok(())
    }

    /// Convert a kuksa.val DataEntry into a SignalBus publish.
    async fn handle_inbound_entry(&self, entry: &DataEntry) {
        let path = entry.path.as_str();

        // Only forward signals we know about
        let vss_path: VssPath = match ALL_SIGNALS.iter().find(|(p, _)| *p == path) {
            Some((p, _)) => p,
            None => return,
        };

        let value = match entry.value.as_ref().and_then(|dp| dp.value.as_ref()) {
            Some(v) => v,
            None => return,
        };

        let signal_value = datapoint_to_signal_value(value);

        match signal_value {
            Some(sv) => {
                if let Err(e) = self.bus.publish(vss_path, sv).await {
                    tracing::warn!(path, error = %e, "failed to publish inbound signal");
                }
            }
            None => {
                tracing::debug!(path, "unsupported datapoint value type, skipping");
            }
        }
    }
}

/// Convert a kuksa.val Datapoint value to our internal SignalValue.
fn datapoint_to_signal_value(value: &datapoint::Value) -> Option<SignalValue> {
    match value {
        datapoint::Value::Bool(v) => Some(SignalValue::Bool(*v)),
        datapoint::Value::Uint32(v) => {
            if *v <= u16::MAX as u32 {
                Some(SignalValue::Uint16(*v as u16))
            } else {
                None
            }
        }
        datapoint::Value::Int32(v) => {
            if *v >= i16::MIN as i32 && *v <= i16::MAX as i32 {
                Some(SignalValue::Int16(*v as i16))
            } else {
                None
            }
        }
        datapoint::Value::Float(v) => Some(SignalValue::Float(*v)),
        datapoint::Value::Double(v) => Some(SignalValue::Float(*v as f32)),
        datapoint::Value::Uint64(v) => {
            if *v <= u16::MAX as u64 {
                Some(SignalValue::Uint16(*v as u16))
            } else {
                None
            }
        }
        datapoint::Value::Int64(v) => {
            if *v >= i16::MIN as i64 && *v <= i16::MAX as i64 {
                Some(SignalValue::Int16(*v as i16))
            } else {
                None
            }
        }
        // String values: try to map known enums
        datapoint::Value::String(_) => None,
        // Array types are not used on the IPC wire
        _ => None,
    }
}

/// Convert an internal SignalValue to a kuksa.val Datapoint for Set RPCs.
fn signal_value_to_datapoint(value: SignalValue) -> Datapoint {
    let dp_value = match value {
        SignalValue::Bool(v) => datapoint::Value::Bool(v),
        SignalValue::Uint8(v) => datapoint::Value::Uint32(v as u32),
        SignalValue::Int16(v) => datapoint::Value::Int32(v as i32),
        SignalValue::Uint16(v) => datapoint::Value::Uint32(v as u32),
        SignalValue::Float(v) => datapoint::Value::Float(v),
        SignalValue::String(v) => datapoint::Value::String(v),
    };

    Datapoint {
        timestamp: None,
        value: Some(dp_value),
    }
}

/// Push a single signal value to the kuksa.val databroker via Set RPC.
pub async fn push_to_kuksa(
    client: &mut ValClient<Channel>,
    path: &str,
    value: SignalValue,
) -> anyhow::Result<()> {
    let update = EntryUpdate {
        entry: Some(DataEntry {
            path: path.to_string(),
            value: Some(signal_value_to_datapoint(value)),
            actuator_target: None,
            metadata: None,
        }),
        fields: vec![Field::Value.into()],
    };

    let resp = client
        .set(proto::SetRequest {
            updates: vec![update],
        })
        .await?
        .into_inner();

    if let Some(error) = resp.error {
        anyhow::bail!("kuksa.val Set error: {} ({})", error.message, error.reason);
    }
    for entry_err in &resp.errors {
        tracing::warn!(
            path = %entry_err.path,
            error = ?entry_err.error,
            "kuksa.val Set entry-level error"
        );
    }
    Ok(())
}
