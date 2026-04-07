//! SignalBus trait — the portability seam between feature logic and transport.
//!
//! Every feature FSM and the Signal Arbiter depend only on this trait.
//! No feature imports any transport type (RPmsg, GLINK, SOME/IP, Mock).

use async_trait::async_trait;
use futures::stream::BoxStream;

use crate::ipc_message::SignalValue;

/// A VSS signal path, e.g. `"Body.Lights.Beam.Low.IsOn"`.
pub type VssPath = &'static str;

/// Result of a publish-and-await-ack operation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AckResult {
    /// Safety Monitor accepted the command.
    Ok,
    /// Safety Monitor vetoed the command with a reason.
    Vetoed(String),
    /// No acknowledgement within the timeout window.
    Timeout,
}

/// The core transport abstraction. Implemented by RpmsgBus, MockBus, etc.
///
/// - `publish` is fire-and-forget (ambient light colour, informational writes).
/// - `publish_await_ack` blocks until CMD_ACK or timeout (safety-relevant features).
/// - `subscribe` returns a stream of state updates from the Safety Monitor.
#[async_trait]
pub trait SignalBus: Send + Sync + 'static {
    /// Publish an arbitrated actuator value downstream (toward Safety Monitor).
    async fn publish(&self, signal: VssPath, value: SignalValue) -> anyhow::Result<()>;

    /// Subscribe to incoming state updates (Safety Monitor → A53).
    async fn subscribe(&self, signal: VssPath) -> BoxStream<'static, SignalValue>;

    /// Publish and await CMD_ACK from the Safety Monitor.
    /// Times out after `timeout_ms` milliseconds.
    async fn publish_await_ack(
        &self,
        signal: VssPath,
        value: SignalValue,
        timeout_ms: u64,
    ) -> anyhow::Result<AckResult>;
}
