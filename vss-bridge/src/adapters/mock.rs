//! MockBus — in-memory SignalBus for unit tests and CI.
//!
//! Uses `tokio::sync::broadcast` to fan out signal updates.
//! Records all published (signal, value) pairs for test assertions.

use std::collections::HashMap;
use std::sync::Mutex;

use async_trait::async_trait;
use futures::stream::BoxStream;
use futures::StreamExt;
use tokio::sync::broadcast;

use crate::ipc_message::SignalValue;
use crate::signal_bus::{AckResult, SignalBus, VssPath};

/// Channel capacity for broadcast subscribers.
const CHANNEL_CAPACITY: usize = 256;

pub struct MockBus {
    /// Per-signal broadcast channels for subscribe().
    channels: Mutex<HashMap<VssPath, broadcast::Sender<SignalValue>>>,
    /// Ordered history of all published signals for test assertions.
    history: Mutex<Vec<(VssPath, SignalValue)>>,
}

impl MockBus {
    pub fn new() -> Self {
        Self {
            channels: Mutex::new(HashMap::new()),
            history: Mutex::new(Vec::new()),
        }
    }

    /// Returns the full publish history for assertions.
    pub fn history(&self) -> Vec<(VssPath, SignalValue)> {
        self.history.lock().unwrap().clone()
    }

    /// Clears the publish history.
    pub fn clear_history(&self) {
        self.history.lock().unwrap().clear();
    }

    /// Inject a signal value as if it came from the Safety Monitor.
    /// Sends to all subscribers of the given signal.
    pub fn inject(&self, signal: VssPath, value: SignalValue) {
        let channels = self.channels.lock().unwrap();
        if let Some(tx) = channels.get(signal) {
            // Ignore send errors (no active subscribers).
            let _ = tx.send(value);
        }
    }

    /// Ensure a broadcast channel exists for a signal, returning the sender.
    fn get_or_create_channel(&self, signal: VssPath) -> broadcast::Sender<SignalValue> {
        let mut channels = self.channels.lock().unwrap();
        channels
            .entry(signal)
            .or_insert_with(|| broadcast::channel(CHANNEL_CAPACITY).0)
            .clone()
    }
}

impl Default for MockBus {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl SignalBus for MockBus {
    async fn publish(&self, signal: VssPath, value: SignalValue) -> anyhow::Result<()> {
        self.history.lock().unwrap().push((signal, value.clone()));
        // Also broadcast so subscribers see outbound values (useful in tests).
        let tx = self.get_or_create_channel(signal);
        let _ = tx.send(value);
        Ok(())
    }

    async fn subscribe(&self, signal: VssPath) -> BoxStream<'static, SignalValue> {
        let tx = self.get_or_create_channel(signal);
        let rx = tx.subscribe();
        let stream = tokio_stream::wrappers::BroadcastStream::new(rx)
            .filter_map(|r: Result<SignalValue, _>| async move { r.ok() });
        Box::pin(stream)
    }

    async fn publish_await_ack(
        &self,
        signal: VssPath,
        value: SignalValue,
        _timeout_ms: u64,
    ) -> anyhow::Result<AckResult> {
        // MockBus always acknowledges immediately — no hardware.
        self.publish(signal, value).await?;
        Ok(AckResult::Ok)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    #[tokio::test]
    async fn publish_records_history() {
        let bus = MockBus::new();
        bus.publish("Body.Lights.Beam.Low.IsOn", SignalValue::Bool(true))
            .await
            .unwrap();
        bus.publish("Body.Horn.IsActive", SignalValue::Bool(false))
            .await
            .unwrap();

        let hist = bus.history();
        assert_eq!(hist.len(), 2);
        assert_eq!(hist[0], ("Body.Lights.Beam.Low.IsOn", SignalValue::Bool(true)));
        assert_eq!(hist[1], ("Body.Horn.IsActive", SignalValue::Bool(false)));
    }

    #[tokio::test]
    async fn subscribe_receives_injected_values() {
        let bus = Arc::new(MockBus::new());
        let mut stream = bus.subscribe("Body.Lights.Beam.Low.IsOn").await;

        let bus2 = Arc::clone(&bus);
        tokio::spawn(async move {
            tokio::time::sleep(std::time::Duration::from_millis(10)).await;
            bus2.inject("Body.Lights.Beam.Low.IsOn", SignalValue::Bool(true));
        });

        let val = tokio::time::timeout(
            std::time::Duration::from_millis(100),
            stream.next(),
        )
        .await
        .expect("timed out")
        .expect("stream ended");

        assert_eq!(val, SignalValue::Bool(true));
    }

    #[tokio::test]
    async fn publish_await_ack_returns_ok() {
        let bus = MockBus::new();
        let result = bus
            .publish_await_ack("Body.Doors.Row1.Left.IsLocked", SignalValue::Bool(true), 100)
            .await
            .unwrap();
        assert_eq!(result, AckResult::Ok);

        // Should also appear in history.
        assert_eq!(bus.history().len(), 1);
    }

    #[tokio::test]
    async fn subscribe_does_not_cross_signals() {
        let bus = Arc::new(MockBus::new());
        let mut stream_a = bus.subscribe("Body.Lights.Beam.Low.IsOn").await;

        // Inject to a different signal
        bus.inject("Body.Horn.IsActive", SignalValue::Bool(true));

        // stream_a should not receive anything
        let result = tokio::time::timeout(
            std::time::Duration::from_millis(50),
            stream_a.next(),
        )
        .await;
        assert!(result.is_err(), "should have timed out — wrong signal");
    }

    #[tokio::test]
    async fn clear_history_works() {
        let bus = MockBus::new();
        bus.publish("Body.Horn.IsActive", SignalValue::Bool(true))
            .await
            .unwrap();
        assert_eq!(bus.history().len(), 1);
        bus.clear_history();
        assert_eq!(bus.history().len(), 0);
    }
}
