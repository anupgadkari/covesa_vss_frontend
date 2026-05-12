//! Window-lockout feature — latches the driver master push into the
//! body-controller's lockout state.
//!
//! Input:
//! - `Body.Switches.WindowLockout.IsPressed` — momentary bool.
//!   The HMI driver-master panel writes `true` on press-down and
//!   `false` on release.
//!
//! Output:
//! - `Body.Switches.Window.LockoutEnabled` — latched bool.  Toggled
//!   on each rising edge of the input switch.  When `true`, the body
//!   controller (and the HMI by visual convention) treats writes to
//!   passenger / rear `Body.Doors.Row*.Window.Position` as ignored.
//!
//! # Boot
//!
//! Publishes `false` once at start-up so HMI snapshots land a
//! defined initial state.
//!
//! # Single writer
//!
//! Only writer of `Body.Switches.Window.LockoutEnabled`.  No
//! arbiter is required because the lockout state has no other
//! contention surface — it's pure driver intent.

use std::sync::Arc;

use futures::StreamExt;

use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

const PRESS: VssPath = "Body.Switches.WindowLockout.IsPressed";
const OUT: VssPath = "Body.Switches.Window.LockoutEnabled";

pub struct WindowLockout<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> WindowLockout<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    pub async fn run(self) {
        tracing::info!("WindowLockout feature started");

        let mut press_rx = self.bus.subscribe(PRESS).await;

        // Deterministic boot state.
        let _ = self
            .bus
            .publish(OUT, SignalValue::Bool(false))
            .await;

        let mut latched: bool = false;
        let mut last_press: bool = false;

        while let Some(val) = press_rx.next().await {
            let now = matches!(val, SignalValue::Bool(true));
            // Rising edge — toggle the latched output.
            if now && !last_press {
                latched = !latched;
                tracing::info!(enabled = latched, "WindowLockout: toggled");
                if let Err(e) = self
                    .bus
                    .publish(OUT, SignalValue::Bool(latched))
                    .await
                {
                    tracing::error!(error = %e, "WindowLockout: publish failed");
                }
            }
            last_press = now;
        }

        tracing::warn!("WindowLockout: press stream closed, exiting");
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;

    async fn settle() {
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    async fn setup() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let f = WindowLockout::new(Arc::clone(&bus));
        tokio::spawn(f.run());
        settle().await;
        bus
    }

    fn out(bus: &MockBus) -> Option<bool> {
        match bus.latest_value(OUT) {
            Some(SignalValue::Bool(b)) => Some(b),
            _ => None,
        }
    }

    #[tokio::test]
    async fn boots_off() {
        let bus = setup().await;
        assert_eq!(out(&bus), Some(false));
    }

    #[tokio::test]
    async fn toggles_on_press_edge() {
        let bus = setup().await;
        bus.inject(PRESS, SignalValue::Bool(true));
        settle().await;
        assert_eq!(out(&bus), Some(true));
        bus.inject(PRESS, SignalValue::Bool(false));
        settle().await;
        assert_eq!(out(&bus), Some(true), "release must not toggle");
        bus.inject(PRESS, SignalValue::Bool(true));
        settle().await;
        assert_eq!(out(&bus), Some(false));
    }

    #[tokio::test]
    async fn held_press_does_not_re_toggle() {
        let bus = setup().await;
        bus.inject(PRESS, SignalValue::Bool(true));
        bus.inject(PRESS, SignalValue::Bool(true));
        bus.inject(PRESS, SignalValue::Bool(true));
        settle().await;
        assert_eq!(out(&bus), Some(true), "must only toggle once on rising edge");
    }
}
