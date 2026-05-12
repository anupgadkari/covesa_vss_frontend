//! Power child-lock feature — single master push on the driver
//! master panel that arms / disarms child-lock on both rear doors.
//!
//! ```text
//!  HMI driver-master button
//!      │  Body.Switches.PowerChildLock.IsPressed  (momentary)
//!      ▼
//!  PowerChildLock feature              ← this module
//!      │
//!      ├─ Body.PowerChildLock.MasterStatus       (latched bool)
//!      ├─ Body.Doors.Row2.Left.IsChildLockActive (per-door fan-out)
//!      └─ Body.Doors.Row2.Right.IsChildLockActive
//! ```
//!
//! On each rising edge of the master push the feature toggles
//! `MasterStatus` and writes the same value to both per-door
//! outputs.  Down-stream consumers observe the per-door signals so
//! they don't have to know about the master concept:
//!
//! * `DoorHandlePlantModel` ignores `Handle.Inside.IsPulled` on a
//!   door whose `IsChildLockActive` is true.
//! * `PowerWindowLocal` suppresses the matching door's local window
//!   switches when the door is child-locked.
//!
//! Per-door granularity is deliberate.  When we eventually wire up
//! door-side mechanical feedback signals, only one of the two doors
//! may report success — at that point the per-door output stays
//! tied to feedback rather than the master.
//!
//! # Boot
//!
//! Publishes `false` on all three outputs at start-up so HMI
//! snapshots land a defined released-state.
//!
//! # Single writer
//!
//! Only writer of the three outputs.  No arbitration needed — there
//! is no other source claiming these signals.

use std::sync::Arc;

use futures::StreamExt;

use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

const PRESS: VssPath = "Body.Switches.PowerChildLock.IsPressed";
const MASTER: VssPath = "Body.PowerChildLock.MasterStatus";
const PER_DOOR: [VssPath; 2] = [
    "Body.Doors.Row2.Left.IsChildLockActive",
    "Body.Doors.Row2.Right.IsChildLockActive",
];

pub struct PowerChildLock<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> PowerChildLock<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    pub async fn run(self) {
        tracing::info!("PowerChildLock feature started");

        let mut press_rx = self.bus.subscribe(PRESS).await;

        // Deterministic boot — all outputs released.
        let _ = self.bus.publish(MASTER, SignalValue::Bool(false)).await;
        for sig in PER_DOOR.iter() {
            let _ = self.bus.publish(sig, SignalValue::Bool(false)).await;
        }

        let mut latched: bool = false;
        let mut last_press: bool = false;

        while let Some(val) = press_rx.next().await {
            let now = matches!(val, SignalValue::Bool(true));
            if now && !last_press {
                // Rising edge — toggle master + fan-out both doors.
                latched = !latched;
                tracing::info!(active = latched, "PowerChildLock: toggled");
                let _ = self
                    .bus
                    .publish(MASTER, SignalValue::Bool(latched))
                    .await;
                for sig in PER_DOOR.iter() {
                    let _ = self.bus.publish(sig, SignalValue::Bool(latched)).await;
                }
            }
            last_press = now;
        }

        tracing::warn!("PowerChildLock: press stream closed, exiting");
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
        let f = PowerChildLock::new(Arc::clone(&bus));
        tokio::spawn(f.run());
        settle().await;
        bus
    }

    fn bv(bus: &MockBus, sig: VssPath) -> Option<bool> {
        match bus.latest_value(sig) {
            Some(SignalValue::Bool(b)) => Some(b),
            _ => None,
        }
    }

    #[tokio::test]
    async fn boots_released_all_three_outputs() {
        let bus = setup().await;
        assert_eq!(bv(&bus, MASTER), Some(false));
        assert_eq!(bv(&bus, PER_DOOR[0]), Some(false));
        assert_eq!(bv(&bus, PER_DOOR[1]), Some(false));
    }

    #[tokio::test]
    async fn one_press_engages_master_and_both_doors() {
        let bus = setup().await;
        bus.inject(PRESS, SignalValue::Bool(true));
        settle().await;
        assert_eq!(bv(&bus, MASTER), Some(true));
        assert_eq!(bv(&bus, PER_DOOR[0]), Some(true));
        assert_eq!(bv(&bus, PER_DOOR[1]), Some(true));
    }

    #[tokio::test]
    async fn release_alone_does_not_toggle() {
        let bus = setup().await;
        bus.inject(PRESS, SignalValue::Bool(true));
        settle().await;
        bus.inject(PRESS, SignalValue::Bool(false));
        settle().await;
        // Single press+release toggles ONCE (on rising edge).
        assert_eq!(bv(&bus, MASTER), Some(true));
    }

    #[tokio::test]
    async fn two_presses_toggle_back_to_released() {
        let bus = setup().await;
        bus.inject(PRESS, SignalValue::Bool(true));
        bus.inject(PRESS, SignalValue::Bool(false));
        bus.inject(PRESS, SignalValue::Bool(true));
        settle().await;
        assert_eq!(bv(&bus, MASTER), Some(false));
        assert_eq!(bv(&bus, PER_DOOR[0]), Some(false));
        assert_eq!(bv(&bus, PER_DOOR[1]), Some(false));
    }

    #[tokio::test]
    async fn held_press_does_not_re_toggle() {
        let bus = setup().await;
        bus.inject(PRESS, SignalValue::Bool(true));
        bus.inject(PRESS, SignalValue::Bool(true));
        bus.inject(PRESS, SignalValue::Bool(true));
        settle().await;
        // Three consecutive `true`s still only one rising edge.
        assert_eq!(bv(&bus, MASTER), Some(true));
    }

    #[tokio::test]
    async fn fan_out_is_symmetric_per_door() {
        let bus = setup().await;
        bus.inject(PRESS, SignalValue::Bool(true));
        settle().await;
        // Both doors must always match the master — no per-door
        // command in this feature.
        let l = bv(&bus, PER_DOOR[0]);
        let r = bv(&bus, PER_DOOR[1]);
        assert_eq!(l, r);
    }
}
