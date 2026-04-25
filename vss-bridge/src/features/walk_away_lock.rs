//! Walk-Away Lock — locks the vehicle when all PEPS devices leave the approach zone.
//!
//! Monitors BLE phone zone signals and key fob zone signals. When at least one
//! device has been detected in the approach zone (or closer) and then ALL tracked
//! devices subsequently leave the approach zone (transition to RfRange or OutOfRange),
//! the feature issues a `LockAll` command and publishes a `"lock"` FeedbackRequest.
//!
//! # Zone hierarchy
//! ```text
//! OutOfRange → RfRange → Approach → DriverDoor / PassengerDoor / Hood / Trunk / …
//! ```
//! "In approach" means zone is `Approach`, `DriverDoor`, `PassengerDoor`, `Hood`,
//! `Trunk`, `TrunkInside`, or `Cabin` (i.e., any zone closer than `RfRange`).
//!
//! # Armed state
//! The feature is *armed* per-device when that device enters the approach zone.
//! It fires when every *currently-armed* device is back outside the approach zone.
//! After firing, the armed set is cleared and the feature waits for the next entry.
//!
//! # Scope
//! Tracks 4 keyfobs and 2 BLE phones (the full device set in the simulation).
//! Walk-away lock does NOT apply to NFC cards (held at the handle — users are
//! still at the vehicle when NFC reads).

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::arbiter::{DoorLockArbiter, DoorLockRequest, LockCommand, FEEDBACK_REQUEST};
use crate::ipc_message::{FeatureId, SignalValue};
use crate::signal_bus::SignalBus;

const FOB_ZONE_SIGNALS: [&str; 4] = [
    "Body.PEPS.Plant.KeyFob.1.Zone",
    "Body.PEPS.Plant.KeyFob.2.Zone",
    "Body.PEPS.Plant.KeyFob.3.Zone",
    "Body.PEPS.Plant.KeyFob.4.Zone",
];

const PHONE_ZONE_SIGNALS: [&str; 2] = [
    "Body.PEPS.Plant.BlePhone.1.Zone",
    "Body.PEPS.Plant.BlePhone.2.Zone",
];

const NUM_FOBS: usize = 4;
const NUM_PHONES: usize = 2;
const NUM_DEVICES: usize = NUM_FOBS + NUM_PHONES;

/// True if a zone string value represents "in approach zone or closer".
fn zone_is_in_approach(val: &SignalValue) -> bool {
    matches!(
        val,
        SignalValue::String(s) if matches!(
            s.as_str(),
            "Approach" | "DriverDoor" | "PassengerDoor" | "Hood" | "Trunk" | "TrunkInside" | "Cabin"
        )
    )
}

/// True if a zone string value represents "outside approach zone".
fn zone_is_outside_approach(val: &SignalValue) -> bool {
    matches!(
        val,
        SignalValue::String(s) if matches!(s.as_str(), "OutOfRange" | "RfRange")
    )
}

pub struct WalkAwayLock<B: SignalBus> {
    bus: Arc<B>,
    arbiter: Arc<DoorLockArbiter>,
}

impl<B: SignalBus + Send + Sync + 'static> WalkAwayLock<B> {
    pub fn new(bus: Arc<B>, arbiter: Arc<DoorLockArbiter>) -> Self {
        Self { bus, arbiter }
    }

    pub async fn run(self) {
        // Subscribe to all fob and phone zone signals.
        let fob_streams =
            futures::future::join_all(FOB_ZONE_SIGNALS.iter().map(|&sig| self.bus.subscribe(sig)))
                .await;
        let phone_streams = futures::future::join_all(
            PHONE_ZONE_SIGNALS
                .iter()
                .map(|&sig| self.bus.subscribe(sig)),
        )
        .await;

        let mut fob_zones = futures::stream::select_all(
            fob_streams
                .into_iter()
                .enumerate()
                .map(|(i, s)| futures::stream::StreamExt::map(s, move |v| (i, v))),
        );
        let mut phone_zones = futures::stream::select_all(
            phone_streams
                .into_iter()
                .enumerate()
                .map(|(i, s)| futures::stream::StreamExt::map(s, move |v| (NUM_FOBS + i, v))),
        );

        // Per-device: true = device is currently in approach zone or closer.
        let mut in_approach = [false; NUM_DEVICES];
        // Per-device: true = device has entered approach zone since last lock.
        let mut was_armed = [false; NUM_DEVICES];

        tracing::info!("WalkAwayLock feature started");

        loop {
            let (device_idx, zone_val) = select! {
                Some(pair) = fob_zones.next() => pair,
                Some(pair) = phone_zones.next() => pair,
                else => break,
            };

            let prev_in = in_approach[device_idx];
            let now_in = zone_is_in_approach(&zone_val);
            let now_out = zone_is_outside_approach(&zone_val);

            in_approach[device_idx] = now_in;

            if now_in {
                // Device entered approach — arm it.
                was_armed[device_idx] = true;
            }

            if now_out && prev_in {
                // Device just left the approach zone — check if all armed devices are now out.
                let any_armed = was_armed.iter().any(|&a| a);
                let all_armed_outside = was_armed
                    .iter()
                    .zip(in_approach.iter())
                    .all(|(&armed, &in_ap)| !armed || !in_ap);

                if any_armed && all_armed_outside {
                    tracing::info!(
                        device = device_idx,
                        "WalkAwayLock: all armed devices left approach — locking"
                    );

                    // Fire lock + feedback
                    if let Err(e) = self
                        .arbiter
                        .request(DoorLockRequest {
                            command: LockCommand::LockAll,
                            feature_id: FeatureId::WalkAwayLock,
                        })
                        .await
                    {
                        tracing::error!(error = %e, "WalkAwayLock: arbiter error");
                    }
                    let _ = self
                        .bus
                        .publish(FEEDBACK_REQUEST, SignalValue::String("lock".into()))
                        .await;

                    // Reset armed state — wait for next approach entry.
                    was_armed = [false; NUM_DEVICES];
                }
            }
        }

        tracing::info!("WalkAwayLock feature stopped");
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
        let feature = WalkAwayLock::new(Arc::clone(&bus), arb);
        let handle = tokio::spawn(feature.run());
        tokio::task::yield_now().await;
        (bus, handle)
    }

    #[tokio::test]
    async fn fob_approach_then_leave_triggers_lock() {
        let (bus, _h) = setup().await;

        // Fob 1 enters approach zone
        bus.inject(FOB_ZONE_SIGNALS[0], SignalValue::String("Approach".into()));
        tokio::task::yield_now().await;

        bus.clear_history();

        // Fob 1 leaves approach zone → all armed devices are outside
        bus.inject(
            FOB_ZONE_SIGNALS[0],
            SignalValue::String("OutOfRange".into()),
        );
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(10)).await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("lock_all".into())),
            "expected lock_all command, history: {:?}",
            h
        );
        assert!(
            h.iter()
                .any(|(s, v)| *s == FEEDBACK_REQUEST && *v == SignalValue::String("lock".into())),
            "expected lock FeedbackRequest, history: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn no_lock_when_second_device_still_in_approach() {
        let (bus, _h) = setup().await;

        // Two fobs enter approach
        bus.inject(FOB_ZONE_SIGNALS[0], SignalValue::String("Approach".into()));
        bus.inject(FOB_ZONE_SIGNALS[1], SignalValue::String("Approach".into()));
        tokio::task::yield_now().await;

        bus.clear_history();

        // Only fob 1 leaves — fob 2 still in approach
        bus.inject(
            FOB_ZONE_SIGNALS[0],
            SignalValue::String("OutOfRange".into()),
        );
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(10)).await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, _)| *s == "Body.Doors.CentralLock.Command"),
            "should NOT lock while fob 2 still in approach, history: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn never_armed_device_does_not_prevent_lock() {
        let (bus, _h) = setup().await;

        // Only fob 1 ever enters approach (fobs 2-4 and phones stay out)
        bus.inject(FOB_ZONE_SIGNALS[0], SignalValue::String("Approach".into()));
        tokio::task::yield_now().await;
        bus.clear_history();

        // Fob 1 leaves — only it was armed, and it's now out
        bus.inject(FOB_ZONE_SIGNALS[0], SignalValue::String("RfRange".into()));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(10)).await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            h.iter().any(|(s, v)| *s == "Body.Doors.CentralLock.Command"
                && *v == SignalValue::String("lock_all".into())),
            "only armed device leaving should trigger lock, history: {:?}",
            h
        );
    }

    #[tokio::test]
    async fn device_never_entered_approach_no_lock_on_leave() {
        let (bus, _h) = setup().await;

        // Fob 1 goes directly from (implicit initial) OutOfRange to RfRange
        // without ever having been in approach — no armed device, no lock.
        bus.inject(
            FOB_ZONE_SIGNALS[0],
            SignalValue::String("OutOfRange".into()),
        );
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(10)).await;
        tokio::task::yield_now().await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, _)| *s == "Body.Doors.CentralLock.Command"),
            "device never in approach should NOT trigger lock, history: {:?}",
            h
        );
    }
}
