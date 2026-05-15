//! NFC Entry — unlock the cabin on an NFC card tap at the
//! driver-handle reader, or an NFC-equipped phone tap at the
//! same B-pillar.
//!
//! # Triggers
//!
//! On the bus, an NFC tap surfaces as a transient signal change:
//!
//! - `Body.PEPS.Plant.NfcCard.{1,2}.Position` rising edge to
//!   `"DriverHandle"` — the user tapped a paired NFC card on the
//!   driver-door B-pillar reader.
//! - `Body.PEPS.Plant.BlePhone.{1,2}.NfcTap` rising edge to
//!   `"DriverHandle"` — the user tapped an NFC-equipped phone on
//!   the driver-door B-pillar.  Phones expose BLE (continuous
//!   proximity, consumed by PassiveEntry on a handle pull) and
//!   NFC (deliberate ~5 cm tap, consumed here) as independent
//!   radios; the simulator models them as independent signals
//!   (`.Zone` for BLE, `.NfcTap` for NFC) so the user can have a
//!   phone in their pocket near the door without it auto-unlocking.
//!
//! Falling edges (`NotPresent`) clear the latch but do not trigger
//! a fresh unlock.
//!
//! # Action
//!
//! On a qualifying edge, dispatch `LockCommand::UnlockAll` through
//! the door-lock arbiter at `FeatureId::NfcCard` (for cards) or
//! `FeatureId::NfcPhone` (for phones) and publish `FEEDBACK_REQUEST
//! = "unlock"` so the LockFeedback feature flashes the indicators.
//!
//! NfcCard / NfcPhone are already in
//! `auto_relock::EXTERNAL_UNLOCK_REQUESTORS` and
//! `perimeter_alarm::EXTERNAL_AUTH_SOURCES`, so a successful tap
//! will arm AutoRelock and disarm an active alarm just like RKE /
//! PassiveEntry.
//!
//! # Short-circuit on already-unlocked
//!
//! If `Cabin.LockStatus` already reads `UNLOCKED`, a tap is a no-op
//! (the user pulled the handle twice, or the car was already open).
//! `DRIVER_UNLOCKED` still escalates — a deliberate tap on the
//! B-pillar reader is a clear "open everything" intent.
//!
//! # Not yet implemented — `PushButton` start
//!
//! `NfcPosition::PushButton` (card) / `Zone::KeyCylinder` (phone)
//! represent an NFC tap on the cylinder / start-button reader.
//! VehicleStartingControl would have to accept this as an auth
//! source alongside its current `Cabin` Authenticated scan; that
//! crosscut is deferred to a follow-up PR.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::arbiter::{DoorLockArbiter, DoorLockRequest, LockCommand, FEEDBACK_REQUEST};
use crate::ipc_message::{FeatureId, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

const LOCK_STATUS: VssPath = "Cabin.LockStatus";

/// Per-NFC-card slot paths.  Two cards in the simulator HMI.
const NFC_CARD_SIGNALS: [VssPath; 2] = [
    "Body.PEPS.Plant.NfcCard.1.Position",
    "Body.PEPS.Plant.NfcCard.2.Position",
];

/// Per-BLE-phone NFC tap paths.  Two phones in the simulator HMI.
/// Phones authenticate over BLE for proximity (handled by PassiveEntry
/// on a handle-pull via `.Zone`) and over NFC for tap-to-unlock
/// (handled here via `.NfcTap`).  Distinct signals so BLE proximity
/// and NFC tap don't conflate.
const BLE_PHONE_SIGNALS: [VssPath; 2] = [
    "Body.PEPS.Plant.BlePhone.1.NfcTap",
    "Body.PEPS.Plant.BlePhone.2.NfcTap",
];

pub struct NfcEntry<B: SignalBus> {
    bus: Arc<B>,
    arbiter: Arc<DoorLockArbiter>,
}

impl<B: SignalBus + Send + Sync + 'static> NfcEntry<B> {
    pub fn new(bus: Arc<B>, arbiter: Arc<DoorLockArbiter>) -> Self {
        Self { bus, arbiter }
    }

    pub async fn run(self) {
        tracing::info!("NfcEntry feature started");

        // Subscribe to all NFC card positions + all phone zones +
        // lock status.  All five streams race in a single select.
        let mut card_rxs = Vec::with_capacity(NFC_CARD_SIGNALS.len());
        for sig in NFC_CARD_SIGNALS {
            card_rxs.push(self.bus.subscribe(sig).await);
        }
        let mut phone_rxs = Vec::with_capacity(BLE_PHONE_SIGNALS.len());
        for sig in BLE_PHONE_SIGNALS {
            phone_rxs.push(self.bus.subscribe(sig).await);
        }
        let mut lock_rx = self.bus.subscribe(LOCK_STATUS).await;

        // Per-device "is the tap currently engaged" latch — used to
        // distinguish a rising edge (`NotPresent` → `DriverHandle`)
        // from a redundant publish (`DriverHandle` → `DriverHandle`).
        let mut card_at_handle = [false; NFC_CARD_SIGNALS.len()];
        let mut phone_at_handle_zone = [false; BLE_PHONE_SIGNALS.len()];

        // Default LOCKED so the boot-time stance is "auth required";
        // a stray pre-boot tap that arrives before any LockStatus
        // publish will still attempt the unlock (cheap arbiter call).
        let mut lock_status = "LOCKED".to_string();

        loop {
            select! {
                Some(val) = lock_rx.next() => {
                    if let SignalValue::String(s) = val {
                        lock_status = s;
                    }
                }
                Some((idx, val)) = next_indexed(&mut card_rxs) => {
                    if let SignalValue::String(s) = val {
                        let engaged = s == "DriverHandle";
                        let was_engaged = card_at_handle[idx];
                        card_at_handle[idx] = engaged;
                        // Rising edge to DriverHandle → fire.
                        if engaged && !was_engaged {
                            self.on_tap(
                                "NfcCard",
                                idx as u8 + 1,
                                FeatureId::NfcCard,
                                &lock_status,
                            )
                            .await;
                        }
                    }
                }
                Some((idx, val)) = next_indexed(&mut phone_rxs) => {
                    if let SignalValue::String(s) = val {
                        // `BlePhone.{N}.NfcTap` uses the same enum as
                        // an NFC card: NotPresent / DriverHandle /
                        // PushButton.  `DriverHandle` is the only
                        // unlock-triggering value; `PushButton` parks
                        // there for the deferred start-button path.
                        let engaged = s == "DriverHandle";
                        let was_engaged = phone_at_handle_zone[idx];
                        phone_at_handle_zone[idx] = engaged;
                        if engaged && !was_engaged {
                            self.on_tap(
                                "NfcPhone",
                                idx as u8 + 1,
                                FeatureId::NfcPhone,
                                &lock_status,
                            )
                            .await;
                        }
                    }
                }
                else => break,
            }
        }

        tracing::warn!("NfcEntry: an input stream closed, exiting");
    }

    /// Common dispatch path for a card or phone tap.  Short-circuits
    /// when the cabin is already fully unlocked.
    async fn on_tap(
        &self,
        device_kind: &'static str,
        slot: u8,
        feature_id: FeatureId,
        lock_status: &str,
    ) {
        if lock_status == "UNLOCKED" {
            tracing::debug!(
                device_kind,
                slot,
                lock_status,
                "NfcEntry: tap ignored — cabin already unlocked"
            );
            return;
        }

        tracing::info!(
            device_kind,
            slot,
            lock_status,
            "NfcEntry: tap accepted — dispatching UnlockAll"
        );

        if let Err(e) = self
            .arbiter
            .request(DoorLockRequest {
                command: LockCommand::UnlockAll,
                feature_id,
            })
            .await
        {
            tracing::error!(error = %e, "NfcEntry: arbiter rejected unlock");
            return;
        }

        let _ = self
            .bus
            .publish(FEEDBACK_REQUEST, SignalValue::String("unlock".into()))
            .await;
    }
}

/// Helper: race a slice of streams, returning `(index, value)` for
/// whichever produced next.  Used because `tokio::select!` can't
/// itself enumerate a `Vec<Stream>`.
async fn next_indexed<S>(streams: &mut [S]) -> Option<(usize, SignalValue)>
where
    S: futures::Stream<Item = SignalValue> + Unpin + Send,
{
    use futures::stream::StreamExt as _;
    let futs: Vec<_> = streams
        .iter_mut()
        .enumerate()
        .map(|(i, s)| {
            Box::pin(async move { s.next().await.map(|v| (i, v)) })
                as std::pin::Pin<Box<dyn std::future::Future<Output = _> + Send>>
        })
        .collect();
    let (first, _idx, _rest) = futures::future::select_all(futs).await;
    first
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::door_lock_arbiter;
    use crate::plant_models::door_lock::DoorLockPlantModel;

    async fn settle() {
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
    }

    /// Spin up bus + door-lock arbiter + plant + NfcEntry.  The plant
    /// is spawned so the arbiter receives acks and can dispatch a
    /// second request after the first completes — otherwise tests
    /// that fire two taps would see the second one queued forever.
    async fn setup() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let (arb, ack_tx, arb_fut) = door_lock_arbiter(Arc::clone(&bus));
        tokio::spawn(arb_fut);
        let arb = Arc::new(arb);

        let dlpm = DoorLockPlantModel::with_ack_tx(Arc::clone(&bus), ack_tx);
        tokio::spawn(dlpm.run());

        let feat = NfcEntry::new(Arc::clone(&bus), arb);
        tokio::spawn(feat.run());
        settle().await;
        bus
    }

    fn unlock_was_dispatched(bus: &MockBus) -> bool {
        bus.history().into_iter().any(|(s, v)| {
            s == "Body.Doors.CentralLock.Command" && {
                matches!(v, SignalValue::String(ref c)
                    if c == "unlock_all" || c == "unlock_driver")
            }
        })
    }

    fn feedback_was_published(bus: &MockBus) -> bool {
        bus.history().into_iter().any(|(s, v)| {
            s == "Body.Doors.CentralLock.FeedbackRequest"
                && matches!(v, SignalValue::String(r) if r == "unlock")
        })
    }

    // ── NFC card path ──────────────────────────────────────────────────

    #[tokio::test]
    async fn card_tap_at_driver_handle_unlocks() {
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        settle().await;
        bus.clear_history();

        bus.inject(
            "Body.PEPS.Plant.NfcCard.1.Position",
            SignalValue::String("DriverHandle".into()),
        );
        settle().await;

        assert!(unlock_was_dispatched(&bus), "tap must dispatch UnlockAll");
        assert!(
            feedback_was_published(&bus),
            "tap must publish unlock feedback"
        );
    }

    #[tokio::test]
    async fn card_tap_at_push_button_does_not_unlock() {
        // PushButton path is the start-button reader.  Unlock should
        // NOT fire — the start path is deferred to a follow-up.
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        settle().await;
        bus.clear_history();

        bus.inject(
            "Body.PEPS.Plant.NfcCard.1.Position",
            SignalValue::String("PushButton".into()),
        );
        settle().await;

        assert!(!unlock_was_dispatched(&bus));
    }

    #[tokio::test]
    async fn card_redundant_position_does_not_re_unlock() {
        // Tap latch: a card sitting at DriverHandle must fire ONCE.
        // A redundant publish of the same value is not a fresh tap.
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        bus.inject(
            "Body.PEPS.Plant.NfcCard.1.Position",
            SignalValue::String("DriverHandle".into()),
        );
        settle().await;
        bus.clear_history();

        // Same position republished — must NOT trigger another unlock.
        bus.inject(
            "Body.PEPS.Plant.NfcCard.1.Position",
            SignalValue::String("DriverHandle".into()),
        );
        settle().await;

        assert!(!unlock_was_dispatched(&bus));
    }

    #[tokio::test]
    async fn card_remove_then_re_tap_fires_again() {
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        bus.inject(
            "Body.PEPS.Plant.NfcCard.1.Position",
            SignalValue::String("DriverHandle".into()),
        );
        settle().await;
        // Remove the card and put it back.
        bus.inject(
            "Body.PEPS.Plant.NfcCard.1.Position",
            SignalValue::String("NotPresent".into()),
        );
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        settle().await;
        bus.clear_history();
        bus.inject(
            "Body.PEPS.Plant.NfcCard.1.Position",
            SignalValue::String("DriverHandle".into()),
        );
        settle().await;

        assert!(unlock_was_dispatched(&bus));
    }

    #[tokio::test]
    async fn card_tap_when_already_unlocked_is_noop() {
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("UNLOCKED".into()));
        settle().await;
        bus.clear_history();

        bus.inject(
            "Body.PEPS.Plant.NfcCard.1.Position",
            SignalValue::String("DriverHandle".into()),
        );
        settle().await;

        assert!(!unlock_was_dispatched(&bus));
    }

    #[tokio::test]
    async fn card_tap_when_driver_unlocked_still_escalates() {
        // DRIVER_UNLOCKED → tap escalates to UnlockAll.
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("DRIVER_UNLOCKED".into()));
        settle().await;
        bus.clear_history();

        bus.inject(
            "Body.PEPS.Plant.NfcCard.1.Position",
            SignalValue::String("DriverHandle".into()),
        );
        settle().await;

        assert!(unlock_was_dispatched(&bus));
    }

    #[tokio::test]
    async fn second_card_taps_independently() {
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        settle().await;
        bus.clear_history();

        bus.inject(
            "Body.PEPS.Plant.NfcCard.2.Position",
            SignalValue::String("DriverHandle".into()),
        );
        settle().await;

        assert!(unlock_was_dispatched(&bus));
    }

    // ── BLE phone NFC path ─────────────────────────────────────────────

    #[tokio::test]
    async fn phone_nfc_tap_at_driver_handle_unlocks() {
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        settle().await;
        bus.clear_history();

        bus.inject(
            "Body.PEPS.Plant.BlePhone.1.NfcTap",
            SignalValue::String("DriverHandle".into()),
        );
        settle().await;

        assert!(unlock_was_dispatched(&bus));
    }

    #[tokio::test]
    async fn phone_nfc_tap_at_push_button_does_not_unlock() {
        // PushButton is the start-button NFC pad — deferred path.
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        settle().await;
        bus.clear_history();

        bus.inject(
            "Body.PEPS.Plant.BlePhone.1.NfcTap",
            SignalValue::String("PushButton".into()),
        );
        settle().await;

        assert!(!unlock_was_dispatched(&bus));
    }

    #[tokio::test]
    async fn phone_ble_zone_changes_do_not_trigger_nfc_unlock() {
        // BLE proximity is observed via `.Zone`; NfcEntry must
        // ignore those events entirely.  A phone arriving at
        // LeftFront via BLE should not auto-unlock — that's
        // PassiveEntry's job, gated on a handle pull.
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        settle().await;
        bus.clear_history();

        bus.inject(
            "Body.PEPS.Plant.BlePhone.1.Zone",
            SignalValue::String("LeftFront".into()),
        );
        settle().await;
        assert!(!unlock_was_dispatched(&bus));

        bus.inject(
            "Body.PEPS.Plant.BlePhone.1.Zone",
            SignalValue::String("Cabin".into()),
        );
        settle().await;
        assert!(!unlock_was_dispatched(&bus));
    }

    #[tokio::test]
    async fn phone_tap_when_already_unlocked_is_noop() {
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("UNLOCKED".into()));
        settle().await;
        bus.clear_history();

        bus.inject(
            "Body.PEPS.Plant.BlePhone.1.NfcTap",
            SignalValue::String("DriverHandle".into()),
        );
        settle().await;

        assert!(!unlock_was_dispatched(&bus));
    }

    #[tokio::test]
    async fn phone_redundant_nfc_tap_does_not_re_unlock() {
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        bus.inject(
            "Body.PEPS.Plant.BlePhone.1.NfcTap",
            SignalValue::String("DriverHandle".into()),
        );
        settle().await;
        bus.clear_history();

        bus.inject(
            "Body.PEPS.Plant.BlePhone.1.NfcTap",
            SignalValue::String("DriverHandle".into()),
        );
        settle().await;

        assert!(!unlock_was_dispatched(&bus));
    }
}
