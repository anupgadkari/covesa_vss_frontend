//! Driver — stub plant model for the VSS v6.0 `Vehicle.Driver.Identifier`
//! branch.
//!
//! ## Why this exists
//!
//! VSS v6.0 introduces `Vehicle.Driver.Identifier.{Type,Subject}` to
//! carry "who is driving / who actuated the vehicle".  On most
//! platforms a separate driver-monitoring or telematics service owns
//! these paths.  On *this* body-controller platform we have no such
//! service: the bridge sits on the only seam between the VSS broker
//! and the CAN / LIN buses, so the bridge has to fill in the canonical
//! values from what the body domain already knows.
//!
//! See `docs/post-peps-backlog.md` item #25 ("Bridge as VSS publisher
//! for non-body domains") for the architectural framing.
//!
//! ## What it publishes
//!
//! On every edge of `Vehicle.Cabin.LockStatus.LastRequestor` we map
//! the requestor name to:
//!
//!   * `Vehicle.Driver.Identifier.Type` — VSS standard enum value
//!     (`"userid"` for credential-based actuation, `"default"` for
//!     autonomous / interior-physical / unknown).  We deliberately
//!     stay on the VSS-conformant value set rather than inventing
//!     extension values for the standard path.
//!   * `Vehicle.Driver.Identifier.Subject` — opaque identifier
//!     string.  For now this is just the requestor name (e.g.
//!     `"KeyfobRke"`, `"PassiveEntry"`).  When RKE / PassiveEntry
//!     later thread slot info through, this becomes e.g.
//!     `"Keyfob:1"` so consumers can tell *which* fob authenticated.
//!     Empty string when the actuation was autonomous (`AutoLock`,
//!     `CrashUnlock`, …) or interior-physical (`DoorTrimButton`,
//!     `SlamLock`).
//!
//! ## Why the bridge owns this and not a feature
//!
//! Features publish *intent* on the bus.  The Driver branch is plant
//! state — "the canonical fact of who is currently considered the
//! driver" — derived from authentication events.  The plant-model
//! layer is the right home for derived canonical signals that have
//! no separate authoritative producer.
//!
//! Future expansion: when driver-monitoring (gaze, fatigue, …) lands,
//! that data flows into `Vehicle.Driver.*` from a different producer.
//! This stub stays scoped to the Identifier sub-branch.

use std::sync::Arc;

use futures::StreamExt;

use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

const LAST_REQUESTOR: VssPath = "Vehicle.Cabin.LockStatus.LastRequestor";

const DRIVER_IDENTIFIER_TYPE: VssPath = "Vehicle.Driver.Identifier.Type";
const DRIVER_IDENTIFIER_SUBJECT: VssPath = "Vehicle.Driver.Identifier.Subject";

/// Maps a `LastRequestor` string to the VSS-standard
/// `Vehicle.Driver.Identifier.Type` enum value.
///
/// VSS v6.0 standard values for this enum: `default`, `email`,
/// `username`, `userid`, `phoneNumber`, `iccid`, `imsi`, `eui48`,
/// `eui64`.  We only ever emit `"userid"` or `"default"` — finer
/// distinctions (BLE MAC → `eui48`, NFC card → `eui48`, …) would
/// require slot-level wiring that doesn't exist yet.
///
/// Returns `(type, subject)` where `subject` is empty for autonomous
/// or interior-physical actuations that don't carry a credential.
fn classify(requestor: &str) -> (&'static str, String) {
    match requestor {
        // Credential-based — a paired device authenticated.
        "KeyfobRke" | "KeyfobPeps" | "PassiveEntry" | "PhoneApp" | "PhoneBle" | "NfcCard"
        | "NfcPhone" | "KeypadLock" => ("userid", requestor.to_string()),

        // Smart-* actuations are driven by a paired key being found
        // in cabin / trunk — credential-based, even though the
        // immediate trigger is internal logic.
        "SmartUnlock" | "SmartTrunkPop" => ("userid", requestor.to_string()),

        // Autonomous body-domain actuations — no driver, no credential.
        // Subject stays empty so downstream consumers can tell
        // "this wasn't a person".
        "AutoLock" | "AutoRelock" | "WalkAwayLock" | "CrashUnlock" | "DoubleLockRelease" => {
            ("default", String::new())
        }

        // Interior physical switches — someone inside the car pressed
        // a button.  Not credential-authenticated; we don't know who.
        "DoorTrimButton" | "SlamLock" => ("default", String::new()),

        // Unknown / future requestor strings.  Pass the raw string
        // through as Subject so it's debuggable, but mark Type as
        // `default` to signal "we don't know how to classify this".
        other => ("default", other.to_string()),
    }
}

pub struct DriverPlantModel<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> DriverPlantModel<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    pub async fn run(self) {
        tracing::info!("DriverPlantModel started");

        let mut rx = self.bus.subscribe(LAST_REQUESTOR).await;

        // Initial state — no actuation yet seen.  Publish so late
        // subscribers (HMI snapshot, telematics) get a deterministic
        // value rather than `None`.
        let _ = self
            .bus
            .publish(
                DRIVER_IDENTIFIER_TYPE,
                SignalValue::String("default".into()),
            )
            .await;
        let _ = self
            .bus
            .publish(
                DRIVER_IDENTIFIER_SUBJECT,
                SignalValue::String(String::new()),
            )
            .await;

        let mut last_type: &'static str = "default";
        let mut last_subject = String::new();

        while let Some(val) = rx.next().await {
            let requestor = match val {
                SignalValue::String(s) => s,
                _ => continue,
            };
            let (ty, subject) = classify(&requestor);

            if ty != last_type {
                if let Err(e) = self
                    .bus
                    .publish(DRIVER_IDENTIFIER_TYPE, SignalValue::String(ty.into()))
                    .await
                {
                    tracing::error!(error = %e, "DriverPlantModel: publish Type failed");
                }
                last_type = ty;
            }
            if subject != last_subject {
                if let Err(e) = self
                    .bus
                    .publish(
                        DRIVER_IDENTIFIER_SUBJECT,
                        SignalValue::String(subject.clone()),
                    )
                    .await
                {
                    tracing::error!(error = %e, "DriverPlantModel: publish Subject failed");
                }
                last_subject = subject;
            }
        }

        tracing::warn!("DriverPlantModel: requestor stream closed, exiting");
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
        let plant = DriverPlantModel::new(Arc::clone(&bus));
        tokio::spawn(plant.run());
        settle().await;
        bus
    }

    fn string_at(bus: &MockBus, path: VssPath) -> Option<String> {
        match bus.latest_value(path)? {
            SignalValue::String(s) => Some(s),
            _ => None,
        }
    }

    #[tokio::test]
    async fn initial_state_is_default_and_empty() {
        let bus = setup().await;
        assert_eq!(
            string_at(&bus, DRIVER_IDENTIFIER_TYPE).as_deref(),
            Some("default")
        );
        assert_eq!(
            string_at(&bus, DRIVER_IDENTIFIER_SUBJECT).as_deref(),
            Some("")
        );
    }

    #[tokio::test]
    async fn keyfob_rke_publishes_userid_and_subject() {
        let bus = setup().await;
        bus.inject(LAST_REQUESTOR, SignalValue::String("KeyfobRke".into()));
        settle().await;
        assert_eq!(
            string_at(&bus, DRIVER_IDENTIFIER_TYPE).as_deref(),
            Some("userid")
        );
        assert_eq!(
            string_at(&bus, DRIVER_IDENTIFIER_SUBJECT).as_deref(),
            Some("KeyfobRke")
        );
    }

    #[tokio::test]
    async fn passive_entry_classified_as_userid() {
        let bus = setup().await;
        bus.inject(LAST_REQUESTOR, SignalValue::String("PassiveEntry".into()));
        settle().await;
        assert_eq!(
            string_at(&bus, DRIVER_IDENTIFIER_TYPE).as_deref(),
            Some("userid")
        );
        assert_eq!(
            string_at(&bus, DRIVER_IDENTIFIER_SUBJECT).as_deref(),
            Some("PassiveEntry")
        );
    }

    #[tokio::test]
    async fn autonomous_lock_clears_subject() {
        let bus = setup().await;
        // First a credential auth.
        bus.inject(LAST_REQUESTOR, SignalValue::String("KeyfobRke".into()));
        settle().await;
        // Then an autonomous AutoLock — Subject should clear.
        bus.inject(LAST_REQUESTOR, SignalValue::String("AutoLock".into()));
        settle().await;
        assert_eq!(
            string_at(&bus, DRIVER_IDENTIFIER_TYPE).as_deref(),
            Some("default")
        );
        assert_eq!(
            string_at(&bus, DRIVER_IDENTIFIER_SUBJECT).as_deref(),
            Some("")
        );
    }

    #[tokio::test]
    async fn interior_switch_is_default_no_subject() {
        let bus = setup().await;
        bus.inject(LAST_REQUESTOR, SignalValue::String("DoorTrimButton".into()));
        settle().await;
        assert_eq!(
            string_at(&bus, DRIVER_IDENTIFIER_TYPE).as_deref(),
            Some("default")
        );
        assert_eq!(
            string_at(&bus, DRIVER_IDENTIFIER_SUBJECT).as_deref(),
            Some("")
        );
    }

    #[tokio::test]
    async fn unknown_requestor_passes_through_as_subject() {
        let bus = setup().await;
        bus.inject(
            LAST_REQUESTOR,
            SignalValue::String("FutureBiometric".into()),
        );
        settle().await;
        assert_eq!(
            string_at(&bus, DRIVER_IDENTIFIER_TYPE).as_deref(),
            Some("default"),
            "unknown requestor maps to default type"
        );
        assert_eq!(
            string_at(&bus, DRIVER_IDENTIFIER_SUBJECT).as_deref(),
            Some("FutureBiometric"),
            "unknown requestor passes raw string through for debugging"
        );
    }

    #[tokio::test]
    async fn redundant_same_requestor_does_not_republish() {
        let bus = setup().await;
        bus.inject(LAST_REQUESTOR, SignalValue::String("KeyfobRke".into()));
        settle().await;
        bus.clear_history();

        bus.inject(LAST_REQUESTOR, SignalValue::String("KeyfobRke".into()));
        bus.inject(LAST_REQUESTOR, SignalValue::String("KeyfobRke".into()));
        settle().await;

        let type_pubs = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == DRIVER_IDENTIFIER_TYPE)
            .count();
        let subj_pubs = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == DRIVER_IDENTIFIER_SUBJECT)
            .count();
        assert_eq!(type_pubs, 0, "no type change → no republish");
        assert_eq!(subj_pubs, 0, "no subject change → no republish");
    }

    #[tokio::test]
    async fn type_unchanged_but_subject_changed_only_republishes_subject() {
        // KeyfobRke → PassiveEntry: both map to Type=userid but
        // different Subject.  Type must NOT re-publish; Subject must.
        let bus = setup().await;
        bus.inject(LAST_REQUESTOR, SignalValue::String("KeyfobRke".into()));
        settle().await;
        bus.clear_history();

        bus.inject(LAST_REQUESTOR, SignalValue::String("PassiveEntry".into()));
        settle().await;

        let type_pubs = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == DRIVER_IDENTIFIER_TYPE)
            .count();
        let subj_pubs = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == DRIVER_IDENTIFIER_SUBJECT)
            .count();
        assert_eq!(type_pubs, 0, "userid → userid should not republish Type");
        assert_eq!(subj_pubs, 1, "Subject changed → exactly one republish");
        assert_eq!(
            string_at(&bus, DRIVER_IDENTIFIER_SUBJECT).as_deref(),
            Some("PassiveEntry")
        );
    }

    #[tokio::test]
    async fn smart_unlock_classified_as_userid() {
        let bus = setup().await;
        bus.inject(LAST_REQUESTOR, SignalValue::String("SmartUnlock".into()));
        settle().await;
        assert_eq!(
            string_at(&bus, DRIVER_IDENTIFIER_TYPE).as_deref(),
            Some("userid"),
            "SmartUnlock is triggered by a paired key being found — credential-backed"
        );
    }

    #[tokio::test]
    async fn crash_unlock_is_autonomous() {
        let bus = setup().await;
        bus.inject(LAST_REQUESTOR, SignalValue::String("CrashUnlock".into()));
        settle().await;
        assert_eq!(
            string_at(&bus, DRIVER_IDENTIFIER_TYPE).as_deref(),
            Some("default")
        );
        assert_eq!(
            string_at(&bus, DRIVER_IDENTIFIER_SUBJECT).as_deref(),
            Some("")
        );
    }

    #[test]
    fn classify_pure_function_covers_all_known_requestors() {
        // Spot-check the classifier without spinning up the actor.
        assert_eq!(classify("KeyfobRke").0, "userid");
        assert_eq!(classify("KeyfobPeps").0, "userid");
        assert_eq!(classify("PassiveEntry").0, "userid");
        assert_eq!(classify("PhoneApp").0, "userid");
        assert_eq!(classify("PhoneBle").0, "userid");
        assert_eq!(classify("NfcCard").0, "userid");
        assert_eq!(classify("NfcPhone").0, "userid");
        assert_eq!(classify("KeypadLock").0, "userid");
        assert_eq!(classify("SmartUnlock").0, "userid");
        assert_eq!(classify("SmartTrunkPop").0, "userid");

        assert_eq!(classify("AutoLock"), ("default", String::new()));
        assert_eq!(classify("AutoRelock"), ("default", String::new()));
        assert_eq!(classify("WalkAwayLock"), ("default", String::new()));
        assert_eq!(classify("CrashUnlock"), ("default", String::new()));
        assert_eq!(classify("DoubleLockRelease"), ("default", String::new()));
        assert_eq!(classify("DoorTrimButton"), ("default", String::new()));
        assert_eq!(classify("SlamLock"), ("default", String::new()));
    }
}
