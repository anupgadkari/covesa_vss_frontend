//! Auth-channel plant — derived "how was the vehicle just actuated"
//! signal, plus a deterministic seed of the VSS v6.0
//! `Vehicle.Driver.Identifier` branch.
//!
//! ## Two responsibilities
//!
//! ### 1. Publish `Vehicle.Controller.Body.LastAuthChannel` (extension)
//!
//! On every edge of `Vehicle.Cabin.LockStatus.LastRequestor` we map
//! the requestor string to a coarse channel classification —
//! `Keyfob`, `Phone`, `Nfc`, `Keypad`, `Manual`, `Automated`,
//! `Unknown`.  This is the actually-useful information carried by
//! LastRequestor: *how* was the vehicle actuated.  It is NOT a
//! statement about *who* the driver is.
//!
//! HMI / telematics / DTC services that previously had to substring-
//! match on LastRequestor strings can subscribe to this signal
//! instead.
//!
//! ### 2. Seed `Vehicle.Driver.Identifier.{Type,Subject}`
//!
//! VSS v6.0 expects something to publish the canonical
//! `Vehicle.Driver.Identifier` branch.  Driver identity = "which
//! paired credential / which person", not auth channel — so until
//! RKE / PassiveEntry thread slot-level info through (a planned
//! follow-up), we don't have enough to populate Subject honestly.
//!
//! What we *can* do is publish a deterministic empty seed at startup:
//!
//!   * `Identifier.Type = "default"` — VSS-standard enum value
//!     meaning "no specific identifier type".
//!   * `Identifier.Subject = ""` — no identity known.
//!
//! That establishes the canonical paths on the bus so late
//! subscribers (HMI snapshot, telematics) get a value rather than
//! `None`.  When slot wiring lands, a future PR populates Subject
//! with e.g. `"fob:1"` and bumps Type to `"userid"`.
//!
//! ## Why not on the Driver branch directly
//!
//! Auth channel ≠ driver identity.  "Keyfob" and "NFC" describe the
//! *channel* used to actuate; the *driver* is the person/credential
//! behind that channel.  Putting "Keyfob" into
//! `Vehicle.Driver.Identifier.Subject` would mislead any standards-
//! aware VSS consumer.  We keep them on separate paths.
//!
//! See `docs/post-peps-backlog.md` item #25 (sub-PR 8b).

use std::sync::Arc;

use futures::StreamExt;

use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

const LAST_REQUESTOR: VssPath = "Vehicle.Cabin.LockStatus.LastRequestor";

const DRIVER_IDENTIFIER_TYPE: VssPath = "Vehicle.Driver.Identifier.Type";
const DRIVER_IDENTIFIER_SUBJECT: VssPath = "Vehicle.Driver.Identifier.Subject";

const LAST_AUTH_CHANNEL: VssPath = "Vehicle.Controller.Body.LastAuthChannel";

/// Coarse classification of *how* the vehicle was just actuated.
///
/// Returns one of: `Keyfob`, `Phone`, `Nfc`, `Keypad`, `Manual`,
/// `Automated`, `Unknown`.  This is deliberately not the driver
/// identity — see module docs.
fn classify_channel(requestor: &str) -> &'static str {
    match requestor {
        // Paired RF / BLE-LE long-range fobs and PEPS scans.
        "KeyfobRke" | "KeyfobPeps" | "PassiveEntry" => "Keyfob",

        // Companion phone — either via cloud app or BLE proximity.
        "PhoneApp" | "PhoneBle" => "Phone",

        // NFC tap (card or phone in card-emulation mode).
        "NfcCard" | "NfcPhone" => "Nfc",

        // Doorhandle / B-pillar keypad.
        "KeypadLock" => "Keypad",

        // Interior physical switch — someone already inside the car.
        // Not channel-authenticated, hence "Manual".
        "DoorTrimButton" | "SlamLock" => "Manual",

        // Internal-logic-driven actuations (no human in the loop at
        // this moment).  SmartUnlock / SmartTrunkPop are also bundled
        // here because the immediate trigger is internal logic, even
        // though the upstream gate is a paired-key search.
        "AutoLock" | "AutoRelock" | "WalkAwayLock" | "CrashUnlock" | "DoubleLockRelease"
        | "SmartUnlock" | "SmartTrunkPop" => "Automated",

        // Future / unrecognised requestor.  Keep the canonical path
        // populated so subscribers always have a value.
        _ => "Unknown",
    }
}

pub struct AuthChannelPlant<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> AuthChannelPlant<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    pub async fn run(self) {
        tracing::info!("AuthChannelPlant started");

        let mut rx = self.bus.subscribe(LAST_REQUESTOR).await;

        // Seed canonical Driver.Identifier paths with deterministic
        // empties.  Future PR (slot-wiring) will populate these for
        // real.  Until then they stay at these initial values.
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

        // Seed the channel signal so late subscribers get a value.
        let _ = self
            .bus
            .publish(LAST_AUTH_CHANNEL, SignalValue::String("Unknown".into()))
            .await;

        let mut last_channel: &'static str = "Unknown";

        while let Some(val) = rx.next().await {
            let requestor = match val {
                SignalValue::String(s) => s,
                _ => continue,
            };
            let channel = classify_channel(&requestor);
            if channel == last_channel {
                continue; // idempotent — no edge.
            }
            if let Err(e) = self
                .bus
                .publish(LAST_AUTH_CHANNEL, SignalValue::String(channel.into()))
                .await
            {
                tracing::error!(error = %e, "AuthChannelPlant: publish failed");
            }
            last_channel = channel;
        }

        tracing::warn!("AuthChannelPlant: requestor stream closed, exiting");
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
        let plant = AuthChannelPlant::new(Arc::clone(&bus));
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
    async fn initial_seed_publishes_default_empty_and_unknown() {
        let bus = setup().await;
        assert_eq!(
            string_at(&bus, DRIVER_IDENTIFIER_TYPE).as_deref(),
            Some("default")
        );
        assert_eq!(
            string_at(&bus, DRIVER_IDENTIFIER_SUBJECT).as_deref(),
            Some("")
        );
        assert_eq!(
            string_at(&bus, LAST_AUTH_CHANNEL).as_deref(),
            Some("Unknown")
        );
    }

    #[tokio::test]
    async fn driver_identifier_never_changes_from_seed() {
        // Whole point of Option-A scope: we never populate Driver
        // branch from auth-channel info, because channel != identity.
        let bus = setup().await;
        for r in [
            "KeyfobRke",
            "PassiveEntry",
            "PhoneBle",
            "NfcCard",
            "KeypadLock",
            "AutoLock",
        ] {
            bus.inject(LAST_REQUESTOR, SignalValue::String(r.into()));
            settle().await;
        }
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
    async fn keyfob_requestors_map_to_keyfob_channel() {
        for r in ["KeyfobRke", "KeyfobPeps", "PassiveEntry"] {
            let bus = setup().await;
            bus.inject(LAST_REQUESTOR, SignalValue::String(r.into()));
            settle().await;
            assert_eq!(
                string_at(&bus, LAST_AUTH_CHANNEL).as_deref(),
                Some("Keyfob"),
                "{r} should map to Keyfob"
            );
        }
    }

    #[tokio::test]
    async fn phone_requestors_map_to_phone_channel() {
        for r in ["PhoneApp", "PhoneBle"] {
            let bus = setup().await;
            bus.inject(LAST_REQUESTOR, SignalValue::String(r.into()));
            settle().await;
            assert_eq!(
                string_at(&bus, LAST_AUTH_CHANNEL).as_deref(),
                Some("Phone"),
                "{r} should map to Phone"
            );
        }
    }

    #[tokio::test]
    async fn nfc_requestors_map_to_nfc_channel() {
        for r in ["NfcCard", "NfcPhone"] {
            let bus = setup().await;
            bus.inject(LAST_REQUESTOR, SignalValue::String(r.into()));
            settle().await;
            assert_eq!(
                string_at(&bus, LAST_AUTH_CHANNEL).as_deref(),
                Some("Nfc"),
                "{r} should map to Nfc"
            );
        }
    }

    #[tokio::test]
    async fn keypad_maps_to_keypad_channel() {
        let bus = setup().await;
        bus.inject(LAST_REQUESTOR, SignalValue::String("KeypadLock".into()));
        settle().await;
        assert_eq!(
            string_at(&bus, LAST_AUTH_CHANNEL).as_deref(),
            Some("Keypad")
        );
    }

    #[tokio::test]
    async fn interior_switches_map_to_manual_channel() {
        for r in ["DoorTrimButton", "SlamLock"] {
            let bus = setup().await;
            bus.inject(LAST_REQUESTOR, SignalValue::String(r.into()));
            settle().await;
            assert_eq!(
                string_at(&bus, LAST_AUTH_CHANNEL).as_deref(),
                Some("Manual"),
                "{r} should map to Manual"
            );
        }
    }

    #[tokio::test]
    async fn autonomous_requestors_map_to_automated_channel() {
        for r in [
            "AutoLock",
            "AutoRelock",
            "WalkAwayLock",
            "CrashUnlock",
            "DoubleLockRelease",
            "SmartUnlock",
            "SmartTrunkPop",
        ] {
            let bus = setup().await;
            bus.inject(LAST_REQUESTOR, SignalValue::String(r.into()));
            settle().await;
            assert_eq!(
                string_at(&bus, LAST_AUTH_CHANNEL).as_deref(),
                Some("Automated"),
                "{r} should map to Automated"
            );
        }
    }

    #[tokio::test]
    async fn unknown_requestor_maps_to_unknown_channel() {
        let bus = setup().await;
        bus.inject(
            LAST_REQUESTOR,
            SignalValue::String("FutureBiometric".into()),
        );
        settle().await;
        assert_eq!(
            string_at(&bus, LAST_AUTH_CHANNEL).as_deref(),
            Some("Unknown")
        );
    }

    #[tokio::test]
    async fn redundant_same_channel_does_not_republish() {
        let bus = setup().await;
        bus.inject(LAST_REQUESTOR, SignalValue::String("KeyfobRke".into()));
        settle().await;
        bus.clear_history();

        bus.inject(LAST_REQUESTOR, SignalValue::String("KeyfobPeps".into()));
        bus.inject(LAST_REQUESTOR, SignalValue::String("PassiveEntry".into()));
        settle().await;

        let pubs = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == LAST_AUTH_CHANNEL)
            .count();
        assert_eq!(pubs, 0, "same channel — no republish");
    }

    #[tokio::test]
    async fn channel_transitions_each_publish() {
        let bus = setup().await;
        bus.clear_history();

        for req in ["KeyfobRke", "PhoneApp", "NfcCard", "AutoLock"] {
            bus.inject(LAST_REQUESTOR, SignalValue::String(req.into()));
            settle().await;
        }
        let pubs: Vec<String> = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == LAST_AUTH_CHANNEL)
            .filter_map(|(_, v)| match v {
                SignalValue::String(s) => Some(s.clone()),
                _ => None,
            })
            .collect();
        assert_eq!(pubs, vec!["Keyfob", "Phone", "Nfc", "Automated"]);
    }

    #[test]
    fn classify_channel_pure_function() {
        assert_eq!(classify_channel("KeyfobRke"), "Keyfob");
        assert_eq!(classify_channel("KeyfobPeps"), "Keyfob");
        assert_eq!(classify_channel("PassiveEntry"), "Keyfob");
        assert_eq!(classify_channel("PhoneApp"), "Phone");
        assert_eq!(classify_channel("PhoneBle"), "Phone");
        assert_eq!(classify_channel("NfcCard"), "Nfc");
        assert_eq!(classify_channel("NfcPhone"), "Nfc");
        assert_eq!(classify_channel("KeypadLock"), "Keypad");
        assert_eq!(classify_channel("DoorTrimButton"), "Manual");
        assert_eq!(classify_channel("SlamLock"), "Manual");
        assert_eq!(classify_channel("AutoLock"), "Automated");
        assert_eq!(classify_channel("AutoRelock"), "Automated");
        assert_eq!(classify_channel("WalkAwayLock"), "Automated");
        assert_eq!(classify_channel("CrashUnlock"), "Automated");
        assert_eq!(classify_channel("DoubleLockRelease"), "Automated");
        assert_eq!(classify_channel("SmartUnlock"), "Automated");
        assert_eq!(classify_channel("SmartTrunkPop"), "Automated");
        assert_eq!(classify_channel("WhateverNew"), "Unknown");
    }
}
