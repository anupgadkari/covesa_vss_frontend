//! PEPS Plant Model — simulates key fobs, BLE phones, and NFC cards.
//!
//! This plant model is the physical-world counterpart to the PEPS and RKE
//! feature logic. It simulates up to 6 key fobs (4 paired + 2 unpaired
//! intruders), 2 BLE-paired phones, and 2 NFC cards.
//!
//! ## How it works
//!
//! The HMI or test harness positions devices by publishing zone signals
//! (e.g., `Body.PEPS.Plant.KeyFob.1.Zone = "DriverDoor"`). When the
//! vehicle-side feature logic sends a challenge, each device in a
//! compatible zone responds with an AES-128 encrypted reply.
//!
//! ```text
//!  ┌───────────┐    zone signals     ┌─────────────────┐
//!  │ HMI/Test  │ ─────────────────→  │  PepsPlantModel │
//!  └───────────┘    button presses   │                 │
//!                                    │  6 KeyFobs      │
//!  ┌───────────┐    challenges       │  2 BlePhones    │
//!  │  Feature   │ ─────────────────→ │  2 NfcCards     │
//!  │  Logic     │ ←───────────────── │                 │
//!  └───────────┘    responses/RSSI   └─────────────────┘
//! ```
//!
//! ## Zone exclusivity
//!
//! Cabin and Trunk are mutually exclusive — a device cannot be in both.
//! The plant model enforces this: moving a device to Trunk automatically
//! clears a Cabin position and vice versa (logged as a warning).
//!
//! ## Default startup
//!
//! On startup, 2 paired fobs and 2 BLE phones are created (all OutOfRange).
//! Additional fobs/phones are provisioned but inactive until pairing
//! signals or zone changes arrive.

pub mod crypto;
pub mod device;
pub mod signals;
pub mod zone;

use std::sync::Arc;

use device::{BlePhone, FobButton, KeyFob, NfcCard};
use zone::{NfcPosition, Zone};

use crate::ipc_message::SignalValue;
use crate::signal_bus::SignalBus;

/// Default shared secrets for simulation (deterministic, per-device).
/// In production these would be provisioned during key pairing.
fn default_secret(device_type: u8, index: u8) -> crypto::SharedSecret {
    let mut key = [0u8; 16];
    key[0] = device_type;
    key[1] = index;
    // Fill rest with a simple pattern for reproducibility
    for (i, byte) in key.iter_mut().enumerate().skip(2) {
        *byte = (device_type.wrapping_mul(17))
            .wrapping_add(index.wrapping_mul(31).wrapping_add(i as u8));
    }
    key
}

/// The PEPS plant model orchestrator.
pub struct PepsPlantModel<B: SignalBus> {
    bus: Arc<B>,
    pub fobs: Vec<KeyFob>,
    pub phones: Vec<BlePhone>,
    pub nfc_cards: Vec<NfcCard>,
}

impl<B: SignalBus> PepsPlantModel<B> {
    /// Create a new PEPS plant model with default device inventory.
    ///
    /// Default: 4 paired fobs + 2 unpaired, 2 BLE phones, 2 NFC cards.
    /// All devices start `OutOfRange` / `NotPresent`.
    pub fn new(bus: Arc<B>) -> Self {
        let fobs = (1..=6)
            .map(|i| {
                let paired = i <= 4;
                KeyFob::new(i, paired, default_secret(b'F', i))
            })
            .collect();

        let phones = (1..=2)
            .map(|i| BlePhone::new(i, default_secret(b'P', i)))
            .collect();

        let nfc_cards = (1..=2)
            .map(|i| NfcCard::new(i, default_secret(b'N', i)))
            .collect();

        Self {
            bus,
            fobs,
            phones,
            nfc_cards,
        }
    }

    /// Move a key fob to a new zone.
    /// Automatically publishes RSSI if the fob enters an LF-capable zone,
    /// or clears RSSI if it leaves LF range.
    pub async fn set_fob_zone(&mut self, index: usize, zone: Zone) {
        let fob = match self.fobs.get_mut(index) {
            Some(f) => f,
            None => return,
        };
        let old = fob.zone;
        fob.zone = zone;
        tracing::debug!(
            fob = fob.index,
            from = %old,
            to = %zone,
            "PEPS plant: fob zone changed"
        );

        // Publish RSSI feedback when in an LF zone, clear when leaving.
        let rssi_signal = signals::KEYFOB_RSSIS[index];
        if zone.supports_rssi() {
            let rssi = device::RssiResponse::for_zone(zone);
            let _ = self
                .bus
                .publish(rssi_signal, SignalValue::String(rssi.to_signal_string()))
                .await;
        } else if old.supports_rssi() {
            // Left LF range — clear RSSI
            let _ = self
                .bus
                .publish(rssi_signal, SignalValue::String("{}".into()))
                .await;
        }
    }

    /// Move a BLE phone to a new zone.
    /// Automatically publishes BLE RSSI feedback.
    pub async fn set_phone_zone(&mut self, index: usize, zone: Zone) {
        let phone = match self.phones.get_mut(index) {
            Some(p) => p,
            None => return,
        };
        let old = phone.zone;
        phone.zone = zone;
        tracing::debug!(
            phone = phone.index,
            from = %old,
            to = %zone,
            "PEPS plant: phone zone changed"
        );

        let rssi_signal = signals::PHONE_RSSIS[index];
        if zone.supports_rssi() {
            let rssi = device::RssiResponse::for_zone(zone);
            let _ = self
                .bus
                .publish(rssi_signal, SignalValue::String(rssi.to_signal_string()))
                .await;
        } else if old.supports_rssi() {
            let _ = self
                .bus
                .publish(rssi_signal, SignalValue::String("{}".into()))
                .await;
        }
    }

    /// Set an NFC card's position.
    pub fn set_nfc_position(&mut self, index: usize, position: NfcPosition) {
        if let Some(card) = self.nfc_cards.get_mut(index) {
            let old = card.position;
            card.position = position;
            tracing::debug!(
                card = card.index,
                from = %old,
                to = %position,
                "PEPS plant: NFC position changed"
            );
        }
    }

    /// Handle a fob button press: increment rolling code and publish RF message.
    pub async fn handle_fob_button(&mut self, fob_index: usize, button: FobButton) {
        let fob = match self.fobs.get_mut(fob_index) {
            Some(f) => f,
            None => return,
        };

        if let Some(rf_msg) = fob.press_button(button) {
            let signal = signals::KEYFOB_RF_MSGS[fob_index];
            // Encode as hex string: action + encrypted rolling code
            let hex: String = rf_msg
                .encrypted_rolling_code
                .iter()
                .map(|b| format!("{b:02x}"))
                .collect();
            let payload = format!("{}:{}", rf_msg.action.as_str(), hex);
            if let Err(e) = self.bus.publish(signal, SignalValue::String(payload)).await {
                tracing::error!(error = %e, "PEPS plant: failed to publish RF message");
            }
            tracing::info!(
                fob = fob.index,
                action = rf_msg.action.as_str(),
                counter = rf_msg.counter,
                "PEPS plant: fob RF button press"
            );
        }
    }

    /// Handle an LF challenge from the vehicle. All fobs and phones in
    /// challenge-response-capable zones respond.
    pub async fn handle_lf_challenge(&self, nonce: &crypto::Challenge) {
        for (i, fob) in self.fobs.iter().enumerate() {
            if let Some(response) = fob.respond_to_challenge(nonce) {
                let signal = signals::KEYFOB_CHALLENGE_RESPS[i];
                let hex: String = response.iter().map(|b| format!("{b:02x}")).collect();
                if let Err(e) = self.bus.publish(signal, SignalValue::String(hex)).await {
                    tracing::error!(error = %e, "PEPS plant: failed to publish fob challenge response");
                }
            }
        }
    }

    /// Handle a BLE challenge. All phones in challenge-response-capable zones respond.
    pub async fn handle_ble_challenge(&self, nonce: &crypto::Challenge) {
        for (i, phone) in self.phones.iter().enumerate() {
            if let Some(response) = phone.respond_to_challenge(nonce) {
                let signal = signals::PHONE_CHALLENGE_RESPS[i];
                let hex: String = response.iter().map(|b| format!("{b:02x}")).collect();
                if let Err(e) = self.bus.publish(signal, SignalValue::String(hex)).await {
                    tracing::error!(error = %e, "PEPS plant: failed to publish phone challenge response");
                }
            }
        }
    }

    /// Handle an NFC challenge. All NFC cards at reader positions respond.
    pub async fn handle_nfc_challenge(&self, nonce: &crypto::Challenge) {
        for (i, card) in self.nfc_cards.iter().enumerate() {
            if let Some(response) = card.respond_to_challenge(nonce) {
                let signal = signals::NFC_CHALLENGE_RESPS[i];
                let hex: String = response.iter().map(|b| format!("{b:02x}")).collect();
                if let Err(e) = self.bus.publish(signal, SignalValue::String(hex)).await {
                    tracing::error!(error = %e, "PEPS plant: failed to publish NFC challenge response");
                }
            }
        }
    }

    /// Handle an approach poll. All fobs/phones in LF-capable zones publish RSSI.
    pub async fn handle_approach_poll(&self) {
        for (i, fob) in self.fobs.iter().enumerate() {
            if let Some(rssi) = fob.rssi_response() {
                let signal = signals::KEYFOB_RSSIS[i];
                if let Err(e) = self
                    .bus
                    .publish(signal, SignalValue::String(rssi.to_signal_string()))
                    .await
                {
                    tracing::error!(error = %e, "PEPS plant: failed to publish fob RSSI");
                }
            }
        }
        for (i, phone) in self.phones.iter().enumerate() {
            if let Some(rssi) = phone.rssi_response() {
                let signal = signals::PHONE_RSSIS[i];
                if let Err(e) = self
                    .bus
                    .publish(signal, SignalValue::String(rssi.to_signal_string()))
                    .await
                {
                    tracing::error!(error = %e, "PEPS plant: failed to publish phone RSSI");
                }
            }
        }
    }

    /// The main event loop — subscribe to all input signals and dispatch.
    pub async fn run(mut self) {
        use futures::StreamExt;

        tracing::info!(
            "PEPS plant model started ({} fobs, {} phones, {} NFC cards)",
            self.fobs.len(),
            self.phones.len(),
            self.nfc_cards.len()
        );

        // Subscribe to fob zone signals — individual bindings to satisfy borrow checker.
        let mut fz0 = self.bus.subscribe(signals::KEYFOB_ZONES[0]).await;
        let mut fz1 = self.bus.subscribe(signals::KEYFOB_ZONES[1]).await;
        let mut fz2 = self.bus.subscribe(signals::KEYFOB_ZONES[2]).await;
        let mut fz3 = self.bus.subscribe(signals::KEYFOB_ZONES[3]).await;
        let mut fz4 = self.bus.subscribe(signals::KEYFOB_ZONES[4]).await;
        let mut fz5 = self.bus.subscribe(signals::KEYFOB_ZONES[5]).await;

        // Fob button signals (paired fobs 1..4 only).
        let mut fb0 = self.bus.subscribe(signals::KEYFOB_BUTTONS[0]).await;
        let mut fb1 = self.bus.subscribe(signals::KEYFOB_BUTTONS[1]).await;
        let mut fb2 = self.bus.subscribe(signals::KEYFOB_BUTTONS[2]).await;
        let mut fb3 = self.bus.subscribe(signals::KEYFOB_BUTTONS[3]).await;

        // Phone zone signals.
        let mut pz0 = self.bus.subscribe(signals::PHONE_ZONES[0]).await;
        let mut pz1 = self.bus.subscribe(signals::PHONE_ZONES[1]).await;

        // NFC position signals.
        let mut np0 = self.bus.subscribe(signals::NFC_POSITIONS[0]).await;
        let mut np1 = self.bus.subscribe(signals::NFC_POSITIONS[1]).await;

        // Vehicle-side challenge/poll signals.
        let mut lf_challenge = self.bus.subscribe(signals::PEPS_LF_CHALLENGE).await;
        let mut ble_challenge = self.bus.subscribe(signals::PEPS_BLE_CHALLENGE).await;
        let mut nfc_challenge = self.bus.subscribe(signals::PEPS_NFC_CHALLENGE).await;
        let mut approach_poll = self.bus.subscribe(signals::PEPS_APPROACH_POLL).await;

        loop {
            tokio::select! {
                // ── Fob zone changes ───────────────────────────────
                Some(val) = fz0.next() => {
                    if let Some(z) = str_to_zone(&val) { self.set_fob_zone(0, z).await; }
                }
                Some(val) = fz1.next() => {
                    if let Some(z) = str_to_zone(&val) { self.set_fob_zone(1, z).await; }
                }
                Some(val) = fz2.next() => {
                    if let Some(z) = str_to_zone(&val) { self.set_fob_zone(2, z).await; }
                }
                Some(val) = fz3.next() => {
                    if let Some(z) = str_to_zone(&val) { self.set_fob_zone(3, z).await; }
                }
                Some(val) = fz4.next() => {
                    if let Some(z) = str_to_zone(&val) { self.set_fob_zone(4, z).await; }
                }
                Some(val) = fz5.next() => {
                    if let Some(z) = str_to_zone(&val) { self.set_fob_zone(5, z).await; }
                }

                // ── Fob button presses ─────────────────────────────
                Some(val) = fb0.next() => {
                    if let Some(b) = str_to_button(&val) { self.handle_fob_button(0, b).await; }
                }
                Some(val) = fb1.next() => {
                    if let Some(b) = str_to_button(&val) { self.handle_fob_button(1, b).await; }
                }
                Some(val) = fb2.next() => {
                    if let Some(b) = str_to_button(&val) { self.handle_fob_button(2, b).await; }
                }
                Some(val) = fb3.next() => {
                    if let Some(b) = str_to_button(&val) { self.handle_fob_button(3, b).await; }
                }

                // ── Phone zone changes ─────────────────────────────
                Some(val) = pz0.next() => {
                    if let Some(z) = str_to_zone(&val) { self.set_phone_zone(0, z).await; }
                }
                Some(val) = pz1.next() => {
                    if let Some(z) = str_to_zone(&val) { self.set_phone_zone(1, z).await; }
                }

                // ── NFC position changes ───────────────────────────
                Some(val) = np0.next() => {
                    if let Some(p) = str_to_nfc_pos(&val) { self.set_nfc_position(0, p); }
                }
                Some(val) = np1.next() => {
                    if let Some(p) = str_to_nfc_pos(&val) { self.set_nfc_position(1, p); }
                }

                // ── Vehicle-side challenges ────────────────────────
                Some(val) = lf_challenge.next() => {
                    if let Some(nonce) = str_to_nonce(&val) {
                        self.handle_lf_challenge(&nonce).await;
                    }
                }
                Some(val) = ble_challenge.next() => {
                    if let Some(nonce) = str_to_nonce(&val) {
                        self.handle_ble_challenge(&nonce).await;
                    }
                }
                Some(val) = nfc_challenge.next() => {
                    if let Some(nonce) = str_to_nonce(&val) {
                        self.handle_nfc_challenge(&nonce).await;
                    }
                }
                Some(_val) = approach_poll.next() => {
                    self.handle_approach_poll().await;
                }

                else => break,
            }
        }

        tracing::warn!("PEPS plant model: all streams closed, exiting");
    }
}

// ---------------------------------------------------------------------------
// Signal value parsing helpers
// ---------------------------------------------------------------------------

fn str_to_zone(val: &SignalValue) -> Option<Zone> {
    match val {
        SignalValue::String(s) => Zone::from_str_value(s),
        _ => None,
    }
}

fn str_to_button(val: &SignalValue) -> Option<FobButton> {
    match val {
        SignalValue::String(s) => FobButton::from_str_value(s),
        _ => None,
    }
}

fn str_to_nfc_pos(val: &SignalValue) -> Option<NfcPosition> {
    match val {
        SignalValue::String(s) => NfcPosition::from_str_value(s),
        _ => None,
    }
}

/// Parse a hex-encoded 128-bit nonce from a string signal value.
fn str_to_nonce(val: &SignalValue) -> Option<crypto::Challenge> {
    match val {
        SignalValue::String(s) => {
            let bytes: Vec<u8> = (0..s.len())
                .step_by(2)
                .filter_map(|i| u8::from_str_radix(&s[i..i + 2], 16).ok())
                .collect();
            if bytes.len() == 16 {
                let mut nonce = [0u8; 16];
                nonce.copy_from_slice(&bytes);
                Some(nonce)
            } else {
                None
            }
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;

    #[test]
    fn default_inventory() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(bus);

        assert_eq!(model.fobs.len(), 6);
        assert_eq!(model.phones.len(), 2);
        assert_eq!(model.nfc_cards.len(), 2);

        // First 4 fobs paired, last 2 unpaired
        for i in 0..4 {
            assert!(model.fobs[i].paired, "fob {} should be paired", i + 1);
        }
        for i in 4..6 {
            assert!(!model.fobs[i].paired, "fob {} should be unpaired", i + 1);
        }

        // All start out of range
        for fob in &model.fobs {
            assert_eq!(fob.zone, Zone::OutOfRange);
        }
        for phone in &model.phones {
            assert_eq!(phone.zone, Zone::OutOfRange);
        }
        for card in &model.nfc_cards {
            assert_eq!(card.position, NfcPosition::NotPresent);
        }
    }

    #[tokio::test]
    async fn set_fob_zone() {
        let bus = Arc::new(MockBus::new());
        let mut model = PepsPlantModel::new(Arc::clone(&bus));

        model.set_fob_zone(0, Zone::DriverDoor).await;
        assert_eq!(model.fobs[0].zone, Zone::DriverDoor);

        model.set_fob_zone(0, Zone::Cabin).await;
        assert_eq!(model.fobs[0].zone, Zone::Cabin);
    }

    #[tokio::test]
    async fn set_phone_zone() {
        let bus = Arc::new(MockBus::new());
        let mut model = PepsPlantModel::new(Arc::clone(&bus));

        model.set_phone_zone(0, Zone::Approach).await;
        assert_eq!(model.phones[0].zone, Zone::Approach);
    }

    #[test]
    fn set_nfc_position() {
        let bus = Arc::new(MockBus::new());
        let mut model = PepsPlantModel::new(bus);

        model.set_nfc_position(0, NfcPosition::DriverHandle);
        assert_eq!(model.nfc_cards[0].position, NfcPosition::DriverHandle);
    }

    #[test]
    fn each_device_has_unique_secret() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(bus);

        let mut secrets: Vec<crypto::SharedSecret> = Vec::new();
        for fob in &model.fobs {
            assert!(
                !secrets.contains(&fob.secret),
                "fob {} has duplicate secret",
                fob.index
            );
            secrets.push(fob.secret);
        }
        for phone in &model.phones {
            assert!(
                !secrets.contains(&phone.secret),
                "phone {} has duplicate secret",
                phone.index
            );
            secrets.push(phone.secret);
        }
        for card in &model.nfc_cards {
            assert!(
                !secrets.contains(&card.secret),
                "NFC card {} has duplicate secret",
                card.index
            );
            secrets.push(card.secret);
        }
    }

    #[test]
    fn str_to_zone_parsing() {
        assert_eq!(
            str_to_zone(&SignalValue::String("DriverDoor".into())),
            Some(Zone::DriverDoor)
        );
        assert_eq!(
            str_to_zone(&SignalValue::String("Trunk".into())),
            Some(Zone::Trunk)
        );
        assert_eq!(str_to_zone(&SignalValue::String("bogus".into())), None);
        assert_eq!(str_to_zone(&SignalValue::Bool(true)), None);
    }

    #[test]
    fn str_to_nonce_parsing() {
        // Valid 32-char hex → 16 bytes
        let hex = "2b7e151628aed2a6abf7158809cf4f3c";
        let nonce = str_to_nonce(&SignalValue::String(hex.into()));
        assert!(nonce.is_some());
        assert_eq!(nonce.unwrap()[0], 0x2b);
        assert_eq!(nonce.unwrap()[15], 0x3c);

        // Too short
        assert!(str_to_nonce(&SignalValue::String("abcd".into())).is_none());

        // Wrong type
        assert!(str_to_nonce(&SignalValue::Bool(true)).is_none());
    }

    #[tokio::test]
    async fn handle_fob_button_publishes_rf_message() {
        let bus = Arc::new(MockBus::new());
        let mut model = PepsPlantModel::new(Arc::clone(&bus));

        model.set_fob_zone(0, Zone::RfRange).await;
        model.handle_fob_button(0, FobButton::Lock).await;

        let history = bus.history();
        let rf_msgs: Vec<_> = history
            .iter()
            .filter(|(path, _)| *path == signals::KEYFOB_1_RF_MSG)
            .collect();
        assert_eq!(rf_msgs.len(), 1, "should publish one RF message");

        if let SignalValue::String(payload) = &rf_msgs[0].1 {
            assert!(payload.starts_with("LOCK:"), "payload: {payload}");
        } else {
            panic!("RF message should be a String");
        }
    }

    #[tokio::test]
    async fn handle_lf_challenge_responds_from_proximity_fobs() {
        let bus = Arc::new(MockBus::new());
        let mut model = PepsPlantModel::new(Arc::clone(&bus));

        // Put fob 1 at driver door, fob 2 at approach, rest out of range
        model.set_fob_zone(0, Zone::DriverDoor).await;
        model.set_fob_zone(1, Zone::Approach).await;

        bus.clear_history(); // clear RSSI publishes from zone changes
        let nonce = [0x42u8; 16];
        model.handle_lf_challenge(&nonce).await;

        let history = bus.history();

        // Fob 1 should respond (proximity)
        let fob1_resps: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_1_CHALLENGE_RESP)
            .collect();
        assert_eq!(fob1_resps.len(), 1, "fob 1 at DriverDoor should respond");

        // Fob 2 should NOT respond (approach = no challenge-response)
        let fob2_resps: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_2_CHALLENGE_RESP)
            .collect();
        assert_eq!(
            fob2_resps.len(),
            0,
            "fob 2 at Approach should not respond to challenge"
        );
    }

    #[tokio::test]
    async fn handle_approach_poll_responds_from_lf_zones() {
        let bus = Arc::new(MockBus::new());
        let mut model = PepsPlantModel::new(Arc::clone(&bus));

        model.set_fob_zone(0, Zone::Approach).await;
        model.set_fob_zone(1, Zone::OutOfRange).await;
        model.set_phone_zone(0, Zone::DriverDoor).await;

        bus.clear_history(); // clear RSSI publishes from zone changes
        model.handle_approach_poll().await;

        let history = bus.history();

        // Fob 1 (approach) should publish RSSI
        assert!(
            history.iter().any(|(p, _)| *p == signals::KEYFOB_1_RSSI),
            "fob 1 at Approach should publish RSSI"
        );
        // Fob 2 (out of range) should not
        assert!(
            !history.iter().any(|(p, _)| *p == signals::KEYFOB_2_RSSI),
            "fob 2 OutOfRange should not publish RSSI"
        );
        // Phone 1 (driver door) should publish RSSI
        assert!(
            history.iter().any(|(p, _)| *p == signals::PHONE_1_RSSI),
            "phone 1 at DriverDoor should publish RSSI"
        );
    }

    // ── Phase 2: automatic RSSI on zone change ────────────────────

    #[tokio::test]
    async fn fob_zone_change_to_lf_publishes_rssi() {
        let bus = Arc::new(MockBus::new());
        let mut model = PepsPlantModel::new(Arc::clone(&bus));

        model.set_fob_zone(0, Zone::DriverDoor).await;

        let history = bus.history();
        let rssi_msgs: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_1_RSSI)
            .collect();
        assert_eq!(
            rssi_msgs.len(),
            1,
            "zone change to LF should auto-publish RSSI"
        );

        if let SignalValue::String(payload) = &rssi_msgs[0].1 {
            assert!(
                payload.contains("\"driver\":"),
                "RSSI should contain driver antenna: {payload}"
            );
        } else {
            panic!("RSSI should be a String signal");
        }
    }

    #[tokio::test]
    async fn fob_zone_change_to_approach_publishes_weaker_rssi() {
        let bus = Arc::new(MockBus::new());
        let mut model = PepsPlantModel::new(Arc::clone(&bus));

        model.set_fob_zone(0, Zone::Approach).await;

        let history = bus.history();
        let rssi_msgs: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_1_RSSI)
            .collect();
        assert_eq!(rssi_msgs.len(), 1);

        // Approach RSSI should have weaker values than proximity
        if let SignalValue::String(payload) = &rssi_msgs[0].1 {
            // -75 dBm for approach vs -30 dBm for proximity
            assert!(
                payload.contains("-75"),
                "approach RSSI should show ~-75 dBm: {payload}"
            );
        }
    }

    #[tokio::test]
    async fn fob_zone_change_to_out_of_range_clears_rssi() {
        let bus = Arc::new(MockBus::new());
        let mut model = PepsPlantModel::new(Arc::clone(&bus));

        // First move to an LF zone, then out of range
        model.set_fob_zone(0, Zone::DriverDoor).await;
        bus.clear_history();

        model.set_fob_zone(0, Zone::OutOfRange).await;

        let history = bus.history();
        let rssi_msgs: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_1_RSSI)
            .collect();
        assert_eq!(rssi_msgs.len(), 1, "should publish cleared RSSI");

        if let SignalValue::String(payload) = &rssi_msgs[0].1 {
            assert_eq!(payload, "{}", "out-of-range should clear RSSI to empty");
        }
    }

    #[tokio::test]
    async fn fob_zone_change_rf_range_no_rssi() {
        let bus = Arc::new(MockBus::new());
        let mut model = PepsPlantModel::new(Arc::clone(&bus));

        // OutOfRange → RfRange: neither has LF, so no RSSI publish
        model.set_fob_zone(0, Zone::RfRange).await;

        let history = bus.history();
        let rssi_msgs: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_1_RSSI)
            .collect();
        assert_eq!(rssi_msgs.len(), 0, "RF range should not publish RSSI");
    }

    #[tokio::test]
    async fn phone_zone_change_publishes_rssi() {
        let bus = Arc::new(MockBus::new());
        let mut model = PepsPlantModel::new(Arc::clone(&bus));

        model.set_phone_zone(0, Zone::PassengerDoor).await;

        let history = bus.history();
        let rssi_msgs: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::PHONE_1_RSSI)
            .collect();
        assert_eq!(
            rssi_msgs.len(),
            1,
            "phone zone change should auto-publish RSSI"
        );

        if let SignalValue::String(payload) = &rssi_msgs[0].1 {
            // At passenger door, passenger antenna should be strongest (-30)
            assert!(
                payload.contains("\"passenger\":-30"),
                "passenger antenna should be strongest: {payload}"
            );
        }
    }

    #[tokio::test]
    async fn fob_rssi_values_differ_by_zone() {
        let bus = Arc::new(MockBus::new());
        let mut model = PepsPlantModel::new(Arc::clone(&bus));

        // Move fob through several zones, check RSSI changes
        model.set_fob_zone(0, Zone::DriverDoor).await;
        model.set_fob_zone(0, Zone::Trunk).await;

        let history = bus.history();
        let rssi_msgs: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_1_RSSI)
            .collect();
        assert_eq!(
            rssi_msgs.len(),
            2,
            "two zone changes should produce two RSSI publishes"
        );

        // The two payloads should differ (driver door vs trunk)
        if let (SignalValue::String(p1), SignalValue::String(p2)) =
            (&rssi_msgs[0].1, &rssi_msgs[1].1)
        {
            assert_ne!(p1, p2, "RSSI should differ between DriverDoor and Trunk");
        }
    }

    #[tokio::test]
    async fn run_loop_responds_to_zone_signal() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));

        let handle = tokio::spawn(model.run());
        // Let the event loop subscribe
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Inject a zone change via the bus
        bus.inject(
            signals::KEYFOB_1_ZONE,
            SignalValue::String("DriverDoor".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let rssi_msgs: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_1_RSSI)
            .collect();
        assert_eq!(
            rssi_msgs.len(),
            1,
            "event loop should auto-publish RSSI on zone change"
        );

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_responds_to_approach_poll() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Place fob 1 at approach, then trigger a poll
        bus.inject(
            signals::KEYFOB_1_ZONE,
            SignalValue::String("Approach".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();
        bus.inject(signals::PEPS_APPROACH_POLL, SignalValue::String("1".into()));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let rssi_msgs: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_1_RSSI)
            .collect();
        assert_eq!(
            rssi_msgs.len(),
            1,
            "approach poll should trigger RSSI from fob in approach zone"
        );

        handle.abort();
    }

    // ── Phase 3: LF / BLE / NFC challenge-response via event loop ──

    /// Helper: decode a hex string to bytes.
    fn hex_to_bytes(hex: &str) -> Vec<u8> {
        (0..hex.len())
            .step_by(2)
            .filter_map(|i| u8::from_str_radix(&hex[i..i + 2], 16).ok())
            .collect()
    }

    /// Helper: encode bytes to hex string (for nonce injection).
    fn bytes_to_hex(bytes: &[u8]) -> String {
        bytes.iter().map(|b| format!("{b:02x}")).collect()
    }

    #[tokio::test]
    async fn run_loop_lf_challenge_fob_in_proximity_responds() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));

        // Grab fob 1's secret before moving model into the task
        let fob1_secret = model.fobs[0].secret;

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Place fob 1 at DriverDoor (proximity)
        bus.inject(
            signals::KEYFOB_1_ZONE,
            SignalValue::String("DriverDoor".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        // Send LF challenge with a known nonce
        let nonce = [0x42u8; 16];
        bus.inject(
            signals::PEPS_LF_CHALLENGE,
            SignalValue::String(bytes_to_hex(&nonce)),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let fob1_resps: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_1_CHALLENGE_RESP)
            .collect();
        assert_eq!(fob1_resps.len(), 1, "fob 1 in proximity should respond");

        // Verify the response is cryptographically correct
        if let SignalValue::String(hex_resp) = &fob1_resps[0].1 {
            let resp_bytes = hex_to_bytes(hex_resp);
            assert_eq!(resp_bytes.len(), 16, "response should be 16 bytes");

            let expected = crypto::compute_challenge_response(&fob1_secret, &nonce);
            assert_eq!(
                resp_bytes.as_slice(),
                &expected,
                "response should match AES-128(secret, nonce)"
            );
        } else {
            panic!("challenge response should be a String");
        }

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_lf_challenge_fob_in_approach_does_not_respond() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Place fob 1 at Approach (only RSSI, no challenge-response)
        bus.inject(
            signals::KEYFOB_1_ZONE,
            SignalValue::String("Approach".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        let nonce = [0x77u8; 16];
        bus.inject(
            signals::PEPS_LF_CHALLENGE,
            SignalValue::String(bytes_to_hex(&nonce)),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let fob1_resps: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_1_CHALLENGE_RESP)
            .collect();
        assert_eq!(
            fob1_resps.len(),
            0,
            "fob in approach should NOT respond to LF challenge"
        );

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_lf_challenge_multiple_fobs_in_different_zones() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));
        let fob1_secret = model.fobs[0].secret;
        let fob3_secret = model.fobs[2].secret;

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Fob 1 at Cabin (proximity), fob 2 at OutOfRange, fob 3 at Trunk (proximity)
        bus.inject(signals::KEYFOB_1_ZONE, SignalValue::String("Cabin".into()));
        bus.inject(signals::KEYFOB_3_ZONE, SignalValue::String("Trunk".into()));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        let nonce = [0xABu8; 16];
        bus.inject(
            signals::PEPS_LF_CHALLENGE,
            SignalValue::String(bytes_to_hex(&nonce)),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();

        // Fob 1 (Cabin) should respond
        let fob1: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_1_CHALLENGE_RESP)
            .collect();
        assert_eq!(fob1.len(), 1, "fob 1 in Cabin should respond");
        if let SignalValue::String(hex) = &fob1[0].1 {
            let expected = crypto::compute_challenge_response(&fob1_secret, &nonce);
            assert_eq!(hex_to_bytes(hex).as_slice(), &expected);
        }

        // Fob 2 (OutOfRange) should NOT respond
        let fob2: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_2_CHALLENGE_RESP)
            .collect();
        assert_eq!(fob2.len(), 0, "fob 2 OutOfRange should not respond");

        // Fob 3 (Trunk) should respond
        let fob3: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_3_CHALLENGE_RESP)
            .collect();
        assert_eq!(fob3.len(), 1, "fob 3 in Trunk should respond");
        if let SignalValue::String(hex) = &fob3[0].1 {
            let expected = crypto::compute_challenge_response(&fob3_secret, &nonce);
            assert_eq!(hex_to_bytes(hex).as_slice(), &expected);
        }

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_unpaired_fob_responds_with_wrong_key() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));
        let fob5_secret = model.fobs[4].secret; // unpaired fob
        let fob1_secret = model.fobs[0].secret; // paired fob

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Place unpaired fob 5 at DriverDoor
        bus.inject(
            signals::KEYFOB_5_ZONE,
            SignalValue::String("DriverDoor".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        let nonce = [0x33u8; 16];
        bus.inject(
            signals::PEPS_LF_CHALLENGE,
            SignalValue::String(bytes_to_hex(&nonce)),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let fob5_resps: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_5_CHALLENGE_RESP)
            .collect();
        assert_eq!(
            fob5_resps.len(),
            1,
            "unpaired fob at proximity should still respond"
        );

        // But its response should NOT match any paired key
        if let SignalValue::String(hex) = &fob5_resps[0].1 {
            let unpaired_resp = hex_to_bytes(hex);
            let paired_resp = crypto::compute_challenge_response(&fob1_secret, &nonce);
            assert_ne!(
                unpaired_resp.as_slice(),
                &paired_resp,
                "unpaired fob response must differ from paired fob"
            );
            // But should match its own secret
            let expected = crypto::compute_challenge_response(&fob5_secret, &nonce);
            assert_eq!(unpaired_resp.as_slice(), &expected);
        }

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_ble_challenge_phone_in_proximity_responds() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));
        let phone1_secret = model.phones[0].secret;

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Place phone 1 at PassengerDoor
        bus.inject(
            signals::PHONE_1_ZONE,
            SignalValue::String("PassengerDoor".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        let nonce = [0xEEu8; 16];
        bus.inject(
            signals::PEPS_BLE_CHALLENGE,
            SignalValue::String(bytes_to_hex(&nonce)),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let phone1_resps: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::PHONE_1_CHALLENGE_RESP)
            .collect();
        assert_eq!(
            phone1_resps.len(),
            1,
            "phone in proximity should respond to BLE challenge"
        );

        if let SignalValue::String(hex) = &phone1_resps[0].1 {
            let expected = crypto::compute_challenge_response(&phone1_secret, &nonce);
            assert_eq!(hex_to_bytes(hex).as_slice(), &expected);
        }

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_ble_challenge_phone_in_approach_does_not_respond() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.inject(
            signals::PHONE_1_ZONE,
            SignalValue::String("Approach".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        let nonce = [0xDDu8; 16];
        bus.inject(
            signals::PEPS_BLE_CHALLENGE,
            SignalValue::String(bytes_to_hex(&nonce)),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let phone1_resps: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::PHONE_1_CHALLENGE_RESP)
            .collect();
        assert_eq!(
            phone1_resps.len(),
            0,
            "phone in approach should NOT respond to BLE challenge"
        );

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_nfc_challenge_card_at_reader_responds() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));
        let nfc1_secret = model.nfc_cards[0].secret;

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Place NFC card 1 at DriverHandle reader
        bus.inject(
            signals::NFC_1_POSITION,
            SignalValue::String("DriverHandle".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        let nonce = [0x99u8; 16];
        bus.inject(
            signals::PEPS_NFC_CHALLENGE,
            SignalValue::String(bytes_to_hex(&nonce)),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let nfc1_resps: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::NFC_1_CHALLENGE_RESP)
            .collect();
        assert_eq!(nfc1_resps.len(), 1, "NFC card at reader should respond");

        if let SignalValue::String(hex) = &nfc1_resps[0].1 {
            let expected = crypto::compute_challenge_response(&nfc1_secret, &nonce);
            assert_eq!(hex_to_bytes(hex).as_slice(), &expected);
        }

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_nfc_challenge_card_not_present_no_response() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // NFC card 1 defaults to NotPresent — don't move it
        bus.clear_history();

        let nonce = [0x11u8; 16];
        bus.inject(
            signals::PEPS_NFC_CHALLENGE,
            SignalValue::String(bytes_to_hex(&nonce)),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let nfc1_resps: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::NFC_1_CHALLENGE_RESP)
            .collect();
        assert_eq!(
            nfc1_resps.len(),
            0,
            "NFC card not at reader should NOT respond"
        );

        handle.abort();
    }

    // ── Phase 4: RF remote actions via event loop ──────────────────

    #[tokio::test]
    async fn run_loop_fob_button_lock_publishes_rf_message() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Place fob 1 in RF range
        bus.inject(
            signals::KEYFOB_1_ZONE,
            SignalValue::String("RfRange".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        // Press LOCK button
        bus.inject(signals::KEYFOB_1_BUTTON, SignalValue::String("LOCK".into()));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let rf_msgs: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_1_RF_MSG)
            .collect();
        assert_eq!(rf_msgs.len(), 1, "LOCK button should publish RF message");

        if let SignalValue::String(payload) = &rf_msgs[0].1 {
            assert!(
                payload.starts_with("LOCK:"),
                "payload should start with LOCK: {payload}"
            );
            // After "LOCK:" should be 32 hex chars (16 bytes encrypted)
            let hex_part = &payload[5..];
            assert_eq!(
                hex_part.len(),
                32,
                "encrypted rolling code should be 32 hex chars"
            );
        }

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_fob_button_all_actions() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.inject(
            signals::KEYFOB_1_ZONE,
            SignalValue::String("RfRange".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Press each button type
        for action in [
            "LOCK",
            "UNLOCK",
            "TRUNK_RELEASE",
            "REMOTE_START",
            "PANIC_ALARM",
        ] {
            bus.clear_history();
            bus.inject(signals::KEYFOB_1_BUTTON, SignalValue::String(action.into()));
            tokio::task::yield_now().await;
            tokio::task::yield_now().await;

            let history = bus.history();
            let rf_msgs: Vec<_> = history
                .iter()
                .filter(|(p, _)| *p == signals::KEYFOB_1_RF_MSG)
                .collect();
            assert_eq!(rf_msgs.len(), 1, "{action} should publish RF message");

            if let SignalValue::String(payload) = &rf_msgs[0].1 {
                assert!(
                    payload.starts_with(&format!("{action}:")),
                    "payload should start with {action}: got {payload}"
                );
            }
        }

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_fob_rolling_code_increments() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));
        let fob1_secret = model.fobs[0].secret;

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.inject(
            signals::KEYFOB_1_ZONE,
            SignalValue::String("RfRange".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Press LOCK three times — each should have a different encrypted payload
        let mut payloads = Vec::new();
        for _ in 0..3 {
            bus.clear_history();
            bus.inject(signals::KEYFOB_1_BUTTON, SignalValue::String("LOCK".into()));
            tokio::task::yield_now().await;
            tokio::task::yield_now().await;

            let history = bus.history();
            let rf: Vec<_> = history
                .iter()
                .filter(|(p, _)| *p == signals::KEYFOB_1_RF_MSG)
                .collect();
            assert_eq!(rf.len(), 1);
            if let SignalValue::String(payload) = &rf[0].1 {
                payloads.push(payload.clone());
            }
        }

        // All three payloads should differ (rolling code increments)
        assert_ne!(
            payloads[0], payloads[1],
            "rolling code should differ between presses"
        );
        assert_ne!(
            payloads[1], payloads[2],
            "rolling code should differ between presses"
        );
        assert_ne!(
            payloads[0], payloads[2],
            "rolling code should differ between presses"
        );

        // Verify each payload decrypts to the expected counter
        for (i, payload) in payloads.iter().enumerate() {
            let hex_part = &payload[5..]; // skip "LOCK:"
            let encrypted = hex_to_bytes(hex_part);
            let expected_encrypted = crypto::encrypt_rolling_code(&fob1_secret, (i + 1) as u32);
            assert_eq!(
                encrypted.as_slice(),
                &expected_encrypted,
                "press {} should encrypt counter {}",
                i + 1,
                i + 1
            );
        }

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_fob_button_out_of_range_no_rf() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Fob 1 defaults to OutOfRange — don't move it
        bus.clear_history();

        bus.inject(signals::KEYFOB_1_BUTTON, SignalValue::String("LOCK".into()));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let rf_msgs: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_1_RF_MSG)
            .collect();
        assert_eq!(
            rf_msgs.len(),
            0,
            "fob out of range should NOT publish RF message"
        );

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_fob_button_from_proximity_works() {
        // RF buttons work from any reachable zone, not just RfRange
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.inject(
            signals::KEYFOB_1_ZONE,
            SignalValue::String("DriverDoor".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();
        bus.inject(
            signals::KEYFOB_1_BUTTON,
            SignalValue::String("PANIC_ALARM".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let rf_msgs: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_1_RF_MSG)
            .collect();
        assert_eq!(
            rf_msgs.len(),
            1,
            "fob button should work from proximity zone too"
        );

        if let SignalValue::String(payload) = &rf_msgs[0].1 {
            assert!(payload.starts_with("PANIC_ALARM:"));
        }

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_different_fobs_have_independent_rolling_codes() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Place fob 1 and fob 2 in range
        bus.inject(
            signals::KEYFOB_1_ZONE,
            SignalValue::String("RfRange".into()),
        );
        bus.inject(
            signals::KEYFOB_2_ZONE,
            SignalValue::String("RfRange".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        // Press LOCK on both fobs
        bus.inject(signals::KEYFOB_1_BUTTON, SignalValue::String("LOCK".into()));
        bus.inject(
            signals::KEYFOB_2_BUTTON,
            SignalValue::String("UNLOCK".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();

        let fob1_rf: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_1_RF_MSG)
            .collect();
        let fob2_rf: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_2_RF_MSG)
            .collect();

        assert_eq!(fob1_rf.len(), 1, "fob 1 should publish");
        assert_eq!(fob2_rf.len(), 1, "fob 2 should publish");

        // Different fobs have different secrets → different encrypted payloads
        if let (SignalValue::String(p1), SignalValue::String(p2)) = (&fob1_rf[0].1, &fob2_rf[0].1) {
            // Strip action prefix, compare encrypted parts
            let enc1 = &p1[5..]; // "LOCK:"
            let enc2 = &p2[7..]; // "UNLOCK:"
            assert_ne!(
                enc1, enc2,
                "different fobs with different secrets should produce different ciphertext"
            );
        }

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_none_button_does_not_publish() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.inject(
            signals::KEYFOB_1_ZONE,
            SignalValue::String("RfRange".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        // "NONE" means button released / no action
        bus.inject(signals::KEYFOB_1_BUTTON, SignalValue::String("NONE".into()));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let rf_msgs: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_1_RF_MSG)
            .collect();
        assert_eq!(
            rf_msgs.len(),
            0,
            "NONE button should not produce RF message"
        );

        handle.abort();
    }

    // ── Phase 5: BLE phone event-loop integration ──────────────────

    #[tokio::test]
    async fn run_loop_phone_zone_change_publishes_rssi() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.inject(
            signals::PHONE_1_ZONE,
            SignalValue::String("PassengerDoor".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let rssi_msgs: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::PHONE_1_RSSI)
            .collect();
        assert_eq!(
            rssi_msgs.len(),
            1,
            "phone zone change via bus should auto-publish RSSI"
        );

        if let SignalValue::String(payload) = &rssi_msgs[0].1 {
            assert!(
                payload.contains("\"passenger\":-30"),
                "passenger antenna should be strongest at PassengerDoor: {payload}"
            );
        }

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_phone_zone_out_of_range_clears_rssi() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Move to LF zone first, then out
        bus.inject(signals::PHONE_1_ZONE, SignalValue::String("Cabin".into()));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();
        bus.inject(
            signals::PHONE_1_ZONE,
            SignalValue::String("OutOfRange".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let rssi_msgs: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::PHONE_1_RSSI)
            .collect();
        assert_eq!(rssi_msgs.len(), 1, "leaving LF range should clear RSSI");
        if let SignalValue::String(payload) = &rssi_msgs[0].1 {
            assert_eq!(payload, "{}");
        }

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_two_phones_independent_ble_challenge() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));
        let phone1_secret = model.phones[0].secret;
        let phone2_secret = model.phones[1].secret;

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Phone 1 at DriverDoor (proximity), Phone 2 at Approach (no challenge)
        bus.inject(
            signals::PHONE_1_ZONE,
            SignalValue::String("DriverDoor".into()),
        );
        bus.inject(
            signals::PHONE_2_ZONE,
            SignalValue::String("Approach".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        let nonce = [0xBBu8; 16];
        bus.inject(
            signals::PEPS_BLE_CHALLENGE,
            SignalValue::String(bytes_to_hex(&nonce)),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();

        // Phone 1 should respond (proximity)
        let p1: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::PHONE_1_CHALLENGE_RESP)
            .collect();
        assert_eq!(p1.len(), 1, "phone 1 at DriverDoor should respond");
        if let SignalValue::String(hex) = &p1[0].1 {
            let expected = crypto::compute_challenge_response(&phone1_secret, &nonce);
            assert_eq!(hex_to_bytes(hex).as_slice(), &expected);
        }

        // Phone 2 should NOT respond (approach)
        let p2: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::PHONE_2_CHALLENGE_RESP)
            .collect();
        assert_eq!(
            p2.len(),
            0,
            "phone 2 at Approach should not respond to challenge"
        );

        // Now move phone 2 to proximity and re-challenge
        bus.inject(signals::PHONE_2_ZONE, SignalValue::String("Hood".into()));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();
        let nonce2 = [0xCCu8; 16];
        bus.inject(
            signals::PEPS_BLE_CHALLENGE,
            SignalValue::String(bytes_to_hex(&nonce2)),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let p2_now: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::PHONE_2_CHALLENGE_RESP)
            .collect();
        assert_eq!(p2_now.len(), 1, "phone 2 now at Hood should respond");
        if let SignalValue::String(hex) = &p2_now[0].1 {
            let expected = crypto::compute_challenge_response(&phone2_secret, &nonce2);
            assert_eq!(hex_to_bytes(hex).as_slice(), &expected);
        }

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_phone_approach_poll_rssi() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.inject(
            signals::PHONE_1_ZONE,
            SignalValue::String("Approach".into()),
        );
        bus.inject(
            signals::PHONE_2_ZONE,
            SignalValue::String("OutOfRange".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();
        bus.inject(signals::PEPS_APPROACH_POLL, SignalValue::String("1".into()));
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        assert!(
            history.iter().any(|(p, _)| *p == signals::PHONE_1_RSSI),
            "phone 1 at Approach should respond to poll"
        );
        assert!(
            !history.iter().any(|(p, _)| *p == signals::PHONE_2_RSSI),
            "phone 2 OutOfRange should not respond to poll"
        );

        handle.abort();
    }

    // ── Phase 6: NFC card event-loop integration ───────────────────

    #[tokio::test]
    async fn run_loop_nfc_position_change_via_bus() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));
        let nfc1_secret = model.nfc_cards[0].secret;

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Place NFC card 1 at DriverHandle
        bus.inject(
            signals::NFC_1_POSITION,
            SignalValue::String("DriverHandle".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        // Send NFC challenge
        let nonce = [0xAAu8; 16];
        bus.inject(
            signals::PEPS_NFC_CHALLENGE,
            SignalValue::String(bytes_to_hex(&nonce)),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let resps: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::NFC_1_CHALLENGE_RESP)
            .collect();
        assert_eq!(resps.len(), 1, "NFC card at DriverHandle should respond");

        if let SignalValue::String(hex) = &resps[0].1 {
            let expected = crypto::compute_challenge_response(&nfc1_secret, &nonce);
            assert_eq!(hex_to_bytes(hex).as_slice(), &expected);
        }

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_nfc_at_push_button_responds() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));
        let nfc1_secret = model.nfc_cards[0].secret;

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.inject(
            signals::NFC_1_POSITION,
            SignalValue::String("PushButton".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        let nonce = [0x55u8; 16];
        bus.inject(
            signals::PEPS_NFC_CHALLENGE,
            SignalValue::String(bytes_to_hex(&nonce)),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let resps: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::NFC_1_CHALLENGE_RESP)
            .collect();
        assert_eq!(resps.len(), 1, "NFC card at PushButton should respond");

        if let SignalValue::String(hex) = &resps[0].1 {
            let expected = crypto::compute_challenge_response(&nfc1_secret, &nonce);
            assert_eq!(hex_to_bytes(hex).as_slice(), &expected);
        }

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_nfc_removed_stops_responding() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Place card, then remove it
        bus.inject(
            signals::NFC_1_POSITION,
            SignalValue::String("DriverHandle".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.inject(
            signals::NFC_1_POSITION,
            SignalValue::String("NotPresent".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        let nonce = [0x77u8; 16];
        bus.inject(
            signals::PEPS_NFC_CHALLENGE,
            SignalValue::String(bytes_to_hex(&nonce)),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let resps: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::NFC_1_CHALLENGE_RESP)
            .collect();
        assert_eq!(
            resps.len(),
            0,
            "NFC card removed from reader should not respond"
        );

        handle.abort();
    }

    // ── Phase 7: Unpaired fob verification ───────────────────────

    #[tokio::test]
    async fn run_loop_unpaired_fob6_also_responds_with_wrong_key() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));
        let fob6_secret = model.fobs[5].secret; // unpaired fob 6

        // Verify its secret differs from every paired fob
        for i in 0..4 {
            assert_ne!(
                fob6_secret,
                model.fobs[i].secret,
                "unpaired fob 6 secret must differ from paired fob {}",
                i + 1
            );
        }

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Place unpaired fob 6 at PassengerDoor
        bus.inject(
            signals::KEYFOB_6_ZONE,
            SignalValue::String("PassengerDoor".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        let nonce = [0xEEu8; 16];
        bus.inject(
            signals::PEPS_LF_CHALLENGE,
            SignalValue::String(bytes_to_hex(&nonce)),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let fob6_resps: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_6_CHALLENGE_RESP)
            .collect();
        assert_eq!(
            fob6_resps.len(),
            1,
            "unpaired fob 6 at proximity should still respond physically"
        );

        // Response must match fob 6's own secret, not any paired fob
        if let SignalValue::String(hex) = &fob6_resps[0].1 {
            let expected = crypto::compute_challenge_response(&fob6_secret, &nonce);
            assert_eq!(hex_to_bytes(hex).as_slice(), &expected);
        }

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_unpaired_fob_response_differs_from_all_paired() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));
        let paired_secrets: Vec<_> = model.fobs[..4].iter().map(|f| f.secret).collect();
        let fob5_secret = model.fobs[4].secret;

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Place unpaired fob 5 and all paired fobs at DriverDoor
        bus.inject(
            signals::KEYFOB_5_ZONE,
            SignalValue::String("DriverDoor".into()),
        );
        bus.inject(
            signals::KEYFOB_1_ZONE,
            SignalValue::String("DriverDoor".into()),
        );
        bus.inject(
            signals::KEYFOB_2_ZONE,
            SignalValue::String("DriverDoor".into()),
        );
        bus.inject(
            signals::KEYFOB_3_ZONE,
            SignalValue::String("DriverDoor".into()),
        );
        bus.inject(
            signals::KEYFOB_4_ZONE,
            SignalValue::String("DriverDoor".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        let nonce = [0xABu8; 16];
        bus.inject(
            signals::PEPS_LF_CHALLENGE,
            SignalValue::String(bytes_to_hex(&nonce)),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();

        // Unpaired fob 5's response should differ from every paired fob's response
        let fob5_resp: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_5_CHALLENGE_RESP)
            .collect();
        assert_eq!(fob5_resp.len(), 1);

        if let SignalValue::String(hex5) = &fob5_resp[0].1 {
            let resp5 = hex_to_bytes(hex5);
            let expected5 = crypto::compute_challenge_response(&fob5_secret, &nonce);
            assert_eq!(resp5.as_slice(), &expected5);

            // Check against all 4 paired fobs
            for (i, secret) in paired_secrets.iter().enumerate() {
                let paired_resp = crypto::compute_challenge_response(secret, &nonce);
                assert_ne!(
                    resp5.as_slice(),
                    &paired_resp,
                    "unpaired fob 5 response must differ from paired fob {}",
                    i + 1
                );
            }
        }

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_unpaired_fob_rssi_still_works() {
        // Physical presence is real — unpaired fobs still emit RSSI
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Place unpaired fob 5 in Approach zone
        bus.inject(
            signals::KEYFOB_5_ZONE,
            SignalValue::String("Approach".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Zone change should trigger RSSI publish
        let history = bus.history();
        let rssi: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_5_RSSI)
            .collect();
        assert_eq!(
            rssi.len(),
            1,
            "unpaired fob still emits RSSI — it physically exists"
        );

        // Also responds to approach poll
        bus.clear_history();
        bus.inject(
            signals::PEPS_APPROACH_POLL,
            SignalValue::String("POLL".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let poll_rssi: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::KEYFOB_5_RSSI)
            .collect();
        assert_eq!(
            poll_rssi.len(),
            1,
            "unpaired fob should respond to approach poll"
        );

        handle.abort();
    }

    #[tokio::test]
    async fn run_loop_unpaired_fob_cannot_press_buttons() {
        // Unpaired fobs have no button signals (only fobs 1-4 do),
        // and press_button() returns None for unpaired fobs anyway.
        // Verify at the device level that unpaired fobs reject button presses.
        let mut fob = KeyFob::new(5, false, [0xFFu8; 16]);
        fob.zone = Zone::RfRange;
        assert!(
            fob.press_button(FobButton::Lock).is_none(),
            "unpaired fob must not generate RF messages"
        );
        assert!(
            fob.press_button(FobButton::Unlock).is_none(),
            "unpaired fob must not generate RF messages"
        );
        assert!(
            fob.press_button(FobButton::TrunkRelease).is_none(),
            "unpaired fob must not generate RF messages"
        );
        assert!(
            fob.press_button(FobButton::PanicAlarm).is_none(),
            "unpaired fob must not generate RF messages"
        );
    }

    #[tokio::test]
    async fn run_loop_two_nfc_cards_independent() {
        let bus = Arc::new(MockBus::new());
        let model = PepsPlantModel::new(Arc::clone(&bus));
        let nfc1_secret = model.nfc_cards[0].secret;
        let nfc2_secret = model.nfc_cards[1].secret;

        let handle = tokio::spawn(model.run());
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        // Card 1 at DriverHandle, Card 2 at PushButton
        bus.inject(
            signals::NFC_1_POSITION,
            SignalValue::String("DriverHandle".into()),
        );
        bus.inject(
            signals::NFC_2_POSITION,
            SignalValue::String("PushButton".into()),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        bus.clear_history();

        let nonce = [0x12u8; 16];
        bus.inject(
            signals::PEPS_NFC_CHALLENGE,
            SignalValue::String(bytes_to_hex(&nonce)),
        );
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;

        let history = bus.history();

        // Both should respond
        let c1: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::NFC_1_CHALLENGE_RESP)
            .collect();
        let c2: Vec<_> = history
            .iter()
            .filter(|(p, _)| *p == signals::NFC_2_CHALLENGE_RESP)
            .collect();
        assert_eq!(c1.len(), 1, "NFC card 1 at DriverHandle should respond");
        assert_eq!(c2.len(), 1, "NFC card 2 at PushButton should respond");

        // Verify different secrets produce different responses
        if let (SignalValue::String(h1), SignalValue::String(h2)) = (&c1[0].1, &c2[0].1) {
            assert_ne!(
                h1, h2,
                "different NFC cards should produce different responses"
            );
            let exp1 = crypto::compute_challenge_response(&nfc1_secret, &nonce);
            let exp2 = crypto::compute_challenge_response(&nfc2_secret, &nonce);
            assert_eq!(hex_to_bytes(h1).as_slice(), &exp1);
            assert_eq!(hex_to_bytes(h2).as_slice(), &exp2);
        }

        handle.abort();
    }
}
