//! PEPS device types — key fobs, BLE phones, and NFC cards.
//!
//! Each device struct holds the plant-model state: current zone/position,
//! shared secret, pairing status, and (for fobs) rolling code counter.
//! The orchestrator (`PepsPlantModel`) owns all devices and drives their
//! state transitions based on incoming VSS signal changes.

use super::crypto::{
    aes_cmac_truncated, compute_challenge_response, Challenge, ChallengeResponse, RfMac,
    SharedSecret, RF_MAC_LEN,
};
use super::zone::{NfcPosition, Zone};

/// RF button actions available on a paired key fob.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FobButton {
    Lock,
    Unlock,
    TrunkRelease,
    RemoteStart,
    PanicAlarm,
}

impl FobButton {
    pub fn from_str_value(s: &str) -> Option<Self> {
        match s {
            "LOCK" => Some(FobButton::Lock),
            "UNLOCK" => Some(FobButton::Unlock),
            "TRUNK_RELEASE" => Some(FobButton::TrunkRelease),
            "REMOTE_START" => Some(FobButton::RemoteStart),
            "PANIC_ALARM" => Some(FobButton::PanicAlarm),
            "NONE" => None,
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            FobButton::Lock => "LOCK",
            FobButton::Unlock => "UNLOCK",
            FobButton::TrunkRelease => "TRUNK_RELEASE",
            FobButton::RemoteStart => "REMOTE_START",
            FobButton::PanicAlarm => "PANIC_ALARM",
        }
    }

    /// Single-byte action code used in the RF message payload covered by the MAC.
    pub fn to_action_byte(self) -> u8 {
        match self {
            FobButton::Lock => 0x01,
            FobButton::Unlock => 0x02,
            FobButton::TrunkRelease => 0x03,
            FobButton::RemoteStart => 0x04,
            FobButton::PanicAlarm => 0x05,
        }
    }

    /// Parse from the action byte in an RF message.
    pub fn from_action_byte(b: u8) -> Option<Self> {
        match b {
            0x01 => Some(FobButton::Lock),
            0x02 => Some(FobButton::Unlock),
            0x03 => Some(FobButton::TrunkRelease),
            0x04 => Some(FobButton::RemoteStart),
            0x05 => Some(FobButton::PanicAlarm),
            _ => None,
        }
    }
}

/// RF message produced by a key-fob button press.
///
/// Wire layout (16 bytes, transmitted over 433/868 MHz RF):
/// ```text
/// ┌──────────────┬────────┬──────────────┬─────────────────┐
/// │  fob_id (4B) │act (1B)│  rolling (4B)│  MAC (7B CMAC)  │
/// └──────────────┴────────┴──────────────┴─────────────────┘
/// ```
/// The AES-CMAC is computed over `fob_id ‖ action_byte ‖ rolling_code`
/// (9 bytes) using the fob's shared secret.
///
/// The rolling code is transmitted **unencrypted** — confidentiality is not
/// the goal; the MAC provides authenticity and the rolling code prevents replay.
#[derive(Debug, Clone)]
pub struct RfMessage {
    /// Fob-unique 32-bit identifier (derived from pairing; 1-based index in sim).
    pub fob_id: u32,
    /// The button the user pressed.
    pub action: FobButton,
    /// Unencrypted rolling code counter (vehicle-side validates forward-only window).
    pub rolling_code: u32,
    /// Truncated AES-CMAC (7 bytes) over `fob_id ‖ action_byte ‖ rolling_code`.
    pub mac: RfMac,
}

impl RfMessage {
    /// Serialise to the 16-byte on-air format (little-endian integers).
    pub fn to_wire(&self) -> [u8; 16] {
        let mut buf = [0u8; 16];
        buf[0..4].copy_from_slice(&self.fob_id.to_le_bytes());
        buf[4] = self.action.to_action_byte();
        buf[5..9].copy_from_slice(&self.rolling_code.to_le_bytes());
        buf[9..16].copy_from_slice(&self.mac);
        buf
    }

    /// Deserialise from the 16-byte on-air format.
    /// Returns `None` if the action byte is unrecognised.
    pub fn from_wire(buf: &[u8; 16]) -> Option<Self> {
        let fob_id = u32::from_le_bytes(buf[0..4].try_into().unwrap());
        let action = FobButton::from_action_byte(buf[4])?;
        let rolling_code = u32::from_le_bytes(buf[5..9].try_into().unwrap());
        let mut mac = [0u8; RF_MAC_LEN];
        mac.copy_from_slice(&buf[9..16]);
        Some(RfMessage { fob_id, action, rolling_code, mac })
    }

    /// Encode as a 32-character lowercase hex string for VSS signal transport.
    pub fn to_hex(&self) -> String {
        self.to_wire().iter().map(|b| format!("{b:02x}")).collect()
    }

    /// Decode from a 32-character hex string.
    pub fn from_hex(s: &str) -> Option<Self> {
        if s.len() != 32 {
            return None;
        }
        let mut buf = [0u8; 16];
        for (i, chunk) in s.as_bytes().chunks(2).enumerate() {
            let hi = hex_nibble(chunk[0])?;
            let lo = hex_nibble(chunk[1])?;
            buf[i] = (hi << 4) | lo;
        }
        Self::from_wire(&buf)
    }
}

fn hex_nibble(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    }
}

/// Simulated RSSI readings from LF antennas.
/// In approach zone, the plant model returns simulated RSSI values per antenna.
#[derive(Debug, Clone)]
pub struct RssiResponse {
    pub driver_door_dbm: i8,
    pub passenger_door_dbm: i8,
    pub hood_dbm: i8,
    pub trunk_dbm: i8,
    pub cabin_dbm: i8,
}

impl RssiResponse {
    /// Generate simulated RSSI based on device zone.
    /// Closer zones produce stronger (less negative) RSSI.
    pub fn for_zone(zone: Zone) -> Self {
        match zone {
            Zone::DriverDoor => RssiResponse {
                driver_door_dbm: -30,
                passenger_door_dbm: -65,
                hood_dbm: -60,
                trunk_dbm: -70,
                cabin_dbm: -55,
            },
            Zone::PassengerDoor => RssiResponse {
                driver_door_dbm: -65,
                passenger_door_dbm: -30,
                hood_dbm: -60,
                trunk_dbm: -70,
                cabin_dbm: -55,
            },
            Zone::Hood => RssiResponse {
                driver_door_dbm: -60,
                passenger_door_dbm: -60,
                hood_dbm: -30,
                trunk_dbm: -75,
                cabin_dbm: -55,
            },
            Zone::Trunk => RssiResponse {
                driver_door_dbm: -70,
                passenger_door_dbm: -70,
                hood_dbm: -75,
                trunk_dbm: -30,
                cabin_dbm: -60,
            },
            Zone::TrunkInside => RssiResponse {
                driver_door_dbm: -75,
                passenger_door_dbm: -75,
                hood_dbm: -80,
                trunk_dbm: -25,
                cabin_dbm: -55,
            },
            Zone::Cabin => RssiResponse {
                driver_door_dbm: -50,
                passenger_door_dbm: -50,
                hood_dbm: -55,
                trunk_dbm: -60,
                cabin_dbm: -25,
            },
            Zone::Approach => RssiResponse {
                driver_door_dbm: -75,
                passenger_door_dbm: -75,
                hood_dbm: -75,
                trunk_dbm: -75,
                cabin_dbm: -80,
            },
            // No LF coverage — shouldn't be called, but return floor values.
            Zone::RfRange | Zone::OutOfRange => RssiResponse {
                driver_door_dbm: -100,
                passenger_door_dbm: -100,
                hood_dbm: -100,
                trunk_dbm: -100,
                cabin_dbm: -100,
            },
        }
    }

    /// Encode as a JSON-ish string for VSS transport.
    pub fn to_signal_string(&self) -> String {
        format!(
            "{{\"driver\":{},\"passenger\":{},\"hood\":{},\"trunk\":{},\"cabin\":{}}}",
            self.driver_door_dbm,
            self.passenger_door_dbm,
            self.hood_dbm,
            self.trunk_dbm,
            self.cabin_dbm
        )
    }
}

// ---------------------------------------------------------------------------
// Key Fob
// ---------------------------------------------------------------------------

/// A simulated key fob (paired or unpaired).
pub struct KeyFob {
    /// 1-based device index (1..=6).
    pub index: u8,
    /// 32-bit fob identifier broadcast in every RF message.
    /// In simulation this is `index as u32`; in real hardware it is burned in
    /// during manufacture and exchanged during pairing.
    pub fob_id: u32,
    /// Current physical zone.
    pub zone: Zone,
    /// Whether this fob is paired with the vehicle.
    pub paired: bool,
    /// 128-bit shared secret (only meaningful if paired).
    pub secret: SharedSecret,
    /// Rolling code counter for RF transmissions. Incremented on each button press.
    pub rolling_counter: u32,
}

impl KeyFob {
    /// Create a new key fob.
    pub fn new(index: u8, paired: bool, secret: SharedSecret) -> Self {
        Self {
            index,
            fob_id: index as u32,
            zone: Zone::OutOfRange,
            paired,
            secret,
            rolling_counter: 0,
        }
    }

    /// Respond to an LF challenge (AES-128 encrypted nonce).
    /// Returns `None` if the fob is not in a zone that supports challenge-response.
    pub fn respond_to_challenge(&self, nonce: &Challenge) -> Option<ChallengeResponse> {
        if !self.zone.supports_challenge_response() {
            return None;
        }
        Some(compute_challenge_response(&self.secret, nonce))
    }

    /// Generate RSSI response for approach/presence polling.
    /// Returns `None` if not in an LF-capable zone.
    pub fn rssi_response(&self) -> Option<RssiResponse> {
        if !self.zone.supports_rssi() {
            return None;
        }
        Some(RssiResponse::for_zone(self.zone))
    }

    /// Handle an RF button press.
    ///
    /// Increments the rolling code counter, computes an AES-CMAC over
    /// `fob_id ‖ action_byte ‖ rolling_code`, and returns the RF message.
    /// Only paired fobs in a reachable zone can press buttons.
    pub fn press_button(&mut self, button: FobButton) -> Option<RfMessage> {
        if !self.paired || !self.zone.is_reachable() {
            return None;
        }
        self.rolling_counter += 1;

        // Build the 9-byte payload that the MAC covers.
        let mac_payload = build_mac_payload(self.fob_id, button, self.rolling_counter);
        let mac = aes_cmac_truncated(&self.secret, &mac_payload);

        Some(RfMessage {
            fob_id: self.fob_id,
            action: button,
            rolling_code: self.rolling_counter,
            mac,
        })
    }
}

/// Build the 9-byte buffer that the AES-CMAC covers:
/// `fob_id (4 LE) ‖ action_byte (1) ‖ rolling_code (4 LE)`
pub fn build_mac_payload(fob_id: u32, action: FobButton, rolling_code: u32) -> [u8; 9] {
    let mut buf = [0u8; 9];
    buf[0..4].copy_from_slice(&fob_id.to_le_bytes());
    buf[4] = action.to_action_byte();
    buf[5..9].copy_from_slice(&rolling_code.to_le_bytes());
    buf
}

// ---------------------------------------------------------------------------
// BLE Phone
// ---------------------------------------------------------------------------

/// A simulated BLE-paired smartphone.
pub struct BlePhone {
    /// 1-based device index (1..=2).
    pub index: u8,
    /// Current physical zone.
    pub zone: Zone,
    /// 128-bit shared secret established during BLE pairing.
    pub secret: SharedSecret,
}

impl BlePhone {
    pub fn new(index: u8, secret: SharedSecret) -> Self {
        Self {
            index,
            zone: Zone::OutOfRange,
            secret,
        }
    }

    /// Respond to a BLE challenge (AES-128 encrypted nonce).
    pub fn respond_to_challenge(&self, nonce: &Challenge) -> Option<ChallengeResponse> {
        if !self.zone.supports_challenge_response() {
            return None;
        }
        Some(compute_challenge_response(&self.secret, nonce))
    }

    /// Generate BLE RSSI response for approach polling.
    pub fn rssi_response(&self) -> Option<RssiResponse> {
        if !self.zone.supports_rssi() {
            return None;
        }
        Some(RssiResponse::for_zone(self.zone))
    }
}

// ---------------------------------------------------------------------------
// NFC Card
// ---------------------------------------------------------------------------

/// A simulated NFC key card.
pub struct NfcCard {
    /// 1-based device index (1..=2).
    pub index: u8,
    /// Current position relative to NFC readers.
    pub position: NfcPosition,
    /// 128-bit shared secret stored on the card.
    pub secret: SharedSecret,
}

impl NfcCard {
    pub fn new(index: u8, secret: SharedSecret) -> Self {
        Self {
            index,
            position: NfcPosition::NotPresent,
            secret,
        }
    }

    /// Respond to an NFC challenge. Only works when positioned at a reader.
    pub fn respond_to_challenge(&self, nonce: &Challenge) -> Option<ChallengeResponse> {
        if !self.position.is_present() {
            return None;
        }
        Some(compute_challenge_response(&self.secret, nonce))
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn test_secret(seed: u8) -> SharedSecret {
        [seed; 16]
    }

    // ── KeyFob tests ───────────────────────────────────────────────

    #[test]
    fn fob_challenge_response_in_proximity_zone() {
        let fob = KeyFob::new(1, true, test_secret(0xAA));
        let mut fob = fob;
        fob.zone = Zone::DriverDoor;

        let nonce: Challenge = [0x55; 16];
        let resp = fob.respond_to_challenge(&nonce);
        assert!(
            resp.is_some(),
            "fob at DriverDoor should respond to challenge"
        );

        // Verify it's a proper AES-128 response
        let expected = compute_challenge_response(&test_secret(0xAA), &nonce);
        assert_eq!(resp.unwrap(), expected);
    }

    #[test]
    fn fob_no_challenge_response_in_approach() {
        let mut fob = KeyFob::new(1, true, test_secret(0xAA));
        fob.zone = Zone::Approach;
        assert!(
            fob.respond_to_challenge(&[0; 16]).is_none(),
            "approach zone should not support challenge-response"
        );
    }

    #[test]
    fn fob_rssi_in_approach() {
        let mut fob = KeyFob::new(1, true, test_secret(0xAA));
        fob.zone = Zone::Approach;
        let rssi = fob.rssi_response();
        assert!(rssi.is_some(), "approach zone should support RSSI");
    }

    #[test]
    fn fob_rssi_in_proximity() {
        let mut fob = KeyFob::new(1, true, test_secret(0xAA));
        fob.zone = Zone::DriverDoor;
        let rssi = fob.rssi_response();
        assert!(rssi.is_some(), "proximity zone should also support RSSI");
    }

    #[test]
    fn fob_no_rssi_out_of_range() {
        let mut fob = KeyFob::new(1, true, test_secret(0xAA));
        fob.zone = Zone::OutOfRange;
        assert!(fob.rssi_response().is_none());
    }

    #[test]
    fn fob_button_press_increments_rolling_code() {
        let mut fob = KeyFob::new(1, true, test_secret(0xBB));
        fob.zone = Zone::RfRange;

        let msg1 = fob.press_button(FobButton::Lock).unwrap();
        assert_eq!(msg1.rolling_code, 1);
        assert_eq!(msg1.action, FobButton::Lock);
        assert_eq!(msg1.fob_id, 1);

        let msg2 = fob.press_button(FobButton::Unlock).unwrap();
        assert_eq!(msg2.rolling_code, 2);
        assert_eq!(msg2.action, FobButton::Unlock);

        // Different counter → different MAC
        assert_ne!(msg1.mac, msg2.mac);
    }

    #[test]
    fn rf_message_wire_roundtrip() {
        let mut fob = KeyFob::new(3, true, test_secret(0xCC));
        fob.zone = Zone::RfRange;
        let msg = fob.press_button(FobButton::TrunkRelease).unwrap();

        let hex = msg.to_hex();
        assert_eq!(hex.len(), 32, "hex-encoded RF message must be 32 chars");

        let decoded = RfMessage::from_hex(&hex).expect("should decode back");
        assert_eq!(decoded.fob_id, msg.fob_id);
        assert_eq!(decoded.action, msg.action);
        assert_eq!(decoded.rolling_code, msg.rolling_code);
        assert_eq!(decoded.mac, msg.mac);
    }

    #[test]
    fn rf_message_mac_covers_all_fields() {
        let secret = test_secret(0xAA);
        let mut fob = KeyFob::new(2, true, secret);
        fob.zone = Zone::RfRange;

        let msg = fob.press_button(FobButton::Lock).unwrap();

        // Verify MAC independently using the public helper
        let payload = build_mac_payload(msg.fob_id, msg.action, msg.rolling_code);
        let expected_mac = super::super::crypto::aes_cmac_truncated(&secret, &payload);
        assert_eq!(msg.mac, expected_mac, "MAC must match independent computation");
    }

    #[test]
    fn fob_button_works_from_any_reachable_zone() {
        let mut fob = KeyFob::new(1, true, test_secret(0xCC));

        // RF buttons should work from any reachable zone (fob always transmits RF)
        for zone in [Zone::DriverDoor, Zone::Approach, Zone::RfRange, Zone::Cabin] {
            fob.zone = zone;
            assert!(
                fob.press_button(FobButton::Lock).is_some(),
                "button should work in {zone}"
            );
        }

        // But not out of range
        fob.zone = Zone::OutOfRange;
        assert!(fob.press_button(FobButton::Lock).is_none());
    }

    #[test]
    fn unpaired_fob_cannot_press_buttons() {
        let mut fob = KeyFob::new(5, false, test_secret(0x00));
        fob.zone = Zone::RfRange;
        assert!(
            fob.press_button(FobButton::Lock).is_none(),
            "unpaired fob should not produce RF messages"
        );
    }

    #[test]
    fn unpaired_fob_still_responds_to_challenge() {
        // Unpaired fobs respond with their (wrong) secret — the vehicle-side
        // feature logic rejects the response. The plant model just simulates
        // the physical device behavior.
        let mut fob = KeyFob::new(5, false, test_secret(0xFF));
        fob.zone = Zone::DriverDoor;
        assert!(fob.respond_to_challenge(&[0x42; 16]).is_some());
    }

    // ── BlePhone tests ─────────────────────────────────────────────

    #[test]
    fn phone_challenge_response_in_proximity() {
        let mut phone = BlePhone::new(1, test_secret(0xDD));
        phone.zone = Zone::DriverDoor;

        let nonce: Challenge = [0x11; 16];
        let resp = phone.respond_to_challenge(&nonce);
        assert!(resp.is_some());

        let expected = compute_challenge_response(&test_secret(0xDD), &nonce);
        assert_eq!(resp.unwrap(), expected);
    }

    #[test]
    fn phone_rssi_in_approach() {
        let mut phone = BlePhone::new(1, test_secret(0xDD));
        phone.zone = Zone::Approach;
        assert!(phone.rssi_response().is_some());
        assert!(phone.respond_to_challenge(&[0; 16]).is_none());
    }

    // ── NfcCard tests ──────────────────────────────────────────────

    #[test]
    fn nfc_responds_at_reader() {
        let mut card = NfcCard::new(1, test_secret(0xEE));
        card.position = NfcPosition::DriverHandle;

        let nonce: Challenge = [0x22; 16];
        let resp = card.respond_to_challenge(&nonce);
        assert!(resp.is_some());
    }

    #[test]
    fn nfc_no_response_when_not_present() {
        let card = NfcCard::new(1, test_secret(0xEE));
        assert!(card.respond_to_challenge(&[0; 16]).is_none());
    }

    // ── FobButton tests ────────────────────────────────────────────

    #[test]
    fn fob_button_string_roundtrip() {
        for btn in [
            FobButton::Lock,
            FobButton::Unlock,
            FobButton::TrunkRelease,
            FobButton::RemoteStart,
            FobButton::PanicAlarm,
        ] {
            let s = btn.as_str();
            assert_eq!(FobButton::from_str_value(s), Some(btn), "roundtrip for {s}");
        }
    }

    #[test]
    fn fob_button_none_value() {
        assert_eq!(FobButton::from_str_value("NONE"), None);
    }

    // ── RssiResponse tests ─────────────────────────────────────────

    #[test]
    fn rssi_driver_door_strongest_at_driver() {
        let rssi = RssiResponse::for_zone(Zone::DriverDoor);
        assert!(
            rssi.driver_door_dbm > rssi.passenger_door_dbm,
            "driver antenna should be strongest at driver door"
        );
        assert!(
            rssi.driver_door_dbm > rssi.trunk_dbm,
            "driver antenna should be stronger than trunk at driver door"
        );
    }

    #[test]
    fn rssi_approach_weaker_than_proximity() {
        let approach = RssiResponse::for_zone(Zone::Approach);
        let proximity = RssiResponse::for_zone(Zone::DriverDoor);
        assert!(
            approach.driver_door_dbm < proximity.driver_door_dbm,
            "approach RSSI should be weaker than proximity"
        );
    }

    #[test]
    fn rssi_to_signal_string_is_valid_json() {
        let rssi = RssiResponse::for_zone(Zone::Cabin);
        let s = rssi.to_signal_string();
        // Should parse as JSON
        assert!(s.starts_with('{') && s.ends_with('}'));
        assert!(s.contains("\"driver\":"));
        assert!(s.contains("\"trunk\":"));
    }
}
