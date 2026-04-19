//! Zone model for PEPS device positioning.
//!
//! Devices (key fobs, BLE phones, NFC cards) exist in exactly one zone at a
//! time. The zone determines what communication protocol is used and what
//! kind of response the plant model generates:
//!
//! | Zone          | LF  | BLE | NFC | RF  | Auth type                    |
//! |---------------|-----|-----|-----|-----|------------------------------|
//! | DriverDoor    | yes | yes | yes | -   | Challenge-response (AES-128) |
//! | PassengerDoor | yes | yes | -   | -   | Challenge-response (AES-128) |
//! | Hood          | yes | yes | -   | -   | Challenge-response (AES-128) |
//! | Trunk         | yes | yes | -   | -   | Challenge-response (AES-128) |
//! | TrunkInside   | yes | yes | -   | -   | Challenge-response (AES-128) |
//! | Cabin         | yes | yes | -   | -   | Challenge-response (AES-128) |
//! | Approach      | yes | yes | -   | -   | RSSI only (no crypto)        |
//! | RfRange       | -   | -   | -   | yes | Rolling code (fob buttons)   |
//! | OutOfRange    | -   | -   | -   | -   | No communication             |
//!
//! **Cabin / TrunkInside exclusivity:** A device cannot be in both Cabin and
//! TrunkInside simultaneously — they represent physically distinct enclosed
//! spaces. Setting a device to TrunkInside clears Cabin and vice versa.
//! This enables features like "key left in trunk" detection.

use std::fmt;

/// Physical zone where a PEPS device is positioned.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Zone {
    /// ~1m from driver door handle — LF antenna present.
    DriverDoor,
    /// ~1m from passenger door handle — LF antenna present.
    PassengerDoor,
    /// ~1m from hood — LF antenna present.
    Hood,
    /// ~1m from trunk/liftgate — LF antenna present (standing behind car).
    Trunk,
    /// Physically inside the trunk/cargo area — LF antenna present.
    /// Exclusive with Cabin.
    TrunkInside,
    /// Inside cabin — LF antennas present. Exclusive with TrunkInside.
    Cabin,
    /// ~5m approach range — LF antennas active but only RSSI polling (no crypto challenge).
    Approach,
    /// ~100m RF range — fob button presses only (rolling code).
    RfRange,
    /// Beyond all communication range.
    OutOfRange,
}

impl Zone {
    /// Whether this zone has LF antenna coverage (proximity or approach).
    pub fn has_lf(self) -> bool {
        matches!(
            self,
            Zone::DriverDoor
                | Zone::PassengerDoor
                | Zone::Hood
                | Zone::Trunk
                | Zone::TrunkInside
                | Zone::Cabin
                | Zone::Approach
        )
    }

    /// Whether this zone supports full AES-128 challenge-response authentication.
    /// Only the ~1m proximity zones do; Approach only supports RSSI polling.
    pub fn supports_challenge_response(self) -> bool {
        matches!(
            self,
            Zone::DriverDoor
                | Zone::PassengerDoor
                | Zone::Hood
                | Zone::Trunk
                | Zone::TrunkInside
                | Zone::Cabin
        )
    }

    /// Whether this zone supports RSSI-based presence detection.
    /// All LF zones support RSSI; proximity zones also support challenge-response.
    pub fn supports_rssi(self) -> bool {
        self.has_lf()
    }

    /// Whether this zone supports RF remote commands (fob buttons).
    pub fn supports_rf_remote(self) -> bool {
        matches!(self, Zone::RfRange)
    }

    /// Whether a device in this zone can communicate at all.
    pub fn is_reachable(self) -> bool {
        !matches!(self, Zone::OutOfRange)
    }

    /// Parse a zone from a VSS string value.
    pub fn from_str_value(s: &str) -> Option<Self> {
        match s {
            "DriverDoor" => Some(Zone::DriverDoor),
            "PassengerDoor" => Some(Zone::PassengerDoor),
            "Hood" => Some(Zone::Hood),
            "Trunk" => Some(Zone::Trunk),
            "TrunkInside" => Some(Zone::TrunkInside),
            "Cabin" => Some(Zone::Cabin),
            "Approach" => Some(Zone::Approach),
            "RfRange" => Some(Zone::RfRange),
            "OutOfRange" => Some(Zone::OutOfRange),
            _ => None,
        }
    }

    /// Convert to a VSS string value.
    pub fn as_str(self) -> &'static str {
        match self {
            Zone::DriverDoor => "DriverDoor",
            Zone::PassengerDoor => "PassengerDoor",
            Zone::Hood => "Hood",
            Zone::Trunk => "Trunk",
            Zone::TrunkInside => "TrunkInside",
            Zone::Cabin => "Cabin",
            Zone::Approach => "Approach",
            Zone::RfRange => "RfRange",
            Zone::OutOfRange => "OutOfRange",
        }
    }
}

impl fmt::Display for Zone {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// NFC reader locations — NFC cards/phones can only be detected at specific
/// physical reader points, not in arbitrary zones.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum NfcPosition {
    /// NFC reader in driver door handle.
    DriverHandle,
    /// NFC reader near cabin push-button start.
    PushButton,
    /// Not near any NFC reader.
    NotPresent,
}

impl NfcPosition {
    pub fn from_str_value(s: &str) -> Option<Self> {
        match s {
            "DriverHandle" => Some(NfcPosition::DriverHandle),
            "PushButton" => Some(NfcPosition::PushButton),
            "NotPresent" => Some(NfcPosition::NotPresent),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            NfcPosition::DriverHandle => "DriverHandle",
            NfcPosition::PushButton => "PushButton",
            NfcPosition::NotPresent => "NotPresent",
        }
    }

    /// Whether this position allows NFC communication.
    pub fn is_present(self) -> bool {
        !matches!(self, NfcPosition::NotPresent)
    }
}

impl fmt::Display for NfcPosition {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn proximity_zones_support_challenge_response() {
        for zone in [
            Zone::DriverDoor,
            Zone::PassengerDoor,
            Zone::Hood,
            Zone::Trunk,
            Zone::TrunkInside,
            Zone::Cabin,
        ] {
            assert!(
                zone.supports_challenge_response(),
                "{zone} should support challenge-response"
            );
            assert!(zone.supports_rssi(), "{zone} should support RSSI");
            assert!(zone.has_lf(), "{zone} should have LF");
        }
    }

    #[test]
    fn approach_supports_rssi_but_not_challenge_response() {
        assert!(Zone::Approach.has_lf());
        assert!(Zone::Approach.supports_rssi());
        assert!(!Zone::Approach.supports_challenge_response());
    }

    #[test]
    fn rf_range_supports_only_rf_remote() {
        assert!(!Zone::RfRange.has_lf());
        assert!(!Zone::RfRange.supports_challenge_response());
        assert!(!Zone::RfRange.supports_rssi());
        assert!(Zone::RfRange.supports_rf_remote());
        assert!(Zone::RfRange.is_reachable());
    }

    #[test]
    fn out_of_range_supports_nothing() {
        assert!(!Zone::OutOfRange.has_lf());
        assert!(!Zone::OutOfRange.supports_challenge_response());
        assert!(!Zone::OutOfRange.supports_rssi());
        assert!(!Zone::OutOfRange.supports_rf_remote());
        assert!(!Zone::OutOfRange.is_reachable());
    }

    #[test]
    fn zone_string_roundtrip() {
        for zone in [
            Zone::DriverDoor,
            Zone::PassengerDoor,
            Zone::Hood,
            Zone::Trunk,
            Zone::TrunkInside,
            Zone::Cabin,
            Zone::Approach,
            Zone::RfRange,
            Zone::OutOfRange,
        ] {
            assert_eq!(Zone::from_str_value(zone.as_str()), Some(zone));
        }
    }

    #[test]
    fn nfc_position_string_roundtrip() {
        for pos in [
            NfcPosition::DriverHandle,
            NfcPosition::PushButton,
            NfcPosition::NotPresent,
        ] {
            assert_eq!(NfcPosition::from_str_value(pos.as_str()), Some(pos));
        }
    }

    #[test]
    fn nfc_presence() {
        assert!(NfcPosition::DriverHandle.is_present());
        assert!(NfcPosition::PushButton.is_present());
        assert!(!NfcPosition::NotPresent.is_present());
    }
}
