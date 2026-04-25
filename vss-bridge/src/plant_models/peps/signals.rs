//! VSS signal path constants for the PEPS plant model.
//!
//! These signals are the interface between the plant model (simulating
//! physical key fobs, phones, NFC cards) and the vehicle-side feature
//! logic (PEPS, RKE). The HMI and test harness set device positions
//! and trigger button presses; the plant model publishes responses.

use crate::signal_bus::VssPath;

// ── Key Fob Zone (input: set by HMI / test) ───────────────────────────
pub const KEYFOB_1_ZONE: VssPath = "Body.PEPS.Plant.KeyFob.1.Zone";
pub const KEYFOB_2_ZONE: VssPath = "Body.PEPS.Plant.KeyFob.2.Zone";
pub const KEYFOB_3_ZONE: VssPath = "Body.PEPS.Plant.KeyFob.3.Zone";
pub const KEYFOB_4_ZONE: VssPath = "Body.PEPS.Plant.KeyFob.4.Zone";
pub const KEYFOB_5_ZONE: VssPath = "Body.PEPS.Plant.KeyFob.5.Zone";
pub const KEYFOB_6_ZONE: VssPath = "Body.PEPS.Plant.KeyFob.6.Zone";

/// All key fob zone signals, indexed 0..6.
pub const KEYFOB_ZONES: [VssPath; 6] = [
    KEYFOB_1_ZONE,
    KEYFOB_2_ZONE,
    KEYFOB_3_ZONE,
    KEYFOB_4_ZONE,
    KEYFOB_5_ZONE,
    KEYFOB_6_ZONE,
];

// ── Key Fob Button Press (input: set by HMI / test) ───────────────────
pub const KEYFOB_1_BUTTON: VssPath = "Body.PEPS.Plant.KeyFob.1.ButtonPress";
pub const KEYFOB_2_BUTTON: VssPath = "Body.PEPS.Plant.KeyFob.2.ButtonPress";
pub const KEYFOB_3_BUTTON: VssPath = "Body.PEPS.Plant.KeyFob.3.ButtonPress";
pub const KEYFOB_4_BUTTON: VssPath = "Body.PEPS.Plant.KeyFob.4.ButtonPress";

/// Button press signals for paired fobs only (1..4). Unpaired fobs (5-6) have no buttons.
pub const KEYFOB_BUTTONS: [VssPath; 4] = [
    KEYFOB_1_BUTTON,
    KEYFOB_2_BUTTON,
    KEYFOB_3_BUTTON,
    KEYFOB_4_BUTTON,
];

// ── Key Fob Pairing (config: set at init) ──────────────────────────────
pub const KEYFOB_1_PAIRED: VssPath = "Body.PEPS.Plant.KeyFob.1.Paired";
pub const KEYFOB_2_PAIRED: VssPath = "Body.PEPS.Plant.KeyFob.2.Paired";
pub const KEYFOB_3_PAIRED: VssPath = "Body.PEPS.Plant.KeyFob.3.Paired";
pub const KEYFOB_4_PAIRED: VssPath = "Body.PEPS.Plant.KeyFob.4.Paired";
pub const KEYFOB_5_PAIRED: VssPath = "Body.PEPS.Plant.KeyFob.5.Paired";
pub const KEYFOB_6_PAIRED: VssPath = "Body.PEPS.Plant.KeyFob.6.Paired";

pub const KEYFOB_PAIRED: [VssPath; 6] = [
    KEYFOB_1_PAIRED,
    KEYFOB_2_PAIRED,
    KEYFOB_3_PAIRED,
    KEYFOB_4_PAIRED,
    KEYFOB_5_PAIRED,
    KEYFOB_6_PAIRED,
];

// ── Key Fob Challenge Response (output: published by plant model) ──────
pub const KEYFOB_1_CHALLENGE_RESP: VssPath = "Body.PEPS.Plant.KeyFob.1.ChallengeResponse";
pub const KEYFOB_2_CHALLENGE_RESP: VssPath = "Body.PEPS.Plant.KeyFob.2.ChallengeResponse";
pub const KEYFOB_3_CHALLENGE_RESP: VssPath = "Body.PEPS.Plant.KeyFob.3.ChallengeResponse";
pub const KEYFOB_4_CHALLENGE_RESP: VssPath = "Body.PEPS.Plant.KeyFob.4.ChallengeResponse";
pub const KEYFOB_5_CHALLENGE_RESP: VssPath = "Body.PEPS.Plant.KeyFob.5.ChallengeResponse";
pub const KEYFOB_6_CHALLENGE_RESP: VssPath = "Body.PEPS.Plant.KeyFob.6.ChallengeResponse";

pub const KEYFOB_CHALLENGE_RESPS: [VssPath; 6] = [
    KEYFOB_1_CHALLENGE_RESP,
    KEYFOB_2_CHALLENGE_RESP,
    KEYFOB_3_CHALLENGE_RESP,
    KEYFOB_4_CHALLENGE_RESP,
    KEYFOB_5_CHALLENGE_RESP,
    KEYFOB_6_CHALLENGE_RESP,
];

// ── Key Fob RSSI Response (output: published by plant model) ───────────
pub const KEYFOB_1_RSSI: VssPath = "Body.PEPS.Plant.KeyFob.1.RssiResponse";
pub const KEYFOB_2_RSSI: VssPath = "Body.PEPS.Plant.KeyFob.2.RssiResponse";
pub const KEYFOB_3_RSSI: VssPath = "Body.PEPS.Plant.KeyFob.3.RssiResponse";
pub const KEYFOB_4_RSSI: VssPath = "Body.PEPS.Plant.KeyFob.4.RssiResponse";
pub const KEYFOB_5_RSSI: VssPath = "Body.PEPS.Plant.KeyFob.5.RssiResponse";
pub const KEYFOB_6_RSSI: VssPath = "Body.PEPS.Plant.KeyFob.6.RssiResponse";

pub const KEYFOB_RSSIS: [VssPath; 6] = [
    KEYFOB_1_RSSI,
    KEYFOB_2_RSSI,
    KEYFOB_3_RSSI,
    KEYFOB_4_RSSI,
    KEYFOB_5_RSSI,
    KEYFOB_6_RSSI,
];

// ── Key Fob RF Message (output: rolling code + action, published by plant) ──
pub const KEYFOB_1_RF_MSG: VssPath = "Body.PEPS.Plant.KeyFob.1.RfMessage";
pub const KEYFOB_2_RF_MSG: VssPath = "Body.PEPS.Plant.KeyFob.2.RfMessage";
pub const KEYFOB_3_RF_MSG: VssPath = "Body.PEPS.Plant.KeyFob.3.RfMessage";
pub const KEYFOB_4_RF_MSG: VssPath = "Body.PEPS.Plant.KeyFob.4.RfMessage";

pub const KEYFOB_RF_MSGS: [VssPath; 4] = [
    KEYFOB_1_RF_MSG,
    KEYFOB_2_RF_MSG,
    KEYFOB_3_RF_MSG,
    KEYFOB_4_RF_MSG,
];

// ── BLE Phone Zone (input: set by HMI / test) ─────────────────────────
pub const PHONE_1_ZONE: VssPath = "Body.PEPS.Plant.BlePhone.1.Zone";
pub const PHONE_2_ZONE: VssPath = "Body.PEPS.Plant.BlePhone.2.Zone";

pub const PHONE_ZONES: [VssPath; 2] = [PHONE_1_ZONE, PHONE_2_ZONE];

// ── BLE Phone Challenge Response (output: published by plant model) ────
pub const PHONE_1_CHALLENGE_RESP: VssPath = "Body.PEPS.Plant.BlePhone.1.ChallengeResponse";
pub const PHONE_2_CHALLENGE_RESP: VssPath = "Body.PEPS.Plant.BlePhone.2.ChallengeResponse";

pub const PHONE_CHALLENGE_RESPS: [VssPath; 2] = [PHONE_1_CHALLENGE_RESP, PHONE_2_CHALLENGE_RESP];

// ── BLE Phone RSSI Response (output: published by plant model) ─────────
pub const PHONE_1_RSSI: VssPath = "Body.PEPS.Plant.BlePhone.1.RssiResponse";
pub const PHONE_2_RSSI: VssPath = "Body.PEPS.Plant.BlePhone.2.RssiResponse";

pub const PHONE_RSSIS: [VssPath; 2] = [PHONE_1_RSSI, PHONE_2_RSSI];

// ── NFC Card Position (input: set by HMI / test) ──────────────────────
pub const NFC_1_POSITION: VssPath = "Body.PEPS.Plant.NfcCard.1.Position";
pub const NFC_2_POSITION: VssPath = "Body.PEPS.Plant.NfcCard.2.Position";

pub const NFC_POSITIONS: [VssPath; 2] = [NFC_1_POSITION, NFC_2_POSITION];

// ── NFC Card Challenge Response (output: published by plant model) ─────
pub const NFC_1_CHALLENGE_RESP: VssPath = "Body.PEPS.Plant.NfcCard.1.ChallengeResponse";
pub const NFC_2_CHALLENGE_RESP: VssPath = "Body.PEPS.Plant.NfcCard.2.ChallengeResponse";

pub const NFC_CHALLENGE_RESPS: [VssPath; 2] = [NFC_1_CHALLENGE_RESP, NFC_2_CHALLENGE_RESP];

// ── Vehicle-side challenge signals (input: feature logic sends challenges) ──
pub const PEPS_LF_CHALLENGE: VssPath = "Body.PEPS.LfChallenge";
pub const PEPS_BLE_CHALLENGE: VssPath = "Body.PEPS.BleChallenge";
pub const PEPS_NFC_CHALLENGE: VssPath = "Body.PEPS.NfcChallenge";
pub const PEPS_APPROACH_POLL: VssPath = "Body.PEPS.ApproachPoll";
