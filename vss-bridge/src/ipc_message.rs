//! IPC wire format for RPmsg communication between A53 (Rust) and M7 (AUTOSAR).
//!
//! All messages share a 16-byte header followed by a type-specific payload.
//! CRC-16/CCITT-FALSE is computed over all bytes excluding the final 4 (CRC + pad).

use crc::{Crc, CRC_16_IBM_3740};
use serde::{Deserialize, Serialize};

/// CRC-16/CCITT-FALSE (IBM 3740), initial value 0xFFFF.
const CRC_ALGO: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_3740);

/// Magic number for all IPC messages.
pub const IPC_MAGIC: u32 = 0xBCC0_1A00;

/// Current schema version.
pub const IPC_VERSION: u8 = 1;

// ---------------------------------------------------------------------------
// Signal value types
// ---------------------------------------------------------------------------

/// Tagged union for VSS signal values.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum SignalValue {
    Bool(bool),
    Uint8(u8),
    Int16(i16),
    Uint16(u16),
    Float(f32),
}

/// Type tag byte for the value union on the wire.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SigType {
    Bool   = 0,
    Uint8  = 1,
    Int16  = 2,
    Uint16 = 3,
    Float  = 4,
}

impl SigType {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Bool),
            1 => Some(Self::Uint8),
            2 => Some(Self::Int16),
            3 => Some(Self::Uint16),
            4 => Some(Self::Float),
            _ => None,
        }
    }
}

impl SignalValue {
    /// Returns the wire type tag for this value.
    pub fn sig_type(&self) -> SigType {
        match self {
            Self::Bool(_)   => SigType::Bool,
            Self::Uint8(_)  => SigType::Uint8,
            Self::Int16(_)  => SigType::Int16,
            Self::Uint16(_) => SigType::Uint16,
            Self::Float(_)  => SigType::Float,
        }
    }

    /// Encode the value into a 4-byte little-endian representation.
    pub fn encode_bytes(&self) -> [u8; 4] {
        match self {
            Self::Bool(v)   => [*v as u8, 0, 0, 0],
            Self::Uint8(v)  => [*v, 0, 0, 0],
            Self::Int16(v)  => {
                let b = v.to_le_bytes();
                [b[0], b[1], 0, 0]
            }
            Self::Uint16(v) => {
                let b = v.to_le_bytes();
                [b[0], b[1], 0, 0]
            }
            Self::Float(v)  => v.to_le_bytes(),
        }
    }

    /// Decode a value from a 4-byte buffer given the type tag.
    pub fn decode_bytes(sig_type: SigType, bytes: [u8; 4]) -> Self {
        match sig_type {
            SigType::Bool   => Self::Bool(bytes[0] != 0),
            SigType::Uint8  => Self::Uint8(bytes[0]),
            SigType::Int16  => Self::Int16(i16::from_le_bytes([bytes[0], bytes[1]])),
            SigType::Uint16 => Self::Uint16(u16::from_le_bytes([bytes[0], bytes[1]])),
            SigType::Float  => Self::Float(f32::from_le_bytes(bytes)),
        }
    }
}

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

/// Feature identifiers — must match Safety Monitor's compiled table.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[repr(u8)]
pub enum FeatureId {
    Peps           = 0x01,
    Hazard         = 0x02,
    TurnIndicator  = 0x03,
    LowBeam        = 0x04,
    HighBeam       = 0x05,
    Drl            = 0x06,
    AutoLock       = 0x07,
    LockFeedback   = 0x08,
    Welcome        = 0x09,
}

impl FeatureId {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x01 => Some(Self::Peps),
            0x02 => Some(Self::Hazard),
            0x03 => Some(Self::TurnIndicator),
            0x04 => Some(Self::LowBeam),
            0x05 => Some(Self::HighBeam),
            0x06 => Some(Self::Drl),
            0x07 => Some(Self::AutoLock),
            0x08 => Some(Self::LockFeedback),
            0x09 => Some(Self::Welcome),
            _    => None,
        }
    }
}

/// Arbiter priority levels.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[repr(u8)]
pub enum Priority {
    Low    = 1,
    Medium = 2,
    High   = 3,
}

impl Priority {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            1 => Some(Self::Low),
            2 => Some(Self::Medium),
            3 => Some(Self::High),
            _ => None,
        }
    }
}

/// IPC message type discriminant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MsgType {
    ActuatorCmd  = 0x01,
    StateUpdate  = 0x02,
    CmdAck       = 0x03,
    FaultReport  = 0x04,
}

impl MsgType {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x01 => Some(Self::ActuatorCmd),
            0x02 => Some(Self::StateUpdate),
            0x03 => Some(Self::CmdAck),
            0x04 => Some(Self::FaultReport),
            _ => None,
        }
    }
}

/// Ack status codes returned by the Safety Monitor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AckStatus {
    Ok            = 0x00,
    ErrSafety     = 0x01,
    ErrPriority   = 0x02,
    ErrState      = 0x03,
    ErrChecksum   = 0x04,
    ErrVersion    = 0x05,
    ErrUnknownSig = 0x06,
}

impl AckStatus {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x00 => Some(Self::Ok),
            0x01 => Some(Self::ErrSafety),
            0x02 => Some(Self::ErrPriority),
            0x03 => Some(Self::ErrState),
            0x04 => Some(Self::ErrChecksum),
            0x05 => Some(Self::ErrVersion),
            0x06 => Some(Self::ErrUnknownSig),
            _    => None,
        }
    }
}

/// Fault codes from the Safety Monitor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FaultCode {
    LampOpenCircuit  = 0x01,
    LampShort        = 0x02,
    ActuatorTimeout  = 0x03,
    SensorLost       = 0x04,
    NvmError         = 0x05,
}

impl FaultCode {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x01 => Some(Self::LampOpenCircuit),
            0x02 => Some(Self::LampShort),
            0x03 => Some(Self::ActuatorTimeout),
            0x04 => Some(Self::SensorLost),
            0x05 => Some(Self::NvmError),
            _    => None,
        }
    }
}

/// Fault severity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FaultSeverity {
    Warning  = 0,
    Critical = 1,
}

// ---------------------------------------------------------------------------
// 16-byte shared header
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct IpcHeader {
    pub magic:     u32,
    pub version:   u8,
    pub msg_type:  MsgType,
    pub seq:       u16,
    pub timestamp: u32,
    pub signal_id: u32,
}

impl IpcHeader {
    pub fn encode(&self, buf: &mut [u8]) {
        buf[0..4].copy_from_slice(&self.magic.to_le_bytes());
        buf[4] = self.version;
        buf[5] = self.msg_type as u8;
        buf[6..8].copy_from_slice(&self.seq.to_le_bytes());
        buf[8..12].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[12..16].copy_from_slice(&self.signal_id.to_le_bytes());
    }

    pub fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        if buf.len() < 16 {
            return Err(DecodeError::BufferTooShort);
        }
        let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        if magic != IPC_MAGIC {
            return Err(DecodeError::BadMagic(magic));
        }
        let version = buf[4];
        if version != IPC_VERSION {
            return Err(DecodeError::BadVersion(version));
        }
        let msg_type = MsgType::from_u8(buf[5]).ok_or(DecodeError::UnknownMsgType(buf[5]))?;
        let seq = u16::from_le_bytes([buf[6], buf[7]]);
        let timestamp = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
        let signal_id = u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]);

        Ok(Self { magic, version, msg_type, seq, timestamp, signal_id })
    }
}

// ---------------------------------------------------------------------------
// Message structs
// ---------------------------------------------------------------------------

/// A53 → M7: actuator command (28 bytes total).
#[derive(Debug, Clone)]
pub struct VssActuatorCmd {
    pub header:     IpcHeader,
    pub feature_id: FeatureId,
    pub priority:   Priority,
    pub value:      SignalValue,
}

impl VssActuatorCmd {
    pub const SIZE: usize = 28;

    pub fn encode(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        self.header.encode(&mut buf);
        buf[16] = self.feature_id as u8;
        buf[17] = self.priority as u8;
        buf[18] = self.value.sig_type() as u8;
        buf[19] = 0; // pad
        buf[20..24].copy_from_slice(&self.value.encode_bytes());
        let crc = CRC_ALGO.checksum(&buf[..24]);
        buf[24..26].copy_from_slice(&crc.to_le_bytes());
        buf[26..28].copy_from_slice(&[0, 0]); // pad
        buf
    }

    pub fn decode(buf: &[u8; Self::SIZE]) -> Result<Self, DecodeError> {
        let header = IpcHeader::decode(buf)?;
        verify_crc(buf, 24)?;
        let feature_id = FeatureId::from_u8(buf[16])
            .ok_or(DecodeError::UnknownFeatureId(buf[16]))?;
        let priority = Priority::from_u8(buf[17])
            .ok_or(DecodeError::UnknownPriority(buf[17]))?;
        let sig_type = SigType::from_u8(buf[18])
            .ok_or(DecodeError::UnknownSigType(buf[18]))?;
        let value = SignalValue::decode_bytes(sig_type, [buf[20], buf[21], buf[22], buf[23]]);
        Ok(Self { header, feature_id, priority, value })
    }
}

/// M7 → A53: state update (28 bytes total).
#[derive(Debug, Clone)]
pub struct VssStateUpdate {
    pub header:       IpcHeader,
    pub value:        SignalValue,
    pub last_feature: FeatureId,
}

impl VssStateUpdate {
    pub const SIZE: usize = 28;

    pub fn encode(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        self.header.encode(&mut buf);
        buf[16] = self.value.sig_type() as u8;
        buf[17] = self.last_feature as u8;
        buf[18..20].copy_from_slice(&[0, 0]); // pad
        buf[20..24].copy_from_slice(&self.value.encode_bytes());
        let crc = CRC_ALGO.checksum(&buf[..24]);
        buf[24..26].copy_from_slice(&crc.to_le_bytes());
        buf[26..28].copy_from_slice(&[0, 0]);
        buf
    }

    pub fn decode(buf: &[u8; Self::SIZE]) -> Result<Self, DecodeError> {
        let header = IpcHeader::decode(buf)?;
        verify_crc(buf, 24)?;
        let sig_type = SigType::from_u8(buf[16])
            .ok_or(DecodeError::UnknownSigType(buf[16]))?;
        let last_feature = FeatureId::from_u8(buf[17])
            .ok_or(DecodeError::UnknownFeatureId(buf[17]))?;
        let value = SignalValue::decode_bytes(sig_type, [buf[20], buf[21], buf[22], buf[23]]);
        Ok(Self { header, value, last_feature })
    }
}

/// M7 → A53: command acknowledgement (24 bytes total).
#[derive(Debug, Clone)]
pub struct VssCmdAck {
    pub header:  IpcHeader,
    pub ack_seq: u16,
    pub status:  AckStatus,
}

impl VssCmdAck {
    pub const SIZE: usize = 24;

    pub fn encode(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        self.header.encode(&mut buf);
        buf[16..18].copy_from_slice(&self.ack_seq.to_le_bytes());
        buf[18] = self.status as u8;
        buf[19] = 0; // pad
        let crc = CRC_ALGO.checksum(&buf[..20]);
        buf[20..22].copy_from_slice(&crc.to_le_bytes());
        buf[22..24].copy_from_slice(&[0, 0]);
        buf
    }

    pub fn decode(buf: &[u8; Self::SIZE]) -> Result<Self, DecodeError> {
        let header = IpcHeader::decode(buf)?;
        verify_crc(buf, 20)?;
        let ack_seq = u16::from_le_bytes([buf[16], buf[17]]);
        let status = AckStatus::from_u8(buf[18])
            .ok_or(DecodeError::UnknownAckStatus(buf[18]))?;
        Ok(Self { header, ack_seq, status })
    }
}

/// M7 → A53: fault report (24 bytes total).
#[derive(Debug, Clone)]
pub struct VssFaultReport {
    pub header:     IpcHeader,
    pub fault_code: FaultCode,
    pub severity:   FaultSeverity,
}

impl VssFaultReport {
    pub const SIZE: usize = 24;

    pub fn encode(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        self.header.encode(&mut buf);
        buf[16] = self.fault_code as u8;
        buf[17] = self.severity as u8;
        buf[18..20].copy_from_slice(&[0, 0]); // pad
        let crc = CRC_ALGO.checksum(&buf[..20]);
        buf[20..22].copy_from_slice(&crc.to_le_bytes());
        buf[22..24].copy_from_slice(&[0, 0]);
        buf
    }

    pub fn decode(buf: &[u8; Self::SIZE]) -> Result<Self, DecodeError> {
        let header = IpcHeader::decode(buf)?;
        verify_crc(buf, 20)?;
        let fault_code = FaultCode::from_u8(buf[16])
            .ok_or(DecodeError::UnknownFaultCode(buf[16]))?;
        let severity = match buf[17] {
            0 => FaultSeverity::Warning,
            1 => FaultSeverity::Critical,
            v => return Err(DecodeError::UnknownSeverity(v)),
        };
        Ok(Self { header, fault_code, severity })
    }
}

/// Parses an inbound message from M7 (STATE_UPDATE, CMD_ACK, or FAULT_REPORT).
pub enum InboundMessage {
    StateUpdate(VssStateUpdate),
    CmdAck(VssCmdAck),
    FaultReport(VssFaultReport),
}

impl InboundMessage {
    pub fn parse(buf: &[u8]) -> Result<Self, DecodeError> {
        if buf.len() < 16 {
            return Err(DecodeError::BufferTooShort);
        }
        let header = IpcHeader::decode(buf)?;
        match header.msg_type {
            MsgType::StateUpdate => {
                if buf.len() < VssStateUpdate::SIZE {
                    return Err(DecodeError::BufferTooShort);
                }
                let arr: &[u8; 28] = buf[..28].try_into().unwrap();
                Ok(Self::StateUpdate(VssStateUpdate::decode(arr)?))
            }
            MsgType::CmdAck => {
                if buf.len() < VssCmdAck::SIZE {
                    return Err(DecodeError::BufferTooShort);
                }
                let arr: &[u8; 24] = buf[..24].try_into().unwrap();
                Ok(Self::CmdAck(VssCmdAck::decode(arr)?))
            }
            MsgType::FaultReport => {
                if buf.len() < VssFaultReport::SIZE {
                    return Err(DecodeError::BufferTooShort);
                }
                let arr: &[u8; 24] = buf[..24].try_into().unwrap();
                Ok(Self::FaultReport(VssFaultReport::decode(arr)?))
            }
            MsgType::ActuatorCmd => Err(DecodeError::UnexpectedMsgType(MsgType::ActuatorCmd)),
        }
    }
}

// ---------------------------------------------------------------------------
// CRC helpers
// ---------------------------------------------------------------------------

fn verify_crc(buf: &[u8], crc_offset: usize) -> Result<(), DecodeError> {
    let expected = CRC_ALGO.checksum(&buf[..crc_offset]);
    let actual = u16::from_le_bytes([buf[crc_offset], buf[crc_offset + 1]]);
    if expected != actual {
        return Err(DecodeError::CrcMismatch { expected, actual });
    }
    Ok(())
}

/// Compute CRC-16/CCITT-FALSE over the given bytes.
pub fn compute_crc16(data: &[u8]) -> u16 {
    CRC_ALGO.checksum(data)
}

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

#[derive(Debug, thiserror::Error)]
pub enum DecodeError {
    #[error("buffer too short")]
    BufferTooShort,
    #[error("bad magic: 0x{0:08X}")]
    BadMagic(u32),
    #[error("unsupported version: {0}")]
    BadVersion(u8),
    #[error("unknown msg_type: 0x{0:02X}")]
    UnknownMsgType(u8),
    #[error("unknown feature_id: 0x{0:02X}")]
    UnknownFeatureId(u8),
    #[error("unknown priority: {0}")]
    UnknownPriority(u8),
    #[error("unknown sig_type: {0}")]
    UnknownSigType(u8),
    #[error("unknown ack status: 0x{0:02X}")]
    UnknownAckStatus(u8),
    #[error("unknown fault code: 0x{0:02X}")]
    UnknownFaultCode(u8),
    #[error("unknown severity: {0}")]
    UnknownSeverity(u8),
    #[error("CRC mismatch: expected 0x{expected:04X}, got 0x{actual:04X}")]
    CrcMismatch { expected: u16, actual: u16 },
    #[error("unexpected msg_type for inbound: {0:?}")]
    UnexpectedMsgType(MsgType),
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_header(msg_type: MsgType, seq: u16, signal_id: u32) -> IpcHeader {
        IpcHeader {
            magic: IPC_MAGIC,
            version: IPC_VERSION,
            msg_type,
            seq,
            timestamp: 12345,
            signal_id,
        }
    }

    #[test]
    fn signal_value_roundtrip() {
        let cases: Vec<SignalValue> = vec![
            SignalValue::Bool(true),
            SignalValue::Bool(false),
            SignalValue::Uint8(42),
            SignalValue::Int16(-1234),
            SignalValue::Uint16(65000),
            SignalValue::Float(3.14),
        ];
        for val in cases {
            let encoded = val.encode_bytes();
            let decoded = SignalValue::decode_bytes(val.sig_type(), encoded);
            match (val, decoded) {
                (SignalValue::Float(a), SignalValue::Float(b)) => {
                    assert!((a - b).abs() < f32::EPSILON);
                }
                _ => assert_eq!(val, decoded),
            }
        }
    }

    #[test]
    fn actuator_cmd_roundtrip() {
        let cmd = VssActuatorCmd {
            header: make_header(MsgType::ActuatorCmd, 1, 0x1000),
            feature_id: FeatureId::Hazard,
            priority: Priority::High,
            value: SignalValue::Bool(true),
        };
        let encoded = cmd.encode();
        assert_eq!(encoded.len(), VssActuatorCmd::SIZE);
        let decoded = VssActuatorCmd::decode(&encoded).unwrap();
        assert_eq!(decoded.feature_id, FeatureId::Hazard);
        assert_eq!(decoded.priority, Priority::High);
        assert_eq!(decoded.value, SignalValue::Bool(true));
        assert_eq!(decoded.header.seq, 1);
        assert_eq!(decoded.header.signal_id, 0x1000);
    }

    #[test]
    fn state_update_roundtrip() {
        let msg = VssStateUpdate {
            header: make_header(MsgType::StateUpdate, 5, 0x2000),
            value: SignalValue::Uint8(100),
            last_feature: FeatureId::Peps,
        };
        let encoded = msg.encode();
        assert_eq!(encoded.len(), VssStateUpdate::SIZE);
        let decoded = VssStateUpdate::decode(&encoded).unwrap();
        assert_eq!(decoded.value, SignalValue::Uint8(100));
        assert_eq!(decoded.last_feature, FeatureId::Peps);
    }

    #[test]
    fn cmd_ack_roundtrip() {
        let msg = VssCmdAck {
            header: make_header(MsgType::CmdAck, 3, 0x1000),
            ack_seq: 1,
            status: AckStatus::Ok,
        };
        let encoded = msg.encode();
        assert_eq!(encoded.len(), VssCmdAck::SIZE);
        let decoded = VssCmdAck::decode(&encoded).unwrap();
        assert_eq!(decoded.ack_seq, 1);
        assert_eq!(decoded.status, AckStatus::Ok);
    }

    #[test]
    fn fault_report_roundtrip() {
        let msg = VssFaultReport {
            header: make_header(MsgType::FaultReport, 10, 0x3000),
            fault_code: FaultCode::LampOpenCircuit,
            severity: FaultSeverity::Critical,
        };
        let encoded = msg.encode();
        assert_eq!(encoded.len(), VssFaultReport::SIZE);
        let decoded = VssFaultReport::decode(&encoded).unwrap();
        assert_eq!(decoded.fault_code, FaultCode::LampOpenCircuit);
        assert_eq!(decoded.severity, FaultSeverity::Critical);
    }

    #[test]
    fn crc_tamper_detected() {
        let cmd = VssActuatorCmd {
            header: make_header(MsgType::ActuatorCmd, 1, 0x1000),
            feature_id: FeatureId::LowBeam,
            priority: Priority::Medium,
            value: SignalValue::Bool(true),
        };
        let mut encoded = cmd.encode();
        // Tamper with the value byte
        encoded[20] ^= 0xFF;
        let result = VssActuatorCmd::decode(&encoded);
        assert!(matches!(result, Err(DecodeError::CrcMismatch { .. })));
    }

    #[test]
    fn bad_magic_rejected() {
        let mut buf = [0u8; 28];
        buf[0..4].copy_from_slice(&0xDEAD_BEEFu32.to_le_bytes());
        let result = IpcHeader::decode(&buf);
        assert!(matches!(result, Err(DecodeError::BadMagic(0xDEAD_BEEF))));
    }

    #[test]
    fn inbound_parse_dispatches_correctly() {
        let update = VssStateUpdate {
            header: make_header(MsgType::StateUpdate, 1, 0x1000),
            value: SignalValue::Bool(true),
            last_feature: FeatureId::Hazard,
        };
        let buf = update.encode();
        let parsed = InboundMessage::parse(&buf).unwrap();
        assert!(matches!(parsed, InboundMessage::StateUpdate(_)));

        let ack = VssCmdAck {
            header: make_header(MsgType::CmdAck, 2, 0x1000),
            ack_seq: 1,
            status: AckStatus::ErrSafety,
        };
        let buf = ack.encode();
        let parsed = InboundMessage::parse(&buf).unwrap();
        assert!(matches!(parsed, InboundMessage::CmdAck(_)));
    }
}
