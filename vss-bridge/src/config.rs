//! Four-tier configuration system for cross-program reusability.
//!
//! The body controller platform serves multiple OEM vehicle programs.
//! Parameters are stratified into four tiers based on who sets them
//! and when they can change:
//!
//! ## Tier 1 — Compile-time constants (`const`)
//!
//! Values fixed at build time. Never change once the binary is compiled.
//! Used for protocol constants, wire format versions, physical limits,
//! and architectural invariants that are the same across every vehicle
//! program that uses this platform.
//!
//! Examples: IPC magic number, CRC polynomial, max doors (4), arbiter
//! crash lockout (10 s), motor actuation time (300 ms), priority levels.
//!
//! Implementation: Rust `const` items in the module that owns them.
//! Zero runtime cost. Enforced by the compiler.
//!
//! ## Tier 2 — Vehicle-line calibrations (`VehicleLineCal`)
//!
//! Values that are the same for every variant of a given vehicle line
//! (e.g., all Camry trims share the same auto-relock timeout) but may
//! differ across vehicle lines (Camry vs. RAV4 vs. Tundra).
//!
//! Shipped as a JSON calibration file baked into the container image
//! at build time for each vehicle line. Loaded once at startup.
//! Changed only by rebuilding and OTA-updating the container.
//!
//! Examples: auto-relock timeout (45 s), PEPS LF transmit window (3 s),
//! DRL brightness level, welcome-light duration.
//!
//! Implementation: `VehicleLineCal` struct deserialized from
//! `/etc/vss-bridge/vehicle_line.json` (bind-mounted or baked in).
//!
//! ## Tier 3 — Variant/trim calibrations (`VariantCal`)
//!
//! Values specific to an individual trim or option package within a
//! vehicle line (e.g., Camry LE vs. Camry XSE). Flashed as a whole
//! calibration file and considered part of the Bill of Materials (BOM)
//! for that trim. Different trims may have different feature sets
//! enabled, different sensor configurations, or different thresholds.
//!
//! Examples: auto-lock speed threshold (20 km/h on base, 15 km/h on
//! sport), double-lock enabled (yes on premium, no on base), NFC
//! enabled (yes on tech package, no on base), welcome light pattern
//! (sequential on premium, simple on base).
//!
//! Implementation: `VariantCal` struct deserialized from
//! `/etc/vss-bridge/variant.json`. This file is flashed by the
//! assembly plant or reflash tool as part of the vehicle order.
//! Separate from the container image — survives OTA software updates.
//!
//! ## Tier 4 — Dealer-configurable parameters (`DealerConfig`)
//!
//! Values that a dealer technician can change using a diagnostic tool
//! via UDS (ISO 14229) WriteDataByIdentifier service 0x2E. The M7
//! AUTOSAR Classic stack owns the UDS server and NVM persistence.
//! After a 0x2E write, the M7 pushes the updated value to the A53
//! via a `CONFIG_UPDATE` IPC message.
//!
//! Examples: auto-relock enable/disable (customer preference),
//! horn chirp on lock (enable/disable), courtesy light timeout,
//! remote start duration.
//!
//! Implementation: `DealerConfig` struct. Initial values loaded from
//! M7 at boot (M7 pushes current config). Updated at runtime when
//! M7 forwards a 0x2E write. The A53 never writes these directly —
//! the M7 is the authority and NVM owner.
//!
//! ## Loading order
//!
//! ```text
//! 1. Compile-time constants — always available, zero-cost
//! 2. VehicleLineCal  ← /etc/vss-bridge/vehicle_line.json (startup)
//! 3. VariantCal      ← /etc/vss-bridge/variant.json      (startup)
//! 4. DealerConfig    ← M7 CONFIG_UPDATE via RPmsg         (startup + runtime)
//! ```
//!
//! Features receive an `Arc<PlatformConfig>` which merges all tiers
//! into a single read-only view. Dealer config changes at runtime
//! are propagated via a `tokio::sync::watch` channel.

use std::path::Path;
use std::sync::Arc;
use std::time::Duration;

use serde::Deserialize;
use tokio::sync::watch;

// ═══════════════════════════════════════════════════════════════════════
// Tier 1 — Compile-time constants
// ═══════════════════════════════════════════════════════════════════════

/// IPC wire format magic number.
pub const IPC_MAGIC: u32 = 0xBCC0_1A00;

/// IPC schema version.
pub const IPC_VERSION: u8 = 1;

/// Maximum number of doors on the platform (architectural limit).
pub const MAX_DOORS: usize = 4;

/// DoorLock arbiter crash lockout duration.
/// After a CrashUnlock, no new requests are accepted for this long.
/// This is an architectural safety invariant — not calibratable.
pub const CRASH_LOCKOUT: Duration = Duration::from_secs(10);

/// Lock motor actuation time (worst-case).
/// Used for arbiter queue timing. Actual time varies by motor and
/// temperature; the ACK from Classic AUTOSAR is the real completion
/// signal. This constant is a timeout guard only.
pub const LOCK_MOTOR_TIMEOUT: Duration = Duration::from_millis(2000);

/// Priority levels for the DomainArbiter.
pub const PRIORITY_LOW: u8 = 1;
pub const PRIORITY_MEDIUM: u8 = 2;
pub const PRIORITY_HIGH: u8 = 3;

// ═══════════════════════════════════════════════════════════════════════
// Tier 2 — Vehicle-line calibrations
// ═══════════════════════════════════════════════════════════════════════

/// Calibration parameters common to an entire vehicle line.
/// Loaded from `/etc/vss-bridge/vehicle_line.json`.
#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct VehicleLineCal {
    /// Auto-relock timeout in seconds.
    /// How long after an unlock event (with no door opened) before the
    /// vehicle automatically re-locks.
    pub auto_relock_timeout_secs: u64,

    /// PEPS LF transmit window in milliseconds.
    /// How long the M7 drives the LF antennas after a capacitive touch
    /// event, waiting for a keyfob RF response.
    pub peps_lf_window_ms: u64,

    /// Welcome light duration in seconds.
    /// How long exterior lights stay on in welcome mode after PEPS unlock.
    pub welcome_light_duration_secs: u64,

    /// DRL brightness percentage (0–100).
    pub drl_brightness_pct: u8,

    /// Lock feedback blink count.
    /// Number of indicator flashes on lock/unlock confirmation.
    pub lock_feedback_blink_count: u8,

    /// Lock feedback blink period in milliseconds (on + off = one period).
    pub lock_feedback_blink_period_ms: u64,

    /// A53 shutdown grace period in seconds.
    /// After ignition OFF or M7-initiated wake work complete, how long
    /// the A53 stays alive for pending operations before powering down.
    pub shutdown_grace_secs: u64,
}

impl Default for VehicleLineCal {
    fn default() -> Self {
        Self {
            auto_relock_timeout_secs: 45,
            peps_lf_window_ms: 3000,
            welcome_light_duration_secs: 30,
            drl_brightness_pct: 100,
            lock_feedback_blink_count: 3,
            lock_feedback_blink_period_ms: 400,
            shutdown_grace_secs: 30,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
// Tier 3 — Variant / trim calibrations
// ═══════════════════════════════════════════════════════════════════════

/// Calibration parameters specific to a vehicle variant or trim level.
/// Loaded from `/etc/vss-bridge/variant.json`.
/// This file is part of the BOM — flashed at the assembly plant or
/// by reflash tool, survives OTA software updates.
#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct VariantCal {
    /// Auto-lock speed threshold in km/h.
    /// Doors lock automatically when vehicle speed exceeds this.
    pub auto_lock_speed_kmh: u16,

    /// Whether double-lock (deadlock / superlocking) is available.
    /// Disables interior door handles when engaged.
    /// Typically enabled on premium trims only.
    pub double_lock_enabled: bool,

    /// Whether NFC card/phone unlock is available.
    /// Requires NFC reader hardware in B-pillar.
    pub nfc_enabled: bool,

    /// Whether BLE digital key is available.
    /// Requires BLE antenna and CCC/ICCE stack.
    pub ble_key_enabled: bool,

    /// Whether phone app remote lock/unlock is available.
    /// Requires telematics connectivity (TCU).
    pub remote_lock_enabled: bool,

    /// Welcome light pattern.
    pub welcome_light_pattern: WelcomeLightPattern,

    /// Door configuration for this variant.
    pub doors: DoorConfig,
}

/// Door configuration — which doors are present and whether they are
/// removable. Features use this to determine which VSS door signals
/// to monitor. A 2-door coupe only has Row1; a Bronco/Wrangler has
/// all four doors but they are removable.
#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct DoorConfig {
    /// Front left door present (always true for any production vehicle).
    pub row1_left: bool,
    /// Front right door present (always true for any production vehicle).
    pub row1_right: bool,
    /// Rear left door present (false for 2-door coupe).
    pub row2_left: bool,
    /// Rear right door present (false for 2-door coupe).
    pub row2_right: bool,

    /// Whether doors are removable (Bronco, Wrangler, etc.).
    /// When true, features must also subscribe to a door-removed sensor
    /// signal and exclude removed doors from open/lock monitoring.
    /// A removed door is neither "open" nor "closed" — it is absent.
    pub removable: bool,
}

impl Default for DoorConfig {
    fn default() -> Self {
        Self {
            row1_left: true,
            row1_right: true,
            row2_left: true,
            row2_right: true,
            removable: false,
        }
    }
}

impl DoorConfig {
    /// Two-door configuration (coupe).
    pub fn two_door() -> Self {
        Self {
            row1_left: true,
            row1_right: true,
            row2_left: false,
            row2_right: false,
            removable: false,
        }
    }

    /// Four-door with removable doors (Bronco, Wrangler).
    pub fn four_door_removable() -> Self {
        Self {
            row1_left: true,
            row1_right: true,
            row2_left: true,
            row2_right: true,
            removable: true,
        }
    }

    /// Returns the list of VSS door signal suffixes for doors that are
    /// present in this variant. Used by features to build their signal
    /// subscription list.
    ///
    /// Returns e.g. `["Row1.Left", "Row1.Right"]` for a 2-door coupe,
    /// or `["Row1.Left", "Row1.Right", "Row2.Left", "Row2.Right"]` for
    /// a 4-door sedan.
    pub fn present_doors(&self) -> Vec<&'static str> {
        let mut doors = Vec::with_capacity(4);
        if self.row1_left {
            doors.push("Row1.Left");
        }
        if self.row1_right {
            doors.push("Row1.Right");
        }
        if self.row2_left {
            doors.push("Row2.Left");
        }
        if self.row2_right {
            doors.push("Row2.Right");
        }
        doors
    }

    /// Returns full VSS paths for IsLocked signals of present doors.
    pub fn lock_signals(&self) -> Vec<&'static str> {
        self.present_doors()
            .iter()
            .map(|d| match *d {
                "Row1.Left" => "Body.Doors.Row1.Left.IsLocked",
                "Row1.Right" => "Body.Doors.Row1.Right.IsLocked",
                "Row2.Left" => "Body.Doors.Row2.Left.IsLocked",
                "Row2.Right" => "Body.Doors.Row2.Right.IsLocked",
                _ => unreachable!(),
            })
            .collect()
    }

    /// Returns full VSS paths for IsOpen signals of present doors.
    pub fn open_signals(&self) -> Vec<&'static str> {
        self.present_doors()
            .iter()
            .map(|d| match *d {
                "Row1.Left" => "Body.Doors.Row1.Left.IsOpen",
                "Row1.Right" => "Body.Doors.Row1.Right.IsOpen",
                "Row2.Left" => "Body.Doors.Row2.Left.IsOpen",
                "Row2.Right" => "Body.Doors.Row2.Right.IsOpen",
                _ => unreachable!(),
            })
            .collect()
    }

    /// Returns full VSS paths for IsRemoved signals (only meaningful
    /// when `removable` is true).
    pub fn removed_signals(&self) -> Vec<&'static str> {
        if !self.removable {
            return Vec::new();
        }
        self.present_doors()
            .iter()
            .map(|d| match *d {
                "Row1.Left" => "Body.Doors.Row1.Left.IsRemoved",
                "Row1.Right" => "Body.Doors.Row1.Right.IsRemoved",
                "Row2.Left" => "Body.Doors.Row2.Left.IsRemoved",
                "Row2.Right" => "Body.Doors.Row2.Right.IsRemoved",
                _ => unreachable!(),
            })
            .collect()
    }
}

/// Welcome light pattern options.
#[derive(Debug, Clone, Default, Deserialize, PartialEq)]
pub enum WelcomeLightPattern {
    /// Simple on/off.
    #[default]
    Simple,
    /// Sequential sweep (premium).
    Sequential,
    /// No welcome lights.
    Disabled,
}

impl Default for VariantCal {
    fn default() -> Self {
        Self {
            auto_lock_speed_kmh: 20,
            double_lock_enabled: false,
            nfc_enabled: false,
            ble_key_enabled: false,
            remote_lock_enabled: false,
            welcome_light_pattern: WelcomeLightPattern::Simple,
            doors: DoorConfig::default(),
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
// Tier 4 — Dealer-configurable parameters
// ═══════════════════════════════════════════════════════════════════════

/// Parameters changeable by a dealer via UDS 0x2E (WriteDataByIdentifier).
/// The M7 owns the UDS server and NVM. It pushes current values to the
/// A53 at boot and on every 0x2E write.
///
/// Each field maps to a specific DID (Data Identifier) in the AUTOSAR
/// diagnostic configuration. DIDs are documented in the comment.
#[derive(Debug, Clone, Deserialize)]
#[serde(default)]
pub struct DealerConfig {
    /// Auto-relock feature enable/disable.
    /// DID 0xF190. Customer may request disable at dealer.
    pub auto_relock_enabled: bool,

    /// Horn chirp on lock confirmation.
    /// DID 0xF191. Some markets/customers prefer silent lock.
    pub horn_chirp_on_lock: bool,

    /// Courtesy light timeout in seconds.
    /// DID 0xF192. How long interior lights stay on after door close.
    pub courtesy_light_timeout_secs: u64,

    /// Remote start maximum duration in minutes.
    /// DID 0xF193. Engine/climate pre-conditioning duration limit.
    pub remote_start_max_minutes: u64,

    /// Approach unlock mode.
    /// DID 0xF194. DRIVER_ONLY = unlock driver door on first PEPS
    /// approach, ALL = unlock all doors.
    pub approach_unlock_mode: ApproachUnlockMode,
}

/// PEPS approach unlock behavior.
#[derive(Debug, Clone, Default, Deserialize, PartialEq)]
pub enum ApproachUnlockMode {
    /// Unlock driver door only on first approach; second pull unlocks all.
    #[default]
    DriverOnly,
    /// Unlock all doors on approach.
    All,
}

impl Default for DealerConfig {
    fn default() -> Self {
        Self {
            auto_relock_enabled: true,
            horn_chirp_on_lock: true,
            courtesy_light_timeout_secs: 30,
            remote_start_max_minutes: 10,
            approach_unlock_mode: ApproachUnlockMode::DriverOnly,
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
// PlatformConfig — merged view of all tiers
// ═══════════════════════════════════════════════════════════════════════

/// Merged read view of all configuration tiers.
/// Features receive `Arc<PlatformConfig>` at construction.
///
/// Tier 2 and 3 are immutable after startup. Tier 4 (dealer config)
/// can change at runtime via the `dealer_config_rx` watch channel.
pub struct PlatformConfig {
    pub vehicle_line: VehicleLineCal,
    pub variant: VariantCal,
    dealer_config_tx: watch::Sender<DealerConfig>,
    dealer_config_rx: watch::Receiver<DealerConfig>,
}

impl PlatformConfig {
    /// Load configuration from the standard file paths.
    ///
    /// Missing files are not an error — defaults are used. This allows
    /// development without a full calibration file set.
    pub fn load() -> Arc<Self> {
        let vehicle_line =
            load_json_or_default::<VehicleLineCal>("/etc/vss-bridge/vehicle_line.json");
        let variant = load_json_or_default::<VariantCal>("/etc/vss-bridge/variant.json");
        let dealer = DealerConfig::default(); // M7 pushes real values at boot

        let (dealer_config_tx, dealer_config_rx) = watch::channel(dealer);

        tracing::info!(
            auto_relock_timeout = vehicle_line.auto_relock_timeout_secs,
            auto_lock_speed = variant.auto_lock_speed_kmh,
            double_lock = variant.double_lock_enabled,
            nfc = variant.nfc_enabled,
            ble = variant.ble_key_enabled,
            "platform config loaded"
        );

        Arc::new(Self {
            vehicle_line,
            variant,
            dealer_config_tx,
            dealer_config_rx,
        })
    }

    /// Load from explicit paths (for testing or non-standard deployments).
    pub fn load_from(vehicle_line_path: Option<&str>, variant_path: Option<&str>) -> Arc<Self> {
        let vehicle_line = vehicle_line_path
            .map(load_json_or_default::<VehicleLineCal>)
            .unwrap_or_default();
        let variant = variant_path
            .map(load_json_or_default::<VariantCal>)
            .unwrap_or_default();
        let dealer = DealerConfig::default();

        let (dealer_config_tx, dealer_config_rx) = watch::channel(dealer);

        Arc::new(Self {
            vehicle_line,
            variant,
            dealer_config_tx,
            dealer_config_rx,
        })
    }

    /// Create with all defaults (for unit tests).
    pub fn defaults() -> Arc<Self> {
        let (dealer_config_tx, dealer_config_rx) = watch::channel(DealerConfig::default());
        Arc::new(Self {
            vehicle_line: VehicleLineCal::default(),
            variant: VariantCal::default(),
            dealer_config_tx,
            dealer_config_rx,
        })
    }

    // ── Tier 2 convenience accessors ────────────────────────────────

    /// Auto-relock timeout as a `Duration`.
    pub fn auto_relock_timeout(&self) -> Duration {
        Duration::from_secs(self.vehicle_line.auto_relock_timeout_secs)
    }

    /// Lock feedback blink period as a `Duration`.
    pub fn lock_feedback_blink_period(&self) -> Duration {
        Duration::from_millis(self.vehicle_line.lock_feedback_blink_period_ms)
    }

    /// A53 shutdown grace period as a `Duration`.
    pub fn shutdown_grace(&self) -> Duration {
        Duration::from_secs(self.vehicle_line.shutdown_grace_secs)
    }

    // ── Tier 3 convenience accessors ────────────────────────────────

    /// Whether a given feature is enabled for this variant.
    pub fn is_feature_enabled(&self, feature: &str) -> bool {
        match feature {
            "double_lock" => self.variant.double_lock_enabled,
            "nfc" => self.variant.nfc_enabled,
            "ble_key" => self.variant.ble_key_enabled,
            "remote_lock" => self.variant.remote_lock_enabled,
            _ => true, // unknown features default to enabled
        }
    }

    /// Door configuration for this variant.
    pub fn doors(&self) -> &DoorConfig {
        &self.variant.doors
    }

    // ── Tier 4 — dealer config (runtime-updatable) ──────────────────

    /// Get a snapshot of the current dealer configuration.
    pub fn dealer_config(&self) -> DealerConfig {
        self.dealer_config_rx.borrow().clone()
    }

    /// Subscribe to dealer config changes. Features that need to react
    /// to runtime config updates (e.g., auto-relock enable/disable)
    /// should clone this receiver and `await changed()`.
    pub fn dealer_config_watch(&self) -> watch::Receiver<DealerConfig> {
        self.dealer_config_rx.clone()
    }

    /// Update dealer config (called when M7 pushes a 0x2E write).
    /// This is the only way dealer config changes — the A53 never
    /// writes it directly.
    pub fn update_dealer_config(&self, new_config: DealerConfig) {
        tracing::info!(
            auto_relock = new_config.auto_relock_enabled,
            horn_chirp = new_config.horn_chirp_on_lock,
            approach_mode = ?new_config.approach_unlock_mode,
            "dealer config updated via M7"
        );
        let _ = self.dealer_config_tx.send(new_config);
    }

    /// Update a single dealer config field by DID.
    /// Called when M7 forwards a single 0x2E write for one parameter.
    pub fn update_dealer_did(&self, did: u16, value: &[u8]) {
        let mut config = self.dealer_config();
        match did {
            0xF190 => {
                if let Some(&v) = value.first() {
                    config.auto_relock_enabled = v != 0;
                }
            }
            0xF191 => {
                if let Some(&v) = value.first() {
                    config.horn_chirp_on_lock = v != 0;
                }
            }
            0xF192 => {
                if value.len() >= 2 {
                    config.courtesy_light_timeout_secs =
                        u16::from_be_bytes([value[0], value[1]]) as u64;
                }
            }
            0xF193 => {
                if let Some(&v) = value.first() {
                    config.remote_start_max_minutes = v as u64;
                }
            }
            0xF194 => {
                if let Some(&v) = value.first() {
                    config.approach_unlock_mode = if v == 0 {
                        ApproachUnlockMode::DriverOnly
                    } else {
                        ApproachUnlockMode::All
                    };
                }
            }
            _ => {
                tracing::warn!(did = did, "unknown dealer DID, ignoring");
            }
        }
        self.update_dealer_config(config);
    }
}

// ═══════════════════════════════════════════════════════════════════════
// File loading helper
// ═══════════════════════════════════════════════════════════════════════

fn load_json_or_default<T: Default + serde::de::DeserializeOwned>(path: &str) -> T {
    let p = Path::new(path);
    if !p.exists() {
        tracing::info!(path = path, "config file not found, using defaults");
        return T::default();
    }
    match std::fs::read_to_string(p) {
        Ok(contents) => match serde_json::from_str(&contents) {
            Ok(val) => {
                tracing::info!(path = path, "config loaded");
                val
            }
            Err(e) => {
                tracing::error!(path = path, error = %e, "config parse error, using defaults");
                T::default()
            }
        },
        Err(e) => {
            tracing::error!(path = path, error = %e, "config read error, using defaults");
            T::default()
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_are_sensible() {
        let vl = VehicleLineCal::default();
        assert_eq!(vl.auto_relock_timeout_secs, 45);
        assert_eq!(vl.lock_feedback_blink_count, 3);
        assert_eq!(vl.shutdown_grace_secs, 30);

        let vc = VariantCal::default();
        assert_eq!(vc.auto_lock_speed_kmh, 20);
        assert!(!vc.double_lock_enabled);
        assert!(!vc.nfc_enabled);
        assert!(vc.doors.row2_left); // 4-door by default
        assert!(!vc.doors.removable); // not removable by default

        let dc = DealerConfig::default();
        assert!(dc.auto_relock_enabled);
        assert!(dc.horn_chirp_on_lock);
        assert_eq!(dc.approach_unlock_mode, ApproachUnlockMode::DriverOnly);
    }

    #[test]
    fn vehicle_line_json_roundtrip() {
        let json = r#"{
            "auto_relock_timeout_secs": 30,
            "welcome_light_duration_secs": 20,
            "drl_brightness_pct": 80
        }"#;
        let vl: VehicleLineCal = serde_json::from_str(json).unwrap();
        assert_eq!(vl.auto_relock_timeout_secs, 30);
        assert_eq!(vl.welcome_light_duration_secs, 20);
        assert_eq!(vl.drl_brightness_pct, 80);
        // Unset fields get defaults
        assert_eq!(vl.peps_lf_window_ms, 3000);
        assert_eq!(vl.lock_feedback_blink_count, 3);
    }

    #[test]
    fn variant_json_premium_trim() {
        let json = r#"{
            "auto_lock_speed_kmh": 15,
            "double_lock_enabled": true,
            "nfc_enabled": true,
            "ble_key_enabled": true,
            "remote_lock_enabled": true,
            "welcome_light_pattern": "Sequential"
        }"#;
        let vc: VariantCal = serde_json::from_str(json).unwrap();
        assert_eq!(vc.auto_lock_speed_kmh, 15);
        assert!(vc.double_lock_enabled);
        assert!(vc.nfc_enabled);
        assert!(vc.ble_key_enabled);
        assert_eq!(vc.welcome_light_pattern, WelcomeLightPattern::Sequential);
    }

    #[test]
    fn variant_json_base_trim() {
        let json = r#"{
            "auto_lock_speed_kmh": 20,
            "double_lock_enabled": false,
            "nfc_enabled": false,
            "ble_key_enabled": false,
            "remote_lock_enabled": false,
            "welcome_light_pattern": "Simple"
        }"#;
        let vc: VariantCal = serde_json::from_str(json).unwrap();
        assert_eq!(vc.auto_lock_speed_kmh, 20);
        assert!(!vc.double_lock_enabled);
        assert!(!vc.nfc_enabled);
        assert_eq!(vc.welcome_light_pattern, WelcomeLightPattern::Simple);
    }

    #[test]
    fn variant_json_coupe_two_door() {
        let json = r#"{
            "auto_lock_speed_kmh": 20,
            "doors": {
                "row1_left": true,
                "row1_right": true,
                "row2_left": false,
                "row2_right": false,
                "removable": false
            }
        }"#;
        let vc: VariantCal = serde_json::from_str(json).unwrap();
        assert_eq!(vc.doors.present_doors().len(), 2);
        assert_eq!(
            vc.doors.lock_signals(),
            vec![
                "Body.Doors.Row1.Left.IsLocked",
                "Body.Doors.Row1.Right.IsLocked",
            ]
        );
    }

    #[test]
    fn variant_json_removable_doors() {
        let json = r#"{
            "doors": {
                "row1_left": true,
                "row1_right": true,
                "row2_left": true,
                "row2_right": true,
                "removable": true
            }
        }"#;
        let vc: VariantCal = serde_json::from_str(json).unwrap();
        assert!(vc.doors.removable);
        assert_eq!(vc.doors.present_doors().len(), 4);
        assert_eq!(vc.doors.removed_signals().len(), 4);
        assert_eq!(
            vc.doors.removed_signals()[0],
            "Body.Doors.Row1.Left.IsRemoved"
        );
    }

    #[test]
    fn door_config_non_removable_has_no_removed_signals() {
        let dc = DoorConfig::default();
        assert!(!dc.removable);
        assert!(dc.removed_signals().is_empty());
    }

    #[test]
    fn platform_config_defaults() {
        let cfg = PlatformConfig::defaults();
        assert_eq!(cfg.auto_relock_timeout(), Duration::from_secs(45));
        assert!(cfg.is_feature_enabled("double_lock") == false);
        assert!(cfg.is_feature_enabled("unknown_feature") == true);
        assert!(cfg.dealer_config().auto_relock_enabled);
    }

    #[test]
    fn dealer_config_update() {
        let cfg = PlatformConfig::defaults();
        assert!(cfg.dealer_config().auto_relock_enabled);

        let mut new_dc = cfg.dealer_config();
        new_dc.auto_relock_enabled = false;
        new_dc.horn_chirp_on_lock = false;
        cfg.update_dealer_config(new_dc);

        assert!(!cfg.dealer_config().auto_relock_enabled);
        assert!(!cfg.dealer_config().horn_chirp_on_lock);
    }

    #[test]
    fn dealer_did_update() {
        let cfg = PlatformConfig::defaults();

        // Disable auto-relock via DID 0xF190
        cfg.update_dealer_did(0xF190, &[0x00]);
        assert!(!cfg.dealer_config().auto_relock_enabled);

        // Re-enable
        cfg.update_dealer_did(0xF190, &[0x01]);
        assert!(cfg.dealer_config().auto_relock_enabled);

        // Set approach unlock to ALL via DID 0xF194
        cfg.update_dealer_did(0xF194, &[0x01]);
        assert_eq!(
            cfg.dealer_config().approach_unlock_mode,
            ApproachUnlockMode::All
        );

        // Unknown DID — should not panic
        cfg.update_dealer_did(0xFFFF, &[0x42]);
    }

    #[test]
    fn missing_file_returns_defaults() {
        let vl = load_json_or_default::<VehicleLineCal>("/nonexistent/path/vehicle_line.json");
        assert_eq!(vl.auto_relock_timeout_secs, 45);
    }

    #[test]
    fn partial_json_fills_defaults() {
        // Only override one field — rest should be defaults
        let json = r#"{"auto_relock_timeout_secs": 60}"#;
        let vl: VehicleLineCal = serde_json::from_str(json).unwrap();
        assert_eq!(vl.auto_relock_timeout_secs, 60);
        assert_eq!(vl.peps_lf_window_ms, 3000); // default
        assert_eq!(vl.lock_feedback_blink_count, 3); // default
    }
}
