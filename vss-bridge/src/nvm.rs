//! NVM persistence layer for plant-model state that must survive a power
//! cycle.
//!
//! In a real BCM, signals like `Body.Doors.Row*.IsLocked` are stored in
//! non-volatile EEPROM/flash and read at boot.  In our simulation we
//! persist them to plain JSON files on disk so the same scenarios can be
//! reproduced.
//!
//! See `docs/signal-ownership-and-state-hydration.md` for the broader
//! architecture (which signals are NVM-backed, why, and the testing
//! affordances this enables).
//!
//! # File layout
//!
//! ```text
//! $VSS_BRIDGE_NVM_PATH/
//!     door_lock.json     ← DoorLockState (locked + double_locked arrays)
//!     trunk.json         ← (future) TrunkState
//!     windows.json       ← (future) WindowState
//! ```
//!
//! Default path: `./nvm/` relative to the bridge's CWD.  Override with
//! the `VSS_BRIDGE_NVM_PATH` environment variable.
//!
//! # Failure modes
//!
//! - **File missing**: returns `Default::default()` for the requested type
//!   and emits a `tracing::info!` (factory-new vehicle behaviour).
//! - **File present but malformed**: returns `Default::default()` and
//!   emits a `tracing::warn!` — service stays up, NVM is treated as
//!   corrupt.  The next save will overwrite cleanly.
//! - **Save failure** (disk full, permissions): emits a `tracing::error!`
//!   and continues — runtime state is unchanged in memory.
//!
//! # Atomic writes
//!
//! `save_*` methods write to `<file>.tmp`, fsync, then rename — so a
//! power cut during a write cannot leave a half-written file in place.
//! Either the old contents (last successful save) or the new contents
//! survive; never something in between.

use std::fs;
use std::io::{self, Write};
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

/// Default directory (relative to CWD) for NVM files when no env override.
const DEFAULT_NVM_DIR: &str = "nvm";

/// Environment variable that overrides the NVM directory path.
const ENV_NVM_PATH: &str = "VSS_BRIDGE_NVM_PATH";

/// Persisted state for the door-lock plant model.
///
/// `locked[i]` and `double_locked[i]` are indexed in the same order as
/// the `DOOR_LOCKED_SIGNALS` array in `plant_models/door_lock.rs`:
/// `[Row1.Left, Row1.Right, Row2.Left, Row2.Right]`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct DoorLockState {
    pub locked: [bool; 4],
    pub double_locked: [bool; 4],
}

/// Persisted state for the trunk plant model.
///
/// In a real vehicle the trunk's open/closed position is read from a
/// physical hall-effect sensor at boot — there's no NVM involved
/// because the position can't change while the BCM is off.  In our
/// simulation we persist it so that "vehicle parked with trunk open,
/// restart bridge, trunk is still open" is reproducible end-to-end.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct TrunkState {
    pub is_open: bool,
}

/// Persisted state for the hood plant model.
///
/// Same simulation rationale as `TrunkState` — in a real vehicle the
/// hood's position is read from a hall-effect / latch-position sensor
/// at boot.  We persist it so "parked with hood open, restart, hood
/// is still open" is reproducible.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct HoodState {
    pub is_open: bool,
}

/// Persisted state for the sunroof plant model — physical positions
/// of the glass panel and the shade.  Both are u8 percentages
/// `[0..=100]` (0 = fully closed, 100 = fully open).
///
/// Cold boot (no file) = factory: both fully closed.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct SunroofState {
    pub position: u8,
    pub shade_position: u8,
}

/// Persisted vehicle-level central lock status.
///
/// Reflects the *commanded* state set by the door-lock arbiter (RKE,
/// PEPS, AutoLock, etc.) and is independent of soldier-knob movements.
/// Used by features that need to reason about the vehicle's lock
/// posture as a whole (MirrorFold AUTO triggers, future security
/// features).
///
/// `status` is one of `'UNLOCKED' | 'DRIVER_UNLOCKED' | 'LOCKED' |
/// 'DOUBLE_LOCKED'`.  Stored as a string to keep the wire format
/// human-readable and stable across enum reordering.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CabinLockStatusState {
    pub status: String,
}

impl Default for CabinLockStatusState {
    fn default() -> Self {
        // Factory default: vehicle delivered unlocked.
        Self {
            status: "UNLOCKED".into(),
        }
    }
}

/// Persisted state for the mirror-fold plant model — physical position
/// of each side mirror.  Separate from feature-level intent
/// (see [`MirrorFoldIntent`]) so the plant stays purely about physics:
/// two positions, no policy.
///
/// Cold boot (no file) = factory: both unfolded.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct MirrorFoldState {
    /// `[left, right]`.  True = folded.
    pub is_folded: [bool; 2],
}

/// Persisted state for the MirrorFold *feature* — last commanded
/// fold direction.  The next manual press of `Body.Switches.Mirror.Fold`
/// commands `!last_fold_cmd` (intended state = inverse of last
/// command), so a power cycle keeps the toggle direction consistent
/// with what the driver last asked for.
///
/// Cold boot (no file) = factory: `last_fold_cmd = false (unfold)`,
/// so the very first press will command a fold.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct MirrorFoldIntent {
    pub last_fold_cmd: bool,
}

/// Handle to the on-disk NVM directory.  Cloneable; cheap to pass around.
///
/// Construct via [`NvmStore::from_env`] (production path — reads
/// `VSS_BRIDGE_NVM_PATH`, falls back to `./nvm/`) or
/// [`NvmStore::with_path`] (tests).  Use [`NvmStore::reset`] to wipe a
/// store back to factory state (used by `--reset-nvm`).
#[derive(Debug, Clone)]
pub struct NvmStore {
    dir: PathBuf,
}

impl NvmStore {
    /// Production constructor — reads `$VSS_BRIDGE_NVM_PATH`, defaults
    /// to `./nvm/` relative to the current working directory.  Creates
    /// the directory if it doesn't exist; logs (but does not fail) on
    /// permission errors so callers can still `load_*` (which will see
    /// "missing" → factory default).
    pub fn from_env() -> Self {
        let dir = std::env::var(ENV_NVM_PATH)
            .map(PathBuf::from)
            .unwrap_or_else(|_| PathBuf::from(DEFAULT_NVM_DIR));

        if let Err(e) = fs::create_dir_all(&dir) {
            tracing::warn!(
                path = %dir.display(),
                error = %e,
                "NVM: failed to create directory; loads will fall back to defaults"
            );
        } else {
            tracing::info!(path = %dir.display(), "NVM store ready");
        }

        Self { dir }
    }

    /// Test / explicit-path constructor.
    pub fn with_path<P: AsRef<Path>>(dir: P) -> Self {
        let dir = dir.as_ref().to_path_buf();
        let _ = fs::create_dir_all(&dir);
        Self { dir }
    }

    /// Wipe all NVM files in this store back to "factory new".
    ///
    /// Used by the `--reset-nvm` CLI flag and by tests that want a
    /// guaranteed clean slate.  Errors are logged but not propagated —
    /// at worst the next load will read the old file.
    pub fn reset(&self) {
        for entry in fs::read_dir(&self.dir).into_iter().flatten().flatten() {
            let path = entry.path();
            if path.extension().is_some_and(|e| e == "json") {
                if let Err(e) = fs::remove_file(&path) {
                    tracing::warn!(
                        path = %path.display(),
                        error = %e,
                        "NVM reset: failed to remove file"
                    );
                }
            }
        }
        tracing::info!(path = %self.dir.display(), "NVM store reset");
    }

    /// Load `DoorLockState` from disk.  Returns `Default::default()` on
    /// missing or corrupt file.  See module doc for failure semantics.
    pub fn load_door_lock(&self) -> DoorLockState {
        self.load("door_lock.json")
    }

    /// Atomically persist `DoorLockState` to disk.
    ///
    /// Uses the standard write-temp-then-rename pattern so a power cut
    /// during the write cannot leave a partially written file.  Errors
    /// are logged but not propagated — runtime state is authoritative
    /// either way.
    pub fn save_door_lock(&self, state: &DoorLockState) {
        self.save("door_lock.json", state);
    }

    /// Load `TrunkState` from disk.  Same fallback semantics as
    /// `load_door_lock`.
    pub fn load_trunk(&self) -> TrunkState {
        self.load("trunk.json")
    }

    /// Atomically persist `TrunkState` to disk.
    pub fn save_trunk(&self, state: &TrunkState) {
        self.save("trunk.json", state);
    }

    /// Load `HoodState` from disk.  Factory default = closed.
    pub fn load_hood(&self) -> HoodState {
        self.load("hood.json")
    }

    /// Atomically persist `HoodState` to disk.
    pub fn save_hood(&self, state: &HoodState) {
        self.save("hood.json", state);
    }

    /// Load `SunroofState` from disk.  Factory default = both closed (0%).
    pub fn load_sunroof(&self) -> SunroofState {
        self.load("sunroof.json")
    }

    /// Atomically persist `SunroofState` to disk.
    pub fn save_sunroof(&self, state: &SunroofState) {
        self.save("sunroof.json", state);
    }

    /// Load central lock status from disk.  Factory default = UNLOCKED.
    pub fn load_cabin_lock_status(&self) -> CabinLockStatusState {
        self.load("cabin_lock_status.json")
    }

    /// Atomically persist central lock status.
    pub fn save_cabin_lock_status(&self, state: &CabinLockStatusState) {
        self.save("cabin_lock_status.json", state);
    }

    /// Load mirror-fold plant state.  Factory default = both unfolded,
    /// `last_fold_cmd = unfold`.
    pub fn load_mirror_fold(&self) -> MirrorFoldState {
        self.load("mirror_fold.json")
    }

    /// Atomically persist mirror-fold plant state.
    pub fn save_mirror_fold(&self, state: &MirrorFoldState) {
        self.save("mirror_fold.json", state);
    }

    /// Load MirrorFold feature intent (last commanded fold direction).
    pub fn load_mirror_fold_intent(&self) -> MirrorFoldIntent {
        self.load("mirror_fold_intent.json")
    }

    /// Atomically persist MirrorFold feature intent.
    pub fn save_mirror_fold_intent(&self, state: &MirrorFoldIntent) {
        self.save("mirror_fold_intent.json", state);
    }

    // ── Internals ──────────────────────────────────────────────────────

    fn load<T: for<'de> Deserialize<'de> + Default>(&self, name: &str) -> T {
        let path = self.dir.join(name);
        match fs::read_to_string(&path) {
            Ok(contents) => match serde_json::from_str::<T>(&contents) {
                Ok(value) => {
                    tracing::info!(file = %path.display(), "NVM: loaded persisted state");
                    value
                }
                Err(e) => {
                    tracing::warn!(
                        file = %path.display(),
                        error = %e,
                        "NVM: file present but malformed — falling back to factory default"
                    );
                    T::default()
                }
            },
            Err(e) if e.kind() == io::ErrorKind::NotFound => {
                tracing::info!(
                    file = %path.display(),
                    "NVM: file missing — using factory default"
                );
                T::default()
            }
            Err(e) => {
                tracing::warn!(
                    file = %path.display(),
                    error = %e,
                    "NVM: read error — falling back to factory default"
                );
                T::default()
            }
        }
    }

    fn save<T: Serialize>(&self, name: &str, value: &T) {
        let path = self.dir.join(name);
        let tmp = self.dir.join(format!("{name}.tmp"));

        let json = match serde_json::to_string_pretty(value) {
            Ok(s) => s,
            Err(e) => {
                tracing::error!(file = %path.display(), error = %e, "NVM: serialize failed");
                return;
            }
        };

        // Write to tmp, fsync, rename — atomic on POSIX.
        let res = (|| -> io::Result<()> {
            let mut f = fs::File::create(&tmp)?;
            f.write_all(json.as_bytes())?;
            f.sync_all()?;
            fs::rename(&tmp, &path)?;
            Ok(())
        })();

        if let Err(e) = res {
            tracing::error!(
                file = %path.display(),
                error = %e,
                "NVM: save failed — runtime state unchanged"
            );
            // Best-effort cleanup of leftover tmp file.
            let _ = fs::remove_file(&tmp);
        }
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    fn store() -> (NvmStore, TempDir) {
        let dir = TempDir::new().expect("tempdir");
        let nvm = NvmStore::with_path(dir.path());
        (nvm, dir)
    }

    #[test]
    fn missing_file_returns_default() {
        let (nvm, _g) = store();
        let s = nvm.load_door_lock();
        assert_eq!(s, DoorLockState::default());
        assert_eq!(s.locked, [false; 4]);
    }

    #[test]
    fn save_then_load_roundtrip() {
        let (nvm, _g) = store();
        let want = DoorLockState {
            locked: [true, true, false, true],
            double_locked: [false, true, false, false],
        };
        nvm.save_door_lock(&want);
        let got = nvm.load_door_lock();
        assert_eq!(got, want);
    }

    #[test]
    fn trunk_state_roundtrip() {
        let (nvm, _g) = store();
        // Default = closed.
        assert!(!nvm.load_trunk().is_open);

        nvm.save_trunk(&TrunkState { is_open: true });
        assert!(nvm.load_trunk().is_open);

        nvm.save_trunk(&TrunkState { is_open: false });
        assert!(!nvm.load_trunk().is_open);
    }

    #[test]
    fn door_lock_and_trunk_files_are_independent() {
        let (nvm, dir) = store();
        nvm.save_door_lock(&DoorLockState {
            locked: [true; 4],
            ..Default::default()
        });
        nvm.save_trunk(&TrunkState { is_open: true });

        // Both files exist independently.
        assert!(dir.path().join("door_lock.json").exists());
        assert!(dir.path().join("trunk.json").exists());

        // Loading one doesn't affect the other.
        assert_eq!(nvm.load_door_lock().locked, [true; 4]);
        assert!(nvm.load_trunk().is_open);
    }

    #[test]
    fn corrupt_file_returns_default_with_warn() {
        let (nvm, dir) = store();
        std::fs::write(dir.path().join("door_lock.json"), b"not valid json").unwrap();
        let got = nvm.load_door_lock();
        assert_eq!(got, DoorLockState::default());
    }

    #[test]
    fn reset_wipes_files() {
        let (nvm, dir) = store();
        let want = DoorLockState {
            locked: [true; 4],
            double_locked: [true; 4],
        };
        nvm.save_door_lock(&want);
        assert!(dir.path().join("door_lock.json").exists());

        nvm.reset();
        assert!(!dir.path().join("door_lock.json").exists());
        assert_eq!(nvm.load_door_lock(), DoorLockState::default());
    }

    #[test]
    fn save_is_atomic_via_tmp_then_rename() {
        // We can't easily simulate a crash mid-write in a unit test, but
        // we can verify the implementation's intermediate state: after a
        // successful save, no .tmp file should remain.
        let (nvm, dir) = store();
        nvm.save_door_lock(&DoorLockState {
            locked: [true, false, true, false],
            ..Default::default()
        });
        assert!(dir.path().join("door_lock.json").exists());
        assert!(!dir.path().join("door_lock.json.tmp").exists());
    }

    #[test]
    fn env_var_overrides_default_path() {
        let dir = TempDir::new().unwrap();
        std::env::set_var(ENV_NVM_PATH, dir.path());
        let nvm = NvmStore::from_env();
        std::env::remove_var(ENV_NVM_PATH);
        // Save and confirm file lands inside the temp dir, not ./nvm/.
        nvm.save_door_lock(&DoorLockState::default());
        assert!(dir.path().join("door_lock.json").exists());
    }
}
