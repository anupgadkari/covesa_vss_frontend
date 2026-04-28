//! MirrorFold — folds / unfolds the side mirrors on driver action or
//! lock-state transition.
//!
//! # Inputs
//!
//! - **`Body.Switches.Mirror.Fold`** — momentary bool.  Each rising
//!   edge (`false → true`) is a manual fold-toggle press.
//! - **`Cabin.LockStatus`** — vehicle-level lock state published by
//!   the door-lock arbiter.  AUTO mode triggers on transitions:
//!   - entering `LOCKED` or `DOUBLE_LOCKED` → fold both mirrors
//!   - entering `UNLOCKED` or `DRIVER_UNLOCKED` → unfold both mirrors
//! - **`Body.Mirror.Left.IsFolded` / `Body.Mirror.Right.IsFolded`** —
//!   per-side feedback from the plant model, used to apply Option-A
//!   mismatch resolution (only command sides whose feedback ≠ target).
//!
//! # Outputs
//!
//! - **`Body.Mirror.Left.FoldCmd` / `Body.Mirror.Right.FoldCmd`** —
//!   bool commands consumed by the plant model.
//!
//! # Mode (dealer cal `dealer.mirror_fold_mode`)
//!
//! - `MANUAL` (default): switch press toggles, no AUTO behaviour.
//! - `AUTO`: switch toggles AND lock-state edges drive folds.
//!   Manual press always toggles even in AUTO.
//! - `OFF`: feature inert (no switch, no AUTO).  Used on trims that
//!   ship without the fold motor.
//!
//! # Toggle semantics
//!
//! `intended_fold_state` is computed as `!last_fold_cmd` where
//! `last_fold_cmd` survives power cycles in
//! [`crate::nvm::MirrorFoldIntent`].  Each accepted trigger writes
//! the new target to NVM before issuing per-side commands.
//!
//! # Mismatch handling (Option A)
//!
//! When the two sides disagree at trigger time (e.g. partial motor
//! failure on a previous cycle, or restored mismatched NVM), only the
//! side(s) whose current feedback ≠ target are commanded.  The other
//! side is already at the target and stays put.

use std::sync::Arc;

use futures::stream::BoxStream;
use futures::StreamExt;
use tokio::select;

use crate::config::{MirrorFoldMode, PlatformConfig};
use crate::ipc_message::{FeatureId, SignalValue};
use crate::nvm::{MirrorFoldIntent, NvmStore};
use crate::signal_bus::{SignalBus, VssPath};

#[allow(dead_code)]
const FEATURE_ID: FeatureId = FeatureId::MirrorFold;

const SWITCH_FOLD: VssPath = "Body.Switches.Mirror.Fold";
const LOCK_STATUS: VssPath = "Cabin.LockStatus";

const FB_LEFT: VssPath = "Body.Mirror.Left.IsFolded";
const FB_RIGHT: VssPath = "Body.Mirror.Right.IsFolded";
const CMD_LEFT: VssPath = "Body.Mirror.Left.FoldCmd";
const CMD_RIGHT: VssPath = "Body.Mirror.Right.FoldCmd";

/// Map a `Cabin.LockStatus` value to the AUTO target it should drive,
/// if any.  Returns `Some(true)` for fold, `Some(false)` for unfold,
/// `None` if the status doesn't trigger AUTO action.
fn auto_target_for(status: &str) -> Option<bool> {
    match status {
        "LOCKED" | "DOUBLE_LOCKED" => Some(true),  // fold
        "UNLOCKED" | "DRIVER_UNLOCKED" => Some(false), // unfold
        _ => None,
    }
}

pub struct MirrorFold<B: SignalBus> {
    bus: Arc<B>,
    nvm: Option<NvmStore>,
    cfg: Arc<PlatformConfig>,
    /// Most recent commanded fold direction.  Sourced from NVM at
    /// boot; updated on every accepted trigger.
    last_fold_cmd: bool,
    /// Per-side feedback cache.  `[left, right]`.  Updated as the
    /// plant model publishes state.
    feedback: [bool; 2],
    /// Latest seen lock status.  Used to detect transitions for AUTO.
    last_lock_status: String,
    /// Latest seen switch state.  Used to detect rising edges
    /// (false→true).  Default false.
    last_switch_state: bool,
}

impl<B: SignalBus + Send + Sync + 'static> MirrorFold<B> {
    pub fn new(bus: Arc<B>, cfg: Arc<PlatformConfig>) -> Self {
        Self {
            bus,
            nvm: None,
            cfg,
            last_fold_cmd: false,
            feedback: [false, false],
            last_lock_status: "UNLOCKED".into(),
            last_switch_state: false,
        }
    }

    /// Production constructor — restores `last_fold_cmd` from NVM and
    /// persists on every accepted trigger.
    pub fn with_nvm(bus: Arc<B>, cfg: Arc<PlatformConfig>, nvm: NvmStore) -> Self {
        let intent = nvm.load_mirror_fold_intent();
        tracing::info!(
            last_fold_cmd = intent.last_fold_cmd,
            "MirrorFold: restored intent from NVM"
        );
        Self {
            bus,
            nvm: Some(nvm),
            cfg,
            last_fold_cmd: intent.last_fold_cmd,
            feedback: [false, false],
            last_lock_status: "UNLOCKED".into(),
            last_switch_state: false,
        }
    }

    fn mode(&self) -> MirrorFoldMode {
        self.cfg.dealer_config().mirror_fold_mode
    }

    fn save_intent(&self) {
        if let Some(nvm) = &self.nvm {
            nvm.save_mirror_fold_intent(&MirrorFoldIntent {
                last_fold_cmd: self.last_fold_cmd,
            });
        }
    }

    /// Issue per-side commands using Option A: command only sides
    /// whose feedback ≠ target.  Updates `last_fold_cmd` + NVM
    /// regardless of how many sides actually move (so the *intent*
    /// is captured even when the mechanical state already matches).
    async fn apply_target(&mut self, target: bool, source: &'static str) {
        let need_left = self.feedback[0] != target;
        let need_right = self.feedback[1] != target;

        tracing::info!(
            target,
            source,
            need_left,
            need_right,
            "MirrorFold: applying target"
        );

        if need_left {
            let _ = self.bus.publish(CMD_LEFT, SignalValue::Bool(target)).await;
        }
        if need_right {
            let _ = self.bus.publish(CMD_RIGHT, SignalValue::Bool(target)).await;
        }

        if self.last_fold_cmd != target {
            self.last_fold_cmd = target;
            self.save_intent();
        }
    }

    pub async fn run(mut self) {
        tracing::info!(
            mode = ?self.mode(),
            last_fold_cmd = self.last_fold_cmd,
            "MirrorFold feature started"
        );

        let mut switch_rx: BoxStream<'static, SignalValue> =
            self.bus.subscribe(SWITCH_FOLD).await;
        let mut lock_rx: BoxStream<'static, SignalValue> =
            self.bus.subscribe(LOCK_STATUS).await;
        let mut fb_left_rx: BoxStream<'static, SignalValue> = self.bus.subscribe(FB_LEFT).await;
        let mut fb_right_rx: BoxStream<'static, SignalValue> = self.bus.subscribe(FB_RIGHT).await;

        loop {
            select! {
                Some(v) = switch_rx.next() => {
                    let new_state = matches!(v, SignalValue::Bool(true));
                    let rising = !self.last_switch_state && new_state;
                    self.last_switch_state = new_state;
                    if !rising { continue; }
                    if self.mode() == MirrorFoldMode::Off {
                        tracing::debug!("MirrorFold: switch press ignored — mode=OFF");
                        continue;
                    }
                    let target = !self.last_fold_cmd;
                    self.apply_target(target, "manual_switch").await;
                }
                Some(v) = lock_rx.next() => {
                    let new_status = match v {
                        SignalValue::String(s) => s,
                        _ => continue,
                    };
                    let prev = std::mem::replace(&mut self.last_lock_status, new_status.clone());
                    if prev == new_status { continue; }
                    if self.mode() != MirrorFoldMode::Auto {
                        continue;
                    }
                    let Some(target) = auto_target_for(&new_status) else { continue; };
                    self.apply_target(target, "lock_status_edge").await;
                }
                Some(v) = fb_left_rx.next() => {
                    if let SignalValue::Bool(b) = v { self.feedback[0] = b; }
                }
                Some(v) = fb_right_rx.next() => {
                    if let SignalValue::Bool(b) = v { self.feedback[1] = b; }
                }
                else => break,
            }
        }
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::config::PlatformConfig;
    use std::time::Duration;
    use tempfile::tempdir;

    async fn settle_yields() {
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    fn cfg_with_mode(mode: MirrorFoldMode) -> Arc<PlatformConfig> {
        let cfg = PlatformConfig::defaults();
        let mut dc = cfg.dealer_config();
        dc.mirror_fold_mode = mode;
        cfg.update_dealer_config(dc);
        cfg
    }

    /// Helper: spin up bus + feature with a given mode.  Returns the
    /// bus so tests can inject and observe.
    async fn setup(mode: MirrorFoldMode) -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let cfg = cfg_with_mode(mode);
        let feat = MirrorFold::new(Arc::clone(&bus), cfg);
        tokio::spawn(feat.run());
        settle_yields().await;
        bus
    }

    /// Press the momentary switch (rising edge then falling edge).
    fn press(bus: &Arc<MockBus>) {
        bus.inject(SWITCH_FOLD, SignalValue::Bool(true));
        bus.inject(SWITCH_FOLD, SignalValue::Bool(false));
    }

    // ── Manual mode ──────────────────────────────────────────────────

    #[tokio::test]
    async fn manual_press_commands_both_sides_to_fold_when_both_unfolded() {
        let bus = setup(MirrorFoldMode::Manual).await;
        press(&bus);
        settle_yields().await;
        // Default last_fold_cmd = false → first press commands fold.
        assert_eq!(bus.latest_value(CMD_LEFT), Some(SignalValue::Bool(true)));
        assert_eq!(bus.latest_value(CMD_RIGHT), Some(SignalValue::Bool(true)));
    }

    #[tokio::test]
    async fn manual_press_toggles_back_to_unfold() {
        let bus = setup(MirrorFoldMode::Manual).await;

        // Press 1: command fold; simulate plant landing both folded.
        press(&bus);
        settle_yields().await;
        bus.inject(FB_LEFT, SignalValue::Bool(true));
        bus.inject(FB_RIGHT, SignalValue::Bool(true));
        settle_yields().await;

        // Press 2: should command unfold on both.
        press(&bus);
        settle_yields().await;
        assert_eq!(bus.latest_value(CMD_LEFT), Some(SignalValue::Bool(false)));
        assert_eq!(bus.latest_value(CMD_RIGHT), Some(SignalValue::Bool(false)));
    }

    #[tokio::test]
    async fn falling_edge_alone_does_not_trigger() {
        let bus = setup(MirrorFoldMode::Manual).await;
        // Press the switch (rising edge).
        bus.inject(SWITCH_FOLD, SignalValue::Bool(true));
        settle_yields().await;
        let after_press = bus.latest_value(CMD_LEFT);
        // Drop history baseline.  A second false→false isn't an edge.
        bus.inject(SWITCH_FOLD, SignalValue::Bool(false));
        settle_yields().await;
        // Still showing the press's command — falling edge is a no-op.
        assert_eq!(bus.latest_value(CMD_LEFT), after_press);
    }

    // ── AUTO mode ────────────────────────────────────────────────────

    #[tokio::test]
    async fn auto_lock_edge_folds_mirrors() {
        let bus = setup(MirrorFoldMode::Auto).await;
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        settle_yields().await;
        assert_eq!(bus.latest_value(CMD_LEFT), Some(SignalValue::Bool(true)));
        assert_eq!(bus.latest_value(CMD_RIGHT), Some(SignalValue::Bool(true)));
    }

    #[tokio::test]
    async fn auto_unlock_edge_unfolds_mirrors() {
        let bus = setup(MirrorFoldMode::Auto).await;
        // Get into LOCKED first so unlock is a real transition.
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        settle_yields().await;
        bus.inject(FB_LEFT, SignalValue::Bool(true));
        bus.inject(FB_RIGHT, SignalValue::Bool(true));
        settle_yields().await;

        bus.inject(LOCK_STATUS, SignalValue::String("DRIVER_UNLOCKED".into()));
        settle_yields().await;
        assert_eq!(bus.latest_value(CMD_LEFT), Some(SignalValue::Bool(false)));
        assert_eq!(bus.latest_value(CMD_RIGHT), Some(SignalValue::Bool(false)));
    }

    #[tokio::test]
    async fn auto_repeating_lock_status_does_not_re_trigger() {
        let bus = setup(MirrorFoldMode::Auto).await;
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        settle_yields().await;
        // Plant landed.
        bus.inject(FB_LEFT, SignalValue::Bool(true));
        bus.inject(FB_RIGHT, SignalValue::Bool(true));
        settle_yields().await;

        // Counts of fold commands so far.
        let count_before = bus
            .history()
            .into_iter()
            .filter(|(s, _)| *s == CMD_LEFT)
            .count();

        // Re-publish same status — must be a no-op.
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        settle_yields().await;

        let count_after = bus
            .history()
            .into_iter()
            .filter(|(s, _)| *s == CMD_LEFT)
            .count();
        assert_eq!(count_before, count_after);
    }

    #[tokio::test]
    async fn auto_manual_press_still_works() {
        let bus = setup(MirrorFoldMode::Auto).await;
        // Manual press without any lock activity → first press = fold.
        press(&bus);
        settle_yields().await;
        assert_eq!(bus.latest_value(CMD_LEFT), Some(SignalValue::Bool(true)));
    }

    // ── OFF mode ─────────────────────────────────────────────────────

    #[tokio::test]
    async fn off_mode_ignores_switch_press() {
        let bus = setup(MirrorFoldMode::Off).await;
        press(&bus);
        settle_yields().await;
        assert_eq!(bus.latest_value(CMD_LEFT), None);
        assert_eq!(bus.latest_value(CMD_RIGHT), None);
    }

    #[tokio::test]
    async fn off_mode_ignores_lock_status_edges() {
        let bus = setup(MirrorFoldMode::Off).await;
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        settle_yields().await;
        assert_eq!(bus.latest_value(CMD_LEFT), None);
    }

    // ── Mismatch (Option A) ──────────────────────────────────────────

    #[tokio::test]
    async fn mismatch_only_commands_the_outlier_side() {
        let bus = setup(MirrorFoldMode::Manual).await;

        // Pre-set feedback to mismatched state: left=folded, right=unfolded.
        bus.inject(FB_LEFT, SignalValue::Bool(true));
        bus.inject(FB_RIGHT, SignalValue::Bool(false));
        settle_yields().await;

        // last_fold_cmd default = false (unfold).  intended next = true (fold).
        // Left already folded; right needs to fold → only CMD_RIGHT fires.
        press(&bus);
        settle_yields().await;

        // CMD_LEFT should have NO entry in history (no command issued).
        let left_cmds: Vec<_> = bus
            .history()
            .into_iter()
            .filter(|(s, _)| *s == CMD_LEFT)
            .collect();
        assert!(
            left_cmds.is_empty(),
            "left already at target; must not be commanded"
        );

        assert_eq!(
            bus.latest_value(CMD_RIGHT),
            Some(SignalValue::Bool(true)),
            "right must be commanded toward target=fold"
        );
    }

    // ── NVM persistence of intent ────────────────────────────────────

    #[tokio::test]
    async fn last_fold_cmd_persists_to_nvm() {
        let dir = tempdir().unwrap();
        let nvm = NvmStore::with_path(dir.path());
        let bus = Arc::new(MockBus::new());
        let cfg = cfg_with_mode(MirrorFoldMode::Manual);
        let feat = MirrorFold::with_nvm(Arc::clone(&bus), cfg, nvm.clone());
        tokio::spawn(feat.run());
        settle_yields().await;

        press(&bus);
        settle_yields().await;
        assert_eq!(nvm.load_mirror_fold_intent().last_fold_cmd, true);
    }

    #[tokio::test]
    async fn second_press_after_nvm_restore_unfolds() {
        let dir = tempdir().unwrap();
        let nvm = NvmStore::with_path(dir.path());

        // Pre-seed intent: last command was fold.
        nvm.save_mirror_fold_intent(&MirrorFoldIntent { last_fold_cmd: true });

        let bus = Arc::new(MockBus::new());
        let cfg = cfg_with_mode(MirrorFoldMode::Manual);
        let feat = MirrorFold::with_nvm(Arc::clone(&bus), cfg, nvm);
        tokio::spawn(feat.run());
        settle_yields().await;

        // Tell feature both sides are folded (post-restore).
        bus.inject(FB_LEFT, SignalValue::Bool(true));
        bus.inject(FB_RIGHT, SignalValue::Bool(true));
        settle_yields().await;

        // Next press should command UNFOLD (intended = !true = false).
        press(&bus);
        settle_yields().await;
        assert_eq!(bus.latest_value(CMD_LEFT), Some(SignalValue::Bool(false)));
        assert_eq!(bus.latest_value(CMD_RIGHT), Some(SignalValue::Bool(false)));
    }
}
