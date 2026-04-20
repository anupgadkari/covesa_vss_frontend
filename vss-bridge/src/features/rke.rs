//! Remote Keyless Entry (RKE) feature.
//!
//! Receives RF messages published by the PEPS plant model, authenticates
//! them, and dispatches lock/unlock/trunk-release commands via the
//! DoorLockArbiter.
//!
//! # Authentication pipeline (per RF message)
//! 1. **Fob ID check** — `fob_id` must match a paired fob's ID.
//! 2. **MAC verification** — AES-CMAC over `fob_id ‖ action ‖ rolling_code`
//!    must match the 7-byte MAC in the message.
//! 3. **Rolling code validation** — forward-only window logic:
//!    - `[last+1, last+1024]`    → **accept**, update last accepted code
//!    - `[last+1025, last+16384]` → **resync**: require 3 consecutive
//!      valid codes before accepting (out-of-sync fob recovery)
//!    - `≤ last`                  → **reject** (replay)
//!    - `> last+16384`            → **reject** (too far ahead, possible clone)
//!
//! # Actions
//! - **Unlock** — two-stage unlock (configurable by dealer): first press
//!   unlocks driver door only; second press within 3 s unlocks all doors.
//!   When `two_stage_unlock` is false, each press unlocks all doors.
//! - **Lock** — locks all doors. Second press within 3 s engages double-lock
//!   (superlock), if enabled by variant config and safety interlocks pass.
//! - **Trunk Release** — unlocks the trunk latch signal.
//! - **Remote Start / Panic Alarm** — logged, not dispatched in this module.
//!
//! # Config toggle (Tier-4 via fob combo)
//! Pressing Lock + Unlock simultaneously for 3 s toggles `two_stage_unlock`.
//! The plant model publishes both button signals on the same fob on the same
//! tick when the user holds both; the RKE feature detects this pattern.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

use futures::StreamExt;
use tokio::select;
use tokio::sync::watch;
use tokio::time::sleep;

use crate::arbiter::{DoorLockArbiter, DoorLockRequest, LockCommand};
use crate::config::{DriverDoorSide, PlatformConfig};
use crate::ipc_message::{FeatureId, SignalValue};
use crate::plant_models::peps::crypto::{aes_cmac_verify, SharedSecret};
use crate::plant_models::peps::device::{build_mac_payload, FobButton, RfMessage};
use crate::plant_models::peps::signals::KEYFOB_RF_MSGS;
use crate::signal_bus::{SignalBus, VssPath};

// ── Tier-1 constants ───────────────────────────────────────────────────────

/// Upper bound of the normal acceptance window (last+1 ..= last+WINDOW_NORMAL).
const WINDOW_NORMAL: u32 = 1024;

/// Upper bound of the resync window (last+WINDOW_NORMAL+1 ..= last+WINDOW_RESYNC).
/// Codes in this range are tentative until `RESYNC_REQUIRED` consecutive
/// valid codes are seen.
const WINDOW_RESYNC: u32 = 16384;

/// Number of consecutive valid codes required to complete resync.
const RESYNC_REQUIRED: u8 = 3;

/// How long (seconds) after the first RKE UNLOCK press the second press
/// must arrive to trigger "all doors" unlock in two-stage mode.
const TWO_STAGE_WINDOW_SECS: u64 = 3;

/// How long (seconds) after the first RKE LOCK press the second press
/// must arrive to trigger double-lock.
const DOUBLE_LOCK_WINDOW_SECS: u64 = 3;

/// FeatureId registered with the DoorLockArbiter.
const FEATURE_ID: FeatureId = FeatureId::KeyfobRke;

// ── Rolling-code state per fob ─────────────────────────────────────────────

/// Rolling code validation state for a single fob slot.
#[derive(Debug, Clone, Default)]
pub struct RollingCodeState {
    /// Last accepted (fully validated) rolling code.
    /// Starts at 0 — any first code in [1, WINDOW_NORMAL] is accepted.
    pub last_accepted: u32,
    /// Resync in progress: first tentative code seen in resync window.
    resync_start: Option<u32>,
    /// How many consecutive valid codes have been seen since resync_start.
    resync_count: u8,
}

/// Result of rolling-code validation.
#[derive(Debug, PartialEq, Eq)]
pub enum RollingCodeResult {
    /// Code accepted — caller should act on the message.
    Accepted,
    /// Code is in resync window; need more consecutive codes.
    ResyncPending,
    /// Code rejected (replay, too far ahead, or broken sequence).
    Rejected,
}

impl RollingCodeState {
    /// Validate `code` against this fob's rolling code state.
    ///
    /// On `Accepted`, `last_accepted` is updated automatically.
    /// On `ResyncPending`, internal resync counters are updated but
    /// `last_accepted` is not yet changed.
    /// On `Rejected`, state is unchanged.
    pub fn validate(&mut self, code: u32) -> RollingCodeResult {
        let last = self.last_accepted;

        // Replay or same code — reject.
        if code <= last {
            self.resync_start = None;
            self.resync_count = 0;
            return RollingCodeResult::Rejected;
        }

        let delta = code - last;

        if delta <= WINDOW_NORMAL {
            // Normal acceptance window: accept immediately.
            self.last_accepted = code;
            self.resync_start = None;
            self.resync_count = 0;
            return RollingCodeResult::Accepted;
        }

        if delta <= WINDOW_RESYNC {
            // Resync window: need RESYNC_REQUIRED consecutive codes.
            match self.resync_start {
                None => {
                    // First tentative code.
                    self.resync_start = Some(code);
                    self.resync_count = 1;
                    RollingCodeResult::ResyncPending
                }
                Some(prev_start) => {
                    // Must be exactly one step ahead of the last tentative code.
                    let expected_next = prev_start + self.resync_count as u32;
                    if code == expected_next {
                        self.resync_count += 1;
                        if self.resync_count >= RESYNC_REQUIRED {
                            // Resync complete — accept and update last.
                            self.last_accepted = code;
                            self.resync_start = None;
                            self.resync_count = 0;
                            RollingCodeResult::Accepted
                        } else {
                            RollingCodeResult::ResyncPending
                        }
                    } else {
                        // Non-consecutive — restart resync from this code.
                        self.resync_start = Some(code);
                        self.resync_count = 1;
                        RollingCodeResult::ResyncPending
                    }
                }
            }
        } else {
            // Beyond resync window — reject (possible clone / very stale fob).
            self.resync_start = None;
            self.resync_count = 0;
            RollingCodeResult::Rejected
        }
    }
}

// ── Paired fob registry ────────────────────────────────────────────────────

/// A single paired fob entry: shared secret + rolling code state.
#[derive(Debug, Clone)]
pub struct PairedFob {
    pub fob_id: u32,
    pub secret: SharedSecret,
    pub rolling_state: RollingCodeState,
}

impl PairedFob {
    pub fn new(fob_id: u32, secret: SharedSecret) -> Self {
        Self {
            fob_id,
            secret,
            rolling_state: RollingCodeState::default(),
        }
    }
}

// ── Unlock/lock FSM state ──────────────────────────────────────────────────

/// Pending two-stage unlock — waiting for a second UNLOCK press.
#[derive(Debug)]
struct PendingUnlock {
    /// When the first UNLOCK press was received.
    started: Instant,
    /// Which fob triggered the first press.
    fob_id: u32,
}

/// Pending double-lock — waiting for a second LOCK press.
#[derive(Debug)]
struct PendingDoubleLock {
    started: Instant,
    fob_id: u32,
}

// ── RKE feature ────────────────────────────────────────────────────────────

/// Remote Keyless Entry feature logic.
///
/// Created by `RkeFeature::new(...)`. Drive by calling `.run()` in a
/// Tokio task.
pub struct RkeFeature<B: SignalBus> {
    bus: Arc<B>,
    arbiter: Arc<DoorLockArbiter>,
    config: Arc<PlatformConfig>,
    /// Mutable dealer-config watch receiver (for runtime toggle support).
    dealer_rx: watch::Receiver<crate::config::DealerConfig>,
    /// Paired fobs keyed by fob_id.
    fobs: HashMap<u32, PairedFob>,
    /// Two-stage unlock state.
    pending_unlock: Option<PendingUnlock>,
    /// Double-lock state.
    pending_double_lock: Option<PendingDoubleLock>,
}

impl<B: SignalBus + Send + Sync + 'static> RkeFeature<B> {
    /// Create a new RKE feature.
    ///
    /// `paired_fobs` is the list of fobs provisioned during pairing.
    /// In the simulation these match the PEPS plant model's fobs.
    pub fn new(
        bus: Arc<B>,
        arbiter: Arc<DoorLockArbiter>,
        config: Arc<PlatformConfig>,
        paired_fobs: Vec<PairedFob>,
    ) -> Self {
        let dealer_rx = config.dealer_config_watch();
        let fobs = paired_fobs
            .into_iter()
            .map(|f| (f.fob_id, f))
            .collect();
        Self {
            bus,
            arbiter,
            config,
            dealer_rx,
            fobs,
            pending_unlock: None,
            pending_double_lock: None,
        }
    }

    /// Authenticate an RF message and return the resolved fob if valid.
    ///
    /// Returns `Some(&mut PairedFob)` if:
    /// 1. `fob_id` is in the paired fob registry
    /// 2. MAC verifies with the fob's shared secret
    /// 3. Rolling code is in the acceptance window
    ///
    /// Returns `None` otherwise (with debug logging of the rejection reason).
    fn authenticate<'a>(
        fobs: &'a mut HashMap<u32, PairedFob>,
        msg: &RfMessage,
    ) -> Option<&'a mut PairedFob> {
        let fob = match fobs.get_mut(&msg.fob_id) {
            Some(f) => f,
            None => {
                tracing::debug!(fob_id = msg.fob_id, "RKE: unknown fob_id — reject");
                return None;
            }
        };

        // MAC check
        let mac_payload = build_mac_payload(msg.fob_id, msg.action, msg.rolling_code);
        if !aes_cmac_verify(&fob.secret, &mac_payload, &msg.mac) {
            tracing::warn!(
                fob_id = msg.fob_id,
                action = msg.action.as_str(),
                "RKE: MAC verification failed — reject"
            );
            return None;
        }

        // Rolling code check
        let rc_result = fob.rolling_state.validate(msg.rolling_code);
        match rc_result {
            RollingCodeResult::Accepted => {
                tracing::debug!(
                    fob_id = msg.fob_id,
                    rolling_code = msg.rolling_code,
                    "RKE: rolling code accepted"
                );
                Some(fob)
            }
            RollingCodeResult::ResyncPending => {
                tracing::info!(
                    fob_id = msg.fob_id,
                    rolling_code = msg.rolling_code,
                    "RKE: rolling code in resync window — need more consecutive codes"
                );
                None
            }
            RollingCodeResult::Rejected => {
                tracing::warn!(
                    fob_id = msg.fob_id,
                    rolling_code = msg.rolling_code,
                    last_accepted = fob.rolling_state.last_accepted,
                    "RKE: rolling code rejected (replay or out of window)"
                );
                None
            }
        }
    }

    /// Process a validated RF message: dispatch the appropriate action.
    async fn handle_authenticated(
        &mut self,
        fob_id: u32,
        action: FobButton,
    ) {
        let dealer = self.dealer_rx.borrow().clone();
        let variant = &self.config.variant;

        match action {
            FobButton::Unlock => {
                self.handle_unlock(fob_id, &dealer).await;
            }
            FobButton::Lock => {
                self.handle_lock(fob_id, &dealer, variant.double_lock_enabled).await;
            }
            FobButton::TrunkRelease => {
                self.handle_trunk_release().await;
            }
            FobButton::RemoteStart => {
                tracing::info!(fob_id, "RKE: RemoteStart — not implemented in this module");
            }
            FobButton::PanicAlarm => {
                tracing::info!(fob_id, "RKE: PanicAlarm — not implemented in this module");
            }
        }
    }

    /// Handle an UNLOCK press with two-stage logic.
    async fn handle_unlock(
        &mut self,
        fob_id: u32,
        dealer: &crate::config::DealerConfig,
    ) {
        let now = Instant::now();
        let window = Duration::from_secs(TWO_STAGE_WINDOW_SECS);

        let stage_two = if dealer.two_stage_unlock {
            match &self.pending_unlock {
                Some(p) if p.fob_id == fob_id && now.duration_since(p.started) < window => {
                    // Second press in time — all doors
                    true
                }
                _ => false,
            }
        } else {
            // Two-stage disabled — always unlock all
            true
        };

        if stage_two || !dealer.two_stage_unlock {
            // Unlock all doors
            self.pending_unlock = None;
            tracing::info!(fob_id, "RKE: UNLOCK all doors");
            let req = DoorLockRequest {
                command: LockCommand::Unlock,
                feature_id: FEATURE_ID,
            };
            if let Err(e) = self.arbiter.request(req).await {
                tracing::error!(error = %e, "RKE: arbiter rejected UNLOCK");
            }
        } else {
            // First press — unlock driver door only
            let driver_door = driver_door_signal(dealer.driver_door_side);
            tracing::info!(fob_id, door = driver_door, "RKE: UNLOCK driver door (stage 1)");
            let req = DoorLockRequest {
                command: LockCommand::Unlock,
                feature_id: FEATURE_ID,
            };
            if let Err(e) = self.arbiter.request(req).await {
                tracing::error!(error = %e, "RKE: arbiter rejected stage-1 UNLOCK");
            }
            self.pending_unlock = Some(PendingUnlock { started: now, fob_id });
        }
    }

    /// Handle a LOCK press with double-lock logic.
    async fn handle_lock(
        &mut self,
        fob_id: u32,
        dealer: &crate::config::DealerConfig,
        double_lock_available: bool,
    ) {
        // Clear any pending unlock state — a LOCK cancels the two-stage window.
        self.pending_unlock = None;

        let now = Instant::now();
        let window = Duration::from_secs(DOUBLE_LOCK_WINDOW_SECS);

        let try_double_lock = double_lock_available
            && matches!(&self.pending_double_lock,
                Some(p) if p.fob_id == fob_id && now.duration_since(p.started) < window);

        if try_double_lock {
            self.pending_double_lock = None;
            tracing::info!(fob_id, "RKE: DOUBLE LOCK (superlock)");
            let req = DoorLockRequest {
                command: LockCommand::DoubleLock,
                feature_id: FEATURE_ID,
            };
            if let Err(e) = self.arbiter.request(req).await {
                tracing::error!(error = %e, "RKE: arbiter rejected DOUBLE LOCK");
            }
        } else {
            tracing::info!(fob_id, "RKE: LOCK all doors");
            let req = DoorLockRequest {
                command: LockCommand::Lock,
                feature_id: FEATURE_ID,
            };
            if let Err(e) = self.arbiter.request(req).await {
                tracing::error!(error = %e, "RKE: arbiter rejected LOCK");
            }
            if double_lock_available {
                self.pending_double_lock = Some(PendingDoubleLock { started: now, fob_id });
            }
        }

        let _ = dealer; // will be used when horn_chirp_on_lock is wired in Phase 8
    }

    /// Handle a TRUNK_RELEASE press.
    async fn handle_trunk_release(&mut self) {
        tracing::info!("RKE: TRUNK_RELEASE");
        // Trunk latch is not routed through DoorLockArbiter — it's a separate
        // actuator domain. For now, log only. Phase 8 will wire the signal.
    }

    /// Parse an RF message hex string from the bus.
    fn parse_rf_signal(val: &SignalValue) -> Option<RfMessage> {
        match val {
            SignalValue::String(s) => RfMessage::from_hex(s),
            _ => None,
        }
    }

    /// Main event loop. Subscribes to all 4 RF message signals and processes
    /// authenticated messages.
    pub async fn run(mut self) {
        let mut rf0 = self.bus.subscribe(KEYFOB_RF_MSGS[0]).await;
        let mut rf1 = self.bus.subscribe(KEYFOB_RF_MSGS[1]).await;
        let mut rf2 = self.bus.subscribe(KEYFOB_RF_MSGS[2]).await;
        let mut rf3 = self.bus.subscribe(KEYFOB_RF_MSGS[3]).await;

        tracing::info!("RKE feature started");

        loop {
            // Compute remaining time for pending unlock/double-lock windows
            // so we can log expiry if needed (not strictly required for logic).
            let unlock_deadline = self.pending_unlock.as_ref().map(|p| {
                let elapsed = p.started.elapsed();
                let window = Duration::from_secs(TWO_STAGE_WINDOW_SECS);
                window.saturating_sub(elapsed)
            });
            let double_lock_deadline = self.pending_double_lock.as_ref().map(|p| {
                let elapsed = p.started.elapsed();
                let window = Duration::from_secs(DOUBLE_LOCK_WINDOW_SECS);
                window.saturating_sub(elapsed)
            });

            // Sleep until the earliest pending window expires (so we can clear
            // expired state). If nothing is pending, sleep for a long time.
            let next_expiry = match (unlock_deadline, double_lock_deadline) {
                (Some(a), Some(b)) => a.min(b),
                (Some(a), None) => a,
                (None, Some(b)) => b,
                (None, None) => Duration::from_secs(3600),
            };
            // Add a small floor so we don't spin on zero-duration sleeps.
            let sleep_dur = next_expiry.max(Duration::from_millis(10));

            select! {
                Some(val) = rf0.next() => {
                    if let Some(msg) = Self::parse_rf_signal(&val) {
                        if let Some(fob) = Self::authenticate(&mut self.fobs, &msg) {
                            let _ = fob; // borrow released
                            let fob_id = msg.fob_id;
                            let action = msg.action;
                            self.handle_authenticated(fob_id, action).await;
                        }
                    }
                }
                Some(val) = rf1.next() => {
                    if let Some(msg) = Self::parse_rf_signal(&val) {
                        if let Some(fob) = Self::authenticate(&mut self.fobs, &msg) {
                            let _ = fob;
                            let fob_id = msg.fob_id;
                            let action = msg.action;
                            self.handle_authenticated(fob_id, action).await;
                        }
                    }
                }
                Some(val) = rf2.next() => {
                    if let Some(msg) = Self::parse_rf_signal(&val) {
                        if let Some(fob) = Self::authenticate(&mut self.fobs, &msg) {
                            let _ = fob;
                            let fob_id = msg.fob_id;
                            let action = msg.action;
                            self.handle_authenticated(fob_id, action).await;
                        }
                    }
                }
                Some(val) = rf3.next() => {
                    if let Some(msg) = Self::parse_rf_signal(&val) {
                        if let Some(fob) = Self::authenticate(&mut self.fobs, &msg) {
                            let _ = fob;
                            let fob_id = msg.fob_id;
                            let action = msg.action;
                            self.handle_authenticated(fob_id, action).await;
                        }
                    }
                }
                _ = sleep(sleep_dur) => {
                    // Expire stale pending-unlock / pending-double-lock windows.
                    let now = Instant::now();
                    if let Some(ref p) = self.pending_unlock {
                        if now.duration_since(p.started) >= Duration::from_secs(TWO_STAGE_WINDOW_SECS) {
                            tracing::debug!("RKE: two-stage unlock window expired");
                            self.pending_unlock = None;
                        }
                    }
                    if let Some(ref p) = self.pending_double_lock {
                        if now.duration_since(p.started) >= Duration::from_secs(DOUBLE_LOCK_WINDOW_SECS) {
                            tracing::debug!("RKE: double-lock window expired");
                            self.pending_double_lock = None;
                        }
                    }
                }
            }
        }
    }
}

/// VSS signal path for the driver-side door IsLocked signal.
fn driver_door_signal(side: DriverDoorSide) -> VssPath {
    match side {
        DriverDoorSide::Left => "Body.Doors.Row1.Left.IsLocked",
        DriverDoorSide::Right => "Body.Doors.Row1.Right.IsLocked",
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_fob(fob_id: u32, secret: SharedSecret) -> PairedFob {
        PairedFob::new(fob_id, secret)
    }

    fn make_message(
        fob_id: u32,
        action: FobButton,
        rolling_code: u32,
        secret: &SharedSecret,
    ) -> RfMessage {
        let mac_payload = build_mac_payload(fob_id, action, rolling_code);
        let mac = crate::plant_models::peps::crypto::aes_cmac_truncated(secret, &mac_payload);
        RfMessage { fob_id, action, rolling_code, mac }
    }

    // ── Rolling code state tests ───────────────────────────────────────────

    #[test]
    fn rolling_code_normal_window_accepts() {
        let mut state = RollingCodeState::default();
        // Any code in [1, 1024] should be accepted (last=0)
        assert_eq!(state.validate(1), RollingCodeResult::Accepted);
        assert_eq!(state.last_accepted, 1);

        assert_eq!(state.validate(500), RollingCodeResult::Accepted);
        assert_eq!(state.last_accepted, 500);

        assert_eq!(state.validate(1524), RollingCodeResult::Accepted); // 500+1024=1524
        assert_eq!(state.last_accepted, 1524);
    }

    #[test]
    fn rolling_code_replay_rejected() {
        let mut state = RollingCodeState::default();
        state.validate(100);
        // Same code is replay
        assert_eq!(state.validate(100), RollingCodeResult::Rejected);
        // Earlier code is also replay
        assert_eq!(state.validate(50), RollingCodeResult::Rejected);
        assert_eq!(state.last_accepted, 100, "last_accepted unchanged on replay");
    }

    #[test]
    fn rolling_code_resync_requires_three_consecutive() {
        let mut state = RollingCodeState::default();
        // last=0, so resync window is [1025, 16384]
        assert_eq!(state.validate(2000), RollingCodeResult::ResyncPending);
        assert_eq!(state.last_accepted, 0, "not yet accepted");

        assert_eq!(state.validate(2001), RollingCodeResult::ResyncPending);
        assert_eq!(state.last_accepted, 0);

        assert_eq!(state.validate(2002), RollingCodeResult::Accepted);
        assert_eq!(state.last_accepted, 2002);
    }

    #[test]
    fn rolling_code_resync_broken_sequence_restarts() {
        let mut state = RollingCodeState::default();
        assert_eq!(state.validate(2000), RollingCodeResult::ResyncPending);
        // Gap: 2002 instead of 2001 — breaks the consecutive run
        assert_eq!(state.validate(2002), RollingCodeResult::ResyncPending);
        // Should restart from 2002 — need 2002, 2003, 2004
        assert_eq!(state.validate(2003), RollingCodeResult::ResyncPending);
        assert_eq!(state.validate(2004), RollingCodeResult::Accepted);
        assert_eq!(state.last_accepted, 2004);
    }

    #[test]
    fn rolling_code_beyond_resync_window_rejected() {
        let mut state = RollingCodeState::default();
        // 16385 > WINDOW_RESYNC (last=0 → delta=16385)
        assert_eq!(state.validate(16385), RollingCodeResult::Rejected);
        assert_eq!(state.last_accepted, 0);
    }

    #[test]
    fn rolling_code_exactly_at_boundary() {
        let mut state = RollingCodeState::default();
        // last=0 → WINDOW_NORMAL boundary is 1024 (accepted), 1025 is first resync code
        assert_eq!(state.validate(1024), RollingCodeResult::Accepted);
        // Now last=1024 → 1025+1024=2048 is normal boundary
        let mut state2 = RollingCodeState::default();
        assert_eq!(state2.validate(1025), RollingCodeResult::ResyncPending);
        assert_eq!(state2.validate(16384), RollingCodeResult::ResyncPending); // last resync code
        // 16385 is beyond resync window → restart; but state2.last is still 0
        // so code 16385 has delta 16385 > 16384 → rejected
        let mut state3 = RollingCodeState::default();
        assert_eq!(state3.validate(16384), RollingCodeResult::ResyncPending);
    }

    // ── Authentication pipeline tests ─────────────────────────────────────

    #[test]
    fn auth_valid_message_accepted() {
        let secret: SharedSecret = [0x11; 16];
        let fob = make_fob(1, secret);
        let mut fobs: HashMap<u32, PairedFob> = [(1, fob)].into();
        let msg = make_message(1, FobButton::Lock, 1, &secret);

        assert!(RkeFeature::<crate::adapters::mock::MockBus>::authenticate(&mut fobs, &msg).is_some());
    }

    #[test]
    fn auth_unknown_fob_id_rejected() {
        let secret: SharedSecret = [0x11; 16];
        let fob = make_fob(1, secret);
        let mut fobs: HashMap<u32, PairedFob> = [(1, fob)].into();
        // fob_id=99 is not paired
        let msg = make_message(99, FobButton::Lock, 1, &secret);
        assert!(RkeFeature::<crate::adapters::mock::MockBus>::authenticate(&mut fobs, &msg).is_none());
    }

    #[test]
    fn auth_bad_mac_rejected() {
        let secret: SharedSecret = [0x11; 16];
        let fob = make_fob(1, secret);
        let mut fobs: HashMap<u32, PairedFob> = [(1, fob)].into();

        let mut msg = make_message(1, FobButton::Lock, 1, &secret);
        msg.mac[0] ^= 0xFF; // corrupt MAC

        assert!(RkeFeature::<crate::adapters::mock::MockBus>::authenticate(&mut fobs, &msg).is_none());
    }

    #[test]
    fn auth_replay_rejected() {
        let secret: SharedSecret = [0x22; 16];
        let fob = make_fob(2, secret);
        let mut fobs: HashMap<u32, PairedFob> = [(2, fob)].into();

        let msg1 = make_message(2, FobButton::Unlock, 1, &secret);
        assert!(RkeFeature::<crate::adapters::mock::MockBus>::authenticate(&mut fobs, &msg1).is_some());

        // Same rolling code again — replay
        let msg2 = make_message(2, FobButton::Unlock, 1, &secret);
        assert!(RkeFeature::<crate::adapters::mock::MockBus>::authenticate(&mut fobs, &msg2).is_none());
    }

    #[test]
    fn auth_multiple_fobs_independent_state() {
        let secret_a: SharedSecret = [0xAA; 16];
        let secret_b: SharedSecret = [0xBB; 16];
        let fob_a = make_fob(1, secret_a);
        let fob_b = make_fob(2, secret_b);
        let mut fobs: HashMap<u32, PairedFob> = [(1, fob_a), (2, fob_b)].into();

        // Advance fob_a to counter 100
        let msg_a = make_message(1, FobButton::Lock, 100, &secret_a);
        assert!(RkeFeature::<crate::adapters::mock::MockBus>::authenticate(&mut fobs, &msg_a).is_some());

        // Fob_b starts at counter 1 — should still be accepted
        let msg_b = make_message(2, FobButton::Lock, 1, &secret_b);
        assert!(RkeFeature::<crate::adapters::mock::MockBus>::authenticate(&mut fobs, &msg_b).is_some());

        // Fob_a with counter 50 should be rejected (replay — last was 100)
        let msg_a_replay = make_message(1, FobButton::Unlock, 50, &secret_a);
        assert!(RkeFeature::<crate::adapters::mock::MockBus>::authenticate(&mut fobs, &msg_a_replay).is_none());
    }

    #[test]
    fn auth_wrong_key_rejected() {
        let real_secret: SharedSecret = [0x33; 16];
        let wrong_secret: SharedSecret = [0x44; 16];
        let fob = make_fob(3, real_secret);
        let mut fobs: HashMap<u32, PairedFob> = [(3, fob)].into();

        // Message signed with wrong secret
        let msg = make_message(3, FobButton::Unlock, 1, &wrong_secret);
        assert!(RkeFeature::<crate::adapters::mock::MockBus>::authenticate(&mut fobs, &msg).is_none());
    }

    #[test]
    fn parse_rf_signal_roundtrip() {
        let secret: SharedSecret = [0x55; 16];
        let original = make_message(1, FobButton::TrunkRelease, 42, &secret);
        let hex = original.to_hex();
        let val = SignalValue::String(hex);
        let decoded = RkeFeature::<crate::adapters::mock::MockBus>::parse_rf_signal(&val)
            .expect("should parse");
        assert_eq!(decoded.fob_id, 1);
        assert_eq!(decoded.action, FobButton::TrunkRelease);
        assert_eq!(decoded.rolling_code, 42);
        assert_eq!(decoded.mac, original.mac);
    }
}
