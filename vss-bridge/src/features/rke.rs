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

use crate::arbiter::{
    ActuatorRequest, DomainArbiter, DoorLockArbiter, DoorLockRequest, LockCommand,
    FEEDBACK_REQUEST, TRUNK_OPEN_CMD,
};
use crate::config::PlatformConfig;
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::plant_models::peps::crypto::{aes_cmac_verify, SharedSecret};
use crate::plant_models::peps::device::{build_mac_payload, FobButton, RfMessage};
use crate::plant_models::peps::signals::KEYFOB_RF_MSGS;
use crate::signal_bus::SignalBus;

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

/// How long (seconds) after the first TRUNK_RELEASE press the second press
/// must arrive to open the trunk.
const TRUNK_RELEASE_WINDOW_SECS: u64 = 3;

/// FeatureId registered with the DoorLockArbiter.
const FEATURE_ID: FeatureId = FeatureId::KeyfobRke;

/// How long (milliseconds) between a LOCK and an UNLOCK press (or vice-versa)
/// for them to be treated as a simultaneous combo for the two_stage_unlock toggle.
/// Two independent presses within this window → toggle; wider gap → normal action.
const COMBO_WINDOW_MS: u64 = 300;

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

/// Pending trunk release — waiting for a second TRUNK_RELEASE press.
#[derive(Debug)]
struct PendingTrunkRelease {
    started: Instant,
    fob_id: u32,
}

/// Tracks the first half of a potential LOCK+UNLOCK combo.
/// If the complementary action arrives within COMBO_WINDOW_MS, the combo
/// fires (toggle two_stage_unlock) instead of the normal action.
#[derive(Debug)]
struct PendingCombo {
    /// Which action was seen first (always Lock or Unlock).
    first_action: FobButton,
    /// When it was received.
    started: Instant,
    /// Which fob sent it.
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
    /// Trunk-open arbiter — RKE TrunkRelease pulses `Body.Trunk.OpenCmd`
    /// through here so the valet-mode `PhysicalGate` and any future
    /// trunk policy gating apply uniformly across all trunk-open
    /// writers (RKE, ExteriorTrunkButton, future phone app).
    trunk_arb: Arc<DomainArbiter>,
    config: Arc<PlatformConfig>,
    /// Mutable dealer-config watch receiver (for runtime toggle support).
    dealer_rx: watch::Receiver<crate::config::DealerConfig>,
    /// Paired fobs keyed by fob_id.
    fobs: HashMap<u32, PairedFob>,
    /// Two-stage unlock state.
    pending_unlock: Option<PendingUnlock>,
    /// Double-lock state.
    pending_double_lock: Option<PendingDoubleLock>,
    /// Trunk double-press state.
    pending_trunk_release: Option<PendingTrunkRelease>,
    /// Pending first half of a LOCK+UNLOCK combo for toggle detection.
    pending_combo: Option<PendingCombo>,
    /// Latched panic-alarm state.  Each authenticated PANIC press flips this
    /// and publishes the new value on Body.Switches.Panic.IsEngaged.
    panic_engaged: bool,
}

impl<B: SignalBus + Send + Sync + 'static> RkeFeature<B> {
    /// Create a new RKE feature.
    ///
    /// `paired_fobs` is the list of fobs provisioned during pairing.
    /// In the simulation these match the PEPS plant model's fobs.
    pub fn new(
        bus: Arc<B>,
        arbiter: Arc<DoorLockArbiter>,
        trunk_arb: Arc<DomainArbiter>,
        config: Arc<PlatformConfig>,
        paired_fobs: Vec<PairedFob>,
    ) -> Self {
        let dealer_rx = config.dealer_config_watch();
        let fobs = paired_fobs.into_iter().map(|f| (f.fob_id, f)).collect();
        Self {
            bus,
            arbiter,
            trunk_arb,
            config,
            dealer_rx,
            fobs,
            pending_unlock: None,
            pending_double_lock: None,
            pending_trunk_release: None,
            pending_combo: None,
            panic_engaged: false,
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
    async fn handle_authenticated(&mut self, fob_id: u32, action: FobButton) {
        // ── Combo detection (LOCK + UNLOCK toggle) ──────────────────────────
        // A LOCK+UNLOCK combo (either order, within COMBO_WINDOW_MS from the
        // same fob) toggles the two_stage_unlock dealer config. Neither action
        // is executed as a normal lock/unlock when combo fires.
        if matches!(action, FobButton::Lock | FobButton::Unlock) {
            let now = Instant::now();
            let combo_window = Duration::from_millis(COMBO_WINDOW_MS);

            let combo_fired = match &self.pending_combo {
                Some(pc)
                    if pc.fob_id == fob_id
                        && pc.first_action != action
                        && now.duration_since(pc.started) < combo_window =>
                {
                    // Complementary action within window from same fob → combo!
                    true
                }
                _ => false,
            };

            if combo_fired {
                self.pending_combo = None;
                self.toggle_two_stage_unlock().await;
                return;
            }

            // No combo yet — record this as the first half and proceed with
            // the normal action (the combo check is opportunistic; if no
            // complement arrives within COMBO_WINDOW_MS, this is a normal press).
            self.pending_combo = Some(PendingCombo {
                first_action: action,
                started: now,
                fob_id,
            });
        } else {
            // Non-Lock/Unlock action clears any pending combo.
            self.pending_combo = None;
        }

        // ── Normal action dispatch ──────────────────────────────────────────
        let dealer = self.dealer_rx.borrow().clone();
        let variant = self.config.variant_cal();

        match action {
            FobButton::Unlock => {
                self.handle_unlock(fob_id, &dealer).await;
            }
            FobButton::Lock => {
                self.handle_lock(fob_id, &dealer, variant.double_lock_enabled)
                    .await;
            }
            FobButton::TrunkRelease => {
                self.handle_trunk_release(fob_id).await;
            }
            FobButton::RemoteStart => {
                tracing::info!(fob_id, "RKE: RemoteStart — not implemented in this module");
            }
            FobButton::PanicAlarm => {
                self.handle_panic(fob_id).await;
            }
        }
    }

    /// Toggle the `two_stage_unlock` dealer config parameter.
    async fn toggle_two_stage_unlock(&self) {
        let mut new_config = self.dealer_rx.borrow().clone();
        new_config.two_stage_unlock = !new_config.two_stage_unlock;
        tracing::info!(
            two_stage_unlock = new_config.two_stage_unlock,
            "RKE: LOCK+UNLOCK combo — toggled two_stage_unlock"
        );
        self.config.update_dealer_config(new_config);
    }

    /// Handle an UNLOCK press with two-stage logic.
    async fn handle_unlock(&mut self, fob_id: u32, dealer: &crate::config::DealerConfig) {
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
            // Unlock all doors (stage 2 or two-stage disabled).
            self.pending_unlock = None;
            tracing::info!(fob_id, "RKE: UNLOCK all doors");
            let req = DoorLockRequest {
                command: LockCommand::UnlockAll,
                feature_id: FEATURE_ID,
            };
            if let Err(e) = self.arbiter.request(req).await {
                tracing::error!(error = %e, "RKE: arbiter rejected UNLOCK ALL");
            }
            let _ = self
                .bus
                .publish(FEEDBACK_REQUEST, SignalValue::String("unlock".into()))
                .await;
        } else {
            // First press — driver door only (stage 1).
            tracing::info!(fob_id, "RKE: UNLOCK driver door (stage 1)");
            let req = DoorLockRequest {
                command: LockCommand::UnlockDriver,
                feature_id: FEATURE_ID,
            };
            if let Err(e) = self.arbiter.request(req).await {
                tracing::error!(error = %e, "RKE: arbiter rejected UNLOCK DRIVER");
            }
            let _ = self
                .bus
                .publish(FEEDBACK_REQUEST, SignalValue::String("unlock".into()))
                .await;
            self.pending_unlock = Some(PendingUnlock {
                started: now,
                fob_id,
            });
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
                command: LockCommand::DoubleLockAll,
                feature_id: FEATURE_ID,
            };
            if let Err(e) = self.arbiter.request(req).await {
                tracing::error!(error = %e, "RKE: arbiter rejected DOUBLE LOCK");
            }
            let _ = self
                .bus
                .publish(FEEDBACK_REQUEST, SignalValue::String("lock".into()))
                .await;
        } else {
            tracing::info!(fob_id, "RKE: LOCK all doors");
            let req = DoorLockRequest {
                command: LockCommand::LockAll,
                feature_id: FEATURE_ID,
            };
            if let Err(e) = self.arbiter.request(req).await {
                tracing::error!(error = %e, "RKE: arbiter rejected LOCK");
            }
            let _ = self
                .bus
                .publish(FEEDBACK_REQUEST, SignalValue::String("lock".into()))
                .await;
            if double_lock_available {
                self.pending_double_lock = Some(PendingDoubleLock {
                    started: now,
                    fob_id,
                });
            }
        }

        let _ = dealer; // will be used when horn_chirp_on_lock is wired in Phase 8
    }

    /// Handle a TRUNK_RELEASE press.
    ///
    /// Requires a double-press within [`TRUNK_RELEASE_WINDOW_SECS`] from the
    /// same fob to open the trunk.  A single press arms the window; the second
    /// press publishes `Body.Trunk.OpenCmd`.  This does not affect cabin door
    /// lock state.
    async fn handle_trunk_release(&mut self, fob_id: u32) {
        let now = Instant::now();
        let window = Duration::from_secs(TRUNK_RELEASE_WINDOW_SECS);

        let second_press = matches!(&self.pending_trunk_release, Some(p) if p.fob_id == fob_id && now.duration_since(p.started) < window);

        if second_press {
            self.pending_trunk_release = None;
            tracing::info!(fob_id, "RKE: TRUNK_RELEASE double-press — opening trunk");
            // Pulse `Body.Trunk.OpenCmd` through the trunk arbiter:
            // request true, then release so the next press can fire
            // again.  The arbiter's ValetGate will silently swallow
            // the request if `Cabin.ValetMode.IsActive` is true.
            let _ = self
                .trunk_arb
                .request(ActuatorRequest {
                    signal: TRUNK_OPEN_CMD,
                    value: SignalValue::Bool(true),
                    priority: Priority::Medium,
                    feature_id: FEATURE_ID,
                })
                .await;
            let _ = self.trunk_arb.release(TRUNK_OPEN_CMD, FEATURE_ID).await;
            // Trunk open is an external unlock — play unlock feedback and arm
            // the trunk-close lock feedback (handled by LockFeedback feature).
            let _ = self
                .bus
                .publish(FEEDBACK_REQUEST, SignalValue::String("trunk_unlock".into()))
                .await;
        } else {
            tracing::info!(
                fob_id,
                "RKE: TRUNK_RELEASE first press — waiting for second"
            );
            self.pending_trunk_release = Some(PendingTrunkRelease {
                started: now,
                fob_id,
            });
        }
    }

    /// Handle a PANIC press from a paired keyfob.
    ///
    /// Each authenticated panic press toggles `Body.Switches.Panic.IsEngaged`.
    /// Engaging the signal arms the PanicAlarm feature (synchronized
    /// indicator blink + horn chirps); disengaging stops it.  This matches
    /// typical OEM behaviour: press once to start, press again to cancel.
    async fn handle_panic(&mut self, fob_id: u32) {
        self.panic_engaged = !self.panic_engaged;
        tracing::info!(
            fob_id,
            engaged = self.panic_engaged,
            "RKE: PANIC press — toggling alarm engaged state"
        );
        let _ = self
            .bus
            .publish(
                "Body.Switches.Panic.IsEngaged",
                crate::ipc_message::SignalValue::Bool(self.panic_engaged),
            )
            .await;
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
        // Mirror Body.Switches.Panic.IsEngaged so the next PANIC press toggles
        // from whatever the bus currently shows — important when PanicAlarm
        // self-cancels on a successful unlock and writes FALSE back to the
        // switch.  Without this, the local latch and the bus could drift.
        let mut panic_rx = self.bus.subscribe("Body.Switches.Panic.IsEngaged").await;

        tracing::info!("RKE feature started");

        loop {
            // Compute remaining time for each pending window.
            let unlock_deadline = self.pending_unlock.as_ref().map(|p| {
                Duration::from_secs(TWO_STAGE_WINDOW_SECS).saturating_sub(p.started.elapsed())
            });
            let double_lock_deadline = self.pending_double_lock.as_ref().map(|p| {
                Duration::from_secs(DOUBLE_LOCK_WINDOW_SECS).saturating_sub(p.started.elapsed())
            });
            let trunk_deadline = self.pending_trunk_release.as_ref().map(|p| {
                Duration::from_secs(TRUNK_RELEASE_WINDOW_SECS).saturating_sub(p.started.elapsed())
            });
            let combo_deadline = self.pending_combo.as_ref().map(|p| {
                Duration::from_millis(COMBO_WINDOW_MS).saturating_sub(p.started.elapsed())
            });

            // Sleep until the earliest pending window expires.
            let next_expiry = [
                unlock_deadline,
                double_lock_deadline,
                trunk_deadline,
                combo_deadline,
            ]
            .into_iter()
            .flatten()
            .min()
            .unwrap_or(Duration::from_secs(3600));
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
                Some(val) = panic_rx.next() => {
                    if let SignalValue::Bool(b) = val {
                        self.panic_engaged = b;
                    }
                }
                _ = sleep(sleep_dur) => {
                    // Expire stale pending windows.
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
                    if let Some(ref p) = self.pending_combo {
                        if now.duration_since(p.started) >= Duration::from_millis(COMBO_WINDOW_MS) {
                            tracing::debug!("RKE: combo window expired — treating first press as normal action");
                            self.pending_combo = None;
                        }
                    }
                }
            }
        }
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
        RfMessage {
            fob_id,
            action,
            rolling_code,
            mac,
        }
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
        assert_eq!(
            state.last_accepted, 100,
            "last_accepted unchanged on replay"
        );
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

        assert!(
            RkeFeature::<crate::adapters::mock::MockBus>::authenticate(&mut fobs, &msg).is_some()
        );
    }

    #[test]
    fn auth_unknown_fob_id_rejected() {
        let secret: SharedSecret = [0x11; 16];
        let fob = make_fob(1, secret);
        let mut fobs: HashMap<u32, PairedFob> = [(1, fob)].into();
        // fob_id=99 is not paired
        let msg = make_message(99, FobButton::Lock, 1, &secret);
        assert!(
            RkeFeature::<crate::adapters::mock::MockBus>::authenticate(&mut fobs, &msg).is_none()
        );
    }

    #[test]
    fn auth_bad_mac_rejected() {
        let secret: SharedSecret = [0x11; 16];
        let fob = make_fob(1, secret);
        let mut fobs: HashMap<u32, PairedFob> = [(1, fob)].into();

        let mut msg = make_message(1, FobButton::Lock, 1, &secret);
        msg.mac[0] ^= 0xFF; // corrupt MAC

        assert!(
            RkeFeature::<crate::adapters::mock::MockBus>::authenticate(&mut fobs, &msg).is_none()
        );
    }

    #[test]
    fn auth_replay_rejected() {
        let secret: SharedSecret = [0x22; 16];
        let fob = make_fob(2, secret);
        let mut fobs: HashMap<u32, PairedFob> = [(2, fob)].into();

        let msg1 = make_message(2, FobButton::Unlock, 1, &secret);
        assert!(
            RkeFeature::<crate::adapters::mock::MockBus>::authenticate(&mut fobs, &msg1).is_some()
        );

        // Same rolling code again — replay
        let msg2 = make_message(2, FobButton::Unlock, 1, &secret);
        assert!(
            RkeFeature::<crate::adapters::mock::MockBus>::authenticate(&mut fobs, &msg2).is_none()
        );
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
        assert!(
            RkeFeature::<crate::adapters::mock::MockBus>::authenticate(&mut fobs, &msg_a).is_some()
        );

        // Fob_b starts at counter 1 — should still be accepted
        let msg_b = make_message(2, FobButton::Lock, 1, &secret_b);
        assert!(
            RkeFeature::<crate::adapters::mock::MockBus>::authenticate(&mut fobs, &msg_b).is_some()
        );

        // Fob_a with counter 50 should be rejected (replay — last was 100)
        let msg_a_replay = make_message(1, FobButton::Unlock, 50, &secret_a);
        assert!(RkeFeature::<crate::adapters::mock::MockBus>::authenticate(
            &mut fobs,
            &msg_a_replay
        )
        .is_none());
    }

    #[test]
    fn auth_wrong_key_rejected() {
        let real_secret: SharedSecret = [0x33; 16];
        let wrong_secret: SharedSecret = [0x44; 16];
        let fob = make_fob(3, real_secret);
        let mut fobs: HashMap<u32, PairedFob> = [(3, fob)].into();

        // Message signed with wrong secret
        let msg = make_message(3, FobButton::Unlock, 1, &wrong_secret);
        assert!(
            RkeFeature::<crate::adapters::mock::MockBus>::authenticate(&mut fobs, &msg).is_none()
        );
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

    // ── Config toggle tests ───────────────────────────────────────────────

    fn make_rke_feature(
        bus: Arc<crate::adapters::mock::MockBus>,
        arbiter: Arc<DoorLockArbiter>,
        paired_fobs: Vec<PairedFob>,
    ) -> RkeFeature<crate::adapters::mock::MockBus> {
        let config = crate::config::PlatformConfig::defaults();
        // Spin up a trunk arbiter on the same bus so RKE TrunkRelease
        // tests work without changing call sites.  No valet — the gate
        // is open by default.
        let (trunk_arb, trunk_fut) = crate::arbiter::trunk_arbiter(Arc::clone(&bus));
        tokio::spawn(trunk_fut);
        let trunk_arb = Arc::new(trunk_arb);
        RkeFeature::new(bus, arbiter, trunk_arb, config, paired_fobs)
    }

    #[tokio::test]
    async fn combo_lock_then_unlock_toggles_two_stage_unlock() {
        let bus = Arc::new(crate::adapters::mock::MockBus::new());
        let (arbiter, _ack_tx, _handle) = crate::arbiter::door_lock_arbiter(Arc::clone(&bus));
        let arbiter = Arc::new(arbiter);
        let secret: SharedSecret = [0xAA; 16];
        let fob = make_fob(1, secret);
        let mut feature = make_rke_feature(Arc::clone(&bus), Arc::clone(&arbiter), vec![fob]);

        // Verify default: two_stage_unlock starts true.
        assert!(feature.dealer_rx.borrow().two_stage_unlock);

        // Send LOCK then UNLOCK within combo window — should toggle.
        feature.handle_authenticated(1, FobButton::Lock).await;
        feature.handle_authenticated(1, FobButton::Unlock).await;

        // After combo: two_stage_unlock should be false.
        assert!(
            !feature.dealer_rx.borrow().two_stage_unlock,
            "two_stage_unlock should be toggled off after LOCK+UNLOCK combo"
        );
    }

    #[tokio::test]
    async fn combo_unlock_then_lock_toggles_two_stage_unlock() {
        let bus = Arc::new(crate::adapters::mock::MockBus::new());
        let (arbiter, _ack_tx, _handle) = crate::arbiter::door_lock_arbiter(Arc::clone(&bus));
        let arbiter = Arc::new(arbiter);
        let secret: SharedSecret = [0xBB; 16];
        let fob = make_fob(1, secret);
        let mut feature = make_rke_feature(Arc::clone(&bus), Arc::clone(&arbiter), vec![fob]);

        assert!(feature.dealer_rx.borrow().two_stage_unlock);

        // UNLOCK then LOCK is also a valid combo.
        feature.handle_authenticated(1, FobButton::Unlock).await;
        feature.handle_authenticated(1, FobButton::Lock).await;

        assert!(
            !feature.dealer_rx.borrow().two_stage_unlock,
            "UNLOCK+LOCK combo should also toggle"
        );
    }

    #[tokio::test]
    async fn combo_different_fobs_do_not_trigger_toggle() {
        let bus = Arc::new(crate::adapters::mock::MockBus::new());
        let (arbiter, _ack_tx, _handle) = crate::arbiter::door_lock_arbiter(Arc::clone(&bus));
        let arbiter = Arc::new(arbiter);
        let secret_a: SharedSecret = [0xAA; 16];
        let secret_b: SharedSecret = [0xBB; 16];
        let fob_a = make_fob(1, secret_a);
        let fob_b = make_fob(2, secret_b);
        let mut feature =
            make_rke_feature(Arc::clone(&bus), Arc::clone(&arbiter), vec![fob_a, fob_b]);

        assert!(feature.dealer_rx.borrow().two_stage_unlock);

        // LOCK from fob 1, UNLOCK from fob 2 — different fobs, no combo.
        feature.handle_authenticated(1, FobButton::Lock).await;
        feature.handle_authenticated(2, FobButton::Unlock).await;

        assert!(
            feature.dealer_rx.borrow().two_stage_unlock,
            "combo from different fobs should NOT toggle"
        );
    }

    #[tokio::test]
    async fn combo_second_toggle_restores_original_value() {
        let bus = Arc::new(crate::adapters::mock::MockBus::new());
        let (arbiter, _ack_tx, _handle) = crate::arbiter::door_lock_arbiter(Arc::clone(&bus));
        let arbiter = Arc::new(arbiter);
        let secret: SharedSecret = [0xCC; 16];
        let fob = make_fob(1, secret);
        let mut feature = make_rke_feature(Arc::clone(&bus), Arc::clone(&arbiter), vec![fob]);

        // Toggle off.
        feature.handle_authenticated(1, FobButton::Lock).await;
        feature.handle_authenticated(1, FobButton::Unlock).await;
        assert!(!feature.dealer_rx.borrow().two_stage_unlock);

        // Toggle back on.
        feature.handle_authenticated(1, FobButton::Lock).await;
        feature.handle_authenticated(1, FobButton::Unlock).await;
        assert!(
            feature.dealer_rx.borrow().two_stage_unlock,
            "second combo should restore two_stage_unlock to true"
        );
    }
}
