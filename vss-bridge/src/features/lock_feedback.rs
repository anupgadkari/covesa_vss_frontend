//! Lock / Unlock Feedback Flash — visual confirmation on direction indicators.
//!
//! Subscribes to `Body.Doors.CentralLock.FeedbackRequest` (published by
//! external-origin features: RKE, WalkAwayLock, ThumbPadLock, AutoRelock)
//! and plays a timed flash pattern on both direction indicators via the
//! Lighting domain arbiter at **priority HIGH**.
//!
//! # Flash patterns
//!
//! Each *flash unit* = 100 ms OFF lead-in → 900 ms ON.
//!
//! | Event            | Pattern                                |
//! |------------------|----------------------------------------|
//! | `"lock"`         | 1 flash unit                           |
//! | `"unlock"`       | flash unit · 300 ms gap · flash unit  |
//! | `"trunk_unlock"` | Same as unlock + arm trunk-close latch |
//!
//! # Preemption
//!
//! If a new `FeedbackRequest` arrives while a pattern is playing, the current
//! task is aborted, the arbiter claims are released, and the new pattern starts
//! immediately from the beginning.
//!
//! # Trunk-close lock feedback
//!
//! When a `"trunk_unlock"` request is received the feature sets an internal
//! flag. When `Body.Trunk.IsOpen` subsequently transitions to `false`, a
//! `"lock"` pattern is played automatically.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;
use tokio::task::JoinHandle;
use tokio::time::{sleep, Duration};

use crate::arbiter::{ActuatorRequest, DomainArbiter, FEEDBACK_REQUEST};
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

// ── Signal constants ───────────────────────────────────────────────────────

const LEFT_SIG: VssPath = "Body.Lights.DirectionIndicator.Left.IsSignaling";
const RIGHT_SIG: VssPath = "Body.Lights.DirectionIndicator.Right.IsSignaling";
const TRUNK_OPEN_SIG: VssPath = "Body.Trunk.IsOpen";

/// Door IsLocked signals tracked to determine whether the cabin is secured.
const DOOR_LOCKED_SIGNALS: [VssPath; 4] = [
    "Body.Doors.Row1.Left.IsLocked",
    "Body.Doors.Row1.Right.IsLocked",
    "Body.Doors.Row2.Left.IsLocked",
    "Body.Doors.Row2.Right.IsLocked",
];

// ── Flash timing ───────────────────────────────────────────────────────────

/// Dark lead-in before each flash unit (ms). Creates a deliberate "OFF" gap
/// so the flash has a visible start edge even when indicators are already lit.
const LEAD_IN_MS: u64 = 100;

/// Duration of the ON phase of each flash unit (ms).
///
/// Must be less than two BlinkRelay half-periods (2 × 333 ms = 666 ms) so that
/// only one 333 ms ON pulse fires per flash unit. At 500 ms the lamp is ON for
/// 333 ms, naturally goes OFF via the blink tick, then the claim is released
/// before the second ON fires — producing exactly 1 lamp flash per flash unit.
const FLASH_ON_MS: u64 = 500;

/// Gap between the two unlock flash units (ms).
const GAP_MS: u64 = 300;

// ── Feature struct ─────────────────────────────────────────────────────────

pub struct LockFeedback<B: SignalBus> {
    bus: Arc<B>,
    lighting_arb: Arc<DomainArbiter>,
}

impl<B: SignalBus + Send + Sync + 'static> LockFeedback<B> {
    pub fn new(bus: Arc<B>, lighting_arb: Arc<DomainArbiter>) -> Self {
        Self { bus, lighting_arb }
    }

    pub async fn run(self) {
        let mut feedback_rx = self.bus.subscribe(FEEDBACK_REQUEST).await;
        let mut trunk_rx = self.bus.subscribe(TRUNK_OPEN_SIG).await;

        // Subscribe to all door IsLocked signals so we always know whether
        // the cabin is fully secured.  Unknown at startup → assume unlocked.
        let mut door_rx0 = self.bus.subscribe(DOOR_LOCKED_SIGNALS[0]).await;
        let mut door_rx1 = self.bus.subscribe(DOOR_LOCKED_SIGNALS[1]).await;
        let mut door_rx2 = self.bus.subscribe(DOOR_LOCKED_SIGNALS[2]).await;
        let mut door_rx3 = self.bus.subscribe(DOOR_LOCKED_SIGNALS[3]).await;
        let mut doors_locked = [false; 4];

        let mut current_flash: Option<JoinHandle<()>> = None;
        // Set when a "trunk_unlock" feedback is received; cleared when trunk closes.
        let mut trunk_opened_externally = false;

        tracing::info!("LockFeedback feature started");

        loop {
            select! {
                Some(val) = feedback_rx.next() => {
                    let kind = match &val {
                        SignalValue::String(s) => match s.as_str() {
                            // Trust the command for direct lock/unlock requests.
                            // Checking door state here races with the plant model
                            // publishing IsLocked — the feedback arrives before the
                            // confirmed state has propagated.
                            "lock"   => "lock",
                            "unlock" => "unlock",
                            "trunk_unlock" => {
                                trunk_opened_externally = true;
                                "unlock"
                            }
                            other => {
                                tracing::warn!(value = other, "LockFeedback: unknown FeedbackRequest — ignored");
                                continue;
                            }
                        },
                        _ => continue,
                    };

                    tracing::info!(kind, "LockFeedback: starting flash sequence");
                    preempt_and_start(
                        &mut current_flash,
                        kind,
                        Arc::clone(&self.lighting_arb),
                    )
                    .await;
                }

                Some(val) = trunk_rx.next() => {
                    // Trunk closed while we were tracking an external open.
                    // If the cabin is secured → lock flash (all good).
                    // If the cabin is still unsecured → unlock flash (warn: not secured).
                    if val == SignalValue::Bool(false) && trunk_opened_externally {
                        trunk_opened_externally = false;
                        let kind = if doors_locked.iter().all(|&l| l) {
                            tracing::info!("LockFeedback: trunk closed, cabin secured — lock flash");
                            "lock"
                        } else {
                            tracing::info!("LockFeedback: trunk closed, cabin UNSECURED — unlock flash (warning)");
                            "unlock"
                        };
                        preempt_and_start(
                            &mut current_flash,
                            kind,
                            Arc::clone(&self.lighting_arb),
                        )
                        .await;
                    }
                }

                // ── Track door lock state ──────────────────────────────────
                Some(val) = door_rx0.next() => {
                    if let SignalValue::Bool(b) = val { doors_locked[0] = b; }
                }
                Some(val) = door_rx1.next() => {
                    if let SignalValue::Bool(b) = val { doors_locked[1] = b; }
                }
                Some(val) = door_rx2.next() => {
                    if let SignalValue::Bool(b) = val { doors_locked[2] = b; }
                }
                Some(val) = door_rx3.next() => {
                    if let SignalValue::Bool(b) = val { doors_locked[3] = b; }
                }

                else => break,
            }
        }

        tracing::info!("LockFeedback feature stopped");
    }
}

// ── Internal helpers ───────────────────────────────────────────────────────

/// Abort any running flash task, release stuck indicator claims, then start
/// a new flash task for `kind` ("lock" | "unlock").
async fn preempt_and_start(
    current: &mut Option<JoinHandle<()>>,
    kind: &str,
    arb: Arc<DomainArbiter>,
) {
    if let Some(handle) = current.take() {
        handle.abort();
        // Wait for the abort to complete (resolves immediately).
        let _ = handle.await;
    }
    // Release any claims left by the aborted task.
    release_both(&arb).await;

    let kind = kind.to_string();
    let arb_clone = Arc::clone(&arb);
    *current = Some(tokio::spawn(async move {
        play_sequence(&kind, &arb_clone).await;
    }));
}

/// Play the full flash sequence for `kind` and release when done.
///
/// Sequence layout:
///
/// ```text
/// lock:   [100ms OFF] [900ms ON] → release
///
/// unlock: [100ms OFF] [900ms ON] [300ms OFF] [100ms OFF] [900ms ON] → release
/// ```
async fn play_sequence(kind: &str, arb: &Arc<DomainArbiter>) {
    let flashes: u8 = if kind == "lock" { 1 } else { 2 };

    for i in 0..flashes {
        if i > 0 {
            // Gap between flashes — keep indicators dark
            claim_both(arb, false).await;
            sleep(Duration::from_millis(GAP_MS)).await;
        }
        // Lead-in: short OFF pulse before the ON flash
        claim_both(arb, false).await;
        sleep(Duration::from_millis(LEAD_IN_MS)).await;
        // Flash ON
        claim_both(arb, true).await;
        sleep(Duration::from_millis(FLASH_ON_MS)).await;
    }

    release_both(arb).await;
}

/// Claim both direction indicators at HIGH priority with the given value.
async fn claim_both(arb: &Arc<DomainArbiter>, on: bool) {
    for &sig in &[LEFT_SIG, RIGHT_SIG] {
        let _ = arb
            .request(ActuatorRequest {
                signal: sig,
                value: SignalValue::Bool(on),
                priority: Priority::High,
                feature_id: FeatureId::LockFeedback,
            })
            .await;
    }
}

/// Release both direction indicator claims held by LockFeedback.
async fn release_both(arb: &Arc<DomainArbiter>) {
    for &sig in &[LEFT_SIG, RIGHT_SIG] {
        let _ = arb.release(sig, FeatureId::LockFeedback).await;
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::lighting_arbiter;
    use tokio::time::advance;

    async fn setup() -> (Arc<MockBus>, Arc<DomainArbiter>) {
        let bus = Arc::new(MockBus::new());
        let (arb, loop_fut) = lighting_arbiter(Arc::clone(&bus));
        tokio::spawn(loop_fut);
        drain().await;
        (bus, Arc::new(arb))
    }

    /// Yield multiple times to flush nested async chains:
    /// flash task → arbiter channel → arbiter loop → bus.publish.
    async fn drain() {
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
    }

    /// Helper: publish a FeedbackRequest to the bus and let tasks run.
    async fn send_feedback(bus: &MockBus, kind: &str) {
        bus.inject(FEEDBACK_REQUEST, SignalValue::String(kind.into()));
        drain().await;
    }

    #[tokio::test(start_paused = true)]
    async fn lock_feedback_single_flash_sequence() {
        // With start_paused=true, we step through each sleep individually because
        // sleeps are set lazily (relative to when the task runs, not when advance() fires).
        // Sequence: claim(false) → sleep(100ms) → claim(true) → sleep(900ms) → release
        let (bus, arb) = setup().await;
        let feature = LockFeedback::new(Arc::clone(&bus), Arc::clone(&arb));
        tokio::spawn(feature.run());
        drain().await;

        bus.clear_history();
        send_feedback(&bus, "lock").await;
        // After send_feedback + drain: flash task has run to sleep(LEAD_IN_MS=100ms).
        // claim(false) has been published already.
        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == LEFT_SIG && *v == SignalValue::Bool(false)),
            "expected OFF claim during lead-in, got: {:?}",
            h
        );

        // Step 1: advance past the 100ms lead-in sleep → flash task wakes, claims true,
        // sets the 900ms ON sleep.
        advance(Duration::from_millis(LEAD_IN_MS + 1)).await;
        drain().await;
        let h = bus.history();
        assert!(
            h.iter()
                .any(|(s, v)| *s == LEFT_SIG && *v == SignalValue::Bool(true)),
            "expected ON claim after lead-in, got: {:?}",
            h
        );

        // Step 2: advance past the 900ms ON sleep → flash task wakes and releases.
        advance(Duration::from_millis(FLASH_ON_MS + 1)).await;
        drain().await;

        let h2 = bus.history();
        let left_seq: Vec<_> = h2
            .iter()
            .filter(|(s, _)| *s == LEFT_SIG)
            .map(|(_, v)| v.clone())
            .collect();
        // Must end with false (released)
        let last_left = left_seq.last().cloned();
        assert_eq!(
            last_left,
            Some(SignalValue::Bool(false)),
            "expected released (false) after sequence, got: {:?}",
            last_left
        );
    }

    #[tokio::test(start_paused = true)]
    async fn unlock_feedback_two_flash_units() {
        // Sequence: [OFF 100ms] [ON 900ms] [OFF 300ms] [OFF 100ms] [ON 900ms] → release
        // We must step through each sleep because they're set lazily.
        let (bus, arb) = setup().await;
        let feature = LockFeedback::new(Arc::clone(&bus), Arc::clone(&arb));
        tokio::spawn(feature.run());
        drain().await;

        bus.clear_history();
        send_feedback(&bus, "unlock").await;
        // Flash task is now at sleep(LEAD_IN_MS) for flash 0.

        // Flash 0 lead-in fires:
        advance(Duration::from_millis(LEAD_IN_MS + 1)).await;
        drain().await;
        // Flash 0 ON fires:
        advance(Duration::from_millis(FLASH_ON_MS + 1)).await;
        drain().await;
        // Gap fires:
        advance(Duration::from_millis(GAP_MS + 1)).await;
        drain().await;
        // Flash 1 lead-in fires:
        advance(Duration::from_millis(LEAD_IN_MS + 1)).await;
        drain().await;
        // Flash 1 ON fires:
        advance(Duration::from_millis(FLASH_ON_MS + 1)).await;
        drain().await;

        let h = bus.history();
        let true_count = h
            .iter()
            .filter(|(s, v)| *s == LEFT_SIG && *v == SignalValue::Bool(true))
            .count();
        assert_eq!(
            true_count, 2,
            "expected exactly 2 ON events for unlock, got {true_count}"
        );
        // Should also have ended with release (false)
        let last_left = h
            .iter()
            .filter(|(s, _)| *s == LEFT_SIG)
            .map(|(_, v)| v.clone())
            .next_back();
        assert_eq!(
            last_left,
            Some(SignalValue::Bool(false)),
            "expected released after unlock sequence, got: {:?}",
            last_left
        );
    }

    #[tokio::test(start_paused = true)]
    async fn preemption_aborts_current_pattern() {
        let (bus, arb) = setup().await;
        let feature = LockFeedback::new(Arc::clone(&bus), Arc::clone(&arb));
        tokio::spawn(feature.run());
        drain().await;

        // Start unlock sequence
        send_feedback(&bus, "unlock").await;

        // Interrupt mid-way through — send lock feedback during first flash ON.
        // With LEAD_IN_MS=100 + FLASH_ON_MS=500, flash 1 ends at 600 ms.
        // Interrupt at 300 ms (200 ms into the ON phase).
        advance(Duration::from_millis(300)).await;
        drain().await;
        bus.clear_history();

        send_feedback(&bus, "lock").await;

        // Advance past the full lock sequence (LEAD_IN_MS + FLASH_ON_MS = 600 ms).
        advance(Duration::from_millis(700)).await;
        drain().await;

        // Should see exactly 1 ON event (the lock flash), not the continuation of unlock
        let h = bus.history();
        let true_count = h
            .iter()
            .filter(|(s, v)| *s == LEFT_SIG && *v == SignalValue::Bool(true))
            .count();
        assert_eq!(
            true_count, 1,
            "expected exactly 1 ON event after preemption (lock flash), got {true_count}"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn trunk_unlock_arms_trunk_close_feedback() {
        let (bus, arb) = setup().await;
        let feature = LockFeedback::new(Arc::clone(&bus), Arc::clone(&arb));
        tokio::spawn(feature.run());
        drain().await;

        // Simulate trunk open via RKE
        send_feedback(&bus, "trunk_unlock").await;
        advance(Duration::from_millis(2400)).await;
        drain().await;

        bus.clear_history();

        // Trunk closes — should trigger lock flash
        bus.inject(TRUNK_OPEN_SIG, SignalValue::Bool(false));
        drain().await;

        advance(Duration::from_millis(1200)).await;
        drain().await;

        let h = bus.history();
        let true_count = h
            .iter()
            .filter(|(s, v)| *s == LEFT_SIG && *v == SignalValue::Bool(true))
            .count();
        assert_eq!(
            true_count, 1,
            "trunk-close should trigger a single lock flash, got {true_count}"
        );
    }

    /// Trunk closes after trunk_unlock, but cabin is unsecured → unlock flash.
    #[tokio::test(start_paused = true)]
    async fn trunk_close_plays_unlock_when_cabin_unsecured() {
        let (bus, arb) = setup().await;
        let feature = LockFeedback::new(Arc::clone(&bus), Arc::clone(&arb));
        tokio::spawn(feature.run());
        drain().await;

        // Doors unlocked (default). Arm the trunk-close latch.
        send_feedback(&bus, "trunk_unlock").await;
        advance(Duration::from_millis(2400)).await;
        drain().await;

        bus.clear_history();

        // Trunk closes — cabin is unsecured.
        bus.inject(TRUNK_OPEN_SIG, SignalValue::Bool(false));
        drain().await;

        // Advance through full unlock sequence (2 flashes).
        advance(Duration::from_millis(LEAD_IN_MS + 1)).await;
        drain().await;
        advance(Duration::from_millis(FLASH_ON_MS + 1)).await;
        drain().await;
        advance(Duration::from_millis(GAP_MS + 1)).await;
        drain().await;
        advance(Duration::from_millis(LEAD_IN_MS + 1)).await;
        drain().await;
        advance(Duration::from_millis(FLASH_ON_MS + 1)).await;
        drain().await;

        let h = bus.history();
        let on_count = h
            .iter()
            .filter(|(s, v)| *s == LEFT_SIG && *v == SignalValue::Bool(true))
            .count();
        assert_eq!(
            on_count, 2,
            "trunk close with unsecured cabin should play 2-flash unlock warning, got {on_count}"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn trunk_close_without_external_open_is_silent() {
        let (bus, arb) = setup().await;
        let feature = LockFeedback::new(Arc::clone(&bus), Arc::clone(&arb));
        tokio::spawn(feature.run());
        drain().await;

        bus.clear_history();

        // Trunk closes without any prior trunk_unlock feedback
        bus.inject(TRUNK_OPEN_SIG, SignalValue::Bool(false));
        drain().await;
        advance(Duration::from_millis(1200)).await;
        drain().await;

        let h = bus.history();
        assert!(
            !h.iter()
                .any(|(s, v)| *s == LEFT_SIG && *v == SignalValue::Bool(true)),
            "trunk close without external open should NOT trigger feedback"
        );
    }
}
