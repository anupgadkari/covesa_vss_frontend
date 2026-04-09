//! Auto Relock — re-locks the vehicle if no door is opened within a
//! configurable timeout (default 45 s) after an unlock event.
//!
//! Subscribes to:
//!   - Body.Doors.Row[1,2].{Left,Right}.IsLocked  (STATE_UPDATE)
//!   - Body.Doors.Row[1,2].{Left,Right}.IsOpen     (STATE_UPDATE)
//!   - Vehicle.Safety.CrashDetected                 (STATE_UPDATE)
//!   - Vehicle.LowVoltageSystemState                (STATE_UPDATE)
//!
//! Outputs:
//!   - DoorLockArbiter → LOCK (requestor: AutoRelock)
//!
//! Safety: if a crash is detected at any time, the feature cancels any
//! running timer and enters DISABLED state. It stays disabled until it
//! observes a full power cycle: LowVoltageSystemState must go to OFF
//! (or ACC) and then back to ON or START. Only then does it re-enable.
//! Rationale: the 10s arbiter lockout expires before our 45s timer, so
//! relying on arbiter rejection alone would allow a post-crash relock.

use std::sync::Arc;
use std::time::Duration;

use futures::StreamExt;
use tokio::select;
use tokio::time::sleep;

use crate::arbiter::{DoorLockArbiter, DoorLockRequest, LockCommand};
use crate::config::{DoorConfig, PlatformConfig};
use crate::ipc_message::{FeatureId, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

/// Default timeout before automatic relock.
/// Used only as a fallback if PlatformConfig is not provided.
const DEFAULT_RELOCK_TIMEOUT: Duration = Duration::from_secs(45);

/// Crash detection signal from the Safety Monitor.
const CRASH_SIGNAL: VssPath = "Vehicle.Safety.CrashDetected";

/// Power state signal — standard VSS v4.0.
const POWER_STATE_SIGNAL: VssPath = "Vehicle.LowVoltageSystemState";

/// Default door lock signals (4-door sedan). Used when no DoorConfig
/// is provided (backward compatibility / simple tests).
const DEFAULT_LOCK_SIGNALS: &[VssPath] = &[
    "Body.Doors.Row1.Left.IsLocked",
    "Body.Doors.Row1.Right.IsLocked",
    "Body.Doors.Row2.Left.IsLocked",
    "Body.Doors.Row2.Right.IsLocked",
];

/// Default door open signals (4-door sedan).
const DEFAULT_OPEN_SIGNALS: &[VssPath] = &[
    "Body.Doors.Row1.Left.IsOpen",
    "Body.Doors.Row1.Right.IsOpen",
    "Body.Doors.Row2.Left.IsOpen",
    "Body.Doors.Row2.Right.IsOpen",
];

/// LowVoltageSystemState values that indicate "power off" (pre-cycle).
fn is_power_off(val: &SignalValue) -> bool {
    matches!(
        val,
        SignalValue::String(s) if s == "OFF" || s == "ACC" || s == "LOCK"
    )
}

/// LowVoltageSystemState values that indicate "power on" (post-cycle).
fn is_power_on(val: &SignalValue) -> bool {
    matches!(
        val,
        SignalValue::String(s) if s == "ON" || s == "START"
    )
}

pub struct AutoRelock<B: SignalBus> {
    arbiter: Arc<DoorLockArbiter>,
    bus: Arc<B>,
    timeout: Duration,
}

impl<B: SignalBus> AutoRelock<B> {
    /// Create with platform configuration (production path).
    /// Reads the auto-relock timeout from Tier 2 (vehicle-line calibration).
    pub fn from_config(
        arbiter: Arc<DoorLockArbiter>,
        bus: Arc<B>,
        config: &PlatformConfig,
    ) -> Self {
        Self {
            arbiter,
            bus,
            timeout: config.auto_relock_timeout(),
        }
    }

    /// Create with default timeout (convenience for tests).
    pub fn new(arbiter: Arc<DoorLockArbiter>, bus: Arc<B>) -> Self {
        Self {
            arbiter,
            bus,
            timeout: DEFAULT_RELOCK_TIMEOUT,
        }
    }

    /// Override timeout (for unit tests with short durations).
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    /// Main run loop. Call via `tokio::spawn(auto_relock.run())`.
    ///
    /// The feature runs in one of two modes:
    /// - **ENABLED**: normal operation — watches for unlocks, runs timers.
    /// - **DISABLED**: after crash — ignores all lock/unlock events, waits
    ///   for a full power cycle (OFF/ACC → ON/START) before re-enabling.
    pub async fn run(self) {
        tracing::info!(
            timeout_secs = self.timeout.as_secs(),
            "AutoRelock feature started"
        );

        // Subscribe to all lock signals — merge into a single stream
        let lock_streams = futures::future::join_all(
            DOOR_LOCK_SIGNALS
                .iter()
                .map(|&sig| self.bus.subscribe(sig)),
        )
        .await;
        let mut lock_stream = futures::stream::select_all(lock_streams);

        // Subscribe to all open signals — merge into a single stream
        let open_streams = futures::future::join_all(
            DOOR_OPEN_SIGNALS
                .iter()
                .map(|&sig| self.bus.subscribe(sig)),
        )
        .await;
        let mut open_stream = futures::stream::select_all(open_streams);

        // Subscribe to crash detection signal
        let mut crash_stream = self.bus.subscribe(CRASH_SIGNAL).await;

        // Subscribe to power state for crash recovery
        let mut power_stream = self.bus.subscribe(POWER_STATE_SIGNAL).await;

        loop {
            // ==== ENABLED MODE: normal relock logic ====

            // Phase 1: Wait for an unlock event.
            // Also watch for crash — enters DISABLED mode.
            let phase1 = loop {
                select! {
                    Some(val) = lock_stream.next() => {
                        if val == SignalValue::Bool(false) {
                            break Phase1Result::Unlocked;
                        }
                    }
                    Some(val) = crash_stream.next() => {
                        if val == SignalValue::Bool(true) {
                            break Phase1Result::CrashDetected;
                        }
                    }
                    else => return, // bus closed
                }
            };

            if let Phase1Result::CrashDetected = phase1 {
                tracing::warn!("AutoRelock: crash detected (idle), entering DISABLED state");
                self.wait_for_power_cycle(
                    &mut crash_stream,
                    &mut power_stream,
                )
                .await;
                continue; // Re-enter phase 1 in ENABLED mode
            }

            tracing::info!("AutoRelock: unlock detected, starting relock timer");

            // Phase 2: Timer is running.
            let timer_result = loop {
                select! {
                    _ = sleep(self.timeout) => {
                        break TimerOutcome::Expired;
                    }
                    Some(val) = open_stream.next() => {
                        if val == SignalValue::Bool(true) {
                            tracing::info!("AutoRelock: door opened, timer cancelled");
                            break TimerOutcome::DoorOpened;
                        }
                    }
                    Some(val) = lock_stream.next() => {
                        if val == SignalValue::Bool(true) {
                            tracing::info!("AutoRelock: doors re-locked externally, timer cancelled");
                            break TimerOutcome::AlreadyLocked;
                        }
                        if val == SignalValue::Bool(false) {
                            tracing::info!("AutoRelock: second unlock, restarting timer");
                            break TimerOutcome::Restart;
                        }
                    }
                    Some(val) = crash_stream.next() => {
                        if val == SignalValue::Bool(true) {
                            break TimerOutcome::CrashDisable;
                        }
                    }
                    else => return,
                }
            };

            match timer_result {
                TimerOutcome::Expired => {
                    tracing::info!("AutoRelock: timeout expired, requesting LOCK");
                    if let Err(e) = self
                        .arbiter
                        .request(DoorLockRequest {
                            command: LockCommand::Lock,
                            feature_id: FeatureId::AutoRelock,
                        })
                        .await
                    {
                        tracing::error!(error = %e, "AutoRelock: failed to submit LOCK");
                    }
                }
                TimerOutcome::DoorOpened | TimerOutcome::AlreadyLocked => {
                    // Back to phase 1
                }
                TimerOutcome::Restart => {
                    continue; // Re-enter phase 2 (timer restarts)
                }
                TimerOutcome::CrashDisable => {
                    tracing::warn!(
                        "AutoRelock: crash detected during timer, entering DISABLED state"
                    );
                    self.wait_for_power_cycle(
                        &mut crash_stream,
                        &mut power_stream,
                    )
                    .await;
                    // Re-enter phase 1 in ENABLED mode
                }
            }
        }
    }

    /// DISABLED state: wait for a full power cycle before re-enabling.
    ///
    /// A power cycle is: LowVoltageSystemState goes to OFF (or ACC/LOCK),
    /// then transitions to ON or START. ACC may or may not appear between
    /// OFF and ON — both sequences are valid:
    ///   OFF → ON          (direct start)
    ///   OFF → ACC → ON    (accessory then start)
    ///
    /// The feature stays in this method (ignoring all lock/unlock events)
    /// until the full OFF→ON transition is observed.
    async fn wait_for_power_cycle(
        &self,
        crash_stream: &mut (impl futures::Stream<Item = SignalValue> + Unpin),
        power_stream: &mut (impl futures::Stream<Item = SignalValue> + Unpin),
    ) {
        tracing::info!("AutoRelock: DISABLED — waiting for power cycle (OFF → ON)");

        // Step 1: Wait for power OFF (or ACC/LOCK)
        loop {
            select! {
                Some(val) = power_stream.next() => {
                    if is_power_off(&val) {
                        tracing::info!(
                            state = ?val,
                            "AutoRelock: power-off state observed, waiting for ON/START"
                        );
                        break;
                    }
                }
                // Keep draining crash stream to avoid backpressure, but
                // we're already disabled so it doesn't change anything.
                Some(_) = crash_stream.next() => {}
                else => return,
            }
        }

        // Step 2: Wait for power ON or START
        loop {
            select! {
                Some(val) = power_stream.next() => {
                    if is_power_on(&val) {
                        tracing::info!(
                            state = ?val,
                            "AutoRelock: power-on observed, re-enabling feature"
                        );
                        return; // Back to ENABLED mode
                    }
                    // ACC between OFF and ON is fine — keep waiting for ON/START
                }
                Some(_) = crash_stream.next() => {}
                else => return,
            }
        }
    }
}

enum Phase1Result {
    Unlocked,
    CrashDetected,
}

/// Internal enum for timer resolution.
enum TimerOutcome {
    /// Timer expired — should relock.
    Expired,
    /// A door was opened — cancel.
    DoorOpened,
    /// Doors were re-locked by another source.
    AlreadyLocked,
    /// Another unlock event — restart timer.
    Restart,
    /// Crash detected — enter DISABLED state.
    CrashDisable,
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::door_lock_arbiter;

    /// Helper: set up AutoRelock with a short timeout for testing.
    async fn setup(
        timeout: Duration,
    ) -> (Arc<MockBus>, Arc<DoorLockArbiter>, tokio::task::JoinHandle<()>) {
        let bus = Arc::new(MockBus::new());
        let (arbiter, _ack_tx, loop_fut) = door_lock_arbiter(Arc::clone(&bus));
        tokio::spawn(loop_fut);
        let arbiter = Arc::new(arbiter);

        let feature = AutoRelock::new(Arc::clone(&arbiter), Arc::clone(&bus))
            .with_timeout(timeout);
        let handle = tokio::spawn(feature.run());

        // Let everything start up
        tokio::task::yield_now().await;

        (bus, arbiter, handle)
    }

    #[tokio::test]
    async fn relock_after_timeout() {
        let (bus, _arb, _handle) = setup(Duration::from_millis(100)).await;

        bus.inject("Body.Doors.Row1.Left.IsLocked", SignalValue::Bool(false));
        tokio::task::yield_now().await;

        sleep(Duration::from_millis(200)).await;
        tokio::task::yield_now().await;

        let history = bus.history();
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == "Body.Doors.Row1.Left.IsLocked" && *val == SignalValue::Bool(true)
            }),
            "expected AutoRelock to dispatch LOCK, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn door_opened_cancels_relock() {
        let (bus, _arb, _handle) = setup(Duration::from_millis(200)).await;

        bus.inject("Body.Doors.Row1.Left.IsLocked", SignalValue::Bool(false));
        tokio::task::yield_now().await;

        sleep(Duration::from_millis(50)).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        tokio::task::yield_now().await;

        sleep(Duration::from_millis(300)).await;
        tokio::task::yield_now().await;

        let history = bus.history();
        assert!(
            !history.iter().any(|(sig, val)| {
                *sig == "Body.Doors.Row1.Left.IsLocked" && *val == SignalValue::Bool(true)
            }),
            "AutoRelock should NOT have dispatched LOCK after door opened, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn external_relock_cancels_timer() {
        let (bus, _arb, _handle) = setup(Duration::from_millis(200)).await;

        bus.inject("Body.Doors.Row1.Left.IsLocked", SignalValue::Bool(false));
        tokio::task::yield_now().await;

        sleep(Duration::from_millis(50)).await;
        bus.inject("Body.Doors.Row1.Left.IsLocked", SignalValue::Bool(true));
        tokio::task::yield_now().await;

        sleep(Duration::from_millis(300)).await;
        tokio::task::yield_now().await;

        let history = bus.history();
        let lock_count = history
            .iter()
            .filter(|(sig, val)| {
                *sig == "Body.Doors.Row1.Left.IsLocked" && *val == SignalValue::Bool(true)
            })
            .count();
        assert_eq!(
            lock_count, 0,
            "AutoRelock should NOT have dispatched LOCK after external relock"
        );
    }

    #[tokio::test]
    async fn crash_during_timer_disables_until_power_cycle() {
        let (bus, _arb, handle) = setup(Duration::from_millis(500)).await;

        // Unlock → timer starts
        bus.inject("Body.Doors.Row1.Left.IsLocked", SignalValue::Bool(false));
        tokio::task::yield_now().await;

        // Crash during timer
        sleep(Duration::from_millis(50)).await;
        bus.inject(CRASH_SIGNAL, SignalValue::Bool(true));
        tokio::task::yield_now().await;

        // Wait well past the original timeout
        sleep(Duration::from_millis(600)).await;
        tokio::task::yield_now().await;

        // No LOCK should have been dispatched
        let history = bus.history();
        assert!(
            !history.iter().any(|(sig, val)| {
                *sig == "Body.Doors.Row1.Left.IsLocked" && *val == SignalValue::Bool(true)
            }),
            "AutoRelock should NOT dispatch LOCK after crash, history: {:?}",
            history
        );

        // Feature should still be running (in DISABLED state, waiting for power cycle)
        assert!(!handle.is_finished(), "AutoRelock should be alive in DISABLED state");

        // Simulate power cycle: OFF → ON
        bus.inject(
            POWER_STATE_SIGNAL,
            SignalValue::String("OFF".to_string()),
        );
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(10)).await;

        bus.inject(
            POWER_STATE_SIGNAL,
            SignalValue::String("ON".to_string()),
        );
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(10)).await;

        // Feature should still be running (re-enabled, back in phase 1)
        assert!(!handle.is_finished(), "AutoRelock should be re-enabled after power cycle");

        // Now verify it works again: unlock → timeout → LOCK
        bus.clear_history();
        bus.inject("Body.Doors.Row1.Left.IsLocked", SignalValue::Bool(false));
        tokio::task::yield_now().await;

        sleep(Duration::from_millis(600)).await;
        tokio::task::yield_now().await;

        let history = bus.history();
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == "Body.Doors.Row1.Left.IsLocked" && *val == SignalValue::Bool(true)
            }),
            "AutoRelock should work again after power cycle, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn crash_before_unlock_disables_until_power_cycle() {
        let (bus, _arb, handle) = setup(Duration::from_millis(100)).await;

        // Crash before any unlock event
        bus.inject(CRASH_SIGNAL, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        // Feature should still be alive (DISABLED, waiting for power cycle)
        assert!(!handle.is_finished(), "AutoRelock should be in DISABLED state");

        // Unlock during DISABLED — should be ignored
        bus.inject("Body.Doors.Row1.Left.IsLocked", SignalValue::Bool(false));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(200)).await;
        tokio::task::yield_now().await;

        let history = bus.history();
        assert!(
            !history.iter().any(|(sig, val)| {
                *sig == "Body.Doors.Row1.Left.IsLocked" && *val == SignalValue::Bool(true)
            }),
            "disabled AutoRelock should NOT dispatch LOCK, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn power_cycle_with_acc_in_between() {
        let (bus, _arb, handle) = setup(Duration::from_millis(100)).await;

        // Crash → DISABLED
        bus.inject(CRASH_SIGNAL, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(50)).await;

        // Power cycle: OFF → ACC → ON
        bus.inject(
            POWER_STATE_SIGNAL,
            SignalValue::String("OFF".to_string()),
        );
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(10)).await;

        bus.inject(
            POWER_STATE_SIGNAL,
            SignalValue::String("ACC".to_string()),
        );
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(10)).await;

        // Should still be disabled — ACC is not ON/START
        // (We can't easily assert DISABLED here, but we verify ON re-enables)

        bus.inject(
            POWER_STATE_SIGNAL,
            SignalValue::String("ON".to_string()),
        );
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(10)).await;

        // Now re-enabled — verify normal operation
        assert!(!handle.is_finished(), "AutoRelock should be re-enabled");

        bus.clear_history();
        bus.inject("Body.Doors.Row1.Left.IsLocked", SignalValue::Bool(false));
        tokio::task::yield_now().await;

        sleep(Duration::from_millis(200)).await;
        tokio::task::yield_now().await;

        let history = bus.history();
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == "Body.Doors.Row1.Left.IsLocked" && *val == SignalValue::Bool(true)
            }),
            "AutoRelock should work after OFF → ACC → ON cycle, history: {:?}",
            history
        );
    }
}
