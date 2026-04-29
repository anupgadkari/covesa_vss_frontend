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

use crate::arbiter::{DoorLockArbiter, DoorLockRequest, LockCommand, FEEDBACK_REQUEST};
use crate::config::PlatformConfig;
use crate::ipc_message::{FeatureId, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

/// Default timeout before automatic relock.
/// Used only as a fallback if PlatformConfig is not provided.
const DEFAULT_RELOCK_TIMEOUT: Duration = Duration::from_secs(45);

/// Crash detection signal from the Safety Monitor.
const CRASH_SIGNAL: VssPath = "Vehicle.Safety.CrashDetected";

/// Power state signal — standard VSS v4.0.
const POWER_STATE_SIGNAL: VssPath = "Vehicle.LowVoltageSystemState";

/// Status signal — TRUE while the relock timer is counting down.
const STATUS_IS_ARMED: VssPath = "Body.Doors.AutoRelock.IsArmed";

/// Status signal — published once on each arm to advertise the configured
/// timeout (seconds) to the HMI / consumers.  Allows the HMI to render a
/// matching client-side countdown without hardcoding the value.
const STATUS_TIMEOUT_SECS: VssPath = "Body.Doors.AutoRelock.TimeoutSeconds";

/// Vehicle-level lock status — published by the door-lock arbiter
/// on every accepted command (status itself is deduplicated; companion
/// signals always publish).  AutoRelock reads the latest value of
/// this signal at every `EVENT_NUM` bump to decide whether to arm.
const LOCK_STATUS: VssPath = "Cabin.LockStatus";

/// Identity of the feature whose request the arbiter accepted on the
/// most recent central-lock dispatch.  AutoRelock filters arming
/// against the `EXTERNAL_UNLOCK_REQUESTORS` set so only physical-key
/// / phone / fob unlocks trigger the relock timer; interior soldier
/// knobs, AutoLock, and HMI diagnostic toggles don't arm.
const LOCK_LAST_REQUESTOR: VssPath = "Cabin.LockStatus.LastRequestor";

/// `LastRequestor` strings that should arm AutoRelock.  These are
/// the physical-external unlock paths — the user is acting from
/// outside the vehicle and might walk away without entering.
/// Internal sources (DoorTrimButton, soldier knob, AutoLock,
/// CrashUnlock, WalkAwayLock, ThumbPadLock, DoubleLockRelease) are
/// deliberately *not* in this set: locking again 45 s after the
/// driver pressed an interior unlock is hostile UX.  Strings come
/// from `<FeatureId as Display>` (Debug-format variant names).
const EXTERNAL_UNLOCK_REQUESTORS: &[&str] = &[
    "KeyfobRke",
    "KeyfobPeps",
    "PassiveEntry",
    "PhoneApp",
    "PhoneBle",
    "NfcCard",
    "NfcPhone",
];

/// `LOCK_STATUS` enum values that count as "the vehicle is unlocked"
/// for the purpose of arming AutoRelock.
const UNLOCK_STATES: &[&str] = &["UNLOCKED", "DRIVER_UNLOCKED"];

/// Decide whether a freshly-published `LastRequestor` event should
/// arm the relock timer, given the current cached `LockStatus`.
/// Returns true only when the state is unlocked AND the requestor is
/// in the external set.
fn event_should_arm(status: Option<&str>, requestor: &str) -> bool {
    let Some(status) = status else { return false };
    if !UNLOCK_STATES.contains(&status) {
        return false;
    }
    EXTERNAL_UNLOCK_REQUESTORS.contains(&requestor)
}

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

        // Subscribe to all lock signals — used in Phase 2 only, to
        // detect an external relock (driver hits LOCK while the timer
        // is running) and cancel the timer.  Phase 1 arming is gated
        // on `ARM_REQUEST` instead, so unlocks from PassiveEntry /
        // ThumbPadLock / HMI direct toggle no longer auto-arm.
        let lock_streams = futures::future::join_all(
            DEFAULT_LOCK_SIGNALS
                .iter()
                .map(|&sig| self.bus.subscribe(sig)),
        )
        .await;
        let mut lock_stream = futures::stream::select_all(lock_streams);

        // Subscribe to all open signals — merge into a single stream
        let open_streams = futures::future::join_all(
            DEFAULT_OPEN_SIGNALS
                .iter()
                .map(|&sig| self.bus.subscribe(sig)),
        )
        .await;
        let mut open_stream = futures::stream::select_all(open_streams);

        // Arming triggers from the door-lock arbiter — published in
        // order on every accepted command:
        //   1. `Cabin.LockStatus`            (current state enum)
        //   2. `Cabin.LockStatus.LastRequestor` (FeatureId string)
        //
        // We track the latest value of each in a local cache and
        // arm on every `LastRequestor` tick — that signal is
        // published per command (no dedup), so consecutive presses
        // that resolve to the same `LockStatus` enum still trigger.
        // `biased` select! ordering ensures the LockStatus message
        // is processed before the LastRequestor message on the same
        // command, so the cache is up-to-date when we evaluate.
        let mut status_stream = self.bus.subscribe(LOCK_STATUS).await;
        let mut requestor_stream = self.bus.subscribe(LOCK_LAST_REQUESTOR).await;
        // Local caches for the current LockStatus + LastRequestor.
        let mut status_cache: Option<String> = None;

        // Subscribe to crash detection signal
        let mut crash_stream = self.bus.subscribe(CRASH_SIGNAL).await;

        // Subscribe to power state for crash recovery
        let mut power_stream = self.bus.subscribe(POWER_STATE_SIGNAL).await;

        loop {
            // ==== ENABLED MODE: normal relock logic ====

            // Phase 1: Wait for a qualifying lock event.  Listen to
            // both LockStatus and LastRequestor; the LastRequestor
            // tick is the per-event trigger.  `biased` ordering
            // ensures the LockStatus update is processed first when
            // both arrive on the same command.
            //
            // We also drain the lock_stream and open_stream here,
            // discarding values — they're meaningful only inside
            // Phase 2.  Without this drain, IsLocked=true messages
            // from a *previous* LOCK command would still be queued
            // in the broadcast buffer when Phase 2 starts, causing
            // an immediate spurious "AlreadyLocked" cancel.
            let phase1 = loop {
                select! {
                    biased;
                    Some(val) = crash_stream.next() => {
                        if val == SignalValue::Bool(true) {
                            break Phase1Result::CrashDetected;
                        }
                    }
                    Some(val) = status_stream.next() => {
                        if let SignalValue::String(s) = val {
                            status_cache = Some(s);
                        }
                    }
                    Some(val) = requestor_stream.next() => {
                        if let SignalValue::String(req) = val {
                            if event_should_arm(status_cache.as_deref(), &req) {
                                break Phase1Result::Unlocked;
                            }
                        }
                    }
                    Some(_) = lock_stream.next() => {
                        // Drain.  Phase 2 cares about IsLocked transitions
                        // *during the timer*; backlog from before Phase 2
                        // started is noise.
                    }
                    Some(_) = open_stream.next() => {
                        // Drain (same reasoning).
                    }
                    else => return, // bus closed
                }
            };

            if let Phase1Result::CrashDetected = phase1 {
                tracing::warn!("AutoRelock: crash detected (idle), entering DISABLED state");
                self.wait_for_power_cycle(&mut crash_stream, &mut power_stream)
                    .await;
                continue; // Re-enter phase 1 in ENABLED mode
            }

            tracing::info!("AutoRelock: unlock detected, starting relock timer");

            // Advertise to the HMI / consumers: timer is armed, here's the
            // configured timeout.
            let _ = self
                .bus
                .publish(
                    STATUS_TIMEOUT_SECS,
                    SignalValue::Uint16(self.timeout.as_secs() as u16),
                )
                .await;
            let _ = self
                .bus
                .publish(STATUS_IS_ARMED, SignalValue::Bool(true))
                .await;

            // Phase 2: Timer is running.  We also keep watching
            // status/requestor so a fresh qualifying unlock press
            // (e.g. user pressed RKE-unlock twice) restarts the
            // timer rather than letting the first press's deadline
            // expire while the user is still actively interacting.
            let timer_result = 'phase2: loop {
                let timer_result = loop {
                    select! {
                        biased;
                        _ = sleep(self.timeout) => {
                            break TimerOutcome::Expired;
                        }
                        Some(val) = crash_stream.next() => {
                            if val == SignalValue::Bool(true) {
                                break TimerOutcome::CrashDisable;
                            }
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
                            // IsLocked=false during an active timer is a no-op:
                            // a single user UNLOCK press causes the door-lock
                            // plant model to publish IsLocked=false for ALL
                            // doors in microseconds (passenger doors superlock-
                            // downgrade through the unlocked state).
                        }
                        Some(val) = status_stream.next() => {
                            if let SignalValue::String(s) = val {
                                status_cache = Some(s);
                            }
                        }
                        Some(val) = requestor_stream.next() => {
                            // A new external unlock press while the timer
                            // is running — restart the timer.
                            if let SignalValue::String(req) = val {
                                if event_should_arm(status_cache.as_deref(), &req) {
                                    tracing::info!("AutoRelock: new external unlock — restarting timer");
                                    break TimerOutcome::Restart;
                                }
                            }
                        }
                        else => return,
                    }
                };

                if matches!(timer_result, TimerOutcome::Restart) {
                    // Re-publish TimeoutSeconds so any HMI countdown
                    // that reads the latest value sees a fresh tick.
                    // The HMI also subscribes to `Cabin.LockStatus.EventNum`
                    // independently and re-stamps its visual timer on
                    // every qualifying unlock event — `IsArmed` stays
                    // `true` across restarts so this signal can stay
                    // a clean state, not an event channel.
                    let _ = self
                        .bus
                        .publish(
                            STATUS_TIMEOUT_SECS,
                            SignalValue::Uint16(self.timeout.as_secs() as u16),
                        )
                        .await;
                    continue 'phase2;
                }
                break 'phase2 timer_result;
            };

            // Whatever happens to the timer below, the armed status flag
            // must clear so the HMI banner hides.
            let _ = self
                .bus
                .publish(STATUS_IS_ARMED, SignalValue::Bool(false))
                .await;

            match timer_result {
                TimerOutcome::Expired => {
                    tracing::info!("AutoRelock: timeout expired, requesting LOCK");
                    if let Err(e) = self
                        .arbiter
                        .request(DoorLockRequest {
                            command: LockCommand::LockAll,
                            feature_id: FeatureId::AutoRelock,
                        })
                        .await
                    {
                        tracing::error!(error = %e, "AutoRelock: failed to submit LOCK");
                    }
                    // Auto-relock follows an external unlock — provide lock feedback.
                    let _ = self
                        .bus
                        .publish(FEEDBACK_REQUEST, SignalValue::String("lock".into()))
                        .await;
                }
                TimerOutcome::DoorOpened | TimerOutcome::AlreadyLocked => {
                    // Back to phase 1
                }
                TimerOutcome::Restart => {
                    // Unreachable here — `Restart` is consumed inside
                    // the Phase 2 loop with `continue 'phase2`, so it
                    // never escapes to this match.
                    unreachable!("Restart should be handled inside Phase 2 loop");
                }
                TimerOutcome::CrashDisable => {
                    tracing::warn!(
                        "AutoRelock: crash detected during timer, entering DISABLED state"
                    );
                    self.wait_for_power_cycle(&mut crash_stream, &mut power_stream)
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
    /// A second qualifying unlock arrived while the timer was running
    /// — restart the timer rather than cancel it.
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
    ) -> (
        Arc<MockBus>,
        Arc<DoorLockArbiter>,
        tokio::task::JoinHandle<()>,
    ) {
        let bus = Arc::new(MockBus::new());
        let (arbiter, _ack_tx, loop_fut) = door_lock_arbiter(Arc::clone(&bus));
        tokio::spawn(loop_fut);
        let arbiter = Arc::new(arbiter);

        let feature = AutoRelock::new(Arc::clone(&arbiter), Arc::clone(&bus)).with_timeout(timeout);
        let handle = tokio::spawn(feature.run());

        // Let everything start up
        tokio::task::yield_now().await;

        (bus, arbiter, handle)
    }

    #[tokio::test]
    async fn relock_after_timeout() {
        let (bus, _arb, _handle) = setup(Duration::from_millis(100)).await;

        bus.inject("Cabin.LockStatus", SignalValue::String("UNLOCKED".into()));
        bus.inject(
            "Cabin.LockStatus.LastRequestor",
            SignalValue::String("KeyfobRke".into()),
        );
        tokio::task::yield_now().await;

        sleep(Duration::from_millis(200)).await;
        tokio::task::yield_now().await;

        let history = bus.history();
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == "Body.Doors.CentralLock.Command"
                    && *val == SignalValue::String("lock_all".into())
            }),
            "expected AutoRelock to dispatch LOCK, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn door_opened_cancels_relock() {
        let (bus, _arb, _handle) = setup(Duration::from_millis(200)).await;

        bus.inject("Cabin.LockStatus", SignalValue::String("UNLOCKED".into()));
        bus.inject(
            "Cabin.LockStatus.LastRequestor",
            SignalValue::String("KeyfobRke".into()),
        );
        tokio::task::yield_now().await;

        sleep(Duration::from_millis(50)).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        tokio::task::yield_now().await;

        sleep(Duration::from_millis(300)).await;
        tokio::task::yield_now().await;

        let history = bus.history();
        assert!(
            !history.iter().any(|(sig, val)| {
                *sig == "Body.Doors.CentralLock.Command"
                    && *val == SignalValue::String("lock_all".into())
            }),
            "AutoRelock should NOT have dispatched LOCK after door opened, history: {:?}",
            history
        );
    }

    #[tokio::test]
    async fn external_relock_cancels_timer() {
        let (bus, _arb, _handle) = setup(Duration::from_millis(200)).await;

        bus.inject("Cabin.LockStatus", SignalValue::String("UNLOCKED".into()));
        bus.inject(
            "Cabin.LockStatus.LastRequestor",
            SignalValue::String("KeyfobRke".into()),
        );
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
                *sig == "Body.Doors.CentralLock.Command"
                    && *val == SignalValue::String("lock_all".into())
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
        bus.inject("Cabin.LockStatus", SignalValue::String("UNLOCKED".into()));
        bus.inject(
            "Cabin.LockStatus.LastRequestor",
            SignalValue::String("KeyfobRke".into()),
        );
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
                *sig == "Body.Doors.CentralLock.Command"
                    && *val == SignalValue::String("lock_all".into())
            }),
            "AutoRelock should NOT dispatch LOCK after crash, history: {:?}",
            history
        );

        // Feature should still be running (in DISABLED state, waiting for power cycle)
        assert!(
            !handle.is_finished(),
            "AutoRelock should be alive in DISABLED state"
        );

        // Simulate power cycle: OFF → ON
        bus.inject(POWER_STATE_SIGNAL, SignalValue::String("OFF".to_string()));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(10)).await;

        bus.inject(POWER_STATE_SIGNAL, SignalValue::String("ON".to_string()));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(10)).await;

        // Feature should still be running (re-enabled, back in phase 1)
        assert!(
            !handle.is_finished(),
            "AutoRelock should be re-enabled after power cycle"
        );

        // Now verify it works again: unlock → timeout → LOCK
        bus.clear_history();
        bus.inject("Cabin.LockStatus", SignalValue::String("UNLOCKED".into()));
        bus.inject(
            "Cabin.LockStatus.LastRequestor",
            SignalValue::String("KeyfobRke".into()),
        );
        tokio::task::yield_now().await;

        sleep(Duration::from_millis(600)).await;
        tokio::task::yield_now().await;

        let history = bus.history();
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == "Body.Doors.CentralLock.Command"
                    && *val == SignalValue::String("lock_all".into())
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
        assert!(
            !handle.is_finished(),
            "AutoRelock should be in DISABLED state"
        );

        // Unlock during DISABLED — should be ignored
        bus.inject("Cabin.LockStatus", SignalValue::String("UNLOCKED".into()));
        bus.inject(
            "Cabin.LockStatus.LastRequestor",
            SignalValue::String("KeyfobRke".into()),
        );
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(200)).await;
        tokio::task::yield_now().await;

        let history = bus.history();
        assert!(
            !history.iter().any(|(sig, val)| {
                *sig == "Body.Doors.CentralLock.Command"
                    && *val == SignalValue::String("lock_all".into())
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
        bus.inject(POWER_STATE_SIGNAL, SignalValue::String("OFF".to_string()));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(10)).await;

        bus.inject(POWER_STATE_SIGNAL, SignalValue::String("ACC".to_string()));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(10)).await;

        // Should still be disabled — ACC is not ON/START
        // (We can't easily assert DISABLED here, but we verify ON re-enables)

        bus.inject(POWER_STATE_SIGNAL, SignalValue::String("ON".to_string()));
        tokio::task::yield_now().await;
        sleep(Duration::from_millis(10)).await;

        // Now re-enabled — verify normal operation
        assert!(!handle.is_finished(), "AutoRelock should be re-enabled");

        bus.clear_history();
        bus.inject("Cabin.LockStatus", SignalValue::String("UNLOCKED".into()));
        bus.inject(
            "Cabin.LockStatus.LastRequestor",
            SignalValue::String("KeyfobRke".into()),
        );
        tokio::task::yield_now().await;

        sleep(Duration::from_millis(200)).await;
        tokio::task::yield_now().await;

        let history = bus.history();
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == "Body.Doors.CentralLock.Command"
                    && *val == SignalValue::String("lock_all".into())
            }),
            "AutoRelock should work after OFF → ACC → ON cycle, history: {:?}",
            history
        );
    }

    /// Door-IsLocked transitions alone (no LockStatus / LastRequestor
    /// trail) must not arm AutoRelock.  Soldier-knob movements are
    /// the practical case: they flip per-door IsLocked but never
    /// dispatch through the central-lock arbiter, so the
    /// LastRequestor signal is silent.
    #[tokio::test]
    async fn is_locked_edge_alone_does_not_arm() {
        let (bus, _arbiter, _h) = setup(Duration::from_millis(100)).await;

        // Simulate a soldier-knob movement that toggles IsLocked
        // without publishing through the arbiter.
        bus.inject("Body.Doors.Row1.Left.IsLocked", SignalValue::Bool(false));
        tokio::task::yield_now().await;

        // Wait well past the timeout.
        sleep(Duration::from_millis(300)).await;
        tokio::task::yield_now().await;

        let history = bus.history();
        assert!(
            !history.iter().any(|(sig, val)| {
                *sig == "Body.Doors.CentralLock.Command"
                    && *val == SignalValue::String("lock_all".into())
            }),
            "AutoRelock must NOT relock when only IsLocked toggles \
             (no LastRequestor); history: {:?}",
            history
        );
    }

    /// PEPS unlock arms AutoRelock — same as RKE, since PEPS is also
    /// an external unlock requestor.
    #[tokio::test]
    async fn passive_entry_unlock_arms_relock() {
        let (bus, _arbiter, _h) = setup(Duration::from_millis(100)).await;

        bus.inject("Cabin.LockStatus", SignalValue::String("UNLOCKED".into()));
        bus.inject(
            "Cabin.LockStatus.LastRequestor",
            SignalValue::String("PassiveEntry".into()),
        );
        sleep(Duration::from_millis(200)).await;
        tokio::task::yield_now().await;

        let history = bus.history();
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == "Body.Doors.CentralLock.Command"
                    && *val == SignalValue::String("lock_all".into())
            }),
            "AutoRelock should fire after PEPS unlock + timeout; history: {:?}",
            history
        );
    }

    /// A second unlock press (which resolves to the same `LockStatus`
    /// but bumps `LastRequestor`'s publish count) must restart the
    /// 45-second timer.  Without this behaviour, a user who unlocks
    /// twice in quick succession would see AutoRelock fire on the
    /// first press's timer, not the second.
    #[tokio::test]
    async fn repeated_unlock_restarts_timer() {
        let (bus, _arbiter, _h) = setup(Duration::from_millis(150)).await;

        // Press 1.
        bus.inject("Cabin.LockStatus", SignalValue::String("UNLOCKED".into()));
        bus.inject(
            "Cabin.LockStatus.LastRequestor",
            SignalValue::String("KeyfobRke".into()),
        );

        // Wait 100 ms — past half the timer but before the first
        // press's timer would expire.
        sleep(Duration::from_millis(100)).await;
        tokio::task::yield_now().await;

        // Press 2 — same status, fresh requestor publish.  This
        // should restart the timer from 0.
        bus.inject("Cabin.LockStatus", SignalValue::String("UNLOCKED".into()));
        bus.inject(
            "Cabin.LockStatus.LastRequestor",
            SignalValue::String("KeyfobRke".into()),
        );

        // Wait 100 ms — at this point, the FIRST press's 150 ms
        // timer would have fired (200 ms total elapsed), but the
        // SECOND press's timer still has 50 ms to go.  No relock
        // should have happened yet.
        sleep(Duration::from_millis(100)).await;
        tokio::task::yield_now().await;

        let mid_history = bus.history();
        assert!(
            !mid_history.iter().any(|(sig, val)| {
                *sig == "Body.Doors.CentralLock.Command"
                    && *val == SignalValue::String("lock_all".into())
            }),
            "AutoRelock fired too early — second press must restart \
             the timer; history: {:?}",
            mid_history
        );

        // Wait the rest of the second press's window.
        sleep(Duration::from_millis(100)).await;
        tokio::task::yield_now().await;

        let final_history = bus.history();
        assert!(
            final_history.iter().any(|(sig, val)| {
                *sig == "Body.Doors.CentralLock.Command"
                    && *val == SignalValue::String("lock_all".into())
            }),
            "AutoRelock should fire after the second press's timer \
             expires; history: {:?}",
            final_history
        );
    }

    /// Regression: an earlier LOCK command leaves `IsLocked=true`
    /// publishes buffered in lock_stream's broadcast channel.  When
    /// a subsequent UNLOCK arms AutoRelock and Phase 2 starts, the
    /// stale buffered messages must NOT be interpreted as a fresh
    /// "external relock" cancel.  Phase 1 must drain the lock /
    /// open streams while waiting.
    #[tokio::test]
    async fn stale_islocked_does_not_cancel_fresh_timer() {
        let (bus, _arbiter, _h) = setup(Duration::from_millis(150)).await;

        // Step 1: simulate a previous RKE LOCK that fills lock_stream
        // with IsLocked=true messages.  No requestor publish — this
        // is just background traffic AutoRelock should ignore.
        for sig in [
            "Body.Doors.Row1.Left.IsLocked",
            "Body.Doors.Row1.Right.IsLocked",
            "Body.Doors.Row2.Left.IsLocked",
            "Body.Doors.Row2.Right.IsLocked",
        ] {
            bus.inject(sig, SignalValue::Bool(true));
        }
        sleep(Duration::from_millis(20)).await;
        tokio::task::yield_now().await;

        // Step 2: now an external RKE unlock arms AutoRelock.
        bus.inject("Cabin.LockStatus", SignalValue::String("UNLOCKED".into()));
        bus.inject(
            "Cabin.LockStatus.LastRequestor",
            SignalValue::String("KeyfobRke".into()),
        );

        // Wait past the 150 ms timer.
        sleep(Duration::from_millis(200)).await;
        tokio::task::yield_now().await;

        // Timer must have fired LockAll — the stale IsLocked=true
        // backlog must NOT have cancelled it.
        let history = bus.history();
        assert!(
            history.iter().any(|(sig, val)| {
                *sig == "Body.Doors.CentralLock.Command"
                    && *val == SignalValue::String("lock_all".into())
            }),
            "Stale IsLocked=true must not cancel a fresh timer; history: {:?}",
            history
        );
    }

    /// Internal unlock requestors (DoorTrimButton, AutoLock, ...)
    /// must NOT arm AutoRelock — only external unlock paths qualify.
    #[tokio::test]
    async fn door_trim_button_unlock_does_not_arm() {
        let (bus, _arbiter, _h) = setup(Duration::from_millis(100)).await;

        bus.inject("Cabin.LockStatus", SignalValue::String("UNLOCKED".into()));
        bus.inject(
            "Cabin.LockStatus.LastRequestor",
            SignalValue::String("DoorTrimButton".into()),
        );
        sleep(Duration::from_millis(200)).await;
        tokio::task::yield_now().await;

        let history = bus.history();
        assert!(
            !history.iter().any(|(sig, val)| {
                *sig == "Body.Doors.CentralLock.Command"
                    && *val == SignalValue::String("lock_all".into())
            }),
            "Interior trim-button unlock must NOT arm AutoRelock; history: {:?}",
            history
        );
    }
}
