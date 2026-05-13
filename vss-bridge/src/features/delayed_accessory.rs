//! Delayed-accessory power feature.
//!
//! Publishes a single latched bool `Body.Power.DelayedAccessory.IsActive`
//! that accessory consumers (power windows, sunroof, radio, …) can
//! subscribe to as a gate.  Behaviour:
//!
//! | Power state (`Vehicle.LowVoltageSystemState`) | DAP active? |
//! |---|---|
//! | `ON` or `START` | yes — accessories powered directly |
//! | first transition into `ACC` / `OFF` / `LOCK` | yes — start a `timeout` (default 2 min) countdown |
//! | inside the countdown, no cabin door has opened yet | yes |
//! | a cabin door opens during the countdown | **no** — immediate kill |
//! | countdown elapses | **no** |
//!
//! Returning to `ON` / `START` cancels any in-flight countdown and
//! re-asserts DAP=true.  The feature is a single writer of the
//! output signal — no arbiter needed.
//!
//! # Boot behaviour
//!
//! At start-up we don't yet know the power state, so we publish
//! `false` once for a defined snapshot.  When the bus delivers the
//! current `LowVoltageSystemState` value the state machine re-runs
//! and the output flips to `true` if appropriate.
//!
//! # Test timing
//!
//! The timeout is parameterised via `with_timeout(Duration)`.
//! Tests use `tokio::time::pause` + a sub-second timeout.

use std::sync::Arc;
use std::time::Duration;

use futures::StreamExt;
use tokio::select;
use tokio::time::{sleep_until, Instant};

use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

const POWER_STATE: VssPath = "Vehicle.LowVoltageSystemState";
const DAP_OUT: VssPath = "Body.Power.DelayedAccessory.IsActive";
const DOOR_OPEN_SIGNALS: [VssPath; 4] = [
    "Body.Doors.Row1.Left.IsOpen",
    "Body.Doors.Row1.Right.IsOpen",
    "Body.Doors.Row2.Left.IsOpen",
    "Body.Doors.Row2.Right.IsOpen",
];

/// Default DAP countdown when ignition transitions off.  Short by
/// design — the demo favours quick verification.  Real OEMs use
/// ~5–10 min.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(120);

/// True when the power-state string indicates the ignition is in
/// any **on** position (engine running or cranking).  ACC / OFF /
/// LOCK / anything unrecognised counts as off.
fn is_ign_on(s: &str) -> bool {
    matches!(s, "ON" | "START")
}

pub struct DelayedAccessory<B: SignalBus> {
    bus: Arc<B>,
    timeout: Duration,
}

impl<B: SignalBus + Send + Sync + 'static> DelayedAccessory<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self {
            bus,
            timeout: DEFAULT_TIMEOUT,
        }
    }

    pub fn with_timeout(mut self, t: Duration) -> Self {
        self.timeout = t;
        self
    }

    pub async fn run(self) {
        tracing::info!(
            timeout_s = self.timeout.as_secs(),
            "DelayedAccessory feature started"
        );

        // Boot: publish a defined false so subscribers see something.
        // Will flip to true as soon as we learn the power state.
        let _ = self.bus.publish(DAP_OUT, SignalValue::Bool(false)).await;

        let mut power_rx = self.bus.subscribe(POWER_STATE).await;
        let mut door_streams: Vec<futures::stream::BoxStream<'static, SignalValue>> =
            Vec::with_capacity(DOOR_OPEN_SIGNALS.len());
        for &sig in DOOR_OPEN_SIGNALS.iter() {
            door_streams.push(self.bus.subscribe(sig).await);
        }

        // State.
        let mut ign_on: bool = false;
        let mut deadline: Option<Instant> = None;
        let mut dap_published: bool = false;

        loop {
            let next = deadline.unwrap_or_else(|| Instant::now() + Duration::from_secs(3600));
            let door_evt = futures::future::select_all(
                door_streams
                    .iter_mut()
                    .enumerate()
                    .map(|(i, s)| Box::pin(async move { (i, s.next().await) })),
            );

            select! {
                biased;
                _ = sleep_until(next) => {
                    if let Some(d) = deadline {
                        if Instant::now() >= d {
                            tracing::info!("DelayedAccessory: countdown expired");
                            deadline = None;
                            Self::publish(
                                &self.bus, &mut dap_published,
                                Self::compute(ign_on, deadline)).await;
                        }
                    }
                }
                Some(val) = power_rx.next() => {
                    if let SignalValue::String(s) = val {
                        let now_on = is_ign_on(&s);
                        if now_on != ign_on {
                            tracing::info!(power_state = %s, "DelayedAccessory: power-state change");
                            if !now_on {
                                // Transitioning into a non-on state —
                                // start the countdown.
                                deadline = Some(Instant::now() + self.timeout);
                            } else {
                                // Back to on — clear countdown.
                                deadline = None;
                            }
                            ign_on = now_on;
                            Self::publish(
                                &self.bus, &mut dap_published,
                                Self::compute(ign_on, deadline)).await;
                        }
                    }
                }
                ((door_idx, opt), _, _) = door_evt => {
                    if matches!(opt, Some(SignalValue::Bool(true))) {
                        // Door open during the countdown kills DAP.
                        if !ign_on && deadline.is_some() {
                            tracing::info!(
                                door = DOOR_OPEN_SIGNALS[door_idx],
                                "DelayedAccessory: door opened — killing delayed window"
                            );
                            deadline = None;
                            Self::publish(
                                &self.bus, &mut dap_published,
                                Self::compute(ign_on, deadline)).await;
                        }
                    }
                }
                else => break,
            }
        }
        tracing::warn!("DelayedAccessory: streams closed, exiting");
    }

    /// DAP active iff the ignition is on OR a countdown is running.
    fn compute(ign_on: bool, deadline: Option<Instant>) -> bool {
        ign_on || deadline.is_some()
    }

    async fn publish(bus: &Arc<B>, last: &mut bool, want: bool) {
        if *last == want {
            return;
        }
        *last = want;
        let _ = bus.publish(DAP_OUT, SignalValue::Bool(want)).await;
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use tokio::time::advance;

    async fn settle() {
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
        tokio::time::sleep(Duration::from_millis(1)).await;
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
    }

    async fn setup(timeout: Duration) -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        tokio::spawn(
            DelayedAccessory::new(Arc::clone(&bus))
                .with_timeout(timeout)
                .run(),
        );
        settle().await;
        bus
    }

    fn dap(bus: &MockBus) -> Option<bool> {
        match bus.latest_value(DAP_OUT) {
            Some(SignalValue::Bool(b)) => Some(b),
            _ => None,
        }
    }

    #[tokio::test]
    async fn boots_inactive() {
        let bus = setup(Duration::from_secs(60)).await;
        assert_eq!(dap(&bus), Some(false));
    }

    #[tokio::test]
    async fn ignition_on_activates() {
        let bus = setup(Duration::from_secs(60)).await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle().await;
        assert_eq!(dap(&bus), Some(true));
    }

    #[tokio::test]
    async fn start_activates_too() {
        let bus = setup(Duration::from_secs(60)).await;
        bus.inject(POWER_STATE, SignalValue::String("START".into()));
        settle().await;
        assert_eq!(dap(&bus), Some(true));
    }

    #[tokio::test]
    async fn acc_treated_as_off() {
        // Per user spec: ACC and OFF behave the same — start countdown.
        let bus = setup(Duration::from_secs(60)).await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle().await;
        bus.inject(POWER_STATE, SignalValue::String("ACC".into()));
        settle().await;
        // Inside countdown.
        assert_eq!(dap(&bus), Some(true));
    }

    #[tokio::test(start_paused = true)]
    async fn timer_expires_after_timeout() {
        let bus = setup(Duration::from_millis(100)).await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle().await;
        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        settle().await;
        assert_eq!(dap(&bus), Some(true));
        advance(Duration::from_millis(200)).await;
        settle().await;
        assert_eq!(dap(&bus), Some(false));
    }

    #[tokio::test(start_paused = true)]
    async fn door_open_kills_dap() {
        let bus = setup(Duration::from_secs(60)).await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle().await;
        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        settle().await;
        assert_eq!(dap(&bus), Some(true));
        bus.inject(DOOR_OPEN_SIGNALS[0], SignalValue::Bool(true));
        settle().await;
        assert_eq!(dap(&bus), Some(false));
    }

    #[tokio::test(start_paused = true)]
    async fn returning_to_on_cancels_timer() {
        let bus = setup(Duration::from_millis(100)).await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle().await;
        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        settle().await;
        // Within countdown.
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle().await;
        // Advance way past the original timeout — should still be on.
        advance(Duration::from_millis(500)).await;
        settle().await;
        assert_eq!(dap(&bus), Some(true));
    }

    #[tokio::test(start_paused = true)]
    async fn door_open_while_ignition_on_does_not_kill() {
        // Door-open is only a kill signal during the post-off countdown.
        let bus = setup(Duration::from_millis(100)).await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle().await;
        bus.inject(DOOR_OPEN_SIGNALS[0], SignalValue::Bool(true));
        settle().await;
        assert_eq!(dap(&bus), Some(true));
    }

    #[tokio::test(start_paused = true)]
    async fn idempotent_publishes() {
        let bus = setup(Duration::from_millis(100)).await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle().await;
        bus.clear_history();
        // Same state again — must not republish.
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle().await;
        let n = bus.history().iter().filter(|(s, _)| *s == DAP_OUT).count();
        assert_eq!(n, 0, "DAP must not republish on no-op transitions");
    }
}
