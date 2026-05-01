//! Hood plant model — tri-state latch matching real-vehicle mechanics.
//!
//! ```text
//!  HMI dash hood-release lever            HMI top-view click on hood
//!      │  Body.Switches.Hood.Release.IsPulled │ Body.Hood.OpenCmd
//!      │  (rising-edge pulses)                │ Body.Hood.CloseCmd
//!      ▼                                      ▼
//!  HoodPlantModel          ← this module
//!      │  publishes:
//!      │    Body.Hood.LatchState  ("LATCHED" | "HALF_LATCHED" | "OPEN")
//!      │    Body.Hood.IsOpen      (true iff LatchState == "OPEN")
//!      ▼
//!  SignalBus → WsBridge → HMI
//! ```
//!
//! # State machine
//!
//! | State          | Event                | Next state           |
//! |----------------|----------------------|----------------------|
//! | `LATCHED`      | first pull           | `LATCHED` + 3 s timer|
//! | `LATCHED` t/o  | second pull in 3 s   | `HALF_LATCHED`       |
//! | `LATCHED` t/o  | timer expires        | `LATCHED`            |
//! | `HALF_LATCHED` | OpenCmd              | `OPEN`               |
//! | `HALF_LATCHED` | CloseCmd             | (ignored)            |
//! | `HALF_LATCHED` | pull                 | (ignored)            |
//! | `OPEN`         | CloseCmd             | `LATCHED`            |
//! | `OPEN`         | OpenCmd / pull       | (ignored)            |
//!
//! `HALF_LATCHED → LATCHED` direct is **not** allowed: real hoods need
//! a gravity drop from the open position to engage both pawls.  Users
//! who half-latch by mistake must `OpenCmd` first then `CloseCmd`.
//!
//! # `IsOpen` companion signal
//!
//! `Body.Hood.IsOpen` is published as `true` iff `LatchState == OPEN`.
//! Other features (alarm, dome-on-open, etc.) can ignore the
//! tri-state and just consume the bool.
//!
//! # NVM persistence
//!
//! When constructed via [`HoodPlantModel::with_nvm`], the full
//! `LatchState` is persisted on every change and re-read at boot so a
//! half-latched hood survives a power cycle.

use std::sync::Arc;
use std::time::{Duration, Instant};

use futures::StreamExt;
use tokio::select;
use tokio::time::sleep_until;

use crate::ipc_message::SignalValue;
use crate::nvm::{HoodState, NvmStore};
use crate::signal_bus::SignalBus;

const OPEN_CMD: &str = "Body.Hood.OpenCmd";
const CLOSE_CMD: &str = "Body.Hood.CloseCmd";
const RELEASE_PULL: &str = "Body.Switches.Hood.Release.IsPulled";
const IS_OPEN: &str = "Body.Hood.IsOpen";
const LATCH_STATE: &str = "Body.Hood.LatchState";

/// Window between the two release-lever pulls required to advance
/// from `LATCHED` to `HALF_LATCHED`.
const DOUBLE_PULL_WINDOW: Duration = Duration::from_secs(3);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Latch {
    Latched,
    HalfLatched,
    Open,
}

impl Latch {
    fn as_str(self) -> &'static str {
        match self {
            Self::Latched => "LATCHED",
            Self::HalfLatched => "HALF_LATCHED",
            Self::Open => "OPEN",
        }
    }

    fn parse(s: &str) -> Self {
        match s {
            "HALF_LATCHED" => Self::HalfLatched,
            "OPEN" => Self::Open,
            _ => Self::Latched,
        }
    }
}

pub struct HoodPlantModel<B: SignalBus> {
    bus: Arc<B>,
    state: Latch,
    /// When `Some`, a first pull was received and the timer expires
    /// at this instant.  Cleared on the second pull (advance) or on
    /// expiry.
    first_pull_deadline: Option<Instant>,
    nvm: Option<NvmStore>,
}

impl<B: SignalBus + Send + Sync + 'static> HoodPlantModel<B> {
    /// Create in volatile mode — no NVM, boots latched.
    pub fn new(bus: Arc<B>) -> Self {
        Self {
            bus,
            state: Latch::Latched,
            first_pull_deadline: None,
            nvm: None,
        }
    }

    /// Production constructor — boots from NVM (full tri-state) and
    /// persists every transition.
    pub fn with_nvm(bus: Arc<B>, nvm: NvmStore) -> Self {
        let persisted = nvm.load_hood();
        let state = Latch::parse(&persisted.latch_state);
        tracing::info!(
            latch_state = state.as_str(),
            "HoodPlantModel: booted from NVM"
        );
        Self {
            bus,
            state,
            first_pull_deadline: None,
            nvm: Some(nvm),
        }
    }

    fn save_to_nvm(&self) {
        if let Some(nvm) = &self.nvm {
            nvm.save_hood(&HoodState {
                latch_state: self.state.as_str().into(),
            });
        }
    }

    /// Publish both `LatchState` and the `IsOpen` companion bool.
    /// Centralised so the two stay in lockstep on every transition.
    async fn publish_state(&self) {
        let _ = self
            .bus
            .publish(LATCH_STATE, SignalValue::String(self.state.as_str().into()))
            .await;
        let _ = self
            .bus
            .publish(IS_OPEN, SignalValue::Bool(self.state == Latch::Open))
            .await;
    }

    /// Apply the state machine for a single event.  Returns true if
    /// the state changed (caller will publish + persist).
    fn apply(&mut self, event: HoodEvent) -> bool {
        match (self.state, event) {
            (Latch::Latched, HoodEvent::Pull) => {
                if self.first_pull_deadline.is_some() {
                    // Second pull within window — advance to HALF.
                    tracing::info!("Hood: double-pull confirmed — HALF_LATCHED");
                    self.first_pull_deadline = None;
                    self.state = Latch::HalfLatched;
                    true
                } else {
                    // First pull — start the 3 s window.
                    tracing::info!("Hood: first pull — waiting for second within 3 s");
                    self.first_pull_deadline = Some(Instant::now() + DOUBLE_PULL_WINDOW);
                    false
                }
            }
            (Latch::HalfLatched, HoodEvent::OpenCmd) => {
                tracing::info!("Hood: OpenCmd — HALF_LATCHED → OPEN");
                self.state = Latch::Open;
                true
            }
            (Latch::Open, HoodEvent::CloseCmd) => {
                tracing::info!("Hood: CloseCmd — OPEN → LATCHED");
                self.state = Latch::Latched;
                true
            }
            // Explicit no-ops so a future contributor sees the rules:
            (Latch::HalfLatched, HoodEvent::CloseCmd) => {
                tracing::info!(
                    "Hood: CloseCmd from HALF_LATCHED ignored — must fully open first \
                     (gravity drop required to re-engage primary latch)"
                );
                false
            }
            (Latch::HalfLatched, HoodEvent::Pull)
            | (Latch::Open, HoodEvent::Pull)
            | (Latch::Open, HoodEvent::OpenCmd)
            | (Latch::Latched, HoodEvent::OpenCmd)
            | (Latch::Latched, HoodEvent::CloseCmd) => false,
        }
    }

    pub async fn run(mut self) {
        let mut open_rx = self.bus.subscribe(OPEN_CMD).await;
        let mut close_rx = self.bus.subscribe(CLOSE_CMD).await;
        let mut pull_rx = self.bus.subscribe(RELEASE_PULL).await;

        // Publish boot state.
        self.publish_state().await;
        tracing::info!(latch_state = self.state.as_str(), "HoodPlantModel started");

        loop {
            // Build the timer future for the double-pull window — fires
            // only when the deadline is set; otherwise pending forever
            // (so the select! never wakes on it spuriously).
            let timer = async {
                match self.first_pull_deadline {
                    Some(d) => sleep_until(tokio::time::Instant::from_std(d)).await,
                    None => std::future::pending::<()>().await,
                }
            };

            select! {
                Some(val) = pull_rx.next() => {
                    if matches!(val, SignalValue::Bool(true)) && self.apply(HoodEvent::Pull) {
                        self.publish_state().await;
                        self.save_to_nvm();
                    }
                }
                Some(val) = open_rx.next() => {
                    if matches!(val, SignalValue::Bool(true)) && self.apply(HoodEvent::OpenCmd) {
                        self.publish_state().await;
                        self.save_to_nvm();
                    }
                }
                Some(val) = close_rx.next() => {
                    if matches!(val, SignalValue::Bool(true)) && self.apply(HoodEvent::CloseCmd) {
                        self.publish_state().await;
                        self.save_to_nvm();
                    }
                }
                _ = timer => {
                    tracing::info!("Hood: double-pull window expired — single pull discarded");
                    self.first_pull_deadline = None;
                    // No state change → no publish.
                }
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
enum HoodEvent {
    Pull,
    OpenCmd,
    CloseCmd,
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use tempfile::TempDir;
    use tokio::time::sleep;

    async fn settle() {
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    fn latch(bus: &MockBus) -> Option<String> {
        match bus.latest_value(LATCH_STATE) {
            Some(SignalValue::String(s)) => Some(s),
            _ => None,
        }
    }

    async fn pulse(bus: &MockBus, sig: &'static str) {
        bus.inject(sig, SignalValue::Bool(true));
        settle().await;
        bus.inject(sig, SignalValue::Bool(false));
        settle().await;
    }

    async fn boot() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        tokio::spawn(HoodPlantModel::new(Arc::clone(&bus)).run());
        settle().await;
        bus
    }

    #[tokio::test]
    async fn single_pull_does_not_advance_state() {
        let bus = boot().await;
        pulse(&bus, RELEASE_PULL).await;

        // Still LATCHED.  The first-pull timer is running internally
        // but the public LatchState is unchanged.
        assert_eq!(latch(&bus).as_deref(), Some("LATCHED"));
        assert_eq!(bus.latest_value(IS_OPEN), Some(SignalValue::Bool(false)));
    }

    #[tokio::test]
    async fn two_pulls_within_window_half_latch() {
        let bus = boot().await;
        pulse(&bus, RELEASE_PULL).await;
        pulse(&bus, RELEASE_PULL).await;

        assert_eq!(latch(&bus).as_deref(), Some("HALF_LATCHED"));
        // IsOpen stays false — half-latched is still mechanically closed.
        assert_eq!(bus.latest_value(IS_OPEN), Some(SignalValue::Bool(false)));
    }

    #[tokio::test]
    async fn second_pull_after_window_does_not_advance() {
        let bus = boot().await;
        pulse(&bus, RELEASE_PULL).await;

        // Wait past the 3 s window.
        sleep(DOUBLE_PULL_WINDOW + Duration::from_millis(200)).await;
        settle().await;

        pulse(&bus, RELEASE_PULL).await;
        // The second pull is treated as a fresh first pull — still LATCHED.
        assert_eq!(latch(&bus).as_deref(), Some("LATCHED"));
    }

    #[tokio::test]
    async fn open_cmd_from_latched_is_ignored() {
        let bus = boot().await;
        pulse(&bus, OPEN_CMD).await;
        assert_eq!(latch(&bus).as_deref(), Some("LATCHED"));
    }

    #[tokio::test]
    async fn open_cmd_from_half_latched_advances_to_open() {
        let bus = boot().await;
        pulse(&bus, RELEASE_PULL).await;
        pulse(&bus, RELEASE_PULL).await;
        assert_eq!(latch(&bus).as_deref(), Some("HALF_LATCHED"));

        pulse(&bus, OPEN_CMD).await;
        assert_eq!(latch(&bus).as_deref(), Some("OPEN"));
        assert_eq!(bus.latest_value(IS_OPEN), Some(SignalValue::Bool(true)));
    }

    #[tokio::test]
    async fn close_cmd_from_half_latched_is_ignored() {
        // Real-vehicle behaviour: pushing down on a half-latched hood
        // doesn't fully relatch it — the latches need a gravity drop
        // from the open position.
        let bus = boot().await;
        pulse(&bus, RELEASE_PULL).await;
        pulse(&bus, RELEASE_PULL).await;
        pulse(&bus, CLOSE_CMD).await;

        assert_eq!(
            latch(&bus).as_deref(),
            Some("HALF_LATCHED"),
            "CloseCmd from HALF_LATCHED must NOT return to LATCHED"
        );
    }

    #[tokio::test]
    async fn close_cmd_from_open_returns_to_latched() {
        let bus = boot().await;
        pulse(&bus, RELEASE_PULL).await;
        pulse(&bus, RELEASE_PULL).await;
        pulse(&bus, OPEN_CMD).await;
        pulse(&bus, CLOSE_CMD).await;

        assert_eq!(latch(&bus).as_deref(), Some("LATCHED"));
        assert_eq!(bus.latest_value(IS_OPEN), Some(SignalValue::Bool(false)));
    }

    #[tokio::test]
    async fn pull_while_open_or_half_latched_is_ignored() {
        // From OPEN.
        let bus = boot().await;
        pulse(&bus, RELEASE_PULL).await;
        pulse(&bus, RELEASE_PULL).await;
        pulse(&bus, OPEN_CMD).await;
        assert_eq!(latch(&bus).as_deref(), Some("OPEN"));
        pulse(&bus, RELEASE_PULL).await;
        assert_eq!(
            latch(&bus).as_deref(),
            Some("OPEN"),
            "pull while OPEN must no-op"
        );

        // From HALF_LATCHED.
        let bus2 = boot().await;
        pulse(&bus2, RELEASE_PULL).await;
        pulse(&bus2, RELEASE_PULL).await;
        assert_eq!(latch(&bus2).as_deref(), Some("HALF_LATCHED"));
        pulse(&bus2, RELEASE_PULL).await;
        assert_eq!(
            latch(&bus2).as_deref(),
            Some("HALF_LATCHED"),
            "pull while HALF_LATCHED must no-op"
        );
    }

    #[tokio::test]
    async fn nvm_round_trip_persists_half_latched() {
        let dir = TempDir::new().unwrap();
        let nvm = NvmStore::with_path(dir.path());
        let bus = Arc::new(MockBus::new());

        let h = tokio::spawn(HoodPlantModel::with_nvm(Arc::clone(&bus), nvm.clone()).run());
        settle().await;
        pulse(&bus, RELEASE_PULL).await;
        pulse(&bus, RELEASE_PULL).await;
        assert_eq!(latch(&bus).as_deref(), Some("HALF_LATCHED"));
        h.abort();

        // Fresh boot — same NVM, new bus.  Boot state must reflect the
        // half-latched hood (a power cycle while popped should preserve it).
        let bus2 = Arc::new(MockBus::new());
        tokio::spawn(HoodPlantModel::with_nvm(Arc::clone(&bus2), nvm).run());
        settle().await;
        assert_eq!(latch(&bus2).as_deref(), Some("HALF_LATCHED"));
    }
}
