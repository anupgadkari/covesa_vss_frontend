//! Mirror-fold motor plant model.
//!
//! Simulates the per-side fold motors that physically swing the
//! exterior mirrors against the door (parking) or out (driving):
//!
//! ```text
//!  MirrorFold feature
//!      │  Body.Mirror.{Left,Right}.FoldCmd  (bool)
//!      ▼
//!  MirrorFoldPlantModel          ← this module
//!      │  publishes Body.Mirror.{Left,Right}.IsFolded
//!      │            (1 s after each command settles)
//!      │  persists  is_folded[2] + last_fold_cmd in NVM
//!      ▼
//!  SignalBus → WsBridge → HMI; PuddleArbiter (PhysicalGate)
//! ```
//!
//! # Per-side independence
//!
//! Each side has its own 1 s timer.  A new command on one side preempts
//! that side's in-flight timer (matching real motor behaviour — the
//! controller reverses immediately on a direction change).  The other
//! side is unaffected.
//!
//! # NVM persistence
//!
//! Per-side fold positions survive a power cycle.  The MirrorFold
//! feature persists `last_fold_cmd` separately
//! ([`crate::nvm::MirrorFoldIntent`]) — keeping the plant model
//! purely about physics.
//!
//! Cold boot (no NVM file) = factory: both unfolded.
//!
//! # Signal ownership
//!
//! `Body.Mirror.{Left,Right}.IsFolded` is **owned** by this plant
//! model.  The HMI must not write to it directly (those signals were
//! removed from `INPUT_SIGNALS`).  Per the project's signal-ownership
//! rules, only the plant model publishes the feedback — features
//! request changes via `FoldCmd`.

use std::sync::Arc;
use std::time::Duration;

use futures::stream::BoxStream;
use futures::StreamExt;
use tokio::select;
use tokio::time::{sleep, Instant};

use crate::ipc_message::SignalValue;
use crate::nvm::{MirrorFoldState, NvmStore};
use crate::signal_bus::SignalBus;

const FOLD_CMD_LEFT: &str = "Body.Mirror.Left.FoldCmd";
const FOLD_CMD_RIGHT: &str = "Body.Mirror.Right.FoldCmd";
const IS_FOLDED_LEFT: &str = "Body.Mirror.Left.IsFolded";
const IS_FOLDED_RIGHT: &str = "Body.Mirror.Right.IsFolded";

/// Time the motor takes to swing the mirror.  Real motors are 0.7–1.5 s
/// depending on temperature; 1 s is a representative midpoint.
pub const MOTOR_SETTLE: Duration = Duration::from_secs(1);

/// Per-side travel state.
#[derive(Debug, Clone, Copy)]
struct SideState {
    /// Current mechanical position (post-settle).
    is_folded: bool,
    /// In-flight target if a command is currently travelling.
    /// `None` ⇒ side is at rest.
    in_flight_target: Option<bool>,
    /// `Instant` at which the in-flight target lands.  Only meaningful
    /// when `in_flight_target.is_some()`.
    settle_at: Instant,
}

impl SideState {
    fn at_rest(is_folded: bool) -> Self {
        Self {
            is_folded,
            in_flight_target: None,
            // Placeholder — never read while in_flight_target is None.
            settle_at: Instant::now(),
        }
    }
}

pub struct MirrorFoldPlantModel<B: SignalBus> {
    bus: Arc<B>,
    state: [SideState; 2], // [left, right]
    nvm: Option<NvmStore>,
    /// Override settle time for tests (defaults to `MOTOR_SETTLE`).
    settle: Duration,
}

impl<B: SignalBus + Send + Sync + 'static> MirrorFoldPlantModel<B> {
    /// Volatile constructor — boots both sides unfolded, no NVM.
    pub fn new(bus: Arc<B>) -> Self {
        Self {
            bus,
            state: [SideState::at_rest(false), SideState::at_rest(false)],
            nvm: None,
            settle: MOTOR_SETTLE,
        }
    }

    /// Production constructor — restores per-side fold positions from
    /// NVM and persists on every settle.
    pub fn with_nvm(bus: Arc<B>, nvm: NvmStore) -> Self {
        let st = nvm.load_mirror_fold();
        tracing::info!(
            left = st.is_folded[0],
            right = st.is_folded[1],
            "MirrorFoldPlantModel: booted from NVM"
        );
        Self {
            bus,
            state: [
                SideState::at_rest(st.is_folded[0]),
                SideState::at_rest(st.is_folded[1]),
            ],
            nvm: Some(nvm),
            settle: MOTOR_SETTLE,
        }
    }

    /// Test hook: shorten the motor settle time.
    pub fn with_settle(mut self, settle: Duration) -> Self {
        self.settle = settle;
        self
    }

    fn save_to_nvm(&self) {
        if let Some(nvm) = &self.nvm {
            nvm.save_mirror_fold(&MirrorFoldState {
                is_folded: [self.state[0].is_folded, self.state[1].is_folded],
            });
        }
    }

    /// Publish initial feedback so subscribers (HMI, PuddleArbiter
    /// gates) see the current state on boot rather than waiting for
    /// the first command.
    async fn publish_initial(&self) {
        for (idx, sig) in [IS_FOLDED_LEFT, IS_FOLDED_RIGHT].iter().enumerate() {
            if let Err(e) = self
                .bus
                .publish(sig, SignalValue::Bool(self.state[idx].is_folded))
                .await
            {
                tracing::warn!(signal = sig, error = %e, "MirrorFoldPlantModel: failed initial publish");
            }
        }
    }

    pub async fn run(mut self) {
        tracing::info!("MirrorFoldPlantModel started");

        let mut cmd_streams: [BoxStream<'static, SignalValue>; 2] = [
            self.bus.subscribe(FOLD_CMD_LEFT).await,
            self.bus.subscribe(FOLD_CMD_RIGHT).await,
        ];

        self.publish_initial().await;

        loop {
            // Build a deadline future for whichever side(s) have an
            // in-flight target.  If neither does, sleep effectively
            // forever — only command events will wake us.
            let next_deadline: Option<Instant> = self
                .state
                .iter()
                .filter_map(|s| s.in_flight_target.map(|_| s.settle_at))
                .min();
            let timer_sleep = match next_deadline {
                Some(at) => at.saturating_duration_since(Instant::now()),
                None => Duration::from_secs(3600),
            };

            let cmd_event = futures::future::select_all(
                cmd_streams
                    .iter_mut()
                    .enumerate()
                    .map(|(i, s)| Box::pin(async move { (i, s.next().await) })),
            );

            select! {
                ((side, opt), _, _) = cmd_event => {
                    let target = match opt {
                        Some(SignalValue::Bool(b)) => b,
                        _ => continue,
                    };
                    self.handle_cmd(side, target).await;
                }
                _ = sleep(timer_sleep) => {
                    self.process_settles().await;
                }
            }
        }
    }

    /// Accept a per-side fold command.  Preempts any in-flight target
    /// on this side; restarts the settle timer.
    async fn handle_cmd(&mut self, side: usize, target: bool) {
        // No-op if already at target and not in flight to a different
        // value — saves NVM thrash on duplicate publishes.
        if self.state[side].in_flight_target.is_none() && self.state[side].is_folded == target {
            tracing::debug!(
                side,
                target,
                "MirrorFold plant: command ignored (already at target)"
            );
        } else {
            tracing::info!(
                side,
                target,
                preempt = self.state[side].in_flight_target.is_some(),
                "MirrorFold plant: command accepted"
            );
            self.state[side].in_flight_target = Some(target);
            self.state[side].settle_at = Instant::now() + self.settle;
        }
    }

    /// Walk through both sides; commit any in-flight targets that have
    /// reached their settle deadline.
    async fn process_settles(&mut self) {
        let now = Instant::now();
        for side in 0..2 {
            if let Some(target) = self.state[side].in_flight_target {
                if now >= self.state[side].settle_at {
                    let prev = self.state[side].is_folded;
                    self.state[side].is_folded = target;
                    self.state[side].in_flight_target = None;
                    if prev != target {
                        let sig = if side == 0 {
                            IS_FOLDED_LEFT
                        } else {
                            IS_FOLDED_RIGHT
                        };
                        if let Err(e) = self.bus.publish(sig, SignalValue::Bool(target)).await {
                            tracing::warn!(side, signal = sig, error = %e, "MirrorFoldPlantModel: publish failed");
                        }
                        tracing::info!(side, is_folded = target, "MirrorFold plant: side settled");
                        self.save_to_nvm();
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use std::time::Duration;
    use tempfile::tempdir;
    use tokio::time::advance;

    async fn settle_yields() {
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    fn nvm_in(dir: &std::path::Path) -> NvmStore {
        NvmStore::with_path(dir)
    }

    #[tokio::test(start_paused = true)]
    async fn fold_command_publishes_after_one_second() {
        let bus = Arc::new(MockBus::new());
        let plant =
            MirrorFoldPlantModel::new(Arc::clone(&bus)).with_settle(Duration::from_millis(100));
        tokio::spawn(plant.run());
        settle_yields().await;

        bus.inject(FOLD_CMD_LEFT, SignalValue::Bool(true));
        settle_yields().await;
        // Not yet — we haven't advanced time.
        assert_eq!(
            bus.latest_value(IS_FOLDED_LEFT),
            Some(SignalValue::Bool(false)),
            "feedback should still be unfolded before settle"
        );

        advance(Duration::from_millis(120)).await;
        settle_yields().await;
        assert_eq!(
            bus.latest_value(IS_FOLDED_LEFT),
            Some(SignalValue::Bool(true)),
            "feedback should flip to folded after settle"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn new_command_preempts_in_flight_target() {
        let bus = Arc::new(MockBus::new());
        let plant =
            MirrorFoldPlantModel::new(Arc::clone(&bus)).with_settle(Duration::from_millis(100));
        tokio::spawn(plant.run());
        settle_yields().await;

        // Fold the left mirror.
        bus.inject(FOLD_CMD_LEFT, SignalValue::Bool(true));
        settle_yields().await;

        // 50 ms in (mid-travel), reverse to unfold.
        advance(Duration::from_millis(50)).await;
        settle_yields().await;
        bus.inject(FOLD_CMD_LEFT, SignalValue::Bool(false));
        settle_yields().await;

        // 50 ms after the original fold (motor still travelling toward
        // unfold target — full settle hasn't elapsed since reversal).
        advance(Duration::from_millis(60)).await;
        settle_yields().await;
        // Still unfolded (settled there before, never reached folded).
        assert_eq!(
            bus.latest_value(IS_FOLDED_LEFT),
            Some(SignalValue::Bool(false))
        );

        // Settle the unfold target.
        advance(Duration::from_millis(120)).await;
        settle_yields().await;
        assert_eq!(
            bus.latest_value(IS_FOLDED_LEFT),
            Some(SignalValue::Bool(false)),
            "preempted command should land at the new target, not the old"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn sides_settle_independently() {
        let bus = Arc::new(MockBus::new());
        let plant =
            MirrorFoldPlantModel::new(Arc::clone(&bus)).with_settle(Duration::from_millis(100));
        tokio::spawn(plant.run());
        settle_yields().await;

        bus.inject(FOLD_CMD_LEFT, SignalValue::Bool(true));
        settle_yields().await;
        advance(Duration::from_millis(50)).await;
        settle_yields().await;
        bus.inject(FOLD_CMD_RIGHT, SignalValue::Bool(true));
        settle_yields().await;

        // After 60 ms more (110 ms after left, 60 ms after right),
        // left has settled but right has not.
        advance(Duration::from_millis(60)).await;
        settle_yields().await;
        assert_eq!(
            bus.latest_value(IS_FOLDED_LEFT),
            Some(SignalValue::Bool(true))
        );
        assert_eq!(
            bus.latest_value(IS_FOLDED_RIGHT),
            Some(SignalValue::Bool(false)),
            "right should still be travelling"
        );

        advance(Duration::from_millis(60)).await;
        settle_yields().await;
        assert_eq!(
            bus.latest_value(IS_FOLDED_RIGHT),
            Some(SignalValue::Bool(true))
        );
    }

    #[tokio::test(start_paused = true)]
    async fn nvm_restores_state_on_boot() {
        let dir = tempdir().unwrap();
        let nvm = nvm_in(dir.path());

        // Pre-seed NVM with mismatched state.
        nvm.save_mirror_fold(&MirrorFoldState {
            is_folded: [true, false],
        });

        let bus = Arc::new(MockBus::new());
        let plant = MirrorFoldPlantModel::with_nvm(Arc::clone(&bus), nvm)
            .with_settle(Duration::from_millis(100));
        tokio::spawn(plant.run());
        settle_yields().await;

        assert_eq!(
            bus.latest_value(IS_FOLDED_LEFT),
            Some(SignalValue::Bool(true)),
            "left should restore folded from NVM"
        );
        assert_eq!(
            bus.latest_value(IS_FOLDED_RIGHT),
            Some(SignalValue::Bool(false)),
            "right should restore unfolded from NVM"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn fold_command_persists_to_nvm() {
        let dir = tempdir().unwrap();
        let nvm = nvm_in(dir.path());
        let bus = Arc::new(MockBus::new());
        let plant = MirrorFoldPlantModel::with_nvm(Arc::clone(&bus), nvm.clone())
            .with_settle(Duration::from_millis(100));
        tokio::spawn(plant.run());
        settle_yields().await;

        bus.inject(FOLD_CMD_LEFT, SignalValue::Bool(true));
        settle_yields().await;
        advance(Duration::from_millis(120)).await;
        settle_yields().await;

        let st = nvm.load_mirror_fold();
        assert!(st.is_folded[0]);
        assert!(!st.is_folded[1]);
    }
}
