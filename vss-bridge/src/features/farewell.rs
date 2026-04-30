//! Farewell — courtesy lighting after the driver steps out of the
//! vehicle.  Companion to Welcome (which arms on PEPS approach).
//!
//! # Behaviour
//!
//! Trigger sequence:
//! 1. Vehicle ignition transitions `ON` or `START` → `OFF` / `ACC` /
//!    `LOCK`.  This arms the feature into a "watching for door open"
//!    state.
//! 2. Within `ARM_WINDOW_SECS` (30 s by default), any door opens.
//!    Farewell claims puddle + dome via the courtesy arbiters and
//!    starts a hold timer.
//! 3. The lights stay on until any of:
//!    - The hold timer expires (`dealer.farewell_hold_secs`,
//!      default 20 s).
//!    - The vehicle is locked (`Cabin.LockStatus` enters `LOCKED` or
//!      `DOUBLE_LOCKED`).  The driver is gone; secure the cabin.
//!    - The ignition comes back ON (driver got back in / never
//!      really left).
//!
//! # Why a separate feature from Welcome?
//!
//! Same outputs (puddle + dome) but completely different trigger
//! semantics — Welcome is approach-on-LF-detection, Farewell is
//! step-out-after-ignition.  Sharing a feature would muddy both;
//! sharing the *arbiters* is the right level of reuse.
//!
//! # Outputs claimed at MEDIUM priority
//! - `Body.Lights.Puddle.Left.IsOn`
//! - `Body.Lights.Puddle.Right.IsOn`
//! - `Cabin.Lights.IsDomeOn`
//!
//! Mirror-folded puddle suppression still applies via the puddle
//! arbiter's `PhysicalGate` — Farewell doesn't need to know.
//!
//! # Coexistence with Welcome
//!
//! Both claim at MEDIUM.  The arbiter resolves ties by latest-wins,
//! so whichever feature claimed most recently holds the actuators.
//! In the realistic sequence (Welcome on approach → driver gets in →
//! ignition ON → drives off → ignition OFF → Farewell arms on door
//! open) Welcome will have released long before Farewell triggers,
//! so they don't overlap in practice.

use std::sync::Arc;
use std::time::Duration;

use futures::StreamExt;
use tokio::select;
use tokio::time::{sleep, Instant};

use crate::arbiter::{ActuatorRequest, DomainArbiter};
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

const FEATURE_ID: FeatureId = FeatureId::Farewell;

const PUDDLE_LEFT: VssPath = "Body.Lights.Puddle.Left.IsOn";
const PUDDLE_RIGHT: VssPath = "Body.Lights.Puddle.Right.IsOn";
const DOME: VssPath = "Cabin.Lights.IsDomeOn";

const POWER_STATE: VssPath = "Vehicle.LowVoltageSystemState";
const LOCK_STATUS: VssPath = "Cabin.LockStatus";

const DOOR_OPEN_SIGNALS: [VssPath; 4] = [
    "Body.Doors.Row1.Left.IsOpen",
    "Body.Doors.Row1.Right.IsOpen",
    "Body.Doors.Row2.Left.IsOpen",
    "Body.Doors.Row2.Right.IsOpen",
];

/// How long after ignition OFF Farewell stays "watching for door
/// open" before going back to idle.  If the driver doesn't open a
/// door within this window they probably aren't getting out — drop
/// the arming.
pub const ARM_WINDOW_SECS: u64 = 30;

/// Default hold duration (`dealer.farewell_hold_secs` overrides).
pub const DEFAULT_HOLD_SECS: u64 = 20;

/// True when `LowVoltageSystemState` is in a state that means
/// "vehicle is operating".
fn is_powered_on(val: &SignalValue) -> bool {
    matches!(val, SignalValue::String(s) if s == "ON" || s == "START")
}

/// True when the lock-status enum value means "vehicle is locked".
fn is_locked(val: &str) -> bool {
    matches!(val, "LOCKED" | "DOUBLE_LOCKED")
}

#[derive(Debug, Clone, Copy)]
enum State {
    /// Idle.  Waiting for ignition transition ON|START → OFF.
    Idle,
    /// Ignition just turned off; watching for the first door-open
    /// edge until `arm_deadline`.  No actuators claimed yet.
    Armed { arm_deadline: Instant },
    /// Door opened — claimed puddle + dome.  Hold until
    /// `hold_deadline` or an early-release event.
    Holding { hold_deadline: Instant },
}

pub struct Farewell<B: SignalBus> {
    bus: Arc<B>,
    courtesy_arb: Arc<DomainArbiter>,
    puddle_arb: Arc<DomainArbiter>,
    arm_window: Duration,
    hold: Duration,
}

impl<B: SignalBus + Send + Sync + 'static> Farewell<B> {
    pub fn new(
        bus: Arc<B>,
        courtesy_arb: Arc<DomainArbiter>,
        puddle_arb: Arc<DomainArbiter>,
    ) -> Self {
        Self {
            bus,
            courtesy_arb,
            puddle_arb,
            arm_window: Duration::from_secs(ARM_WINDOW_SECS),
            hold: Duration::from_secs(DEFAULT_HOLD_SECS),
        }
    }

    /// Test hook: shorter arm window.
    pub fn with_arm_window(mut self, window: Duration) -> Self {
        self.arm_window = window;
        self
    }

    /// Test hook / cal override: shorter hold duration.
    pub fn with_hold(mut self, hold: Duration) -> Self {
        self.hold = hold;
        self
    }

    pub async fn run(self) {
        tracing::info!(
            arm_window_secs = self.arm_window.as_secs(),
            hold_secs = self.hold.as_secs(),
            "Farewell feature started"
        );

        let mut power_rx = self.bus.subscribe(POWER_STATE).await;
        let mut lock_rx = self.bus.subscribe(LOCK_STATUS).await;
        let mut door_streams: Vec<futures::stream::BoxStream<'static, SignalValue>> =
            Vec::with_capacity(DOOR_OPEN_SIGNALS.len());
        for &sig in DOOR_OPEN_SIGNALS.iter() {
            door_streams.push(self.bus.subscribe(sig).await);
        }

        // Track the previous power-state value so we can detect the
        // ON|START → OFF edge.  Default to `OFF` so a fresh boot in
        // a powered-down vehicle doesn't mis-trigger.
        let mut prev_powered_on = false;
        let mut state = State::Idle;

        loop {
            let door_event = futures::future::select_all(
                door_streams
                    .iter_mut()
                    .enumerate()
                    .map(|(i, s)| Box::pin(async move { (i, s.next().await) })),
            );
            let timer_sleep = match state {
                State::Armed { arm_deadline } => {
                    arm_deadline.saturating_duration_since(Instant::now())
                }
                State::Holding { hold_deadline } => {
                    hold_deadline.saturating_duration_since(Instant::now())
                }
                State::Idle => Duration::from_secs(3600),
            };

            select! {
                Some(val) = power_rx.next() => {
                    let now_on = is_powered_on(&val);
                    let edge_off = prev_powered_on && !now_on;
                    let edge_on = !prev_powered_on && now_on;
                    prev_powered_on = now_on;
                    if edge_off {
                        tracing::info!("Farewell: ignition OFF — armed, watching for door open");
                        state = State::Armed {
                            arm_deadline: Instant::now() + self.arm_window,
                        };
                    } else if edge_on {
                        // Driver got back in / never really left —
                        // drop any armed/holding state.
                        match state {
                            State::Holding { .. } => {
                                tracing::info!("Farewell: ignition back ON — releasing");
                                self.release_all().await;
                            }
                            State::Armed { .. } => {
                                tracing::info!("Farewell: ignition back ON before door open — disarming");
                            }
                            State::Idle => {}
                        }
                        state = State::Idle;
                    }
                }
                Some(val) = lock_rx.next() => {
                    if let SignalValue::String(s) = val {
                        if is_locked(&s) && matches!(state, State::Holding { .. }) {
                            // Driver locked the vehicle — they're gone.
                            // Release lights immediately.
                            tracing::info!("Farewell: vehicle locked — releasing");
                            self.release_all().await;
                            state = State::Idle;
                        }
                    }
                }
                ((door_idx, opt), _, _) = door_event => {
                    if !matches!(opt, Some(SignalValue::Bool(true))) {
                        continue;
                    }
                    if let State::Armed { .. } = state {
                        tracing::info!(
                            door = DOOR_OPEN_SIGNALS[door_idx],
                            "Farewell: door opened — claiming courtesy lights"
                        );
                        self.claim_all(true).await;
                        state = State::Holding {
                            hold_deadline: Instant::now() + self.hold,
                        };
                    }
                }
                _ = sleep(timer_sleep) => {
                    match state {
                        State::Armed { .. } => {
                            tracing::debug!("Farewell: arm window expired — back to idle");
                            state = State::Idle;
                        }
                        State::Holding { .. } => {
                            tracing::info!("Farewell: hold expired — releasing");
                            self.release_all().await;
                            state = State::Idle;
                        }
                        State::Idle => {}
                    }
                }
                else => break,
            }
        }
    }

    async fn claim_all(&self, on: bool) {
        for &sig in &[PUDDLE_LEFT, PUDDLE_RIGHT] {
            let _ = self
                .puddle_arb
                .request(ActuatorRequest {
                    signal: sig,
                    value: SignalValue::Bool(on),
                    priority: Priority::Medium,
                    feature_id: FEATURE_ID,
                })
                .await;
        }
        let _ = self
            .courtesy_arb
            .request(ActuatorRequest {
                signal: DOME,
                value: SignalValue::Bool(on),
                priority: Priority::Medium,
                feature_id: FEATURE_ID,
            })
            .await;
    }

    async fn release_all(&self) {
        for &sig in &[PUDDLE_LEFT, PUDDLE_RIGHT] {
            let _ = self.puddle_arb.release(sig, FEATURE_ID).await;
        }
        let _ = self.courtesy_arb.release(DOME, FEATURE_ID).await;
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::{courtesy_arbiter, puddle_arbiter};
    use tokio::time::advance;

    async fn settle() {
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        tokio::time::sleep(Duration::from_millis(2)).await;
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    /// Spin up bus + courtesy arbiters + Farewell with short windows
    /// for fast virtual-time tests.
    async fn setup() -> (Arc<MockBus>, tokio::task::JoinHandle<()>) {
        let bus = Arc::new(MockBus::new());
        let (carb, cfut) = courtesy_arbiter(Arc::clone(&bus));
        let (parb, pfut) = puddle_arbiter(Arc::clone(&bus));
        tokio::spawn(cfut);
        tokio::spawn(pfut);
        let carb = Arc::new(carb);
        let parb = Arc::new(parb);
        let feat = Farewell::new(Arc::clone(&bus), carb, parb)
            .with_arm_window(Duration::from_millis(200))
            .with_hold(Duration::from_millis(100));
        let h = tokio::spawn(feat.run());
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        (bus, h)
    }

    #[tokio::test(start_paused = true)]
    async fn ignition_off_then_door_open_claims_lights() {
        let (bus, _h) = setup().await;

        // Boot with ignition ON, then turn OFF (the edge that arms).
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle().await;
        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        settle().await;

        // Driver opens door.
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle().await;

        assert_eq!(bus.latest_value(PUDDLE_LEFT), Some(SignalValue::Bool(true)));
        assert_eq!(
            bus.latest_value(PUDDLE_RIGHT),
            Some(SignalValue::Bool(true))
        );
        assert_eq!(bus.latest_value(DOME), Some(SignalValue::Bool(true)));
    }

    #[tokio::test(start_paused = true)]
    async fn door_open_without_prior_ignition_off_does_not_arm() {
        let (bus, _h) = setup().await;

        // Boot powered up.
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle().await;

        // Door opens with ignition still on — no Farewell trigger.
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle().await;
        assert_eq!(
            bus.latest_value(PUDDLE_LEFT),
            None,
            "puddle must stay quiescent; Farewell only arms after ignition OFF"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn arm_window_expires_without_door_open() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle().await;
        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        settle().await;

        // Wait past the arm window.
        advance(Duration::from_millis(220)).await;
        settle().await;

        // Door open after window — too late.
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle().await;

        assert_eq!(
            bus.latest_value(PUDDLE_LEFT),
            None,
            "arm window expired — door open should not claim"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn lock_command_releases_during_hold() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle().await;
        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        settle().await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle().await;
        assert_eq!(bus.latest_value(PUDDLE_LEFT), Some(SignalValue::Bool(true)));

        // Driver locks the vehicle on the way out.
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        settle().await;

        assert_eq!(
            bus.latest_value(PUDDLE_LEFT),
            Some(SignalValue::Bool(false)),
            "lock command must release Farewell hold"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn hold_expires() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle().await;
        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        settle().await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle().await;
        assert_eq!(bus.latest_value(PUDDLE_LEFT), Some(SignalValue::Bool(true)));

        advance(Duration::from_millis(120)).await;
        settle().await;

        assert_eq!(
            bus.latest_value(PUDDLE_LEFT),
            Some(SignalValue::Bool(false)),
            "hold timer must release at expiry"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn ignition_back_on_during_hold_releases() {
        let (bus, _h) = setup().await;

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle().await;
        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        settle().await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle().await;
        assert_eq!(bus.latest_value(PUDDLE_LEFT), Some(SignalValue::Bool(true)));

        // Driver got back in.
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle().await;

        assert_eq!(
            bus.latest_value(PUDDLE_LEFT),
            Some(SignalValue::Bool(false)),
            "ignition-back-on during hold must release"
        );
    }
}
