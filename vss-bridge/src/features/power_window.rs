//! Power-window feature — combined controller for all 4 windows and
//! both switch sources (driver-master pack + per-door local switch).
//!
//! Replaces the legacy `power_window_driver` + `power_window_local`
//! pair.  Both sources are now at the same arbiter priority
//! (`Priority::Medium`), and the feature itself produces a **single**
//! resolved claim per window.  Cross-source conflict is handled
//! internally, not at the arbiter level.
//!
//! # Per (window, source) state machine
//!
//! Same 5-detent rocker pattern as the sunroof:
//!
//! ```text
//!   Detent ∈ { NEUTRAL, UP_HOLD, UP_AUTO, DOWN_HOLD, DOWN_AUTO }
//!
//!   State ∈ { Idle, Holding(dir), Auto(dir), AwaitingRelease, Stuck }
//! ```
//!
//! Detent edges advance the state; `NEUTRAL` cancels a Holding or
//! clears an AwaitingRelease; any new press while in an Auto state
//! cancels via AwaitingRelease (the cancel-on-same-switch rule from
//! the sunroof carries over per-source unchanged).
//!
//! # Conflict resolution
//!
//! Every state update on a window triggers a per-window check:
//!
//! ```text
//!   if both sources have active intent (Holding or Auto) →
//!       force each source's state from Holding/Auto to:
//!           AwaitingRelease  if its detent is non-NEUTRAL
//!           Idle             if its detent is already NEUTRAL
//! ```
//!
//! So AUTO is **not preserved** across a conflict (per spec — the
//! user re-presses to start a new motion).
//!
//! # Stuck-switch watchdog
//!
//! Per (window, source) pair, a watchdog starts when the detent leaves
//! NEUTRAL.  If the detent fails to return to NEUTRAL within
//! `stuck_timeout` (default 5 s — short for demo / test convenience),
//! the source is marked **Stuck**: its intent is forced to None until
//! the detent transitions to NEUTRAL.  Logged at `WARN`.
//!
//! # Child-lock gate
//!
//! For Row2 doors only: when
//! `Body.Doors.Row2.{Left,Right}.IsChildLockActive` is true, the
//! matching **local** source's intent is forced to None.  The
//! driver-master source is unaffected — driver can still operate
//! rear windows.
//!
//! # Single writer per window
//!
//! The feature claims `Body.Doors.Row{1,2}.{Left,Right}.Window.MotorDirection`
//! via the window arbiter at `Priority::Medium`.  Future anti-pinch
//! (Critical) and security override (High) features can pre-empt
//! via the arbiter; an RKE / phone-app global requestor sits at
//! `Low`.

use std::sync::Arc;
use std::time::Duration;

use futures::StreamExt;
use tokio::select;
use tokio::time::{sleep_until, Instant};

use crate::arbiter::{ActuatorRequest, DomainArbiter};
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

const FEATURE_ID: FeatureId = FeatureId::PowerWindow;
const PRIORITY: Priority = Priority::Medium;
const NUM_WINDOWS: usize = 4;

/// Default time a single non-NEUTRAL detent is allowed before the
/// watchdog declares the switch stuck.  Short by design — the demo
/// favours quick verification.
pub const DEFAULT_STUCK_TIMEOUT: Duration = Duration::from_secs(5);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Detent {
    Neutral,
    UpHold,
    UpAuto,
    DownHold,
    DownAuto,
}

impl Detent {
    fn parse(v: &SignalValue) -> Option<Self> {
        match v {
            SignalValue::String(s) => match s.as_str() {
                "NEUTRAL" => Some(Self::Neutral),
                "UP_HOLD" => Some(Self::UpHold),
                "UP_AUTO" => Some(Self::UpAuto),
                "DOWN_HOLD" => Some(Self::DownHold),
                "DOWN_AUTO" => Some(Self::DownAuto),
                _ => None,
            },
            _ => None,
        }
    }
    fn is_neutral(self) -> bool {
        matches!(self, Self::Neutral)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dir {
    Up,
    Down,
}

impl Dir {
    fn as_str(self) -> &'static str {
        match self {
            Self::Up => "UP",
            Self::Down => "DOWN",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    Holding(Dir),
    Auto(Dir),
    AwaitingRelease,
    /// Detent stuck non-NEUTRAL past the watchdog timeout — intent
    /// is forced to None until the detent transitions back to NEUTRAL.
    Stuck,
}

impl State {
    /// The motor-direction the state machine wants right now.  `None`
    /// when no driving intent is active.
    fn intent(self) -> Option<Dir> {
        match self {
            Self::Holding(d) | Self::Auto(d) => Some(d),
            Self::Idle | Self::AwaitingRelease | Self::Stuck => None,
        }
    }
}

/// Per-source identity — used for log lines and array indexing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Source {
    DriverMaster,
    Local,
}
impl Source {
    fn label(self) -> &'static str {
        match self {
            Self::DriverMaster => "DriverMaster",
            Self::Local => "Local",
        }
    }
}
const SOURCES: [Source; 2] = [Source::DriverMaster, Source::Local];

/// One row per window: switch signal paths + motor output + optional
/// child-lock gate.
const WINDOW_LABELS: [&str; NUM_WINDOWS] = ["Row1.Left", "Row1.Right", "Row2.Left", "Row2.Right"];

const DRIVER_DETENTS: [VssPath; NUM_WINDOWS] = [
    "Body.Switches.Window.DriverMaster.Row1.Left.Detent",
    "Body.Switches.Window.DriverMaster.Row1.Right.Detent",
    "Body.Switches.Window.DriverMaster.Row2.Left.Detent",
    "Body.Switches.Window.DriverMaster.Row2.Right.Detent",
];
const LOCAL_DETENTS: [VssPath; NUM_WINDOWS] = [
    "Body.Switches.Window.Local.Row1.Left.Detent",
    "Body.Switches.Window.Local.Row1.Right.Detent",
    "Body.Switches.Window.Local.Row2.Left.Detent",
    "Body.Switches.Window.Local.Row2.Right.Detent",
];
const MOTOR_SIGNALS: [VssPath; NUM_WINDOWS] = [
    "Body.Doors.Row1.Left.Window.MotorDirection",
    "Body.Doors.Row1.Right.Window.MotorDirection",
    "Body.Doors.Row2.Left.Window.MotorDirection",
    "Body.Doors.Row2.Right.Window.MotorDirection",
];
const POSITION_SIGNALS: [VssPath; NUM_WINDOWS] = [
    "Body.Doors.Row1.Left.Window.Position",
    "Body.Doors.Row1.Right.Window.Position",
    "Body.Doors.Row2.Left.Window.Position",
    "Body.Doors.Row2.Right.Window.Position",
];
/// Row2 doors have a child-lock signal; Row1 entries are None.
const CHILD_LOCK_SIGNALS: [Option<VssPath>; NUM_WINDOWS] = [
    None,
    None,
    Some("Body.Doors.Row2.Left.IsChildLockActive"),
    Some("Body.Doors.Row2.Right.IsChildLockActive"),
];

pub struct PowerWindow<B: SignalBus> {
    bus: Arc<B>,
    arb: Arc<DomainArbiter>,
    stuck_timeout: Duration,
}

impl<B: SignalBus + Send + Sync + 'static> PowerWindow<B> {
    pub fn new(bus: Arc<B>, arb: Arc<DomainArbiter>) -> Self {
        Self {
            bus,
            arb,
            stuck_timeout: DEFAULT_STUCK_TIMEOUT,
        }
    }

    /// Override the default 5 s stuck-switch timeout — tests use
    /// `tokio::time::pause` + a tiny timeout to exercise the watchdog.
    pub fn with_stuck_timeout(mut self, t: Duration) -> Self {
        self.stuck_timeout = t;
        self
    }

    pub async fn run(self) {
        tracing::info!(
            stuck_timeout_ms = self.stuck_timeout.as_millis() as u64,
            "PowerWindow feature started"
        );

        // Per-(window, source) state + detent + watchdog deadline.
        let mut state = [[State::Idle; 2]; NUM_WINDOWS];
        let mut detent = [[Detent::Neutral; 2]; NUM_WINDOWS];
        let mut deadlines: [[Option<Instant>; 2]; NUM_WINDOWS] = [[None; 2]; NUM_WINDOWS];

        // Per-window state.
        let mut child_locked = [false; NUM_WINDOWS];
        let mut position = [0u8; NUM_WINDOWS];
        let mut last_motor: [Option<&'static str>; NUM_WINDOWS] = [None; NUM_WINDOWS];

        // Subscribe to all the things.
        let mut driver_rx: Vec<futures::stream::BoxStream<'static, SignalValue>> =
            Vec::with_capacity(NUM_WINDOWS);
        for &sig in DRIVER_DETENTS.iter() {
            driver_rx.push(self.bus.subscribe(sig).await);
        }
        let mut local_rx: Vec<futures::stream::BoxStream<'static, SignalValue>> =
            Vec::with_capacity(NUM_WINDOWS);
        for &sig in LOCAL_DETENTS.iter() {
            local_rx.push(self.bus.subscribe(sig).await);
        }
        let mut pos_rx: Vec<futures::stream::BoxStream<'static, SignalValue>> =
            Vec::with_capacity(NUM_WINDOWS);
        for &sig in POSITION_SIGNALS.iter() {
            pos_rx.push(self.bus.subscribe(sig).await);
        }
        let mut child_rx: Vec<futures::stream::BoxStream<'static, SignalValue>> =
            Vec::with_capacity(NUM_WINDOWS);
        for sig in CHILD_LOCK_SIGNALS.iter() {
            match sig {
                Some(s) => child_rx.push(self.bus.subscribe(s).await),
                None => child_rx.push(Box::pin(futures::stream::pending())),
            }
        }

        loop {
            // Find the next watchdog deadline across the per-source
            // grid.  Pinging is wasteful — instead pick the soonest
            // and sleep until it (or any other event wakes us).
            let next_deadline = deadlines
                .iter()
                .flatten()
                .filter_map(|x| *x)
                .min()
                .unwrap_or_else(|| Instant::now() + Duration::from_secs(3600));

            let driver_evt = futures::future::select_all(
                driver_rx
                    .iter_mut()
                    .enumerate()
                    .map(|(i, s)| Box::pin(async move { (i, s.next().await) })),
            );
            let local_evt = futures::future::select_all(
                local_rx
                    .iter_mut()
                    .enumerate()
                    .map(|(i, s)| Box::pin(async move { (i, s.next().await) })),
            );
            let pos_evt = futures::future::select_all(
                pos_rx
                    .iter_mut()
                    .enumerate()
                    .map(|(i, s)| Box::pin(async move { (i, s.next().await) })),
            );
            let child_evt = futures::future::select_all(
                child_rx
                    .iter_mut()
                    .enumerate()
                    .map(|(i, s)| Box::pin(async move { (i, s.next().await) })),
            );

            select! {
                biased;
                _ = sleep_until(next_deadline) => {
                    let now = Instant::now();
                    for w in 0..NUM_WINDOWS {
                        for src_idx in 0..SOURCES.len() {
                            if let Some(d) = deadlines[w][src_idx] {
                                if d <= now && !detent[w][src_idx].is_neutral() {
                                    tracing::warn!(
                                        window = WINDOW_LABELS[w],
                                        source = SOURCES[src_idx].label(),
                                        "PowerWindow: switch stuck — ignoring"
                                    );
                                    state[w][src_idx] = State::Stuck;
                                    deadlines[w][src_idx] = None;
                                }
                            }
                        }
                    }
                    self.evaluate_window_motor_cmds(
                        &mut state, &mut last_motor, &position, &child_locked).await;
                }
                ((w, opt), _, _) = driver_evt => {
                    if let Some(d) = opt.as_ref().and_then(Detent::parse) {
                        Self::on_detent(
                            w, 0, d, &mut state, &mut detent, &mut deadlines,
                            self.stuck_timeout);
                        Self::resolve_conflict(w, &mut state, &detent, &child_locked);
                        self.evaluate_window_motor_cmds(
                            &mut state, &mut last_motor, &position, &child_locked).await;
                    }
                }
                ((w, opt), _, _) = local_evt => {
                    if let Some(d) = opt.as_ref().and_then(Detent::parse) {
                        Self::on_detent(
                            w, 1, d, &mut state, &mut detent, &mut deadlines,
                            self.stuck_timeout);
                        Self::resolve_conflict(w, &mut state, &detent, &child_locked);
                        self.evaluate_window_motor_cmds(
                            &mut state, &mut last_motor, &position, &child_locked).await;
                    }
                }
                ((w, opt), _, _) = pos_evt => {
                    if let Some(SignalValue::Uint8(p)) = opt {
                        position[w] = p;
                        // Natural completion of an auto motion.
                        for cell in state[w].iter_mut() {
                            if let State::Auto(d) = *cell {
                                if (d == Dir::Up && p == 100) || (d == Dir::Down && p == 0) {
                                    *cell = State::Idle;
                                }
                            }
                        }
                        self.evaluate_window_motor_cmds(
                            &mut state, &mut last_motor, &position, &child_locked).await;
                    }
                }
                ((w, opt), _, _) = child_evt => {
                    if let Some(SignalValue::Bool(b)) = opt {
                        if child_locked[w] != b {
                            child_locked[w] = b;
                            tracing::info!(
                                window = WINDOW_LABELS[w], locked = b,
                                "PowerWindow: child-lock state");
                            // Re-run conflict check — when child lock
                            // releases, the local source's intent
                            // becomes visible again and may now
                            // conflict with driver.
                            Self::resolve_conflict(w, &mut state, &detent, &child_locked);
                            self.evaluate_window_motor_cmds(
                                &mut state, &mut last_motor, &position, &child_locked).await;
                        }
                    }
                }
                else => break,
            }
        }

        tracing::warn!("PowerWindow: streams closed, exiting");
    }

    /// Apply a detent edge to (window, source).  Manages the
    /// watchdog deadline and exits Stuck state on a NEUTRAL detent.
    fn on_detent(
        w: usize,
        src_idx: usize,
        d: Detent,
        state: &mut [[State; 2]; NUM_WINDOWS],
        detent: &mut [[Detent; 2]; NUM_WINDOWS],
        deadlines: &mut [[Option<Instant>; 2]; NUM_WINDOWS],
        stuck_timeout: Duration,
    ) {
        let prev_detent = detent[w][src_idx];
        let prev_state = state[w][src_idx];
        detent[w][src_idx] = d;

        // Watchdog bookkeeping.
        if d.is_neutral() {
            deadlines[w][src_idx] = None;
        } else if prev_detent.is_neutral() {
            // Transitioning from NEUTRAL → non-NEUTRAL: start watchdog.
            deadlines[w][src_idx] = Some(Instant::now() + stuck_timeout);
        }
        // Non-NEUTRAL → non-NEUTRAL keeps the existing deadline.

        // Stuck-state recovery: a NEUTRAL transition clears the flag.
        if matches!(prev_state, State::Stuck) {
            if d.is_neutral() {
                state[w][src_idx] = State::Idle;
                tracing::info!(
                    window = WINDOW_LABELS[w],
                    source = SOURCES[src_idx].label(),
                    "PowerWindow: stuck switch released"
                );
            }
            // While Stuck, any non-NEUTRAL detent is ignored.
            return;
        }

        state[w][src_idx] = Self::transition(prev_state, d);
    }

    /// Pure per-source transition table — same shape as the sunroof.
    fn transition(prev: State, d: Detent) -> State {
        match prev {
            State::Idle => match d {
                Detent::Neutral => State::Idle,
                Detent::UpHold => State::Holding(Dir::Up),
                Detent::UpAuto => State::Auto(Dir::Up),
                Detent::DownHold => State::Holding(Dir::Down),
                Detent::DownAuto => State::Auto(Dir::Down),
            },
            State::Holding(Dir::Up) => match d {
                Detent::Neutral => State::Idle,
                Detent::UpHold => State::Holding(Dir::Up),
                _ => State::AwaitingRelease,
            },
            State::Holding(Dir::Down) => match d {
                Detent::Neutral => State::Idle,
                Detent::DownHold => State::Holding(Dir::Down),
                _ => State::AwaitingRelease,
            },
            State::Auto(dir) => match d {
                Detent::Neutral => State::Auto(dir),
                _ => State::AwaitingRelease,
            },
            State::AwaitingRelease => match d {
                Detent::Neutral => State::Idle,
                _ => State::AwaitingRelease,
            },
            // Stuck handled above before this function is called.
            State::Stuck => State::Stuck,
        }
    }

    /// Cross-source conflict for a single window: if both sources
    /// have active intent, cancel both per the user's spec.  The
    /// child-lock gate suppresses the local source's intent first —
    /// a child-locked local switch cannot create a conflict that
    /// would block the driver master.
    fn resolve_conflict(
        w: usize,
        state: &mut [[State; 2]; NUM_WINDOWS],
        detent: &[[Detent; 2]; NUM_WINDOWS],
        child_locked: &[bool; NUM_WINDOWS],
    ) {
        let drv_active = state[w][0].intent().is_some();
        let loc_active = !child_locked[w] && state[w][1].intent().is_some();
        if !(drv_active && loc_active) {
            return;
        }
        for src_idx in 0..SOURCES.len() {
            match state[w][src_idx] {
                State::Holding(_) | State::Auto(_) => {
                    state[w][src_idx] = if detent[w][src_idx].is_neutral() {
                        State::Idle
                    } else {
                        State::AwaitingRelease
                    };
                }
                _ => {}
            }
        }
        tracing::info!(
            window = WINDOW_LABELS[w],
            "PowerWindow: cross-source conflict — both sources cancelled"
        );
    }

    /// Compute the resolved motor direction for each window and emit
    /// arbiter claims on change.  Quiet when nothing changed.
    async fn evaluate_window_motor_cmds(
        &self,
        state: &mut [[State; 2]; NUM_WINDOWS],
        last_motor: &mut [Option<&'static str>; NUM_WINDOWS],
        _position: &[u8; NUM_WINDOWS],
        child_locked: &[bool; NUM_WINDOWS],
    ) {
        for w in 0..NUM_WINDOWS {
            let drv_intent = state[w][0].intent();
            let loc_intent = if child_locked[w] {
                None
            } else {
                state[w][1].intent()
            };
            let resolved: Option<Dir> = match (drv_intent, loc_intent) {
                (None, None) => None,
                (Some(d), None) | (None, Some(d)) => Some(d),
                // Both active → defensive None.  `resolve_conflict`
                // should already have cleared one side, but the
                // child-lock path can also produce a single-intent
                // outcome, so we cover both here.
                (Some(_), Some(_)) => None,
            };
            let want: &'static str = match resolved {
                Some(d) => d.as_str(),
                None => "STOPPED",
            };
            let issue = match want {
                "STOPPED" => None,
                v => Some(v),
            };
            if last_motor[w] == issue {
                continue;
            }
            last_motor[w] = issue;
            match issue {
                Some(v) => {
                    let _ = self
                        .arb
                        .request(ActuatorRequest {
                            signal: MOTOR_SIGNALS[w],
                            value: SignalValue::String(v.into()),
                            priority: PRIORITY,
                            feature_id: FEATURE_ID,
                        })
                        .await;
                }
                None => {
                    let _ = self.arb.release(MOTOR_SIGNALS[w], FEATURE_ID).await;
                }
            }
        }
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::window_arbiter;
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

    async fn setup_with_timeout(stuck: Duration) -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let (arb, fut) = window_arbiter(Arc::clone(&bus));
        tokio::spawn(fut);
        let arb = Arc::new(arb);
        tokio::spawn(
            PowerWindow::new(Arc::clone(&bus), arb)
                .with_stuck_timeout(stuck)
                .run(),
        );
        settle().await;
        bus
    }

    async fn setup() -> Arc<MockBus> {
        setup_with_timeout(Duration::from_secs(60)).await
    }

    fn motor(bus: &MockBus, w: usize) -> Option<String> {
        match bus.latest_value(MOTOR_SIGNALS[w]) {
            Some(SignalValue::String(s)) => Some(s),
            _ => None,
        }
    }

    fn set_detent(bus: &MockBus, sig: VssPath, v: &str) {
        bus.inject(sig, SignalValue::String(v.into()));
    }

    // Scenario 1
    #[tokio::test]
    async fn driver_alone_drives_up() {
        let bus = setup().await;
        set_detent(&bus, DRIVER_DETENTS[0], "UP_HOLD");
        settle().await;
        assert_eq!(motor(&bus, 0).as_deref(), Some("UP"));
    }

    // Scenario 2
    #[tokio::test]
    async fn local_alone_drives_up() {
        let bus = setup().await;
        set_detent(&bus, LOCAL_DETENTS[1], "UP_HOLD");
        settle().await;
        assert_eq!(motor(&bus, 1).as_deref(), Some("UP"));
    }

    // Scenario 3
    #[tokio::test]
    async fn both_pressed_same_direction_stops() {
        let bus = setup().await;
        set_detent(&bus, DRIVER_DETENTS[0], "UP_HOLD");
        set_detent(&bus, LOCAL_DETENTS[0], "UP_HOLD");
        settle().await;
        // Both intend → STOPPED (default).  Arbiter publishes STOPPED.
        assert_eq!(motor(&bus, 0).as_deref(), Some("STOPPED"));
    }

    // Scenario 4
    #[tokio::test]
    async fn both_pressed_opposite_stops() {
        let bus = setup().await;
        set_detent(&bus, DRIVER_DETENTS[0], "UP_HOLD");
        set_detent(&bus, LOCAL_DETENTS[0], "DOWN_HOLD");
        settle().await;
        assert_eq!(motor(&bus, 0).as_deref(), Some("STOPPED"));
    }

    // Scenario 5
    #[tokio::test]
    async fn auto_runs_alone_until_endstop() {
        let bus = setup().await;
        set_detent(&bus, DRIVER_DETENTS[2], "UP_AUTO");
        set_detent(&bus, DRIVER_DETENTS[2], "NEUTRAL"); // spring release
        settle().await;
        assert_eq!(motor(&bus, 2).as_deref(), Some("UP"));
        // Plant publishes Position = 100 → natural completion.
        bus.inject(POSITION_SIGNALS[2], SignalValue::Uint8(100));
        settle().await;
        assert_eq!(motor(&bus, 2).as_deref(), Some("STOPPED"));
    }

    // Scenarios 6 + 7
    #[tokio::test]
    async fn auto_cancelled_on_conflict_does_not_resume() {
        let bus = setup().await;
        set_detent(&bus, DRIVER_DETENTS[0], "UP_AUTO");
        set_detent(&bus, DRIVER_DETENTS[0], "NEUTRAL");
        settle().await;
        assert_eq!(motor(&bus, 0).as_deref(), Some("UP"));
        // Local presses DOWN_HOLD → conflict.
        set_detent(&bus, LOCAL_DETENTS[0], "DOWN_HOLD");
        settle().await;
        assert_eq!(motor(&bus, 0).as_deref(), Some("STOPPED"));
        // Local releases.  Driver's AUTO was cancelled — motor stays STOPPED.
        set_detent(&bus, LOCAL_DETENTS[0], "NEUTRAL");
        settle().await;
        assert_eq!(motor(&bus, 0).as_deref(), Some("STOPPED"));
        // A fresh driver press starts a new motion.
        set_detent(&bus, DRIVER_DETENTS[0], "UP_HOLD");
        settle().await;
        assert_eq!(motor(&bus, 0).as_deref(), Some("UP"));
    }

    // Scenario 8
    #[tokio::test(start_paused = true)]
    async fn stuck_driver_lets_local_drive() {
        let bus = setup_with_timeout(Duration::from_millis(100)).await;
        set_detent(&bus, DRIVER_DETENTS[0], "UP_HOLD"); // stuck press
        settle().await;
        // Before timeout: driver alone driving UP.
        assert_eq!(motor(&bus, 0).as_deref(), Some("UP"));
        // Watchdog fires.
        advance(Duration::from_millis(200)).await;
        settle().await;
        // Driver now Stuck (intent None) → motor STOPPED while no
        // other source is active.
        assert_eq!(motor(&bus, 0).as_deref(), Some("STOPPED"));
        // Local takes over normally.
        set_detent(&bus, LOCAL_DETENTS[0], "DOWN_HOLD");
        settle().await;
        assert_eq!(motor(&bus, 0).as_deref(), Some("DOWN"));
    }

    // Scenario 9
    #[tokio::test(start_paused = true)]
    async fn both_stuck_locks_window() {
        let bus = setup_with_timeout(Duration::from_millis(100)).await;
        set_detent(&bus, DRIVER_DETENTS[3], "UP_HOLD");
        set_detent(&bus, LOCAL_DETENTS[3], "DOWN_HOLD");
        settle().await;
        // Initial: conflict → STOPPED.
        assert_eq!(motor(&bus, 3).as_deref(), Some("STOPPED"));
        advance(Duration::from_millis(200)).await;
        settle().await;
        // Both stuck — still STOPPED, no other source can drive
        // until one detent unsticks.
        assert_eq!(motor(&bus, 3).as_deref(), Some("STOPPED"));
    }

    // Scenario 10 — cancel-on-same-source
    #[tokio::test]
    async fn driver_auto_then_self_press_cancels() {
        let bus = setup().await;
        set_detent(&bus, DRIVER_DETENTS[0], "UP_AUTO");
        set_detent(&bus, DRIVER_DETENTS[0], "NEUTRAL");
        settle().await;
        assert_eq!(motor(&bus, 0).as_deref(), Some("UP"));
        // Same source presses DOWN_HOLD → cancels via AwaitingRelease.
        set_detent(&bus, DRIVER_DETENTS[0], "DOWN_HOLD");
        settle().await;
        assert_eq!(motor(&bus, 0).as_deref(), Some("STOPPED"));
    }

    // Scenario 11 — child lock gates local on Row2
    #[tokio::test]
    async fn child_lock_suppresses_rear_local_no_conflict() {
        let bus = setup().await;
        // Engage child lock on Row2.Left.
        bus.inject(
            "Body.Doors.Row2.Left.IsChildLockActive",
            SignalValue::Bool(true),
        );
        settle().await;
        // Driver presses UP_HOLD on Row2.Left.
        set_detent(&bus, DRIVER_DETENTS[2], "UP_HOLD");
        // Local also presses DOWN_HOLD on Row2.Left.
        set_detent(&bus, LOCAL_DETENTS[2], "DOWN_HOLD");
        settle().await;
        // Local intent suppressed by child lock → no conflict, driver drives UP.
        assert_eq!(motor(&bus, 2).as_deref(), Some("UP"));
    }

    // Stuck recovery
    #[tokio::test(start_paused = true)]
    async fn stuck_clears_on_neutral() {
        let bus = setup_with_timeout(Duration::from_millis(100)).await;
        set_detent(&bus, DRIVER_DETENTS[0], "UP_HOLD");
        advance(Duration::from_millis(200)).await;
        settle().await;
        // Driver Stuck.
        // Detent goes NEUTRAL — Stuck clears.
        set_detent(&bus, DRIVER_DETENTS[0], "NEUTRAL");
        settle().await;
        // Fresh press works.
        set_detent(&bus, DRIVER_DETENTS[0], "UP_HOLD");
        settle().await;
        assert_eq!(motor(&bus, 0).as_deref(), Some("UP"));
    }
}
