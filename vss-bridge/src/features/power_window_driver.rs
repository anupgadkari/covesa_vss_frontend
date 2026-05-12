//! Power-window driver-master feature.
//!
//! Watches the 8 driver-master pack switches
//! `Body.Switches.Window.DriverMaster.Row{1,2}.{Left,Right}.{IsUpPressed,IsDownPressed}`
//! and claims the matching window's motor direction
//! `Body.Doors.Row{1,2}.{Left,Right}.Window.MotorDirection` (String
//! enum `UP` / `DOWN` / `STOPPED`) via the window arbiter at
//! `Priority::Medium`.
//!
//! Per window:
//!
//! | UpPressed | DownPressed | Action |
//! |---|---|---|
//! | true  | false | claim `UP`     |
//! | false | true  | claim `DOWN`   |
//! | false | false | **release**    |
//! | true  | true  | **release** (ambiguous — defer to next-lower claimant) |
//!
//! No child-lock check — the driver master always wins.  When the
//! driver releases their button, this feature releases its claim and
//! a `PowerWindowLocal` claim (if any) for that window takes over via
//! the arbiter; if there's no lower claim, the arbiter publishes the
//! configured default `STOPPED`.
//!
//! # Single point of contention
//!
//! The arbiter does the priority work — this feature does not look
//! at any other source.  Future `WindowAntiPinch` / `Security` /
//! `Global` features slot in via the arbiter allow-list without
//! changing this code.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::arbiter::{ActuatorRequest, DomainArbiter};
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

const FEATURE_ID: FeatureId = FeatureId::PowerWindowDriver;
const PRIORITY: Priority = Priority::Medium;

/// One row per window: (UpSwitch, DownSwitch, MotorOutput).
const WINDOWS: [(VssPath, VssPath, VssPath); 4] = [
    (
        "Body.Switches.Window.DriverMaster.Row1.Left.IsUpPressed",
        "Body.Switches.Window.DriverMaster.Row1.Left.IsDownPressed",
        "Body.Doors.Row1.Left.Window.MotorDirection",
    ),
    (
        "Body.Switches.Window.DriverMaster.Row1.Right.IsUpPressed",
        "Body.Switches.Window.DriverMaster.Row1.Right.IsDownPressed",
        "Body.Doors.Row1.Right.Window.MotorDirection",
    ),
    (
        "Body.Switches.Window.DriverMaster.Row2.Left.IsUpPressed",
        "Body.Switches.Window.DriverMaster.Row2.Left.IsDownPressed",
        "Body.Doors.Row2.Left.Window.MotorDirection",
    ),
    (
        "Body.Switches.Window.DriverMaster.Row2.Right.IsUpPressed",
        "Body.Switches.Window.DriverMaster.Row2.Right.IsDownPressed",
        "Body.Doors.Row2.Right.Window.MotorDirection",
    ),
];

pub struct PowerWindowDriver<B: SignalBus> {
    bus: Arc<B>,
    arb: Arc<DomainArbiter>,
}

impl<B: SignalBus + Send + Sync + 'static> PowerWindowDriver<B> {
    pub fn new(bus: Arc<B>, arb: Arc<DomainArbiter>) -> Self {
        Self { bus, arb }
    }

    pub async fn run(self) {
        tracing::info!("PowerWindowDriver feature started");

        // Per-window switch streams + a tracked state for the held
        // buttons.  Anonymous `Vec` of streams; iterated each loop
        // turn via `futures::future::select_all`.
        let mut up_streams = Vec::with_capacity(4);
        let mut down_streams = Vec::with_capacity(4);
        for (u, d, _) in WINDOWS.iter() {
            up_streams.push(self.bus.subscribe(u).await);
            down_streams.push(self.bus.subscribe(d).await);
        }
        let mut up_state: [bool; 4] = [false; 4];
        let mut down_state: [bool; 4] = [false; 4];
        // Last claim issued per window — used to avoid spamming the
        // arbiter with redundant requests.
        let mut last_claim: [Option<&'static str>; 4] = [None; 4];

        loop {
            let up_evt = futures::future::select_all(
                up_streams
                    .iter_mut()
                    .enumerate()
                    .map(|(i, s)| Box::pin(async move { (i, s.next().await) })),
            );
            let down_evt = futures::future::select_all(
                down_streams
                    .iter_mut()
                    .enumerate()
                    .map(|(i, s)| Box::pin(async move { (i, s.next().await) })),
            );

            select! {
                ((idx, opt), _, _) = up_evt => {
                    if let Some(SignalValue::Bool(b)) = opt {
                        if up_state[idx] != b {
                            up_state[idx] = b;
                            self.resolve(idx, up_state[idx], down_state[idx], &mut last_claim).await;
                        }
                    }
                }
                ((idx, opt), _, _) = down_evt => {
                    if let Some(SignalValue::Bool(b)) = opt {
                        if down_state[idx] != b {
                            down_state[idx] = b;
                            self.resolve(idx, up_state[idx], down_state[idx], &mut last_claim).await;
                        }
                    }
                }
                else => break,
            }
        }

        tracing::warn!("PowerWindowDriver: streams closed, exiting");
    }

    /// Map the current up/down held state to a motor request or a
    /// release, and emit only when the resolved claim actually
    /// changed.
    async fn resolve(
        &self,
        idx: usize,
        up: bool,
        down: bool,
        last: &mut [Option<&'static str>; 4],
    ) {
        let (_, _, motor) = WINDOWS[idx];
        let want: Option<&'static str> = match (up, down) {
            (true, false) => Some("UP"),
            (false, true) => Some("DOWN"),
            _ => None, // none or both — defer
        };
        if last[idx] == want {
            return;
        }
        last[idx] = want;
        match want {
            Some(v) => {
                let _ = self
                    .arb
                    .request(ActuatorRequest {
                        signal: motor,
                        value: SignalValue::String(v.into()),
                        priority: PRIORITY,
                        feature_id: FEATURE_ID,
                    })
                    .await;
            }
            None => {
                let _ = self.arb.release(motor, FEATURE_ID).await;
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

    async fn settle() {
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        tokio::time::sleep(std::time::Duration::from_millis(2)).await;
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    async fn setup() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let (arb, fut) = window_arbiter(Arc::clone(&bus));
        tokio::spawn(fut);
        let arb = Arc::new(arb);
        tokio::spawn(PowerWindowDriver::new(Arc::clone(&bus), arb).run());
        settle().await;
        bus
    }

    fn motor(bus: &MockBus, idx: usize) -> Option<String> {
        let (_, _, m) = WINDOWS[idx];
        match bus.latest_value(m) {
            Some(SignalValue::String(s)) => Some(s),
            _ => None,
        }
    }

    #[tokio::test]
    async fn up_press_drives_motor_up() {
        let bus = setup().await;
        bus.inject(WINDOWS[0].0, SignalValue::Bool(true));
        settle().await;
        assert_eq!(motor(&bus, 0).as_deref(), Some("UP"));
    }

    #[tokio::test]
    async fn release_returns_to_stopped() {
        let bus = setup().await;
        bus.inject(WINDOWS[0].0, SignalValue::Bool(true));
        settle().await;
        bus.inject(WINDOWS[0].0, SignalValue::Bool(false));
        settle().await;
        assert_eq!(motor(&bus, 0).as_deref(), Some("STOPPED"));
    }

    #[tokio::test]
    async fn down_press_drives_motor_down() {
        let bus = setup().await;
        bus.inject(WINDOWS[2].1, SignalValue::Bool(true));
        settle().await;
        assert_eq!(motor(&bus, 2).as_deref(), Some("DOWN"));
    }

    #[tokio::test]
    async fn both_pressed_defers_to_stopped() {
        let bus = setup().await;
        bus.inject(WINDOWS[0].0, SignalValue::Bool(true));
        bus.inject(WINDOWS[0].1, SignalValue::Bool(true));
        settle().await;
        // Ambiguous — feature releases; arbiter default is STOPPED.
        assert_eq!(motor(&bus, 0).as_deref(), Some("STOPPED"));
    }

    #[tokio::test]
    async fn independent_windows() {
        let bus = setup().await;
        bus.inject(WINDOWS[0].0, SignalValue::Bool(true));
        bus.inject(WINDOWS[3].1, SignalValue::Bool(true));
        settle().await;
        assert_eq!(motor(&bus, 0).as_deref(), Some("UP"));
        assert_eq!(motor(&bus, 3).as_deref(), Some("DOWN"));
        assert_eq!(motor(&bus, 1).as_deref(), Some("STOPPED"));
        assert_eq!(motor(&bus, 2).as_deref(), Some("STOPPED"));
    }
}
