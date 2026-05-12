//! Power-window local switch feature.
//!
//! Watches the 8 per-door local switches
//! `Body.Switches.Window.Local.Row{1,2}.{Left,Right}.{IsUpPressed,IsDownPressed}`
//! and claims the matching window's motor direction
//! `Body.Doors.Row{1,2}.{Left,Right}.Window.MotorDirection` via the
//! window arbiter at `Priority::Low`.
//!
//! ## Child-lock gating
//!
//! For each Row2 door, the feature also subscribes to
//! `Body.Doors.Row2.{side}.IsChildLockActive`.  While that signal is
//! true, the corresponding door's local switch is **ignored** — the
//! feature releases any claim it holds for that window and refuses
//! to issue new ones.  The driver-master pack
//! (`PowerWindowDriver`, Medium) is unaffected.
//!
//! Row1 doors are never child-locked in this implementation.
//!
//! Same single-rising-edge / release-on-quiet pattern as
//! `PowerWindowDriver` — see that module for the truth table.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::arbiter::{ActuatorRequest, DomainArbiter};
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

const FEATURE_ID: FeatureId = FeatureId::PowerWindowLocal;
const PRIORITY: Priority = Priority::Low;

/// One entry per window: (UpSwitch, DownSwitch, MotorOutput,
/// optional ChildLockSignal).  Row1 doors have no child-lock; Row2
/// references the per-door fan-out from `PowerChildLock`.
const WINDOWS: [(VssPath, VssPath, VssPath, Option<VssPath>); 4] = [
    (
        "Body.Switches.Window.Local.Row1.Left.IsUpPressed",
        "Body.Switches.Window.Local.Row1.Left.IsDownPressed",
        "Body.Doors.Row1.Left.Window.MotorDirection",
        None,
    ),
    (
        "Body.Switches.Window.Local.Row1.Right.IsUpPressed",
        "Body.Switches.Window.Local.Row1.Right.IsDownPressed",
        "Body.Doors.Row1.Right.Window.MotorDirection",
        None,
    ),
    (
        "Body.Switches.Window.Local.Row2.Left.IsUpPressed",
        "Body.Switches.Window.Local.Row2.Left.IsDownPressed",
        "Body.Doors.Row2.Left.Window.MotorDirection",
        Some("Body.Doors.Row2.Left.IsChildLockActive"),
    ),
    (
        "Body.Switches.Window.Local.Row2.Right.IsUpPressed",
        "Body.Switches.Window.Local.Row2.Right.IsDownPressed",
        "Body.Doors.Row2.Right.Window.MotorDirection",
        Some("Body.Doors.Row2.Right.IsChildLockActive"),
    ),
];

pub struct PowerWindowLocal<B: SignalBus> {
    bus: Arc<B>,
    arb: Arc<DomainArbiter>,
}

impl<B: SignalBus + Send + Sync + 'static> PowerWindowLocal<B> {
    pub fn new(bus: Arc<B>, arb: Arc<DomainArbiter>) -> Self {
        Self { bus, arb }
    }

    pub async fn run(self) {
        tracing::info!("PowerWindowLocal feature started");

        let mut up_streams = Vec::with_capacity(4);
        let mut down_streams = Vec::with_capacity(4);
        for (u, d, _, _) in WINDOWS.iter() {
            up_streams.push(self.bus.subscribe(u).await);
            down_streams.push(self.bus.subscribe(d).await);
        }
        // Child-lock streams are only present for Row2 (indices 2,3).
        // For Row1 slots we substitute a permanently-pending stream so
        // the slot exists in the index-aligned select_all but never
        // wakes the loop — `stream::pending()` is forever-blocked, vs
        // `stream::empty()` which yields None immediately and would
        // busy-loop the select.
        let mut child_streams: Vec<futures::stream::BoxStream<'static, SignalValue>> =
            Vec::with_capacity(4);
        for (_, _, _, cl) in WINDOWS.iter() {
            match cl {
                Some(sig) => child_streams.push(self.bus.subscribe(sig).await),
                None => child_streams.push(Box::pin(futures::stream::pending())),
            }
        }

        let mut up_state: [bool; 4] = [false; 4];
        let mut down_state: [bool; 4] = [false; 4];
        let mut child_locked: [bool; 4] = [false; 4];
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
            let child_evt = futures::future::select_all(
                child_streams
                    .iter_mut()
                    .enumerate()
                    .map(|(i, s)| Box::pin(async move { (i, s.next().await) })),
            );

            select! {
                ((idx, opt), _, _) = up_evt => {
                    if let Some(SignalValue::Bool(b)) = opt {
                        if up_state[idx] != b {
                            up_state[idx] = b;
                            self.resolve(idx, up_state[idx], down_state[idx], child_locked[idx], &mut last_claim).await;
                        }
                    }
                }
                ((idx, opt), _, _) = down_evt => {
                    if let Some(SignalValue::Bool(b)) = opt {
                        if down_state[idx] != b {
                            down_state[idx] = b;
                            self.resolve(idx, up_state[idx], down_state[idx], child_locked[idx], &mut last_claim).await;
                        }
                    }
                }
                ((idx, opt), _, _) = child_evt => {
                    if let Some(SignalValue::Bool(b)) = opt {
                        if child_locked[idx] != b {
                            child_locked[idx] = b;
                            tracing::info!(window = idx, child_locked = b, "PowerWindowLocal: child-lock state");
                            // Child lock just engaged: drop any active claim
                            // for this window so the driver/global can take over.
                            self.resolve(idx, up_state[idx], down_state[idx], child_locked[idx], &mut last_claim).await;
                        }
                    }
                }
                else => break,
            }
        }

        tracing::warn!("PowerWindowLocal: streams closed, exiting");
    }

    async fn resolve(
        &self,
        idx: usize,
        up: bool,
        down: bool,
        child_locked: bool,
        last: &mut [Option<&'static str>; 4],
    ) {
        let (_, _, motor, _) = WINDOWS[idx];
        let want: Option<&'static str> = if child_locked {
            None // Suppressed — defer to higher-priority claimants.
        } else {
            match (up, down) {
                (true, false) => Some("UP"),
                (false, true) => Some("DOWN"),
                _ => None,
            }
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
        tokio::spawn(PowerWindowLocal::new(Arc::clone(&bus), arb).run());
        settle().await;
        bus
    }

    fn motor(bus: &MockBus, idx: usize) -> Option<String> {
        let (_, _, m, _) = WINDOWS[idx];
        match bus.latest_value(m) {
            Some(SignalValue::String(s)) => Some(s),
            _ => None,
        }
    }

    #[tokio::test]
    async fn front_passenger_local_up_drives_motor_up() {
        let bus = setup().await;
        bus.inject(WINDOWS[1].0, SignalValue::Bool(true));
        settle().await;
        assert_eq!(motor(&bus, 1).as_deref(), Some("UP"));
    }

    #[tokio::test]
    async fn rear_local_works_when_unlocked() {
        let bus = setup().await;
        bus.inject(WINDOWS[2].0, SignalValue::Bool(true));
        settle().await;
        assert_eq!(motor(&bus, 2).as_deref(), Some("UP"));
    }

    #[tokio::test]
    async fn rear_local_suppressed_when_child_locked() {
        let bus = setup().await;
        // Engage child lock first.
        bus.inject(
            "Body.Doors.Row2.Left.IsChildLockActive",
            SignalValue::Bool(true),
        );
        settle().await;
        bus.inject(WINDOWS[2].0, SignalValue::Bool(true));
        settle().await;
        // No claim ⇒ arbiter default ⇒ STOPPED.
        assert_eq!(motor(&bus, 2).as_deref(), Some("STOPPED"));
    }

    #[tokio::test]
    async fn child_lock_engaging_mid_press_drops_active_claim() {
        let bus = setup().await;
        bus.inject(WINDOWS[3].1, SignalValue::Bool(true)); // pressing down
        settle().await;
        assert_eq!(motor(&bus, 3).as_deref(), Some("DOWN"));
        bus.inject(
            "Body.Doors.Row2.Right.IsChildLockActive",
            SignalValue::Bool(true),
        );
        settle().await;
        // Claim must be dropped — arbiter falls back to STOPPED.
        assert_eq!(motor(&bus, 3).as_deref(), Some("STOPPED"));
    }

    #[tokio::test]
    async fn row1_never_gated_by_child_lock() {
        let bus = setup().await;
        // Try to inject child-lock on a Row1 signal — meaningless but
        // shouldn't matter; Row1's WINDOWS entry has no child-lock sub.
        bus.inject(WINDOWS[0].0, SignalValue::Bool(true));
        settle().await;
        assert_eq!(motor(&bus, 0).as_deref(), Some("UP"));
    }
}
