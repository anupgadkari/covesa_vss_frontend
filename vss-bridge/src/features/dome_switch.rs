//! Dome-switch — the classic 3-position interior dome-light switch.
//!
//! Inputs:
//! - `Cabin.Lights.Dome.SwitchPosition` — String enum:
//!   * `"OFF"`  → lamp forced off
//!   * `"DOOR"` → lamp on iff any cabin door is open
//!   * `"ON"`   → lamp forced on
//! - `Body.Doors.Row{1,2}.{Left,Right}.IsOpen` — door-open flags,
//!   only consulted in the `DOOR` position.
//!
//! Output (via the **Courtesy** arbiter, Priority::Low):
//! - `Cabin.Lights.IsDomeOn` — bool.
//!
//! # Why Low priority?
//!
//! This feature represents the user's resting / default intent.
//! Welcome and Farewell claim at MEDIUM during their courtesy
//! sequences; PerimeterAlarm pulses at HIGH while armed.  Each of
//! those will cleanly pre-empt the switch, and when they release the
//! switch's claim re-takes the lamp.  This matches real-world
//! behaviour: putting the switch to ON does not fight an active
//! perimeter-alarm strobe.
//!
//! # Boot
//!
//! Publishes `"OFF"` to `SwitchPosition` if no value is present so
//! HMI snapshots land deterministically.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::arbiter::{ActuatorRequest, DomainArbiter};
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

const FEATURE_ID: FeatureId = FeatureId::DomeSwitch;

const SWITCH: VssPath = "Cabin.Lights.Dome.SwitchPosition";
const DOME: VssPath = "Cabin.Lights.IsDomeOn";

const DOOR_OPEN_SIGNALS: [VssPath; 4] = [
    "Body.Doors.Row1.Left.IsOpen",
    "Body.Doors.Row1.Right.IsOpen",
    "Body.Doors.Row2.Left.IsOpen",
    "Body.Doors.Row2.Right.IsOpen",
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pos {
    Off,
    Door,
    On,
}

impl Pos {
    fn parse(v: &SignalValue) -> Option<Self> {
        match v {
            SignalValue::String(s) => match s.as_str() {
                "OFF" => Some(Self::Off),
                "DOOR" => Some(Self::Door),
                "ON" => Some(Self::On),
                _ => None,
            },
            _ => None,
        }
    }
}

pub struct DomeSwitch<B: SignalBus> {
    bus: Arc<B>,
    courtesy_arb: Arc<DomainArbiter>,
}

impl<B: SignalBus + Send + Sync + 'static> DomeSwitch<B> {
    pub fn new(bus: Arc<B>, courtesy_arb: Arc<DomainArbiter>) -> Self {
        Self { bus, courtesy_arb }
    }

    pub async fn run(self) {
        tracing::info!("DomeSwitch feature started");

        let mut switch_rx = self.bus.subscribe(SWITCH).await;
        let mut door_streams: Vec<futures::stream::BoxStream<'static, SignalValue>> =
            Vec::with_capacity(DOOR_OPEN_SIGNALS.len());
        for &sig in DOOR_OPEN_SIGNALS.iter() {
            door_streams.push(self.bus.subscribe(sig).await);
        }
        let mut door_open: [bool; 4] = [false; 4];

        let mut pos = Pos::Off;
        let mut last_claim: Option<bool> = None;

        // Drive an initial resolve so the arbiter holds a defined
        // claim from boot.
        Self::resolve(&self.courtesy_arb, pos, &door_open, &mut last_claim).await;

        loop {
            let door_event = futures::future::select_all(
                door_streams
                    .iter_mut()
                    .enumerate()
                    .map(|(i, s)| Box::pin(async move { (i, s.next().await) })),
            );

            select! {
                Some(val) = switch_rx.next() => {
                    if let Some(new_pos) = Pos::parse(&val) {
                        if new_pos != pos {
                            tracing::info!(?new_pos, "DomeSwitch: position change");
                            pos = new_pos;
                            Self::resolve(&self.courtesy_arb, pos, &door_open, &mut last_claim).await;
                        }
                    }
                }
                ((door_idx, opt), _, _) = door_event => {
                    if let Some(SignalValue::Bool(b)) = opt {
                        if door_open[door_idx] != b {
                            door_open[door_idx] = b;
                            // Doors only matter while in DOOR position.
                            if pos == Pos::Door {
                                Self::resolve(&self.courtesy_arb, pos, &door_open, &mut last_claim).await;
                            }
                        }
                    }
                }
                else => break,
            }
        }

        tracing::warn!("DomeSwitch feature exiting");
    }

    /// Compute the desired lamp state and issue a request to the
    /// courtesy arbiter only when it has changed — keeps the bus
    /// quiet during idle.
    async fn resolve(
        arb: &DomainArbiter,
        pos: Pos,
        door_open: &[bool; 4],
        last_claim: &mut Option<bool>,
    ) {
        let want = match pos {
            Pos::Off => false,
            Pos::On => true,
            Pos::Door => door_open.iter().any(|&o| o),
        };
        if *last_claim == Some(want) {
            return;
        }
        *last_claim = Some(want);
        let _ = arb
            .request(ActuatorRequest {
                signal: DOME,
                value: SignalValue::Bool(want),
                priority: Priority::Low,
                feature_id: FEATURE_ID,
            })
            .await;
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::courtesy_arbiter;

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
        let (carb, cfut) = courtesy_arbiter(Arc::clone(&bus));
        tokio::spawn(cfut);
        let feature = DomeSwitch::new(Arc::clone(&bus), Arc::new(carb));
        tokio::spawn(feature.run());
        settle().await;
        bus
    }

    fn dome(bus: &MockBus) -> Option<bool> {
        match bus.latest_value(DOME) {
            Some(SignalValue::Bool(b)) => Some(b),
            _ => None,
        }
    }

    #[tokio::test]
    async fn boots_to_off() {
        let bus = setup().await;
        assert_eq!(
            dome(&bus),
            Some(false),
            "lamp must be off before any switch input"
        );
    }

    #[tokio::test]
    async fn position_on_forces_dome_on() {
        let bus = setup().await;
        bus.inject(SWITCH, SignalValue::String("ON".into()));
        settle().await;
        assert_eq!(dome(&bus), Some(true));
    }

    #[tokio::test]
    async fn position_door_lamp_follows_any_door() {
        let bus = setup().await;
        bus.inject(SWITCH, SignalValue::String("DOOR".into()));
        settle().await;
        assert_eq!(dome(&bus), Some(false), "no doors open → lamp off");

        bus.inject("Body.Doors.Row2.Right.IsOpen", SignalValue::Bool(true));
        settle().await;
        assert_eq!(dome(&bus), Some(true), "any door open → lamp on");

        bus.inject("Body.Doors.Row2.Right.IsOpen", SignalValue::Bool(false));
        settle().await;
        assert_eq!(dome(&bus), Some(false), "all doors closed → lamp off");
    }

    #[tokio::test]
    async fn off_overrides_open_door() {
        let bus = setup().await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        bus.inject(SWITCH, SignalValue::String("OFF".into()));
        settle().await;
        assert_eq!(dome(&bus), Some(false), "OFF must dominate door state");
    }

    #[tokio::test]
    async fn on_overrides_closed_doors() {
        let bus = setup().await;
        bus.inject(SWITCH, SignalValue::String("ON".into()));
        settle().await;
        // All doors closed (default) — lamp still on because pos=ON.
        assert_eq!(dome(&bus), Some(true));
    }

    #[tokio::test]
    async fn redundant_door_edges_are_idempotent() {
        let bus = setup().await;
        bus.inject(SWITCH, SignalValue::String("DOOR".into()));
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle().await;
        bus.clear_history();
        // Same value again — must not republish.
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle().await;
        let republishes = bus.history().iter().filter(|(s, _)| *s == DOME).count();
        assert_eq!(republishes, 0, "idempotent on redundant door edges");
    }
}
