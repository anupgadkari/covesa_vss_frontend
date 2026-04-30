//! DoorOpenAssist — exterior puddle lamps when any door opens at
//! night, regardless of ignition state.
//!
//! # Behaviour
//!
//! - Subscribes to all four `IsOpen` signals + the ambient light
//!   sensor.
//! - When the ambient lux ≤ `vehicle_line.auto_headlamp_lux_threshold`
//!   (the same dark-detection threshold ManualLighting / FollowMeHome
//!   use) AND any door transitions `false → true`, claims both puddle
//!   lamps at LOW priority via the puddle arbiter.
//! - Releases the claim once every door has closed.
//!
//! # Why LOW priority?
//!
//! Welcome / Farewell / PerimeterAlarm all want the same outputs at
//! higher priority for more deliberate user-facing reasons.
//! DoorOpenAssist is the always-on convenience layer; it gives way
//! cleanly when any of those preempt.  The arbiter's release-back-to-
//! the-next-highest-active-claim logic does the right thing for free.
//!
//! # Mirror-fold suppression
//!
//! Inherited from the puddle arbiter's `PhysicalGate` — a folded
//! mirror's puddle lamp is force-off regardless of which feature is
//! claiming.  DoorOpenAssist doesn't need to know.

use std::sync::Arc;

use futures::stream::BoxStream;
use futures::StreamExt;
use tokio::select;

use crate::arbiter::{ActuatorRequest, DomainArbiter};
use crate::config::PlatformConfig;
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

const FEATURE_ID: FeatureId = FeatureId::DoorOpenAssist;

const PUDDLE_LEFT: VssPath = "Body.Lights.Puddle.Left.IsOn";
const PUDDLE_RIGHT: VssPath = "Body.Lights.Puddle.Right.IsOn";

const LUX_SIGNAL: VssPath = "Body.Lights.AmbientLightSensor.Illuminance";

const DOOR_OPEN_SIGNALS: [VssPath; 4] = [
    "Body.Doors.Row1.Left.IsOpen",
    "Body.Doors.Row1.Right.IsOpen",
    "Body.Doors.Row2.Left.IsOpen",
    "Body.Doors.Row2.Right.IsOpen",
];

pub struct DoorOpenAssist<B: SignalBus> {
    bus: Arc<B>,
    puddle_arb: Arc<DomainArbiter>,
    /// Lux value at or below which we treat the world as "dark"
    /// (matches ManualLighting AUTO low-beam threshold).
    lux_threshold: u16,
}

impl<B: SignalBus + Send + Sync + 'static> DoorOpenAssist<B> {
    pub fn new(bus: Arc<B>, puddle_arb: Arc<DomainArbiter>, cfg: &PlatformConfig) -> Self {
        Self {
            bus,
            puddle_arb,
            lux_threshold: cfg.vehicle_line.auto_headlamp_lux_threshold,
        }
    }

    /// Test hook: override the dark-threshold without a full PlatformConfig.
    pub fn with_lux_threshold(mut self, lux: u16) -> Self {
        self.lux_threshold = lux;
        self
    }

    pub async fn run(self) {
        tracing::info!(
            lux_threshold = self.lux_threshold,
            "DoorOpenAssist feature started"
        );

        let mut door_streams: Vec<BoxStream<'static, SignalValue>> =
            Vec::with_capacity(DOOR_OPEN_SIGNALS.len());
        for &sig in DOOR_OPEN_SIGNALS.iter() {
            door_streams.push(self.bus.subscribe(sig).await);
        }
        let mut lux_rx = self.bus.subscribe(LUX_SIGNAL).await;

        // Per-door open state cache.  Updated on every door tick;
        // used to compute `any_open` without re-querying the bus.
        let mut door_open: [bool; 4] = [false; 4];
        // Latest lux reading.  Default `u16::MAX` (full daylight) so a
        // missing sensor at boot yields a "daytime, don't claim" stance
        // rather than a "dark, claim immediately" one.
        let mut lux: u16 = u16::MAX;
        // Whether we currently hold the puddle claim.
        let mut claimed = false;

        loop {
            let door_event = futures::future::select_all(
                door_streams
                    .iter_mut()
                    .enumerate()
                    .map(|(i, s)| Box::pin(async move { (i, s.next().await) })),
            );

            select! {
                Some(val) = lux_rx.next() => {
                    if let SignalValue::Uint16(v) = val {
                        lux = v;
                        // Lux change can flip a claim ON or OFF.
                        // Re-evaluate.
                    }
                    self.reconcile(&mut claimed, &door_open, lux).await;
                }
                ((idx, opt), _, _) = door_event => {
                    if let Some(SignalValue::Bool(b)) = opt {
                        door_open[idx] = b;
                    }
                    self.reconcile(&mut claimed, &door_open, lux).await;
                }
                else => break,
            }
        }
    }

    /// Idempotent reconciler: claim iff `(dark AND any door open)`,
    /// release otherwise.  Cheap to call on every tick.
    async fn reconcile(&self, claimed: &mut bool, door_open: &[bool; 4], lux: u16) {
        let dark = lux <= self.lux_threshold;
        let any_open = door_open.iter().any(|b| *b);
        let want_claim = dark && any_open;

        if want_claim && !*claimed {
            tracing::info!(lux, "DoorOpenAssist: dark + door open — claiming puddle");
            for &sig in &[PUDDLE_LEFT, PUDDLE_RIGHT] {
                let _ = self
                    .puddle_arb
                    .request(ActuatorRequest {
                        signal: sig,
                        value: SignalValue::Bool(true),
                        priority: Priority::Low,
                        feature_id: FEATURE_ID,
                    })
                    .await;
            }
            *claimed = true;
        } else if !want_claim && *claimed {
            tracing::info!(
                lux,
                any_open,
                "DoorOpenAssist: condition cleared — releasing puddle"
            );
            for &sig in &[PUDDLE_LEFT, PUDDLE_RIGHT] {
                let _ = self.puddle_arb.release(sig, FEATURE_ID).await;
            }
            *claimed = false;
        }
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::puddle_arbiter;

    async fn settle() {
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    /// Spin up bus + puddle arbiter + DoorOpenAssist.  Threshold = 100
    /// lux so tests can simulate "dark" with values like 10 and "day"
    /// with values like 10000.
    async fn setup() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let (parb, pfut) = puddle_arbiter(Arc::clone(&bus));
        tokio::spawn(pfut);
        let parb = Arc::new(parb);
        let feat = DoorOpenAssist {
            bus: Arc::clone(&bus),
            puddle_arb: parb,
            lux_threshold: 100,
        };
        tokio::spawn(feat.run());
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        bus
    }

    #[tokio::test]
    async fn dark_plus_door_open_claims_puddle() {
        let bus = setup().await;

        bus.inject(LUX_SIGNAL, SignalValue::Uint16(10)); // dark
        settle().await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle().await;

        assert_eq!(bus.latest_value(PUDDLE_LEFT), Some(SignalValue::Bool(true)));
        assert_eq!(
            bus.latest_value(PUDDLE_RIGHT),
            Some(SignalValue::Bool(true))
        );
    }

    #[tokio::test]
    async fn daylight_door_open_does_not_claim() {
        let bus = setup().await;

        bus.inject(LUX_SIGNAL, SignalValue::Uint16(50_000)); // bright
        settle().await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle().await;

        assert_eq!(bus.latest_value(PUDDLE_LEFT), None);
    }

    #[tokio::test]
    async fn all_doors_close_releases_claim() {
        let bus = setup().await;

        bus.inject(LUX_SIGNAL, SignalValue::Uint16(10));
        settle().await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle().await;
        assert_eq!(bus.latest_value(PUDDLE_LEFT), Some(SignalValue::Bool(true)));

        // Door closes.
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(false));
        settle().await;
        assert_eq!(
            bus.latest_value(PUDDLE_LEFT),
            Some(SignalValue::Bool(false)),
            "puddle should release once the only open door closes"
        );
    }

    #[tokio::test]
    async fn second_door_open_keeps_claim_while_first_closes() {
        let bus = setup().await;

        bus.inject(LUX_SIGNAL, SignalValue::Uint16(10));
        settle().await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle().await;
        bus.inject("Body.Doors.Row1.Right.IsOpen", SignalValue::Bool(true));
        settle().await;

        // First door closes — second still open, should stay claimed.
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(false));
        settle().await;
        assert_eq!(bus.latest_value(PUDDLE_LEFT), Some(SignalValue::Bool(true)));

        // Second door closes — now release.
        bus.inject("Body.Doors.Row1.Right.IsOpen", SignalValue::Bool(false));
        settle().await;
        assert_eq!(
            bus.latest_value(PUDDLE_LEFT),
            Some(SignalValue::Bool(false))
        );
    }

    #[tokio::test]
    async fn lux_rising_above_threshold_releases() {
        let bus = setup().await;

        bus.inject(LUX_SIGNAL, SignalValue::Uint16(10));
        settle().await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle().await;
        assert_eq!(bus.latest_value(PUDDLE_LEFT), Some(SignalValue::Bool(true)));

        // Sun comes up while the door is still open (overnight repair
        // scenario) — release.
        bus.inject(LUX_SIGNAL, SignalValue::Uint16(50_000));
        settle().await;
        assert_eq!(
            bus.latest_value(PUDDLE_LEFT),
            Some(SignalValue::Bool(false)),
            "lux rising above threshold should release the claim"
        );
    }
}
