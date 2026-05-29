//! Lost-paired-key scan (item #14d of the post-PEPS backlog,
//! renamed from `door_close_scan` to reflect the actual safety
//! purpose).
//!
//! # Customer-visible behaviour
//!
//! Driver gets in, closes the door, starts the car — but somehow
//! the paired fob is no longer with them (left it on the porch,
//! dropped it on the driveway, handed it to a passenger who got
//! out before the door shut, …).  The car is now running without
//! a paired key on board.  This feature notices and pops up a
//! cluster warning so the driver doesn't unknowingly drive away.
//!
//! # Trigger
//!
//! On the door-state transition **any-door-open → all-doors-closed**
//! **AND** `Vehicle.LowVoltageSystemState` ∈ {`ON`, `START`},
//! submit a Presence-mode scan covering every on-vehicle zone via
//! the [`KeySearchArbiter`].  The scan shape is an
//! [`AntennaSet::Sequence`] of:
//!
//! - [`AntennaSet::AllApproach`] — approach + front / hood / trunk
//!   proximity zones.
//! - [`AntennaSet::TrunkInside`] — cargo-area antenna.
//! - [`AntennaSet::Cabin`]       — cabin antennas.
//!
//! # Outputs
//!
//! - `Vehicle.Controller.Body.PEPS.LostKeyWarning` (Bool).  Set `true` when the scan
//!   completes with zero paired keys found anywhere on the vehicle.
//!   Cleared back to `false` on either:
//!   * the next scan finding at least one key, or
//!   * ignition leaving the live state (driver shut the car off).
//!
//! The HMI cluster subscribes and renders the warning popup.
//!
//! # Why "lost PK" and not generic "door-close snapshot"?
//!
//! The original 14d plan was a generic on-close snapshot that
//! also fed Smart Unlock (#15).  In practice, Smart Unlock's
//! trigger is the **lock** edge (driver locks the door from
//! outside with a fob potentially still in the trunk) — a
//! different event entirely — so Smart Unlock should run its own
//! scan when it needs one.  Conflating the two muddied both
//! features.  The on-close-during-running case is squarely the
//! lost-paired-key safety check; calling it that.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::features::key_search_arbiter::{
    AntennaSet, Coalescing, KeySearchArbiterHandle, SearchMode,
};
use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

const DOOR_OPEN_SIGNALS: [VssPath; 4] = [
    "Vehicle.Cabin.Door.Row1.Left.IsOpen",
    "Vehicle.Cabin.Door.Row1.Right.IsOpen",
    "Vehicle.Cabin.Door.Row2.Left.IsOpen",
    "Vehicle.Cabin.Door.Row2.Right.IsOpen",
];

const IGNITION_STATE: VssPath = "Vehicle.LowVoltageSystemState";
const LOST_KEY_WARNING: VssPath = "Vehicle.Controller.Body.PEPS.LostKeyWarning";

/// True when the ignition is live (engine running or cranking).
fn ignition_is_live(s: &str) -> bool {
    matches!(s, "ON" | "START")
}

pub struct LostPkScan<B: SignalBus> {
    bus: Arc<B>,
    key_search: KeySearchArbiterHandle,
}

impl<B: SignalBus + Send + Sync + 'static> LostPkScan<B> {
    pub fn new(bus: Arc<B>, key_search: KeySearchArbiterHandle) -> Self {
        Self { bus, key_search }
    }

    pub async fn run(self) {
        tracing::info!("LostPkScan feature started");

        let mut door_streams: Vec<futures::stream::BoxStream<'static, SignalValue>> =
            Vec::with_capacity(DOOR_OPEN_SIGNALS.len());
        for &sig in DOOR_OPEN_SIGNALS.iter() {
            door_streams.push(self.bus.subscribe(sig).await);
        }
        let mut ign_rx = self.bus.subscribe(IGNITION_STATE).await;

        // Per-door cache (false = closed at boot).
        let mut door_open = [false; DOOR_OPEN_SIGNALS.len()];
        let mut prev_any_open = false;
        let mut ign_live = false;
        let mut warning_active = false;

        // Boot publish so HMI snapshots see a defined value.
        let _ = self
            .bus
            .publish(LOST_KEY_WARNING, SignalValue::Bool(false))
            .await;

        loop {
            let door_event = futures::future::select_all(
                door_streams
                    .iter_mut()
                    .enumerate()
                    .map(|(i, s)| Box::pin(async move { (i, s.next().await) })),
            );

            select! {
                Some(val) = ign_rx.next() => {
                    if let SignalValue::String(s) = val {
                        let now_live = ignition_is_live(&s);
                        if !now_live && warning_active {
                            // Ignition went off — clear any lingering
                            // warning so the next start cycle has a
                            // clean slate.
                            warning_active = false;
                            let _ = self
                                .bus
                                .publish(LOST_KEY_WARNING, SignalValue::Bool(false))
                                .await;
                        }
                        ign_live = now_live;
                    }
                }

                ((idx, opt), _, _) = door_event => {
                    let Some(val) = opt else { break };
                    let is_open = matches!(val, SignalValue::Bool(true));
                    if door_open[idx] == is_open {
                        continue; // redundant edge
                    }
                    door_open[idx] = is_open;
                    let now_any_open = door_open.iter().any(|&b| b);
                    let close_edge = prev_any_open && !now_any_open;
                    prev_any_open = now_any_open;
                    if !close_edge || !ign_live {
                        continue;
                    }

                    tracing::info!(
                        "LostPkScan: all doors closed while ignition live — submitting key search"
                    );

                    // Run the scan inline (await the result) so we
                    // can drive the warning state from this loop.
                    // The arbiter serialises requests anyway; door
                    // events are rare so blocking for ~150 ms of
                    // simulated airtime is fine.
                    let result = self
                        .key_search
                        .submit(
                            "LostPkScan",
                            AntennaSet::Sequence(vec![
                                (AntennaSet::AllApproach, SearchMode::Presence),
                                (AntennaSet::TrunkInside, SearchMode::Presence),
                                (AntennaSet::Cabin, SearchMode::Presence),
                            ]),
                            SearchMode::Presence,
                            Coalescing::Disallowed,
                        )
                        .await;
                    let any_found = result
                        .as_ref()
                        .map(|r| !r.keys_found.is_empty())
                        .unwrap_or(false);
                    let want_warning = !any_found;
                    if want_warning != warning_active {
                        warning_active = want_warning;
                        tracing::info!(
                            warning_active,
                            "LostPkScan: cluster warning state changed"
                        );
                        let _ = self
                            .bus
                            .publish(LOST_KEY_WARNING, SignalValue::Bool(warning_active))
                            .await;
                    }
                }
            }
        }

        tracing::warn!("LostPkScan: input stream closed, exiting");
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::features::key_search_arbiter::KeySearchArbiter;
    use std::time::Duration;

    async fn settle() {
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
    }

    async fn setup() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let (ksa, handle, rx) = KeySearchArbiter::new_with_rx(Arc::clone(&bus));
        tokio::spawn(ksa.run(rx));
        tokio::spawn(LostPkScan::new(Arc::clone(&bus), handle).run());
        settle().await;
        bus
    }

    fn open(bus: &MockBus, idx: usize, b: bool) {
        bus.inject(DOOR_OPEN_SIGNALS[idx], SignalValue::Bool(b));
    }

    fn place(bus: &MockBus, slot: u8, zone: &str) {
        let path = match slot {
            1 => "Vehicle.Simulation.PEPS.Plant.KeyFob.1.PlacedZone",
            2 => "Vehicle.Simulation.PEPS.Plant.KeyFob.2.PlacedZone",
            _ => panic!("unknown slot"),
        };
        bus.inject(path, SignalValue::String(zone.into()));
    }

    fn set_ignition(bus: &MockBus, state: &str) {
        bus.inject(IGNITION_STATE, SignalValue::String(state.into()));
    }

    fn latest_warning(bus: &MockBus) -> Option<bool> {
        match bus.latest_value(LOST_KEY_WARNING) {
            Some(SignalValue::Bool(b)) => Some(b),
            _ => None,
        }
    }

    /// Wait for an arbiter scan to complete (sequence airtime
    /// ~150 ms) and for the spawned tasks to fan out.
    async fn wait_for_scan() {
        tokio::time::sleep(Duration::from_millis(250)).await;
        settle().await;
    }

    #[tokio::test]
    async fn boots_with_warning_false() {
        let bus = setup().await;
        assert_eq!(latest_warning(&bus), Some(false));
    }

    #[tokio::test]
    async fn ignition_off_door_close_does_not_scan_or_warn() {
        let bus = setup().await;
        // No ignition injected = OFF by default.
        open(&bus, 0, true);
        settle().await;
        open(&bus, 0, false);
        wait_for_scan().await;
        assert_eq!(latest_warning(&bus), Some(false));
    }

    #[tokio::test]
    async fn ignition_live_door_close_with_no_key_raises_warning() {
        let bus = setup().await;
        set_ignition(&bus, "ON");
        settle().await;
        open(&bus, 0, true);
        settle().await;
        open(&bus, 0, false);
        wait_for_scan().await;
        assert_eq!(
            latest_warning(&bus),
            Some(true),
            "no key on/near vehicle + ignition live + all doors closed → warning"
        );
    }

    #[tokio::test]
    async fn ignition_live_door_close_with_cabin_key_no_warning() {
        let bus = setup().await;
        place(&bus, 1, "Cabin");
        set_ignition(&bus, "ON");
        settle().await;
        open(&bus, 0, true);
        settle().await;
        open(&bus, 0, false);
        wait_for_scan().await;
        assert_eq!(latest_warning(&bus), Some(false));
    }

    #[tokio::test]
    async fn warning_clears_when_key_reappears_on_next_cycle() {
        let bus = setup().await;
        set_ignition(&bus, "ON");
        settle().await;
        // Cycle 1: no key → warning.
        open(&bus, 0, true);
        settle().await;
        open(&bus, 0, false);
        wait_for_scan().await;
        assert_eq!(latest_warning(&bus), Some(true));

        // Cycle 2: place key + close door again → warning clears.
        place(&bus, 1, "Cabin");
        open(&bus, 0, true);
        settle().await;
        open(&bus, 0, false);
        wait_for_scan().await;
        assert_eq!(latest_warning(&bus), Some(false));
    }

    #[tokio::test]
    async fn warning_clears_on_ignition_off() {
        let bus = setup().await;
        set_ignition(&bus, "ON");
        settle().await;
        open(&bus, 0, true);
        settle().await;
        open(&bus, 0, false);
        wait_for_scan().await;
        assert_eq!(latest_warning(&bus), Some(true));

        set_ignition(&bus, "OFF");
        settle().await;
        assert_eq!(latest_warning(&bus), Some(false));
    }

    #[tokio::test]
    async fn partial_close_while_ignition_live_does_not_scan() {
        let bus = setup().await;
        set_ignition(&bus, "ON");
        place(&bus, 1, "Cabin");
        // Open both Row1 doors.
        open(&bus, 0, true);
        open(&bus, 1, true);
        settle().await;
        // Close only one — aggregate stays "any open".
        open(&bus, 0, false);
        wait_for_scan().await;
        assert_eq!(
            latest_warning(&bus),
            Some(false),
            "scan must wait until ALL doors are closed"
        );
    }
}
