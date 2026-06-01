//! Smart Trunk Pop — re-pops the trunk on an external lock event
//! when a paired key is detected inside the cargo area.
//!
//! # Customer-visible behaviour
//!
//! Driver loads the trunk, closes the lid, locks the vehicle from
//! outside (RKE button, walk-away to PEPS range, NFC tap on the
//! handle, phone app, keypad …) — but a paired phone or fob is
//! still in the trunk.  The classic "I just locked my partner's
//! phone in the trunk" failure mode.  Smart Trunk Pop notices and
//! pops the trunk (doors stay locked) with an audible cue so the
//! user retrieves the device.
//!
//! # Why a separate feature from SmartUnlock (#15)
//!
//! The remediation is different.  SmartUnlock's job is to undo a
//! lock when a paired key is in the *cabin* — it dispatches
//! `UnlockAll` because someone needs to get back in.  A key locked
//! in the *trunk* is the opposite situation: the user is outside
//! and just walked away; reopening the cabin would invite a
//! passerby in.  Popping the trunk alone is the right answer —
//! easy to retrieve the item, doors stay locked, perimeter intact.
//!
//! SmartUnlock has a regression test asserting it does NOT cover
//! `Zone::TrunkInside` (`key_in_trunk_inside_alone_does_not_unlock`).
//! This feature is the missing complement.
//!
//! # Trigger
//!
//! On a fresh `Vehicle.Cabin.LockStatus.EventNum` bump with all of:
//!
//! 1. The cabin transitioned into a locked state
//!    (`Vehicle.Cabin.LockStatus` ∈ {`LOCKED`, `DOUBLE_LOCKED`}).
//! 2. `LastRequestor` is an **external auth source** — any path
//!    the user invokes from outside the vehicle.  See
//!    [`EXTERNAL_LOCK_SOURCES`] below.
//! 3. `Vehicle.LowVoltageSystemState` is quiescent
//!    (`OFF` / `LOCK`) — `ACC` is occupant-present and excluded
//!    so we don't fight a user at the wheel.
//! 4. `Vehicle.Body.Trunk.Rear.IsOpen` is **false** — if someone's
//!    already at the trunk, there's nothing to pop.
//! 5. `dealer.smart_trunk_pop_enabled` is true (cal gate; default
//!    true).
//! 6. The vehicle is a PEPS build, not a KeyCylinder build (no
//!    interior LF antennas on KeyCylinder trims).
//!
//! When all hold, submit `AntennaSet::TrunkInside` /
//! `SearchMode::Authenticated` / `Coalescing::Disallowed`.  On a
//! non-empty result:
//!
//! - Publish `FeedbackRequest = "mislock_trunk"` (audible cue —
//!   distinct from SmartUnlock's `"mislock"` so the cabin and
//!   trunk cases can play different chime sequences if/when
//!   LockFeedback adds variants).
//! - Pulse `Vehicle.Controller.Body.Trunk.Rear.OpenCmd` through the
//!   trunk arbiter (request `true` then immediately release) as
//!   `FeatureId::SmartTrunkPop`.
//!
//! On an empty result, do nothing — the cabin case is SmartUnlock's
//! call.  No internal cooldown / latch: a subsequent external lock
//! event (e.g. AutoRelock kicking after a manual unlock) will be
//! re-evaluated naturally.
//!
//! # Risks
//!
//! - **Latency window.**  The trunk pop happens after a real
//!   arbiter scan (~100 ms simulated LF airtime) plus arbiter
//!   request serialisation.  The audible `"mislock_trunk"` cue
//!   is essential to make the cause-and-effect clear.
//! - **Phantom-fob trigger.**  An unpaired blank wouldn't trigger
//!   because the scan is `Authenticated` — the HMAC challenge
//!   filters unpaired fobs out at the arbiter layer.
//! - **AutoRelock loop.**  After the pop, doors stay locked.  If
//!   AutoRelock is enabled and the trunk closes again, the trigger
//!   only re-fires if a *new* external lock event comes through —
//!   AutoRelock fires as `FeatureId::AutoRelock`, which is NOT in
//!   `EXTERNAL_LOCK_SOURCES`, so the loop cannot perpetuate.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::arbiter::{ActuatorRequest, DomainArbiter, FEEDBACK_REQUEST, TRUNK_OPEN_CMD};
use crate::config::{KeySource, PlatformConfig};
use crate::features::key_search_arbiter::{
    AntennaSet, Coalescing, KeySearchArbiterHandle, SearchMode,
};
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

// ── Signal constants ───────────────────────────────────────────────────────

const LOCK_STATUS: VssPath = "Vehicle.Cabin.LockStatus";
const LAST_REQUESTOR: VssPath = "Vehicle.Cabin.LockStatus.LastRequestor";
const LOCK_EVENT_NUM: VssPath = "Vehicle.Cabin.LockStatus.EventNum";
const IGNITION_STATE: VssPath = "Vehicle.LowVoltageSystemState";
const TRUNK_OPEN: VssPath = "Vehicle.Body.Trunk.Rear.IsOpen";

// ── Constants ──────────────────────────────────────────────────────────────

const FEATURE_ID: FeatureId = FeatureId::SmartTrunkPop;

/// `LastRequestor` strings that should trigger Smart Trunk Pop.
///
/// Wider than SmartUnlock's set because the multi-fob / partner-
/// phone-in-trunk scenario is real: the user can have a paired
/// credential on their person AND a *different* paired credential
/// in the trunk.  So even sources that confirm "a credential
/// holder is at the vehicle" (RKE press, PEPS handle touch) can
/// legitimately precede a trunk-inside finding for a *second*
/// paired device.  We trigger on every external lock path and
/// rely on the `Authenticated` scan + the "trunk closed" gate to
/// filter false positives.
///
/// Internal lock paths (`DoorTrimButton`, `SlamLock`) are excluded
/// — they imply a user inside the cabin who isn't interacting with
/// the trunk.  Auto-paths (`AutoLock`, `AutoRelock`, `WalkAwayLock`)
/// are excluded too: they fire long after the user departed and
/// piling on a delayed trunk pop would be confusing rather than
/// helpful.
const EXTERNAL_LOCK_SOURCES: &[&str] = &[
    "KeyfobRke",
    "KeyfobPeps",
    "PassiveEntry",
    "KeypadLock",
    "PhoneApp",
    "PhoneBle",
    "NfcCard",
    "NfcPhone",
];

const LOCKED_STATES: &[&str] = &["LOCKED", "DOUBLE_LOCKED"];

/// `Vehicle.LowVoltageSystemState` values that mean "nobody is at
/// the wheel — safe to act on an external lock event."  Matches
/// SmartUnlock's gate.
fn ignition_is_quiescent(s: &str) -> bool {
    matches!(s, "OFF" | "LOCK")
}

// ── Feature struct ─────────────────────────────────────────────────────────

pub struct SmartTrunkPop<B: SignalBus> {
    bus: Arc<B>,
    trunk_arb: Arc<DomainArbiter>,
    key_search: KeySearchArbiterHandle,
    cfg: Arc<PlatformConfig>,
}

impl<B: SignalBus + Send + Sync + 'static> SmartTrunkPop<B> {
    pub fn new(
        bus: Arc<B>,
        trunk_arb: Arc<DomainArbiter>,
        key_search: KeySearchArbiterHandle,
        cfg: Arc<PlatformConfig>,
    ) -> Self {
        Self {
            bus,
            trunk_arb,
            key_search,
            cfg,
        }
    }

    pub async fn run(self) {
        // KeyCylinder builds have no interior LF antennas in the
        // trunk; the TrunkInside scan would never find anything.
        // Skip the run loop to save CPU.
        if self.cfg.vehicle_line.key_source_cfg != KeySource::Peps {
            tracing::info!("SmartTrunkPop: not a PEPS vehicle, feature disabled");
            return;
        }

        tracing::info!("SmartTrunkPop feature started");

        let mut status_rx = self.bus.subscribe(LOCK_STATUS).await;
        let mut requestor_rx = self.bus.subscribe(LAST_REQUESTOR).await;
        let mut event_num_rx = self.bus.subscribe(LOCK_EVENT_NUM).await;
        let mut ignition_rx = self.bus.subscribe(IGNITION_STATE).await;
        let mut trunk_open_rx = self.bus.subscribe(TRUNK_OPEN).await;

        // Local caches.  The arbiter publishes status → requestor →
        // event_num on every lock command; `biased` select! ordering
        // ensures the caches are coherent by the time event_num fires.
        let mut lock_status: String = String::new();
        let mut last_requestor: String = String::new();
        // Ignition unknown at boot = treat as quiescent (matches VSS
        // defaults and SmartUnlock's behaviour).
        let mut ignition_quiescent = true;
        let mut trunk_open = false;

        loop {
            select! {
                biased;
                Some(val) = status_rx.next() => {
                    if let SignalValue::String(s) = val {
                        lock_status = s;
                    }
                }
                Some(val) = requestor_rx.next() => {
                    if let SignalValue::String(s) = val {
                        last_requestor = s;
                    }
                }
                Some(val) = ignition_rx.next() => {
                    if let SignalValue::String(s) = val {
                        ignition_quiescent = ignition_is_quiescent(&s);
                    }
                }
                Some(val) = trunk_open_rx.next() => {
                    if let SignalValue::Bool(b) = val {
                        trunk_open = b;
                    }
                }
                Some(_) = event_num_rx.next() => {
                    if !self.cfg.dealer_config().smart_trunk_pop_enabled {
                        continue;
                    }
                    if !ignition_quiescent {
                        continue;
                    }
                    if !LOCKED_STATES.contains(&lock_status.as_str()) {
                        continue;
                    }
                    if !EXTERNAL_LOCK_SOURCES.contains(&last_requestor.as_str()) {
                        continue;
                    }
                    if trunk_open {
                        tracing::debug!(
                            "SmartTrunkPop: trunk already open — no-op"
                        );
                        continue;
                    }

                    tracing::info!(
                        requestor = %last_requestor,
                        status = %lock_status,
                        "SmartTrunkPop: qualifying external lock — running TrunkInside scan"
                    );

                    let result = self
                        .key_search
                        .submit(
                            "SmartTrunkPop",
                            AntennaSet::TrunkInside,
                            SearchMode::Authenticated,
                            Coalescing::Disallowed,
                        )
                        .await;
                    let Some(result) = result else {
                        tracing::warn!("SmartTrunkPop: key search returned no result");
                        continue;
                    };

                    if result.keys_found.is_empty() {
                        // No key in the trunk — cabin case is
                        // SmartUnlock's call, we stay out of it.
                        continue;
                    }

                    tracing::warn!(
                        keys = result.keys_found.len(),
                        "SmartTrunkPop: paired key detected in trunk — popping trunk"
                    );
                    // Audible cue BEFORE the pop, same ordering as
                    // SmartUnlock — the cue overlaps the still-
                    // fading lock chirp/flash so the "lock didn't
                    // quite take" beat is clear.
                    let _ = self
                        .bus
                        .publish(
                            FEEDBACK_REQUEST,
                            SignalValue::String("mislock_trunk".into()),
                        )
                        .await;
                    self.pulse_trunk_open().await;
                }
                else => break,
            }
        }

        tracing::warn!("SmartTrunkPop: input stream closed, exiting");
    }

    /// Pulse `Vehicle.Controller.Body.Trunk.Rear.OpenCmd` through the trunk
    /// arbiter as a momentary edge: request `true`, then immediately
    /// release so the arbiter publishes `true` → `false` and a
    /// subsequent trigger can re-fire.  Same shape ExteriorTrunkButton
    /// and CabinTrunkRelease use.
    async fn pulse_trunk_open(&self) {
        let _ = self
            .trunk_arb
            .request(ActuatorRequest {
                signal: TRUNK_OPEN_CMD,
                value: SignalValue::Bool(true),
                priority: Priority::Medium,
                feature_id: FEATURE_ID,
            })
            .await;
        let _ = self.trunk_arb.release(TRUNK_OPEN_CMD, FEATURE_ID).await;
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::trunk_arbiter;
    use crate::config::DealerConfig;
    use crate::features::key_search_arbiter::KeySearchArbiter;
    use crate::plant_models::peps::zone::Zone;
    use std::time::Duration;

    async fn settle() {
        // KeySearchArbiter run_scan sleeps a real 100 ms for
        // TrunkInside + Authenticated.  Wait long enough for the
        // arbiter task to return through the oneshot.
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
        tokio::time::sleep(Duration::from_millis(120)).await;
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    fn peps_cfg() -> Arc<PlatformConfig> {
        PlatformConfig::load()
    }

    fn keycylinder_cfg() -> Arc<PlatformConfig> {
        let cfg = PlatformConfig::load();
        let mut vl = cfg.vehicle_line.clone();
        vl.key_source_cfg = KeySource::KeyCylinder;
        PlatformConfig::with_vehicle_line(vl)
    }

    /// Boot a fresh bus + trunk arbiter + KeySearchArbiter + the
    /// feature under test.  Returns the bus so tests can inject /
    /// observe; the trunk arbiter handle is captured inside the
    /// feature.
    async fn setup_with(cfg: Arc<PlatformConfig>) -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let (tarb, tarb_fut) = trunk_arbiter(Arc::clone(&bus));
        tokio::spawn(tarb_fut);
        let tarb = Arc::new(tarb);

        let (ksa, ksa_handle, ksa_rx) = KeySearchArbiter::new_with_rx(Arc::clone(&bus));
        tokio::spawn(ksa.run(ksa_rx));

        // Seed the trunk-closed baseline so the gate is reachable.
        bus.inject(TRUNK_OPEN, SignalValue::Bool(false));
        bus.inject(IGNITION_STATE, SignalValue::String("OFF".into()));

        let feature = SmartTrunkPop::new(Arc::clone(&bus), tarb, ksa_handle, cfg);
        tokio::spawn(feature.run());

        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
        bus
    }

    async fn setup() -> Arc<MockBus> {
        setup_with(peps_cfg()).await
    }

    /// Place a paired fob in a zone — the arbiter's `Authenticated`
    /// search filters out unpaired fobs, so set the `Paired` flag
    /// too.  Slots 1-4 are PEPS plant fobs.
    fn place(bus: &MockBus, slot: u8, zone: Zone) {
        let zone_path: &'static str = match slot {
            1 => "Vehicle.Simulation.KeyFob.1.PlacedZone",
            2 => "Vehicle.Simulation.KeyFob.2.PlacedZone",
            3 => "Vehicle.Simulation.KeyFob.3.PlacedZone",
            _ => panic!("unknown slot"),
        };
        let paired_path: &'static str = match slot {
            1 => "Vehicle.Simulation.KeyFob.1.Paired",
            2 => "Vehicle.Simulation.KeyFob.2.Paired",
            3 => "Vehicle.Simulation.KeyFob.3.Paired",
            _ => unreachable!(),
        };
        bus.inject(zone_path, SignalValue::String(zone.as_str().into()));
        bus.inject(paired_path, SignalValue::Bool(true));
    }

    /// Publish the lock tuple in the same order the door-lock
    /// arbiter does (status → requestor → event_num).  SmartUnlock's
    /// tests use the same pattern.
    fn fire_lock_event(bus: &MockBus, status: &str, requestor: &str, event_num: u16) {
        bus.inject(LOCK_STATUS, SignalValue::String(status.into()));
        bus.inject(LAST_REQUESTOR, SignalValue::String(requestor.into()));
        bus.inject(LOCK_EVENT_NUM, SignalValue::Uint16(event_num));
    }

    fn trunk_pop_dispatched(bus: &MockBus) -> bool {
        bus.history()
            .iter()
            .any(|(sig, val)| *sig == TRUNK_OPEN_CMD && *val == SignalValue::Bool(true))
    }

    fn mislock_trunk_published(bus: &MockBus) -> bool {
        bus.history().iter().any(|(sig, val)| {
            *sig == FEEDBACK_REQUEST && *val == SignalValue::String("mislock_trunk".into())
        })
    }

    /// Happy path: paired key in TrunkInside, external lock → trunk
    /// pops, mislock_trunk cue published.
    #[tokio::test]
    async fn key_in_trunk_with_external_lock_pops_trunk() {
        let bus = setup().await;
        place(&bus, 1, Zone::TrunkInside);

        // Externally-locked.  PhoneApp is the prototypical case.
        fire_lock_event(&bus, "LOCKED", "PhoneApp", 1);
        settle().await;

        assert!(
            trunk_pop_dispatched(&bus),
            "trunk should have popped via the trunk arbiter"
        );
        assert!(
            mislock_trunk_published(&bus),
            "FeedbackRequest = mislock_trunk should have been published"
        );
    }

    /// Cabin case: paired key in Cabin (not TrunkInside) + external
    /// lock → SmartTrunkPop is a no-op (SmartUnlock owns this case).
    #[tokio::test]
    async fn key_in_cabin_alone_does_not_pop_trunk() {
        let bus = setup().await;
        place(&bus, 1, Zone::Cabin);

        fire_lock_event(&bus, "LOCKED", "PhoneApp", 1);
        settle().await;

        assert!(
            !trunk_pop_dispatched(&bus),
            "cabin-only finding is SmartUnlock's job, not SmartTrunkPop's"
        );
        assert!(!mislock_trunk_published(&bus));
    }

    /// Both cabin and trunk have paired keys → trunk pops (the
    /// cabin unlock is handled separately by SmartUnlock running
    /// alongside, which this feature does not depend on).
    #[tokio::test]
    async fn key_in_trunk_and_cabin_still_pops_trunk() {
        let bus = setup().await;
        place(&bus, 1, Zone::TrunkInside);
        place(&bus, 2, Zone::Cabin);

        fire_lock_event(&bus, "LOCKED", "PhoneApp", 1);
        settle().await;

        assert!(
            trunk_pop_dispatched(&bus),
            "trunk pop should fire regardless of what else is in the cabin"
        );
    }

    /// No paired key anywhere → no-op.
    #[tokio::test]
    async fn no_key_in_trunk_does_not_pop() {
        let bus = setup().await;
        // Place a paired fob somewhere irrelevant so the arbiter
        // sees authentication state and doesn't default-accept.
        place(&bus, 1, Zone::Approach);

        fire_lock_event(&bus, "LOCKED", "PhoneApp", 1);
        settle().await;

        assert!(!trunk_pop_dispatched(&bus));
    }

    /// Trunk is already open at lock time → no-op (don't pop an
    /// already-open trunk).
    #[tokio::test]
    async fn trunk_already_open_at_lock_time_is_no_op() {
        let bus = setup().await;
        place(&bus, 1, Zone::TrunkInside);
        bus.inject(TRUNK_OPEN, SignalValue::Bool(true));
        settle().await;

        fire_lock_event(&bus, "LOCKED", "PhoneApp", 1);
        settle().await;

        assert!(!trunk_pop_dispatched(&bus));
    }

    /// KeyCylinder build → feature is no-op'd at start; never fires.
    #[tokio::test]
    async fn keycylinder_build_does_not_pop() {
        let bus = setup_with(keycylinder_cfg()).await;
        place(&bus, 1, Zone::TrunkInside);

        fire_lock_event(&bus, "LOCKED", "PhoneApp", 1);
        settle().await;

        assert!(
            !trunk_pop_dispatched(&bus),
            "KeyCylinder build must not run the trunk-pop logic"
        );
    }

    /// Internal-source lock (DoorTrimButton) → no-op.  User is
    /// inside the cabin; popping their trunk from the trim button
    /// would be surprising.
    #[tokio::test]
    async fn internal_source_lock_does_not_pop() {
        let bus = setup().await;
        place(&bus, 1, Zone::TrunkInside);

        fire_lock_event(&bus, "LOCKED", "DoorTrimButton", 1);
        settle().await;

        assert!(!trunk_pop_dispatched(&bus));
    }

    /// AutoRelock kick after a successful pop → no perpetual loop.
    /// AutoRelock's identity is NOT in EXTERNAL_LOCK_SOURCES, so a
    /// follow-up auto-lock event does NOT re-trigger SmartTrunkPop.
    #[tokio::test]
    async fn autorelock_after_pop_does_not_re_trigger() {
        let bus = setup().await;
        place(&bus, 1, Zone::TrunkInside);

        // First lock (external) — pops the trunk.
        fire_lock_event(&bus, "LOCKED", "PhoneApp", 1);
        settle().await;
        assert!(trunk_pop_dispatched(&bus));

        // Trunk closes, AutoRelock fires a fresh LOCKED event.
        bus.inject(TRUNK_OPEN, SignalValue::Bool(false));
        bus.clear_history();
        fire_lock_event(&bus, "LOCKED", "AutoRelock", 2);
        settle().await;

        assert!(
            !trunk_pop_dispatched(&bus),
            "AutoRelock is not an external lock source; must not re-trigger"
        );
    }

    /// Dealer cal `smart_trunk_pop_enabled = false` → feature is
    /// dormant even with all other conditions met.
    #[tokio::test]
    async fn disabled_via_dealer_cal_does_not_pop() {
        let cfg = peps_cfg();
        let mut dc = cfg.dealer_config();
        dc.smart_trunk_pop_enabled = false;
        cfg.update_dealer_config(dc);

        let bus = setup_with(cfg).await;
        place(&bus, 1, Zone::TrunkInside);

        fire_lock_event(&bus, "LOCKED", "PhoneApp", 1);
        settle().await;

        assert!(!trunk_pop_dispatched(&bus));
    }

    /// Ignition `ACC` (occupant present) → no-op.
    #[tokio::test]
    async fn ignition_acc_is_no_op() {
        let bus = setup().await;
        place(&bus, 1, Zone::TrunkInside);
        bus.inject(IGNITION_STATE, SignalValue::String("ACC".into()));
        settle().await;

        fire_lock_event(&bus, "LOCKED", "PhoneApp", 1);
        settle().await;

        assert!(!trunk_pop_dispatched(&bus));
    }

    /// Quietness check: the implementation imports `Duration` and
    /// `DealerConfig` only for tests — keep the unused-import
    /// surface explicit so clippy stays clean.
    #[allow(dead_code)]
    fn _imports_used(_d: Duration, _dc: DealerConfig) {}
}
