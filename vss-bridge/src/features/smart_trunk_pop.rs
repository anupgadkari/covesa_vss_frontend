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
//! # Triggers
//!
//! Two paths land at the same scan + pop:
//!
//! ## Path A — direct, cabin-and-trunk locked together
//!
//! On a fresh `Vehicle.Cabin.LockStatus.EventNum` bump with all of:
//!
//! 1. The cabin transitioned into a locked state
//!    (`Vehicle.Cabin.LockStatus` ∈ {`LOCKED`, `DOUBLE_LOCKED`}).
//! 2. `LastRequestor` is an **external auth source** — any path
//!    the user invokes from outside the vehicle.  See
//!    [`EXTERNAL_LOCK_SOURCES`] below.
//! 3. `Vehicle.LowVoltageSystemState` is quiescent (`OFF` /
//!    `LOCK`) — `ACC` is occupant-present and excluded so we
//!    don't fight a user at the wheel.
//! 4. `Vehicle.Body.Trunk.Rear.IsOpen` is **false** — the cabin
//!    seal is complete; safe to scan.
//! 5. `dealer.smart_trunk_pop_enabled` is true (cal gate; default
//!    true).
//! 6. The vehicle is a PEPS build, not a KeyCylinder build.
//!
//! ## Path B — pending after trunk-close (power-tailgate case)
//!
//! Power-tailgate scenario: user locks the vehicle (RKE / PEPS /
//! phone …) while the trunk is **already open**.  The cabin
//! doors lock; the trunk stays open (it can't lock until it
//! closes).  The user then loads the trunk — fob inside the
//! backpack — and presses the close switch; the power tailgate
//! shuts.  *No new `EventNum` bump arrives* because the cabin
//! was already locked; only the `Trunk.IsOpen` edge fires.
//!
//! We handle this by latching a `pending_after_trunk_close` bit
//! whenever Path A's conditions hold *except* the trunk is open.
//! On the next `Trunk.IsOpen` true→false edge, if the latch is
//! still set and the gates still hold, run the scan as if a
//! fresh lock event had arrived.  The latch clears when:
//!
//! - The scan runs (consumed).
//! - The cabin leaves the locked state (driver unlocked — got
//!   their bag back themselves).
//! - Ignition transitions to non-quiescent (driver came back in).
//! - The dealer cal is toggled off.
//!
//! No time-window: the latch is purely state-driven.  If the user
//! never closes the trunk, the latch stays set indefinitely —
//! that's fine, it's a no-op until the trunk closes.
//!
//! ## Common path — scan + dispatch
//!
//! Submit `AntennaSet::TrunkInside` / `SearchMode::Authenticated`
//! / `Coalescing::Disallowed`.  On a non-empty result:
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
        // Power-tailgate latch — see Path B in the module docs.  Set
        // true when a qualifying lock event arrives while the trunk is
        // open; consumed on the trunk's true→false edge; cleared if
        // the cabin un-locks or ignition de-quiesces in the meantime.
        let mut pending_after_trunk_close = false;

        loop {
            select! {
                biased;
                Some(val) = status_rx.next() => {
                    if let SignalValue::String(s) = val {
                        lock_status = s;
                        // Driver unlocked (or someone else did) —
                        // they're back at the vehicle; don't re-fire
                        // a deferred trunk pop after the fact.
                        if !LOCKED_STATES.contains(&lock_status.as_str()) {
                            pending_after_trunk_close = false;
                        }
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
                        // Driver came back in (ignition no longer
                        // quiescent) — drop any latched pending pop;
                        // they have access to the cabin and can
                        // retrieve the bag themselves.
                        if !ignition_quiescent {
                            pending_after_trunk_close = false;
                        }
                    }
                }
                Some(val) = trunk_open_rx.next() => {
                    if let SignalValue::Bool(b) = val {
                        let was_open = trunk_open;
                        trunk_open = b;
                        // Power-tailgate close edge — if we latched
                        // a pending pop when the user locked with the
                        // trunk open, and the gates still hold, run
                        // the scan now.
                        if was_open && !trunk_open && pending_after_trunk_close {
                            pending_after_trunk_close = false;
                            if self.gates_still_hold(
                                &lock_status, &last_requestor, ignition_quiescent,
                            ) {
                                self.scan_and_maybe_pop("trunk_close_after_pending").await;
                            }
                        }
                    }
                }
                Some(_) = event_num_rx.next() => {
                    if !self.cfg.dealer_config().smart_trunk_pop_enabled {
                        // Cal off — clear any latched state so a
                        // re-enable mid-stream doesn't fire stale.
                        pending_after_trunk_close = false;
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
                        // Path B: defer until the trunk closes.
                        tracing::info!(
                            requestor = %last_requestor,
                            "SmartTrunkPop: external lock with trunk open — \
                             latching pending pop until trunk closes"
                        );
                        pending_after_trunk_close = true;
                        continue;
                    }

                    self.scan_and_maybe_pop("event_num_bump").await;
                }
                else => break,
            }
        }

        tracing::warn!("SmartTrunkPop: input stream closed, exiting");
    }

    /// Re-check the gates that must still hold by the time a deferred
    /// `pending_after_trunk_close` consumption fires.  Dealer cal
    /// included so a mid-stream cal toggle bites here too.
    fn gates_still_hold(
        &self,
        lock_status: &str,
        last_requestor: &str,
        ignition_quiescent: bool,
    ) -> bool {
        self.cfg.dealer_config().smart_trunk_pop_enabled
            && ignition_quiescent
            && LOCKED_STATES.contains(&lock_status)
            && EXTERNAL_LOCK_SOURCES.contains(&last_requestor)
    }

    /// Run the TrunkInside scan and, on a non-empty result, publish
    /// the `mislock_trunk` feedback cue and pulse the trunk arbiter.
    /// Empty result is a deliberate no-op (SmartUnlock owns the cabin
    /// case).
    async fn scan_and_maybe_pop(&self, trigger: &'static str) {
        tracing::info!(trigger, "SmartTrunkPop: running TrunkInside scan");

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
            return;
        };

        if result.keys_found.is_empty() {
            return;
        }

        tracing::warn!(
            trigger,
            keys = result.keys_found.len(),
            "SmartTrunkPop: paired key detected in trunk — popping trunk"
        );
        // Audible cue BEFORE the pop, same ordering as SmartUnlock —
        // the cue overlaps the still-fading lock chirp/flash so the
        // "lock didn't quite take" beat is clear.
        let _ = self
            .bus
            .publish(
                FEEDBACK_REQUEST,
                SignalValue::String("mislock_trunk".into()),
            )
            .await;
        self.pulse_trunk_open().await;
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

    // ── Power-tailgate scenario (Path B) ──────────────────────────

    /// User locks the vehicle while the trunk is open (cabin doors
    /// lock; trunk stays open), then later closes the trunk with the
    /// fob inside.  No new lock event fires on the trunk-close edge
    /// — SmartTrunkPop must remember the deferred decision and run
    /// the scan when the trunk closes.
    #[tokio::test]
    async fn lock_with_trunk_open_then_close_fires_on_trunk_close() {
        let bus = setup().await;

        // Driver opens the trunk to load it.
        bus.inject(TRUNK_OPEN, SignalValue::Bool(true));
        settle().await;

        // Driver locks the vehicle externally with the trunk still
        // open.  The lock event fires; SmartTrunkPop sees trunk_open
        // is true and latches a deferred pop.
        fire_lock_event(&bus, "LOCKED", "KeyfobRke", 1);
        settle().await;
        assert!(
            !trunk_pop_dispatched(&bus),
            "must not pop while the trunk is still open"
        );

        // Driver puts the bag (with paired fob) in the trunk and
        // presses the power-tailgate close switch.
        place(&bus, 1, Zone::TrunkInside);
        bus.inject(TRUNK_OPEN, SignalValue::Bool(false));
        settle().await;

        assert!(
            trunk_pop_dispatched(&bus),
            "trunk-close edge after a deferred lock must run the scan + pop"
        );
        assert!(
            mislock_trunk_published(&bus),
            "audible cue must accompany the deferred pop"
        );
    }

    /// Same starting sequence, but the driver unlocks before closing
    /// the trunk (e.g. realises they need something else).  The
    /// latch must drop on the unlock — closing the trunk later must
    /// NOT fire a stale deferred pop.
    #[tokio::test]
    async fn unlock_clears_pending_after_trunk_close_latch() {
        let bus = setup().await;

        bus.inject(TRUNK_OPEN, SignalValue::Bool(true));
        settle().await;
        fire_lock_event(&bus, "LOCKED", "KeyfobRke", 1);
        settle().await;

        // Driver unlocks (decides to take the bag with them after all).
        fire_lock_event(&bus, "UNLOCKED", "KeyfobRke", 2);
        settle().await;

        // Now they close the trunk — latch should be gone.
        place(&bus, 1, Zone::TrunkInside);
        bus.inject(TRUNK_OPEN, SignalValue::Bool(false));
        settle().await;

        assert!(
            !trunk_pop_dispatched(&bus),
            "unlock between deferred lock and trunk-close must drop the latch"
        );
    }

    /// Same starting sequence, but the driver gets back in the
    /// vehicle (ignition transitions to ON / START / ACC) before
    /// closing the trunk.  Latch must drop — they have cabin access.
    #[tokio::test]
    async fn ignition_returns_clears_pending_latch() {
        let bus = setup().await;

        bus.inject(TRUNK_OPEN, SignalValue::Bool(true));
        settle().await;
        fire_lock_event(&bus, "LOCKED", "KeyfobRke", 1);
        settle().await;

        // Driver gets back in.
        bus.inject(IGNITION_STATE, SignalValue::String("ON".into()));
        settle().await;

        // Trunk closes later.
        place(&bus, 1, Zone::TrunkInside);
        bus.inject(TRUNK_OPEN, SignalValue::Bool(false));
        settle().await;

        assert!(
            !trunk_pop_dispatched(&bus),
            "ignition non-quiescent must clear the latch"
        );
    }

    /// No paired key actually ends up in the trunk (e.g. user put
    /// the fob in their pocket after all): the scan runs on the
    /// trunk-close edge but returns empty → no pop.
    #[tokio::test]
    async fn pending_pop_with_empty_scan_is_no_op() {
        let bus = setup().await;

        bus.inject(TRUNK_OPEN, SignalValue::Bool(true));
        settle().await;
        fire_lock_event(&bus, "LOCKED", "KeyfobRke", 1);
        settle().await;

        // Trunk closes; no paired key placed inside.
        bus.inject(TRUNK_OPEN, SignalValue::Bool(false));
        settle().await;

        assert!(!trunk_pop_dispatched(&bus));
        assert!(!mislock_trunk_published(&bus));
    }
}
