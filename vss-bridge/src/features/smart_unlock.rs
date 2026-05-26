//! Smart Unlock — prevents key-locked-in-cabin scenarios on PEPS
//! vehicles.
//!
//! # Customer-visible behaviour
//!
//! Driver drops a paired fob on the passenger seat, gets out, and
//! locks the door from outside with a different key (or a phone-app
//! lock, or a passenger taps the trim lock then closes the door).
//! Without intervention the owner's primary fob is now sealed inside
//! a locked vehicle.  Smart Unlock notices this on the lock edge and
//! immediately re-unlocks the vehicle so the fob is recoverable
//! without resorting to a spare key / dealer call.
//!
//! # Trigger
//!
//! On every `Cabin.LockStatus.EventNum` bump (the arbiter's "this
//! lock event is fully published" wakeup, with `Cabin.LockStatus` and
//! `Cabin.LockStatus.LastRequestor` guaranteed coherent), evaluate:
//!
//! 1. Vehicle is on a PEPS line (`key_source_cfg == Peps`).
//!    Cylinder-key vehicles physically can't lock with the key in
//!    the cabin (the driver holds the mechanical blade), so the
//!    feature is meaningless there.
//! 2. Ignition is OFF (`Vehicle.LowVoltageSystemState ∈ {OFF, LOCK}`).
//!    ACC / ON / START mean somebody is at the wheel — Smart Unlock
//!    must not contradict an active driver.
//! 3. `Cabin.LockStatus` ∈ {`LOCKED`, `DOUBLE_LOCKED`} — only a fresh
//!    lock event matters.
//! 4. `LastRequestor` is in `EXTERNAL_LOCK_SOURCES` — interior
//!    `DoorTrimButton` presses are an explicit "lock yourself in"
//!    action and must not auto-undo.  `AutoLock` is also excluded
//!    (occupant-driven autolock); `AutoRelock` is excluded (its job
//!    is to re-secure a vehicle the user already walked away from —
//!    overriding it would defeat the feature).
//!
//! If all four hold, run a [`Sequence`] scan covering the cabin and
//! the on-vehicle approach zones, both [`Authenticated`] (pairing
//! filter on — a stranger's fob does not count):
//!
//! - [`AntennaSet::Cabin`]        — paired key on the seat / floor.
//! - [`AntennaSet::AllApproach`]  — paired key in any external
//!   proximity zone (LeftFront, RightFront, Hood, Trunk, Approach).
//!
//! Unlock decision:
//!
//! - **At least one paired key found in Cabin** AND
//! - **Zero paired keys found in any approach zone**
//!
//! → dispatch [`LockCommand::UnlockAll`] through the door-lock
//!   arbiter as [`FeatureId::SmartUnlock`].  Otherwise leave the
//!   vehicle locked.
//!
//! # Why not include TrunkInside?
//!
//! A fob locked in the trunk is the "lost-key-in-trunk" / boot-pop
//! scenario — distinct customer flow with different remediation
//! (driver wants the trunk popped, not the doors unlocked).  Smart
//! Unlock here is strictly the cabin case.
//!
//! # Why not retry?
//!
//! The arbiter publishes a fresh `EventNum` on every accepted lock
//! command, so if a subsequent lock attempt occurs (e.g. an
//! AutoRelock kick after our UnlockAll), Smart Unlock will re-evaluate
//! naturally.  No internal cooldown / one-shot latch needed.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::arbiter::{DoorLockArbiter, DoorLockRequest, LockCommand};
use crate::config::{KeySource, PlatformConfig};
use crate::features::key_search_arbiter::{
    AntennaSet, Coalescing, KeySearchArbiterHandle, SearchMode,
};
use crate::ipc_message::{FeatureId, SignalValue};
use crate::plant_models::peps::zone::Zone;
use crate::signal_bus::{SignalBus, VssPath};

const LOCK_STATUS: VssPath = "Cabin.LockStatus";
const LAST_REQUESTOR: VssPath = "Cabin.LockStatus.LastRequestor";
const LOCK_EVENT_NUM: VssPath = "Cabin.LockStatus.EventNum";
const IGNITION_STATE: VssPath = "Vehicle.LowVoltageSystemState";

/// `LastRequestor` strings that count as an "exterior lock source"
/// for Smart Unlock.  These are the cases where the user is plausibly
/// outside the vehicle and could have left a paired key behind:
///
/// - `KeyfobRke` / `KeyfobPeps` — physical fob lock press / capacitive
///   handle touch.
/// - `PassiveEntry` — handle-touch lock confirmation.
/// - `ThumbPadLock` — exterior thumb-pad.
/// - `PhoneApp` / `PhoneBle` — remote / proximity phone lock.
/// - `NfcCard` / `NfcPhone` — B-pillar tap lock.
/// - `SlamLock` — slam-lock-with-trim-button → door-close pattern,
///   user is by definition outside when the door clicks shut.
/// - `WalkAwayLock` — driver walked away from a still-unlocked
///   vehicle and the timer secured it; the literal scenario this
///   feature exists to undo.
///
/// **Deliberately excluded:**
/// - `DoorTrimButton` — interior trim press; user explicitly chose to
///   lock the cabin from the inside.  Not Smart Unlock's job to
///   second-guess that.
/// - `AutoLock` — drive-away auto-lock fires with the occupant in the
///   seat.
/// - `AutoRelock` — re-secures a vehicle the user walked away from
///   after an unlock that produced no door open.  Overriding it here
///   would defeat its purpose.
/// - `CrashUnlock` / `DoubleLockRelease` — neither produces a LOCK
///   transition.
const EXTERNAL_LOCK_SOURCES: &[&str] = &[
    "KeyfobRke",
    "KeyfobPeps",
    "PassiveEntry",
    "ThumbPadLock",
    "PhoneApp",
    "PhoneBle",
    "NfcCard",
    "NfcPhone",
    "SlamLock",
    "WalkAwayLock",
];

const LOCKED_STATES: &[&str] = &["LOCKED", "DOUBLE_LOCKED"];

/// `Vehicle.LowVoltageSystemState` values that mean "nobody is at the
/// wheel — safe to act on an external lock event."  `ACC` is treated
/// as occupant-present (radio on, keys turned one click) and excluded
/// so Smart Unlock does not interfere with somebody actively using
/// accessory power.
fn ignition_is_quiescent(s: &str) -> bool {
    matches!(s, "OFF" | "LOCK")
}

pub struct SmartUnlock<B: SignalBus> {
    bus: Arc<B>,
    arbiter: Arc<DoorLockArbiter>,
    key_search: KeySearchArbiterHandle,
    cfg: Arc<PlatformConfig>,
}

impl<B: SignalBus + Send + Sync + 'static> SmartUnlock<B> {
    pub fn new(
        bus: Arc<B>,
        arbiter: Arc<DoorLockArbiter>,
        key_search: KeySearchArbiterHandle,
        cfg: Arc<PlatformConfig>,
    ) -> Self {
        Self {
            bus,
            arbiter,
            key_search,
            cfg,
        }
    }

    pub async fn run(self) {
        // KeyCylinder vehicles can't get into this state — the driver
        // physically holds the only mechanical key.  Don't spawn a
        // hot loop that will never fire.
        if self.cfg.vehicle_line.key_source_cfg != KeySource::Peps {
            tracing::info!(
                "SmartUnlock: not a PEPS vehicle, feature disabled"
            );
            return;
        }

        tracing::info!("SmartUnlock feature started");

        let mut status_rx = self.bus.subscribe(LOCK_STATUS).await;
        let mut requestor_rx = self.bus.subscribe(LAST_REQUESTOR).await;
        let mut event_num_rx = self.bus.subscribe(LOCK_EVENT_NUM).await;
        let mut ignition_rx = self.bus.subscribe(IGNITION_STATE).await;

        // Local caches.  The arbiter publishes status → requestor →
        // event_num on every lock command; `biased` select! ordering
        // ensures the caches are coherent by the time event_num fires.
        let mut lock_status: String = String::new();
        let mut last_requestor: String = String::new();
        // Ignition unknown at boot = treat as quiescent (matches the
        // VSS default).  Updated on the first publish.
        let mut ignition_quiescent = true;

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
                Some(_) = event_num_rx.next() => {
                    if !ignition_quiescent {
                        continue;
                    }
                    if !LOCKED_STATES.contains(&lock_status.as_str()) {
                        continue;
                    }
                    if !EXTERNAL_LOCK_SOURCES.contains(&last_requestor.as_str()) {
                        continue;
                    }

                    tracing::info!(
                        requestor = %last_requestor,
                        status = %lock_status,
                        "SmartUnlock: qualifying external lock — running scan"
                    );

                    // Sequence: Cabin first (the predicate we care
                    // about most), then AllApproach.  Authenticated on
                    // both so unpaired fobs are excluded.  Coalescing
                    // disallowed — safety-class decision; a stale
                    // result could leave a paired key locked in.
                    let result = self
                        .key_search
                        .submit(
                            "SmartUnlock",
                            AntennaSet::Sequence(vec![
                                (AntennaSet::Cabin, SearchMode::Authenticated),
                                (AntennaSet::AllApproach, SearchMode::Authenticated),
                            ]),
                            SearchMode::Authenticated,
                            Coalescing::Disallowed,
                        )
                        .await;
                    let Some(result) = result else {
                        tracing::warn!("SmartUnlock: key search returned no result");
                        continue;
                    };

                    let mut key_in_cabin = false;
                    let mut key_outside = false;
                    for finding in &result.keys_found {
                        if finding.zone == Zone::Cabin {
                            key_in_cabin = true;
                        } else if is_outside_zone(finding.zone) {
                            key_outside = true;
                        }
                    }

                    if key_in_cabin && !key_outside {
                        tracing::warn!(
                            "SmartUnlock: paired key detected in cabin with no key \
                             outside — dispatching UnlockAll"
                        );
                        if let Err(e) = self
                            .arbiter
                            .request(DoorLockRequest {
                                command: LockCommand::UnlockAll,
                                feature_id: FeatureId::SmartUnlock,
                            })
                            .await
                        {
                            tracing::error!(error = %e, "SmartUnlock: arbiter request failed");
                        }
                    } else {
                        tracing::debug!(
                            key_in_cabin,
                            key_outside,
                            "SmartUnlock: predicate not satisfied — leaving locked"
                        );
                    }
                }
                else => break,
            }
        }

        tracing::warn!("SmartUnlock: input stream closed, exiting");
    }
}

/// Zones that represent "the key is reachable by somebody standing
/// outside the vehicle."  Anyone in approach distance could plausibly
/// be holding the paired key — Smart Unlock must NOT re-open the
/// cabin in that case, because doing so would defeat a deliberate
/// "lock the doors while I walk away" gesture.
fn is_outside_zone(z: Zone) -> bool {
    matches!(
        z,
        Zone::LeftFront
            | Zone::RightFront
            | Zone::Hood
            | Zone::Trunk
            | Zone::Approach
    )
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::{door_lock_arbiter, CENTRAL_LOCK_CMD};
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

    async fn wait_for_scan() {
        tokio::time::sleep(Duration::from_millis(250)).await;
        settle().await;
    }

    fn make_cfg(src: KeySource) -> Arc<PlatformConfig> {
        use crate::config::{DealerConfig, VariantCal, VehicleLineCal};
        use tokio::sync::watch;
        let vl = VehicleLineCal {
            key_source_cfg: src,
            ..VehicleLineCal::default()
        };
        let (tx, rx) = watch::channel(DealerConfig::default());
        Arc::new(PlatformConfig::test_construct(
            vl,
            VariantCal::default(),
            tx,
            rx,
        ))
    }

    fn peps_cfg() -> Arc<PlatformConfig> {
        make_cfg(KeySource::Peps)
    }

    fn cylinder_cfg() -> Arc<PlatformConfig> {
        make_cfg(KeySource::KeyCylinder)
    }

    async fn setup_with(cfg: Arc<PlatformConfig>) -> (Arc<MockBus>, Arc<DoorLockArbiter>) {
        let bus = Arc::new(MockBus::new());
        let (lock_arb, _ack_tx, lock_loop) = door_lock_arbiter(Arc::clone(&bus));
        tokio::spawn(lock_loop);
        let lock_arb = Arc::new(lock_arb);
        let (ksa, handle, rx) = KeySearchArbiter::new_with_rx(Arc::clone(&bus));
        tokio::spawn(ksa.run(rx));
        tokio::spawn(
            SmartUnlock::new(Arc::clone(&bus), Arc::clone(&lock_arb), handle, cfg).run(),
        );
        settle().await;
        (bus, lock_arb)
    }

    async fn setup() -> (Arc<MockBus>, Arc<DoorLockArbiter>) {
        setup_with(peps_cfg()).await
    }

    fn place(bus: &MockBus, slot: u8, zone: Zone) {
        let path = match slot {
            1 => "Body.PEPS.Plant.KeyFob.1.PlacedZone",
            2 => "Body.PEPS.Plant.KeyFob.2.PlacedZone",
            3 => "Body.PEPS.Plant.KeyFob.3.PlacedZone",
            _ => panic!("unknown slot"),
        };
        bus.inject(path, SignalValue::String(zone.as_str().into()));
        // The arbiter's `Authenticated` path filters out unpaired fobs.
        let paired_path = match slot {
            1 => "Body.PEPS.Plant.KeyFob.1.Paired",
            2 => "Body.PEPS.Plant.KeyFob.2.Paired",
            3 => "Body.PEPS.Plant.KeyFob.3.Paired",
            _ => unreachable!(),
        };
        bus.inject(paired_path, SignalValue::Bool(true));
    }

    /// Publish a complete lock event (status + requestor + EventNum)
    /// in the same order the production door-lock arbiter does.
    fn fire_lock_event(bus: &MockBus, status: &str, requestor: &str, event_num: u16) {
        bus.inject(LOCK_STATUS, SignalValue::String(status.into()));
        bus.inject(LAST_REQUESTOR, SignalValue::String(requestor.into()));
        bus.inject(LOCK_EVENT_NUM, SignalValue::Uint16(event_num));
    }

    fn unlock_all_dispatched(bus: &MockBus) -> bool {
        bus.history().iter().any(|(sig, val)| {
            *sig == CENTRAL_LOCK_CMD
                && *val == SignalValue::String("unlock_all".into())
        })
    }

    #[tokio::test]
    async fn external_lock_with_key_in_cabin_and_nothing_outside_unlocks() {
        let (bus, _arb) = setup().await;
        place(&bus, 1, Zone::Cabin);
        bus.inject(IGNITION_STATE, SignalValue::String("OFF".into()));
        settle().await;

        fire_lock_event(&bus, "LOCKED", "KeyfobRke", 1);
        wait_for_scan().await;

        assert!(
            unlock_all_dispatched(&bus),
            "expected SmartUnlock to dispatch UnlockAll; history: {:?}",
            bus.history()
        );
    }

    #[tokio::test]
    async fn external_lock_with_key_in_cabin_but_also_outside_stays_locked() {
        let (bus, _arb) = setup().await;
        place(&bus, 1, Zone::Cabin);
        // Driver is outside holding their own paired fob — the
        // "lock and walk away" gesture must be respected.
        place(&bus, 2, Zone::Approach);
        bus.inject(IGNITION_STATE, SignalValue::String("OFF".into()));
        settle().await;

        fire_lock_event(&bus, "LOCKED", "KeyfobPeps", 1);
        wait_for_scan().await;

        assert!(
            !unlock_all_dispatched(&bus),
            "must not unlock when a paired key is reachable from outside"
        );
    }

    #[tokio::test]
    async fn interior_trim_lock_is_ignored() {
        let (bus, _arb) = setup().await;
        place(&bus, 1, Zone::Cabin);
        bus.inject(IGNITION_STATE, SignalValue::String("OFF".into()));
        settle().await;

        // DoorTrimButton is interior — an occupant chose to lock the
        // cabin from inside.  Smart Unlock must not override that.
        fire_lock_event(&bus, "LOCKED", "DoorTrimButton", 1);
        wait_for_scan().await;

        assert!(!unlock_all_dispatched(&bus));
    }

    #[tokio::test]
    async fn autorelock_is_ignored() {
        let (bus, _arb) = setup().await;
        place(&bus, 1, Zone::Cabin);
        bus.inject(IGNITION_STATE, SignalValue::String("OFF".into()));
        settle().await;

        // AutoRelock's whole point is to re-secure a vehicle the user
        // walked away from.  Cancelling it here would defeat the
        // feature.
        fire_lock_event(&bus, "LOCKED", "AutoRelock", 1);
        wait_for_scan().await;

        assert!(!unlock_all_dispatched(&bus));
    }

    #[tokio::test]
    async fn ignition_live_blocks_unlock() {
        let (bus, _arb) = setup().await;
        place(&bus, 1, Zone::Cabin);
        bus.inject(IGNITION_STATE, SignalValue::String("ON".into()));
        settle().await;

        fire_lock_event(&bus, "LOCKED", "KeyfobRke", 1);
        wait_for_scan().await;

        assert!(
            !unlock_all_dispatched(&bus),
            "must not act while ignition is live"
        );
    }

    #[tokio::test]
    async fn no_key_in_cabin_no_unlock() {
        let (bus, _arb) = setup().await;
        bus.inject(IGNITION_STATE, SignalValue::String("OFF".into()));
        settle().await;

        fire_lock_event(&bus, "LOCKED", "KeyfobRke", 1);
        wait_for_scan().await;

        assert!(!unlock_all_dispatched(&bus));
    }

    #[tokio::test]
    async fn double_locked_also_triggers() {
        let (bus, _arb) = setup().await;
        place(&bus, 1, Zone::Cabin);
        bus.inject(IGNITION_STATE, SignalValue::String("OFF".into()));
        settle().await;

        fire_lock_event(&bus, "DOUBLE_LOCKED", "PhoneApp", 1);
        wait_for_scan().await;

        assert!(unlock_all_dispatched(&bus));
    }

    #[tokio::test]
    async fn key_in_trunk_inside_alone_does_not_unlock() {
        let (bus, _arb) = setup().await;
        // TrunkInside is the "key locked in trunk" scenario — distinct
        // remediation (boot pop), not Smart Unlock's job.
        place(&bus, 1, Zone::TrunkInside);
        bus.inject(IGNITION_STATE, SignalValue::String("OFF".into()));
        settle().await;

        fire_lock_event(&bus, "LOCKED", "KeyfobRke", 1);
        wait_for_scan().await;

        assert!(!unlock_all_dispatched(&bus));
    }

    #[tokio::test]
    async fn cylinder_vehicle_feature_is_a_noop() {
        let (bus, _arb) = setup_with(cylinder_cfg()).await;
        place(&bus, 1, Zone::Cabin);
        bus.inject(IGNITION_STATE, SignalValue::String("OFF".into()));
        settle().await;

        fire_lock_event(&bus, "LOCKED", "KeyfobRke", 1);
        wait_for_scan().await;

        assert!(!unlock_all_dispatched(&bus));
    }
}
