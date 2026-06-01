//! KeySearch arbiter — serialiser of LF airtime for PEPS searches.
//!
//! See `docs/key-search-arbiter-and-ignition.md` for the full design.
//!
//! # What this module does
//!
//! - Publishes `AntennaSet`, `SearchMode`, `Coalescing` enums.
//! - Publishes `KeySearchRequest` / `KeySearchResult` / `KeyFinding`
//!   types.
//! - Exposes `KeySearchArbiterHandle` for features to submit search
//!   requests; one tokio task processes them serially.
//! - Simulates per-antenna-set LF latency.
//! - Holds a 50 ms coalescing window for repeat `(antennas, mode)`
//!   requests flagged `Coalescing::Allowed`.
//!
//! # What this module deliberately does NOT do
//!
//! The arbiter is **purely a request serialiser**.  It does **not**
//! run periodic scans, hold timers, or react to ignition state.
//! Every feature that needs PEPS / phone presence triggers its own
//! scan tied to a specific physical event it owns (a button, an
//! edge, a periodic check).  See `features::welcome` for the
//! `AllApproach / Presence` approach poll; `features::key_lost_warning`
//! for the cabin scan; `features::smart_trunk_pop` for the
//! `TrunkInside` scan; etc.  This shape was settled in backlog item
//! #24 — see the [backlog plan](../../../../docs/post-peps-backlog.md).
//!
//! # How the arbiter learns fob positions
//!
//! Subscribes to per-fob `PlacedZone` + `Paired` signals from the
//! PEPS plant and maintains internal `HashMap<KeySlot, _>` caches.
//! Each submitted scan reads these caches synchronously and runs
//! the simulated antenna firing against them.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use futures::StreamExt;
use tokio::select;
use tokio::sync::{mpsc, oneshot};
use tokio::time::{sleep, Instant};

use crate::ipc_message::SignalValue;
use crate::plant_models::peps::zone::Zone;
use crate::signal_bus::{SignalBus, VssPath};

// ── Public types ──────────────────────────────────────────────────────────

/// Identifier for a paired key (fob or phone) — matches the slot
/// index used by the PEPS plant (`Body.PEPS.Plant.KeyFob.{N}.*`).
pub type KeySlot = u8;

/// Physical antenna group fired by the LF subsystem.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AntennaSet {
    /// Cabin antennas — covers fobs in `Zone::Cabin`.
    Cabin,
    /// Registry / cylinder antenna — covers fobs in the
    /// `Zone::KeyCylinder` zone introduced in phase 7.  Until then
    /// this set covers nothing and always returns empty.
    Cylinder,
    /// All corner / handle / hood / trunk-outside antennas —
    /// approach + each proximity zone.
    AllApproach,
    /// One specific handle's antenna.
    SingleHandle(DoorRef),
    /// Trunk-outside (rear bumper / liftgate) antenna.
    TrunkOutside,
    /// Cargo-area antenna inside the trunk.
    TrunkInside,
    /// Chain of scans.  Latencies sum; results accumulate.
    Sequence(Vec<(AntennaSet, SearchMode)>),
}

/// Identifies one physical door for `SingleHandle(door)` searches.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DoorRef {
    pub row: u8, // 1 or 2
    pub side: Side,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Side {
    Left,
    Right,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SearchMode {
    /// LF "ping" — fob ack only, no HMAC.  Cheap (~50 ms) and used
    /// for approach polling.
    Presence,
    /// Full LF challenge → RF response → HMAC verify.  Slower
    /// (100–150 ms) and used wherever the vehicle has to act on a
    /// fob being near (unlock, start, trunk pop).
    Authenticated,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Coalescing {
    /// Re-use any just-completed result for the same `(antennas,
    /// mode)` within `COALESCE_WINDOW`.  Cheaper but stale.
    Allowed,
    /// Always run a fresh scan.  Used for security-critical paths
    /// (handle pull, start press, smart unlock) where a stale
    /// result is unsafe.
    Disallowed,
}

pub struct KeySearchRequest {
    pub requester: &'static str,
    pub antennas: AntennaSet,
    pub mode: SearchMode,
    pub coalescing: Coalescing,
    pub response: oneshot::Sender<KeySearchResult>,
}

#[derive(Debug, Clone)]
pub struct KeySearchResult {
    pub keys_found: Vec<KeyFinding>,
    pub antennas_fired: AntennaSet,
    pub mode: SearchMode,
    pub took: Duration,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KeyFinding {
    pub slot: KeySlot,
    pub zone: Zone,
    pub rssi: i8,
}

// ── Constants ────────────────────────────────────────────────────────────

/// Coalescing window — repeat requests within this period after a
/// completed search get the cached result rather than a fresh scan.
pub const COALESCE_WINDOW: Duration = Duration::from_millis(50);

/// Per-fob slot count.  Mirrors the PEPS plant configuration —
/// 6 slots cover 4 key fobs + 2 phones.
pub const NUM_KEY_SLOTS: usize = 6;

// Note: the periodic AllApproach / Presence poll, its adaptive
// cadence constants, and the `ApproachState` / `ApproachKeys` /
// `ApproachPollInterval` publishes moved into `features::welcome`
// as part of backlog item #24 — Welcome is the only consumer of
// the poll's result, and the "every key search is feature-driven"
// principle (KeyLostWarning #17, SmartTrunkPop #23) makes the
// arbiter purely a request serialiser.

/// Simulated latency per antenna set + mode.
const fn latency(antennas: &AntennaSet, mode: SearchMode) -> Duration {
    use AntennaSet::*;
    match (antennas, mode) {
        (AllApproach, SearchMode::Presence) => Duration::from_millis(50),
        (AllApproach, SearchMode::Authenticated) => Duration::from_millis(150),
        (Cylinder, SearchMode::Authenticated) => Duration::from_millis(50),
        (Cabin, SearchMode::Authenticated)
        | (SingleHandle(_), SearchMode::Authenticated)
        | (TrunkOutside, SearchMode::Authenticated)
        | (TrunkInside, SearchMode::Authenticated) => Duration::from_millis(100),
        // Sequence is summed at runtime; this is per-leg fallback.
        _ => Duration::from_millis(50),
    }
}

// ── Arbiter ──────────────────────────────────────────────────────────────

/// Public handle features submit requests against.  Cheap to clone.
#[derive(Clone)]
pub struct KeySearchArbiterHandle {
    tx: mpsc::Sender<KeySearchRequest>,
}

impl KeySearchArbiterHandle {
    /// Submit a search request.  Returns the result when the
    /// arbiter has run (or coalesced) the scan.  Drop the future to
    /// cancel the receive end; the arbiter still completes the scan.
    pub async fn submit(
        &self,
        requester: &'static str,
        antennas: AntennaSet,
        mode: SearchMode,
        coalescing: Coalescing,
    ) -> Option<KeySearchResult> {
        let (tx, rx) = oneshot::channel();
        let req = KeySearchRequest {
            requester,
            antennas,
            mode,
            coalescing,
            response: tx,
        };
        self.tx.send(req).await.ok()?;
        rx.await.ok()
    }
}

pub struct KeySearchArbiter<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> KeySearchArbiter<B> {
    pub fn new(bus: Arc<B>) -> (Self, KeySearchArbiterHandle) {
        let (tx, _rx) = mpsc::channel::<KeySearchRequest>(64);
        let handle = KeySearchArbiterHandle { tx };
        (Self { bus }, handle)
    }

    /// Bundled constructor that also returns the request receiver
    /// the run loop consumes — keeps the wiring local in `main.rs`.
    pub fn new_with_rx(
        bus: Arc<B>,
    ) -> (
        Self,
        KeySearchArbiterHandle,
        mpsc::Receiver<KeySearchRequest>,
    ) {
        let (tx, rx) = mpsc::channel::<KeySearchRequest>(64);
        let handle = KeySearchArbiterHandle { tx };
        (Self { bus }, handle, rx)
    }

    // Note: `with_cadence` removed; approach-poll cadence is now a
    // Welcome concern.  Callers that need to tune the cadence for
    // tests should construct Welcome with `Welcome::with_cadence`
    // instead.

    /// Run loop.  Consumes `self` and the request receiver returned
    /// from `new_with_rx`.  The arbiter is purely a request
    /// serialiser: it processes feature-submitted searches and
    /// caches per-fob zone / pairing state from PEPS-plant signals.
    /// No time-driven scans of its own — see backlog #24.
    pub async fn run(self, mut rx: mpsc::Receiver<KeySearchRequest>) {
        tracing::info!("KeySearchArbiter started");

        // Per-fob position cache, updated from continuous Zone signals.
        let zones: Arc<tokio::sync::Mutex<HashMap<KeySlot, Zone>>> =
            Arc::new(tokio::sync::Mutex::new(HashMap::new()));
        // Per-fob pairing cache.  `SearchMode::Authenticated` filters
        // out unpaired fobs — a stranger's fob (or a mechanically-
        // compatible blank cut to fit the cylinder, like Fob 5 in the
        // simulator) ack's the LF ping at the physical level but
        // fails the HMAC challenge, so it must not pass auth scans.
        // `SearchMode::Presence` does not filter — physical RF
        // detection is independent of cryptographic pairing.
        let paired: Arc<tokio::sync::Mutex<HashMap<KeySlot, bool>>> =
            Arc::new(tokio::sync::Mutex::new(HashMap::new()));
        // Per-fob `LastObservedZone` map — updated whenever a scan
        // touches a fob.  Used to detect "fob has left coverage"
        // transitions: if a fob's previous LastObserved was inside
        // the current scan's coverage and the scan didn't see it,
        // publish OutOfRange.
        let last_observed: Arc<tokio::sync::Mutex<HashMap<KeySlot, Zone>>> =
            Arc::new(tokio::sync::Mutex::new(HashMap::new()));

        // Subscribe to every fob's PlacedZone + Paired signals once.
        // The arbiter simulates the LF antenna subsystem; PlacedZone
        // is the HMI-set ground truth it samples from each scan.
        for slot in 0..NUM_KEY_SLOTS as KeySlot {
            let zones_clone = Arc::clone(&zones);
            let mut rx_placed = self.bus.subscribe(fob_placed_zone_signal(slot)).await;
            tokio::spawn(async move {
                while let Some(v) = rx_placed.next().await {
                    if let SignalValue::String(s) = v {
                        if let Some(z) = Zone::from_str_value(&s) {
                            zones_clone.lock().await.insert(slot, z);
                        }
                    }
                }
            });
            let pair_path = fob_paired_signal(slot);
            let mut rx_pair = self.bus.subscribe(pair_path).await;
            let paired_clone = Arc::clone(&paired);
            tokio::spawn(async move {
                while let Some(v) = rx_pair.next().await {
                    if let SignalValue::Bool(b) = v {
                        paired_clone.lock().await.insert(slot, b);
                    }
                }
            });
        }

        // Recent-result cache for coalescing window.
        let mut cache: Vec<(AntennaSet, SearchMode, KeySearchResult, Instant)> = Vec::new();

        // Seed `LastObservedZone = OutOfRange` for every slot so HMI
        // snapshots see a defined value before the first scan runs.
        for slot in 0..NUM_KEY_SLOTS as KeySlot {
            let _ = self
                .bus
                .publish(
                    fob_last_observed_zone_signal(slot),
                    SignalValue::String(Zone::OutOfRange.to_string()),
                )
                .await;
        }

        loop {
            select! {
                // Feature-submitted search request.  Welcome's
                // periodic AllApproach poll is one such submitter
                // now, indistinguishable from PassiveEntry's
                // handle-pull scan or VehicleStartingControl's
                // brake pre-auth at this layer.
                Some(req) = rx.recv() => {
                    handle_request(req, &self.bus, &zones, &paired, &last_observed, &mut cache).await;
                }

                else => break,
            }
        }

        tracing::warn!("KeySearchArbiter: request channel closed, exiting");
    }
}

/// Process a single submitted search request — pure function over
/// the zones cache and the coalescing window.  Extracted so the
/// `select!` body stays readable.
#[allow(clippy::too_many_arguments)]
async fn handle_request<B: SignalBus + Send + Sync + 'static>(
    req: KeySearchRequest,
    bus: &Arc<B>,
    zones: &Arc<tokio::sync::Mutex<HashMap<KeySlot, Zone>>>,
    paired: &Arc<tokio::sync::Mutex<HashMap<KeySlot, bool>>>,
    last_observed: &Arc<tokio::sync::Mutex<HashMap<KeySlot, Zone>>>,
    cache: &mut Vec<(AntennaSet, SearchMode, KeySearchResult, Instant)>,
) {
    // Drop cache entries older than the coalesce window.
    let now = Instant::now();
    cache.retain(|(_, _, _, t)| now.duration_since(*t) <= COALESCE_WINDOW);

    if req.coalescing == Coalescing::Allowed {
        if let Some((_, _, hit, _)) = cache
            .iter()
            .find(|(a, m, _, _)| *a == req.antennas && *m == req.mode)
        {
            tracing::debug!(requester = req.requester, "KeySearchArbiter: coalesced");
            let _ = req.response.send(hit.clone());
            return;
        }
    }

    let started = Instant::now();
    let zones_snapshot = zones.lock().await.clone();
    let paired_snapshot = paired.lock().await.clone();
    let result = run_scan(&req.antennas, req.mode, &zones_snapshot, &paired_snapshot).await;
    let result = KeySearchResult {
        took: started.elapsed(),
        ..result
    };

    tracing::debug!(
        requester = req.requester,
        antennas = ?req.antennas,
        mode = ?req.mode,
        keys_found = result.keys_found.len(),
        took_ms = result.took.as_millis() as u64,
        "KeySearchArbiter: scan complete"
    );

    // Publish per-fob LastObservedZone updates from this scan.
    publish_last_observed(bus, last_observed, &req.antennas, &result).await;

    cache.push((
        req.antennas.clone(),
        req.mode,
        result.clone(),
        Instant::now(),
    ));
    let _ = req.response.send(result);
}

/// Update the `LastObservedZone` signal for each fob touched by the
/// scan, and for any fob whose previous last-observed zone was within
/// the scan's coverage but isn't in the current result (meaning it
/// has left that coverage area).
///
/// This is the per-device counterpart to the aggregate `ApproachKeys`
/// / `ApproachState` signals — features that need per-key positions
/// should subscribe to `Body.PEPS.Plant.{...}.LastObservedZone`
/// rather than the legacy `.Zone` (which is currently a mirror of
/// the HMI-set ground truth).
async fn publish_last_observed<B: SignalBus + Send + Sync + 'static>(
    bus: &Arc<B>,
    last_observed: &Arc<tokio::sync::Mutex<HashMap<KeySlot, Zone>>>,
    antennas: &AntennaSet,
    result: &KeySearchResult,
) {
    // For Sequence scans the result already concatenates all legs;
    // the coverage union is the union of each leg.  Zone is Hash so
    // we dedup via a HashSet rather than sort+dedup (Zone doesn't
    // implement Ord and there's no compelling reason to add it).
    let coverage: std::collections::HashSet<Zone> = match antennas {
        AntennaSet::Sequence(legs) => legs.iter().flat_map(|(a, _)| coverage_zones(a)).collect(),
        other => coverage_zones(other).into_iter().collect(),
    };

    let mut lo = last_observed.lock().await;

    // Slots found in this scan → publish their observed zone.
    let mut found_slots: Vec<KeySlot> = Vec::with_capacity(result.keys_found.len());
    for finding in &result.keys_found {
        found_slots.push(finding.slot);
        let prev = lo.get(&finding.slot).copied();
        if prev != Some(finding.zone) {
            lo.insert(finding.slot, finding.zone);
            let _ = bus
                .publish(
                    fob_last_observed_zone_signal(finding.slot),
                    SignalValue::String(finding.zone.to_string()),
                )
                .await;
        }
    }

    // Slots whose previous LastObserved was in this scan's coverage
    // but who weren't found this time → they've left the coverage
    // area.  Publish OutOfRange to give consumers the "fob gone"
    // event.  Slots that were previously seen outside this scan's
    // coverage are unaffected (the scan can't speak to them).
    let to_clear: Vec<KeySlot> = lo
        .iter()
        .filter_map(|(slot, prev_zone)| {
            if found_slots.contains(slot) {
                return None;
            }
            if !coverage.contains(prev_zone) {
                return None;
            }
            if *prev_zone == Zone::OutOfRange {
                return None;
            }
            Some(*slot)
        })
        .collect();
    for slot in to_clear {
        lo.insert(slot, Zone::OutOfRange);
        let _ = bus
            .publish(
                fob_last_observed_zone_signal(slot),
                SignalValue::String(Zone::OutOfRange.to_string()),
            )
            .await;
    }
}

// ── Internal scan execution ───────────────────────────────────────────────

/// Returns the list of zones an `AntennaSet` covers.  `Sequence` is
/// handled by the caller (runs each leg separately).
fn coverage_zones(antennas: &AntennaSet) -> Vec<Zone> {
    match antennas {
        AntennaSet::Cabin => vec![Zone::Cabin],
        // Phase 7 — cylinder antenna covers the short-range
        // `Zone::KeyCylinder` introduced alongside the KeySource cal.
        AntennaSet::Cylinder => vec![Zone::KeyCylinder],
        AntennaSet::AllApproach => vec![
            Zone::Approach,
            Zone::LeftFront,
            Zone::RightFront,
            Zone::Hood,
            Zone::Trunk,
        ],
        AntennaSet::SingleHandle(door) => vec![match (door.row, door.side) {
            (1, Side::Left) => Zone::LeftFront,
            (1, Side::Right) => Zone::RightFront,
            // Row2 doors don't have their own handle antenna in the
            // current model (only Row1 + hood + trunk).  Return empty.
            _ => return vec![],
        }],
        AntennaSet::TrunkOutside => vec![Zone::Trunk],
        AntennaSet::TrunkInside => vec![Zone::TrunkInside],
        AntennaSet::Sequence(_) => vec![],
    }
}

/// Run a single scan (or a Sequence) and return the result.
fn run_scan<'a>(
    antennas: &'a AntennaSet,
    mode: SearchMode,
    zones_snapshot: &'a HashMap<KeySlot, Zone>,
    paired_snapshot: &'a HashMap<KeySlot, bool>,
) -> std::pin::Pin<Box<dyn std::future::Future<Output = KeySearchResult> + Send + 'a>> {
    Box::pin(async move {
        if let AntennaSet::Sequence(legs) = antennas {
            let mut combined: Vec<KeyFinding> = Vec::new();
            for (leg, leg_mode) in legs {
                let leg_result = run_scan(leg, *leg_mode, zones_snapshot, paired_snapshot).await;
                combined.extend(leg_result.keys_found);
            }
            return KeySearchResult {
                keys_found: combined,
                antennas_fired: antennas.clone(),
                mode,
                took: Duration::ZERO, // overwritten by caller
            };
        }

        // Sleep the simulated airtime — features experience real
        // asynchrony so latency-sensitive code can be tested honestly.
        sleep(latency(antennas, mode)).await;

        let coverage = coverage_zones(antennas);
        let mut found: Vec<KeyFinding> = Vec::new();
        for (slot, zone) in zones_snapshot.iter() {
            if !coverage.contains(zone) {
                continue;
            }
            // Authenticated scans require the fob to be paired —
            // unpaired fobs ack at the physical LF layer but fail
            // the HMAC challenge.  Default `true` if we haven't
            // received the Paired signal yet (avoids dropping fobs
            // before the boot snapshot lands).
            if mode == SearchMode::Authenticated {
                let is_paired = paired_snapshot.get(slot).copied().unwrap_or(true);
                if !is_paired {
                    continue;
                }
            }
            found.push(KeyFinding {
                slot: *slot,
                zone: *zone,
                rssi: rssi_for_zone(*zone),
            });
        }

        KeySearchResult {
            keys_found: found,
            antennas_fired: antennas.clone(),
            mode,
            took: Duration::ZERO,
        }
    })
}

fn rssi_for_zone(z: Zone) -> i8 {
    // Strongest reading in close-proximity zones, weakest at Approach.
    match z {
        Zone::Cabin | Zone::TrunkInside => -45,
        Zone::LeftFront | Zone::RightFront | Zone::Hood | Zone::Trunk => -55,
        Zone::Approach => -75,
        _ => -127,
    }
}

/// Where to read each fob's HMI-set physical position from.
///
/// The arbiter is the one consumer that legitimately needs ground-
/// truth position — it's *simulating* the LF antenna subsystem.
/// Every other consumer should subscribe to LastObservedZone
/// (published below from inside scan results) so it experiences the
/// same partial-information world a real PEPS feature does.
fn fob_placed_zone_signal(slot: KeySlot) -> VssPath {
    match slot {
        0 => "Vehicle.Simulation.KeyFob.1.PlacedZone",
        1 => "Vehicle.Simulation.KeyFob.2.PlacedZone",
        2 => "Vehicle.Simulation.KeyFob.3.PlacedZone",
        3 => "Vehicle.Simulation.KeyFob.4.PlacedZone",
        4 => "Vehicle.Simulation.BlePhone.1.PlacedZone",
        5 => "Vehicle.Simulation.BlePhone.2.PlacedZone",
        _ => "Vehicle.Simulation.KeyFob.1.PlacedZone", // defensive; never hit at runtime
    }
}

/// Output signal — what the arbiter saw on its most recent scan
/// that covered this fob.  This is what features should subscribe
/// to instead of the legacy `.Zone` mirror.
fn fob_last_observed_zone_signal(slot: KeySlot) -> VssPath {
    match slot {
        0 => "Vehicle.Simulation.KeyFob.1.LastObservedZone",
        1 => "Vehicle.Simulation.KeyFob.2.LastObservedZone",
        2 => "Vehicle.Simulation.KeyFob.3.LastObservedZone",
        3 => "Vehicle.Simulation.KeyFob.4.LastObservedZone",
        4 => "Vehicle.Simulation.BlePhone.1.LastObservedZone",
        5 => "Vehicle.Simulation.BlePhone.2.LastObservedZone",
        _ => "Vehicle.Simulation.KeyFob.1.LastObservedZone",
    }
}

/// Paired-flag signal path for the same slot indexing as
/// `fob_zone_signal`.  BlePhones don't publish a `.Paired` signal
/// in the current model (they're "always paired" once provisioned),
/// so we fall back to the always-`true` KeyFob.1 path — the run
/// loop only ever reads `SignalValue::Bool` here, and a steady
/// `true` keeps phones in the authenticated cohort by default.
fn fob_paired_signal(slot: KeySlot) -> VssPath {
    match slot {
        0 => "Vehicle.Simulation.KeyFob.1.Paired",
        1 => "Vehicle.Simulation.KeyFob.2.Paired",
        2 => "Vehicle.Simulation.KeyFob.3.Paired",
        3 => "Vehicle.Simulation.KeyFob.4.Paired",
        // Phones: no .Paired signal in the simulator today — point at
        // a fob path that we never write `false` to so the subscription
        // is harmless.  Filter defaults to "paired" when no value
        // arrives, so phones remain eligible for Authenticated scans.
        4 => "Vehicle.Simulation.KeyFob.1.Paired",
        5 => "Vehicle.Simulation.KeyFob.1.Paired",
        _ => "Vehicle.Simulation.KeyFob.1.Paired",
    }
}

// ── Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;

    async fn settle() {
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        sleep(Duration::from_millis(2)).await;
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    async fn setup() -> (Arc<MockBus>, KeySearchArbiterHandle) {
        let bus = Arc::new(MockBus::new());
        let (arb, handle, rx) = KeySearchArbiter::new_with_rx(Arc::clone(&bus));
        tokio::spawn(arb.run(rx));
        settle().await;
        (bus, handle)
    }

    fn place(bus: &MockBus, slot: KeySlot, zone: Zone) {
        bus.inject(
            fob_placed_zone_signal(slot),
            SignalValue::String(zone.as_str().into()),
        );
    }

    #[tokio::test]
    async fn empty_when_no_fobs() {
        let (_bus, h) = setup().await;
        let r = h
            .submit(
                "test",
                AntennaSet::AllApproach,
                SearchMode::Presence,
                Coalescing::Disallowed,
            )
            .await
            .expect("response");
        assert!(r.keys_found.is_empty());
    }

    #[tokio::test]
    async fn presence_finds_fob_in_approach() {
        let (bus, h) = setup().await;
        place(&bus, 0, Zone::Approach);
        settle().await;
        let r = h
            .submit(
                "test",
                AntennaSet::AllApproach,
                SearchMode::Presence,
                Coalescing::Disallowed,
            )
            .await
            .unwrap();
        assert_eq!(r.keys_found.len(), 1);
        assert_eq!(r.keys_found[0].slot, 0);
        assert_eq!(r.keys_found[0].zone, Zone::Approach);
    }

    #[tokio::test]
    async fn cabin_search_only_returns_cabin_fobs() {
        let (bus, h) = setup().await;
        place(&bus, 0, Zone::Approach);
        place(&bus, 1, Zone::Cabin);
        place(&bus, 2, Zone::Trunk);
        settle().await;
        let r = h
            .submit(
                "test",
                AntennaSet::Cabin,
                SearchMode::Authenticated,
                Coalescing::Disallowed,
            )
            .await
            .unwrap();
        assert_eq!(r.keys_found.len(), 1);
        assert_eq!(r.keys_found[0].slot, 1);
    }

    #[tokio::test]
    async fn single_handle_returns_only_that_door() {
        let (bus, h) = setup().await;
        place(&bus, 0, Zone::LeftFront);
        place(&bus, 1, Zone::RightFront);
        settle().await;
        let r = h
            .submit(
                "test",
                AntennaSet::SingleHandle(DoorRef {
                    row: 1,
                    side: Side::Left,
                }),
                SearchMode::Authenticated,
                Coalescing::Disallowed,
            )
            .await
            .unwrap();
        assert_eq!(r.keys_found.len(), 1);
        assert_eq!(r.keys_found[0].slot, 0);
    }

    #[tokio::test]
    async fn trunk_inside_vs_outside() {
        let (bus, h) = setup().await;
        place(&bus, 0, Zone::TrunkInside);
        place(&bus, 1, Zone::Trunk);
        settle().await;
        let inside = h
            .submit(
                "test",
                AntennaSet::TrunkInside,
                SearchMode::Authenticated,
                Coalescing::Disallowed,
            )
            .await
            .unwrap();
        let outside = h
            .submit(
                "test",
                AntennaSet::TrunkOutside,
                SearchMode::Authenticated,
                Coalescing::Disallowed,
            )
            .await
            .unwrap();
        assert_eq!(inside.keys_found.len(), 1);
        assert_eq!(inside.keys_found[0].slot, 0);
        assert_eq!(outside.keys_found.len(), 1);
        assert_eq!(outside.keys_found[0].slot, 1);
    }

    #[tokio::test]
    async fn sequence_runs_all_legs_and_combines() {
        let (bus, h) = setup().await;
        place(&bus, 0, Zone::TrunkInside);
        place(&bus, 1, Zone::Trunk);
        settle().await;
        let r = h
            .submit(
                "test",
                AntennaSet::Sequence(vec![
                    (AntennaSet::TrunkInside, SearchMode::Authenticated),
                    (AntennaSet::TrunkOutside, SearchMode::Authenticated),
                ]),
                SearchMode::Authenticated,
                Coalescing::Disallowed,
            )
            .await
            .unwrap();
        assert_eq!(r.keys_found.len(), 2);
        let slots: Vec<_> = r.keys_found.iter().map(|k| k.slot).collect();
        assert!(slots.contains(&0));
        assert!(slots.contains(&1));
    }

    #[tokio::test]
    async fn presence_latency_is_50ms_for_approach() {
        let (_bus, h) = setup().await;
        let start = Instant::now();
        let _ = h
            .submit(
                "test",
                AntennaSet::AllApproach,
                SearchMode::Presence,
                Coalescing::Disallowed,
            )
            .await
            .unwrap();
        let elapsed = start.elapsed();
        // 50 ms ± slack for scheduler jitter
        assert!(
            elapsed >= Duration::from_millis(45),
            "presence too fast: {elapsed:?}"
        );
        assert!(
            elapsed < Duration::from_millis(200),
            "presence too slow: {elapsed:?}"
        );
    }

    #[tokio::test]
    async fn authenticated_handle_latency_is_100ms() {
        let (_bus, h) = setup().await;
        let start = Instant::now();
        let _ = h
            .submit(
                "test",
                AntennaSet::SingleHandle(DoorRef {
                    row: 1,
                    side: Side::Left,
                }),
                SearchMode::Authenticated,
                Coalescing::Disallowed,
            )
            .await
            .unwrap();
        let elapsed = start.elapsed();
        assert!(
            elapsed >= Duration::from_millis(95),
            "auth too fast: {elapsed:?}"
        );
        assert!(
            elapsed < Duration::from_millis(250),
            "auth too slow: {elapsed:?}"
        );
    }

    #[tokio::test]
    async fn coalescing_returns_cached_result_within_window() {
        let (bus, h) = setup().await;
        place(&bus, 0, Zone::Approach);
        settle().await;
        // Burn the first request.
        let _ = h
            .submit(
                "test",
                AntennaSet::AllApproach,
                SearchMode::Presence,
                Coalescing::Allowed,
            )
            .await
            .unwrap();
        // Second request immediately afterward must short-circuit.
        let start = Instant::now();
        let _ = h
            .submit(
                "test",
                AntennaSet::AllApproach,
                SearchMode::Presence,
                Coalescing::Allowed,
            )
            .await
            .unwrap();
        let coalesced_elapsed = start.elapsed();
        assert!(
            coalesced_elapsed < Duration::from_millis(20),
            "coalesced should be near-instant: {coalesced_elapsed:?}"
        );
    }

    #[tokio::test]
    async fn coalescing_disallowed_always_runs_fresh() {
        let (bus, h) = setup().await;
        place(&bus, 0, Zone::Approach);
        settle().await;
        let _ = h
            .submit(
                "test",
                AntennaSet::AllApproach,
                SearchMode::Presence,
                Coalescing::Disallowed,
            )
            .await
            .unwrap();
        let start = Instant::now();
        let _ = h
            .submit(
                "test",
                AntennaSet::AllApproach,
                SearchMode::Presence,
                Coalescing::Disallowed,
            )
            .await
            .unwrap();
        let elapsed = start.elapsed();
        // Must have run a real scan, so at least one latency window.
        assert!(
            elapsed >= Duration::from_millis(45),
            "disallowed should not coalesce: {elapsed:?}"
        );
    }

    #[tokio::test]
    async fn requests_serialize() {
        let (bus, h) = setup().await;
        place(&bus, 0, Zone::Approach);
        settle().await;
        let h1 = h.clone();
        let h2 = h.clone();
        // Two simultaneous requests with Disallowed coalescing must
        // be serialized — total time ≥ 2 × latency.
        let start = Instant::now();
        let (_a, _b) = tokio::join!(
            h1.submit(
                "t1",
                AntennaSet::AllApproach,
                SearchMode::Presence,
                Coalescing::Disallowed
            ),
            h2.submit(
                "t2",
                AntennaSet::AllApproach,
                SearchMode::Presence,
                Coalescing::Disallowed
            ),
        );
        let elapsed = start.elapsed();
        assert!(
            elapsed >= Duration::from_millis(95),
            "two scans should serialize, got {elapsed:?}"
        );
    }

    // The approach-poll cadence-flip / ignition-suspension tests
    // moved to `features::welcome` along with the poll itself.
    // See backlog item #24 and PR #50 for the architectural shift
    // (every key search is now feature-driven).
}
