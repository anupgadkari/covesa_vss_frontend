//! KeySearch arbiter — owns LF airtime for PEPS searches.
//!
//! See `docs/key-search-arbiter-and-ignition.md` for the full design.
//! This is **phase 1**: the foundation only.  Subsequent phases add
//! the adaptive approach-poll loop (phase 2), feature migrations
//! (phases 3–6, 8), and downstream consumers.
//!
//! # What this phase delivers
//!
//! - `AntennaSet`, `SearchMode`, `Coalescing` enums.
//! - `KeySearchRequest` / `KeySearchResult` / `KeyFinding` types.
//! - `KeySearchArbiter::handle` for features to submit requests; one
//!   tokio task processes them serially.
//! - Simulated LF latency per antenna set + mode.
//! - 50 ms coalescing window for repeat
//!   `(antennas, mode)` requests flagged `Coalescing::Allowed`.
//!
//! # What is NOT yet in place
//!
//! - Adaptive approach poll loop (phase 2).
//! - Priority queue / preemption between request classes (phase 2 +).
//! - `Body.PEPS.ApproachState` / `ApproachKeys` derived signals
//!   (phase 2).
//! - HMAC challenge integration with `plant_models/peps/crypto.rs`
//!   for `SearchMode::Authenticated` — currently the simulated auth
//!   passes for every fob in coverage.  Wired up in phase 7 when
//!   `VehicleStartingControl` lands.
//!
//! # How the arbiter learns fob positions
//!
//! Subscribes to the existing per-fob Zone signals
//! (`Body.PEPS.Plant.KeyFob.{1..N}.Zone`) and maintains an internal
//! `HashMap<KeySlot, Zone>`.  When a search request arrives, the
//! arbiter consults this cache to determine which fobs are in
//! coverage of the requested antenna set.  In phase 9 the plant
//! splits this into `PlacedZone` (HMI drag, instant) +
//! `LastObservedZone` (per-search feedback); for now we re-use the
//! continuous Zone signal.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use futures::StreamExt;
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
    pub row: u8,  // 1 or 2
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
    ) -> (Self, KeySearchArbiterHandle, mpsc::Receiver<KeySearchRequest>) {
        let (tx, rx) = mpsc::channel::<KeySearchRequest>(64);
        let handle = KeySearchArbiterHandle { tx };
        (Self { bus }, handle, rx)
    }

    /// Run loop.  Consumes `self` and the request receiver returned
    /// from `new_with_rx`.
    pub async fn run(self, mut rx: mpsc::Receiver<KeySearchRequest>) {
        tracing::info!("KeySearchArbiter started");

        // Per-fob position cache, updated from continuous Zone signals.
        let zones: Arc<tokio::sync::Mutex<HashMap<KeySlot, Zone>>> =
            Arc::new(tokio::sync::Mutex::new(HashMap::new()));

        // Subscribe to every fob's Zone signal once.
        for slot in 0..NUM_KEY_SLOTS as KeySlot {
            let path = fob_zone_signal(slot);
            let mut rx_zone = self.bus.subscribe(path).await;
            let zones_clone = Arc::clone(&zones);
            tokio::spawn(async move {
                while let Some(v) = rx_zone.next().await {
                    if let SignalValue::String(s) = v {
                        if let Some(z) = Zone::from_str_value(&s) {
                            zones_clone.lock().await.insert(slot, z);
                        }
                    }
                }
            });
        }

        // Recent-result cache for coalescing window.
        let mut cache: Vec<(AntennaSet, SearchMode, KeySearchResult, Instant)> = Vec::new();

        while let Some(req) = rx.recv().await {
            // Drop cache entries older than the coalesce window.
            let now = Instant::now();
            cache.retain(|(_, _, _, t)| now.duration_since(*t) <= COALESCE_WINDOW);

            // Coalesce if a fresh cached result matches.
            if req.coalescing == Coalescing::Allowed {
                if let Some((_, _, hit, _)) = cache
                    .iter()
                    .find(|(a, m, _, _)| *a == req.antennas && *m == req.mode)
                {
                    tracing::debug!(
                        requester = req.requester,
                        "KeySearchArbiter: coalesced"
                    );
                    let _ = req.response.send(hit.clone());
                    continue;
                }
            }

            // Run the scan.
            let started = Instant::now();
            let zones_snapshot = zones.lock().await.clone();
            let result = run_scan(&req.antennas, req.mode, &zones_snapshot).await;
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

            cache.push((req.antennas.clone(), req.mode, result.clone(), Instant::now()));
            let _ = req.response.send(result);
        }

        tracing::warn!("KeySearchArbiter: request channel closed, exiting");
    }
}

// ── Internal scan execution ───────────────────────────────────────────────

/// Returns the list of zones an `AntennaSet` covers.  `Sequence` is
/// handled by the caller (runs each leg separately).
fn coverage_zones(antennas: &AntennaSet) -> Vec<Zone> {
    match antennas {
        AntennaSet::Cabin => vec![Zone::Cabin],
        AntennaSet::Cylinder => vec![],
        // Phase 7 adds Zone::KeyCylinder; until then Cylinder covers nothing.
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
) -> std::pin::Pin<Box<dyn std::future::Future<Output = KeySearchResult> + Send + 'a>> {
    Box::pin(async move {
        if let AntennaSet::Sequence(legs) = antennas {
            let mut combined: Vec<KeyFinding> = Vec::new();
            for (leg, leg_mode) in legs {
                let leg_result = run_scan(leg, *leg_mode, zones_snapshot).await;
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
            if coverage.contains(zone) {
                // For SearchMode::Presence: every fob in coverage acks.
                // For SearchMode::Authenticated: every fob in coverage
                // also passes (phase 7 will plumb real HMAC verify
                // through this branch using the existing
                // plant_models/peps/crypto module).
                found.push(KeyFinding {
                    slot: *slot,
                    zone: *zone,
                    rssi: rssi_for_zone(*zone),
                });
            }
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

fn fob_zone_signal(slot: KeySlot) -> VssPath {
    // Constant string indices — matches the existing PEPS plant's
    // published per-fob zone signals.  Slot range 0..NUM_KEY_SLOTS.
    match slot {
        0 => "Body.PEPS.Plant.KeyFob.1.Zone",
        1 => "Body.PEPS.Plant.KeyFob.2.Zone",
        2 => "Body.PEPS.Plant.KeyFob.3.Zone",
        3 => "Body.PEPS.Plant.KeyFob.4.Zone",
        4 => "Body.PEPS.Plant.BlePhone.1.Zone",
        5 => "Body.PEPS.Plant.BlePhone.2.Zone",
        _ => "Body.PEPS.Plant.KeyFob.1.Zone", // defensive; never hit at runtime
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
            fob_zone_signal(slot),
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
}
