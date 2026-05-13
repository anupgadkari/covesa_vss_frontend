//! KeySearch arbiter — owns LF airtime for PEPS searches.
//!
//! See `docs/key-search-arbiter-and-ignition.md` for the full design.
//! This module covers **phases 1 + 2**: the foundation plus the
//! adaptive approach-poll loop.  Subsequent phases (3–10) migrate
//! existing features onto the arbiter and add new consumers.
//!
//! # What this module delivers today
//!
//! - `AntennaSet`, `SearchMode`, `Coalescing` enums.
//! - `KeySearchRequest` / `KeySearchResult` / `KeyFinding` types.
//! - `KeySearchArbiter::handle` for features to submit requests; one
//!   tokio task processes them serially.
//! - Simulated LF latency per antenna set + mode.
//! - 50 ms coalescing window for repeat `(antennas, mode)` requests
//!   flagged `Coalescing::Allowed`.
//! - Adaptive approach-poll loop: 700 ms cadence when no key is in
//!   approach, 10 s cadence when one is detected, suspended while
//!   the ignition is in `ACC`/`ON`/`START`.  Publishes
//!   `Body.PEPS.ApproachState` (bool), `Body.PEPS.ApproachKeys`
//!   (count), `Body.PEPS.ApproachPollInterval` (current cadence ms).
//!   Cadences are overridable via `with_cadence` for tests.
//!
//! # What is NOT yet in place
//!
//! - Priority queue / preemption between request classes.
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
use tokio::select;
use tokio::sync::{mpsc, oneshot};
use tokio::time::{sleep, sleep_until, Instant};

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

/// Approach-poll cadence when no key is currently in approach —
/// scan briskly so we detect arrivals quickly.
pub const APPROACH_POLL_FAST: Duration = Duration::from_millis(700);

/// Approach-poll cadence when a key is already in approach —
/// confirm presence less often (saves both vehicle and fob battery).
pub const APPROACH_POLL_SLOW: Duration = Duration::from_secs(10);

/// VSS path the arbiter watches to know whether to suspend the
/// poll (driving — fob is in cabin anyway).
const IGNITION_STATE_SIGNAL: VssPath = "Vehicle.LowVoltageSystemState";

/// VSS paths the arbiter writes from the poll loop.
const APPROACH_STATE_OUT: VssPath = "Body.PEPS.ApproachState";
const APPROACH_KEYS_OUT: VssPath = "Body.PEPS.ApproachKeys";
const APPROACH_POLL_INTERVAL_OUT: VssPath = "Body.PEPS.ApproachPollInterval";

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
    fast_cadence: Duration,
    slow_cadence: Duration,
}

impl<B: SignalBus + Send + Sync + 'static> KeySearchArbiter<B> {
    pub fn new(bus: Arc<B>) -> (Self, KeySearchArbiterHandle) {
        let (tx, _rx) = mpsc::channel::<KeySearchRequest>(64);
        let handle = KeySearchArbiterHandle { tx };
        (
            Self {
                bus,
                fast_cadence: APPROACH_POLL_FAST,
                slow_cadence: APPROACH_POLL_SLOW,
            },
            handle,
        )
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
        (
            Self {
                bus,
                fast_cadence: APPROACH_POLL_FAST,
                slow_cadence: APPROACH_POLL_SLOW,
            },
            handle,
            rx,
        )
    }

    /// Override the adaptive approach-poll cadences.  Production
    /// builds use the public constants (700 ms / 10 s); tests use
    /// much shorter durations so they run in real time without
    /// dragging the test suite.
    pub fn with_cadence(mut self, fast: Duration, slow: Duration) -> Self {
        self.fast_cadence = fast;
        self.slow_cadence = slow;
        self
    }

    /// Run loop.  Consumes `self` and the request receiver returned
    /// from `new_with_rx`.
    ///
    /// Three sources of work:
    ///   1. Feature-submitted search requests (`rx`).
    ///   2. Adaptive approach-poll deadline (cadence flips between
    ///      `APPROACH_POLL_FAST` and `APPROACH_POLL_SLOW` based on
    ///      the most recent poll result).  Suspended while the
    ///      ignition is in `ACC`/`ON`/`START`.
    ///   3. `Vehicle.LowVoltageSystemState` updates — drives the
    ///      suspension flag.
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

        // Approach-poll state.
        let mut approach_state: bool = false;
        let mut approach_keys: u8 = 0;
        let mut ign_suspended: bool = false;
        let mut poll_deadline: Instant = Instant::now() + self.fast_cadence;

        // Seed the derived signals so HMI snapshots see defined values.
        let _ = self
            .bus
            .publish(APPROACH_STATE_OUT, SignalValue::Bool(false))
            .await;
        let _ = self
            .bus
            .publish(APPROACH_KEYS_OUT, SignalValue::Uint8(0))
            .await;
        let _ = self
            .bus
            .publish(
                APPROACH_POLL_INTERVAL_OUT,
                SignalValue::Uint16(self.fast_cadence.as_millis() as u16),
            )
            .await;

        // Subscribe to ignition state for poll suspension.
        let mut ign_rx = self.bus.subscribe(IGNITION_STATE_SIGNAL).await;

        loop {
            // While suspended, push the deadline far out so the
            // sleep branch never wins; only `rx` and ignition events
            // wake the loop.
            let next_deadline = if ign_suspended {
                Instant::now() + Duration::from_secs(3600)
            } else {
                poll_deadline
            };

            select! {
                biased;

                // Ignition state changes — suspend / resume the poll.
                Some(val) = ign_rx.next() => {
                    if let SignalValue::String(s) = val {
                        let now_susp = matches!(s.as_str(), "ACC" | "ON" | "START");
                        if now_susp != ign_suspended {
                            ign_suspended = now_susp;
                            tracing::info!(suspended = ign_suspended,
                                "KeySearchArbiter: approach poll suspension changed");
                            if ign_suspended {
                                // Going suspended — clear state and publish.
                                approach_state = false;
                                approach_keys = 0;
                                let _ = self.bus.publish(APPROACH_STATE_OUT, SignalValue::Bool(false)).await;
                                let _ = self.bus.publish(APPROACH_KEYS_OUT, SignalValue::Uint8(0)).await;
                                let _ = self.bus.publish(APPROACH_POLL_INTERVAL_OUT, SignalValue::Uint16(0)).await;
                            } else {
                                // Resuming — kick an immediate poll.
                                poll_deadline = Instant::now();
                            }
                        }
                    }
                }

                // Feature-submitted search request.
                Some(req) = rx.recv() => {
                    handle_request(req, &zones, &mut cache).await;
                }

                // Periodic approach poll.
                _ = sleep_until(next_deadline), if !ign_suspended => {
                    let zones_snapshot = zones.lock().await.clone();
                    let started = Instant::now();
                    let result = run_scan(
                        &AntennaSet::AllApproach,
                        SearchMode::Presence,
                        &zones_snapshot,
                    ).await;
                    let now_any = !result.keys_found.is_empty();
                    let now_count = result.keys_found.len() as u8;

                    if now_any != approach_state || now_count != approach_keys {
                        approach_state = now_any;
                        approach_keys = now_count;
                        let next_interval = if now_any {
                            self.slow_cadence
                        } else {
                            self.fast_cadence
                        };
                        let _ = self.bus.publish(APPROACH_STATE_OUT, SignalValue::Bool(now_any)).await;
                        let _ = self.bus.publish(APPROACH_KEYS_OUT, SignalValue::Uint8(now_count)).await;
                        let _ = self
                            .bus
                            .publish(
                                APPROACH_POLL_INTERVAL_OUT,
                                SignalValue::Uint16(next_interval.as_millis() as u16),
                            )
                            .await;
                        tracing::debug!(
                            approach_state, approach_keys, ?next_interval,
                            "KeySearchArbiter: approach state changed"
                        );
                    }

                    // Schedule next poll.
                    let next_interval = if approach_state {
                        self.slow_cadence
                    } else {
                        self.fast_cadence
                    };
                    poll_deadline = started + next_interval;
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
async fn handle_request(
    req: KeySearchRequest,
    zones: &Arc<tokio::sync::Mutex<HashMap<KeySlot, Zone>>>,
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

    cache.push((
        req.antennas.clone(),
        req.mode,
        result.clone(),
        Instant::now(),
    ));
    let _ = req.response.send(result);
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

    // ── Phase 2: approach poll loop ─────────────────────────────────────

    fn approach_state(bus: &MockBus) -> Option<bool> {
        match bus.latest_value(APPROACH_STATE_OUT) {
            Some(SignalValue::Bool(b)) => Some(b),
            _ => None,
        }
    }

    fn approach_keys(bus: &MockBus) -> Option<u8> {
        match bus.latest_value(APPROACH_KEYS_OUT) {
            Some(SignalValue::Uint8(v)) => Some(v),
            _ => None,
        }
    }

    fn approach_interval(bus: &MockBus) -> Option<u16> {
        match bus.latest_value(APPROACH_POLL_INTERVAL_OUT) {
            Some(SignalValue::Uint16(v)) => Some(v),
            _ => None,
        }
    }

    /// Yield + a tiny sleep in real time so spawned subscribers
    /// process injected signals before we assert.  Used in tests
    /// that don't pause virtual time.
    async fn settle_real() {
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
        sleep(Duration::from_millis(5)).await;
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
    }

    /// Spawn an arbiter with very short cadences so we can exercise
    /// the poll loop in real time without making the test suite slow.
    /// Fast=20 ms, slow=200 ms.  Adds the 50 ms poll latency on top.
    async fn setup_short_cadence() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let (arb, _handle, rx) = KeySearchArbiter::new_with_rx(Arc::clone(&bus));
        tokio::spawn(
            arb.with_cadence(Duration::from_millis(20), Duration::from_millis(200))
                .run(rx),
        );
        settle_real().await;
        bus
    }

    #[tokio::test]
    async fn approach_state_starts_false_with_no_keys() {
        let (bus, _h) = setup().await;
        settle_real().await;
        assert_eq!(approach_state(&bus), Some(false));
        assert_eq!(approach_keys(&bus), Some(0));
        // Initial cadence published is fast (no key detected).
        assert_eq!(approach_interval(&bus), Some(700));
    }

    #[tokio::test]
    async fn approach_state_flips_to_true_when_key_enters_approach() {
        let bus = setup_short_cadence().await;
        place(&bus, 0, Zone::Approach);
        // One fast cadence (20) + scan latency (50) = ~70 ms; give plenty of slack.
        sleep(Duration::from_millis(200)).await;
        assert_eq!(approach_state(&bus), Some(true));
        assert_eq!(approach_keys(&bus), Some(1));
        assert_eq!(approach_interval(&bus), Some(200)); // slow cadence
    }

    #[tokio::test]
    async fn approach_state_flips_back_when_key_leaves() {
        let bus = setup_short_cadence().await;
        place(&bus, 0, Zone::Approach);
        sleep(Duration::from_millis(200)).await;
        assert_eq!(approach_state(&bus), Some(true));

        // Move the fob out — should flip back after one slow cycle.
        place(&bus, 0, Zone::OutOfRange);
        sleep(Duration::from_millis(400)).await;
        assert_eq!(approach_state(&bus), Some(false));
        assert_eq!(approach_keys(&bus), Some(0));
        assert_eq!(approach_interval(&bus), Some(20)); // fast cadence
    }

    #[tokio::test]
    async fn poll_suspended_on_ignition_on() {
        let bus = setup_short_cadence().await;
        // Place a fob in Approach and immediately turn ignition ON.
        place(&bus, 0, Zone::Approach);
        bus.inject(IGNITION_STATE_SIGNAL, SignalValue::String("ON".into()));
        // Wait well past what would be a poll cycle.
        sleep(Duration::from_millis(400)).await;
        // Suspension forces ApproachState=false and interval=0.
        assert_eq!(approach_state(&bus), Some(false));
        assert_eq!(approach_keys(&bus), Some(0));
        assert_eq!(approach_interval(&bus), Some(0));
    }

    #[tokio::test]
    async fn poll_resumes_when_ignition_returns_to_off() {
        let bus = setup_short_cadence().await;
        // Suspend first.
        bus.inject(IGNITION_STATE_SIGNAL, SignalValue::String("ON".into()));
        sleep(Duration::from_millis(50)).await;
        place(&bus, 0, Zone::Approach);
        sleep(Duration::from_millis(200)).await;
        assert_eq!(approach_state(&bus), Some(false), "suspended");

        // Resume — kick is immediate, plus the scan latency.
        bus.inject(IGNITION_STATE_SIGNAL, SignalValue::String("OFF".into()));
        sleep(Duration::from_millis(150)).await;
        assert_eq!(approach_state(&bus), Some(true));
        assert_eq!(approach_interval(&bus), Some(200));
    }
}
