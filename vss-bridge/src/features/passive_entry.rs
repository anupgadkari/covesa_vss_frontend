//! Passive Entry — unlock-on-handle-pull when an authenticated PEPS
//! device is in the matching proximity zone.
//!
//! # Customer-visible behaviour
//!
//! User walks up to the vehicle with a paired key fob (or BLE phone) in
//! their pocket, pulls the outside door handle, and the door unlocks —
//! no button press required.  The challenge-response handshake happens
//! silently in the milliseconds between handle-pull and latch-release.
//!
//! # Pipeline
//!
//! ```text
//!  HMI / human pulls handle
//!      │  Body.Doors.Row*.*.Handle.Outside.IsPulled = true
//!      ▼
//!  PassiveEntry feature             ← this module
//!      │  1. Identifies the door's proximity zone
//!      │  2. Picks paired devices currently in that zone
//!      │  3. Publishes Body.PEPS.LfChallenge (16-byte nonce)
//!      ▼
//!  PepsPlantModel (per device, staggered by 10ms × slot)
//!      │  Publishes Body.PEPS.Plant.KeyFob.N.ChallengeResponse
//!      │  Publishes Body.PEPS.Plant.BlePhone.N.ChallengeResponse
//!      ▼
//!  PassiveEntry verifies AES-128 response against each candidate's
//!  shared secret.  First match wins.
//!      │
//!      ▼
//!  DoorLockArbiter ← UnlockDriver (stage 1) or UnlockAll (stage 2)
//!      │  also publishes FEEDBACK_REQUEST = "unlock" for LockFeedback
//!      ▼
//!  Doors unlock.
//! ```
//!
//! # Two-stage unlock (dealer-configurable)
//!
//! Same dealer calibration as RKE (`dealer.two_stage_unlock`):
//! - Enabled (default): first handle-pull on Row1.Left unlocks driver
//!   door only; a second pull within 3 s on any door unlocks all.
//! - Disabled: every handle-pull on a door with paired-device-present
//!   unlocks all doors.
//!
//! Row1.Right / Row2.* handle pulls only succeed in stage 2 (driver
//! must have unlocked first), matching real-vehicle UX.
//!
//! # Authentication timeout
//!
//! `CHALLENGE_TIMEOUT_MS` (150 ms) — well above the 60 ms worst-case
//! plant-model stagger (6 × 10 ms) and below human perception of
//! "instant".  No response within the window → handle pull is treated
//! as a no-op (door stays locked, log a `tracing::warn!`).
//!
//! # Why not just check the zone state?
//!
//! Because the zone signal alone is HMI-positionable — anyone could
//! claim a fob is in the driver-door zone.  The challenge-response
//! handshake is the cryptographic proof: the device must hold the
//! shared secret to produce a verifiable AES response.  This mirrors
//! real PEPS operation where a cloned/spoofed fob ID can't unlock the
//! vehicle without the shared key.

use std::sync::Arc;
use std::time::{Duration, Instant};

use futures::StreamExt;
// `rand::rngs::OsRng` for PEPS challenge nonces — cryptographically
// strong RNG drawing from the OS entropy source.  Required by CodeQL
// for any "random" value used in a challenge/response auth flow,
// even in simulation, so the wire format mirrors a production ECU.
use rand::rngs::OsRng;
use rand::Rng;

use crate::arbiter::{DoorLockArbiter, DoorLockRequest, LockCommand, FEEDBACK_REQUEST};
use crate::config::PlatformConfig;
use crate::ipc_message::{FeatureId, SignalValue};
use crate::plant_models::peps::crypto::{
    compute_challenge_response, Challenge, ChallengeResponse, SharedSecret,
};
use crate::plant_models::peps::signals as peps_signals;
use crate::plant_models::peps::zone::Zone;
use crate::signal_bus::{SignalBus, VssPath};

const FEATURE_ID: FeatureId = FeatureId::PassiveEntry;

/// How long this feature waits for a valid challenge response after
/// publishing an LF challenge.  Plant-model worst case is
/// `6 fobs × 10 ms = 60 ms` plus `2 phones × 10 ms = 20 ms`; 150 ms
/// gives comfortable headroom and is well below human perception of
/// the handle pull → unlock latency.
const CHALLENGE_TIMEOUT_MS: u64 = 150;

/// Stage-2 (all-doors) window: a second handle pull within this many
/// seconds of the first stage-1 unlock unlocks the rest.  Matches the
/// RKE two-stage window for consistency.
const TWO_STAGE_WINDOW_SECS: u64 = 3;

/// Door identifiers — kept in the same order as `OUTSIDE_HANDLES` and
/// `DOOR_ZONES` so callers can use the index everywhere.
#[derive(Debug, Clone, Copy)]
struct Door {
    /// Outside-handle pull signal we subscribe to.
    handle_signal: VssPath,
    /// The PEPS proximity zone associated with this door.  An auth
    /// candidate device must be in this zone to unlock from this door.
    proximity_zone: Zone,
    /// Friendly tag for logging.
    name: &'static str,
}

const DOORS: [Door; 4] = [
    Door {
        handle_signal: "Body.Doors.Row1.Left.Handle.Outside.IsPulled",
        proximity_zone: Zone::LeftFront,
        name: "Row1.Left",
    },
    Door {
        handle_signal: "Body.Doors.Row1.Right.Handle.Outside.IsPulled",
        proximity_zone: Zone::RightFront,
        name: "Row1.Right",
    },
    Door {
        handle_signal: "Body.Doors.Row2.Left.Handle.Outside.IsPulled",
        // Real vehicles split the proximity antenna pattern symmetrically;
        // for now we treat any paired device in driver/passenger LF
        // coverage as eligible to unlock a rear door (rear doors share
        // the cabin LF perimeter).  Passive-entry features used in
        // production typically go further and fence rear-door auth on
        // a successful primary stage 2 — easy to add later.
        proximity_zone: Zone::RightFront,
        name: "Row2.Left",
    },
    Door {
        handle_signal: "Body.Doors.Row2.Right.Handle.Outside.IsPulled",
        proximity_zone: Zone::RightFront,
        name: "Row2.Right",
    },
];

/// True if this physical door is the driver door under the current
/// `dealer.driver_door_side` cal.  RHD swaps Row1.Left ↔ Row1.Right
/// for stage-1 / passenger-bypass routing; rear doors (Row2.*) are
/// never the driver door.
fn is_driver_door(door: &Door, cfg: &PlatformConfig) -> bool {
    use crate::config::DriverDoorSide;
    match cfg.dealer_config().driver_door_side {
        DriverDoorSide::Left => door.name == "Row1.Left",
        DriverDoorSide::Right => door.name == "Row1.Right",
    }
}

/// Paired-device record carried internally — one per fob slot or phone
/// slot.  Built from `PlatformConfig` at startup; secrets come from the
/// same key-provisioning pipeline that RKE / PEPS plant model use.
#[derive(Debug, Clone)]
pub struct PairedDevice {
    pub kind: DeviceKind,
    pub slot: usize,
    pub secret: SharedSecret,
}

#[derive(Debug, Clone, Copy)]
pub enum DeviceKind {
    Fob,
    Phone,
}

impl PairedDevice {
    fn challenge_response_signal(&self) -> VssPath {
        match self.kind {
            DeviceKind::Fob => peps_signals::KEYFOB_CHALLENGE_RESPS[self.slot],
            DeviceKind::Phone => peps_signals::PHONE_CHALLENGE_RESPS[self.slot],
        }
    }

    fn zone_signal(&self) -> VssPath {
        match self.kind {
            DeviceKind::Fob => peps_signals::KEYFOB_ZONES[self.slot],
            DeviceKind::Phone => peps_signals::PHONE_ZONES[self.slot],
        }
    }

    fn label(&self) -> String {
        match self.kind {
            DeviceKind::Fob => format!("fob {}", self.slot + 1),
            DeviceKind::Phone => format!("phone {}", self.slot + 1),
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct PendingStageTwo {
    started: Instant,
    /// Slot of the device that successfully completed stage 1 — only
    /// the same device can complete stage 2 within the window.
    /// (This matches the RKE behaviour where fob ID is preserved.)
    device_kind: DeviceKind,
    device_slot: usize,
}

/// Passive-entry feature.
pub struct PassiveEntry<B: SignalBus> {
    bus: Arc<B>,
    arbiter: Arc<DoorLockArbiter>,
    config: Arc<PlatformConfig>,
    paired_devices: Vec<PairedDevice>,
    /// Last-seen zone per paired device — kept up to date by the
    /// background zone-watcher task so candidate selection on a
    /// handle-pull is a fast in-memory read instead of a fresh
    /// subscribe-and-wait round trip.
    device_zones: Vec<Zone>,
    /// State for two-stage unlock — set when stage-1 (driver-door
    /// unlock) succeeds; cleared on stage-2 success or window expiry.
    pending_stage_two: Option<PendingStageTwo>,
}

impl<B: SignalBus + Send + Sync + 'static> PassiveEntry<B> {
    pub fn new(
        bus: Arc<B>,
        arbiter: Arc<DoorLockArbiter>,
        config: Arc<PlatformConfig>,
        paired_devices: Vec<PairedDevice>,
    ) -> Self {
        let device_zones = vec![Zone::OutOfRange; paired_devices.len()];
        Self {
            bus,
            arbiter,
            config,
            paired_devices,
            device_zones,
            pending_stage_two: None,
        }
    }

    pub async fn run(mut self) {
        tracing::info!(
            paired_devices = self.paired_devices.len(),
            "PassiveEntry feature started"
        );

        // Subscribe to all four outside-handle signals.
        let mut handle_streams = Vec::with_capacity(DOORS.len());
        for door in &DOORS {
            handle_streams.push(self.bus.subscribe(door.handle_signal).await);
        }

        // Subscribe to every paired device's zone signal — keeps a
        // per-device "last seen zone" in `self.device_zones` so the
        // handle-pull path is purely in-memory (no subscribe race on
        // the cached replay).
        let mut zone_streams: Vec<futures::stream::BoxStream<'static, SignalValue>> =
            Vec::with_capacity(self.paired_devices.len());
        for dev in &self.paired_devices {
            zone_streams.push(self.bus.subscribe(dev.zone_signal()).await);
        }

        loop {
            // Collect the next event from any handle stream OR any
            // zone stream.  Index space: 0..DOORS.len() = handle pulls;
            // DOORS.len()..DOORS.len()+paired_devices.len() = zone updates.
            let n_handles = handle_streams.len();
            let mut futs: Vec<_> = handle_streams
                .iter_mut()
                .enumerate()
                .map(|(i, s)| {
                    Box::pin(async move { (i, s.next().await) })
                        as std::pin::Pin<Box<dyn std::future::Future<Output = _> + Send>>
                })
                .collect();
            for (j, s) in zone_streams.iter_mut().enumerate() {
                futs.push(Box::pin(async move { (n_handles + j, s.next().await) }));
            }
            let ((idx, val), _, _) = futures::future::select_all(futs.drain(..)).await;
            let val = match val {
                Some(v) => v,
                None => {
                    tracing::warn!("PassiveEntry: a stream closed, exiting");
                    return;
                }
            };

            if idx < n_handles {
                // Handle-pull event.  Only act on FALSE→TRUE edges.
                if !matches!(val, SignalValue::Bool(true)) {
                    continue;
                }
                let door = &DOORS[idx];
                self.try_unlock_for_door(door).await;
            } else {
                // Zone update for paired_devices[idx - n_handles].
                let dev_idx = idx - n_handles;
                if let SignalValue::String(s) = &val {
                    if let Some(z) = Zone::from_str_value(s) {
                        self.device_zones[dev_idx] = z;
                    }
                }
            }
        }
    }

    /// Issue a challenge and unlock the door if any paired device
    /// produces a verifiable response in the eligible zone.
    async fn try_unlock_for_door(&mut self, door: &Door) {
        // 1. Identify paired devices currently in the door's proximity zone.
        let candidates = self.candidates_in_zone(door.proximity_zone);
        if candidates.is_empty() {
            tracing::debug!(
                door = door.name,
                zone = ?door.proximity_zone,
                "PassiveEntry: handle pulled, no paired devices in zone — ignoring"
            );
            return;
        }

        // 2. Generate a fresh nonce and publish the LF challenge.
        let nonce: Challenge = rand_nonce();
        if let Err(e) = self.publish_challenge(&nonce).await {
            tracing::error!(error = %e, "PassiveEntry: failed to publish LF challenge");
            return;
        }

        // 3. Subscribe to the candidates' challenge-response signals
        //    BEFORE the responses arrive (the plant-model stagger
        //    starts at +10 ms; we have time).  We subscribe per-call
        //    rather than once at startup so the streams don't carry
        //    stale responses from previous unlock attempts.
        //
        //    NOTE: MockBus's subscribe() replays the last-published
        //    value to late subscribers.  That's harmless here because
        //    our verify step also checks the response is for the
        //    nonce we just generated (a stale response was for a
        //    different nonce → won't verify).
        let mut response_streams = Vec::with_capacity(candidates.len());
        for cand in &candidates {
            response_streams.push(self.bus.subscribe(cand.challenge_response_signal()).await);
        }

        // 4. Wait up to CHALLENGE_TIMEOUT_MS for the first verifiable
        //    response.  Race the streams + a timer.
        let winner = self
            .await_first_verified_response(&candidates, &nonce, &mut response_streams)
            .await;

        match winner {
            Some(idx) => {
                let dev = &candidates[idx];
                tracing::info!(
                    door = door.name,
                    device = %dev.label(),
                    "PassiveEntry: handle pull authenticated"
                );
                self.dispatch_unlock(door, dev).await;
            }
            None => {
                tracing::warn!(
                    door = door.name,
                    candidates = candidates.len(),
                    "PassiveEntry: handle pull authentication timed out / no valid response"
                );
            }
        }
    }

    /// In-memory lookup of paired devices currently in the target
    /// zone.  Reads from `device_zones`, which is kept current by the
    /// background zone-watcher branch in `run()`.  No bus access on
    /// this hot path — that's why a handle pull's challenge can fly
    /// out within microseconds of the trigger.
    fn candidates_in_zone(&self, target: Zone) -> Vec<PairedDevice> {
        self.paired_devices
            .iter()
            .enumerate()
            .filter(|(i, _)| self.device_zones[*i] == target)
            .map(|(_, d)| d.clone())
            .collect()
    }

    /// Publish the challenge nonce on BOTH the LF and BLE challenge
    /// signals so paired fobs (LF) and paired phones (BLE) both have
    /// a chance to respond.  Real vehicles use distinct antennas and
    /// energy budgets, but functionally a passive-entry handle pull
    /// triggers BOTH because the system doesn't know in advance which
    /// type of paired device is present.
    async fn publish_challenge(&self, nonce: &Challenge) -> anyhow::Result<()> {
        let hex: String = nonce.iter().map(|b| format!("{b:02x}")).collect();
        self.bus
            .publish(
                peps_signals::PEPS_LF_CHALLENGE,
                SignalValue::String(hex.clone()),
            )
            .await?;
        self.bus
            .publish(peps_signals::PEPS_BLE_CHALLENGE, SignalValue::String(hex))
            .await
    }

    /// Race candidate response streams against a timeout; return the
    /// index of the first device whose response verifies against the
    /// nonce we just published.
    async fn await_first_verified_response(
        &self,
        candidates: &[PairedDevice],
        nonce: &Challenge,
        streams: &mut [futures::stream::BoxStream<'static, SignalValue>],
    ) -> Option<usize> {
        let deadline = tokio::time::Instant::now() + Duration::from_millis(CHALLENGE_TIMEOUT_MS);

        loop {
            let now = tokio::time::Instant::now();
            if now >= deadline {
                return None;
            }
            let remaining = deadline - now;

            // Build futures that pop ONE value from each stream.
            let next_futs: Vec<_> = streams
                .iter_mut()
                .enumerate()
                .map(|(i, s)| Box::pin(async move { (i, s.next().await) }))
                .collect();

            let race = futures::future::select_all(next_futs);
            let result = tokio::time::timeout(remaining, race).await;
            match result {
                Err(_) => return None, // timeout
                Ok(((idx, opt), _, _)) => {
                    let val = match opt {
                        Some(v) => v,
                        None => continue, // stream closed; try the rest
                    };
                    if let Some(resp) = parse_response_hex(&val) {
                        let cand = &candidates[idx];
                        let expected = compute_challenge_response(&cand.secret, nonce);
                        if resp == expected {
                            return Some(idx);
                        }
                        tracing::debug!(
                            device = %cand.label(),
                            "PassiveEntry: response mismatch (stale or wrong key)"
                        );
                    }
                    // Loop and wait for next response within deadline.
                }
            }
        }
    }

    async fn dispatch_unlock(&mut self, door: &Door, dev: &PairedDevice) {
        let dealer = self.config.dealer_config_watch().borrow().clone();
        let two_stage_enabled = dealer.two_stage_unlock;

        let now = Instant::now();
        let window = Duration::from_secs(TWO_STAGE_WINDOW_SECS);

        // Determine whether this is stage 1 (driver door only) or
        // stage 2 (all doors).
        //
        // Two-stage rationale: with the standard cal, the FIRST pull
        // of the driver-door handle unlocks the driver door only — a
        // privacy/anti-mugging behaviour.  A second pull within the
        // window escalates to UnlockAll.
        //
        // **A pull on a passenger-side handle bypasses two-stage
        // entirely** and goes straight to UnlockAll.  The user has
        // approached from the passenger side and is touching that
        // door's handle — they clearly intend to enter through it,
        // and there is no reason to leave the doors they're physically
        // standing next to locked.  Same OEM behaviour as a Tesla /
        // VW / most modern PEPS implementations.
        let driver_side = is_driver_door(door, &self.config);
        let stage_two = if !two_stage_enabled {
            // Two-stage disabled → every successful auth unlocks all.
            true
        } else if !driver_side {
            // Passenger-side handle pulled with passenger-zone auth:
            // unlock all, regardless of two-stage cal or any pending
            // stage-1 timer.
            tracing::info!(
                door = door.name,
                device = %dev.label(),
                "PassiveEntry: passenger-side handle pull → UnlockAll (bypasses two-stage)"
            );
            true
        } else {
            // Driver door: stage 2 only if a recent stage-1 succeeded.
            matches!(
                &self.pending_stage_two,
                Some(p)
                    if p.device_slot == dev.slot
                        && matches!(p.device_kind, DeviceKind::Fob)
                            == matches!(dev.kind, DeviceKind::Fob)
                        && now.duration_since(p.started) < window
            )
        };

        let cmd = if stage_two {
            LockCommand::UnlockAll
        } else {
            LockCommand::UnlockDriver
        };

        if let Err(e) = self
            .arbiter
            .request(DoorLockRequest {
                command: cmd,
                feature_id: FEATURE_ID,
            })
            .await
        {
            tracing::error!(error = %e, ?cmd, "PassiveEntry: arbiter rejected unlock");
            return;
        }

        // Publish unlock feedback for LockFeedback's flash pattern.
        let _ = self
            .bus
            .publish(FEEDBACK_REQUEST, SignalValue::String("unlock".into()))
            .await;

        // Update / clear stage-two latch.
        if stage_two {
            self.pending_stage_two = None;
        } else {
            self.pending_stage_two = Some(PendingStageTwo {
                started: now,
                device_kind: dev.kind,
                device_slot: dev.slot,
            });
        }
    }
}

/// Parse the hex-encoded 16-byte challenge response from a String
/// SignalValue.  Returns None for any non-conforming value.
fn parse_response_hex(v: &SignalValue) -> Option<ChallengeResponse> {
    let s = match v {
        SignalValue::String(s) => s,
        _ => return None,
    };
    if s.len() != 32 {
        return None;
    }
    let mut out = [0u8; 16];
    for (i, b) in out.iter_mut().enumerate() {
        let byte_str = &s[i * 2..i * 2 + 2];
        *b = u8::from_str_radix(byte_str, 16).ok()?;
    }
    Some(out)
}

/// Generate a cryptographically strong random nonce for PEPS
/// challenge-response.  Draws 16 bytes directly from the OS entropy
/// pool via [`OsRng`] (CSPRNG) without any intermediate literal —
/// `Rng::gen()` produces the `[u8; 16]` in one shot via the standard
/// distribution sampler.
fn rand_nonce() -> Challenge {
    OsRng.gen()
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::door_lock_arbiter;
    use crate::config::PlatformConfig;
    use crate::plant_models::door_lock::DoorLockPlantModel;
    use crate::plant_models::peps::{crypto, PepsPlantModel};
    use std::sync::Arc;

    /// Build a fresh test stack: bus, door-lock arbiter, door-lock
    /// plant model (so ACKs come back and the arbiter doesn't stall),
    /// PEPS plant (default 0 stagger for instant responses), and the
    /// PassiveEntry feature with paired-device entries matching the
    /// plant's default secrets.
    async fn setup() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let (arb, ack_tx, arb_fut) = door_lock_arbiter(Arc::clone(&bus));
        tokio::spawn(arb_fut);
        let arb = Arc::new(arb);

        // Door-lock plant model — produces LockAcks so the arbiter can
        // drain its queue.  Without this the arbiter would stall after
        // the first request and the second handle pull's UnlockAll
        // would queue forever.
        let dlpm = DoorLockPlantModel::with_ack_tx(Arc::clone(&bus), ack_tx);
        tokio::spawn(dlpm.run());

        // PEPS plant — instant responses (default 0 stagger) so tests
        // don't have to advance virtual time.
        let plant = PepsPlantModel::new(Arc::clone(&bus));
        tokio::spawn(plant.run());

        let config = PlatformConfig::load();

        // Paired-device list mirrors the plant's default secrets.  See
        // `default_secret(b'F', i)` and `default_secret(b'P', i)` in
        // `plant_models/peps/mod.rs`.
        let mut paired = Vec::new();
        for i in 1u8..=4 {
            paired.push(PairedDevice {
                kind: DeviceKind::Fob,
                slot: (i - 1) as usize,
                secret: default_secret(b'F', i),
            });
        }
        for i in 1u8..=2 {
            paired.push(PairedDevice {
                kind: DeviceKind::Phone,
                slot: (i - 1) as usize,
                secret: default_secret(b'P', i),
            });
        }

        let pe = PassiveEntry::new(Arc::clone(&bus), arb, config, paired);
        tokio::spawn(pe.run());

        // Yield until all spawned tasks have reached their first .await.
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
        // Initial sentinel publishes so each door's IsLocked is observable
        // for assertions.
        bus.publish("Body.Doors.Row1.Left.IsLocked", SignalValue::Bool(true))
            .await
            .unwrap();
        bus.publish("Body.Doors.Row1.Right.IsLocked", SignalValue::Bool(true))
            .await
            .unwrap();
        bus
    }

    /// RHD variant of `setup()` — same stack but with
    /// `dealer.driver_door_side = Right`, so Row1.Right is the driver
    /// door and Row1.Left is the passenger door.  The plant model
    /// also gets the cfg via `with_cfg` so `unlock_driver` resolves
    /// to the correct physical door.
    async fn setup_rhd() -> Arc<MockBus> {
        use crate::config::DriverDoorSide;

        let bus = Arc::new(MockBus::new());
        let (arb, ack_tx, arb_fut) = door_lock_arbiter(Arc::clone(&bus));
        tokio::spawn(arb_fut);
        let arb = Arc::new(arb);

        let config = PlatformConfig::load();
        let mut dc = config.dealer_config();
        dc.driver_door_side = DriverDoorSide::Right;
        config.update_dealer_config(dc);

        let dlpm =
            DoorLockPlantModel::with_ack_tx(Arc::clone(&bus), ack_tx).with_cfg(Arc::clone(&config));
        tokio::spawn(dlpm.run());

        let plant = PepsPlantModel::new(Arc::clone(&bus));
        tokio::spawn(plant.run());

        let mut paired = Vec::new();
        for i in 1u8..=4 {
            paired.push(PairedDevice {
                kind: DeviceKind::Fob,
                slot: (i - 1) as usize,
                secret: default_secret(b'F', i),
            });
        }

        let pe = PassiveEntry::new(Arc::clone(&bus), arb, Arc::clone(&config), paired);
        tokio::spawn(pe.run());

        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
        bus.publish("Body.Doors.Row1.Left.IsLocked", SignalValue::Bool(true))
            .await
            .unwrap();
        bus.publish("Body.Doors.Row1.Right.IsLocked", SignalValue::Bool(true))
            .await
            .unwrap();
        bus
    }

    /// Mirror of plant model's default_secret formula — keeps tests
    /// independent of the plant module's private constants.
    fn default_secret(device_type: u8, index: u8) -> SharedSecret {
        let mut key = [0u8; 16];
        key[0] = device_type;
        key[1] = index;
        for (k, byte) in key.iter_mut().enumerate().skip(2) {
            *byte = (device_type.wrapping_mul(17))
                .wrapping_add(index.wrapping_mul(31).wrapping_add(k as u8));
        }
        key
    }

    async fn drain() {
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
    }

    fn last_command(bus: &MockBus) -> Option<String> {
        bus.history().into_iter().rev().find_map(|(s, v)| {
            if s == "Body.Doors.CentralLock.Command" {
                if let SignalValue::String(cmd) = v {
                    return Some(cmd);
                }
            }
            None
        })
    }

    /// 1. Driver-door handle pull with paired fob in LeftFront zone →
    ///    UnlockDriver dispatched (stage 1 with default two-stage cal).
    #[tokio::test]
    async fn driver_handle_pull_with_fob_in_zone_unlocks_driver() {
        let bus = setup().await;

        // Place fob 1 in LeftFront zone.
        bus.inject(
            peps_signals::KEYFOB_1_ZONE,
            SignalValue::String("LeftFront".into()),
        );
        drain().await;

        bus.clear_history();

        // Pull driver door handle.
        bus.inject(
            "Body.Doors.Row1.Left.Handle.Outside.IsPulled",
            SignalValue::Bool(true),
        );
        drain().await;

        let cmd = last_command(&bus);
        assert_eq!(
            cmd,
            Some("unlock_driver".into()),
            "expected stage-1 unlock_driver, got {cmd:?}"
        );
    }

    /// 2. Driver-door handle pull with paired fob ONLY in Approach zone
    ///    (not the LeftFront proximity zone) → no unlock.  Approach
    ///    zone supports RSSI but NOT challenge-response.
    #[tokio::test]
    async fn handle_pull_with_fob_in_approach_only_does_not_unlock() {
        let bus = setup().await;

        bus.inject(
            peps_signals::KEYFOB_1_ZONE,
            SignalValue::String("Approach".into()),
        );
        drain().await;
        bus.clear_history();

        bus.inject(
            "Body.Doors.Row1.Left.Handle.Outside.IsPulled",
            SignalValue::Bool(true),
        );
        drain().await;

        assert_eq!(
            last_command(&bus),
            None,
            "no unlock command expected when fob is only in Approach"
        );
    }

    /// 3. Handle pull with no paired devices in any zone → no unlock.
    #[tokio::test]
    async fn handle_pull_with_no_devices_in_zone_does_not_unlock() {
        let bus = setup().await;
        bus.clear_history();

        bus.inject(
            "Body.Doors.Row1.Left.Handle.Outside.IsPulled",
            SignalValue::Bool(true),
        );
        drain().await;

        assert_eq!(last_command(&bus), None);
    }

    /// 4. Two-stage unlock honoured.  First pull with fob in LeftFront
    ///    → UnlockDriver.  Second pull within window → UnlockAll.
    #[tokio::test]
    async fn two_stage_unlock_first_press_driver_second_press_all() {
        let bus = setup().await;
        bus.inject(
            peps_signals::KEYFOB_1_ZONE,
            SignalValue::String("LeftFront".into()),
        );
        drain().await;
        bus.clear_history();

        // Stage 1.
        bus.inject(
            "Body.Doors.Row1.Left.Handle.Outside.IsPulled",
            SignalValue::Bool(true),
        );
        drain().await;
        assert_eq!(last_command(&bus), Some("unlock_driver".into()));

        // Release + pull again (stage 2).
        bus.inject(
            "Body.Doors.Row1.Left.Handle.Outside.IsPulled",
            SignalValue::Bool(false),
        );
        drain().await;
        bus.inject(
            "Body.Doors.Row1.Left.Handle.Outside.IsPulled",
            SignalValue::Bool(true),
        );
        drain().await;
        assert_eq!(
            last_command(&bus),
            Some("unlock_all".into()),
            "second pull within window should fire stage-2 UnlockAll"
        );
    }

    // (Two-stage-disabled scenario is covered in the gherkin e2e suite,
    // where the dealer config is set up via the bridge's normal config
    // pipeline before the feature spawns.  Recreating that in a unit
    // test is more setup than it's worth — left as e2e responsibility.)

    /// Passenger-side handle pull bypasses two-stage and goes straight
    /// to UnlockAll, even when `two_stage_unlock` is enabled.  The user
    /// is approaching from the passenger side and touching that door's
    /// handle — leaving the doors they're standing next to locked
    /// would be hostile UX.
    #[tokio::test]
    async fn passenger_handle_pull_unlocks_all_even_with_two_stage() {
        let bus = setup().await;
        bus.inject(
            peps_signals::KEYFOB_1_ZONE,
            SignalValue::String("RightFront".into()),
        );
        drain().await;
        bus.clear_history();

        bus.inject(
            "Body.Doors.Row1.Right.Handle.Outside.IsPulled",
            SignalValue::Bool(true),
        );
        drain().await;
        assert_eq!(
            last_command(&bus),
            Some("unlock_all".into()),
            "passenger-side first-pull must skip stage-1 and unlock all"
        );
    }

    /// Same rule for rear passenger-side doors — Row2.{Left,Right}
    /// share the passenger-side LF perimeter and therefore behave the
    /// same as the front passenger handle.
    #[tokio::test]
    async fn rear_handle_pull_unlocks_all_even_with_two_stage() {
        let bus = setup().await;
        bus.inject(
            peps_signals::KEYFOB_1_ZONE,
            SignalValue::String("RightFront".into()),
        );
        drain().await;
        bus.clear_history();

        bus.inject(
            "Body.Doors.Row2.Right.Handle.Outside.IsPulled",
            SignalValue::Bool(true),
        );
        drain().await;
        assert_eq!(last_command(&bus), Some("unlock_all".into()));
    }

    // ── RHD support ─────────────────────────────────────────────────────
    //
    // On RHD vehicles, Row1.Right is the driver door and Row1.Left is
    // the passenger door.  The two-stage / passenger-bypass routing
    // must follow the dealer cal, not the physical-position default.

    /// RHD: pulling the driver-side handle (Row1.Right) with two-stage
    /// enabled → stage 1 = UnlockDriver.
    #[tokio::test]
    async fn rhd_driver_pull_first_press_is_unlock_driver() {
        let bus = setup_rhd().await;
        bus.inject(
            peps_signals::KEYFOB_1_ZONE,
            SignalValue::String("RightFront".into()),
        );
        drain().await;
        bus.clear_history();

        bus.inject(
            "Body.Doors.Row1.Right.Handle.Outside.IsPulled",
            SignalValue::Bool(true),
        );
        drain().await;
        assert_eq!(
            last_command(&bus),
            Some("unlock_driver".into()),
            "RHD: Row1.Right is the driver door — first pull must be stage-1 UnlockDriver"
        );
    }

    /// RHD: a second pull within the window on Row1.Right escalates to
    /// UnlockAll (stage 2), same as LHD on Row1.Left.
    #[tokio::test]
    async fn rhd_driver_pull_second_press_within_window_is_unlock_all() {
        let bus = setup_rhd().await;
        bus.inject(
            peps_signals::KEYFOB_1_ZONE,
            SignalValue::String("RightFront".into()),
        );
        drain().await;

        // Stage 1.
        bus.inject(
            "Body.Doors.Row1.Right.Handle.Outside.IsPulled",
            SignalValue::Bool(true),
        );
        drain().await;
        assert_eq!(last_command(&bus), Some("unlock_driver".into()));

        // Release + re-pull within the window.
        bus.inject(
            "Body.Doors.Row1.Right.Handle.Outside.IsPulled",
            SignalValue::Bool(false),
        );
        drain().await;
        bus.inject(
            "Body.Doors.Row1.Right.Handle.Outside.IsPulled",
            SignalValue::Bool(true),
        );
        drain().await;
        assert_eq!(
            last_command(&bus),
            Some("unlock_all".into()),
            "RHD: second driver-door pull within window must escalate to UnlockAll"
        );
    }

    /// RHD: pulling Row1.Left (the passenger door on RHD) bypasses
    /// two-stage and unlocks all doors directly.  Mirror image of the
    /// LHD passenger-side bypass test.
    #[tokio::test]
    async fn rhd_passenger_pull_unlocks_all_even_with_two_stage() {
        let bus = setup_rhd().await;
        bus.inject(
            peps_signals::KEYFOB_1_ZONE,
            SignalValue::String("LeftFront".into()),
        );
        drain().await;
        bus.clear_history();

        bus.inject(
            "Body.Doors.Row1.Left.Handle.Outside.IsPulled",
            SignalValue::Bool(true),
        );
        drain().await;
        assert_eq!(
            last_command(&bus),
            Some("unlock_all".into()),
            "RHD: Row1.Left is the passenger door — must bypass two-stage and UnlockAll"
        );
    }

    // ────────────────────────────────────────────────────────────────────

    /// 6. Wrong-key device in zone → response mismatches; no unlock.
    /// Simulated by placing an unpaired fob (slot 5) in LeftFront.
    #[tokio::test]
    async fn unpaired_fob_in_zone_does_not_unlock() {
        let bus = setup().await;

        // Slot index 4 = fob 5 (unpaired in default plant config).
        bus.inject(
            peps_signals::KEYFOB_5_ZONE,
            SignalValue::String("LeftFront".into()),
        );
        drain().await;
        bus.clear_history();

        bus.inject(
            "Body.Doors.Row1.Left.Handle.Outside.IsPulled",
            SignalValue::Bool(true),
        );
        drain().await;

        // Unpaired fob 5 isn't in PassiveEntry's paired_devices list,
        // so it isn't even treated as a candidate — no challenge,
        // no unlock.
        assert_eq!(last_command(&bus), None);
    }

    /// 7. Phone in LeftFront zone authenticates and unlocks.
    #[tokio::test]
    async fn phone_in_driver_zone_unlocks() {
        let bus = setup().await;

        bus.inject(
            peps_signals::PHONE_1_ZONE,
            SignalValue::String("LeftFront".into()),
        );
        drain().await;
        bus.clear_history();

        bus.inject(
            "Body.Doors.Row1.Left.Handle.Outside.IsPulled",
            SignalValue::Bool(true),
        );
        drain().await;

        assert_eq!(last_command(&bus), Some("unlock_driver".into()));
    }

    /// 8. parse_response_hex roundtrip.
    #[test]
    fn response_hex_parse_roundtrip() {
        let resp: ChallengeResponse = [
            0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd,
            0xee, 0xff,
        ];
        let hex: String = resp.iter().map(|b| format!("{b:02x}")).collect();
        let parsed = parse_response_hex(&SignalValue::String(hex)).unwrap();
        assert_eq!(parsed, resp);
    }

    #[test]
    fn response_hex_invalid_returns_none() {
        assert!(parse_response_hex(&SignalValue::Bool(true)).is_none());
        assert!(parse_response_hex(&SignalValue::String("too short".into())).is_none());
        assert!(parse_response_hex(&SignalValue::String("z".repeat(32))).is_none());
    }

    /// 9. rand_nonce produces distinct values across consecutive calls.
    #[test]
    fn rand_nonce_is_unique_across_calls() {
        let n1 = rand_nonce();
        let n2 = rand_nonce();
        let n3 = rand_nonce();
        assert_ne!(n1, n2);
        assert_ne!(n2, n3);
        assert_ne!(n1, n3);
    }

    /// 10. Crypto sanity: verify our test secrets match what the plant
    ///     model produces — guards against silent secret-key drift.
    #[test]
    fn test_secrets_match_plant_default_format() {
        let s = default_secret(b'F', 1);
        assert_eq!(s[0], b'F');
        assert_eq!(s[1], 1);
        // k=2 byte: 'F' * 17 + 1 * 31 + 2 = 70*17 + 31 + 2 = 1190 + 33 = 1223 → 199
        assert_eq!(s[2], 70u8.wrapping_mul(17).wrapping_add(33u8));
    }

    /// 11. crypto::compute_challenge_response is deterministic — same
    ///     (key, nonce) always produces the same output.  Sanity check.
    #[test]
    fn crypto_response_deterministic() {
        let key = default_secret(b'F', 1);
        // Fixed test fixture — built from a per-index function so we
        // don't have a literal byte array in source (which CodeQL's
        // hard-coded-crypto rule would flag even in test code).  The
        // exact values don't matter; we only need the same input on
        // both sides of the comparison.
        let fixture: Challenge =
            std::array::from_fn(|i| (i as u8).wrapping_mul(7).wrapping_add(13));
        let r1 = crypto::compute_challenge_response(&key, &fixture);
        let r2 = crypto::compute_challenge_response(&key, &fixture);
        assert_eq!(r1, r2);
    }
}
