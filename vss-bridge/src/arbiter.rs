//! Domain-based Signal Arbiters — grouped by actuator domain.
//!
//! Two arbiter patterns coexist:
//!
//! 1. **DomainArbiter** (Lighting, Horn, Comfort) — instant per-signal priority
//!    resolution. A new request replaces the winner if priority >= current winner.
//!
//! 2. **DoorLockArbiter** — serialized command queue with ACK handshake. The lock
//!    motor takes ~300 ms, so requests are serialized through a one-deep queue
//!    (active + pending). Special crash-unlock rules prevent the queue from being
//!    overwritten and impose a 10-second lockout after crash events.

use std::collections::HashMap;
use std::sync::Arc;

use futures::stream::StreamExt;
use tokio::sync::mpsc;
use tracing;

use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

// ---------------------------------------------------------------------------
// ActuatorRequest — what features submit to the arbiter
// ---------------------------------------------------------------------------

/// A request from a feature module to control an actuator signal.
#[derive(Debug, Clone)]
pub struct ActuatorRequest {
    pub signal: VssPath,
    pub value: SignalValue,
    pub priority: Priority,
    pub feature_id: FeatureId,
}

// ---------------------------------------------------------------------------
// AllowEntry — static priority table row
// ---------------------------------------------------------------------------

/// One row in a domain's static allow-list.
/// Defines which (feature, signal, priority) combinations are permitted.
#[derive(Debug, Clone)]
pub struct AllowEntry {
    pub feature_id: FeatureId,
    pub signal: VssPath,
    pub priority: Priority,
}

// ---------------------------------------------------------------------------
// PhysicalGate — runtime suppression of a target signal based on another
// signal's state.  Models hardware constraints that no feature should have
// to know about (e.g. puddle lamps live inside the side mirror housing,
// so when the mirror is folded the lamp can't physically project onto
// the ground regardless of which feature is requesting it).
// ---------------------------------------------------------------------------

/// A physical-layer suppression rule.  When the bus value of `gate_signal`
/// equals `suppress_when`, the arbiter forces `target` to `Bool(false)`
/// regardless of which feature is currently winning.  When the gate
/// re-opens, the arbiter re-resolves and publishes the highest-priority
/// active claim normally.
///
/// Use this for *physical* constraints (mirror folded, hood up, charge port
/// open) rather than feature-level gating.  Feature priority remains the
/// way to express *policy* contention between features.
#[derive(Debug, Clone)]
pub struct PhysicalGate {
    pub target: VssPath,
    pub gate_signal: VssPath,
    pub suppress_when: SignalValue,
}

// ---------------------------------------------------------------------------
// DomainArbiter — one per actuator domain
// ---------------------------------------------------------------------------

/// A domain-scoped arbiter that resolves per-actuator priority conflicts.
///
/// Features submit `ActuatorRequest` via the `request()` method to **claim** a
/// signal at a given priority+value. Claims persist until the feature explicitly
/// **releases** them via `release()`. The resolved winner is the highest-priority
/// active claim across all features (latest-wins on ties). When no claims remain
/// for a signal, the arbiter publishes the default-off value (`Bool(false)` for
/// boolean signals).
///
/// This claim/release model matches real body-ECU behavior: a feature actively
/// holds the actuator while it wants control, and lower-priority features
/// automatically resume when a higher-priority feature withdraws.
pub struct DomainArbiter {
    pub name: &'static str,
    tx: mpsc::Sender<ArbiterMsg>,
}

/// Internal channel message — request to claim, or release to withdraw.
#[derive(Debug)]
enum ArbiterMsg {
    Request(ActuatorRequest),
    Release {
        signal: VssPath,
        feature_id: FeatureId,
    },
    /// A physical gate's source value updated.  `target` is the signal
    /// being gated (not the gate signal itself); `closed` is true when
    /// the gate is currently suppressing (i.e. the source value matches
    /// `suppress_when`).
    GateChanged {
        target: VssPath,
        closed: bool,
    },
}

impl DomainArbiter {
    /// Create a new domain arbiter with its static allow-list.
    ///
    /// Returns the arbiter handle (for features to submit requests) and
    /// a future that must be spawned as a tokio task.
    pub fn new<B: SignalBus>(
        name: &'static str,
        allow_list: Vec<AllowEntry>,
        bus: Arc<B>,
    ) -> (Self, impl std::future::Future<Output = ()>) {
        Self::new_with_gates(name, allow_list, Vec::new(), bus)
    }

    /// Like `new`, but with one or more `PhysicalGate` entries that
    /// suppress specific target signals based on a runtime gate-signal
    /// value.  See `PhysicalGate` for semantics.
    pub fn new_with_gates<B: SignalBus>(
        name: &'static str,
        allow_list: Vec<AllowEntry>,
        gates: Vec<PhysicalGate>,
        bus: Arc<B>,
    ) -> (Self, impl std::future::Future<Output = ()>) {
        Self::new_with_gates_and_defaults(name, allow_list, gates, HashMap::new(), bus)
    }

    /// Like `new_with_gates`, plus per-signal default values that
    /// override the global `Bool(false)` fallback when no claims are
    /// active.  Required for non-boolean actuators (e.g. the window
    /// motor-direction String enum where the off-state is `STOPPED`,
    /// not `Bool(false)`).
    pub fn new_with_gates_and_defaults<B: SignalBus>(
        name: &'static str,
        allow_list: Vec<AllowEntry>,
        gates: Vec<PhysicalGate>,
        defaults: HashMap<VssPath, SignalValue>,
        bus: Arc<B>,
    ) -> (Self, impl std::future::Future<Output = ()>) {
        let (tx, rx) = mpsc::channel::<ArbiterMsg>(256);

        // Spawn one watcher per gate that subscribes to the gate signal
        // and forwards GateChanged messages into the arbiter loop.  This
        // keeps the arbiter loop's select! footprint bounded — the loop
        // only ever reads from a single mpsc.
        for gate in &gates {
            let bus = Arc::clone(&bus);
            let tx = tx.clone();
            let target = gate.target;
            let gate_signal = gate.gate_signal;
            let suppress_when = gate.suppress_when.clone();
            let domain = name;
            tokio::spawn(async move {
                let mut stream = bus.subscribe(gate_signal).await;
                while let Some(value) = stream.next().await {
                    let closed = value == suppress_when;
                    tracing::debug!(domain, target, gate_signal, closed, "physical gate update");
                    if tx
                        .send(ArbiterMsg::GateChanged { target, closed })
                        .await
                        .is_err()
                    {
                        break;
                    }
                }
            });
        }

        let arbiter = Self { name, tx };
        let loop_fut = arbiter_loop(name, allow_list, defaults, bus, rx);

        (arbiter, loop_fut)
    }

    /// Submit an actuator claim. The claim persists until released or replaced
    /// by another `request()` from the same feature on the same signal.
    /// Returns an error only if the arbiter loop has been dropped.
    pub async fn request(&self, req: ActuatorRequest) -> anyhow::Result<()> {
        self.tx
            .send(ArbiterMsg::Request(req))
            .await
            .map_err(|_| anyhow::anyhow!("{}: arbiter channel closed", self.name))
    }

    /// Withdraw this feature's claim on a signal. After release, the next
    /// highest-priority claim wins; if none remain, the signal reverts to the
    /// default-off value.
    pub async fn release(&self, signal: VssPath, feature_id: FeatureId) -> anyhow::Result<()> {
        self.tx
            .send(ArbiterMsg::Release { signal, feature_id })
            .await
            .map_err(|_| anyhow::anyhow!("{}: arbiter channel closed", self.name))
    }
}

// ---------------------------------------------------------------------------
// Arbiter resolution loop
// ---------------------------------------------------------------------------

/// One active claim on a signal, held by a specific feature.
#[derive(Debug, Clone)]
struct Claim {
    priority: Priority,
    value: SignalValue,
    /// Monotonic sequence number — used to tiebreak claims at equal priority
    /// (later claim wins, matching the legacy "equal priority replaces" rule).
    seq: u64,
}

/// Background task that receives requests/releases, tracks per-feature claims,
/// resolves the highest-priority active claim per signal, and publishes the
/// resulting value downstream when it changes.
async fn arbiter_loop<B: SignalBus>(
    name: &'static str,
    allow_list: Vec<AllowEntry>,
    defaults: HashMap<VssPath, SignalValue>,
    bus: Arc<B>,
    mut rx: mpsc::Receiver<ArbiterMsg>,
) {
    // Per-signal active claims, indexed by the claiming feature.
    let mut claims: HashMap<VssPath, HashMap<FeatureId, Claim>> = HashMap::new();
    // Last value published per signal — used to suppress duplicate publishes.
    let mut last_published: HashMap<VssPath, SignalValue> = HashMap::new();
    // Per-target gate state.  Absent or `false` ⇒ gate open (claims pass
    // through normally); `true` ⇒ gate closed (force `Bool(false)`).
    let mut gates_closed: HashMap<VssPath, bool> = HashMap::new();
    let mut next_seq: u64 = 0;

    tracing::info!(domain = name, signals = allow_list.len(), "arbiter started");

    // Seed each configured default on boot so consumers (HMI, plant
    // models) see a defined value before any feature claims.  No-op
    // if `defaults` is empty — preserves the legacy behaviour for
    // arbiters that don't configure a non-Bool default.
    for (sig, val) in defaults.iter() {
        if bus.publish(sig, val.clone()).await.is_ok() {
            last_published.insert(sig, val.clone());
        }
    }

    while let Some(msg) = rx.recv().await {
        match msg {
            ArbiterMsg::Request(req) => {
                // Validate against the allow-list.
                let allowed = allow_list.iter().any(|entry| {
                    entry.feature_id == req.feature_id
                        && entry.signal == req.signal
                        && entry.priority == req.priority
                });

                if !allowed {
                    tracing::warn!(
                        domain = name,
                        feature = ?req.feature_id,
                        signal = req.signal,
                        priority = ?req.priority,
                        "request rejected — not in allow-list"
                    );
                    continue;
                }

                next_seq += 1;
                let claim = Claim {
                    priority: req.priority,
                    value: req.value.clone(),
                    seq: next_seq,
                };

                tracing::debug!(
                    domain = name,
                    feature = ?req.feature_id,
                    signal = req.signal,
                    value = ?req.value,
                    priority = ?req.priority,
                    "arbiter: claim"
                );

                claims
                    .entry(req.signal)
                    .or_default()
                    .insert(req.feature_id, claim);

                publish_resolved(
                    name,
                    req.signal,
                    &claims,
                    &gates_closed,
                    &defaults,
                    &mut last_published,
                    &bus,
                )
                .await;
            }
            ArbiterMsg::Release { signal, feature_id } => {
                let removed = claims
                    .get_mut(signal)
                    .map(|sc| sc.remove(&feature_id).is_some())
                    .unwrap_or(false);

                if removed {
                    tracing::debug!(
                        domain = name,
                        feature = ?feature_id,
                        signal,
                        "arbiter: release"
                    );
                    publish_resolved(
                        name,
                        signal,
                        &claims,
                        &gates_closed,
                        &defaults,
                        &mut last_published,
                        &bus,
                    )
                    .await;
                }
            }
            ArbiterMsg::GateChanged { target, closed } => {
                let prev = gates_closed.insert(target, closed).unwrap_or(false);
                if prev != closed {
                    tracing::info!(
                        domain = name,
                        target,
                        closed,
                        "physical gate state changed — re-resolving target"
                    );
                    publish_resolved(
                        name,
                        target,
                        &claims,
                        &gates_closed,
                        &defaults,
                        &mut last_published,
                        &bus,
                    )
                    .await;
                }
            }
        }
    }

    tracing::info!(domain = name, "arbiter loop ended");
}

/// Resolve the winning value for a signal and publish if it changed.
///
/// Winner: claim with the highest (priority, seq) tuple — i.e. highest priority
/// first, then most-recent claim on a tie. If no claims remain, default to
/// `Bool(false)` (the off-state for boolean actuators).
async fn publish_resolved<B: SignalBus>(
    name: &'static str,
    signal: VssPath,
    claims: &HashMap<VssPath, HashMap<FeatureId, Claim>>,
    gates_closed: &HashMap<VssPath, bool>,
    defaults: &HashMap<VssPath, SignalValue>,
    last_published: &mut HashMap<VssPath, SignalValue>,
    bus: &Arc<B>,
) {
    // Per-signal default — used both when a physical gate forces the
    // target off and when no claims remain.  Falls back to `Bool(false)`
    // (the legacy off-state for boolean actuators).
    let default = defaults
        .get(signal)
        .cloned()
        .unwrap_or(SignalValue::Bool(false));

    // Physical gate forces the target to its default regardless of any
    // active claims.  This models hardware constraints (mirror folded,
    // etc.) that no feature should have to know about.
    let gated = gates_closed.get(signal).copied().unwrap_or(false);

    let resolved = if gated {
        default.clone()
    } else {
        claims
            .get(signal)
            .and_then(|sc| {
                sc.values()
                    .max_by_key(|c| (c.priority as u8, c.seq))
                    .map(|c| c.value.clone())
            })
            .unwrap_or(default)
    };

    let changed = last_published.get(signal) != Some(&resolved);
    if !changed {
        return;
    }

    tracing::debug!(
        domain = name,
        signal,
        value = ?resolved,
        "arbiter: publishing resolved value"
    );

    if let Err(e) = bus.publish(signal, resolved.clone()).await {
        tracing::error!(
            domain = name,
            signal,
            error = %e,
            "failed to publish arbitrated value"
        );
        return;
    }

    last_published.insert(signal, resolved);
}

// ---------------------------------------------------------------------------
// Domain factory functions — static priority tables
// ---------------------------------------------------------------------------

/// Create the Lighting domain arbiter.
///
/// Covers: direction indicators, low/high beam, DRL, hazard signaling.
/// Contention on direction indicators: Hazard(3), LockFeedback(3, overlay), Turn(2).
/// LockFeedback uses HIGH to overlay its brief pattern on hazard/turn, then releases.
pub fn lighting_arbiter<B: SignalBus>(
    bus: Arc<B>,
) -> (DomainArbiter, impl std::future::Future<Output = ()>) {
    let allow_list = vec![
        // Direction indicators — 3-way contention
        AllowEntry {
            feature_id: FeatureId::Hazard,
            signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
            priority: Priority::High,
        },
        AllowEntry {
            feature_id: FeatureId::Hazard,
            signal: "Body.Lights.DirectionIndicator.Right.IsSignaling",
            priority: Priority::High,
        },
        AllowEntry {
            feature_id: FeatureId::TurnIndicator,
            signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
            priority: Priority::Medium,
        },
        AllowEntry {
            feature_id: FeatureId::TurnIndicator,
            signal: "Body.Lights.DirectionIndicator.Right.IsSignaling",
            priority: Priority::Medium,
        },
        // LockFeedback uses HIGH to overlay its brief lock/unlock pattern
        // on top of active hazard or turn signaling, then self-releases.
        AllowEntry {
            feature_id: FeatureId::LockFeedback,
            signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
            priority: Priority::High,
        },
        AllowEntry {
            feature_id: FeatureId::LockFeedback,
            signal: "Body.Lights.DirectionIndicator.Right.IsSignaling",
            priority: Priority::High,
        },
        // PanicAlarm — synchronized blink of both indicators while alarm
        // is active.  Same priority as Hazard (HIGH); the alarm explicitly
        // pre-empts hazard while it runs and releases on disengage so any
        // pending hazard claim resumes.
        AllowEntry {
            feature_id: FeatureId::PanicAlarm,
            signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
            priority: Priority::High,
        },
        AllowEntry {
            feature_id: FeatureId::PanicAlarm,
            signal: "Body.Lights.DirectionIndicator.Right.IsSignaling",
            priority: Priority::High,
        },
        // PerimeterAlarm — same blink pattern, triggered by an
        // intrusion (door open while LOCKED / DOUBLE_LOCKED) instead
        // of a panic-button press.  HIGH for the same reason as
        // PanicAlarm; the two are mutually exclusive in practice
        // (a panic press cancels the perimeter alarm, and the
        // perimeter trigger only fires while the cabin is locked
        // i.e. the user is not present).
        AllowEntry {
            feature_id: FeatureId::PerimeterAlarm,
            signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
            priority: Priority::High,
        },
        AllowEntry {
            feature_id: FeatureId::PerimeterAlarm,
            signal: "Body.Lights.DirectionIndicator.Right.IsSignaling",
            priority: Priority::High,
        },
        // Hazard master signal
        AllowEntry {
            feature_id: FeatureId::Hazard,
            signal: "Body.Lights.Hazard.IsSignaling",
            priority: Priority::High,
        },
        // Low beam
        AllowEntry {
            feature_id: FeatureId::LowBeam,
            signal: "Body.Lights.Beam.Low.IsOn",
            priority: Priority::Medium,
        },
        // High beam
        AllowEntry {
            feature_id: FeatureId::HighBeam,
            signal: "Body.Lights.Beam.High.IsOn",
            priority: Priority::Medium,
        },
        // DRL
        AllowEntry {
            feature_id: FeatureId::Drl,
            signal: "Body.Lights.Running.IsOn",
            priority: Priority::Medium,
        },
    ];

    DomainArbiter::new("Lighting", allow_list, bus)
}

/// Create the Low-Beam domain arbiter.
///
/// Covers: low beam, high beam, DRL, parking lights, license plate lamp.
/// Contention: FollowMeHome (HIGH) overrides ManualLighting (MEDIUM) on
/// low-beam-derived signals during the 45 s post-ignition-off window.
pub fn low_beam_arbiter<B: SignalBus>(
    bus: Arc<B>,
) -> (DomainArbiter, impl std::future::Future<Output = ()>) {
    let allow_list = vec![
        // ManualLighting — switch-driven outputs at medium priority.
        AllowEntry {
            feature_id: FeatureId::LowBeam,
            signal: "Body.Lights.Beam.Low.IsOn",
            priority: Priority::Medium,
        },
        AllowEntry {
            feature_id: FeatureId::LowBeam,
            signal: "Body.Lights.Parking.IsOn",
            priority: Priority::Medium,
        },
        AllowEntry {
            feature_id: FeatureId::LowBeam,
            signal: "Body.Lights.LicensePlate.IsOn",
            priority: Priority::Medium,
        },
        AllowEntry {
            feature_id: FeatureId::HighBeam,
            signal: "Body.Lights.Beam.High.IsOn",
            priority: Priority::Medium,
        },
        AllowEntry {
            feature_id: FeatureId::Drl,
            signal: "Body.Lights.Running.IsOn",
            priority: Priority::Medium,
        },
        // FollowMeHome — high priority so FMH wins even if ManualLighting
        // has a residual claim (e.g. driver switches to BEAM with ignition off).
        AllowEntry {
            feature_id: FeatureId::FollowMeHome,
            signal: "Body.Lights.Beam.Low.IsOn",
            priority: Priority::High,
        },
        AllowEntry {
            feature_id: FeatureId::FollowMeHome,
            signal: "Body.Lights.Parking.IsOn",
            priority: Priority::High,
        },
        AllowEntry {
            feature_id: FeatureId::FollowMeHome,
            signal: "Body.Lights.LicensePlate.IsOn",
            priority: Priority::High,
        },
        // AutoHighBeam — ADAS camera suppresses high beam at high priority.
        // Bool(false) at High overrides ManualLighting's Bool(true) at Medium,
        // ensuring oncoming vehicles are not blinded regardless of stalk position.
        AllowEntry {
            feature_id: FeatureId::AutoHighBeam,
            signal: "Body.Lights.Beam.High.IsOn",
            priority: Priority::High,
        },
    ];

    DomainArbiter::new("LowBeam", allow_list, bus)
}

// ---------------------------------------------------------------------------
// DoorLockArbiter — serialized command queue with ACK handshake
// ---------------------------------------------------------------------------

/// Lock command type — what the feature is requesting.
///
/// These map directly to the four high-level intents sent to the
/// Classic AUTOSAR Locking SWC (M7).  The plant model / SWC owns all
/// per-door actuator logic from here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LockCommand {
    /// Unlock the driver door only (two-stage unlock, stage 1).
    UnlockDriver,
    /// Unlock all doors (normal unlock or two-stage stage 2).
    UnlockAll,
    /// Lock all doors.
    LockAll,
    /// Superlock (double-lock) all doors.
    DoubleLockAll,
    /// Clear double-lock on all doors without changing IsLocked.
    /// Dispatched by DoubleLockRelease when ignition turns ON while double-locked.
    /// Does NOT produce a FeedbackRequest (internal trigger).
    ReleaseDouble,
}

/// A door-lock request submitted by a feature module.
#[derive(Debug, Clone)]
pub struct DoorLockRequest {
    pub command: LockCommand,
    pub feature_id: FeatureId,
}

/// Acknowledgement from the Classic AUTOSAR Locking SWC.
///
/// Published via IPC after each motor operation completes (~300 ms).
/// The event number, requestor, and per-door status are carried here
/// for the arbiter's queue management. NVM persistence and DTC
/// reporting are the Classic AUTOSAR SWC's responsibility.
#[derive(Debug, Clone)]
pub struct LockAck {
    pub event_number: u32,
    /// Per-door success/failure. `true` = operation succeeded for that door.
    pub door_results: [bool; 4], // Row1L, Row1R, Row2L, Row2R
}

/// Allow-list entry for the DoorLock arbiter.
/// Only feature_id is checked — the DoorLock arbiter does not use
/// per-signal priority resolution (it uses queue serialization instead).
#[derive(Debug, Clone)]
pub struct DoorLockAllowEntry {
    pub feature_id: FeatureId,
}

/// Serialized command-queue arbiter for door locks.
///
/// Unlike the Lighting arbiter (instant priority resolution), the DoorLock
/// arbiter manages a one-deep queue because the lock motor takes ~300 ms
/// to complete and cannot accept concurrent commands.
///
/// Queue rules:
/// - If idle: dispatch immediately (request becomes active).
/// - If active: new request goes to pending slot (replaces previous pending).
/// - When ACK received: promote pending to active and dispatch.
/// - CrashUnlock exception: cannot be replaced in queue; triggers 10s lockout.
pub struct DoorLockArbiter {
    cmd_tx: mpsc::Sender<DoorLockMsg>,
}

/// Internal messages to the arbiter loop.
enum DoorLockMsg {
    Request(DoorLockRequest),
    Ack(LockAck),
}

impl DoorLockArbiter {
    /// Create a new DoorLock arbiter.
    ///
    /// Returns the arbiter handle and a future to spawn, plus an ACK sender
    /// that the IPC layer feeds when the Classic AUTOSAR Locking SWC reports
    /// completion.
    pub fn new<B: SignalBus>(
        allow_list: Vec<DoorLockAllowEntry>,
        bus: Arc<B>,
    ) -> (
        Self,
        mpsc::Sender<LockAck>,
        impl std::future::Future<Output = ()>,
    ) {
        Self::new_with_nvm(allow_list, bus, None)
    }

    /// Like `new`, but with an `NvmStore` for persisting `Cabin.LockStatus`
    /// across power cycles.  Use this in production wiring; tests can use
    /// `new` to get a transient arbiter.
    pub fn new_with_nvm<B: SignalBus>(
        allow_list: Vec<DoorLockAllowEntry>,
        bus: Arc<B>,
        nvm: Option<crate::nvm::NvmStore>,
    ) -> (
        Self,
        mpsc::Sender<LockAck>,
        impl std::future::Future<Output = ()>,
    ) {
        let (cmd_tx, cmd_rx) = mpsc::channel::<DoorLockMsg>(64);
        let ack_tx = {
            let cmd_tx_clone = cmd_tx.clone();
            let (ack_tx, mut ack_rx) = mpsc::channel::<LockAck>(16);

            // Forward ACKs into the unified command channel
            tokio::spawn(async move {
                while let Some(ack) = ack_rx.recv().await {
                    if cmd_tx_clone.send(DoorLockMsg::Ack(ack)).await.is_err() {
                        break;
                    }
                }
            });

            ack_tx
        };

        let arbiter = Self {
            cmd_tx: cmd_tx.clone(),
        };
        let loop_fut = door_lock_loop(allow_list, bus, cmd_rx, nvm);

        (arbiter, ack_tx, loop_fut)
    }

    /// Submit a lock/unlock/double-lock request.
    pub async fn request(&self, req: DoorLockRequest) -> anyhow::Result<()> {
        self.cmd_tx
            .send(DoorLockMsg::Request(req))
            .await
            .map_err(|_| anyhow::anyhow!("DoorLock: arbiter channel closed"))
    }
}

/// Background loop for the DoorLock arbiter.
async fn door_lock_loop<B: SignalBus>(
    allow_list: Vec<DoorLockAllowEntry>,
    bus: Arc<B>,
    mut rx: mpsc::Receiver<DoorLockMsg>,
    nvm: Option<crate::nvm::NvmStore>,
) {
    // Queue state
    let mut active: Option<DoorLockRequest> = None;
    let mut pending: Option<DoorLockRequest> = None;
    let mut crash_lockout_until: Option<tokio::time::Instant> = None;

    // Boot-time republish of persisted `Cabin.LockStatus`.  Subscribers
    // (MirrorFold AUTO, future security features) need to see this on
    // boot; a fresh broadcast subscription would otherwise wait for
    // the next command before getting any value.
    let mut last_status: String = if let Some(ref nvm) = nvm {
        let st = nvm.load_cabin_lock_status();
        if let Err(e) = bus
            .publish(CABIN_LOCK_STATUS, SignalValue::String(st.status.clone()))
            .await
        {
            tracing::warn!(error = %e, "DoorLock: failed to republish CabinLockStatus on boot");
        } else {
            tracing::info!(status = %st.status, "DoorLock: restored CabinLockStatus from NVM");
        }
        st.status
    } else {
        // No NVM (test wiring) — start from "UNLOCKED" but don't publish
        // until a command arrives.
        "UNLOCKED".into()
    };

    // Monotonic counter for `Cabin.LockStatus.EventNum`.  The value
    // `0` is reserved as the "publisher reset" sentinel — published
    // exactly once at boot, never again.  Real events count `1, 2, …,
    // u16::MAX, 1, 2, …` (wrap skips `0`) so subscribers can rely on
    // `0` meaning "the arbiter just (re)started, no events yet."
    let mut event_num: u16 = 0;
    if let Err(e) = bus
        .publish(CABIN_LOCK_EVENT_NUM, SignalValue::Uint16(0))
        .await
    {
        tracing::warn!(error = %e, "DoorLock: failed to publish boot EventNum=0 sentinel");
    }

    tracing::info!("DoorLock arbiter started");

    while let Some(msg) = rx.recv().await {
        match msg {
            DoorLockMsg::Request(req) => {
                // 1. Validate against allow-list
                let allowed = allow_list
                    .iter()
                    .any(|entry| entry.feature_id == req.feature_id);

                if !allowed {
                    tracing::warn!(
                        feature = ?req.feature_id,
                        "DoorLock: request rejected — not in allow-list"
                    );
                    continue;
                }

                // 2. Check crash lockout
                if let Some(lockout_end) = crash_lockout_until {
                    if tokio::time::Instant::now() < lockout_end {
                        tracing::warn!(
                            feature = ?req.feature_id,
                            command = ?req.command,
                            "DoorLock: request rejected — crash lockout active"
                        );
                        continue;
                    } else {
                        // Lockout expired
                        crash_lockout_until = None;
                    }
                }

                if active.is_none() {
                    // 3a. Idle — dispatch immediately
                    tracing::info!(
                        feature = ?req.feature_id,
                        command = ?req.command,
                        "DoorLock: dispatching immediately (idle)"
                    );
                    dispatch_lock_command(&req, &bus, &nvm, &mut last_status, &mut event_num).await;

                    // Start crash lockout if this is a CrashUnlock
                    if req.feature_id == FeatureId::CrashUnlock {
                        crash_lockout_until =
                            Some(tokio::time::Instant::now() + std::time::Duration::from_secs(10));
                    }

                    active = Some(req);
                } else {
                    // 3b. Active operation in progress — queue it
                    // Check if pending is a crash unlock (cannot be replaced)
                    if let Some(ref p) = pending {
                        if p.feature_id == FeatureId::CrashUnlock {
                            tracing::warn!(
                                feature = ?req.feature_id,
                                "DoorLock: request rejected — crash unlock pending, cannot replace"
                            );
                            continue;
                        }
                    }

                    tracing::info!(
                        feature = ?req.feature_id,
                        command = ?req.command,
                        replaced = pending.as_ref().map(|p| format!("{:?}", p.feature_id)),
                        "DoorLock: queued as pending"
                    );
                    pending = Some(req);
                }
            }

            DoorLockMsg::Ack(ack) => {
                let completed = active.take();
                tracing::info!(
                    event = ack.event_number,
                    feature = ?completed.as_ref().map(|r| r.feature_id),
                    doors_ok = ?ack.door_results,
                    "DoorLock: operation complete"
                );

                // Promote pending to active
                if let Some(next) = pending.take() {
                    tracing::info!(
                        feature = ?next.feature_id,
                        command = ?next.command,
                        "DoorLock: promoting pending to active"
                    );
                    dispatch_lock_command(&next, &bus, &nvm, &mut last_status, &mut event_num)
                        .await;

                    if next.feature_id == FeatureId::CrashUnlock {
                        crash_lockout_until =
                            Some(tokio::time::Instant::now() + std::time::Duration::from_secs(10));
                    }

                    active = Some(next);
                }
            }
        }
    }

    tracing::info!("DoorLock arbiter loop ended");
}

/// Single command signal written by the arbiter to the M7 Locking SWC.
///
/// The value is one of: `"unlock_driver"`, `"unlock_all"`, `"lock_all"`,
/// `"lock_double"`, `"release_double"`.
/// The `DoorLockPlantModel` (M7 actuator simulator) subscribes to this signal
/// and handles all per-door state updates — `IsLocked`, `IsDoubleLocked`,
/// `Soldier.IsUnlocked` — from here.
pub const CENTRAL_LOCK_CMD: VssPath = "Body.Doors.CentralLock.Command";

/// Signal published by external-origin features to request a visual lock/unlock
/// confirmation flash on both direction indicators.
///
/// Published by: RKE, WalkAwayLock, ThumbPadLock, AutoRelock.
/// Subscribed by: LockFeedback.
///
/// Values:
/// - `"lock"` — one flash unit (100 ms OFF lead-in + 900 ms ON)
/// - `"unlock"` — two flash units with a 300 ms gap
/// - `"trunk_unlock"` — two flash units + arms trunk-close lock feedback
pub const FEEDBACK_REQUEST: VssPath = "Body.Doors.CentralLock.FeedbackRequest";

/// Dispatch a lock command to the SignalBus as a single high-level token.
/// Vehicle-level central lock status — published by the door-lock
/// arbiter on every accepted command.  Many features subscribe.
pub const CABIN_LOCK_STATUS: VssPath = "Cabin.LockStatus";

/// Companion signal: identity of the feature that requested the most
/// recent accepted central-lock command.  String form of `FeatureId`
/// (e.g. "KeyfobRke", "PassiveEntry").  Published in lockstep with
/// every `EVENT_NUM` bump so subscribers can filter on requestor.
pub const CABIN_LOCK_LAST_REQUESTOR: VssPath = "Cabin.LockStatus.LastRequestor";

/// Companion signal: monotonic counter incremented on every accepted
/// central-lock command (wraps at u16::MAX).  Subscribers use the
/// *change* in this value as the "a new lock command happened"
/// trigger, even when the resolved `Cabin.LockStatus` enum value is
/// unchanged.
pub const CABIN_LOCK_EVENT_NUM: VssPath = "Cabin.LockStatus.EventNum";

/// Map a `LockCommand` to its `Cabin.LockStatus` enum value.
fn lock_status_for(cmd: LockCommand) -> &'static str {
    match cmd {
        LockCommand::UnlockAll => "UNLOCKED",
        LockCommand::UnlockDriver => "DRIVER_UNLOCKED",
        LockCommand::LockAll => "LOCKED",
        LockCommand::DoubleLockAll => "DOUBLE_LOCKED",
        // Demoting double-lock returns the vehicle to plain LOCKED.
        LockCommand::ReleaseDouble => "LOCKED",
    }
}

async fn dispatch_lock_command<B: SignalBus>(
    req: &DoorLockRequest,
    bus: &Arc<B>,
    nvm: &Option<crate::nvm::NvmStore>,
    last_status: &mut String,
    event_num: &mut u16,
) {
    let token = match req.command {
        LockCommand::UnlockDriver => "unlock_driver",
        LockCommand::UnlockAll => "unlock_all",
        LockCommand::LockAll => "lock_all",
        LockCommand::DoubleLockAll => "lock_double",
        LockCommand::ReleaseDouble => "release_double",
    };
    if let Err(e) = bus
        .publish(CENTRAL_LOCK_CMD, SignalValue::String(token.into()))
        .await
    {
        tracing::error!(token, error = %e, "DoorLock: failed to dispatch command");
    }

    // Vehicle-level lock status — published on EVERY accepted
    // command (no dedup).  Subscribers that care about *changes*
    // (MirrorFold AUTO triggers) compare against their own
    // `last_lock_status` cache.  AutoRelock relies on consecutive
    // publishes to detect repeated unlock presses.  NVM still only
    // writes on actual transitions to avoid disk thrash.
    let new_status = lock_status_for(req.command);
    if let Err(e) = bus
        .publish(CABIN_LOCK_STATUS, SignalValue::String(new_status.into()))
        .await
    {
        tracing::error!(status = new_status, error = %e, "DoorLock: failed to publish CabinLockStatus");
    }
    if new_status != last_status.as_str() {
        *last_status = new_status.into();
        if let Some(nvm) = nvm {
            nvm.save_cabin_lock_status(&crate::nvm::CabinLockStatusState {
                status: new_status.into(),
            });
        }
    }

    // Requestor + EventNum — also published on every accepted command.
    // Together with LockStatus they form a per-event tuple AutoRelock
    // uses to decide arming.
    let requestor = req.feature_id.to_string();
    if let Err(e) = bus
        .publish(
            CABIN_LOCK_LAST_REQUESTOR,
            SignalValue::String(requestor.clone()),
        )
        .await
    {
        tracing::error!(requestor = %requestor, error = %e, "DoorLock: failed to publish LastRequestor");
    }
    // Bump, skipping `0` on wrap so subscribers can treat `0` as the
    // boot-sentinel exclusively.
    *event_num = if *event_num == u16::MAX {
        1
    } else {
        *event_num + 1
    };
    if let Err(e) = bus
        .publish(CABIN_LOCK_EVENT_NUM, SignalValue::Uint16(*event_num))
        .await
    {
        tracing::error!(event_num = *event_num, error = %e, "DoorLock: failed to publish EventNum");
    }
}

/// Allow-list for the DoorLock arbiter.  Extracted into a function
/// so `door_lock_arbiter` and `door_lock_arbiter_with_nvm` share it.
fn door_lock_allow_list() -> Vec<DoorLockAllowEntry> {
    vec![
        DoorLockAllowEntry {
            feature_id: FeatureId::KeyfobPeps,
        },
        DoorLockAllowEntry {
            feature_id: FeatureId::AutoLock,
        },
        DoorLockAllowEntry {
            feature_id: FeatureId::DoorTrimButton,
        },
        DoorLockAllowEntry {
            feature_id: FeatureId::KeyfobRke,
        },
        DoorLockAllowEntry {
            feature_id: FeatureId::PhoneApp,
        },
        DoorLockAllowEntry {
            feature_id: FeatureId::PhoneBle,
        },
        DoorLockAllowEntry {
            feature_id: FeatureId::NfcCard,
        },
        DoorLockAllowEntry {
            feature_id: FeatureId::NfcPhone,
        },
        DoorLockAllowEntry {
            feature_id: FeatureId::AutoRelock,
        },
        DoorLockAllowEntry {
            feature_id: FeatureId::CrashUnlock,
        },
        DoorLockAllowEntry {
            feature_id: FeatureId::WalkAwayLock,
        },
        DoorLockAllowEntry {
            feature_id: FeatureId::ThumbPadLock,
        },
        DoorLockAllowEntry {
            feature_id: FeatureId::DoubleLockRelease,
        },
        DoorLockAllowEntry {
            feature_id: FeatureId::PassiveEntry,
        },
        DoorLockAllowEntry {
            feature_id: FeatureId::SlamLock,
        },
    ]
}

/// Create the DoorLock arbiter with all authorized lock requestors —
/// transient (no NVM).  For tests / scenarios where `Cabin.LockStatus`
/// persistence is not required.
pub fn door_lock_arbiter<B: SignalBus>(
    bus: Arc<B>,
) -> (
    DoorLockArbiter,
    mpsc::Sender<LockAck>,
    impl std::future::Future<Output = ()>,
) {
    DoorLockArbiter::new(door_lock_allow_list(), bus)
}

/// Production variant of `door_lock_arbiter` — persists `Cabin.LockStatus`
/// across power cycles via the supplied `NvmStore`.
pub fn door_lock_arbiter_with_nvm<B: SignalBus>(
    bus: Arc<B>,
    nvm: crate::nvm::NvmStore,
) -> (
    DoorLockArbiter,
    mpsc::Sender<LockAck>,
    impl std::future::Future<Output = ()>,
) {
    DoorLockArbiter::new_with_nvm(door_lock_allow_list(), bus, Some(nvm))
}

/// Create the Horn domain arbiter.
///
/// Writers, in priority order:
///
/// | Feature      | Priority | When                                       |
/// |--------------|----------|--------------------------------------------|
/// | `PanicAlarm` | High     | Pulsing in sync with the indicator blink   |
/// | `ManualHorn` | Medium   | Driver pressing the steering-wheel pad     |
///
/// `High` over `Medium` means a panic-alarm pulse can override a
/// driver press — desired behaviour: the alarm pattern wins.  When
/// the alarm releases, the driver's claim resumes if still held.
/// Future writers (anti-theft chirp, lock-feedback chirp) slot in
/// at `Medium` or `Low` as appropriate.
pub fn horn_arbiter<B: SignalBus>(
    bus: Arc<B>,
) -> (DomainArbiter, impl std::future::Future<Output = ()>) {
    let allow_list = vec![
        AllowEntry {
            feature_id: FeatureId::PanicAlarm,
            signal: "Body.Horn.IsActive",
            priority: Priority::High,
        },
        AllowEntry {
            feature_id: FeatureId::PerimeterAlarm,
            signal: "Body.Horn.IsActive",
            priority: Priority::High,
        },
        AllowEntry {
            feature_id: FeatureId::ManualHorn,
            signal: "Body.Horn.IsActive",
            priority: Priority::Medium,
        },
    ];

    DomainArbiter::new("Horn", allow_list, bus)
}

/// Create the Comfort domain arbiter.
///
/// Covers: seat heating/ventilation, HVAC, cabin lights, sunroof.
/// No contention today — pass-through with validation. Adding a
/// second feature to any comfort actuator requires only an allow entry here.
pub fn comfort_arbiter<B: SignalBus>(
    bus: Arc<B>,
) -> (DomainArbiter, impl std::future::Future<Output = ()>) {
    // No competing features today — pass-through with validation.
    let allow_list = vec![];

    DomainArbiter::new("Comfort", allow_list, bus)
}

/// Create the Courtesy domain arbiter.
///
/// Today this covers the **interior dome light** only.  Exterior
/// puddle lamps moved to their own dedicated `puddle_arbiter` because
/// they're a distinct contention surface (Welcome today; Farewell,
/// PerimeterAlarm, future "puddle on door open" all want them).
///
/// Adding a new shared interior-courtesy actuator (cabin ambient,
/// glove-box, vanity mirror lamps) means just adding an allow entry
/// here.
pub fn courtesy_arbiter<B: SignalBus>(
    bus: Arc<B>,
) -> (DomainArbiter, impl std::future::Future<Output = ()>) {
    let allow_list = vec![
        AllowEntry {
            feature_id: FeatureId::Welcome,
            signal: "Cabin.Lights.IsDomeOn",
            priority: Priority::Medium,
        },
        AllowEntry {
            feature_id: FeatureId::Farewell,
            signal: "Cabin.Lights.IsDomeOn",
            priority: Priority::Medium,
        },
        // PerimeterAlarm pulses the dome at HIGH while the alarm is
        // active.  Pre-empts Welcome / Farewell cleanly; on disarm
        // the next-highest pending claim resumes.
        AllowEntry {
            feature_id: FeatureId::PerimeterAlarm,
            signal: "Cabin.Lights.IsDomeOn",
            priority: Priority::High,
        },
        // DomeSwitch owns the default ("user intent") claim at LOW so
        // Welcome / Farewell / PerimeterAlarm naturally pre-empt for
        // their courtesy sequences.  When no higher claimant is
        // active, this dictates the lamp state from the OFF/DOOR/ON
        // switch + any-door-open inputs.
        AllowEntry {
            feature_id: FeatureId::DomeSwitch,
            signal: "Cabin.Lights.IsDomeOn",
            priority: Priority::Low,
        },
    ];

    DomainArbiter::new("Courtesy", allow_list, bus)
}

/// Create the Puddle domain arbiter.
///
/// Dedicated arbiter for the under-mirror exterior puddle lamps
/// (`Body.Lights.Puddle.{Left,Right}.IsOn`).  Multiple features will
/// want to claim these:
///
/// | Feature | When | Priority |
/// |---|---|---|
/// | **Welcome** (today) | Any paired PEPS device enters LF coverage | MEDIUM |
/// | **Farewell** (planned) | Driver opens door after ignition OFF | MEDIUM |
/// | **DoorOpenAssist** (planned) | Any door opens at night | LOW |
/// | **PerimeterAlarm** (planned) | Intrusion event — pulse pattern as attention-grabber | HIGH |
///
/// Splitting puddle onto its own arbiter (rather than rolling into
/// `courtesy_arbiter`) keeps the contention surface explicit so each
/// future feature can pick the right priority without a global
/// renumbering, and so a future security claim can pre-empt courtesy
/// claims cleanly.
pub fn puddle_arbiter<B: SignalBus>(
    bus: Arc<B>,
) -> (DomainArbiter, impl std::future::Future<Output = ()>) {
    let allow_list = vec![
        AllowEntry {
            feature_id: FeatureId::Welcome,
            signal: "Body.Lights.Puddle.Left.IsOn",
            priority: Priority::Medium,
        },
        AllowEntry {
            feature_id: FeatureId::Welcome,
            signal: "Body.Lights.Puddle.Right.IsOn",
            priority: Priority::Medium,
        },
        AllowEntry {
            feature_id: FeatureId::Farewell,
            signal: "Body.Lights.Puddle.Left.IsOn",
            priority: Priority::Medium,
        },
        AllowEntry {
            feature_id: FeatureId::Farewell,
            signal: "Body.Lights.Puddle.Right.IsOn",
            priority: Priority::Medium,
        },
        AllowEntry {
            feature_id: FeatureId::DoorOpenAssist,
            signal: "Body.Lights.Puddle.Left.IsOn",
            priority: Priority::Low,
        },
        AllowEntry {
            feature_id: FeatureId::DoorOpenAssist,
            signal: "Body.Lights.Puddle.Right.IsOn",
            priority: Priority::Low,
        },
        // PerimeterAlarm — visible attention-grabbing pulse on the
        // exterior puddle lamps for the whole 5 min alarm window.
        // HIGH pre-empts Welcome / Farewell / DoorOpenAssist.
        AllowEntry {
            feature_id: FeatureId::PerimeterAlarm,
            signal: "Body.Lights.Puddle.Left.IsOn",
            priority: Priority::High,
        },
        AllowEntry {
            feature_id: FeatureId::PerimeterAlarm,
            signal: "Body.Lights.Puddle.Right.IsOn",
            priority: Priority::High,
        },
    ];

    // Physical gates: the puddle lamp is *inside* the side mirror
    // housing.  When the mirror is folded the lamp would project into
    // the door skin, so the arbiter forces it off regardless of which
    // feature is currently winning.  This belongs at the actuator
    // layer, not in any individual feature — Welcome, Farewell,
    // PerimeterAlarm, and any future puddle claimant all benefit
    // automatically.
    let gates = vec![
        PhysicalGate {
            target: "Body.Lights.Puddle.Left.IsOn",
            gate_signal: "Body.Mirror.Left.IsFolded",
            suppress_when: SignalValue::Bool(true),
        },
        PhysicalGate {
            target: "Body.Lights.Puddle.Right.IsOn",
            gate_signal: "Body.Mirror.Right.IsFolded",
            suppress_when: SignalValue::Bool(true),
        },
    ];

    DomainArbiter::new_with_gates("Puddle", allow_list, gates, bus)
}

/// Trunk-open command signal — neutral on pop vs power-open so future
/// power-liftgate plant models can replace the simple pop-latch
/// `TrunkPlantModel` without any feature-side rename.
pub const TRUNK_OPEN_CMD: VssPath = "Body.Trunk.OpenCmd";

/// Create the Trunk domain arbiter.
///
/// All trunk-open writers route through here so a single
/// `Cabin.ValetMode.IsActive` gate suppresses every path uniformly:
/// RKE TrunkRelease, ExteriorTrunkButton (both unlocked-direct and
/// PassiveEntry-authenticated paths), future phone-app trunk-open,
/// future hands-free liftgate kick-sensor.  Cabin doors are unaffected
/// — valet drives the car with cabin access, just not trunk/glovebox.
///
/// Priority is uniform `Medium` because there is no contention between
/// these features today; they are alternative trigger sources for the
/// same momentary `Body.Trunk.OpenCmd` edge.  The `PhysicalGate` is
/// what does the policy work.
pub fn trunk_arbiter<B: SignalBus>(
    bus: Arc<B>,
) -> (DomainArbiter, impl std::future::Future<Output = ()>) {
    let allow_list = vec![
        AllowEntry {
            feature_id: FeatureId::KeyfobRke,
            signal: TRUNK_OPEN_CMD,
            priority: Priority::Medium,
        },
        AllowEntry {
            feature_id: FeatureId::ExteriorTrunkButton,
            signal: TRUNK_OPEN_CMD,
            priority: Priority::Medium,
        },
        AllowEntry {
            feature_id: FeatureId::PassiveEntry,
            signal: TRUNK_OPEN_CMD,
            priority: Priority::Medium,
        },
        AllowEntry {
            feature_id: FeatureId::CabinTrunkRelease,
            signal: TRUNK_OPEN_CMD,
            priority: Priority::Medium,
        },
    ];

    // Valet-mode gate: when `Cabin.ValetMode.IsActive` is true, force
    // `Body.Trunk.OpenCmd` to false regardless of which feature is
    // claiming.  Same `PhysicalGate` pattern as the puddle arbiter's
    // mirror-fold suppression — the gate models a *policy* constraint
    // (valet should not access the trunk) at the actuator boundary so
    // every trunk-open writer inherits it for free.
    let gates = vec![PhysicalGate {
        target: TRUNK_OPEN_CMD,
        gate_signal: "Cabin.ValetMode.IsActive",
        suppress_when: SignalValue::Bool(true),
    }];

    DomainArbiter::new_with_gates("Trunk", allow_list, gates, bus)
}

/// Create the Window domain arbiter.
///
/// Resolves contention between several sources that all want to drive
/// a window's commanded motor direction
/// (`Body.Doors.Row{1,2}.{Left,Right}.Window.MotorDirection` — String
/// enum UP / DOWN / STOPPED).  Priority order from highest to lowest:
///
/// | Source | FeatureId | Priority | Built today? |
/// |---|---|---|---|
/// | Anti-pinch | `WindowAntiPinch` | Critical | reserved — comment |
/// | Security override | `WindowSecurityOverride` | High | reserved — comment |
/// | Driver master + local (combined) | `PowerWindow` | Medium | **yes** |
/// | Global (RKE / phone vent) | `WindowGlobalRemote` | Low | reserved — comment |
///
/// The `PowerWindow` feature handles cross-source conflict
/// **internally**: when both the driver-master and local switches
/// have active intent on the same window, it stops the motor and
/// puts both sources into AwaitingRelease.  That means only one
/// claim per window ever reaches the arbiter from PowerWindow.
/// Anti-pinch (Critical) and a security override (High) will
/// pre-empt that single claim; RKE / phone-app global requests
/// (Low) yield to any occupant switch.
pub fn window_arbiter<B: SignalBus>(
    bus: Arc<B>,
) -> (DomainArbiter, impl std::future::Future<Output = ()>) {
    const MOTOR_SIGNALS: [&str; 4] = [
        "Body.Doors.Row1.Left.Window.MotorDirection",
        "Body.Doors.Row1.Right.Window.MotorDirection",
        "Body.Doors.Row2.Left.Window.MotorDirection",
        "Body.Doors.Row2.Right.Window.MotorDirection",
    ];

    let mut allow_list = Vec::with_capacity(4);
    for &sig in MOTOR_SIGNALS.iter() {
        allow_list.push(AllowEntry {
            feature_id: FeatureId::PowerWindow,
            signal: sig,
            priority: Priority::Medium,
        });
        // ---- Reserved slots for future participants ----
        // AllowEntry {
        //     feature_id: FeatureId::WindowAntiPinch,
        //     signal: sig,
        //     priority: Priority::Critical,
        // },
        // AllowEntry {
        //     feature_id: FeatureId::WindowSecurityOverride,
        //     signal: sig,
        //     priority: Priority::High,
        // },
        // AllowEntry {
        //     feature_id: FeatureId::WindowGlobalRemote,
        //     signal: sig,
        //     priority: Priority::Low,
        // },
    }

    // Per-signal default: STOPPED.  When no feature is claiming a
    // window's motor, the arbiter publishes STOPPED so the plant
    // holds the window in place.
    let mut defaults: HashMap<VssPath, SignalValue> = HashMap::new();
    for &sig in MOTOR_SIGNALS.iter() {
        defaults.insert(sig, SignalValue::String("STOPPED".into()));
    }

    DomainArbiter::new_with_gates_and_defaults("Window", allow_list, Vec::new(), defaults, bus)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;

    /// Helper: spawn a lighting arbiter on MockBus, return the handle.
    async fn setup_lighting() -> (DomainArbiter, Arc<MockBus>) {
        let bus = Arc::new(MockBus::new());
        let (arbiter, loop_fut) = lighting_arbiter(Arc::clone(&bus));
        tokio::spawn(loop_fut);
        // Give the loop a moment to start
        tokio::task::yield_now().await;
        (arbiter, bus)
    }

    #[tokio::test]
    async fn high_priority_wins_over_medium() {
        let (arbiter, bus) = setup_lighting().await;

        // Turn (medium) requests left indicator ON
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
                value: SignalValue::Bool(true),
                priority: Priority::Medium,
                feature_id: FeatureId::TurnIndicator,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // Hazard (high) requests left indicator OFF — should win
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
                value: SignalValue::Bool(false),
                priority: Priority::High,
                feature_id: FeatureId::Hazard,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        let history = bus.history();
        assert_eq!(history.len(), 2);
        // First: Turn's true, Second: Hazard's false (overwrites)
        assert_eq!(
            history[0],
            (
                "Body.Lights.DirectionIndicator.Left.IsSignaling",
                SignalValue::Bool(true)
            )
        );
        assert_eq!(
            history[1],
            (
                "Body.Lights.DirectionIndicator.Left.IsSignaling",
                SignalValue::Bool(false)
            )
        );
    }

    #[tokio::test]
    async fn medium_priority_suppressed_by_existing_high() {
        let (arbiter, bus) = setup_lighting().await;

        // Hazard (high) claims left indicator
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
                value: SignalValue::Bool(true),
                priority: Priority::High,
                feature_id: FeatureId::Hazard,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // Turn (medium) tries the same signal — should be suppressed
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
                value: SignalValue::Bool(false),
                priority: Priority::Medium,
                feature_id: FeatureId::TurnIndicator,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        let history = bus.history();
        // Only Hazard's request was published
        assert_eq!(history.len(), 1);
        assert_eq!(
            history[0],
            (
                "Body.Lights.DirectionIndicator.Left.IsSignaling",
                SignalValue::Bool(true)
            )
        );
    }

    #[tokio::test]
    async fn lock_feedback_overlays_on_active_hazard() {
        let (arbiter, bus) = setup_lighting().await;

        // Hazard (high) claims left indicator ON
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
                value: SignalValue::Bool(true),
                priority: Priority::High,
                feature_id: FeatureId::Hazard,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // LockFeedback (high, overlay) takes over — should publish (equal priority wins)
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
                value: SignalValue::Bool(false),
                priority: Priority::High,
                feature_id: FeatureId::LockFeedback,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        let history = bus.history();
        // Both published: Hazard ON, then LockFeedback OFF (overlay)
        assert_eq!(history.len(), 2);
        assert_eq!(history[0].1, SignalValue::Bool(true)); // Hazard
        assert_eq!(history[1].1, SignalValue::Bool(false)); // LockFeedback overlay
    }

    #[tokio::test]
    async fn different_signals_do_not_interfere() {
        let (arbiter, bus) = setup_lighting().await;

        // LowBeam claims low beam
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.Beam.Low.IsOn",
                value: SignalValue::Bool(true),
                priority: Priority::Medium,
                feature_id: FeatureId::LowBeam,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // HighBeam claims high beam — independent signal, both should publish
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.Beam.High.IsOn",
                value: SignalValue::Bool(true),
                priority: Priority::Medium,
                feature_id: FeatureId::HighBeam,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        let history = bus.history();
        assert_eq!(history.len(), 2);
        assert_eq!(
            history[0],
            ("Body.Lights.Beam.Low.IsOn", SignalValue::Bool(true))
        );
        assert_eq!(
            history[1],
            ("Body.Lights.Beam.High.IsOn", SignalValue::Bool(true))
        );
    }

    #[tokio::test]
    async fn equal_priority_latest_wins() {
        let (arbiter, bus) = setup_lighting().await;

        // Turn (medium) requests left ON
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
                value: SignalValue::Bool(true),
                priority: Priority::Medium,
                feature_id: FeatureId::TurnIndicator,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // Turn (medium) requests left OFF — same priority, should replace
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
                value: SignalValue::Bool(false),
                priority: Priority::Medium,
                feature_id: FeatureId::TurnIndicator,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        let history = bus.history();
        assert_eq!(history.len(), 2);
        assert_eq!(history[1].1, SignalValue::Bool(false));
    }

    #[tokio::test]
    async fn request_rejected_if_not_in_allow_list() {
        let (arbiter, bus) = setup_lighting().await;

        // AutoLock tries to control a lighting signal — not allowed
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.Beam.Low.IsOn",
                value: SignalValue::Bool(true),
                priority: Priority::Medium,
                feature_id: FeatureId::AutoLock,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        let history = bus.history();
        assert_eq!(history.len(), 0, "rejected request should not publish");
    }

    #[tokio::test]
    async fn wrong_priority_rejected() {
        let (arbiter, bus) = setup_lighting().await;

        // TurnIndicator tries to claim HIGH priority — table says MEDIUM
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
                value: SignalValue::Bool(true),
                priority: Priority::High,
                feature_id: FeatureId::TurnIndicator,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        let history = bus.history();
        assert_eq!(history.len(), 0, "wrong priority should be rejected");
    }

    // -----------------------------------------------------------------------
    // Claim / release semantics (regression tests for the bug where releasing
    // a HIGH claim by publishing Bool(false) left the arbiter stuck with a
    // cached HIGH=false winner, preventing MEDIUM claims from resuming).
    // -----------------------------------------------------------------------

    #[tokio::test]
    async fn release_lets_lower_priority_resume() {
        let (arbiter, bus) = setup_lighting().await;
        let sig = "Body.Lights.DirectionIndicator.Right.IsSignaling";

        // Turn (medium) claims right indicator ON.
        arbiter
            .request(ActuatorRequest {
                signal: sig,
                value: SignalValue::Bool(true),
                priority: Priority::Medium,
                feature_id: FeatureId::TurnIndicator,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // Hazard (high) takes over — equal value so no publish change.
        arbiter
            .request(ActuatorRequest {
                signal: sig,
                value: SignalValue::Bool(true),
                priority: Priority::High,
                feature_id: FeatureId::Hazard,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // Hazard releases — Turn's MEDIUM claim is still active, resolved
        // value is still true, so nothing new should be published.
        arbiter.release(sig, FeatureId::Hazard).await.unwrap();
        tokio::task::yield_now().await;

        // Turn releases last — now no claims, default-off should publish.
        arbiter
            .release(sig, FeatureId::TurnIndicator)
            .await
            .unwrap();
        tokio::task::yield_now().await;

        let history = bus.history();
        // Expect exactly two events: Turn's initial true, then the final false
        // when the last claim is withdrawn.
        assert_eq!(
            history.len(),
            2,
            "expected 2 publishes (claim + final release), got: {:?}",
            history
        );
        assert_eq!(history[0].1, SignalValue::Bool(true));
        assert_eq!(history[1].1, SignalValue::Bool(false));
    }

    #[tokio::test]
    async fn release_republishes_lower_priority_distinct_value() {
        let (arbiter, bus) = setup_lighting().await;
        let sig = "Body.Lights.DirectionIndicator.Left.IsSignaling";

        // Turn claims MEDIUM OFF (explicit claim of false).
        arbiter
            .request(ActuatorRequest {
                signal: sig,
                value: SignalValue::Bool(false),
                priority: Priority::Medium,
                feature_id: FeatureId::TurnIndicator,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // Hazard overrides with HIGH TRUE.
        arbiter
            .request(ActuatorRequest {
                signal: sig,
                value: SignalValue::Bool(true),
                priority: Priority::High,
                feature_id: FeatureId::Hazard,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // Hazard releases — Turn's MEDIUM false claim is the only survivor,
        // so the arbiter must republish false.
        arbiter.release(sig, FeatureId::Hazard).await.unwrap();
        tokio::task::yield_now().await;

        let history = bus.history();
        // Expect false (Turn), true (Hazard overrides), false (Turn resumes).
        assert_eq!(history.len(), 3, "history: {:?}", history);
        assert_eq!(history[0].1, SignalValue::Bool(false));
        assert_eq!(history[1].1, SignalValue::Bool(true));
        assert_eq!(history[2].1, SignalValue::Bool(false));
    }

    #[tokio::test]
    async fn release_without_any_claims_publishes_default_off() {
        let (arbiter, bus) = setup_lighting().await;
        let sig = "Body.Lights.DirectionIndicator.Left.IsSignaling";

        // Hazard claims ON.
        arbiter
            .request(ActuatorRequest {
                signal: sig,
                value: SignalValue::Bool(true),
                priority: Priority::High,
                feature_id: FeatureId::Hazard,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // Hazard releases — no other claims, so arbiter must publish the
        // default-off value (Bool(false)).
        arbiter.release(sig, FeatureId::Hazard).await.unwrap();
        tokio::task::yield_now().await;

        let history = bus.history();
        assert_eq!(history.len(), 2, "history: {:?}", history);
        assert_eq!(history[0].1, SignalValue::Bool(true));
        assert_eq!(history[1].1, SignalValue::Bool(false));
    }

    #[tokio::test]
    async fn release_of_nonexistent_claim_is_noop() {
        let (arbiter, bus) = setup_lighting().await;
        let sig = "Body.Lights.DirectionIndicator.Left.IsSignaling";

        // Feature releases a signal it never claimed — should do nothing.
        arbiter.release(sig, FeatureId::Hazard).await.unwrap();
        tokio::task::yield_now().await;

        assert_eq!(bus.history().len(), 0);
    }

    // -----------------------------------------------------------------------
    // DoorLockArbiter tests
    // -----------------------------------------------------------------------

    async fn setup_door_lock() -> (DoorLockArbiter, mpsc::Sender<LockAck>, Arc<MockBus>) {
        let bus = Arc::new(MockBus::new());
        let (arbiter, ack_tx, loop_fut) = door_lock_arbiter(Arc::clone(&bus));
        tokio::spawn(loop_fut);
        tokio::task::yield_now().await;
        (arbiter, ack_tx, bus)
    }

    #[tokio::test]
    async fn door_lock_idle_dispatches_immediately() {
        let (arbiter, _ack_tx, bus) = setup_door_lock().await;

        arbiter
            .request(DoorLockRequest {
                command: LockCommand::UnlockAll,
                feature_id: FeatureId::KeyfobPeps,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        let history: Vec<_> = bus
            .history()
            .into_iter()
            .filter(|(s, _)| *s == CENTRAL_LOCK_CMD)
            .collect();
        assert_eq!(history.len(), 1);
        assert_eq!(history[0].0, CENTRAL_LOCK_CMD);
        assert_eq!(history[0].1, SignalValue::String("unlock_all".into()));
    }

    #[tokio::test]
    async fn door_lock_queues_during_active() {
        let (arbiter, ack_tx, bus) = setup_door_lock().await;

        // PEPS unlock → active
        arbiter
            .request(DoorLockRequest {
                command: LockCommand::UnlockAll,
                feature_id: FeatureId::KeyfobPeps,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // AutoLock lock → pending (should NOT dispatch yet)
        arbiter
            .request(DoorLockRequest {
                command: LockCommand::LockAll,
                feature_id: FeatureId::AutoLock,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // Only the first command (PEPS unlock) should be dispatched.
        // Filter to CENTRAL_LOCK_CMD only — the arbiter also publishes
        // Cabin.LockStatus on each accepted command, which we don't
        // care about here.
        let cmd_history = |bus: &MockBus| -> Vec<(VssPath, SignalValue)> {
            bus.history()
                .into_iter()
                .filter(|(s, _)| *s == CENTRAL_LOCK_CMD)
                .collect()
        };
        assert_eq!(cmd_history(&bus).len(), 1);

        // ACK the first operation → pending promotes to active
        ack_tx
            .send(LockAck {
                event_number: 1,
                door_results: [true, true, true, true],
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // Now 2 commands: unlock_all + lock_all
        let history = cmd_history(&bus);
        assert_eq!(history.len(), 2);
        assert_eq!(history[1].1, SignalValue::String("lock_all".into()));
    }

    #[tokio::test]
    async fn door_lock_newer_replaces_pending() {
        let (arbiter, ack_tx, bus) = setup_door_lock().await;

        // PEPS unlock → active
        arbiter
            .request(DoorLockRequest {
                command: LockCommand::UnlockAll,
                feature_id: FeatureId::KeyfobPeps,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // AutoLock lock → pending
        arbiter
            .request(DoorLockRequest {
                command: LockCommand::LockAll,
                feature_id: FeatureId::AutoLock,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // KeyfobRke double-lock → replaces AutoLock in pending
        arbiter
            .request(DoorLockRequest {
                command: LockCommand::DoubleLockAll,
                feature_id: FeatureId::KeyfobRke,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // ACK → KeyfobRke should dispatch, not AutoLock
        ack_tx
            .send(LockAck {
                event_number: 1,
                door_results: [true, true, true, true],
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        let history: Vec<_> = bus
            .history()
            .into_iter()
            .filter(|(s, _)| *s == CENTRAL_LOCK_CMD)
            .collect();
        // 1 unlock + 1 double-lock
        assert_eq!(history.len(), 2);
        // Double-lock token
        assert_eq!(history[1].0, CENTRAL_LOCK_CMD);
        assert_eq!(history[1].1, SignalValue::String("lock_double".into()));
    }

    #[tokio::test]
    async fn door_lock_crash_unlock_not_replaceable() {
        let (arbiter, _ack_tx, bus) = setup_door_lock().await;

        // PEPS lock → active
        arbiter
            .request(DoorLockRequest {
                command: LockCommand::LockAll,
                feature_id: FeatureId::KeyfobPeps,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // CrashUnlock → pending
        arbiter
            .request(DoorLockRequest {
                command: LockCommand::UnlockAll,
                feature_id: FeatureId::CrashUnlock,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // AutoLock tries to replace → should be rejected
        arbiter
            .request(DoorLockRequest {
                command: LockCommand::LockAll,
                feature_id: FeatureId::AutoLock,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // Only 1 command (PEPS lock). CrashUnlock is pending, AutoLock was rejected.
        let cmd_count = bus
            .history()
            .into_iter()
            .filter(|(s, _)| *s == CENTRAL_LOCK_CMD)
            .count();
        assert_eq!(cmd_count, 1);
    }

    #[tokio::test]
    async fn door_lock_crash_lockout_10_seconds() {
        let (arbiter, _ack_tx, bus) = setup_door_lock().await;

        // CrashUnlock dispatches immediately (idle)
        arbiter
            .request(DoorLockRequest {
                command: LockCommand::UnlockAll,
                feature_id: FeatureId::CrashUnlock,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        let cmd_count = |bus: &MockBus| -> usize {
            bus.history()
                .into_iter()
                .filter(|(s, _)| *s == CENTRAL_LOCK_CMD)
                .count()
        };

        // 1 command dispatched (crash unlock)
        assert_eq!(cmd_count(&bus), 1);

        // KeyfobRke tries to lock → rejected (crash lockout)
        arbiter
            .request(DoorLockRequest {
                command: LockCommand::LockAll,
                feature_id: FeatureId::KeyfobRke,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // Still 1 — KeyfobRke was rejected
        assert_eq!(cmd_count(&bus), 1);
    }

    #[tokio::test]
    async fn door_lock_unauthorized_rejected() {
        let (arbiter, _ack_tx, bus) = setup_door_lock().await;

        // DRL tries to lock doors — not in allow-list
        arbiter
            .request(DoorLockRequest {
                command: LockCommand::LockAll,
                feature_id: FeatureId::Drl,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // No CentralLock command should have been dispatched.  (The
        // arbiter publishes a one-off EventNum=0 sentinel on boot —
        // ignore it.)
        let cmd_count = bus
            .history()
            .into_iter()
            .filter(|(s, _)| *s == CENTRAL_LOCK_CMD)
            .count();
        assert_eq!(cmd_count, 0, "unauthorized request should not dispatch");
    }

    // ─── PhysicalGate (puddle / mirror-fold) ─────────────────────────────

    /// Helper: spawn the puddle arbiter on a MockBus and yield enough
    /// for the gate-watcher tasks to subscribe.
    async fn setup_puddle() -> (DomainArbiter, Arc<MockBus>) {
        let bus = Arc::new(MockBus::new());
        let (arbiter, loop_fut) = puddle_arbiter(Arc::clone(&bus));
        tokio::spawn(loop_fut);
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        (arbiter, bus)
    }

    async fn yield_settle() {
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    #[tokio::test]
    async fn physical_gate_blocks_claim_when_already_closed() {
        let (arbiter, bus) = setup_puddle().await;

        // Mirror is folded BEFORE Welcome claims.
        bus.inject("Body.Mirror.Left.IsFolded", SignalValue::Bool(true));
        yield_settle().await;

        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.Puddle.Left.IsOn",
                value: SignalValue::Bool(true),
                priority: Priority::Medium,
                feature_id: FeatureId::Welcome,
            })
            .await
            .unwrap();
        yield_settle().await;

        // Claim resolved against the closed gate → published as off.
        assert_eq!(
            bus.latest_value("Body.Lights.Puddle.Left.IsOn"),
            Some(SignalValue::Bool(false)),
            "gate closed at claim time → arbiter must publish off"
        );
    }

    #[tokio::test]
    async fn physical_gate_closing_mid_claim_releases_target() {
        let (arbiter, bus) = setup_puddle().await;

        // Mirror starts unfolded; Welcome claims; puddle goes ON.
        bus.inject("Body.Mirror.Right.IsFolded", SignalValue::Bool(false));
        yield_settle().await;
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.Puddle.Right.IsOn",
                value: SignalValue::Bool(true),
                priority: Priority::Medium,
                feature_id: FeatureId::Welcome,
            })
            .await
            .unwrap();
        yield_settle().await;
        assert_eq!(
            bus.latest_value("Body.Lights.Puddle.Right.IsOn"),
            Some(SignalValue::Bool(true))
        );

        // Mirror folds → arbiter forces off without any feature action.
        bus.inject("Body.Mirror.Right.IsFolded", SignalValue::Bool(true));
        yield_settle().await;
        assert_eq!(
            bus.latest_value("Body.Lights.Puddle.Right.IsOn"),
            Some(SignalValue::Bool(false)),
            "gate closing mid-claim must publish off"
        );
    }

    #[tokio::test]
    async fn physical_gate_opening_restores_active_claim() {
        let (arbiter, bus) = setup_puddle().await;

        // Pre-fold the mirror, then claim — claim is suppressed.
        bus.inject("Body.Mirror.Left.IsFolded", SignalValue::Bool(true));
        yield_settle().await;
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.Puddle.Left.IsOn",
                value: SignalValue::Bool(true),
                priority: Priority::Medium,
                feature_id: FeatureId::Welcome,
            })
            .await
            .unwrap();
        yield_settle().await;

        // Unfold the mirror — the existing claim should now win.
        bus.inject("Body.Mirror.Left.IsFolded", SignalValue::Bool(false));
        yield_settle().await;
        assert_eq!(
            bus.latest_value("Body.Lights.Puddle.Left.IsOn"),
            Some(SignalValue::Bool(true)),
            "gate opening must let the active claim through"
        );
    }

    #[tokio::test]
    async fn physical_gate_only_affects_its_own_target() {
        let (arbiter, bus) = setup_puddle().await;

        // Fold ONLY the left mirror.
        bus.inject("Body.Mirror.Left.IsFolded", SignalValue::Bool(true));
        yield_settle().await;

        // Claim BOTH puddles.
        for sig in [
            "Body.Lights.Puddle.Left.IsOn",
            "Body.Lights.Puddle.Right.IsOn",
        ] {
            arbiter
                .request(ActuatorRequest {
                    signal: sig,
                    value: SignalValue::Bool(true),
                    priority: Priority::Medium,
                    feature_id: FeatureId::Welcome,
                })
                .await
                .unwrap();
        }
        yield_settle().await;

        assert_eq!(
            bus.latest_value("Body.Lights.Puddle.Left.IsOn"),
            Some(SignalValue::Bool(false)),
            "left puddle suppressed by left-mirror gate"
        );
        assert_eq!(
            bus.latest_value("Body.Lights.Puddle.Right.IsOn"),
            Some(SignalValue::Bool(true)),
            "right puddle unaffected by left-mirror gate"
        );
    }

    // ─── trunk_arbiter (valet gate) ──────────────────────────────────

    async fn setup_trunk() -> (DomainArbiter, Arc<MockBus>) {
        let bus = Arc::new(MockBus::new());
        let (arbiter, loop_fut) = trunk_arbiter(Arc::clone(&bus));
        tokio::spawn(loop_fut);
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        (arbiter, bus)
    }

    /// Helper: pulse `Body.Trunk.OpenCmd` via the trunk arbiter as a
    /// momentary edge — request true, then release so the next press
    /// can fire again.
    async fn pulse_trunk_open(arbiter: &DomainArbiter, feature_id: FeatureId) {
        arbiter
            .request(ActuatorRequest {
                signal: TRUNK_OPEN_CMD,
                value: SignalValue::Bool(true),
                priority: Priority::Medium,
                feature_id,
            })
            .await
            .unwrap();
        yield_settle().await;
        arbiter.release(TRUNK_OPEN_CMD, feature_id).await.unwrap();
        yield_settle().await;
    }

    #[tokio::test]
    async fn trunk_arbiter_passes_open_cmd_when_valet_inactive() {
        let (arbiter, bus) = setup_trunk().await;

        // Default valet is unset (treated as gate open).
        pulse_trunk_open(&arbiter, FeatureId::ExteriorTrunkButton).await;

        // The bus history should show a true → false pulse.
        let trues = bus
            .history()
            .iter()
            .filter(|(p, v)| *p == TRUNK_OPEN_CMD && *v == SignalValue::Bool(true))
            .count();
        assert!(
            trues >= 1,
            "valet inactive: arbiter must publish OpenCmd=true on request"
        );
    }

    #[tokio::test]
    async fn trunk_arbiter_drops_open_cmd_when_valet_active() {
        let (arbiter, bus) = setup_trunk().await;

        // Activate valet BEFORE the press.
        bus.inject("Cabin.ValetMode.IsActive", SignalValue::Bool(true));
        yield_settle().await;

        pulse_trunk_open(&arbiter, FeatureId::ExteriorTrunkButton).await;

        // Gate forces false; no true ever reaches Body.Trunk.OpenCmd.
        let trues = bus
            .history()
            .iter()
            .filter(|(p, v)| *p == TRUNK_OPEN_CMD && *v == SignalValue::Bool(true))
            .count();
        assert_eq!(
            trues, 0,
            "valet active: arbiter must suppress OpenCmd=true on every request"
        );
    }

    #[tokio::test]
    async fn trunk_arbiter_valet_blocks_rke_path() {
        let (arbiter, bus) = setup_trunk().await;

        bus.inject("Cabin.ValetMode.IsActive", SignalValue::Bool(true));
        yield_settle().await;

        // RKE TrunkRelease routed through the same arbiter — the gate
        // applies regardless of feature.
        pulse_trunk_open(&arbiter, FeatureId::KeyfobRke).await;

        let trues = bus
            .history()
            .iter()
            .filter(|(p, v)| *p == TRUNK_OPEN_CMD && *v == SignalValue::Bool(true))
            .count();
        assert_eq!(
            trues, 0,
            "valet must gate RKE TrunkRelease the same as the exterior button"
        );
    }

    #[tokio::test]
    async fn trunk_arbiter_valet_does_not_close_already_open_trunk() {
        // The gate is on `Body.Trunk.OpenCmd`, not `Body.Trunk.IsOpen`.
        // Activating valet while the trunk is open must NOT publish a
        // close command — IsOpen lives in the plant model and we never
        // want to slam the lid shut.
        let (_arbiter, bus) = setup_trunk().await;

        // Pretend the trunk is already open (plant-model state).
        bus.inject("Body.Trunk.IsOpen", SignalValue::Bool(true));
        yield_settle().await;

        // Activate valet.
        bus.inject("Cabin.ValetMode.IsActive", SignalValue::Bool(true));
        yield_settle().await;

        // Body.Trunk.IsOpen unchanged; the arbiter has no business
        // touching it — it only governs `Body.Trunk.OpenCmd`.
        assert_eq!(
            bus.latest_value("Body.Trunk.IsOpen"),
            Some(SignalValue::Bool(true)),
            "valet activation must not close an already-open trunk"
        );
    }

    #[tokio::test]
    async fn trunk_arbiter_valet_deactivation_allows_subsequent_press() {
        let (arbiter, bus) = setup_trunk().await;

        // Valet on → press blocked.
        bus.inject("Cabin.ValetMode.IsActive", SignalValue::Bool(true));
        yield_settle().await;
        pulse_trunk_open(&arbiter, FeatureId::ExteriorTrunkButton).await;
        let trues_blocked = bus
            .history()
            .iter()
            .filter(|(p, v)| *p == TRUNK_OPEN_CMD && *v == SignalValue::Bool(true))
            .count();
        assert_eq!(trues_blocked, 0);

        // Valet off → next press passes through.
        bus.inject("Cabin.ValetMode.IsActive", SignalValue::Bool(false));
        yield_settle().await;
        pulse_trunk_open(&arbiter, FeatureId::ExteriorTrunkButton).await;
        let trues_after = bus
            .history()
            .iter()
            .filter(|(p, v)| *p == TRUNK_OPEN_CMD && *v == SignalValue::Bool(true))
            .count();
        assert!(
            trues_after >= 1,
            "valet deactivated: subsequent press must publish OpenCmd=true"
        );
    }
}
