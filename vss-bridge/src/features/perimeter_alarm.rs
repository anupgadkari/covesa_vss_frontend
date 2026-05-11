//! Perimeter alarm — anti-intrusion alarm triggered by an unauthorised
//! door opening while the cabin is locked.
//!
//! # Trigger
//!
//! Any door's `IsOpen` transitions FALSE→TRUE while
//! `Cabin.LockStatus` is `LOCKED` or `DOUBLE_LOCKED` AND the cabin
//! has been continuously locked for at least `PRE_ARM_DURATION_SECS`
//! (20 s).  The 20 s pre-arm window absorbs the common case of "lock
//! the car, then realise you forgot something" — re-opening a door
//! within 20 s of locking is silently ignored, no chime / horn / light
//! show.  After the pre-arm window the alarm is fully armed and a
//! door-open is a "thief opens the door without unlocking first" event.
//!
//! # Three phases on a single 1 Hz pulse train (400 ms ON, 600 ms OFF):
//!
//! 1. **First 12 s — Chime warning.**  Soft interior chime only
//!    (`Body.Chime.IsActive`).  No horn, no flashing.  Gives a
//!    legitimate driver who entered with a mechanical-blade key (low
//!    fob battery) a chance to authenticate before a full alarm
//!    fires.  If they unlock with a valid fob/phone or hit panic in
//!    this window, the alarm cancels silently — no exterior light
//!    show, no horn.
//! 2. **Next 30 s — Full alarm with horn.**  Direction indicators
//!    (both), dome, exterior puddle lamps, AND the main horn all
//!    pulse together.
//! 3. **Remaining 4 min 18 s — Lights only.**  Horn stops; visible
//!    flashing continues until the 5-minute total elapses, then the
//!    pulse loop self-exits.
//!
//! # Disarm conditions (any one ends the alarm immediately)
//!
//! 1. **Auth-validated unlock** — `Cabin.LockStatus.LastRequestor`
//!    transitions to one of the external auth sources (RKE,
//!    PassiveEntry, PEPS, phone, NFC) AND `Cabin.LockStatus` is
//!    `UNLOCKED` or `DRIVER_UNLOCKED`.  Models "the user came back
//!    and proved they own the vehicle."
//! 2. **Panic button press** — `Body.Switches.Panic.IsEngaged`
//!    transitions to `true` from any source (paired fob, HMI test,
//!    telematics).  Same pattern PanicAlarm uses, treats the user
//!    grabbing the panic button as a "stop everything" override.
//!
//! 3. **Ignition goes live** — `Vehicle.LowVoltageSystemState`
//!    transitions to `ON` or `START` while a sequence is in flight.
//!    Cranking / running the engine proves the operator has
//!    legitimate access (matched key, immobiliser pass), so we kill
//!    the chime / alarm rather than have the owner drive away with
//!    the horn blaring.  `ACC` alone does NOT cancel — that state can
//!    be reached by jiggling a pried ignition cylinder.
//!
//! Thumb-pad / soldier-knob / HMI-direct unlocks do NOT disarm —
//! they are not authenticated against a paired device.
//!
//! # Independence from PanicAlarm
//!
//! PanicAlarm and PerimeterAlarm both pulse the same indicators +
//! horn at HIGH priority on the lighting / horn arbiters.  They are
//! mutually exclusive in practice: a panic-button press disarms
//! PerimeterAlarm here; a successful unlock cancels PanicAlarm via
//! its existing `FEEDBACK_REQUEST = "unlock"` watcher.  No bus-level
//! contention because at any time only one of the two is claiming.
//!
//! # Status flag
//!
//! `Vehicle.Body.Alarm.IsActive` (already published by PanicAlarm)
//! is also asserted by this feature for the duration of the
//! alarm.  Telematics / HMI consumers see a single "alarm active"
//! bool regardless of source.

use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;

use futures::stream::{select_all, StreamExt};
use tokio::sync::mpsc;
use tokio::task::JoinHandle;
use tokio::time::{sleep, Instant, Sleep};

use crate::arbiter::{ActuatorRequest, DomainArbiter};
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

const FEATURE_ID: FeatureId = FeatureId::PerimeterAlarm;

const DOOR_OPEN_SIGNALS: [VssPath; 4] = [
    "Body.Doors.Row1.Left.IsOpen",
    "Body.Doors.Row1.Right.IsOpen",
    "Body.Doors.Row2.Left.IsOpen",
    "Body.Doors.Row2.Right.IsOpen",
];

const LOCK_STATUS: VssPath = "Cabin.LockStatus";
const LAST_REQUESTOR: VssPath = "Cabin.LockStatus.LastRequestor";
const PANIC_SWITCH: VssPath = "Body.Switches.Panic.IsEngaged";
const ALARM_STATUS: VssPath = "Vehicle.Body.Alarm.IsActive";
/// Authoritative perimeter-alarm state, published by this feature on
/// every transition so the HMI / telematics display a single string
/// enum rather than computing state from secondary signals (lock
/// status × chime active × alarm status × wall-clock).  Values:
///   * "DISARMED"   — cabin not armable (UNLOCKED / DRIVER_UNLOCKED).
///   * "PRE_ARMED"  — cabin just locked; 20 s grace window in flight.
///   * "ARMED"      — cabin locked, pre-arm window elapsed; watching.
///   * "ACTIVATED"  — chime / full-alarm / lights-only sequence running.
const ALARM_STATE: VssPath = "Vehicle.Body.Alarm.State";
const POWER_STATE: VssPath = "Vehicle.LowVoltageSystemState";

const LEFT_INDICATOR: VssPath = "Body.Lights.DirectionIndicator.Left.IsSignaling";
const RIGHT_INDICATOR: VssPath = "Body.Lights.DirectionIndicator.Right.IsSignaling";
const HORN: VssPath = "Body.Horn.IsActive";
const CHIME: VssPath = "Body.Chime.IsActive";
const DOME: VssPath = "Cabin.Lights.IsDomeOn";
const PUDDLE_LEFT: VssPath = "Body.Lights.Puddle.Left.IsOn";
const PUDDLE_RIGHT: VssPath = "Body.Lights.Puddle.Right.IsOn";

/// Pre-arm grace period after the cabin first becomes locked (s).
/// Lock → close door → the alarm watches but is not yet armed.  A
/// door-opened-while-locked event during this window is silently
/// ignored — gives the user time to walk away, fumble with bags, or
/// realise they need to re-open the door for a forgotten item without
/// instantly tripping their own alarm.  After the window elapses the
/// alarm is fully armed.
const PRE_ARM_DURATION_SECS: u64 = 20;
/// Pre-alarm chime warning duration (s).  Gives a legitimate driver
/// who entered with a mechanical-blade key (low fob battery) a window
/// to authenticate before the full alarm escalates.
const CHIME_DURATION_SECS: u64 = 12;
/// How long the horn pulses after the chime phase ends (s).
const HORN_DURATION_SECS: u64 = 30;
/// How long the visible flashing continues after the chime phase ends (s).
const LIGHTS_DURATION_SECS: u64 = 5 * 60;

/// Pulse cadence — same as PanicAlarm so the two look identical to
/// the user when active.
const ON_MS: u64 = 400;
const OFF_MS: u64 = 600;

/// Effective "sleep forever" used to park the pre-arm deadline timer
/// while the cabin is unarmed.  Long enough that we never accidentally
/// re-fire it; short enough to avoid `Duration` overflow.
const IDLE_SLEEP: Duration = Duration::from_secs(86_400 * 365);

/// External auth-source identities that disarm the alarm on unlock.
/// Mirrors `auto_relock::EXTERNAL_UNLOCK_REQUESTORS` — same trust
/// boundary (you must have proven possession of a paired device).
const EXTERNAL_AUTH_SOURCES: &[&str] = &[
    "KeyfobRke",
    "KeyfobPeps",
    "PassiveEntry",
    "PhoneApp",
    "PhoneBle",
    "NfcCard",
    "NfcPhone",
];

/// Internal (non-authenticated) unlock sources.  An unlock from any of
/// these while the cabin is fully armed is treated as a tampering /
/// intrusion event and triggers the perimeter alarm.  Today this is
/// just the interior door-trim Lock/Unlock buttons; if a sill-knob or
/// other un-authenticated interior unlock source is added later it
/// belongs in this list too.
const INTERNAL_UNLOCK_SOURCES: &[&str] = &["DoorTrimButton"];

/// Lock requestors that are recognised as deliberate "arm the perimeter
/// alarm" actions — i.e., the only way to leave `DISARMED` is for one
/// of these requestors to publish a fresh lock cycle.  AutoLock and
/// DoorTrimButton are deliberately omitted: they fire while an occupant
/// is in the cabin (drive-away auto-lock, child hits the trim switch),
/// neither of which represents the owner walking away from a locked
/// vehicle.  The user must produce a fresh external lock event after
/// every disarm — no auto-rearm on ignition off.
const EXTERNAL_LOCK_REQUESTORS: &[&str] = &[
    "KeyfobRke",
    "KeyfobPeps",
    "ThumbPadLock",
    "AutoRelock",
    "WalkAwayLock",
    "PhoneApp",
    "PhoneBle",
    "NfcCard",
    "NfcPhone",
    // SlamLock is in this list (arming-trusted) but deliberately NOT
    // in `EXTERNAL_AUTH_SOURCES` (disarming-trusted) or
    // `INTERNAL_UNLOCK_SOURCES` (tampering-trigger).  Lets a US slam-
    // lock-with-door-open arm the alarm at door-close, while ensuring
    // a thief who hits trim lock during a chime cannot use the EU
    // SlamLock-driven inversion to silently disarm the alarm.
    "SlamLock",
];

/// Cached EventNum signal path.  The arbiter publishes
/// `Cabin.LockStatus` then `Cabin.LockStatus.LastRequestor` then
/// `Cabin.LockStatus.EventNum` on every accepted lock command — by
/// the time we receive an EventNum bump we are guaranteed to have
/// the matching status + requestor cached, which gives us a clean
/// "the entire lock event is now visible" wakeup.  Drives the
/// `DISARMED → PRE_ARMED` transition.
const LOCK_EVENT_NUM: VssPath = "Cabin.LockStatus.EventNum";

const UNLOCKED_STATES: &[&str] = &["UNLOCKED", "DRIVER_UNLOCKED"];

/// Returns true if `status` represents a locked cabin and a fresh
/// door-open should arm the alarm.
fn is_armable_lock_state(status: &str) -> bool {
    matches!(status, "LOCKED" | "DOUBLE_LOCKED")
}

/// Authoritative perimeter-alarm states.  Maintained as the single
/// source of truth by `run()`; the HMI displays whatever it sees on
/// `Vehicle.Body.Alarm.State` rather than reconstructing the state
/// from secondary signals.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AlarmState {
    Disarmed,
    PreArmed,
    Armed,
    Activated,
}

impl AlarmState {
    fn as_str(&self) -> &'static str {
        match self {
            AlarmState::Disarmed => "DISARMED",
            AlarmState::PreArmed => "PRE_ARMED",
            AlarmState::Armed => "ARMED",
            AlarmState::Activated => "ACTIVATED",
        }
    }
}

/// True if `Vehicle.LowVoltageSystemState` represents a "live" ignition
/// state (engine running or cranking).  An owner who successfully
/// started the vehicle has clearly authenticated themselves — pull the
/// plug on any chime / alarm sequence in flight.  `ACC` is NOT enough:
/// it can be reached by twisting a stolen key blade or jiggling a
/// pried-open ignition cylinder; only `ON` / `START` cancels.
fn is_ignition_live(val: &SignalValue) -> bool {
    matches!(val, SignalValue::String(s) if s == "ON" || s == "START")
}

pub struct PerimeterAlarm<B: SignalBus> {
    bus: Arc<B>,
    lighting_arb: Arc<DomainArbiter>,
    horn_arb: Arc<DomainArbiter>,
    courtesy_arb: Arc<DomainArbiter>,
    puddle_arb: Arc<DomainArbiter>,
}

impl<B: SignalBus + Send + Sync + 'static> PerimeterAlarm<B> {
    pub fn new(
        bus: Arc<B>,
        lighting_arb: Arc<DomainArbiter>,
        horn_arb: Arc<DomainArbiter>,
        courtesy_arb: Arc<DomainArbiter>,
        puddle_arb: Arc<DomainArbiter>,
    ) -> Self {
        Self {
            bus,
            lighting_arb,
            horn_arb,
            courtesy_arb,
            puddle_arb,
        }
    }

    pub async fn run(self) {
        tracing::info!(
            horn_secs = HORN_DURATION_SECS,
            lights_secs = LIGHTS_DURATION_SECS,
            "PerimeterAlarm feature started"
        );

        // Subscribe to all four door-open streams as a single merged stream.
        let door_streams =
            futures::future::join_all(DOOR_OPEN_SIGNALS.iter().map(|&sig| self.bus.subscribe(sig)))
                .await;
        let mut door_stream = select_all(door_streams);

        let mut status_rx = self.bus.subscribe(LOCK_STATUS).await;
        let mut requestor_rx = self.bus.subscribe(LAST_REQUESTOR).await;
        let mut event_num_rx = self.bus.subscribe(LOCK_EVENT_NUM).await;
        let mut panic_rx = self.bus.subscribe(PANIC_SWITCH).await;
        let mut power_rx = self.bus.subscribe(POWER_STATE).await;

        // Cached signals — handlers below read them, but the explicit
        // FSM (current_state) is the only thing that decides what the
        // alarm "is."  Reduces the previous design's overlap between
        // a sticky `armed_session` flag and the published state — the
        // published state is now the single source of truth.
        let mut lock_status: String = "UNLOCKED".into();
        let mut last_requestor: String = String::new();
        let mut active: Option<JoinHandle<()>> = None;
        let mut ignition_live = false;
        // Latched at the armable→non-armable edge while the alarm was
        // armed — the requestor handler then decides whether the
        // unlock came from an internal (tampering) source.  Cleared
        // on disarm or fresh lock cycle.
        let mut armed_when_unlocked = false;
        // Pre-arm deadline timer.  Armed when DISARMED→PRE_ARMED fires;
        // when it elapses we transition PRE_ARMED→ARMED.  Idle value
        // is "far future" so the select! branch is dormant otherwise.
        let mut pre_arm_deadline: Pin<Box<Sleep>> = Box::pin(sleep(IDLE_SLEEP));
        // Natural-completion channel for run_alarm (5 min lights phase).
        // Aborts (disarm path) skip the send because they cancel the
        // task body before the line after `.await`.
        let (alarm_done_tx, mut alarm_done_rx) = mpsc::unbounded_channel::<()>();

        // Authoritative FSM state.  ALL transitions go through
        // `transition_to` below; the published `Vehicle.Body.Alarm.State`
        // and `current_state` are kept in sync there.
        let mut current_state = AlarmState::Disarmed;
        let _ = self
            .bus
            .publish(
                ALARM_STATE,
                SignalValue::String(current_state.as_str().into()),
            )
            .await;

        loop {
            tokio::select! {
                // `biased;` polls branches in declaration order rather
                // than randomly.  All lock-event-driven state
                // transitions live in `event_num_rx` and depend on
                // (lock_status, last_requestor) being fresh.  When the
                // arbiter publishes status → requestor → event_num in
                // quick succession, all three streams can be Ready in
                // the same select! poll; without biased ordering,
                // `event_num_rx` could win first and the transition
                // logic would run against a stale cache (re-arming on
                // an unlock, triggering tampering on a fresh lock, etc.).
                // Listing status_rx and requestor_rx first guarantees
                // the cache is fresh by the time we evaluate event_num.
                biased;
                Some(val) = status_rx.next() => {
                    // Pure cache updater — state transitions on lock
                    // events live in `event_num_rx` below, where the
                    // (status, requestor) tuple is guaranteed coherent.
                    // The only side-effect here is the
                    // `armed_when_unlocked` latch, set on the
                    // armable→non-armable edge so the tampering branch
                    // in `event_num_rx` knows the alarm was armed at
                    // the moment of unlock.
                    if let SignalValue::String(s) = val {
                        let was_armable = is_armable_lock_state(&lock_status);
                        lock_status = s;
                        let now_armable = is_armable_lock_state(&lock_status);
                        if was_armable
                            && !now_armable
                            && (current_state == AlarmState::Armed
                                || current_state == AlarmState::Activated)
                        {
                            armed_when_unlocked = true;
                        }
                    }
                }
                Some(val) = requestor_rx.next() => {
                    // Pure cache updater — see status_rx above.  The
                    // arbiter publishes status → requestor → EventNum,
                    // so by the time `event_num_rx` fires both caches
                    // are coherent and we can decide arming / disarming
                    // / tampering with confidence.
                    if let SignalValue::String(s) = val {
                        last_requestor = s;
                    }
                }
                Some(val) = door_stream.next() => {
                    // The only state where a door-open trips the alarm
                    // is ARMED — door-opens during PRE_ARMED are part
                    // of the 20 s grace, and during ACTIVATED / DISARMED
                    // they are irrelevant.
                    if !matches!(val, SignalValue::Bool(true)) { continue; }
                    if current_state != AlarmState::Armed { continue; }
                    tracing::warn!(
                        lock_status = %lock_status,
                        "PerimeterAlarm: door opened while ARMED — chime phase started"
                    );
                    self.start_alarm(&mut active, &alarm_done_tx);
                    armed_when_unlocked = false;
                    self.transition_to(&mut current_state, AlarmState::Activated).await;
                }
                Some(_) = event_num_rx.next() => {
                    // The arbiter publishes EventNum AFTER the matching
                    // status + requestor, so by the time this branch
                    // fires every lock-event-driven decision can be
                    // made on a coherent tuple.  All four lock-event
                    // transitions live here:
                    //
                    //   1. ARM       — Disarmed + cabin armable + external requestor
                    //   2. DISARM    — alarm running + auth unlock
                    //   3. TAMPERING — alarm idle + was-armed-when-unlocked + internal source
                    //   4. CLEANUP   — alarm idle + cabin unlocked + state != Disarmed
                    //
                    // These are mutually exclusive (different lock
                    // states / requestors), so the order below doesn't
                    // matter — at most one branch fires.

                    // 1. ARM
                    if current_state == AlarmState::Disarmed
                        && !ignition_live
                        && is_armable_lock_state(&lock_status)
                        && EXTERNAL_LOCK_REQUESTORS.contains(&last_requestor.as_str())
                    {
                        pre_arm_deadline
                            .as_mut()
                            .reset(Instant::now() + Duration::from_secs(PRE_ARM_DURATION_SECS));
                        tracing::info!(
                            requestor = %last_requestor,
                            pre_arm_secs = PRE_ARM_DURATION_SECS,
                            "PerimeterAlarm: external lock event — pre-arm window started"
                        );
                        self.transition_to(&mut current_state, AlarmState::PreArmed).await;
                    }
                    // 2. DISARM — auth unlock cancels a running alarm.
                    if active.is_some()
                        && self.is_auth_unlock(&lock_status, &last_requestor)
                    {
                        tracing::info!("PerimeterAlarm: auth unlock — DISARM");
                        self.disarm(&mut active).await;
                        armed_when_unlocked = false;
                        self.transition_to(&mut current_state, AlarmState::Disarmed).await;
                        pre_arm_deadline.as_mut().reset(Instant::now() + IDLE_SLEEP);
                    }
                    // 3. TAMPERING — armed → unlocked by interior source.
                    if active.is_none()
                        && armed_when_unlocked
                        && self.is_internal_unlock(&lock_status, &last_requestor)
                    {
                        tracing::warn!(
                            requestor = %last_requestor,
                            "PerimeterAlarm: internal unlock while ARMED — chime phase started"
                        );
                        self.start_alarm(&mut active, &alarm_done_tx);
                        armed_when_unlocked = false;
                        self.transition_to(&mut current_state, AlarmState::Activated).await;
                    }
                    // 4. CLEANUP — cabin unlocked, alarm idle, but
                    //    state is still PRE_ARMED / ARMED.  Drop to
                    //    DISARMED.  Skips ACTIVATED because a running
                    //    alarm survives a non-auth unlock (e.g.
                    //    ThumbPadLock-as-requestor on an unlock).
                    if active.is_none()
                        && !is_armable_lock_state(&lock_status)
                        && current_state != AlarmState::Disarmed
                    {
                        self.transition_to(&mut current_state, AlarmState::Disarmed).await;
                        armed_when_unlocked = false;
                        pre_arm_deadline.as_mut().reset(Instant::now() + IDLE_SLEEP);
                    }
                }
                Some(val) = panic_rx.next() => {
                    if matches!(val, SignalValue::Bool(true)) && active.is_some() {
                        tracing::info!("PerimeterAlarm: panic switch engaged — DISARM");
                        self.disarm(&mut active).await;
                        armed_when_unlocked = false;
                        self.transition_to(&mut current_state, AlarmState::Disarmed).await;
                        pre_arm_deadline.as_mut().reset(Instant::now() + IDLE_SLEEP);
                    }
                }
                Some(val) = power_rx.next() => {
                    // Ignition live (ON / START) ⇒ engine running ⇒
                    // operator passed the immobiliser ⇒ alarm goes to
                    // DISARMED.  Cabin can stay LOCKED forever after
                    // this; only a fresh external lock event re-arms.
                    // ACC alone does NOT cancel — it can be reached by
                    // jiggling a pried cylinder.
                    ignition_live = is_ignition_live(&val);
                    if ignition_live {
                        if active.is_some() {
                            tracing::info!("PerimeterAlarm: ignition live — DISARM");
                            self.disarm(&mut active).await;
                            armed_when_unlocked = false;
                        }
                        if current_state != AlarmState::Disarmed {
                            self.transition_to(&mut current_state, AlarmState::Disarmed).await;
                            pre_arm_deadline.as_mut().reset(Instant::now() + IDLE_SLEEP);
                        }
                    }
                }
                _ = pre_arm_deadline.as_mut() => {
                    // 20 s pre-arm window elapsed.  Promote PRE_ARMED →
                    // ARMED.  Park the deadline far out so we don't
                    // busy-wake.
                    pre_arm_deadline.as_mut().reset(Instant::now() + IDLE_SLEEP);
                    if current_state == AlarmState::PreArmed {
                        self.transition_to(&mut current_state, AlarmState::Armed).await;
                    }
                }
                Some(_) = alarm_done_rx.recv() => {
                    // 5 min sequence completed naturally — drop the
                    // JoinHandle and follow the sticky-disarm rule:
                    // never auto-rearm without a fresh external lock.
                    if let Some(h) = active.take() {
                        let _ = h.await;
                    }
                    tracing::info!("PerimeterAlarm: 5 min sequence completed naturally");
                    armed_when_unlocked = false;
                    self.transition_to(&mut current_state, AlarmState::Disarmed).await;
                }
                else => {
                    tracing::warn!("PerimeterAlarm: a stream closed, exiting");
                    self.disarm(&mut active).await;
                    return;
                }
            }
        }
    }

    /// Single point of state mutation — keeps the cached `current_state`
    /// and the published `Vehicle.Body.Alarm.State` signal in lockstep.
    /// Idempotent: redundant calls (same state) are a no-op so the bus
    /// history stays clean.
    async fn transition_to(&self, current: &mut AlarmState, new_state: AlarmState) {
        if *current == new_state {
            return;
        }
        tracing::info!(
            from = current.as_str(),
            to = new_state.as_str(),
            "PerimeterAlarm: state transition"
        );
        let _ = self
            .bus
            .publish(ALARM_STATE, SignalValue::String(new_state.as_str().into()))
            .await;
        *current = new_state;
    }

    fn is_auth_unlock(&self, lock_status: &str, last_requestor: &str) -> bool {
        UNLOCKED_STATES.contains(&lock_status) && EXTERNAL_AUTH_SOURCES.contains(&last_requestor)
    }

    /// True if the most recent lock-status update represents an unlock
    /// caused by a non-authenticated interior source (today: just
    /// `DoorTrimButton`).  Combined with `armed_when_unlocked` this is
    /// the signature of "intruder hit the trim unlock from inside."
    fn is_internal_unlock(&self, lock_status: &str, last_requestor: &str) -> bool {
        UNLOCKED_STATES.contains(&lock_status) && INTERNAL_UNLOCK_SOURCES.contains(&last_requestor)
    }

    /// Spawn the three-phase pulse task.  ALARM_STATUS stays FALSE
    /// during the 12 s chime phase — we only flip it true once the
    /// chime escalates into the full horn + lights alarm.  Lets HMI /
    /// telematics distinguish "warning chime" from "real intrusion."
    /// The `done_tx` is signalled when run_alarm finishes on its own
    /// (5 min natural completion); aborts skip the send because the
    /// task body never reaches the line after `.await`.
    fn start_alarm(
        &self,
        active: &mut Option<JoinHandle<()>>,
        done_tx: &mpsc::UnboundedSender<()>,
    ) {
        let bus = Arc::clone(&self.bus);
        let lighting = Arc::clone(&self.lighting_arb);
        let horn = Arc::clone(&self.horn_arb);
        let courtesy = Arc::clone(&self.courtesy_arb);
        let puddle = Arc::clone(&self.puddle_arb);
        let tx = done_tx.clone();
        *active = Some(tokio::spawn(async move {
            run_alarm(bus, lighting, horn, courtesy, puddle).await;
            let _ = tx.send(());
        }));
    }

    async fn disarm(&self, active: &mut Option<JoinHandle<()>>) {
        if let Some(handle) = active.take() {
            handle.abort();
            let _ = handle.await;
        }
        release_all(
            &self.bus,
            &self.lighting_arb,
            &self.horn_arb,
            &self.courtesy_arb,
            &self.puddle_arb,
        )
        .await;
        let _ = self
            .bus
            .publish(ALARM_STATUS, SignalValue::Bool(false))
            .await;
    }
}

/// Runs the pulsed alarm sequence until the parent task aborts us.
/// Three phases:
///
/// 1. **Chime warning** (0..12 s) — pulses `Body.Chime.IsActive` only.
///    Light/horn outputs stay quiet so a legitimate driver who entered
///    with a mechanical-blade key can authenticate without an
///    embarrassing exterior light show.
/// 2. **Full alarm with horn** (12..42 s) — direction indicators,
///    dome, puddle, AND horn pulse together.
/// 3. **Lights only** (42..312 s) — horn stops, visible flashing
///    continues until 5 min total then the loop self-exits.
async fn run_alarm<B: SignalBus + Send + Sync + 'static>(
    bus: Arc<B>,
    lighting: Arc<DomainArbiter>,
    horn: Arc<DomainArbiter>,
    courtesy: Arc<DomainArbiter>,
    puddle: Arc<DomainArbiter>,
) {
    let started = Instant::now();
    let chime_until = started + Duration::from_secs(CHIME_DURATION_SECS);
    // Horn + lights timers start AFTER the chime phase ends, so the
    // 30 s horn / 5 min lights are measured from escalation, not from
    // the original door-open.
    let horn_until = chime_until + Duration::from_secs(HORN_DURATION_SECS);
    let lights_until = chime_until + Duration::from_secs(LIGHTS_DURATION_SECS);
    let mut horn_was_active = false;
    let mut chime_was_active = false;
    let mut horn_phase_logged = false;
    let mut lights_phase_logged = false;
    tracing::info!(
        chime_secs = CHIME_DURATION_SECS,
        "PerimeterAlarm: chime phase ENTERED"
    );

    loop {
        let now = Instant::now();
        if now >= lights_until {
            tracing::info!("PerimeterAlarm: 5 min total elapsed — sequence COMPLETE");
            release_all(&bus, &lighting, &horn, &courtesy, &puddle).await;
            return;
        }

        if now < chime_until {
            // ── Chime warning phase ──
            let _ = bus.publish(CHIME, SignalValue::Bool(true)).await;
            chime_was_active = true;
            sleep(Duration::from_millis(ON_MS)).await;
            let _ = bus.publish(CHIME, SignalValue::Bool(false)).await;
            sleep(Duration::from_millis(OFF_MS)).await;
            continue;
        }

        // Chime phase has ended — make sure CHIME is OFF on the
        // first alarm-phase iteration so it doesn't get stranded on
        // a residual ON edge, and assert ALARM_STATUS=true now that
        // we are escalating to the real intrusion alarm.
        if chime_was_active {
            tracing::warn!(
                horn_secs = HORN_DURATION_SECS,
                "PerimeterAlarm: chime → FULL ALARM (horn + lights pulsing)"
            );
            let _ = bus.publish(CHIME, SignalValue::Bool(false)).await;
            let _ = bus.publish(ALARM_STATUS, SignalValue::Bool(true)).await;
            chime_was_active = false;
            horn_phase_logged = true;
        }

        let horn_active = now < horn_until;
        if !horn_active && horn_phase_logged && !lights_phase_logged {
            tracing::info!("PerimeterAlarm: horn done — LIGHTS ONLY phase");
            lights_phase_logged = true;
        }

        // ON edge.
        claim_lights(&lighting, &courtesy, &puddle, true).await;
        if horn_active {
            claim_horn(&horn, true).await;
            horn_was_active = true;
        } else if horn_was_active {
            // Horn phase just ended — release the claim once so the
            // arbiter publishes false and stops the next ON tick from
            // re-asserting it.
            let _ = horn.release(HORN, FEATURE_ID).await;
            horn_was_active = false;
        }
        sleep(Duration::from_millis(ON_MS)).await;

        // OFF edge.
        claim_lights(&lighting, &courtesy, &puddle, false).await;
        if horn_active {
            claim_horn(&horn, false).await;
        }
        sleep(Duration::from_millis(OFF_MS)).await;
    }
}

async fn claim_lights(
    lighting: &Arc<DomainArbiter>,
    courtesy: &Arc<DomainArbiter>,
    puddle: &Arc<DomainArbiter>,
    on: bool,
) {
    for &sig in &[LEFT_INDICATOR, RIGHT_INDICATOR] {
        let _ = lighting
            .request(ActuatorRequest {
                signal: sig,
                value: SignalValue::Bool(on),
                priority: Priority::High,
                feature_id: FEATURE_ID,
            })
            .await;
    }
    let _ = courtesy
        .request(ActuatorRequest {
            signal: DOME,
            value: SignalValue::Bool(on),
            priority: Priority::High,
            feature_id: FEATURE_ID,
        })
        .await;
    for &sig in &[PUDDLE_LEFT, PUDDLE_RIGHT] {
        let _ = puddle
            .request(ActuatorRequest {
                signal: sig,
                value: SignalValue::Bool(on),
                priority: Priority::High,
                feature_id: FEATURE_ID,
            })
            .await;
    }
}

async fn claim_horn(horn: &Arc<DomainArbiter>, on: bool) {
    let _ = horn
        .request(ActuatorRequest {
            signal: HORN,
            value: SignalValue::Bool(on),
            priority: Priority::High,
            feature_id: FEATURE_ID,
        })
        .await;
}

async fn release_all<B: SignalBus + Send + Sync + 'static>(
    bus: &Arc<B>,
    lighting: &Arc<DomainArbiter>,
    horn: &Arc<DomainArbiter>,
    courtesy: &Arc<DomainArbiter>,
    puddle: &Arc<DomainArbiter>,
) {
    for &sig in &[LEFT_INDICATOR, RIGHT_INDICATOR] {
        let _ = lighting.release(sig, FEATURE_ID).await;
    }
    let _ = horn.release(HORN, FEATURE_ID).await;
    let _ = courtesy.release(DOME, FEATURE_ID).await;
    for &sig in &[PUDDLE_LEFT, PUDDLE_RIGHT] {
        let _ = puddle.release(sig, FEATURE_ID).await;
    }
    // The chime is single-writer (no arbiter) so we publish false
    // directly here to ensure clean teardown if a disarm lands during
    // the chime phase.
    let _ = bus.publish(CHIME, SignalValue::Bool(false)).await;
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::{courtesy_arbiter, horn_arbiter, lighting_arbiter, puddle_arbiter};

    async fn settle(ms: u64) {
        // In start_paused mode the runtime auto-advances the mocked
        // clock whenever all tasks are parked on timers — so a real
        // `sleep(...)` here drives the pulse train inside run_alarm
        // through arbitrarily many sleep boundaries, which a single
        // direct `advance(...)` can't do because it only fires the
        // sleeps already pending when it's called.
        tokio::time::sleep(Duration::from_millis(ms)).await;
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    /// Mirrors the order in which `door_lock_arbiter` emits a lock
    /// event onto the bus: status → requestor → event_num, all three
    /// from the same logical command.  Tests must use this helper
    /// rather than poking `Cabin.LockStatus` directly — only the
    /// EventNum bump triggers a `DISARMED → PRE_ARMED` transition,
    /// and only when the requestor is in `EXTERNAL_LOCK_REQUESTORS`.
    /// `event_num_seq` is bumped by the caller so each event is unique.
    async fn inject_lock(bus: &Arc<MockBus>, status: &str, requestor: &str, event_num: u16) {
        // Yield between injects so PerimeterAlarm's select! processes
        // each branch in order (status → requestor → event_num),
        // matching what the real arbiter produces.  Without these
        // yields all three values land on their channels in one batch
        // and tokio's pseudo-random select polling can deliver
        // event_num to the feature *before* status / requestor — the
        // arming check then runs against stale caches and silently
        // skips, leaving the cabin DISARMED.  In production each
        // arbiter `bus.publish().await` carries an implicit yield, so
        // this helper just mimics that ordering for tests.
        bus.inject(LOCK_STATUS, SignalValue::String(status.into()));
        for _ in 0..4 {
            tokio::task::yield_now().await;
        }
        bus.inject(LAST_REQUESTOR, SignalValue::String(requestor.into()));
        for _ in 0..4 {
            tokio::task::yield_now().await;
        }
        bus.inject(LOCK_EVENT_NUM, SignalValue::Uint16(event_num));
        for _ in 0..8 {
            tokio::task::yield_now().await;
        }
    }

    async fn setup() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let (la, lf) = lighting_arbiter(Arc::clone(&bus));
        let (ha, hf) = horn_arbiter(Arc::clone(&bus));
        let (ca, cf) = courtesy_arbiter(Arc::clone(&bus));
        let (pa, pf) = puddle_arbiter(Arc::clone(&bus));
        tokio::spawn(lf);
        tokio::spawn(hf);
        tokio::spawn(cf);
        tokio::spawn(pf);
        let la = Arc::new(la);
        let ha = Arc::new(ha);
        let ca = Arc::new(ca);
        let pa = Arc::new(pa);

        let feat = PerimeterAlarm::new(
            Arc::clone(&bus),
            Arc::clone(&la),
            Arc::clone(&ha),
            Arc::clone(&ca),
            Arc::clone(&pa),
        );
        tokio::spawn(feat.run());
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        bus
    }

    #[tokio::test(start_paused = true)]
    async fn door_open_while_locked_arms_alarm() {
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        // Skip past the 20 s pre-arm window so subsequent door-opens
        // are treated as armed-alarm trips.
        settle(21_000).await;

        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle(1).await;

        // Chime phase fires CHIME, but NOT horn / indicators / puddle /
        // dome — gives a legitimate driver who entered with a mech-blade
        // key 12 s to authenticate before the exterior light show.
        // ALARM_STATUS also stays false during the chime so HMI /
        // telematics can distinguish "warning" from "real intrusion".
        settle(1).await;
        assert_eq!(bus.latest_value(CHIME), Some(SignalValue::Bool(true)));
        assert_eq!(bus.latest_value(HORN), None);
        assert_eq!(bus.latest_value(LEFT_INDICATOR), None);
        assert_eq!(bus.latest_value(DOME), None);
        assert_eq!(bus.latest_value(PUDDLE_LEFT), None);
        assert!(
            bus.history()
                .iter()
                .all(|(s, v)| !(*s == ALARM_STATUS && *v == SignalValue::Bool(true))),
            "ALARM_STATUS must stay false during chime phase"
        );

        // After the 12 s chime phase, horn + lights take over and
        // ALARM_STATUS finally flips true.
        settle(13_000).await;
        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(true))
        );
        assert_eq!(bus.latest_value(HORN), Some(SignalValue::Bool(true)));
        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(true))
        );
        assert_eq!(bus.latest_value(DOME), Some(SignalValue::Bool(true)));
        assert_eq!(bus.latest_value(PUDDLE_LEFT), Some(SignalValue::Bool(true)));
    }

    #[tokio::test(start_paused = true)]
    async fn auth_unlock_during_chime_disarms_silently() {
        // Authenticated unlock during the 12 s chime window cancels the
        // alarm before it ever escalates to horn + lights.
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        // Skip past the 20 s pre-arm window so subsequent door-opens
        // are treated as armed-alarm trips.
        settle(21_000).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle(50).await;
        // Mid-chime — chime is pulsing, horn / lights not yet active.
        settle(5_000).await;
        assert_eq!(bus.latest_value(HORN), None);

        inject_lock(&bus, "UNLOCKED", "KeyfobRke", 2).await;
        settle(50).await;

        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(false))
        );
        // Chime cleanly released.
        assert_eq!(bus.latest_value(CHIME), Some(SignalValue::Bool(false)));
        // Horn never fired during chime — no light show happened.
        // Now advance past where horn would have started; still must
        // not pulse because the task was aborted.
        settle(20_000).await;
        assert_eq!(bus.latest_value(HORN), None);
    }

    #[tokio::test(start_paused = true)]
    async fn panic_press_during_chime_disarms_silently() {
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        // Skip past the 20 s pre-arm window so subsequent door-opens
        // are treated as armed-alarm trips.
        settle(21_000).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle(2_000).await;
        assert_eq!(bus.latest_value(HORN), None);

        bus.inject(PANIC_SWITCH, SignalValue::Bool(true));
        settle(50).await;

        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(false))
        );
        assert_eq!(bus.latest_value(CHIME), Some(SignalValue::Bool(false)));
    }

    #[tokio::test(start_paused = true)]
    async fn door_open_while_unlocked_does_not_arm() {
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("UNLOCKED".into()));
        settle(1).await;

        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle(50).await;

        // ALARM_STATUS was never published (no claim was ever made on
        // it).  We assert by looking at history rather than latest_value.
        assert!(
            bus.history()
                .iter()
                .all(|(s, v)| !(*s == ALARM_STATUS && *v == SignalValue::Bool(true))),
            "ALARM_STATUS must not flip true on door open while unlocked"
        );
        assert!(
            bus.history()
                .iter()
                .all(|(s, v)| !(*s == HORN && *v == SignalValue::Bool(true))),
            "horn must not pulse when door opens on unlocked cabin"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn door_open_while_double_locked_arms_alarm() {
        let bus = setup().await;
        inject_lock(&bus, "DOUBLE_LOCKED", "KeyfobRke", 1).await;
        settle(21_000).await;
        bus.inject("Body.Doors.Row2.Right.IsOpen", SignalValue::Bool(true));
        // Advance past the 12 s chime phase so ALARM_STATUS gets
        // asserted true.
        settle(13_000).await;
        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(true))
        );
    }

    #[tokio::test(start_paused = true)]
    async fn auth_unlock_disarms_alarm() {
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        // Skip past the 20 s pre-arm window so subsequent door-opens
        // are treated as armed-alarm trips.
        settle(21_000).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        // Skip past the chime so we are in the full alarm phase
        // (horn + lights pulsing, ALARM_STATUS=true).
        settle(13_000).await;
        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(true))
        );

        // Owner returns and unlocks via PassiveEntry.
        inject_lock(&bus, "UNLOCKED", "PassiveEntry", 2).await;
        settle(50).await;

        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(false))
        );
        // Horn + indicator should be released — arbiter republishes false.
        assert_eq!(bus.latest_value(HORN), Some(SignalValue::Bool(false)));
    }

    #[tokio::test(start_paused = true)]
    async fn thumb_pad_unlock_does_not_disarm_alarm() {
        // ThumbPadLock unlock isn't authenticated against a paired
        // device — must NOT cancel the alarm.
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        // Skip past the 20 s pre-arm window so subsequent door-opens
        // are treated as armed-alarm trips.
        settle(21_000).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle(13_000).await;
        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(true))
        );

        inject_lock(&bus, "UNLOCKED", "ThumbPadLock", 2).await;
        settle(50).await;

        // Still alarming.
        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(true))
        );
    }

    #[tokio::test(start_paused = true)]
    async fn panic_press_disarms_alarm() {
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        // Skip past the 20 s pre-arm window so subsequent door-opens
        // are treated as armed-alarm trips.
        settle(21_000).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        // Past the chime — full alarm is now active.
        settle(13_000).await;
        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(true))
        );

        bus.inject(PANIC_SWITCH, SignalValue::Bool(true));
        settle(50).await;

        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(false))
        );
    }

    #[tokio::test(start_paused = true)]
    async fn horn_stops_after_30_seconds_lights_continue() {
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        // Skip past the 20 s pre-arm window so subsequent door-opens
        // are treated as armed-alarm trips.
        settle(21_000).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle(50).await;

        // Chime runs 0..12 s, horn 12..42 s.  Advance to ~43 s — horn
        // should now be released; lights still pulsing.
        settle(43_000).await;
        assert_eq!(
            bus.latest_value(HORN),
            Some(SignalValue::Bool(false)),
            "horn must stop after the 12 s chime + 30 s horn window"
        );
        // Lights continue — alarm status still true.
        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(true))
        );
    }

    #[tokio::test(start_paused = true)]
    async fn alarm_naturally_expires_after_5_minutes() {
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        // Skip past the 20 s pre-arm window so subsequent door-opens
        // are treated as armed-alarm trips.
        settle(21_000).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle(13_000).await;
        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(true))
        );

        // Chime is 12 s, lights run another 5 min after escalation.
        // After ~5 min from escalation + a beat, the alarm task
        // self-exits; status flag is NOT cleared automatically (we
        // only clear on explicit disarm).  Verifying here that the
        // lights eventually stop being driven active (latest publish
        // should be false from the OFF edge).
        settle(5 * 60 * 1000 + 2_000).await;
        // Indicators should rest at false (last OFF edge of the pulse train).
        assert_eq!(
            bus.latest_value(LEFT_INDICATOR),
            Some(SignalValue::Bool(false))
        );
    }

    #[tokio::test(start_paused = true)]
    async fn second_door_open_during_alarm_is_ignored() {
        // The alarm doesn't restart or extend on additional door opens.
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        // Skip past the 20 s pre-arm window so subsequent door-opens
        // are treated as armed-alarm trips.
        settle(21_000).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle(50).await;
        bus.inject("Body.Doors.Row1.Right.IsOpen", SignalValue::Bool(true));
        // Past the chime so ALARM_STATUS is asserted.
        settle(13_000).await;
        // Still one alarm active — no panic about re-arm.
        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(true))
        );
    }

    #[tokio::test(start_paused = true)]
    async fn door_open_during_pre_arm_window_is_ignored() {
        // Lock the cabin, then immediately re-open a door (within the
        // 20 s pre-arm window).  Common case: user locks, realises they
        // forgot something, re-opens within a few seconds.  Must NOT
        // trip the alarm.
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        settle(5_000).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle(50).await;

        assert!(
            bus.history()
                .iter()
                .all(|(s, v)| !(*s == ALARM_STATUS && *v == SignalValue::Bool(true))),
            "ALARM_STATUS must stay false during pre-arm window"
        );
        assert!(
            bus.history()
                .iter()
                .all(|(s, v)| !(*s == CHIME && *v == SignalValue::Bool(true))),
            "CHIME must not pulse during pre-arm window"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn door_open_just_after_pre_arm_window_arms_alarm() {
        // Boundary case: a door-open at 20 s + 1 ms past lock should
        // trip the alarm (chime phase begins).
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        settle(20_001).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle(50).await;

        assert_eq!(bus.latest_value(CHIME), Some(SignalValue::Bool(true)));
    }

    #[tokio::test(start_paused = true)]
    async fn trim_unlock_while_armed_triggers_alarm() {
        // Cabin armed (locked + 21 s elapsed past pre-arm), then an
        // interior trim Unlock button fires.  The arbiter publishes
        // UNLOCKED + LastRequestor = "DoorTrimButton".  PerimeterAlarm
        // must escalate into the chime phase as if a door had been
        // forced open.
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        settle(21_000).await;

        // Internal unlock event delivered as a coherent tuple via the
        // helper.  Both the natural status→requestor order and the
        // reversed order are covered by the dedicated helper test
        // below; here we just want to verify the trigger fires.
        inject_lock(&bus, "UNLOCKED", "DoorTrimButton", 2).await;
        settle(50).await;

        // Chime phase started.
        assert_eq!(bus.latest_value(CHIME), Some(SignalValue::Bool(true)));
        // ALARM_STATUS still false — chime hasn't escalated yet.
        assert_ne!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(true))
        );
    }

    #[tokio::test(start_paused = true)]
    async fn trim_unlock_while_armed_triggers_alarm_requestor_first() {
        // Same as above but with the requestor published BEFORE the
        // status — the FSM must still trigger the chime because all
        // lock-event-driven transitions live in event_num_rx, which
        // fires last and observes a coherent (status, requestor)
        // tuple regardless of the order the prior two arrived.
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        settle(21_000).await;

        // Reversed order: requestor first, then status, then event_num.
        bus.inject(LAST_REQUESTOR, SignalValue::String("DoorTrimButton".into()));
        for _ in 0..4 {
            tokio::task::yield_now().await;
        }
        bus.inject(LOCK_STATUS, SignalValue::String("UNLOCKED".into()));
        for _ in 0..4 {
            tokio::task::yield_now().await;
        }
        bus.inject(LOCK_EVENT_NUM, SignalValue::Uint16(2));
        settle(50).await;

        assert_eq!(bus.latest_value(CHIME), Some(SignalValue::Bool(true)));
    }

    #[tokio::test(start_paused = true)]
    async fn trim_unlock_during_pre_arm_does_not_trigger() {
        // Inside the 20 s pre-arm window, even a "tampering"-shaped
        // unlock from DoorTrimButton must NOT trigger the alarm —
        // matches the door-open suppression in pre-arm.
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        settle(5_000).await;

        inject_lock(&bus, "UNLOCKED", "DoorTrimButton", 2).await;
        settle(100).await;

        assert!(
            bus.history()
                .iter()
                .all(|(s, v)| !(*s == CHIME && *v == SignalValue::Bool(true))),
            "trim unlock during pre-arm must not trigger the chime"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn auth_unlock_while_armed_does_not_trigger_alarm() {
        // Sanity: a legitimate authenticated unlock at exactly the
        // same moment in the lock cycle that would trigger from
        // DoorTrimButton must be a clean no-op.
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        settle(21_000).await;

        inject_lock(&bus, "UNLOCKED", "KeyfobRke", 2).await;
        settle(100).await;

        assert!(
            bus.history()
                .iter()
                .all(|(s, v)| !(*s == CHIME && *v == SignalValue::Bool(true))),
            "auth unlock must not trigger the alarm"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn ignition_on_during_chime_disarms_silently() {
        // Driver entered with mech-blade key, chime is warning, then
        // they crank the engine — proves legitimate access; cancel
        // the chime cleanly with no exterior light show.
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        settle(21_000).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle(2_000).await;
        // Mid-chime — chime is pulsing, horn / lights not yet active.
        assert_eq!(bus.latest_value(HORN), None);

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle(50).await;

        assert_eq!(bus.latest_value(CHIME), Some(SignalValue::Bool(false)));
        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(false))
        );
        // Horn never activated during chime — no light show happened.
        settle(20_000).await;
        assert_eq!(bus.latest_value(HORN), None);
    }

    #[tokio::test(start_paused = true)]
    async fn ignition_on_during_full_alarm_disarms() {
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        settle(21_000).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        // Past the chime → full alarm phase running.
        settle(13_000).await;
        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(true))
        );

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle(50).await;

        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(false))
        );
        assert_eq!(bus.latest_value(HORN), Some(SignalValue::Bool(false)));
    }

    #[tokio::test(start_paused = true)]
    async fn ignition_acc_during_chime_does_not_disarm() {
        // ACC alone does NOT cancel the alarm — jigglable from a
        // pried cylinder.  Only ON / START prove legit access.
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        settle(21_000).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle(2_000).await;

        bus.inject(POWER_STATE, SignalValue::String("ACC".into()));
        settle(50).await;

        // Chime still pulsing — escalation will still happen at 12 s.
        assert_eq!(bus.latest_value(CHIME), Some(SignalValue::Bool(true)));
    }

    #[tokio::test(start_paused = true)]
    async fn ignition_on_when_no_alarm_active_is_a_noop() {
        // The handler must only fire when an alarm sequence is in
        // flight — a routine engine start while idle should not
        // republish ALARM_STATUS=false (no claim was ever made).
        let bus = setup().await;
        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle(50).await;

        assert!(
            bus.history().iter().all(|(s, _)| *s != ALARM_STATUS),
            "idle ignition-on must not touch ALARM_STATUS"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn unlock_during_pre_arm_clears_window() {
        // Lock → unlock within 5 s → re-lock → wait 6 s → open door.
        // Total elapsed since first lock is 11 s + 6 s = 17 s, but the
        // pre-arm timer was reset on unlock so only 6 s have elapsed
        // since the latest lock.  The door open must NOT trip the alarm.
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        settle(5_000).await;
        inject_lock(&bus, "UNLOCKED", "KeyfobRke", 2).await;
        settle(5_000).await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 3).await;
        settle(6_000).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle(50).await;

        assert!(
            bus.history()
                .iter()
                .all(|(s, v)| !(*s == CHIME && *v == SignalValue::Bool(true))),
            "pre-arm window must reset on each lock; door-open at 6 s into the second lock must not trip"
        );
    }

    // ── Vehicle.Body.Alarm.State authoritative state-machine tests ──────

    #[tokio::test(start_paused = true)]
    async fn state_machine_walks_disarmed_prearmed_armed_activated() {
        // Boot → DISARMED, lock → PRE_ARMED, +20 s → ARMED, door-open
        // → ACTIVATED.  Single end-to-end pass through every state.
        let bus = setup().await;
        // After boot.
        settle(10).await;
        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("DISARMED".into()))
        );

        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        settle(50).await;
        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("PRE_ARMED".into()))
        );

        // Past the 20 s pre-arm window.
        settle(20_500).await;
        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("ARMED".into()))
        );

        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle(50).await;
        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("ACTIVATED".into()))
        );
    }

    #[tokio::test(start_paused = true)]
    async fn ignition_live_forces_disarmed_state_during_chime() {
        // Ignition ON proves immobiliser pass; the alarm goes to
        // DISARMED regardless of lock_status (which is still LOCKED).
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        settle(21_000).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle(2_000).await;
        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("ACTIVATED".into()))
        );

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle(100).await;

        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("DISARMED".into())),
            "ignition-cancelled chime must drop straight to DISARMED — engine running ⇒ immobiliser passed"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn ignition_live_forces_disarmed_and_does_not_auto_rearm() {
        // No alarm in flight, just LOCKED + ARMED.  Cranking the engine
        // forces DISARMED.  Turning ignition back OFF must NOT
        // auto-rearm — the user has to actively re-lock to leave
        // DISARMED.  This is the sticky-disarm guarantee.
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        settle(21_000).await;
        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("ARMED".into()))
        );

        bus.inject(POWER_STATE, SignalValue::String("ON".into()));
        settle(100).await;
        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("DISARMED".into()))
        );

        // Ignition back to OFF — cabin still LOCKED, but the armed
        // session was cleared by the live transition above.  State
        // must stay DISARMED until a fresh external lock event.
        bus.inject(POWER_STATE, SignalValue::String("OFF".into()));
        settle(100).await;
        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("DISARMED".into())),
            "ignition off after a disarm must NOT auto-rearm — sticky disarm requires a fresh lock event"
        );

        // Fresh lock cycle: unlock → re-lock.  Now we should be back
        // in PRE_ARMED, then ARMED after the 20 s window.
        inject_lock(&bus, "UNLOCKED", "KeyfobRke", 2).await;
        settle(50).await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 3).await;
        settle(50).await;
        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("PRE_ARMED".into()))
        );
        settle(20_500).await;
        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("ARMED".into()))
        );
    }

    #[tokio::test(start_paused = true)]
    async fn internal_lock_sources_do_not_arm() {
        // AutoLock (drive-away auto-lock) and DoorTrimButton (interior
        // trim switch) are not user-initiated "I am leaving the
        // vehicle" events.  Locking via either must NOT take the alarm
        // out of DISARMED — the rule is that arming requires a
        // deliberate external lock action (RKE, ThumbPadLock,
        // AutoRelock, WalkAwayLock, paired phone, NFC).
        for internal_requestor in ["AutoLock", "DoorTrimButton"] {
            let bus = setup().await;
            inject_lock(&bus, "LOCKED", internal_requestor, 1).await;
            // Wait well past the 20 s pre-arm window — if we'd
            // mistakenly armed, this is more than enough time to
            // surface ARMED in the published state.
            settle(25_000).await;
            assert_eq!(
                bus.latest_value(ALARM_STATE),
                Some(SignalValue::String("DISARMED".into())),
                "internal lock requestor {} must not arm the alarm",
                internal_requestor
            );
        }
    }

    #[tokio::test(start_paused = true)]
    async fn external_lock_sources_arm() {
        // Inverse of the test above: each external lock source must
        // take us into PRE_ARMED → ARMED.
        for external_requestor in [
            "KeyfobRke",
            "KeyfobPeps",
            "ThumbPadLock",
            "AutoRelock",
            "WalkAwayLock",
            "PhoneApp",
            "PhoneBle",
            "NfcCard",
            "NfcPhone",
        ] {
            let bus = setup().await;
            inject_lock(&bus, "LOCKED", external_requestor, 1).await;
            settle(50).await;
            assert_eq!(
                bus.latest_value(ALARM_STATE),
                Some(SignalValue::String("PRE_ARMED".into())),
                "external lock requestor {} must arm the alarm",
                external_requestor
            );
        }
    }

    // ── Slam-lock subsystem cross-feature checks ────────────────────────

    #[tokio::test(start_paused = true)]
    async fn slam_lock_arms_alarm_us_path() {
        // US slam-lock-allowed flow: a `LockAll` event with requestor
        // `SlamLock` (the requestor that DoorTrimButton stamps when
        // it sees a trim-lock-with-door-open under cal=false) must
        // take the alarm DISARMED → PRE_ARMED → ARMED.  Ensures
        // SlamLock is in EXTERNAL_LOCK_REQUESTORS.
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "SlamLock", 1).await;
        settle(50).await;
        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("PRE_ARMED".into()))
        );
        settle(20_500).await;
        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("ARMED".into()))
        );
    }

    #[tokio::test(start_paused = true)]
    async fn slam_lock_intruder_chime_survives_us_slamlock_path() {
        // Hostile-input regression: vehicle ARMED, thief smashes window,
        // hits trim UNLOCK, opens the door, then hits trim LOCK during
        // the chime.  US cal: DoorTrimButton dispatches the LOCK as
        // `SlamLock`.  This must NOT disarm the alarm.
        //
        // Prior to this test, `SlamLock` was *only* in
        // EXTERNAL_LOCK_REQUESTORS; if anyone later "tidies up" by
        // adding it to EXTERNAL_AUTH_SOURCES this test will scream.
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        settle(21_000).await; // → ARMED

        // Thief: trim UNLOCK → tampering trigger → chime starts.
        inject_lock(&bus, "UNLOCKED", "DoorTrimButton", 2).await;
        settle(50).await;
        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("ACTIVATED".into()))
        );

        // Mid-chime: thief opens door (door event ignored in ACTIVATED),
        // then hits trim LOCK.  US cal → DoorTrimButton dispatches as
        // SlamLock.  Bus delivers (LOCKED, SlamLock, 3).
        settle(2_000).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        inject_lock(&bus, "LOCKED", "SlamLock", 3).await;
        settle(50).await;

        // Alarm must still be ACTIVATED.  None of the disarm gates
        // (auth unlock / panic / ignition / armable→non-armable
        // cleanup) should have fired.
        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("ACTIVATED".into())),
            "thief's trim-lock during chime must NOT disarm under US slam-lock cal"
        );

        // Chime escalates to full alarm at 12 s; verify by advancing
        // past the chime window.
        settle(11_000).await;
        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(true)),
            "alarm must escalate to full alarm phase as expected"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn slam_lock_intruder_chime_survives_eu_slamlock_protection() {
        // EU mirror of the test above.  Vehicle ARMED, thief breaks in,
        // chime starts.  Mid-chime, thief hits trim LOCK with door open.
        // EU cal flow:
        //   * DoorTrimButton dispatches LockAll as DoorTrimButton (event N+1)
        //   * SlamLock then dispatches the inversion as SlamLock (event N+2)
        // Neither event must disarm the alarm.
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        settle(21_000).await; // → ARMED

        // Trim UNLOCK starts the chime (tampering).
        inject_lock(&bus, "UNLOCKED", "DoorTrimButton", 2).await;
        settle(50).await;
        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("ACTIVATED".into()))
        );

        // Mid-chime: door open, trim LOCK pressed.  EU two-event
        // sequence on the bus.
        settle(2_000).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        inject_lock(&bus, "LOCKED", "DoorTrimButton", 3).await;
        inject_lock(&bus, "UNLOCKED", "SlamLock", 4).await;
        settle(50).await;

        // Alarm must STILL be ACTIVATED — neither DoorTrimButton's
        // brief lock event (internal-class) nor SlamLock's unlock
        // event (neither auth nor internal-tampering) qualifies as a
        // disarm path.
        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("ACTIVATED".into())),
            "thief's slam-lock-protect inversion must NOT disarm a running chime"
        );

        // Chime escalates to full alarm normally.
        settle(11_000).await;
        assert_eq!(
            bus.latest_value(ALARM_STATUS),
            Some(SignalValue::Bool(true)),
            "alarm must escalate even after the SlamLock inversion fired"
        );
    }

    #[tokio::test(start_paused = true)]
    async fn ignition_acc_does_not_force_disarmed() {
        // ACC is jigglable from a stolen blade key — must not force
        // the alarm into DISARMED.
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        settle(21_000).await;
        bus.inject(POWER_STATE, SignalValue::String("ACC".into()));
        settle(100).await;
        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("ARMED".into()))
        );
    }

    #[tokio::test(start_paused = true)]
    async fn state_returns_to_disarmed_on_auth_unlock() {
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        settle(21_000).await;
        bus.inject("Body.Doors.Row1.Left.IsOpen", SignalValue::Bool(true));
        settle(13_000).await;
        // Past chime → ACTIVATED.
        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("ACTIVATED".into()))
        );

        inject_lock(&bus, "UNLOCKED", "KeyfobRke", 2).await;
        settle(100).await;

        assert_eq!(
            bus.latest_value(ALARM_STATE),
            Some(SignalValue::String("DISARMED".into()))
        );
    }

    #[tokio::test(start_paused = true)]
    async fn state_does_not_republish_on_redundant_events() {
        // Redundant lock-status updates with the same value must NOT
        // emit duplicate state publishes — the bus history would
        // bloat and the HMI would see useless state-change blips.
        let bus = setup().await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        settle(50).await;
        // Re-publish the same value.
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        inject_lock(&bus, "LOCKED", "KeyfobRke", 1).await;
        settle(50).await;

        let publishes: Vec<_> = bus
            .history()
            .iter()
            .filter(|(s, _)| *s == ALARM_STATE)
            .cloned()
            .collect();
        // Boot DISARMED + first lock PRE_ARMED = 2 expected publishes.
        assert_eq!(
            publishes.len(),
            2,
            "redundant lock-status updates must not re-publish state: {:?}",
            publishes
        );
    }
}
