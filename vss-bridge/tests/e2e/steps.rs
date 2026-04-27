//! Cucumber World + step definitions for the VSS body-controller E2E tests.
//!
//! Tier 1 — in-process: the full feature stack runs on a `MockBus` inside the
//! test process with Tokio virtual time (`start_paused`).  Step assertions
//! verify observable effects on the bus (published signal values), not internal
//! arbiter state.  This is deliberate: the gherkin scenarios describe *what*
//! the system does; these steps verify *that it did it*.

use std::sync::Arc;
use std::time::Duration;

use cucumber::{given, then, when, World};
use tokio::time::advance;

use vss_bridge::adapters::mock::MockBus;
use vss_bridge::arbiter::{self, DomainArbiter};
use vss_bridge::features::hazard_lighting::HazardLighting;
use vss_bridge::features::panic_alarm::PanicAlarm;
use vss_bridge::features::turn_indicator::TurnIndicator;
use vss_bridge::ipc_message::SignalValue;
use vss_bridge::plant_models::blink_relay::BlinkRelay;
use vss_bridge::signal_bus::VssPath;

// ---------------------------------------------------------------------------
// Signal constants (mirror the feature modules)
// ---------------------------------------------------------------------------

const LEFT_SIGNALING: VssPath = "Body.Lights.DirectionIndicator.Left.IsSignaling";
const RIGHT_SIGNALING: VssPath = "Body.Lights.DirectionIndicator.Right.IsSignaling";
const HORN: VssPath = "Body.Horn.IsActive";
const PANIC_SWITCH: VssPath = "Body.Switches.Panic.IsEngaged";
const ALARM_STATUS: VssPath = "Vehicle.Body.Alarm.IsActive";

// PanicAlarm pulse cadence — must mirror src/features/panic_alarm.rs.
const PANIC_ON_MS: u64 = 400;
const PANIC_OFF_MS: u64 = 600;

// ---------------------------------------------------------------------------
// World
// ---------------------------------------------------------------------------

#[derive(Default, World)]
#[world(init = Self::new)]
pub struct VssWorld {
    bus: Option<Arc<MockBus>>,
    _arbiter: Option<Arc<DomainArbiter>>,
    _tasks: Vec<tokio::task::JoinHandle<()>>,
    started: bool,
}

impl std::fmt::Debug for VssWorld {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("VssWorld")
            .field("started", &self.started)
            .field("tasks", &self._tasks.len())
            .finish()
    }
}

impl Drop for VssWorld {
    fn drop(&mut self) {
        // Abort all spawned tasks to prevent zombie tasks from accumulating
        // across scenarios in the shared tokio runtime. Zombie timers from
        // old BlinkRelay tasks would fire during advance() calls in later
        // scenarios, corrupting virtual-time determinism.
        for task in self._tasks.drain(..) {
            task.abort();
        }
    }
}

impl VssWorld {
    async fn new() -> Self {
        Self::default()
    }

    /// Boot the full feature stack exactly once per scenario.
    async fn ensure_started(&mut self) {
        if self.started {
            return;
        }

        let bus = Arc::new(MockBus::new());
        let (arb, arb_fut) = arbiter::lighting_arbiter(Arc::clone(&bus));
        let (horn_arb, horn_fut) = arbiter::horn_arbiter(Arc::clone(&bus));
        self._tasks.push(tokio::spawn(arb_fut));
        self._tasks.push(tokio::spawn(horn_fut));
        let arb = Arc::new(arb);
        let horn_arb = Arc::new(horn_arb);

        self._tasks.push(tokio::spawn(
            TurnIndicator::new(Arc::clone(&arb), Arc::clone(&bus)).run(),
        ));
        self._tasks.push(tokio::spawn(
            HazardLighting::new(Arc::clone(&arb), Arc::clone(&bus)).run(),
        ));
        self._tasks.push(tokio::spawn(
            PanicAlarm::new(Arc::clone(&arb), Arc::clone(&horn_arb), Arc::clone(&bus)).run(),
        ));
        self._tasks
            .push(tokio::spawn(BlinkRelay::new(Arc::clone(&bus)).run()));

        // Yield enough times for every spawned task to reach its first
        // `.subscribe().await` so injections don't get lost.
        settle().await;

        self.bus = Some(bus);
        self._arbiter = Some(arb);
        self.started = true;
    }

    fn bus(&self) -> &Arc<MockBus> {
        self.bus.as_ref().expect("scenario did not start the stack")
    }

    /// Inject a signal into the MockBus and let the system settle.
    async fn inject(&self, path: VssPath, val: SignalValue) {
        self.bus().inject(path, val);
        settle().await;
    }

    /// Current resolved value for `path` — survives `clear_history()`.
    fn current_value(&self, path: VssPath) -> Option<SignalValue> {
        self.bus().latest_value(path)
    }

    /// Count publishes to `path` since the last `clear_history`.
    fn publish_count(&self, path: VssPath) -> usize {
        self.bus()
            .history()
            .iter()
            .filter(|(s, _)| *s == path)
            .count()
    }
}

/// Yield + small advance so every spawned task processes pending messages.
async fn settle() {
    for _ in 0..8 {
        tokio::task::yield_now().await;
    }
    advance(Duration::from_millis(1)).await;
    for _ in 0..4 {
        tokio::task::yield_now().await;
    }
}

/// Advance time for N complete flash cycles at the normal blink rate
/// (333ms half-period = 666ms per flash cycle).
///
/// Uses a single `advance(666ms)` per cycle rather than two 333ms
/// advances with yields in between.  Splitting into half-periods with
/// `yield_now()` between them is unsafe: tokio's auto-advance in paused
/// mode can fire intermediate BlinkRelay timers during yields when all
/// *other* tasks are idle, advancing the clock by extra 333ms increments
/// and producing spurious flash counts.
async fn advance_flashes(n: u32) {
    for _ in 0..n {
        advance(Duration::from_millis(666)).await;
    }
    settle().await;
}

/// Map a short indicator name to its full VSS path.
fn indicator_path(name: &str) -> VssPath {
    match name {
        "Left" | "left" => LEFT_SIGNALING,
        "Right" | "right" => RIGHT_SIGNALING,
        _ => panic!("unknown indicator side: {name}"),
    }
}

// ===========================================================================
// GIVEN steps
// ===========================================================================

// ---- Infrastructure (Background) ----

#[given("the Lighting domain arbiter is running")]
#[given("the Turn Indicator feature is running")]
#[given("the Hazard feature is running")]
async fn infrastructure(w: &mut VssWorld) {
    w.ensure_started().await;
}

// ---- Ignition state ----

#[given(regex = r#"^the vehicle low-voltage system is in state "(ON|OFF|ACC|START)"$"#)]
async fn given_ignition(w: &mut VssWorld, state: String) {
    w.ensure_started().await;
    w.inject("Vehicle.LowVoltageSystemState", SignalValue::String(state))
        .await;
}

// ---- Turn stalk position ----

#[given(regex = r#"^the turn stalk is in position (OFF|LEFT|RIGHT)$"#)]
async fn given_stalk(w: &mut VssWorld, dir: String) {
    w.ensure_started().await;
    // Turn feature requires ignition ON/START — ensure it's set so
    // the stalk position actually produces a signal. If ignition was
    // explicitly set to OFF/ACC by a prior Given step in this scenario,
    // that step runs before this one and will override after.
    let ign = w.current_value("Vehicle.LowVoltageSystemState");
    if ign.is_none() {
        w.inject(
            "Vehicle.LowVoltageSystemState",
            SignalValue::String("ON".to_string()),
        )
        .await;
    }
    w.inject(
        "Body.Switches.TurnIndicator.Direction",
        SignalValue::String(dir),
    )
    .await;
}

// ---- Hazard switch ----

#[given("the hazard switch is not engaged")]
async fn given_hazard_off(w: &mut VssWorld) {
    w.ensure_started().await;
    w.inject("Body.Switches.Hazard.IsEngaged", SignalValue::Bool(false))
        .await;
}

#[given("the hazard switch is engaged")]
#[given("the hazard switch is engaged (overriding the turn signal)")]
async fn given_hazard_on(w: &mut VssWorld) {
    w.ensure_started().await;
    w.inject("Body.Switches.Hazard.IsEngaged", SignalValue::Bool(true))
        .await;
}

// ---- Pre-condition: indicator already signaling ----
// These are implied by the preceding Given (stalk/hazard) — just settle.

#[given("the left direction indicator is signaling")]
#[given("the left direction indicator is signaling at priority MEDIUM")]
async fn given_left_signaling(w: &mut VssWorld) {
    settle().await;
    assert_eq!(
        w.current_value(LEFT_SIGNALING),
        Some(SignalValue::Bool(true)),
        "expected Left.IsSignaling = TRUE as precondition"
    );
}

#[given("the right direction indicator is signaling at priority MEDIUM")]
async fn given_right_signaling(w: &mut VssWorld) {
    settle().await;
    assert_eq!(
        w.current_value(RIGHT_SIGNALING),
        Some(SignalValue::Bool(true)),
        "expected Right.IsSignaling = TRUE as precondition"
    );
}

#[given("both direction indicators are signaling due to hazard")]
#[given("both indicators are signaling at priority HIGH")]
#[given("both direction indicators are signaling at priority HIGH")]
async fn given_both_signaling(w: &mut VssWorld) {
    settle().await;
    assert_eq!(
        w.current_value(LEFT_SIGNALING),
        Some(SignalValue::Bool(true)),
        "expected Left.IsSignaling = TRUE as precondition"
    );
    assert_eq!(
        w.current_value(RIGHT_SIGNALING),
        Some(SignalValue::Bool(true)),
        "expected Right.IsSignaling = TRUE as precondition"
    );
}

#[given(regex = r#"^Vehicle.LowVoltageSystemState was "([^"]+)" \(turn inactive\)$"#)]
async fn given_ignition_was(w: &mut VssWorld, state: String) {
    w.inject("Vehicle.LowVoltageSystemState", SignalValue::String(state))
        .await;
}

// ===========================================================================
// WHEN steps
// ===========================================================================

#[when(regex = r#"^the driver moves the turn stalk to (OFF|LEFT|RIGHT)$"#)]
#[when(regex = r#"^the driver returns the turn stalk to (OFF|LEFT|RIGHT)$"#)]
async fn when_stalk(w: &mut VssWorld, dir: String) {
    w.bus().clear_history();
    w.inject(
        "Body.Switches.TurnIndicator.Direction",
        SignalValue::String(dir),
    )
    .await;
}

#[when("the driver engages the hazard switch")]
async fn when_hazard_on(w: &mut VssWorld) {
    w.bus().clear_history();
    w.inject("Body.Switches.Hazard.IsEngaged", SignalValue::Bool(true))
        .await;
}

#[when("the driver disengages the hazard switch")]
async fn when_hazard_off(w: &mut VssWorld) {
    w.bus().clear_history();
    w.inject("Body.Switches.Hazard.IsEngaged", SignalValue::Bool(false))
        .await;
}

#[when(regex = r#"^Vehicle.LowVoltageSystemState transitions to "(ON|OFF|ACC|START)"$"#)]
async fn when_ignition(w: &mut VssWorld, state: String) {
    w.bus().clear_history();
    w.inject("Vehicle.LowVoltageSystemState", SignalValue::String(state))
        .await;
}

// ---- Comfort blink timing steps ----

#[when(regex = r"^(\d+) complete flash cycles? elapses?$")]
async fn when_flash_cycles(_w: &mut VssWorld, count: u32) {
    advance_flashes(count).await;
}

// Lock Feedback When/Then steps are intentionally NOT defined here.
// cucumber-rs treats unmatched steps as "skipped" (yellow/pending),
// which correctly signals that this scenario is not yet testable.

// ===========================================================================
// THEN steps
// ===========================================================================

// ---- Sensor echo (verifies the bus injection path) ----

#[then(regex = r#"^Body\.Switches\.TurnIndicator\.Direction becomes "(OFF|LEFT|RIGHT)"$"#)]
async fn then_stalk_becomes(w: &mut VssWorld, dir: String) {
    let last = w.current_value("Body.Switches.TurnIndicator.Direction");
    assert_eq!(
        last,
        Some(SignalValue::String(dir.clone())),
        "expected stalk direction = {dir}"
    );
}

#[then("Body.Switches.Hazard.IsEngaged becomes TRUE")]
async fn then_hazard_becomes_true(w: &mut VssWorld) {
    let last = w.current_value("Body.Switches.Hazard.IsEngaged");
    assert_eq!(last, Some(SignalValue::Bool(true)));
}

#[then("Body.Switches.Hazard.IsEngaged becomes FALSE")]
async fn then_hazard_becomes_false(w: &mut VssWorld) {
    let last = w.current_value("Body.Switches.Hazard.IsEngaged");
    assert_eq!(last, Some(SignalValue::Bool(false)));
}

// ---- Feature requests (observable effect: signal becomes TRUE) ----

#[then(
    regex = r"^the Turn feature requests DirectionIndicator\.(Left|Right)\.IsSignaling = TRUE at priority MEDIUM$"
)]
#[then(
    regex = r"^the Hazard feature requests DirectionIndicator\.(Left|Right)\.IsSignaling = TRUE at priority HIGH$"
)]
async fn then_indicator_true(w: &mut VssWorld, side: String) {
    let path = indicator_path(&side);
    assert_eq!(
        w.current_value(path),
        Some(SignalValue::Bool(true)),
        "expected {side}.IsSignaling = TRUE"
    );
}

// ---- Feature releases (observable: signal becomes FALSE if no other claim) ----

#[then(
    regex = r"^the Turn feature releases its claim on DirectionIndicator\.(Left|Right)\.IsSignaling$"
)]
#[then(
    regex = r"^the Hazard feature releases its claim on DirectionIndicator\.(Left|Right)\.IsSignaling$"
)]
async fn then_indicator_released(w: &mut VssWorld, side: String) {
    // A release doesn't necessarily mean FALSE — another feature's claim
    // may keep it TRUE. We verify by checking the resolved state:
    // if only Turn had a claim and released, the arbiter publishes
    // default-off. If Hazard still holds, it stays TRUE.
    // So we just record that the step ran — the *combined* outcome
    // is verified by the subsequent steps in the scenario.
    let path = indicator_path(&side);
    let _current = w.current_value(path);
    // No assertion — the observable effect depends on what other claims
    // exist, which is asserted by the next Then step(s).
}

// ---- Comfort blink: indicator continues signaling during countdown ----

#[then(
    regex = r"^the (left|right) direction indicator continues signaling during comfort blink countdown$"
)]
async fn then_indicator_still_signaling(w: &mut VssWorld, side: String) {
    let path = indicator_path(&side);
    assert_eq!(
        w.current_value(path),
        Some(SignalValue::Bool(true)),
        "{side}.IsSignaling should remain TRUE during comfort blink"
    );
}

// ---- Hazard disengaged → default-off when no other claim ----

#[then(
    "with no other active claim, the arbiter publishes the default-off value on both indicators"
)]
async fn then_both_default_off(w: &mut VssWorld) {
    assert_eq!(
        w.current_value(LEFT_SIGNALING),
        Some(SignalValue::Bool(false)),
        "Left should be default-off"
    );
    assert_eq!(
        w.current_value(RIGHT_SIGNALING),
        Some(SignalValue::Bool(false)),
        "Right should be default-off"
    );
}

// ---- Both indicators signaling (hazard active) ----

#[then("both direction indicators signal at priority HIGH")]
#[then("both direction indicators continue signaling at priority HIGH")]
async fn then_both_signaling(w: &mut VssWorld) {
    assert_eq!(
        w.current_value(LEFT_SIGNALING),
        Some(SignalValue::Bool(true)),
        "Left.IsSignaling should be TRUE"
    );
    assert_eq!(
        w.current_value(RIGHT_SIGNALING),
        Some(SignalValue::Bool(true)),
        "Right.IsSignaling should be TRUE"
    );
}

// ---- Turn suppressed while hazard active ----

#[then("the Turn feature's MEDIUM claim on DirectionIndicator.Left.IsSignaling is recorded by the arbiter")]
async fn then_turn_claim_recorded(w: &mut VssWorld) {
    // The claim is recorded internally; observable effect is that Left
    // remains TRUE (Hazard HIGH still wins). Assert that:
    assert_eq!(
        w.current_value(LEFT_SIGNALING),
        Some(SignalValue::Bool(true)),
        "Left.IsSignaling should still be TRUE (Hazard HIGH wins)"
    );
}

#[then("the turn signal's MEDIUM request is suppressed by the arbiter")]
async fn then_turn_suppressed(_w: &mut VssWorld) {
    // Suppressed means no visible change — the preceding step already
    // verified indicators are still TRUE from Hazard's claims.
}

#[then("both indicators continue signaling because Hazard's HIGH claims win arbitration")]
async fn then_hazard_wins(w: &mut VssWorld) {
    assert_eq!(
        w.current_value(LEFT_SIGNALING),
        Some(SignalValue::Bool(true)),
    );
    assert_eq!(
        w.current_value(RIGHT_SIGNALING),
        Some(SignalValue::Bool(true)),
    );
}

// ---- Turn resumes after hazard releases ----

#[then("the Hazard feature releases both indicators at priority HIGH")]
async fn then_hazard_releases_both(_w: &mut VssWorld) {
    // Internal action — observable consequence is that the pending
    // Turn MEDIUM claim takes effect (next step).
}

#[then("the Turn feature's pending LEFT request at priority MEDIUM takes effect")]
async fn then_turn_resumes_left(w: &mut VssWorld) {
    assert_eq!(
        w.current_value(LEFT_SIGNALING),
        Some(SignalValue::Bool(true)),
        "Left should be TRUE — Turn MEDIUM resumed"
    );
    assert_eq!(
        w.current_value(RIGHT_SIGNALING),
        Some(SignalValue::Bool(false)),
        "Right should be FALSE — only Turn LEFT is active"
    );
}

// ---- No blink timing from feature ----

#[then("the Hazard feature publishes IsSignaling = TRUE once")]
async fn then_hazard_publishes_once(w: &mut VssWorld) {
    let left_count = w.publish_count(LEFT_SIGNALING);
    let right_count = w.publish_count(RIGHT_SIGNALING);
    assert_eq!(left_count, 1, "Left.IsSignaling should be published once");
    assert_eq!(right_count, 1, "Right.IsSignaling should be published once");
}

#[then("the Hazard feature does NOT publish periodic on/off toggles")]
async fn then_no_periodic_toggles(w: &mut VssWorld) {
    // Wait 1 second (virtual) and check no additional publishes happen
    // on IsSignaling (the blink relay publishes Lamp.IsOn, not IsSignaling).
    let before_left = w.publish_count(LEFT_SIGNALING);
    let before_right = w.publish_count(RIGHT_SIGNALING);

    advance(Duration::from_secs(1)).await;
    for _ in 0..4 {
        tokio::task::yield_now().await;
    }

    let after_left = w.publish_count(LEFT_SIGNALING);
    let after_right = w.publish_count(RIGHT_SIGNALING);
    assert_eq!(
        before_left, after_left,
        "no extra Left.IsSignaling publishes"
    );
    assert_eq!(
        before_right, after_right,
        "no extra Right.IsSignaling publishes"
    );
}

// ---- Turn does NOT act when ignition is off/acc ----

#[then("the Turn feature does NOT request any indicator change")]
async fn then_no_change(w: &mut VssWorld) {
    let left_count = w.publish_count(LEFT_SIGNALING);
    let right_count = w.publish_count(RIGHT_SIGNALING);
    assert_eq!(left_count, 0, "no Left.IsSignaling publish expected");
    assert_eq!(right_count, 0, "no Right.IsSignaling publish expected");
}

// Lock Feedback Then steps are also intentionally NOT defined here.
// See note on When steps above.

// ===========================================================================
// Panic Alarm — Background, Given, When, Then steps
// ===========================================================================

// ---- Background ----
// (The "Lighting domain arbiter is running" Given is shared above.)

#[given("the Horn domain arbiter is running")]
#[given("the PanicAlarm feature is running")]
async fn panic_infrastructure(w: &mut VssWorld) {
    w.ensure_started().await;
}

// ---- Given: panic switch state ----

#[given("the panic switch is not engaged")]
async fn given_panic_off(w: &mut VssWorld) {
    w.ensure_started().await;
    w.inject(PANIC_SWITCH, SignalValue::Bool(false)).await;
}

#[given("the panic switch is engaged")]
async fn given_panic_on(w: &mut VssWorld) {
    w.ensure_started().await;
    w.inject(PANIC_SWITCH, SignalValue::Bool(true)).await;
    // Settle into the first ON window so subsequent steps see active claims.
    settle().await;
}

#[given("Vehicle.Body.Alarm.IsActive is TRUE")]
async fn given_alarm_status_true(w: &mut VssWorld) {
    settle().await;
    assert_eq!(
        w.current_value(ALARM_STATUS),
        Some(SignalValue::Bool(true)),
        "expected Vehicle.Body.Alarm.IsActive = TRUE as precondition"
    );
}

#[given("both indicators and the horn are active under PanicAlarm")]
async fn given_panic_outputs_active(w: &mut VssWorld) {
    settle().await;
    assert_eq!(
        w.current_value(LEFT_SIGNALING),
        Some(SignalValue::Bool(true)),
        "expected Left.IsSignaling = TRUE as precondition"
    );
    assert_eq!(
        w.current_value(RIGHT_SIGNALING),
        Some(SignalValue::Bool(true)),
        "expected Right.IsSignaling = TRUE as precondition"
    );
    assert_eq!(
        w.current_value(HORN),
        Some(SignalValue::Bool(true)),
        "expected Body.Horn.IsActive = TRUE as precondition"
    );
}

#[given("both indicators are signaling at priority HIGH due to hazard")]
async fn given_hazard_high(w: &mut VssWorld) {
    settle().await;
    assert_eq!(
        w.current_value(LEFT_SIGNALING),
        Some(SignalValue::Bool(true))
    );
    assert_eq!(
        w.current_value(RIGHT_SIGNALING),
        Some(SignalValue::Bool(true))
    );
}

/// Engage/disengage helper — does an extra settle pass after `inject()` so
/// the spawned `pulse_loop`'s first claim has time to traverse:
///   PanicAlarm task → mpsc → arbiter loop → publish_resolved → bus.publish
/// (~8 awaits) before the first Then assertion runs.
async fn panic_engage_helper(w: &mut VssWorld, val: bool) {
    w.bus().clear_history();
    w.inject(PANIC_SWITCH, SignalValue::Bool(val)).await;
    settle().await;
    settle().await;
}

// ---- When: panic switch transitions ----

#[when("Body.Switches.Panic.IsEngaged transitions to TRUE")]
async fn when_panic_engage(w: &mut VssWorld) {
    panic_engage_helper(w, true).await;
}

#[when("Body.Switches.Panic.IsEngaged transitions to FALSE")]
async fn when_panic_disengage(w: &mut VssWorld) {
    panic_engage_helper(w, false).await;
}

#[when("Body.Switches.Panic.IsEngaged is set to TRUE again")]
async fn when_panic_re_engage(w: &mut VssWorld) {
    w.bus().clear_history();
    w.inject(PANIC_SWITCH, SignalValue::Bool(true)).await;
}

// ---- When: panic timing windows ----

#[when("the panic alarm has been running long enough to enter a complete OFF window")]
async fn when_into_off_window(_w: &mut VssWorld) {
    // After engage we are within the first ON window.  Advance past it
    // into the OFF window with a small margin to clear the boundary.
    advance(Duration::from_millis(PANIC_ON_MS + 50)).await;
    for _ in 0..8 {
        tokio::task::yield_now().await;
    }
}

#[when("the panic alarm advances into the next ON window")]
async fn when_into_next_on(_w: &mut VssWorld) {
    // From inside an OFF window, step past the OFF→ON edge.
    advance(Duration::from_millis(PANIC_OFF_MS + 50)).await;
    for _ in 0..8 {
        tokio::task::yield_now().await;
    }
}

#[when("the panic alarm runs through three complete pulse cycles")]
async fn when_three_cycles(_w: &mut VssWorld) {
    advance(Duration::from_millis((PANIC_ON_MS + PANIC_OFF_MS) * 3)).await;
    for _ in 0..16 {
        tokio::task::yield_now().await;
    }
}

// ---- Then: PanicAlarm requests ----

#[then(
    regex = r"^the PanicAlarm feature requests DirectionIndicator\.(Left|Right)\.IsSignaling = TRUE at priority HIGH$"
)]
async fn then_panic_indicator_true(w: &mut VssWorld, side: String) {
    let path = indicator_path(&side);
    assert_eq!(
        w.current_value(path),
        Some(SignalValue::Bool(true)),
        "expected {side}.IsSignaling = TRUE under PanicAlarm"
    );
}

#[then("the PanicAlarm feature requests Body.Horn.IsActive = TRUE at priority HIGH")]
async fn then_panic_horn_true(w: &mut VssWorld) {
    assert_eq!(
        w.current_value(HORN),
        Some(SignalValue::Bool(true)),
        "expected Body.Horn.IsActive = TRUE under PanicAlarm"
    );
}

// ---- Then: pulse window state ----

#[then("both direction indicators are OFF")]
async fn then_indicators_off(w: &mut VssWorld) {
    assert_eq!(
        w.current_value(LEFT_SIGNALING),
        Some(SignalValue::Bool(false)),
        "Left should be OFF in OFF window"
    );
    assert_eq!(
        w.current_value(RIGHT_SIGNALING),
        Some(SignalValue::Bool(false)),
        "Right should be OFF in OFF window"
    );
}

#[then("both direction indicators are ON")]
async fn then_indicators_on(w: &mut VssWorld) {
    assert_eq!(
        w.current_value(LEFT_SIGNALING),
        Some(SignalValue::Bool(true)),
        "Left should be ON in ON window"
    );
    assert_eq!(
        w.current_value(RIGHT_SIGNALING),
        Some(SignalValue::Bool(true)),
        "Right should be ON in ON window"
    );
}

#[then("Body.Horn.IsActive is FALSE")]
async fn then_horn_false(w: &mut VssWorld) {
    assert_eq!(
        w.current_value(HORN),
        Some(SignalValue::Bool(false)),
        "Horn should be FALSE in OFF window (in sync with lights)"
    );
}

#[then("Body.Horn.IsActive is TRUE")]
async fn then_horn_true(w: &mut VssWorld) {
    assert_eq!(
        w.current_value(HORN),
        Some(SignalValue::Bool(true)),
        "Horn should be TRUE in ON window (in sync with lights)"
    );
}

// ---- Then: alarm status flag ----

#[then("Vehicle.Body.Alarm.IsActive becomes TRUE")]
async fn then_alarm_status_true(w: &mut VssWorld) {
    assert_eq!(w.current_value(ALARM_STATUS), Some(SignalValue::Bool(true)));
}

#[then("Vehicle.Body.Alarm.IsActive becomes FALSE")]
async fn then_alarm_status_false(w: &mut VssWorld) {
    assert_eq!(
        w.current_value(ALARM_STATUS),
        Some(SignalValue::Bool(false))
    );
}

#[then("Vehicle.Body.Alarm.IsActive has been published exactly once with value TRUE")]
async fn then_alarm_status_once(w: &mut VssWorld) {
    let publishes: Vec<_> = w
        .bus()
        .history()
        .into_iter()
        .filter(|(s, _)| *s == ALARM_STATUS)
        .collect();
    // Background sets it to FALSE on engage transition then never again.
    // History was NOT cleared between engage and the cycles, so we expect
    // exactly one TRUE entry from the FALSE→TRUE transition.
    let true_count = publishes
        .iter()
        .filter(|(_, v)| *v == SignalValue::Bool(true))
        .count();
    assert_eq!(
        true_count, 1,
        "expected exactly one TRUE publish on Vehicle.Body.Alarm.IsActive, got {true_count} (history: {publishes:?})"
    );
}

#[then("Vehicle.Body.Alarm.IsActive has not been re-published on any pulse edge")]
async fn then_alarm_status_no_duty_cycle(w: &mut VssWorld) {
    let publishes: Vec<_> = w
        .bus()
        .history()
        .into_iter()
        .filter(|(s, _)| *s == ALARM_STATUS)
        .collect();
    // Same total of 1 TRUE — confirms no per-pulse re-publishes.
    assert!(
        publishes.len() <= 1,
        "ALARM_STATUS must not be republished per pulse, got {} entries: {:?}",
        publishes.len(),
        publishes
    );
}

#[then("Vehicle.Body.Alarm.IsActive is not re-published")]
async fn then_alarm_status_idempotent(w: &mut VssWorld) {
    let publishes = w.publish_count(ALARM_STATUS);
    assert_eq!(
        publishes, 0,
        "no ALARM_STATUS publish expected on idempotent re-engage, got {publishes}"
    );
}

// ---- Then: claim release + arbiter default-off ----

#[then(
    regex = r"^the PanicAlarm feature releases its claim on DirectionIndicator\.(Left|Right)\.IsSignaling$"
)]
async fn then_panic_releases_indicator(_w: &mut VssWorld, _side: String) {
    // Internal state — observable consequence checked by the next step.
}

#[then("the PanicAlarm feature releases its claim on Body.Horn.IsActive")]
async fn then_panic_releases_horn(_w: &mut VssWorld) {}

#[then("with no other active claim, the arbiters publish default-off on indicators and horn")]
async fn then_panic_default_off(w: &mut VssWorld) {
    assert_eq!(
        w.current_value(LEFT_SIGNALING),
        Some(SignalValue::Bool(false))
    );
    assert_eq!(
        w.current_value(RIGHT_SIGNALING),
        Some(SignalValue::Bool(false))
    );
    assert_eq!(w.current_value(HORN), Some(SignalValue::Bool(false)));
}

// ---- Then: pulse loop continues uninterrupted ----

#[then("the pulse loop continues uninterrupted at the existing cadence")]
async fn then_pulse_continues(w: &mut VssWorld) {
    // After a re-engage no-op, advance past the next ON→OFF→ON edges and
    // verify the lights still toggle on schedule.
    advance(Duration::from_millis(PANIC_ON_MS + 50)).await;
    for _ in 0..8 {
        tokio::task::yield_now().await;
    }
    assert_eq!(
        w.current_value(LEFT_SIGNALING),
        Some(SignalValue::Bool(false)),
        "Should be in OFF window after ON_MS"
    );
    advance(Duration::from_millis(PANIC_OFF_MS + 50)).await;
    for _ in 0..8 {
        tokio::task::yield_now().await;
    }
    assert_eq!(
        w.current_value(LEFT_SIGNALING),
        Some(SignalValue::Bool(true)),
        "Should be in ON window after OFF_MS"
    );
}

// ---- Then: Hazard / PanicAlarm interaction ----

#[then("the PanicAlarm feature claims both indicators at priority HIGH (latest-wins on tie)")]
async fn then_panic_takes_indicators(w: &mut VssWorld) {
    // PanicAlarm claim is more recent than Hazard's, both at HIGH —
    // arbiter's max_by_key on (priority, seq) lets PanicAlarm win.
    // Indicators are ON during the first ON window after engage.
    assert_eq!(
        w.current_value(LEFT_SIGNALING),
        Some(SignalValue::Bool(true))
    );
    assert_eq!(
        w.current_value(RIGHT_SIGNALING),
        Some(SignalValue::Bool(true))
    );
}

#[then("the PanicAlarm feature releases both indicators")]
async fn then_panic_releases_both(_w: &mut VssWorld) {}

#[then("Hazard's still-engaged claim resumes control of both indicators")]
async fn then_hazard_resumes(w: &mut VssWorld) {
    // After PanicAlarm releases, Hazard's HIGH claim (still active) wins.
    assert_eq!(
        w.current_value(LEFT_SIGNALING),
        Some(SignalValue::Bool(true)),
        "Hazard should resume Left.IsSignaling = TRUE"
    );
    assert_eq!(
        w.current_value(RIGHT_SIGNALING),
        Some(SignalValue::Bool(true)),
        "Hazard should resume Right.IsSignaling = TRUE"
    );
}

// ---- Cancel-on-unlock (REQ-PANIC-010..012) ----

const FEEDBACK_REQUEST: VssPath = "Body.Doors.CentralLock.FeedbackRequest";

#[when(r#"a successful authenticated unlock publishes FeedbackRequest = "unlock""#)]
async fn when_unlock_feedback(w: &mut VssWorld) {
    w.bus().clear_history();
    w.inject(FEEDBACK_REQUEST, SignalValue::String("unlock".into()))
        .await;
    settle().await;
    settle().await;
}

#[when(r#"the central-lock bus publishes FeedbackRequest = "lock""#)]
async fn when_lock_feedback(w: &mut VssWorld) {
    w.bus().clear_history();
    w.inject(FEEDBACK_REQUEST, SignalValue::String("lock".into()))
        .await;
    settle().await;
}

#[then("Body.Switches.Panic.IsEngaged is self-published as FALSE")]
async fn then_panic_switch_self_published_false(w: &mut VssWorld) {
    assert_eq!(
        w.current_value(PANIC_SWITCH),
        Some(SignalValue::Bool(false)),
        "PanicAlarm must self-publish PANIC_SWITCH = FALSE on unlock cancel"
    );
}

#[then("Vehicle.Body.Alarm.IsActive remains TRUE")]
async fn then_alarm_status_remains_true(w: &mut VssWorld) {
    assert_eq!(
        w.current_value(ALARM_STATUS),
        Some(SignalValue::Bool(true)),
        "ALARM_STATUS must remain TRUE — lock-feedback must not cancel"
    );
}
