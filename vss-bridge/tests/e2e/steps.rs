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
use vss_bridge::features::turn_indicator::TurnIndicator;
use vss_bridge::ipc_message::SignalValue;
use vss_bridge::plant_models::blink_relay::BlinkRelay;
use vss_bridge::signal_bus::VssPath;

// ---------------------------------------------------------------------------
// Signal constants (mirror the feature modules)
// ---------------------------------------------------------------------------

const LEFT_SIGNALING: VssPath = "Body.Lights.DirectionIndicator.Left.IsSignaling";
const RIGHT_SIGNALING: VssPath = "Body.Lights.DirectionIndicator.Right.IsSignaling";

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
        self._tasks.push(tokio::spawn(arb_fut));
        let arb = Arc::new(arb);

        self._tasks.push(tokio::spawn(
            TurnIndicator::new(Arc::clone(&arb), Arc::clone(&bus)).run(),
        ));
        self._tasks.push(tokio::spawn(
            HazardLighting::new(Arc::clone(&arb), Arc::clone(&bus)).run(),
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
