//! Blink Relay — plant model for direction indicator lamp oscillation.
//!
//! Subscribes to the arbitrated indicator intent signals
//! (`Body.Lights.DirectionIndicator.{Left,Right}.IsSignaling`) and the
//! per-lamp defect signals
//! (`Body.Lights.DirectionIndicator.{Left,Right}.Lamp.{Front,Side,Rear}.IsDefect`),
//! and publishes the actual lamp state for each of the six physical lamps
//! (`Body.Lights.DirectionIndicator.{Left,Right}.Lamp.{Front,Side,Rear}.IsOn`).
//!
//! Timing (UN ECE Regulation No. 6):
//!   - Normal operation:  ~1.5 Hz  (333 ms half-period)
//!   - Bulb defect (turn signal only): ~3.0 Hz  (167 ms half-period)
//!
//! Per current interpretation of UN ECE R48 §6.5.9, the doubled rate on
//! bulb failure applies to turn signals only and affects *all* lamps on
//! the failed side. Hazard lighting (both sides active) stays at the
//! normal cadence regardless of lamp defects, matching typical OEM
//! practice (hazard is already a warning mode).
//!
//! Sync: when both sides are signaling (hazard, or turn stalk engaged
//! while hazards are on), the left and right lamp groups share a single
//! phase — they turn on together and off together. This mimics the
//! classic body-ECU behavior where a single flasher drives both sides
//! in hazard mode.
//!
//! In production this behavior would live in the body ECU's lamp
//! driver (M7 / smart actuator firmware). The plant model lets us
//! exercise the full HMI/feature loop on dev hosts without M7 firmware.
//!
//! Plant models bypass the arbiter — they represent physical hardware
//! and publish feedback signals directly to the SignalBus.

use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;

use futures::StreamExt;
use tokio::select;
use tokio::time::{sleep, Sleep};

use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

const LEFT_INTENT: VssPath = "Body.Lights.DirectionIndicator.Left.IsSignaling";
const RIGHT_INTENT: VssPath = "Body.Lights.DirectionIndicator.Right.IsSignaling";

const LEFT_DEFECT_FRONT: VssPath = "Body.Lights.DirectionIndicator.Left.Lamp.Front.IsDefect";
const LEFT_DEFECT_SIDE: VssPath = "Body.Lights.DirectionIndicator.Left.Lamp.Side.IsDefect";
const LEFT_DEFECT_REAR: VssPath = "Body.Lights.DirectionIndicator.Left.Lamp.Rear.IsDefect";
const RIGHT_DEFECT_FRONT: VssPath = "Body.Lights.DirectionIndicator.Right.Lamp.Front.IsDefect";
const RIGHT_DEFECT_SIDE: VssPath = "Body.Lights.DirectionIndicator.Right.Lamp.Side.IsDefect";
const RIGHT_DEFECT_REAR: VssPath = "Body.Lights.DirectionIndicator.Right.Lamp.Rear.IsDefect";

const LEFT_LAMP_FRONT: VssPath = "Body.Lights.DirectionIndicator.Left.Lamp.Front.IsOn";
const LEFT_LAMP_SIDE: VssPath = "Body.Lights.DirectionIndicator.Left.Lamp.Side.IsOn";
const LEFT_LAMP_REAR: VssPath = "Body.Lights.DirectionIndicator.Left.Lamp.Rear.IsOn";
const RIGHT_LAMP_FRONT: VssPath = "Body.Lights.DirectionIndicator.Right.Lamp.Front.IsOn";
const RIGHT_LAMP_SIDE: VssPath = "Body.Lights.DirectionIndicator.Right.Lamp.Side.IsOn";
const RIGHT_LAMP_REAR: VssPath = "Body.Lights.DirectionIndicator.Right.Lamp.Rear.IsOn";

const LEFT_LAMPS: [VssPath; 3] = [LEFT_LAMP_FRONT, LEFT_LAMP_SIDE, LEFT_LAMP_REAR];
const RIGHT_LAMPS: [VssPath; 3] = [RIGHT_LAMP_FRONT, RIGHT_LAMP_SIDE, RIGHT_LAMP_REAR];

const PERIOD_NORMAL: Duration = Duration::from_millis(333);
const PERIOD_DEFECT: Duration = Duration::from_millis(167);
const IDLE: Duration = Duration::from_secs(3600);

/// Per-side blink state.
#[derive(Default)]
struct SideState {
    signaling: bool,
    defect_front: bool,
    defect_side: bool,
    defect_rear: bool,
    lamp_on: bool,
}

impl SideState {
    fn any_defect(&self) -> bool {
        self.defect_front || self.defect_side || self.defect_rear
    }
}

/// Compute the active period for a side, considering hazard (both sides
/// signaling) mode. In hazard mode we always use the normal cadence.
fn period_for(side: &SideState, other_signaling: bool) -> Duration {
    if other_signaling {
        // Hazard / both-sides mode: ignore defect rate.
        PERIOD_NORMAL
    } else if side.any_defect() {
        PERIOD_DEFECT
    } else {
        PERIOD_NORMAL
    }
}

pub struct BlinkRelay<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus> BlinkRelay<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    pub async fn run(self) {
        tracing::info!("BlinkRelay plant model started");

        let mut left_intent = self.bus.subscribe(LEFT_INTENT).await;
        let mut right_intent = self.bus.subscribe(RIGHT_INTENT).await;

        let mut left_def_f = self.bus.subscribe(LEFT_DEFECT_FRONT).await;
        let mut left_def_s = self.bus.subscribe(LEFT_DEFECT_SIDE).await;
        let mut left_def_r = self.bus.subscribe(LEFT_DEFECT_REAR).await;
        let mut right_def_f = self.bus.subscribe(RIGHT_DEFECT_FRONT).await;
        let mut right_def_s = self.bus.subscribe(RIGHT_DEFECT_SIDE).await;
        let mut right_def_r = self.bus.subscribe(RIGHT_DEFECT_REAR).await;

        let mut left = SideState::default();
        let mut right = SideState::default();

        // Sleep futures for each side. Always exist; armed far in the
        // future when the side is idle, and reset to the next tick
        // deadline when the side is actively blinking.
        let mut left_tick: Pin<Box<Sleep>> = Box::pin(sleep(IDLE));
        let mut right_tick: Pin<Box<Sleep>> = Box::pin(sleep(IDLE));

        loop {
            select! {
                Some(val) = left_intent.next() => {
                    let was = left.signaling;
                    left.signaling = as_bool(&val);
                    tracing::debug!(was, now = left.signaling, "BlinkRelay: LEFT intent");
                    handle_intent_change(
                        &self.bus, &mut left, was, right.signaling,
                        &LEFT_LAMPS, &mut left_tick,
                        // If this is a rising edge and the other side is
                        // already signaling, sync to the other side's phase.
                        Some((&mut right, &RIGHT_LAMPS, &mut right_tick)),
                    ).await;
                }
                Some(val) = right_intent.next() => {
                    let was = right.signaling;
                    right.signaling = as_bool(&val);
                    tracing::debug!(was, now = right.signaling, "BlinkRelay: RIGHT intent");
                    handle_intent_change(
                        &self.bus, &mut right, was, left.signaling,
                        &RIGHT_LAMPS, &mut right_tick,
                        Some((&mut left, &LEFT_LAMPS, &mut left_tick)),
                    ).await;
                }
                Some(val) = left_def_f.next() => {
                    left.defect_front = as_bool(&val);
                    rearm_on_defect(&left, &right, &mut left_tick, &mut right_tick);
                }
                Some(val) = left_def_s.next() => {
                    left.defect_side = as_bool(&val);
                    rearm_on_defect(&left, &right, &mut left_tick, &mut right_tick);
                }
                Some(val) = left_def_r.next() => {
                    left.defect_rear = as_bool(&val);
                    rearm_on_defect(&left, &right, &mut left_tick, &mut right_tick);
                }
                Some(val) = right_def_f.next() => {
                    right.defect_front = as_bool(&val);
                    rearm_on_defect(&right, &left, &mut right_tick, &mut left_tick);
                }
                Some(val) = right_def_s.next() => {
                    right.defect_side = as_bool(&val);
                    rearm_on_defect(&right, &left, &mut right_tick, &mut left_tick);
                }
                Some(val) = right_def_r.next() => {
                    right.defect_rear = as_bool(&val);
                    rearm_on_defect(&right, &left, &mut right_tick, &mut left_tick);
                }
                _ = &mut left_tick, if left.signaling => {
                    left.lamp_on = !left.lamp_on;
                    publish_side(&self.bus, &LEFT_LAMPS, left.lamp_on).await;
                    let p = period_for(&left, right.signaling);
                    left_tick.as_mut().reset(tokio::time::Instant::now() + p);
                }
                _ = &mut right_tick, if right.signaling => {
                    right.lamp_on = !right.lamp_on;
                    publish_side(&self.bus, &RIGHT_LAMPS, right.lamp_on).await;
                    let p = period_for(&right, left.signaling);
                    right_tick.as_mut().reset(tokio::time::Instant::now() + p);
                }
                else => break,
            }
        }

        tracing::warn!("BlinkRelay: streams closed, exiting");
    }
}

/// Apply an intent change to a side. On rising edge the lamp group turns
/// on immediately and the next tick is scheduled. On falling edge the
/// lamp group is forced off. If the other side is already signaling on
/// a rising edge, the other side's phase is also reset so the two sides
/// blink in sync (classic hazard behavior).
#[allow(clippy::type_complexity)]
async fn handle_intent_change<B: SignalBus>(
    bus: &Arc<B>,
    side: &mut SideState,
    was_signaling: bool,
    other_signaling: bool,
    lamp_signals: &[VssPath; 3],
    tick: &mut Pin<Box<Sleep>>,
    other: Option<(&mut SideState, &[VssPath; 3], &mut Pin<Box<Sleep>>)>,
) {
    match (was_signaling, side.signaling) {
        (false, true) => {
            // Rising edge: lamps on, schedule next toggle.
            side.lamp_on = true;
            publish_side(bus, lamp_signals, true).await;

            let now = tokio::time::Instant::now();
            let p = period_for(side, other_signaling);
            tick.as_mut().reset(now + p);

            // Sync with other side if it's already signaling: re-arm its
            // lamps to ON and reset its timer to the same deadline.
            if other_signaling {
                if let Some((other_state, other_lamps, other_tick)) = other {
                    if !other_state.lamp_on {
                        other_state.lamp_on = true;
                        publish_side(bus, other_lamps, true).await;
                    }
                    let op = period_for(other_state, side.signaling);
                    other_tick.as_mut().reset(now + op);
                }
            }
        }
        (true, false) => {
            // Falling edge: lamps off, idle the timer.
            side.lamp_on = false;
            publish_side(bus, lamp_signals, false).await;
            tick.as_mut().reset(tokio::time::Instant::now() + IDLE);

            // If the other side is still signaling and its period changed
            // (because we're leaving hazard mode and defect rate now
            // applies), re-arm its timer from now.
            if other_signaling {
                if let Some((other_state, _, other_tick)) = other {
                    let op = period_for(other_state, side.signaling);
                    other_tick.as_mut().reset(tokio::time::Instant::now() + op);
                }
            }
        }
        _ => {}
    }
}

/// Re-arm timers after a defect change.  In hazard mode (both sides
/// signaling) both timers must be reset together to preserve phase
/// sync — otherwise only the defect side's timer shifts and the two
/// sides drift apart.
fn rearm_on_defect(
    side: &SideState,
    other: &SideState,
    tick: &mut Pin<Box<Sleep>>,
    other_tick: &mut Pin<Box<Sleep>>,
) {
    if !side.signaling {
        return;
    }
    let now = tokio::time::Instant::now();
    let p = period_for(side, other.signaling);
    tick.as_mut().reset(now + p);
    // In hazard mode, keep the other side's timer in sync.
    if other.signaling {
        let op = period_for(other, side.signaling);
        other_tick.as_mut().reset(now + op);
    }
}

async fn publish_side<B: SignalBus>(bus: &Arc<B>, signals: &[VssPath; 3], on: bool) {
    for &sig in signals {
        publish(bus, sig, on).await;
    }
}

async fn publish<B: SignalBus>(bus: &Arc<B>, signal: VssPath, on: bool) {
    if let Err(e) = bus.publish(signal, SignalValue::Bool(on)).await {
        tracing::error!(signal, error = %e, "BlinkRelay: publish failed");
    }
}

fn as_bool(val: &SignalValue) -> bool {
    matches!(val, SignalValue::Bool(true))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use tokio::time::advance;

    fn lamp_events(history: &[(VssPath, SignalValue)], signal: VssPath) -> Vec<bool> {
        history
            .iter()
            .filter(|(s, _)| *s == signal)
            .map(|(_, v)| matches!(v, SignalValue::Bool(true)))
            .collect()
    }

    async fn setup() -> (Arc<MockBus>, tokio::task::JoinHandle<()>) {
        let bus = Arc::new(MockBus::new());
        let relay = BlinkRelay::new(Arc::clone(&bus));
        let handle = tokio::spawn(relay.run());
        // Let the relay subscribe before we inject anything.
        tokio::task::yield_now().await;
        tokio::task::yield_now().await;
        (bus, handle)
    }

    #[tokio::test(start_paused = true)]
    async fn intent_rising_edge_turns_all_left_lamps_on() {
        let (bus, _h) = setup().await;

        bus.inject(LEFT_INTENT, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        advance(Duration::from_millis(1)).await;

        for lamp in LEFT_LAMPS {
            let events = lamp_events(&bus.history(), lamp);
            assert_eq!(events, vec![true], "{} should turn on at rising edge", lamp);
        }
    }

    #[tokio::test(start_paused = true)]
    async fn intent_falling_edge_forces_all_lamps_off() {
        let (bus, _h) = setup().await;

        bus.inject(LEFT_INTENT, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        advance(Duration::from_millis(1)).await;

        bus.clear_history();
        bus.inject(LEFT_INTENT, SignalValue::Bool(false));
        tokio::task::yield_now().await;
        advance(Duration::from_millis(1)).await;

        for lamp in LEFT_LAMPS {
            let events = lamp_events(&bus.history(), lamp);
            assert_eq!(
                events,
                vec![false],
                "{} should turn off at falling edge",
                lamp
            );
        }
    }

    #[tokio::test(start_paused = true)]
    async fn normal_blink_rate_is_1_5hz() {
        let (bus, _h) = setup().await;

        bus.inject(LEFT_INTENT, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        advance(Duration::from_millis(1)).await;

        for _ in 0..3 {
            advance(Duration::from_millis(333)).await;
            tokio::task::yield_now().await;
        }

        let events = lamp_events(&bus.history(), LEFT_LAMP_FRONT);
        assert_eq!(
            events,
            vec![true, false, true, false],
            "expected 4 events at 1.5 Hz on front lamp: {:?}",
            events
        );
    }

    #[tokio::test(start_paused = true)]
    async fn defect_on_any_lamp_doubles_blink_rate_for_whole_side() {
        let (bus, _h) = setup().await;

        // Only the rear lamp is defective, but the entire side must
        // flash at 3 Hz per UN ECE R48 §6.5.9.
        bus.inject(LEFT_DEFECT_REAR, SignalValue::Bool(true));
        tokio::task::yield_now().await;

        bus.inject(LEFT_INTENT, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        advance(Duration::from_millis(1)).await;

        for _ in 0..4 {
            advance(Duration::from_millis(167)).await;
            tokio::task::yield_now().await;
        }

        // All three lamps on the left side should show the same pattern.
        for lamp in LEFT_LAMPS {
            let events = lamp_events(&bus.history(), lamp);
            assert_eq!(
                events,
                vec![true, false, true, false, true],
                "expected 5 events at 3 Hz (defect) on {}: {:?}",
                lamp,
                events
            );
        }
    }

    #[tokio::test(start_paused = true)]
    async fn hazard_ignores_defect_rate() {
        // Both sides signaling — even with a defect, cadence stays at 1.5 Hz.
        let (bus, _h) = setup().await;

        bus.inject(LEFT_DEFECT_FRONT, SignalValue::Bool(true));
        tokio::task::yield_now().await;

        bus.inject(LEFT_INTENT, SignalValue::Bool(true));
        bus.inject(RIGHT_INTENT, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        advance(Duration::from_millis(1)).await;

        // Advance one normal half-period. At 1.5 Hz we expect a toggle;
        // at 3 Hz we'd see two toggles. So we can distinguish by counting.
        advance(Duration::from_millis(333)).await;
        tokio::task::yield_now().await;

        let events = lamp_events(&bus.history(), LEFT_LAMP_FRONT);
        // initial ON + one OFF toggle => 2 events
        assert_eq!(
            events,
            vec![true, false],
            "hazard should use 1.5 Hz despite defect: {:?}",
            events
        );
    }

    #[tokio::test(start_paused = true)]
    async fn left_and_right_sync_in_hazard() {
        let (bus, _h) = setup().await;

        bus.inject(LEFT_INTENT, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        advance(Duration::from_millis(100)).await;
        tokio::task::yield_now().await;

        // Right joins mid-cycle; should sync to left's phase starting now.
        bus.inject(RIGHT_INTENT, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        advance(Duration::from_millis(1)).await;

        // Both sides should now transition together.
        bus.clear_history();
        advance(Duration::from_millis(333)).await;
        tokio::task::yield_now().await;

        let left_events = lamp_events(&bus.history(), LEFT_LAMP_FRONT);
        let right_events = lamp_events(&bus.history(), RIGHT_LAMP_FRONT);
        assert_eq!(
            left_events, right_events,
            "left and right should toggle in sync in hazard: L={:?} R={:?}",
            left_events, right_events
        );
        assert_eq!(left_events.len(), 1, "expected exactly one toggle in 333ms");
    }

    #[tokio::test(start_paused = true)]
    async fn defect_during_hazard_keeps_sides_in_sync() {
        // Regression: injecting a lamp defect while hazards are blinking
        // used to reset only the defect side's timer, breaking phase sync.
        let (bus, _h) = setup().await;

        bus.inject(LEFT_INTENT, SignalValue::Bool(true));
        bus.inject(RIGHT_INTENT, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        advance(Duration::from_millis(1)).await;

        // Let one tick happen so both sides are mid-cycle.
        advance(Duration::from_millis(333)).await;
        tokio::task::yield_now().await;

        // Advance partway into the next half-period, then inject a defect.
        advance(Duration::from_millis(150)).await;
        tokio::task::yield_now().await;

        bus.inject(LEFT_DEFECT_FRONT, SignalValue::Bool(true));
        tokio::task::yield_now().await;

        // After the defect, both timers should be re-armed together.
        // Collect transitions over the next 2 seconds — left and right
        // front lamps must toggle the same number of times.
        bus.clear_history();
        for _ in 0..6 {
            advance(Duration::from_millis(333)).await;
            tokio::task::yield_now().await;
        }

        let left_events = lamp_events(&bus.history(), LEFT_LAMP_FRONT);
        let right_events = lamp_events(&bus.history(), RIGHT_LAMP_FRONT);
        assert_eq!(
            left_events, right_events,
            "left and right must stay in sync after defect during hazard: L={:?} R={:?}",
            left_events, right_events
        );
    }

    #[tokio::test(start_paused = true)]
    async fn exiting_hazard_keeps_remaining_side_blinking() {
        // Regression for: "when I stop Hazard with the switch, the right
        // indicators should still continue blinking — they don't".
        // The plant model must not turn off the remaining side.
        let (bus, _h) = setup().await;

        bus.inject(LEFT_INTENT, SignalValue::Bool(true));
        bus.inject(RIGHT_INTENT, SignalValue::Bool(true));
        tokio::task::yield_now().await;
        advance(Duration::from_millis(1)).await;

        // Left goes away (hazard released while turn stalk still right).
        bus.inject(LEFT_INTENT, SignalValue::Bool(false));
        tokio::task::yield_now().await;
        advance(Duration::from_millis(1)).await;

        bus.clear_history();
        // Right should keep blinking at 1.5 Hz.
        for _ in 0..2 {
            advance(Duration::from_millis(333)).await;
            tokio::task::yield_now().await;
        }

        let events = lamp_events(&bus.history(), RIGHT_LAMP_FRONT);
        assert!(
            events.len() >= 2,
            "right should keep blinking after left falling edge: {:?}",
            events
        );
    }
}
