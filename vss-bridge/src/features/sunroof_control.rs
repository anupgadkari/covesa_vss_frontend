//! Sunroof control feature — interprets the overhead-console rocker
//! and drives the sunroof + shade motors in the correct sequence.
//!
//! ```text
//!  HMI / sim-panel rocker
//!      │  Body.Switches.Sunroof.Detent
//!      │     NEUTRAL / OPEN_HOLD / OPEN_AUTO / CLOSE_HOLD / CLOSE_AUTO
//!      ▼
//!  SunroofControl feature              ← this module
//!      │  Body.Sunroof.MoveCmd        (OPEN / CLOSE / STOP)
//!      │  Body.Sunroof.Shade.MoveCmd  (OPEN / CLOSE / STOP)
//!      ▼
//!  SunroofPlantModel — integrates positions over time
//! ```
//!
//! # Sequencing
//!
//! Both **proportional (hold)** and **auto (latched)** motions move
//! the two motors in this order:
//!
//! | Direction | Sequence |
//! |---|---|
//! | OPEN  | shade opens fully → then roof opens |
//! | CLOSE | roof closes fully → then shade closes |
//!
//! While the feature is "driving OPEN" it issues
//!   `shade.MoveCmd = OPEN, roof.MoveCmd = STOP`
//! until `shade.Position == 100`, then switches to
//!   `shade.MoveCmd = STOP, roof.MoveCmd = OPEN`
//! until `roof.Position == 100`.  The mirror logic applies for
//! CLOSE.  When both end-stops are reached in auto mode the feature
//! naturally returns to IDLE.
//!
//! # State machine
//!
//! | State | Detent → NEUTRAL | Detent → HOLD_* | Detent → AUTO_* |
//! |---|---|---|---|
//! | Idle             | – | → Holding (drive)       | → Auto (drive, latched) |
//! | Holding(dir)     | → Idle (STOP) | same dir: no-op; other: → AwaitingRelease (STOP) | → AwaitingRelease (STOP) |
//! | Auto(dir)        | no-op (latched) | → AwaitingRelease (STOP) | → AwaitingRelease (STOP) |
//! | AwaitingRelease  | → Idle | no-op | no-op |
//!
//! AwaitingRelease implements the user spec: "any press cancels auto
//! motion; a new button push (i.e. release-then-press) is required
//! to start a new motion".
//!
//! # Single writer
//!
//! Sole *intended* writer of `Body.Sunroof.MoveCmd` /
//! `Body.Sunroof.Shade.MoveCmd` in the cockpit flow.  The legacy
//! sim-panel `SunroofMotorRow` writes the same signals directly for
//! engineering use; race is acceptable for that surface.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

const DETENT: VssPath = "Body.Switches.Sunroof.Detent";
const ROOF_CMD: VssPath = "Body.Sunroof.MoveCmd";
const SHADE_CMD: VssPath = "Body.Sunroof.Shade.MoveCmd";
const ROOF_POS: VssPath = "Body.Sunroof.Position";
const SHADE_POS: VssPath = "Body.Sunroof.Shade.Position";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Detent {
    Neutral,
    OpenHold,
    OpenAuto,
    CloseHold,
    CloseAuto,
}

impl Detent {
    fn parse(v: &SignalValue) -> Option<Self> {
        match v {
            SignalValue::String(s) => match s.as_str() {
                "NEUTRAL" => Some(Self::Neutral),
                "OPEN_HOLD" => Some(Self::OpenHold),
                "OPEN_AUTO" => Some(Self::OpenAuto),
                "CLOSE_HOLD" => Some(Self::CloseHold),
                "CLOSE_AUTO" => Some(Self::CloseAuto),
                _ => None,
            },
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dir {
    Open,
    Close,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    Holding(Dir),
    Auto(Dir),
    AwaitingRelease,
}

pub struct SunroofControl<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> SunroofControl<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    pub async fn run(self) {
        tracing::info!("SunroofControl feature started");

        let mut detent_rx = self.bus.subscribe(DETENT).await;
        let mut roof_pos_rx = self.bus.subscribe(ROOF_POS).await;
        let mut shade_pos_rx = self.bus.subscribe(SHADE_POS).await;

        let mut state = State::Idle;
        let mut roof_pos: u8 = 0;
        let mut shade_pos: u8 = 0;
        // Last published motor commands — used to suppress duplicates
        // and to leave the bus quiet while the legacy sim-panel rows
        // drive the same signals directly.
        let mut last_roof: Option<&'static str> = None;
        let mut last_shade: Option<&'static str> = None;

        loop {
            select! {
                Some(val) = detent_rx.next() => {
                    if let Some(d) = Detent::parse(&val) {
                        state = Self::on_detent(state, d);
                        Self::drive(state, roof_pos, shade_pos,
                            &self.bus, &mut last_roof, &mut last_shade).await;
                    }
                }
                Some(val) = roof_pos_rx.next() => {
                    if let SignalValue::Uint8(v) = val {
                        roof_pos = v;
                        // Re-evaluate motor sequencing on every position
                        // update.  Auto motions may naturally complete here.
                        state = Self::on_position(state, roof_pos, shade_pos);
                        Self::drive(state, roof_pos, shade_pos,
                            &self.bus, &mut last_roof, &mut last_shade).await;
                    }
                }
                Some(val) = shade_pos_rx.next() => {
                    if let SignalValue::Uint8(v) = val {
                        shade_pos = v;
                        state = Self::on_position(state, roof_pos, shade_pos);
                        Self::drive(state, roof_pos, shade_pos,
                            &self.bus, &mut last_roof, &mut last_shade).await;
                    }
                }
                else => break,
            }
        }

        tracing::warn!("SunroofControl: streams closed, exiting");
    }

    /// Compute the next state from a detent edge.  No I/O.
    fn on_detent(state: State, d: Detent) -> State {
        match state {
            State::Idle => match d {
                Detent::Neutral => State::Idle,
                Detent::OpenHold => State::Holding(Dir::Open),
                Detent::OpenAuto => State::Auto(Dir::Open),
                Detent::CloseHold => State::Holding(Dir::Close),
                Detent::CloseAuto => State::Auto(Dir::Close),
            },
            // Holding — release returns to idle; same direction is a
            // no-op; any other detent cancels via AwaitingRelease.
            State::Holding(Dir::Open) => match d {
                Detent::Neutral => State::Idle,
                Detent::OpenHold => State::Holding(Dir::Open),
                _ => State::AwaitingRelease,
            },
            State::Holding(Dir::Close) => match d {
                Detent::Neutral => State::Idle,
                Detent::CloseHold => State::Holding(Dir::Close),
                _ => State::AwaitingRelease,
            },
            // Auto — latched: NEUTRAL keeps running; any press cancels.
            State::Auto(dir) => match d {
                Detent::Neutral => State::Auto(dir),
                _ => State::AwaitingRelease,
            },
            // AwaitingRelease — only NEUTRAL clears it.
            State::AwaitingRelease => match d {
                Detent::Neutral => State::Idle,
                _ => State::AwaitingRelease,
            },
        }
    }

    /// Re-evaluate state after a position update — auto motions may
    /// have just reached their end-stop.
    fn on_position(state: State, roof: u8, shade: u8) -> State {
        match state {
            State::Auto(Dir::Open) if roof == 100 && shade == 100 => State::Idle,
            State::Auto(Dir::Close) if roof == 0 && shade == 0 => State::Idle,
            other => other,
        }
    }

    /// Drive the two motors according to current state + positions.
    /// Idempotent: only publishes a motor command when it actually
    /// changes, so the bus stays quiet while idle.
    async fn drive(
        state: State,
        roof: u8,
        shade: u8,
        bus: &Arc<B>,
        last_roof: &mut Option<&'static str>,
        last_shade: &mut Option<&'static str>,
    ) {
        let (roof_cmd, shade_cmd) = match state {
            State::Idle | State::AwaitingRelease => ("STOP", "STOP"),
            State::Holding(Dir::Open) | State::Auto(Dir::Open) => {
                // Open sequence: shade fully opens, then roof opens.
                if shade < 100 {
                    ("STOP", "OPEN")
                } else if roof < 100 {
                    ("OPEN", "STOP")
                } else {
                    ("STOP", "STOP")
                }
            }
            State::Holding(Dir::Close) | State::Auto(Dir::Close) => {
                // Close sequence: roof fully closes, then shade closes.
                if roof > 0 {
                    ("CLOSE", "STOP")
                } else if shade > 0 {
                    ("STOP", "CLOSE")
                } else {
                    ("STOP", "STOP")
                }
            }
        };
        if *last_roof != Some(roof_cmd) {
            *last_roof = Some(roof_cmd);
            let _ = bus
                .publish(ROOF_CMD, SignalValue::String(roof_cmd.into()))
                .await;
        }
        if *last_shade != Some(shade_cmd) {
            *last_shade = Some(shade_cmd);
            let _ = bus
                .publish(SHADE_CMD, SignalValue::String(shade_cmd.into()))
                .await;
        }
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;

    async fn settle() {
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        tokio::time::sleep(std::time::Duration::from_millis(2)).await;
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    async fn setup() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        // Seed positions so the feature has a defined starting view.
        bus.inject(ROOF_POS, SignalValue::Uint8(0));
        bus.inject(SHADE_POS, SignalValue::Uint8(0));
        bus.inject(DETENT, SignalValue::String("NEUTRAL".into()));
        tokio::spawn(SunroofControl::new(Arc::clone(&bus)).run());
        settle().await;
        bus
    }

    fn cmd(bus: &MockBus, sig: VssPath) -> Option<String> {
        match bus.latest_value(sig) {
            Some(SignalValue::String(s)) => Some(s),
            _ => None,
        }
    }

    #[tokio::test]
    async fn idle_at_boot_does_not_drive() {
        let bus = setup().await;
        // Both motors should rest at STOP (or unpublished).
        let r = cmd(&bus, ROOF_CMD);
        let s = cmd(&bus, SHADE_CMD);
        assert!(r.is_none() || r.as_deref() == Some("STOP"));
        assert!(s.is_none() || s.as_deref() == Some("STOP"));
    }

    #[tokio::test]
    async fn open_hold_drives_shade_first() {
        let bus = setup().await;
        bus.inject(DETENT, SignalValue::String("OPEN_HOLD".into()));
        settle().await;
        // Shade is at 0, opens before roof.
        assert_eq!(cmd(&bus, SHADE_CMD).as_deref(), Some("OPEN"));
        assert_eq!(cmd(&bus, ROOF_CMD).as_deref(), Some("STOP"));
    }

    #[tokio::test]
    async fn open_sequence_advances_to_roof_when_shade_full() {
        let bus = setup().await;
        bus.inject(DETENT, SignalValue::String("OPEN_AUTO".into()));
        settle().await;
        // Simulate the plant reporting shade reached 100.
        bus.inject(SHADE_POS, SignalValue::Uint8(100));
        settle().await;
        assert_eq!(cmd(&bus, SHADE_CMD).as_deref(), Some("STOP"));
        assert_eq!(cmd(&bus, ROOF_CMD).as_deref(), Some("OPEN"));
    }

    #[tokio::test]
    async fn close_drives_roof_first() {
        let bus = setup().await;
        // Start from fully-open.
        bus.inject(ROOF_POS, SignalValue::Uint8(100));
        bus.inject(SHADE_POS, SignalValue::Uint8(100));
        settle().await;
        bus.inject(DETENT, SignalValue::String("CLOSE_HOLD".into()));
        settle().await;
        assert_eq!(cmd(&bus, ROOF_CMD).as_deref(), Some("CLOSE"));
        assert_eq!(cmd(&bus, SHADE_CMD).as_deref(), Some("STOP"));
    }

    #[tokio::test]
    async fn close_sequence_advances_to_shade_when_roof_zero() {
        let bus = setup().await;
        bus.inject(ROOF_POS, SignalValue::Uint8(100));
        bus.inject(SHADE_POS, SignalValue::Uint8(100));
        settle().await;
        bus.inject(DETENT, SignalValue::String("CLOSE_AUTO".into()));
        settle().await;
        bus.inject(ROOF_POS, SignalValue::Uint8(0));
        settle().await;
        assert_eq!(cmd(&bus, ROOF_CMD).as_deref(), Some("STOP"));
        assert_eq!(cmd(&bus, SHADE_CMD).as_deref(), Some("CLOSE"));
    }

    #[tokio::test]
    async fn hold_release_stops_motors() {
        let bus = setup().await;
        bus.inject(DETENT, SignalValue::String("OPEN_HOLD".into()));
        settle().await;
        bus.inject(DETENT, SignalValue::String("NEUTRAL".into()));
        settle().await;
        assert_eq!(cmd(&bus, SHADE_CMD).as_deref(), Some("STOP"));
        assert_eq!(cmd(&bus, ROOF_CMD).as_deref(), Some("STOP"));
    }

    #[tokio::test]
    async fn auto_latches_through_release() {
        let bus = setup().await;
        bus.inject(DETENT, SignalValue::String("OPEN_AUTO".into()));
        settle().await;
        // The user "releases" — switch springs back to NEUTRAL.
        bus.inject(DETENT, SignalValue::String("NEUTRAL".into()));
        settle().await;
        // Auto remains latched — motors still driving the open sequence.
        assert_eq!(cmd(&bus, SHADE_CMD).as_deref(), Some("OPEN"));
    }

    #[tokio::test]
    async fn any_press_cancels_auto() {
        let bus = setup().await;
        bus.inject(DETENT, SignalValue::String("OPEN_AUTO".into()));
        settle().await;
        bus.inject(DETENT, SignalValue::String("NEUTRAL".into()));
        settle().await;
        // Now press 1st-detent open while auto is still latched.
        bus.inject(DETENT, SignalValue::String("OPEN_HOLD".into()));
        settle().await;
        // Both motors must STOP (cancel via AwaitingRelease).
        assert_eq!(cmd(&bus, SHADE_CMD).as_deref(), Some("STOP"));
        assert_eq!(cmd(&bus, ROOF_CMD).as_deref(), Some("STOP"));
    }

    #[tokio::test]
    async fn awaiting_release_requires_neutral_before_new_press_works() {
        let bus = setup().await;
        bus.inject(DETENT, SignalValue::String("OPEN_AUTO".into()));
        settle().await;
        bus.inject(DETENT, SignalValue::String("OPEN_HOLD".into())); // cancels
        settle().await;
        // Still holding — no motion (we're in AwaitingRelease).
        assert_eq!(cmd(&bus, SHADE_CMD).as_deref(), Some("STOP"));
        // Release.
        bus.inject(DETENT, SignalValue::String("NEUTRAL".into()));
        settle().await;
        // Now a new press starts motion.
        bus.inject(DETENT, SignalValue::String("OPEN_HOLD".into()));
        settle().await;
        assert_eq!(cmd(&bus, SHADE_CMD).as_deref(), Some("OPEN"));
    }

    #[tokio::test]
    async fn auto_completes_at_end_stop() {
        let bus = setup().await;
        bus.inject(DETENT, SignalValue::String("OPEN_AUTO".into()));
        settle().await;
        bus.inject(SHADE_POS, SignalValue::Uint8(100));
        settle().await;
        bus.inject(ROOF_POS, SignalValue::Uint8(100));
        settle().await;
        // Both at end-stop → motors STOP; state Idle.
        assert_eq!(cmd(&bus, SHADE_CMD).as_deref(), Some("STOP"));
        assert_eq!(cmd(&bus, ROOF_CMD).as_deref(), Some("STOP"));
        // A new press starts a fresh motion (e.g. CLOSE).
        bus.inject(DETENT, SignalValue::String("CLOSE_HOLD".into()));
        settle().await;
        assert_eq!(cmd(&bus, ROOF_CMD).as_deref(), Some("CLOSE"));
    }
}
