//! Transmission plant — models the gearbox's response to the driver's
//! selector position, gated by the Brake / Transmission Shift Interlock
//! (BTSI).
//!
//! In production this is the transmission controller (TCU) on the
//! powertrain side: it watches the driver's PRND/S selector
//! (`Powertrain.Transmission.SelectedGear`) and, depending on
//! interlocks (brake pedal, ignition state, vehicle speed) and shift
//! logic, engages the actual gear and reports it back as
//! `Powertrain.Transmission.CurrentGear`.
//!
//! # Brake / Transmission Shift Interlock
//!
//! Every modern automatic-transmission car gates shifts out of `P`
//! (Park) on two driver-present conditions:
//!
//!   1. The brake pedal must be applied (`Chassis.Brake.IsApplied`).
//!   2. The ignition must be live — `Vehicle.LowVoltageSystemState`
//!      ∈ {`ACC`, `ON`, `START`}.  `OFF` doesn't power the
//!      shift-lock solenoid.
//!
//! If the driver attempts to shift out of `P` without both
//! conditions, the plant rejects the request:
//!
//!   * `CurrentGear` stays at `PARK`.
//!   * `SelectedGear` is republished as `PARK` so the HMI shifter
//!     visually snaps back (mirrors a real selector that simply
//!     wouldn't release the detent).
//!   * `Powertrain.Transmission.ShiftLockEngaged` stays `true`.
//!
//! Once the driver is out of `P`, shifts between R / N / D / S /
//! manual gears are accepted unconditionally — the demo doesn't
//! model speed-based shift inhibits today.
//!
//! # Published signals
//!
//! - `Powertrain.Transmission.CurrentGear` (Int16) — the engaged
//!   gear.  Boot value: `126` (PARK).
//! - `Powertrain.Transmission.ShiftLockEngaged` (Bool) — true while
//!   the interlock is suppressing motion.  Boot value: `true` (we
//!   boot with ignition OFF and brake released).
//!
//! Both signals are single-writer.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::ipc_message::SignalValue;
use crate::signal_bus::{SignalBus, VssPath};

// ── Signal constants ─────────────────────────────────────────────────────

const SELECTED: VssPath = "Powertrain.Transmission.SelectedGear";
const CURRENT: VssPath = "Powertrain.Transmission.CurrentGear";
const BRAKE_IN: VssPath = "Chassis.Brake.IsApplied";
const IGN_IN: VssPath = "Vehicle.LowVoltageSystemState";
const SHIFT_LOCK_OUT: VssPath = "Powertrain.Transmission.ShiftLockEngaged";

const PARK: i16 = 126;

/// "Live enough to release the shift lock."  Real automatic
/// transmissions only energize the shift-lock solenoid once the
/// engine controller is online — `ACC` powers radio + accessories
/// but not the powertrain, so it does NOT release P.  `OFF` and
/// `LOCK` are obviously not live.  Only `ON` (run) and `START`
/// (cranking) qualify.
fn ignition_is_live(s: &str) -> bool {
    matches!(s, "ON" | "START")
}

pub struct TransmissionPlant<B: SignalBus> {
    bus: Arc<B>,
}

impl<B: SignalBus + Send + Sync + 'static> TransmissionPlant<B> {
    pub fn new(bus: Arc<B>) -> Self {
        Self { bus }
    }

    pub async fn run(self) {
        tracing::info!("TransmissionPlant started");

        let mut sel_rx = self.bus.subscribe(SELECTED).await;
        let mut brake_rx = self.bus.subscribe(BRAKE_IN).await;
        let mut ign_rx = self.bus.subscribe(IGN_IN).await;

        // Deterministic boot — publish Park + ShiftLockEngaged so
        // late subscribers (HMI snapshot) see a defined state before
        // any input arrives.
        let _ = self.bus.publish(CURRENT, SignalValue::Int16(PARK)).await;
        // At boot: gear is Park, ignition is OFF, brake is released
        // → interlock IS engaged.
        let _ = self
            .bus
            .publish(SHIFT_LOCK_OUT, SignalValue::Bool(true))
            .await;

        let mut current: i16 = PARK;
        let mut brake_applied: bool = false;
        let mut ign_live: bool = false;
        let mut shift_lock: bool = true;

        loop {
            select! {
                Some(val) = brake_rx.next() => {
                    if let SignalValue::Bool(b) = val {
                        brake_applied = b;
                        self.update_shift_lock(current, brake_applied, ign_live, &mut shift_lock).await;
                    }
                }

                Some(val) = ign_rx.next() => {
                    if let SignalValue::String(s) = val {
                        ign_live = ignition_is_live(&s);
                        self.update_shift_lock(current, brake_applied, ign_live, &mut shift_lock).await;
                    }
                }

                Some(val) = sel_rx.next() => {
                    // The ws-bridge JSON converter routes JS numbers into
                    // Uint8 / Uint16 / Int16 by range — positive small ints
                    // arrive as Uint8 even though the gear catalog is i16.
                    // Accept any numeric variant and coerce to i16.
                    let want: i16 = match val {
                        SignalValue::Int16(v) => v,
                        SignalValue::Uint8(v) => v as i16,
                        SignalValue::Uint16(v) => v.try_into().unwrap_or(i16::MAX),
                        _ => continue,
                    };
                    if want == current {
                        continue; // idempotent — no shift edge
                    }
                    // BTSI: shifts OUT of PARK require brake + live ignition.
                    if current == PARK && want != PARK {
                        if !brake_applied || !ign_live {
                            tracing::info!(
                                want, brake = brake_applied, ign_live,
                                "TransmissionPlant: BTSI blocked shift out of PARK"
                            );
                            // Snap the selector chip back to PARK so the
                            // HMI shifter visually matches the engaged
                            // gear.  Plant remains in PARK.
                            let _ = self
                                .bus
                                .publish(SELECTED, SignalValue::Int16(PARK))
                                .await;
                            continue;
                        }
                    }
                    tracing::info!(from = current, to = want, "TransmissionPlant: shift");
                    current = want;
                    if let Err(e) = self.bus.publish(CURRENT, SignalValue::Int16(current)).await {
                        tracing::error!(error = %e, "TransmissionPlant: publish failed");
                    }
                    self.update_shift_lock(current, brake_applied, ign_live, &mut shift_lock).await;
                }

                else => break,
            }
        }

        tracing::warn!("TransmissionPlant: selector stream closed, exiting");
    }

    /// Recompute the ShiftLockEngaged flag and publish on change.
    /// Engaged means: gear is PARK AND (brake not applied OR ignition
    /// not live) — i.e. the next attempted P→other shift would be
    /// rejected.
    async fn update_shift_lock(
        &self,
        current: i16,
        brake_applied: bool,
        ign_live: bool,
        shift_lock: &mut bool,
    ) {
        let want = current == PARK && (!brake_applied || !ign_live);
        if want == *shift_lock {
            return;
        }
        *shift_lock = want;
        if let Err(e) = self
            .bus
            .publish(SHIFT_LOCK_OUT, SignalValue::Bool(want))
            .await
        {
            tracing::error!(error = %e, "TransmissionPlant: shift-lock publish failed");
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
    }

    async fn setup() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let plant = TransmissionPlant::new(Arc::clone(&bus));
        tokio::spawn(plant.run());
        settle().await;
        bus
    }

    fn current(bus: &MockBus) -> Option<i16> {
        match bus.latest_value(CURRENT) {
            Some(SignalValue::Int16(v)) => Some(v),
            _ => None,
        }
    }

    fn selected(bus: &MockBus) -> Option<i16> {
        match bus.latest_value(SELECTED) {
            Some(SignalValue::Int16(v)) => Some(v),
            Some(SignalValue::Uint8(v)) => Some(v as i16),
            _ => None,
        }
    }

    fn shift_lock(bus: &MockBus) -> Option<bool> {
        match bus.latest_value(SHIFT_LOCK_OUT) {
            Some(SignalValue::Bool(b)) => Some(b),
            _ => None,
        }
    }

    /// Make brake + ignition live so shifts out of PARK are allowed.
    fn driver_ready(bus: &MockBus) {
        bus.inject(IGN_IN, SignalValue::String("ON".into()));
        bus.inject(BRAKE_IN, SignalValue::Bool(true));
    }

    #[tokio::test]
    async fn boots_to_park_with_shift_lock_engaged() {
        let bus = setup().await;
        assert_eq!(current(&bus), Some(126));
        assert_eq!(shift_lock(&bus), Some(true),
            "boot: P + brake released + ignition OFF must engage shift lock");
    }

    #[tokio::test]
    async fn btsi_blocks_shift_out_of_park_without_brake() {
        let bus = setup().await;
        bus.inject(IGN_IN, SignalValue::String("ON".into()));
        // brake not pressed
        settle().await;
        bus.inject(SELECTED, SignalValue::Int16(127)); // Drive
        settle().await;
        assert_eq!(current(&bus), Some(126), "shift to D must be rejected");
        assert_eq!(selected(&bus), Some(126), "selector chip must snap back to P");
    }

    #[tokio::test]
    async fn btsi_blocks_shift_out_of_park_without_ignition() {
        let bus = setup().await;
        bus.inject(BRAKE_IN, SignalValue::Bool(true));
        // ignition stays OFF
        settle().await;
        bus.inject(SELECTED, SignalValue::Int16(127));
        settle().await;
        assert_eq!(current(&bus), Some(126));
        assert_eq!(selected(&bus), Some(126));
    }

    #[tokio::test]
    async fn btsi_allows_shift_with_brake_and_ignition() {
        let bus = setup().await;
        driver_ready(&bus);
        settle().await;
        bus.inject(SELECTED, SignalValue::Int16(127)); // Drive
        settle().await;
        assert_eq!(current(&bus), Some(127));
        assert_eq!(shift_lock(&bus), Some(false),
            "out of P → shift lock disengages");
    }

    #[tokio::test]
    async fn shift_back_to_park_always_allowed() {
        let bus = setup().await;
        driver_ready(&bus);
        settle().await;
        bus.inject(SELECTED, SignalValue::Int16(127));
        settle().await;
        // Release brake; shifting back to PARK is still allowed.
        bus.inject(BRAKE_IN, SignalValue::Bool(false));
        bus.inject(SELECTED, SignalValue::Int16(126));
        settle().await;
        assert_eq!(current(&bus), Some(126));
    }

    #[tokio::test]
    async fn shifts_between_non_park_gears_unconstrained() {
        // Once out of P, R ↔ N ↔ D ↔ manual all work without
        // re-checking brake / ignition (matches the user's spec —
        // only the P-exit transition is gated).
        let bus = setup().await;
        driver_ready(&bus);
        settle().await;
        bus.inject(SELECTED, SignalValue::Int16(127)); // P → D
        settle().await;
        // Lift brake and shift D → R; should still work.
        bus.inject(BRAKE_IN, SignalValue::Bool(false));
        bus.inject(SELECTED, SignalValue::Int16(-1));
        settle().await;
        assert_eq!(current(&bus), Some(-1));
        bus.inject(SELECTED, SignalValue::Int16(0)); // R → N
        settle().await;
        assert_eq!(current(&bus), Some(0));
    }

    #[tokio::test]
    async fn acc_does_not_release_shift_lock() {
        // ACC powers radio + accessories only; the powertrain stays
        // dormant so the shift lock must NOT release.  Driver has to
        // crank to ON (or START) before the selector frees up.
        let bus = setup().await;
        bus.inject(IGN_IN, SignalValue::String("ACC".into()));
        bus.inject(BRAKE_IN, SignalValue::Bool(true));
        settle().await;
        assert_eq!(shift_lock(&bus), Some(true), "ACC must not release the lock");
        bus.inject(SELECTED, SignalValue::Int16(127));
        settle().await;
        assert_eq!(current(&bus), Some(126), "shift attempt at ACC must be rejected");
        assert_eq!(selected(&bus), Some(126), "selector chip snaps back");
    }

    #[tokio::test]
    async fn shift_lock_tracks_conditions() {
        let bus = setup().await;
        assert_eq!(shift_lock(&bus), Some(true));
        bus.inject(IGN_IN, SignalValue::String("ON".into()));
        settle().await;
        assert_eq!(shift_lock(&bus), Some(true), "ignition alone not enough");
        bus.inject(BRAKE_IN, SignalValue::Bool(true));
        settle().await;
        assert_eq!(shift_lock(&bus), Some(false), "brake + ignition → released");
        bus.inject(BRAKE_IN, SignalValue::Bool(false));
        settle().await;
        assert_eq!(shift_lock(&bus), Some(true), "brake released → re-engaged");
    }

    #[tokio::test]
    async fn accepts_uint8_for_positive_gear_codes() {
        let bus = setup().await;
        driver_ready(&bus);
        settle().await;
        bus.inject(SELECTED, SignalValue::Uint8(127));
        settle().await;
        assert_eq!(current(&bus), Some(127));
        bus.inject(SELECTED, SignalValue::Uint8(0));
        settle().await;
        assert_eq!(current(&bus), Some(0));
    }

    #[tokio::test]
    async fn manual_gears_pass_through() {
        let bus = setup().await;
        driver_ready(&bus);
        settle().await;
        for g in [1, 2, 3, 4, 5, 6] {
            bus.inject(SELECTED, SignalValue::Int16(g));
            settle().await;
            assert_eq!(current(&bus), Some(g));
        }
    }

    #[tokio::test]
    async fn redundant_selection_is_idempotent() {
        let bus = setup().await;
        driver_ready(&bus);
        settle().await;
        bus.inject(SELECTED, SignalValue::Int16(127));
        settle().await;
        bus.clear_history();
        bus.inject(SELECTED, SignalValue::Int16(127));
        bus.inject(SELECTED, SignalValue::Int16(127));
        settle().await;
        let republishes = bus.history().iter().filter(|(s, _)| *s == CURRENT).count();
        assert_eq!(republishes, 0);
    }
}
