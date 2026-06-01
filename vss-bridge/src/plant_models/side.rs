//! Logical ↔ physical door-side mapping.
//!
//! VSS v6.0 has no `Left` / `Right` door paths — only `DriverSide` and
//! `PassengerSide`.  A vehicle line ships LHD or RHD; the wiring is
//! physical (`Row1.Left` is the leftmost door no matter who sits in
//! it), but the *logical* role of each door (driver vs. passenger)
//! depends on the build.
//!
//! This module is the single source of truth for that mapping.  All
//! feature code that reasons about "which door is the driver" calls
//! [`VehicleOrientation::driver_physical`] (or the higher-level path
//! helpers on `PlatformConfig`) instead of pattern-matching on
//! `Left` / `Right` directly — that way the LHD-vs-RHD knowledge
//! lives in exactly one place.

/// Which physical side of the vehicle the driver sits on.  Constant
/// for the lifetime of a vehicle build; we model it as runtime
/// configuration because e2e tests need to flip it per-scenario.
#[derive(
    Debug, Clone, Copy, Default, PartialEq, Eq, Hash, serde::Deserialize, serde::Serialize,
)]
pub enum VehicleOrientation {
    /// Driver on the physical left (most of NA, EU, China).
    #[default]
    Lhd,
    /// Driver on the physical right (UK, JP, AU, IN, ZA …).
    Rhd,
}

/// Driver-relative role of a Row1 door.  Used by features whose
/// behaviour differs between the driver and passenger doors
/// (two-stage unlock, slam-lock keypad, lock-feedback flash pattern).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LogicalSide {
    Driver,
    Passenger,
}

/// Physical wiring side of a Row1 door.  Used by plant models and by
/// features that genuinely reason about all four physical doors as a
/// fan-out (perimeter alarm, walk-away lock, dome switch).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PhysicalSide {
    Left,
    Right,
}

impl PhysicalSide {
    /// Path segment used in VSS paths (e.g. `"Left"` in
    /// `Vehicle.Cabin.Door.Row1.Left.IsLocked`).
    pub const fn as_path_segment(self) -> &'static str {
        match self {
            Self::Left => "Left",
            Self::Right => "Right",
        }
    }
}

impl LogicalSide {
    /// Path segment used in canonical VSS v6.0 paths (e.g.
    /// `"DriverSide"` in `Vehicle.Cabin.Door.Row1.DriverSide.IsLocked`).
    pub const fn as_path_segment(self) -> &'static str {
        match self {
            Self::Driver => "DriverSide",
            Self::Passenger => "PassengerSide",
        }
    }
}

impl VehicleOrientation {
    /// Which physical side the driver sits on for this orientation.
    pub const fn driver_physical(self) -> PhysicalSide {
        match self {
            Self::Lhd => PhysicalSide::Left,
            Self::Rhd => PhysicalSide::Right,
        }
    }

    /// Which physical side the front-passenger sits on for this
    /// orientation.
    pub const fn passenger_physical(self) -> PhysicalSide {
        match self {
            Self::Lhd => PhysicalSide::Right,
            Self::Rhd => PhysicalSide::Left,
        }
    }

    /// Resolve a logical (driver/passenger) side to its physical
    /// (left/right) wiring side for this orientation.
    pub const fn physical(self, side: LogicalSide) -> PhysicalSide {
        match side {
            LogicalSide::Driver => self.driver_physical(),
            LogicalSide::Passenger => self.passenger_physical(),
        }
    }

    /// Resolve a physical wiring side to its logical role on this
    /// vehicle.  Inverse of [`physical`].
    pub const fn logical(self, side: PhysicalSide) -> LogicalSide {
        // Can't compare PhysicalSide in const fn via PartialEq; match instead.
        match (self, side) {
            (Self::Lhd, PhysicalSide::Left) | (Self::Rhd, PhysicalSide::Right) => {
                LogicalSide::Driver
            }
            (Self::Lhd, PhysicalSide::Right) | (Self::Rhd, PhysicalSide::Left) => {
                LogicalSide::Passenger
            }
        }
    }
}

/// Map a physical `Row{1,2}.{Left,Right}` VSS path to its canonical
/// `Row{N}.{DriverSide,PassengerSide}` sibling under the given
/// orientation.  Used by plant models that dual-publish each
/// physical-side state signal under its VSS v6.0 canonical name so
/// external Kuksa consumers can subscribe canonically (backlog #22
/// sub-PR 4b).
///
/// The mapping is a one-shot string substitution: under LHD the
/// `Left` segment becomes `DriverSide`, `Right` becomes
/// `PassengerSide`; under RHD they swap.  Row1 and Row2 use the
/// same rule (VSS v6.0 keeps driver/passenger semantics for both
/// rows — the door behind the driver is the rear-driver-side door).
///
/// Returns a leaked `&'static str` so callers can stash it in the
/// same `[&'static str; 4]` arrays the plant models already use for
/// physical paths.  The leak runs at most once per plant-model
/// construction; total memory footprint is bounded by the number of
/// dual-published per-door signals (currently 12 for door_lock).
///
/// `physical` must contain exactly one `.Left.` or `.Right.` segment.
/// Other paths return a leaked copy of the input — call sites today
/// only pass per-door VSS paths so this branch is unreached in
/// practice; the fallback exists so the helper composes cleanly into
/// `[&'static str; 4]` array initialisers without per-entry guards.
pub fn canonical_door_path(physical: &str, orientation: VehicleOrientation) -> &'static str {
    let canonical: String = if physical.contains(".Left.") {
        let segment = match orientation {
            VehicleOrientation::Lhd => "DriverSide",
            VehicleOrientation::Rhd => "PassengerSide",
        };
        physical.replacen(".Left.", &format!(".{segment}."), 1)
    } else if physical.contains(".Right.") {
        let segment = match orientation {
            VehicleOrientation::Lhd => "PassengerSide",
            VehicleOrientation::Rhd => "DriverSide",
        };
        physical.replacen(".Right.", &format!(".{segment}."), 1)
    } else {
        physical.to_string()
    };
    Box::leak(canonical.into_boxed_str())
}

/// Convenience for plant-model startup: transform a `[&str; 4]`
/// array of physical per-door paths into the matching canonical
/// `[&'static str; 4]`.
pub fn canonical_door_path_array(
    physical: &[&str; 4],
    orientation: VehicleOrientation,
) -> [&'static str; 4] {
    [
        canonical_door_path(physical[0], orientation),
        canonical_door_path(physical[1], orientation),
        canonical_door_path(physical[2], orientation),
        canonical_door_path(physical[3], orientation),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lhd_driver_is_left_passenger_is_right() {
        let o = VehicleOrientation::Lhd;
        assert_eq!(o.driver_physical(), PhysicalSide::Left);
        assert_eq!(o.passenger_physical(), PhysicalSide::Right);
    }

    #[test]
    fn rhd_driver_is_right_passenger_is_left() {
        let o = VehicleOrientation::Rhd;
        assert_eq!(o.driver_physical(), PhysicalSide::Right);
        assert_eq!(o.passenger_physical(), PhysicalSide::Left);
    }

    #[test]
    fn physical_and_logical_round_trip() {
        for o in [VehicleOrientation::Lhd, VehicleOrientation::Rhd] {
            for side in [LogicalSide::Driver, LogicalSide::Passenger] {
                assert_eq!(o.logical(o.physical(side)), side);
            }
            for side in [PhysicalSide::Left, PhysicalSide::Right] {
                assert_eq!(o.physical(o.logical(side)), side);
            }
        }
    }

    #[test]
    fn path_segments_match_vss_v6_vocabulary() {
        assert_eq!(LogicalSide::Driver.as_path_segment(), "DriverSide");
        assert_eq!(LogicalSide::Passenger.as_path_segment(), "PassengerSide");
        assert_eq!(PhysicalSide::Left.as_path_segment(), "Left");
        assert_eq!(PhysicalSide::Right.as_path_segment(), "Right");
    }

    #[test]
    fn default_orientation_is_lhd() {
        assert_eq!(VehicleOrientation::default(), VehicleOrientation::Lhd);
    }

    // ── canonical_door_path ─────────────────────────────────────────────

    #[test]
    fn canonical_lhd_left_to_driver() {
        let c = canonical_door_path(
            "Vehicle.Cabin.Door.Row1.Left.IsLocked",
            VehicleOrientation::Lhd,
        );
        assert_eq!(c, "Vehicle.Cabin.Door.Row1.DriverSide.IsLocked");
    }

    #[test]
    fn canonical_lhd_right_to_passenger() {
        let c = canonical_door_path(
            "Vehicle.Cabin.Door.Row1.Right.IsLocked",
            VehicleOrientation::Lhd,
        );
        assert_eq!(c, "Vehicle.Cabin.Door.Row1.PassengerSide.IsLocked");
    }

    #[test]
    fn canonical_rhd_left_to_passenger() {
        let c = canonical_door_path(
            "Vehicle.Cabin.Door.Row1.Left.IsLocked",
            VehicleOrientation::Rhd,
        );
        assert_eq!(c, "Vehicle.Cabin.Door.Row1.PassengerSide.IsLocked");
    }

    #[test]
    fn canonical_rhd_right_to_driver() {
        let c = canonical_door_path(
            "Vehicle.Cabin.Door.Row1.Right.IsLocked",
            VehicleOrientation::Rhd,
        );
        assert_eq!(c, "Vehicle.Cabin.Door.Row1.DriverSide.IsLocked");
    }

    /// Row2 uses the same orientation rule — the door behind the
    /// driver is the rear-driver-side door.
    #[test]
    fn canonical_row2_lhd() {
        let c = canonical_door_path(
            "Vehicle.Cabin.Door.Row2.Left.IsLocked",
            VehicleOrientation::Lhd,
        );
        assert_eq!(c, "Vehicle.Cabin.Door.Row2.DriverSide.IsLocked");
    }

    /// Nested-segment paths (e.g. `.Soldier.IsUnlocked`) replace only
    /// the one row+side segment, not anything deeper.
    #[test]
    fn canonical_preserves_deeper_segments() {
        let c = canonical_door_path(
            "Vehicle.Cabin.Door.Row1.Left.Soldier.IsUnlocked",
            VehicleOrientation::Lhd,
        );
        assert_eq!(c, "Vehicle.Cabin.Door.Row1.DriverSide.Soldier.IsUnlocked");
    }

    #[test]
    fn canonical_array_resolves_all_four_doors_lhd() {
        let physical: [&str; 4] = [
            "Vehicle.Cabin.Door.Row1.Left.IsLocked",
            "Vehicle.Cabin.Door.Row1.Right.IsLocked",
            "Vehicle.Cabin.Door.Row2.Left.IsLocked",
            "Vehicle.Cabin.Door.Row2.Right.IsLocked",
        ];
        let c = canonical_door_path_array(&physical, VehicleOrientation::Lhd);
        assert_eq!(c[0], "Vehicle.Cabin.Door.Row1.DriverSide.IsLocked");
        assert_eq!(c[1], "Vehicle.Cabin.Door.Row1.PassengerSide.IsLocked");
        assert_eq!(c[2], "Vehicle.Cabin.Door.Row2.DriverSide.IsLocked");
        assert_eq!(c[3], "Vehicle.Cabin.Door.Row2.PassengerSide.IsLocked");
    }

    #[test]
    fn canonical_array_resolves_all_four_doors_rhd() {
        let physical: [&str; 4] = [
            "Vehicle.Cabin.Door.Row1.Left.IsLocked",
            "Vehicle.Cabin.Door.Row1.Right.IsLocked",
            "Vehicle.Cabin.Door.Row2.Left.IsLocked",
            "Vehicle.Cabin.Door.Row2.Right.IsLocked",
        ];
        let c = canonical_door_path_array(&physical, VehicleOrientation::Rhd);
        assert_eq!(c[0], "Vehicle.Cabin.Door.Row1.PassengerSide.IsLocked");
        assert_eq!(c[1], "Vehicle.Cabin.Door.Row1.DriverSide.IsLocked");
        assert_eq!(c[2], "Vehicle.Cabin.Door.Row2.PassengerSide.IsLocked");
        assert_eq!(c[3], "Vehicle.Cabin.Door.Row2.DriverSide.IsLocked");
    }
}
