Feature: Double-Lock Release on Ignition ON
  As the body controller platform
  I must automatically clear the double-lock (superlock) state when the
  driver turns the ignition ON
  so that exterior door handles are physically re-connected and emergency
  egress is not impaired during a drive cycle.

  # -------------------------------------------------------------------------
  # Implementation model
  # -------------------------------------------------------------------------
  # Monitors Vehicle.LowVoltageSystemState.
  # OFF states:  "OFF", "ACC", "LOCK"
  # ON  states:  "ON", "START"
  #
  # Fires ReleaseDouble via DoorLockArbiter on an OFF-→-ON transition.
  # ReleaseDouble: clears IsDoubleLocked on all doors, preserves IsLocked.
  #
  # No FeedbackRequest is published — this is an internal automatic trigger,
  # not a user-initiated command.
  # -------------------------------------------------------------------------

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-DLR-001: When Vehicle.LowVoltageSystemState transitions from an OFF
  #              state (OFF / ACC / LOCK) to an ON state (ON / START), the
  #              feature SHALL dispatch ReleaseDouble via the DoorLockArbiter.
  #
  # REQ-DLR-002: ReleaseDouble SHALL clear IsDoubleLocked on all doors while
  #              keeping IsLocked = true. The vehicle remains locked; only the
  #              mechanical handle-linkage disconnection is restored.
  #
  # REQ-DLR-003: If the power state transitions ON → ON without an intervening
  #              OFF state, the feature SHALL NOT dispatch ReleaseDouble again.
  #              Only the first OFF→ON edge fires.
  #
  # REQ-DLR-004: ACC is treated as an OFF state for the purposes of transition
  #              detection. An OFF → ACC → ON sequence SHALL trigger release.
  #
  # REQ-DLR-005: No FeedbackRequest SHALL be published — this is an internal
  #              automatic trigger with no user-visible flash pattern.
  #
  # REQ-DLR-006: The feature SHALL assume ignition OFF at boot (safe default)
  #              so that the initial ON signal at startup triggers release if
  #              the vehicle was double-locked before the power cycle.
  # -------------------------------------------------------------------------

  Background:
    Given the DoorLock domain arbiter is running
    And the Double-Lock Release feature is running
    And all doors are double-locked (IsDoubleLocked = true, IsLocked = true)

  # --- REQ-DLR-001, REQ-DLR-002 ---
  Scenario: Ignition OFF then ON clears superlock
    Given Vehicle.LowVoltageSystemState is "OFF"
    When Vehicle.LowVoltageSystemState transitions to "ON"
    Then a ReleaseDouble command is issued via the DoorLockArbiter
    And IsDoubleLocked is false on all doors
    And IsLocked remains true on all doors
    And no FeedbackRequest is published

  # --- REQ-DLR-003 ---
  Scenario: Second ON without intervening OFF does not re-dispatch
    Given Vehicle.LowVoltageSystemState is "ON"
    And ReleaseDouble has already been dispatched this cycle
    When Vehicle.LowVoltageSystemState is published as "ON" again
    Then no additional ReleaseDouble command is issued

  # --- REQ-DLR-004 ---
  Scenario: OFF → ACC → ON sequence triggers release
    Given Vehicle.LowVoltageSystemState transitions to "OFF"
    And Vehicle.LowVoltageSystemState transitions to "ACC"
    When Vehicle.LowVoltageSystemState transitions to "ON"
    Then a ReleaseDouble command is issued

  # --- REQ-DLR-004 ---
  Scenario: ACC treated as OFF — ACC → ON triggers release
    Given Vehicle.LowVoltageSystemState is "ACC"
    When Vehicle.LowVoltageSystemState transitions to "ON"
    Then a ReleaseDouble command is issued

  # --- REQ-DLR-001 ---
  Scenario: START state also triggers release after OFF
    Given Vehicle.LowVoltageSystemState is "OFF"
    When Vehicle.LowVoltageSystemState transitions to "START"
    Then a ReleaseDouble command is issued

  # --- REQ-DLR-006 ---
  Scenario: Boot-time ON signal triggers release (safe default = was OFF)
    Given the Double-Lock Release feature has just started (no prior state received)
    When the first Vehicle.LowVoltageSystemState value received is "ON"
    Then a ReleaseDouble command is issued
    Because the feature defaults to last_was_off = true at boot

  # --- REQ-DLR-005 ---
  Scenario: No lock-feedback flash when superlock is released
    When Vehicle.LowVoltageSystemState transitions from "OFF" to "ON"
    Then a ReleaseDouble command is issued
    And Body.Doors.CentralLock.FeedbackRequest is NOT published
    And no direction indicator flash pattern is played
