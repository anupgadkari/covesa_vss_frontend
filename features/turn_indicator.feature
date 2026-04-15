Feature: Turn Indicator
  As the body controller platform
  I must activate the correct direction indicator
  when the driver moves the turn signal stalk
  so that the vehicle signals its intended direction of travel.

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-TURN-001: When Body.Switches.TurnIndicator.Direction transitions to
  #               "LEFT", the feature SHALL request
  #               DirectionIndicator.Left.IsSignaling = TRUE at priority
  #               MEDIUM (2) via the Lighting arbiter, and SHALL release
  #               any existing TurnIndicator claim on
  #               DirectionIndicator.Right.IsSignaling.
  #
  # REQ-TURN-002: When Body.Switches.TurnIndicator.Direction transitions to
  #               "RIGHT", the feature SHALL request
  #               DirectionIndicator.Right.IsSignaling = TRUE at priority
  #               MEDIUM (2) via the Lighting arbiter, and SHALL release
  #               any existing TurnIndicator claim on
  #               DirectionIndicator.Left.IsSignaling.
  #
  # REQ-TURN-003: When Body.Switches.TurnIndicator.Direction transitions to
  #               "OFF", the feature SHALL release any TurnIndicator claims
  #               on both DirectionIndicator.Left.IsSignaling and
  #               DirectionIndicator.Right.IsSignaling. A released claim
  #               lets any lower-priority or concurrent claim from another
  #               feature (e.g. Hazard) resume; if no other claim exists,
  #               the arbiter publishes the default-off value.
  #
  # REQ-TURN-004: The turn indicator feature SHALL subscribe to the physical
  #               stalk input (Body.Switches.TurnIndicator.Direction), NOT
  #               the actuator outputs.
  #
  # REQ-TURN-005: Turn indicator requests SHALL use priority MEDIUM (2).
  #               The Hazard feature (HIGH/3) and Lock Feedback (HIGH/3,
  #               overlay) can temporarily override. Lock Feedback
  #               self-releases after its brief pattern, allowing the
  #               turn signal to resume.
  #
  # REQ-TURN-006: The turn indicator feature SHALL NOT set blink timing.
  #
  # REQ-TURN-007: The turn indicator feature SHALL have no dependency on
  #               any other feature module.
  #
  # REQ-TURN-008: The turn indicator feature SHALL only process stalk
  #               inputs when Vehicle.LowVoltageSystemState is "ON" or
  #               "START". When ignition transitions to any other state
  #               (OFF, LOCK, ACC), the feature SHALL immediately release
  #               any TurnIndicator claims on both
  #               DirectionIndicator.Left.IsSignaling and
  #               DirectionIndicator.Right.IsSignaling.
  #               Rationale: turn signals require ignition ON per vehicle
  #               electrical architecture — the turn signal relay is
  #               powered from the ignition-switched bus.
  # -------------------------------------------------------------------------

  Background:
    Given the vehicle low-voltage system is in state "ON"
    And the Lighting domain arbiter is running
    And the Turn Indicator feature is running

  # --- REQ-TURN-001 ---
  Scenario: Turn stalk moved to LEFT
    Given the turn stalk is in position OFF
    When the driver moves the turn stalk to LEFT
    Then Body.Switches.TurnIndicator.Direction becomes "LEFT"
    And the Turn feature requests DirectionIndicator.Left.IsSignaling = TRUE at priority MEDIUM
    And the Turn feature releases its claim on DirectionIndicator.Right.IsSignaling

  # --- REQ-TURN-002 ---
  Scenario: Turn stalk moved to RIGHT
    Given the turn stalk is in position OFF
    When the driver moves the turn stalk to RIGHT
    Then Body.Switches.TurnIndicator.Direction becomes "RIGHT"
    And the Turn feature requests DirectionIndicator.Right.IsSignaling = TRUE at priority MEDIUM
    And the Turn feature releases its claim on DirectionIndicator.Left.IsSignaling

  # --- REQ-TURN-003 ---
  Scenario: Turn stalk returned to OFF
    Given the turn stalk is in position LEFT
    And the left direction indicator is signaling
    When the driver returns the turn stalk to OFF
    Then Body.Switches.TurnIndicator.Direction becomes "OFF"
    And the Turn feature releases its claim on DirectionIndicator.Left.IsSignaling
    And the Turn feature releases its claim on DirectionIndicator.Right.IsSignaling

  # --- REQ-TURN-001, REQ-TURN-002 ---
  Scenario: Turn stalk changes directly from LEFT to RIGHT
    Given the turn stalk is in position LEFT
    When the driver moves the turn stalk to RIGHT
    Then the Turn feature releases its claim on DirectionIndicator.Left.IsSignaling
    And the Turn feature requests DirectionIndicator.Right.IsSignaling = TRUE at priority MEDIUM

  # --- REQ-TURN-005 ---
  Scenario: Turn signal suppressed while hazard is active
    Given the hazard switch is engaged
    And both indicators are signaling at priority HIGH
    When the driver moves the turn stalk to LEFT
    Then the Turn feature's MEDIUM claim on DirectionIndicator.Left.IsSignaling is recorded by the arbiter
    And the Turn feature releases its claim on DirectionIndicator.Right.IsSignaling
    And both indicators continue signaling because Hazard's HIGH claims win arbitration

  # ===========================================================================
  # Ignition gating (REQ-TURN-008)
  # ===========================================================================

  # --- REQ-TURN-008 ---
  Scenario: Turn stalk ignored when ignition is OFF
    Given the vehicle low-voltage system is in state "OFF"
    When the driver moves the turn stalk to LEFT
    Then the Turn feature does NOT request any indicator change

  # --- REQ-TURN-008 ---
  Scenario: Turn stalk ignored when ignition is ACC
    Given the vehicle low-voltage system is in state "ACC"
    When the driver moves the turn stalk to LEFT
    Then the Turn feature does NOT request any indicator change

  # --- REQ-TURN-008 ---
  Scenario: Active turn signal deactivated when ignition turns OFF
    Given the turn stalk is in position LEFT
    And the left direction indicator is signaling at priority MEDIUM
    When Vehicle.LowVoltageSystemState transitions to "OFF"
    Then the Turn feature releases its claim on DirectionIndicator.Left.IsSignaling
    And the Turn feature releases its claim on DirectionIndicator.Right.IsSignaling

  # --- REQ-TURN-008 ---
  Scenario: Active turn signal deactivated when ignition goes to ACC
    Given the turn stalk is in position RIGHT
    And the right direction indicator is signaling at priority MEDIUM
    When Vehicle.LowVoltageSystemState transitions to "ACC"
    Then the Turn feature releases its claim on DirectionIndicator.Left.IsSignaling
    And the Turn feature releases its claim on DirectionIndicator.Right.IsSignaling

  # --- REQ-TURN-008 ---
  Scenario: Turn signal resumes when ignition returns to ON
    Given the turn stalk is in position LEFT
    And Vehicle.LowVoltageSystemState was "ACC" (turn inactive)
    When Vehicle.LowVoltageSystemState transitions to "ON"
    Then the Turn feature requests DirectionIndicator.Left.IsSignaling = TRUE at priority MEDIUM
    And the Turn feature releases its claim on DirectionIndicator.Right.IsSignaling
