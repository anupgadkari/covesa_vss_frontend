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
  #               DirectionIndicator.Left.IsSignaling = TRUE and
  #               DirectionIndicator.Right.IsSignaling = FALSE via the
  #               Lighting arbiter at priority MEDIUM (2).
  #
  # REQ-TURN-002: When Body.Switches.TurnIndicator.Direction transitions to
  #               "RIGHT", the feature SHALL request
  #               DirectionIndicator.Right.IsSignaling = TRUE and
  #               DirectionIndicator.Left.IsSignaling = FALSE via the
  #               Lighting arbiter at priority MEDIUM (2).
  #
  # REQ-TURN-003: When Body.Switches.TurnIndicator.Direction transitions to
  #               "OFF", the feature SHALL request both
  #               DirectionIndicator.Left.IsSignaling = FALSE and
  #               DirectionIndicator.Right.IsSignaling = FALSE at priority
  #               MEDIUM (2).
  #
  # REQ-TURN-004: The turn indicator feature SHALL subscribe to the physical
  #               stalk input (Body.Switches.TurnIndicator.Direction), NOT
  #               the actuator outputs.
  #
  # REQ-TURN-005: Turn indicator requests SHALL use priority MEDIUM (2).
  #               The Hazard feature (HIGH/3) can override; Lock Feedback
  #               (LOW/1) cannot.
  #
  # REQ-TURN-006: The turn indicator feature SHALL NOT set blink timing.
  #
  # REQ-TURN-007: The turn indicator feature SHALL have no dependency on
  #               any other feature module.
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
    And the Turn feature requests DirectionIndicator.Right.IsSignaling = FALSE at priority MEDIUM

  # --- REQ-TURN-002 ---
  Scenario: Turn stalk moved to RIGHT
    Given the turn stalk is in position OFF
    When the driver moves the turn stalk to RIGHT
    Then Body.Switches.TurnIndicator.Direction becomes "RIGHT"
    And the Turn feature requests DirectionIndicator.Right.IsSignaling = TRUE at priority MEDIUM
    And the Turn feature requests DirectionIndicator.Left.IsSignaling = FALSE at priority MEDIUM

  # --- REQ-TURN-003 ---
  Scenario: Turn stalk returned to OFF
    Given the turn stalk is in position LEFT
    And the left direction indicator is signaling
    When the driver returns the turn stalk to OFF
    Then Body.Switches.TurnIndicator.Direction becomes "OFF"
    And the Turn feature requests DirectionIndicator.Left.IsSignaling = FALSE at priority MEDIUM
    And the Turn feature requests DirectionIndicator.Right.IsSignaling = FALSE at priority MEDIUM

  # --- REQ-TURN-001, REQ-TURN-002 ---
  Scenario: Turn stalk changes directly from LEFT to RIGHT
    Given the turn stalk is in position LEFT
    When the driver moves the turn stalk to RIGHT
    Then the Turn feature requests DirectionIndicator.Left.IsSignaling = FALSE at priority MEDIUM
    And the Turn feature requests DirectionIndicator.Right.IsSignaling = TRUE at priority MEDIUM

  # --- REQ-TURN-005 ---
  Scenario: Turn signal suppressed while hazard is active
    Given the hazard switch is engaged
    And both indicators are signaling at priority HIGH
    When the driver moves the turn stalk to LEFT
    Then the Turn feature's MEDIUM request is suppressed by the Lighting arbiter
    And both indicators continue signaling due to Hazard's HIGH priority
