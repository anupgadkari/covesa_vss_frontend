Feature: Hazard Lighting
  As the body controller platform
  I must activate both direction indicators simultaneously
  when the driver engages the physical hazard switch
  so that surrounding traffic is warned of a stationary or emergency vehicle.

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-HAZ-001: When Body.Switches.Hazard.IsEngaged transitions to TRUE,
  #              the feature SHALL request both DirectionIndicator.Left.IsSignaling
  #              and DirectionIndicator.Right.IsSignaling to TRUE via the
  #              Lighting arbiter at priority HIGH (3).
  #
  # REQ-HAZ-002: When Body.Switches.Hazard.IsEngaged transitions to FALSE,
  #              the feature SHALL request both DirectionIndicator.Left.IsSignaling
  #              and DirectionIndicator.Right.IsSignaling to FALSE via the
  #              Lighting arbiter at priority HIGH (3).
  #
  # REQ-HAZ-003: The hazard feature SHALL subscribe to the physical switch
  #              input (Body.Switches.Hazard.IsEngaged), NOT the actuator
  #              output (Body.Lights.Hazard.IsSignaling), to prevent
  #              feedback loops.
  #
  # REQ-HAZ-004: The hazard feature SHALL NOT set blink timing. The 1-2 Hz
  #              UN R48-compliant cadence is the responsibility of the LED
  #              driver IC or body ECU firmware.
  #
  # REQ-HAZ-005: Hazard requests SHALL use priority HIGH (3), which wins
  #              over Turn Indicator (MEDIUM/2) and Lock Feedback (LOW/1)
  #              in the Lighting arbiter.
  #
  # REQ-HAZ-006: The hazard feature SHALL have no dependency on any other
  #              feature module.
  # -------------------------------------------------------------------------

  Background:
    Given the vehicle low-voltage system is in state "ON"
    And the Lighting domain arbiter is running
    And the Hazard feature is running

  # --- REQ-HAZ-001 ---
  Scenario: Hazard switch engaged activates both indicators
    Given the hazard switch is not engaged
    When the driver engages the hazard switch
    Then Body.Switches.Hazard.IsEngaged becomes TRUE
    And the Hazard feature requests DirectionIndicator.Left.IsSignaling = TRUE at priority HIGH
    And the Hazard feature requests DirectionIndicator.Right.IsSignaling = TRUE at priority HIGH

  # --- REQ-HAZ-002 ---
  Scenario: Hazard switch disengaged deactivates both indicators
    Given the hazard switch is engaged
    And both direction indicators are signaling due to hazard
    When the driver disengages the hazard switch
    Then Body.Switches.Hazard.IsEngaged becomes FALSE
    And the Hazard feature requests DirectionIndicator.Left.IsSignaling = FALSE at priority HIGH
    And the Hazard feature requests DirectionIndicator.Right.IsSignaling = FALSE at priority HIGH

  # --- REQ-HAZ-005 ---
  Scenario: Hazard overrides active turn signal
    Given the turn stalk is in position LEFT
    And the left direction indicator is signaling at priority MEDIUM
    When the driver engages the hazard switch
    Then both direction indicators signal at priority HIGH
    And the turn signal's MEDIUM request is suppressed by the arbiter

  # --- REQ-HAZ-005 ---
  Scenario: Hazard overrides lock feedback flash
    Given a lock feedback flash is active at priority LOW
    When the driver engages the hazard switch
    Then both direction indicators signal at priority HIGH
    And the lock feedback's LOW request is suppressed by the arbiter

  # --- REQ-HAZ-002, REQ-HAZ-005 ---
  Scenario: Turn signal resumes after hazard is disengaged
    Given the turn stalk is in position LEFT
    And the hazard switch is engaged (overriding the turn signal)
    When the driver disengages the hazard switch
    Then the Hazard feature releases both indicators at priority HIGH
    And the Turn feature's pending LEFT request at priority MEDIUM takes effect

  # --- REQ-HAZ-004 ---
  Scenario: Hazard feature does not control blink timing
    When the driver engages the hazard switch
    Then the Hazard feature publishes IsSignaling = TRUE once
    And the Hazard feature does NOT publish periodic on/off toggles
