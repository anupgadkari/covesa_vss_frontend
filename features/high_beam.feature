Feature: High Beam Headlamps
  As the body controller platform
  I must activate the high beam headlamps
  when the driver engages the high beam stalk switch
  so that the driver has extended forward visibility on unlit roads.

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-HIGH-001: When Body.Switches.HighBeam.IsEngaged transitions to TRUE,
  #               the feature SHALL request
  #               Body.Lights.Beam.High.IsOn = TRUE via the Lighting arbiter
  #               at priority MEDIUM (2).
  #
  # REQ-HIGH-002: When Body.Switches.HighBeam.IsEngaged transitions to FALSE,
  #               the feature SHALL request
  #               Body.Lights.Beam.High.IsOn = FALSE via the Lighting arbiter
  #               at priority MEDIUM (2).
  #
  # REQ-HIGH-003: The high beam feature SHALL subscribe to the physical stalk
  #               input (Body.Switches.HighBeam.IsEngaged), NOT the actuator
  #               output.
  #
  # REQ-HIGH-004: High beam requests SHALL use priority MEDIUM (2).
  #
  # REQ-HIGH-005: The high beam feature SHALL have no dependency on any other
  #               feature module. It does NOT interlock with low beam —
  #               mutual exclusion (if required) is the Safety Monitor's
  #               responsibility.
  # -------------------------------------------------------------------------

  Background:
    Given the vehicle low-voltage system is in state "ON"
    And the Lighting domain arbiter is running
    And the High Beam feature is running

  # --- REQ-HIGH-001 ---
  Scenario: High beam stalk engaged
    Given the high beam switch is not engaged
    When the driver engages the high beam stalk
    Then Body.Switches.HighBeam.IsEngaged becomes TRUE
    And the High Beam feature requests Body.Lights.Beam.High.IsOn = TRUE at priority MEDIUM

  # --- REQ-HIGH-002 ---
  Scenario: High beam stalk released
    Given the high beam switch is engaged
    And high beams are on
    When the driver releases the high beam stalk
    Then Body.Switches.HighBeam.IsEngaged becomes FALSE
    And the High Beam feature requests Body.Lights.Beam.High.IsOn = FALSE at priority MEDIUM

  # --- REQ-HIGH-001 ---
  Scenario: Flash-to-pass (momentary high beam)
    Given the high beam switch is not engaged
    When the driver momentarily pulls the high beam stalk
    Then Body.Switches.HighBeam.IsEngaged becomes TRUE briefly
    And the High Beam feature requests high beam ON
    When the stalk returns to rest
    Then Body.Switches.HighBeam.IsEngaged becomes FALSE
    And the High Beam feature requests high beam OFF

  # --- REQ-HIGH-005 ---
  Scenario: High beam and low beam operate independently
    Given the low beam is on
    When the driver engages the high beam stalk
    Then the High Beam feature requests high beam ON
    And the Low Beam feature's request is unaffected
    And both beams may be on simultaneously (per Safety Monitor rules)
