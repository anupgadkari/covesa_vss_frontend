Feature: Low Beam Headlamps
  As the body controller platform
  I must activate the low beam headlamps
  when the driver turns the light switch to the headlamp position
  so that the road ahead is illuminated for safe driving at night.

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-LOW-001: When Body.Lights.LightSwitch indicates headlamps should be
  #              active, the feature SHALL request
  #              Body.Lights.Beam.Low.IsOn = TRUE via the Lighting arbiter
  #              at priority MEDIUM (2).
  #
  # REQ-LOW-002: When Body.Lights.LightSwitch indicates headlamps should be
  #              off, the feature SHALL request
  #              Body.Lights.Beam.Low.IsOn = FALSE via the Lighting arbiter
  #              at priority MEDIUM (2).
  #
  # REQ-LOW-003: The low beam feature SHALL subscribe to
  #              Body.Lights.LightSwitch (the physical rotary/stalk input),
  #              NOT the actuator output Body.Lights.Beam.Low.IsOn.
  #
  # REQ-LOW-004: Low beam requests SHALL use priority MEDIUM (2).
  #
  # REQ-LOW-005: The low beam feature SHALL have no dependency on any other
  #              feature module.
  # -------------------------------------------------------------------------

  Background:
    Given the vehicle low-voltage system is in state "ON"
    And the Lighting domain arbiter is running
    And the Low Beam feature is running

  # --- REQ-LOW-001 ---
  Scenario: Light switch turned to headlamp position
    Given the light switch is in the OFF position
    When the driver turns the light switch to the headlamp position
    Then the Low Beam feature requests Body.Lights.Beam.Low.IsOn = TRUE at priority MEDIUM

  # --- REQ-LOW-002 ---
  Scenario: Light switch turned off
    Given the light switch is in the headlamp position
    And low beams are on
    When the driver turns the light switch to OFF
    Then the Low Beam feature requests Body.Lights.Beam.Low.IsOn = FALSE at priority MEDIUM

  # --- REQ-LOW-001 ---
  Scenario: Light switch to parking does not activate low beam
    Given the light switch is in the OFF position
    When the driver turns the light switch to the parking position
    Then the Low Beam feature does NOT request Body.Lights.Beam.Low.IsOn = TRUE
