Feature: Daytime Running Lights (DRL)
  As the body controller platform
  I must activate the daytime running lights automatically
  based on vehicle power state and parking brake status
  so that the vehicle is visible to other road users during daylight
  as required by regulation (UN R87 / FMVSS 108).

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-DRL-001: When Vehicle.LowVoltageSystemState transitions to "ON" or
  #              "START" AND Chassis.ParkingBrake.IsEngaged is FALSE, the
  #              feature SHALL request Body.Lights.Running.IsOn = TRUE via
  #              the Lighting arbiter at priority MEDIUM (2).
  #
  # REQ-DRL-002: When Vehicle.LowVoltageSystemState transitions to any state
  #              other than "ON" or "START" (i.e., "OFF", "LOCK", "ACC"),
  #              the feature SHALL request Body.Lights.Running.IsOn = FALSE
  #              at priority MEDIUM (2).
  #
  # REQ-DRL-003: When Vehicle.LowVoltageSystemState is "ON" or "START" AND
  #              Chassis.ParkingBrake.IsEngaged transitions to TRUE, the
  #              feature SHALL request Body.Lights.Running.IsOn = FALSE
  #              at priority MEDIUM (2). (DRL off while parked.)
  #
  # REQ-DRL-004: When Chassis.ParkingBrake.IsEngaged transitions to FALSE
  #              while Vehicle.LowVoltageSystemState is "ON" or "START",
  #              the feature SHALL request Body.Lights.Running.IsOn = TRUE
  #              at priority MEDIUM (2). (DRL on when driving.)
  #
  # REQ-DRL-005: The DRL feature SHALL subscribe to
  #              Vehicle.LowVoltageSystemState (standard VSS v4.0) and
  #              Chassis.ParkingBrake.IsEngaged (overlay). It SHALL NOT
  #              use Vehicle.Powertrain.Engine.IsRunning, which is
  #              powertrain-specific and does not work for BEV/HEV.
  #
  # REQ-DRL-006: DRL requests SHALL use priority MEDIUM (2).
  #
  # REQ-DRL-007: The DRL feature SHALL have no dependency on any other
  #              feature module.
  #
  # REQ-DRL-008: The DRL feature SHALL work identically across ICE, HEV,
  #              and BEV powertrains. It depends only on
  #              LowVoltageSystemState, not engine state.
  # -------------------------------------------------------------------------

  Background:
    Given the Lighting domain arbiter is running
    And the DRL feature is running

  # --- REQ-DRL-001 ---
  Scenario: Vehicle powered on, parking brake released — DRL activates
    Given Vehicle.LowVoltageSystemState is "OFF"
    And Chassis.ParkingBrake.IsEngaged is FALSE
    When Vehicle.LowVoltageSystemState transitions to "ON"
    Then the DRL feature requests Body.Lights.Running.IsOn = TRUE at priority MEDIUM

  # --- REQ-DRL-002 ---
  Scenario: Vehicle powered off — DRL deactivates
    Given Vehicle.LowVoltageSystemState is "ON"
    And DRL is active
    When Vehicle.LowVoltageSystemState transitions to "OFF"
    Then the DRL feature requests Body.Lights.Running.IsOn = FALSE at priority MEDIUM

  # --- REQ-DRL-002 ---
  Scenario: Vehicle in ACC mode — DRL off
    Given Vehicle.LowVoltageSystemState is "OFF"
    When Vehicle.LowVoltageSystemState transitions to "ACC"
    Then the DRL feature requests Body.Lights.Running.IsOn = FALSE at priority MEDIUM

  # --- REQ-DRL-003 ---
  Scenario: Parking brake engaged while running — DRL deactivates
    Given Vehicle.LowVoltageSystemState is "ON"
    And Chassis.ParkingBrake.IsEngaged is FALSE
    And DRL is active
    When Chassis.ParkingBrake.IsEngaged transitions to TRUE
    Then the DRL feature requests Body.Lights.Running.IsOn = FALSE at priority MEDIUM

  # --- REQ-DRL-004 ---
  Scenario: Parking brake released while running — DRL reactivates
    Given Vehicle.LowVoltageSystemState is "ON"
    And Chassis.ParkingBrake.IsEngaged is TRUE
    And DRL is off
    When Chassis.ParkingBrake.IsEngaged transitions to FALSE
    Then the DRL feature requests Body.Lights.Running.IsOn = TRUE at priority MEDIUM

  # --- REQ-DRL-001 ---
  Scenario: Vehicle started with parking brake already released
    Given Vehicle.LowVoltageSystemState is "OFF"
    And Chassis.ParkingBrake.IsEngaged is FALSE
    When Vehicle.LowVoltageSystemState transitions to "START"
    Then the DRL feature requests Body.Lights.Running.IsOn = TRUE at priority MEDIUM

  # --- REQ-DRL-003 ---
  Scenario: Vehicle started with parking brake engaged
    Given Vehicle.LowVoltageSystemState is "OFF"
    And Chassis.ParkingBrake.IsEngaged is TRUE
    When Vehicle.LowVoltageSystemState transitions to "ON"
    Then the DRL feature does NOT request Body.Lights.Running.IsOn = TRUE

  # --- REQ-DRL-008 ---
  Scenario: DRL works on electric vehicle (no engine)
    Given Vehicle.Powertrain.Type is "ELECTRIC"
    And Vehicle.LowVoltageSystemState is "OFF"
    And Chassis.ParkingBrake.IsEngaged is FALSE
    When Vehicle.LowVoltageSystemState transitions to "ON"
    Then the DRL feature requests Body.Lights.Running.IsOn = TRUE at priority MEDIUM
    And the feature does not reference any engine-related signal

  # --- REQ-DRL-008 ---
  Scenario: DRL works on hybrid vehicle
    Given Vehicle.Powertrain.Type is "HYBRID"
    And Vehicle.LowVoltageSystemState is "ON"
    And Chassis.ParkingBrake.IsEngaged is FALSE
    Then the DRL feature requests Body.Lights.Running.IsOn = TRUE at priority MEDIUM
    And the feature does not depend on engine running state
