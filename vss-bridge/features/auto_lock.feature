Feature: Speed-Based Auto Lock
  As the body controller platform
  I must automatically lock all doors
  when the vehicle reaches a configurable speed threshold
  so that occupants are secured during driving as a passive
  safety measure.

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-ALK-001: When Vehicle.Speed rises above the lock threshold (default:
  #              15 km/h), the feature SHALL request all four door lock
  #              signals (Body.Doors.Row[1,2].{Left,Right}.IsLocked) = TRUE
  #              via the DoorLock arbiter at priority MEDIUM (2).
  #
  # REQ-ALK-002: The auto lock feature SHALL NOT automatically unlock the
  #              doors when speed drops below the threshold. Unlocking is
  #              exclusively the driver's action (via PEPS or manual).
  #
  # REQ-ALK-003: The auto lock feature SHALL NOT re-lock doors that have
  #              been manually unlocked by the driver while the vehicle is
  #              above the speed threshold. The feature SHALL only trigger
  #              on the rising-edge speed crossing.
  #
  # REQ-ALK-004: Auto lock requests SHALL use priority MEDIUM (2). The PEPS
  #              feature (HIGH/3) can override auto lock in the DoorLock
  #              arbiter.
  #
  # REQ-ALK-005: The auto lock feature SHALL subscribe to Vehicle.Speed
  #              (standard VSS v4.0 sensor signal).
  #
  # REQ-ALK-006: The auto lock feature SHALL have no dependency on any other
  #              feature module.
  #
  # REQ-ALK-007: The speed threshold SHALL be configurable (compile-time
  #              constant or runtime configuration). Default: 15 km/h.
  # -------------------------------------------------------------------------

  Background:
    Given the vehicle low-voltage system is in state "ON"
    And the DoorLock domain arbiter is running
    And the Auto Lock feature is running
    And the auto lock speed threshold is 15 km/h

  # --- REQ-ALK-001 ---
  Scenario: Vehicle exceeds speed threshold — doors auto-lock
    Given all four doors are unlocked
    And Vehicle.Speed is 10 km/h
    When Vehicle.Speed rises to 16 km/h
    Then the Auto Lock feature requests all four door lock signals = TRUE at priority MEDIUM

  # --- REQ-ALK-001 ---
  Scenario: Vehicle already above threshold at startup — doors lock
    Given all four doors are unlocked
    When Vehicle.Speed is reported as 20 km/h on initial state sync
    Then the Auto Lock feature requests all four door lock signals = TRUE at priority MEDIUM

  # --- REQ-ALK-002 ---
  Scenario: Vehicle slows below threshold — doors do NOT auto-unlock
    Given all four doors are locked by auto lock
    And Vehicle.Speed is 20 km/h
    When Vehicle.Speed drops to 5 km/h
    Then the Auto Lock feature does NOT request any door unlock
    And all doors remain locked

  # --- REQ-ALK-003 ---
  Scenario: Doors manually unlocked above threshold — no re-lock
    Given all four doors were auto-locked at 16 km/h
    And the driver manually unlocks the driver door (via PEPS or interior switch)
    When Vehicle.Speed remains at 20 km/h
    Then the Auto Lock feature does NOT re-lock the driver door
    And no additional lock requests are published

  # --- REQ-ALK-003 ---
  Scenario: Rising-edge trigger only — no repeated locks
    Given all four doors were auto-locked at 16 km/h
    When Vehicle.Speed fluctuates between 14 and 16 km/h repeatedly
    Then the Auto Lock feature locks doors on each rising-edge crossing
    But does NOT publish duplicate lock requests while continuously above threshold

  # --- REQ-ALK-004 ---
  Scenario: PEPS unlock overrides auto lock
    Given all four doors are locked by auto lock at priority MEDIUM
    When Body.PEPS.KeyPresent becomes TRUE
    Then the PEPS feature's HIGH unlock request wins in the DoorLock arbiter
    And all four doors are requested unlocked despite auto lock

  # --- REQ-ALK-001 ---
  Scenario: Vehicle stopped — auto lock does not trigger
    Given all four doors are unlocked
    And Vehicle.Speed is 0 km/h
    Then the Auto Lock feature does NOT request any door lock
