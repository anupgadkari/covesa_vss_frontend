Feature: Passive Entry / Passive Start (PEPS)
  As the body controller platform
  I must unlock or lock all four doors
  when the Safety Monitor reports a successful key authentication
  so that the driver can enter or secure the vehicle without
  pressing a physical key button.

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-PEPS-001: When Body.PEPS.KeyPresent transitions to TRUE, the feature
  #               SHALL request all four door lock signals
  #               (Body.Doors.Row[1,2].{Left,Right}.IsLocked) = FALSE
  #               via the DoorLock arbiter at priority HIGH (3).
  #
  # REQ-PEPS-002: When Body.PEPS.KeyPresent transitions to FALSE, the feature
  #               SHALL request all four door lock signals = TRUE
  #               via the DoorLock arbiter at priority HIGH (3).
  #
  # REQ-PEPS-003: The PEPS feature on the A53 is NOT in the critical unlock
  #               path. The Safety Monitor (M7) executes the actual
  #               unlock via LLCE/LIN directly. The A53 PEPS feature
  #               receives the KeyPresent signal AFTER the M7 has already
  #               acted, and its arbiter request serves to keep the
  #               application-layer state consistent.
  #
  # REQ-PEPS-004: PEPS requests SHALL use priority HIGH (3), which wins
  #               over AutoLock (MEDIUM/2) in the DoorLock arbiter.
  #
  # REQ-PEPS-005: The PEPS feature SHALL subscribe to the synthetic sensor
  #               signal Body.PEPS.KeyPresent, which is injected by the
  #               vss-bridge when the Safety Monitor reports a successful
  #               LF key authentication.
  #
  # REQ-PEPS-006: The PEPS feature SHALL have no dependency on any other
  #               feature module.
  # -------------------------------------------------------------------------

  Background:
    Given the vehicle low-voltage system is in state "ON"
    And the DoorLock domain arbiter is running
    And the PEPS feature is running

  # --- REQ-PEPS-001 ---
  Scenario: Key authenticated — unlock all doors
    Given all four doors are locked
    When Body.PEPS.KeyPresent becomes TRUE
    Then the PEPS feature requests Body.Doors.Row1.Left.IsLocked = FALSE at priority HIGH
    And the PEPS feature requests Body.Doors.Row1.Right.IsLocked = FALSE at priority HIGH
    And the PEPS feature requests Body.Doors.Row2.Left.IsLocked = FALSE at priority HIGH
    And the PEPS feature requests Body.Doors.Row2.Right.IsLocked = FALSE at priority HIGH

  # --- REQ-PEPS-002 ---
  Scenario: Key departed — lock all doors
    Given all four doors are unlocked
    And Body.PEPS.KeyPresent is TRUE
    When Body.PEPS.KeyPresent becomes FALSE
    Then the PEPS feature requests all four door lock signals = TRUE at priority HIGH

  # --- REQ-PEPS-004 ---
  Scenario: PEPS unlock overrides AutoLock
    Given AutoLock has locked all doors at priority MEDIUM
    When Body.PEPS.KeyPresent becomes TRUE
    Then the PEPS feature's HIGH unlock request wins over AutoLock's MEDIUM lock
    And all four doors are requested unlocked

  # --- REQ-PEPS-003 ---
  Scenario: A53 PEPS is not in the critical path
    # This scenario documents the architectural constraint, not a testable behavior.
    # The actual unlock is performed by the M7 Safety Monitor via LLCE/LIN.
    Given the driver approaches the vehicle with a valid key
    When the M7 Safety Monitor authenticates the key via LF
    Then the M7 drives the lock actuators directly (< 80 ms)
    And the M7 pushes STATE_UPDATE for Body.Doors.*.IsLocked to the A53
    And only then does Body.PEPS.KeyPresent become TRUE on the A53

  # --- REQ-PEPS-001 ---
  Scenario: Partial door state — some doors already unlocked
    Given Row1.Left and Row1.Right are unlocked
    And Row2.Left and Row2.Right are locked
    When Body.PEPS.KeyPresent becomes TRUE
    Then the PEPS feature requests all four doors unlocked
    And the arbiter publishes unlock for all four signals regardless of current state
