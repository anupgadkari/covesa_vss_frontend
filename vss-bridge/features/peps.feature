Feature: Keyfob Passive Entry / Passive Start (KeyfobPeps)
  As the body controller platform
  I must unlock or lock all four doors
  when the Safety Monitor reports a successful keyfob proximity
  authentication (LF antenna)
  so that the driver can enter or secure the vehicle without
  pressing a physical key button.

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-PEPS-001: When Body.PEPS.KeyPresent transitions to TRUE, the feature
  #               SHALL request UNLOCK via the DoorLock arbiter.
  #
  # REQ-PEPS-002: When Body.PEPS.KeyPresent transitions to FALSE, the feature
  #               SHALL request LOCK via the DoorLock arbiter.
  #
  # REQ-PEPS-003: The KeyfobPeps feature on the A53 is NOT in the critical
  #               unlock path. The Safety Monitor (M7) executes the actual
  #               unlock via LLCE/LIN directly. The A53 KeyfobPeps feature
  #               receives the KeyPresent signal AFTER the M7 has already
  #               acted, and its arbiter request serves to keep the
  #               application-layer state consistent.
  #
  # REQ-PEPS-004: KeyfobPeps is a separate requestor from KeyfobRke (manual
  #               button press). Both use the same physical keyfob but are
  #               tracked independently in the DoorLock arbiter and in the
  #               NVM diagnostic log maintained by the Classic AUTOSAR
  #               Locking SWC.
  #
  # REQ-PEPS-005: The KeyfobPeps feature SHALL subscribe to the synthetic
  #               sensor signal Body.PEPS.KeyPresent, which is injected by
  #               the vss-bridge when the Safety Monitor reports a
  #               successful LF key authentication.
  #
  # REQ-PEPS-006: The KeyfobPeps feature SHALL have no dependency on any
  #               other feature module.
  # -------------------------------------------------------------------------

  Background:
    Given the vehicle low-voltage system is in state "ON"
    And the DoorLock arbiter is running
    And the KeyfobPeps feature is running

  # --- REQ-PEPS-001 ---
  Scenario: Key authenticated — unlock all doors
    Given all four doors are locked
    When Body.PEPS.KeyPresent becomes TRUE
    Then the KeyfobPeps feature requests UNLOCK via the DoorLock arbiter

  # --- REQ-PEPS-002 ---
  Scenario: Key departed — lock all doors
    Given all four doors are unlocked
    And Body.PEPS.KeyPresent is TRUE
    When Body.PEPS.KeyPresent becomes FALSE
    Then the KeyfobPeps feature requests LOCK via the DoorLock arbiter

  # --- REQ-PEPS-004 ---
  Scenario: KeyfobPeps tracked separately from KeyfobRke
    Given all four doors are locked
    When Body.PEPS.KeyPresent becomes TRUE (keyfob proximity)
    Then the DoorLock arbiter records requestor = KeyfobPeps
    And the NVM diagnostic entry shows KeyfobPeps as the requestor
    And this is distinct from a KeyfobRke (manual button) request

  # --- REQ-PEPS-003 ---
  Scenario: A53 KeyfobPeps is not in the critical path
    # This scenario documents the architectural constraint, not a testable behavior.
    # The actual unlock is performed by the M7 Safety Monitor via LLCE/LIN.
    Given the driver approaches the vehicle with a valid keyfob
    When the M7 Safety Monitor authenticates the key via LF
    Then the M7 drives the lock actuators directly (< 80 ms)
    And the M7 pushes STATE_UPDATE for Body.Doors.*.IsLocked to the A53
    And only then does Body.PEPS.KeyPresent become TRUE on the A53

  # --- REQ-PEPS-001 ---
  Scenario: Partial door state — some doors already unlocked
    Given Row1.Left and Row1.Right are unlocked
    And Row2.Left and Row2.Right are locked
    When Body.PEPS.KeyPresent becomes TRUE
    Then the KeyfobPeps feature requests UNLOCK via the DoorLock arbiter
    And the Locking SWC drives all four motors regardless of current state
