Feature: Keyfob Passive Entry / Passive Start (KeyfobPeps)
  As the body controller platform
  I must unlock or lock all four doors
  when the Safety Monitor reports a successful keyfob proximity
  authentication
  so that the driver can enter or secure the vehicle without
  pressing a physical key button.

  # -------------------------------------------------------------------------
  # Wake-up chain (hardware, not A53 software)
  # -------------------------------------------------------------------------
  # The PEPS unlock sequence begins in hardware, not in this Rust feature:
  #
  #   1. Door handle capacitive touch sensor detects hand presence
  #      (always powered, µA draw — the only always-on component)
  #   2. Capacitive sensor interrupt wakes the M7 from sleep
  #   3. M7 drives LF antennas in door handles (transmit challenge,
  #      a few seconds window)
  #   4. Keyfob receives LF, replies on UHF RF (315/433 MHz)
  #   5. M7 validates RF response (crypto challenge-response)
  #   6. M7 drives lock motors directly via LLCE/LIN
  #   7. M7 wakes A53, pushes STATE_UPDATE for Body.Doors.*.IsLocked
  #   8. A53 receives Body.PEPS.KeyPresent = TRUE (post-facto)
  #
  # The A53 and this Rust feature are NOT in the critical path.
  # Steps 1–6 complete in < 80 ms. The A53 may still be booting
  # when the doors are already unlocked.
  # -------------------------------------------------------------------------

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
  #               unlock path. The wake-up chain (capacitive touch → M7 LF
  #               challenge → keyfob RF response → M7 lock motor drive)
  #               executes entirely on M7 hardware while the A53 may still
  #               be asleep or booting. The A53 KeyfobPeps feature receives
  #               the KeyPresent signal AFTER the M7 has already acted, and
  #               its arbiter request serves only to keep the application-
  #               layer state consistent.
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
  #               successful keyfob authentication.
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
    # The actual unlock is performed entirely by M7 hardware:
    #   capacitive touch → LF challenge → RF response → lock motor
    Given the vehicle is in LowVoltageSystemState "OFF" (parked, A53 asleep)
    When the driver touches the door handle (capacitive sensor)
    Then the capacitive sensor interrupt wakes the M7
    And the M7 transmits LF challenge via door handle antennas
    And the keyfob replies on UHF RF (315/433 MHz)
    And the M7 validates the crypto challenge-response
    And the M7 drives the lock actuators directly via LLCE/LIN (< 80 ms total)
    And the M7 wakes the A53 and pushes STATE_UPDATE for Body.Doors.*.IsLocked
    And only then does Body.PEPS.KeyPresent become TRUE on the A53

  # --- REQ-PEPS-001 ---
  Scenario: Partial door state — some doors already unlocked
    Given Row1.Left and Row1.Right are unlocked
    And Row2.Left and Row2.Right are locked
    When Body.PEPS.KeyPresent becomes TRUE
    Then the KeyfobPeps feature requests UNLOCK via the DoorLock arbiter
    And the Locking SWC drives all four motors regardless of current state
