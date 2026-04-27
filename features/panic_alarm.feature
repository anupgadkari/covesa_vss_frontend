Feature: Panic Alarm
  As the body controller platform
  I must flash both direction indicators and chirp the horn in unison
  whenever the panic-alarm signal is engaged
  so that an authorised user can summon attention to the vehicle in distress.

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-PANIC-001: When Body.Switches.Panic.IsEngaged transitions FALSE→TRUE,
  #               the PanicAlarm feature SHALL request both
  #               DirectionIndicator.Left.IsSignaling and
  #               DirectionIndicator.Right.IsSignaling = TRUE via the
  #               Lighting arbiter at priority HIGH (3).
  #
  # REQ-PANIC-002: When Body.Switches.Panic.IsEngaged is TRUE, the
  #               PanicAlarm feature SHALL request Body.Horn.IsActive = TRUE
  #               via the Horn arbiter at priority HIGH (3) on the same edges
  #               as the indicator pulses, producing chirps perfectly
  #               synchronized with the flash.
  #
  # REQ-PANIC-003: While engaged, the PanicAlarm feature SHALL pulse the
  #               indicators and horn at a 1 Hz cadence (400 ms ON, 600 ms
  #               OFF), matching typical OEM panic alarms.
  #
  # REQ-PANIC-004: When Body.Switches.Panic.IsEngaged transitions TRUE→FALSE,
  #               the PanicAlarm feature SHALL release its Lighting arbiter
  #               claims on both DirectionIndicator.Left.IsSignaling and
  #               DirectionIndicator.Right.IsSignaling, and its Horn arbiter
  #               claim on Body.Horn.IsActive.  Released claims allow lower-
  #               priority pending claims (Turn, Hazard) to resume; if none
  #               exist, the arbiters publish the default-off value.
  #
  # REQ-PANIC-005: On engage transition, the PanicAlarm feature SHALL
  #               publish Vehicle.Body.Alarm.IsActive = TRUE.  On disengage
  #               transition it SHALL publish Vehicle.Body.Alarm.IsActive
  #               = FALSE.  This signal is a steady status flag (single-
  #               owner direct publish, NOT duty-cycled with the pulses)
  #               and serves as the canonical "alarm sounding" signal for
  #               telematics, HMI badges, and BCM event logging.
  #
  # REQ-PANIC-006: The PanicAlarm feature SHALL operate regardless of
  #               Vehicle.LowVoltageSystemState.  A panic press must work
  #               when the vehicle is parked, locked and the ignition is
  #               OFF — this is a security requirement.
  #
  # REQ-PANIC-007: A redundant TRUE while the alarm is already engaged, or
  #               a redundant FALSE while disengaged, SHALL be a no-op.
  #               The feature SHALL NOT restart the pulse loop, SHALL NOT
  #               re-publish Vehicle.Body.Alarm.IsActive, and SHALL NOT
  #               release-and-reclaim arbiter slots on a duplicate edge.
  #
  # REQ-PANIC-008: Each authenticated PANIC press from a paired keyfob
  #               SHALL toggle Body.Switches.Panic.IsEngaged.  Press once
  #               to start the alarm; press again to cancel.  The RKE
  #               feature is the publisher of this signal; PanicAlarm only
  #               consumes it.
  #
  # REQ-PANIC-009: The PanicAlarm feature SHALL claim Lighting indicators
  #               at the same priority as Hazard (HIGH/3).  When a panic
  #               edge is more recent than a Hazard edge, the arbiter's
  #               latest-wins-on-tie rule guarantees PanicAlarm controls
  #               the indicators.  On disengage, the released claim allows
  #               a still-engaged Hazard to resume control of the
  #               indicators automatically.
  #
  # REQ-PANIC-010: While the alarm is engaged, ANY successful authenticated
  #               unlock command SHALL cancel the alarm.  Sources of valid
  #               unlock include: RKE keyfob UNLOCK, smart entry handle
  #               pull (PEPS authenticated), phone-app remote unlock, BLE
  #               key, NFC card.  All such sources publish
  #               Body.Doors.CentralLock.FeedbackRequest = "unlock"; the
  #               PanicAlarm feature SHALL subscribe to this signal and
  #               disengage when it sees "unlock" while engaged.
  #
  # REQ-PANIC-011: When the alarm is cancelled by an unlock feedback, the
  #               PanicAlarm feature SHALL self-publish
  #               Body.Switches.Panic.IsEngaged = FALSE so the switch state
  #               on the bus matches the alarm's actual state.
  #
  # REQ-PANIC-012: A "lock" feedback (AutoRelock, WalkAwayLock, ThumbPad)
  #               SHALL NOT cancel the alarm.  Only "unlock" feedback
  #               cancels.
  # -------------------------------------------------------------------------

  Background:
    Given the Lighting domain arbiter is running
    And the Horn domain arbiter is running
    And the PanicAlarm feature is running

  # --- REQ-PANIC-001, REQ-PANIC-002, REQ-PANIC-005 ---
  Scenario: Engaging panic alarm starts synchronized blink + chirp
    Given the panic switch is not engaged
    When Body.Switches.Panic.IsEngaged transitions to TRUE
    Then Vehicle.Body.Alarm.IsActive becomes TRUE
    And the PanicAlarm feature requests DirectionIndicator.Left.IsSignaling = TRUE at priority HIGH
    And the PanicAlarm feature requests DirectionIndicator.Right.IsSignaling = TRUE at priority HIGH
    And the PanicAlarm feature requests Body.Horn.IsActive = TRUE at priority HIGH

  # --- REQ-PANIC-003 ---
  Scenario: Lights and horn share the same on/off edges (synchronized)
    Given the panic switch is engaged
    When the panic alarm has been running long enough to enter a complete OFF window
    Then both direction indicators are OFF
    And Body.Horn.IsActive is FALSE
    When the panic alarm advances into the next ON window
    Then both direction indicators are ON
    And Body.Horn.IsActive is TRUE

  # --- REQ-PANIC-004, REQ-PANIC-005 ---
  Scenario: Disengaging panic alarm stops blink + chirp and clears the status flag
    Given the panic switch is engaged
    And both indicators and the horn are active under PanicAlarm
    When Body.Switches.Panic.IsEngaged transitions to FALSE
    Then the PanicAlarm feature releases its claim on DirectionIndicator.Left.IsSignaling
    And the PanicAlarm feature releases its claim on DirectionIndicator.Right.IsSignaling
    And the PanicAlarm feature releases its claim on Body.Horn.IsActive
    And Vehicle.Body.Alarm.IsActive becomes FALSE
    And with no other active claim, the arbiters publish default-off on indicators and horn

  # --- REQ-PANIC-005 ---
  Scenario: Vehicle.Body.Alarm.IsActive is a steady status flag, not duty-cycled
    Given the panic switch is engaged
    When the panic alarm runs through three complete pulse cycles
    Then Vehicle.Body.Alarm.IsActive has been published exactly once with value TRUE
    And Vehicle.Body.Alarm.IsActive has not been re-published on any pulse edge

  # --- REQ-PANIC-006 ---
  Scenario: Panic alarm operates with ignition OFF
    Given the vehicle low-voltage system is in state "OFF"
    And the panic switch is not engaged
    When Body.Switches.Panic.IsEngaged transitions to TRUE
    Then Vehicle.Body.Alarm.IsActive becomes TRUE
    And the PanicAlarm feature requests DirectionIndicator.Left.IsSignaling = TRUE at priority HIGH
    And the PanicAlarm feature requests Body.Horn.IsActive = TRUE at priority HIGH

  # --- REQ-PANIC-006 ---
  Scenario: Panic alarm operates with ignition ACC
    Given the vehicle low-voltage system is in state "ACC"
    And the panic switch is not engaged
    When Body.Switches.Panic.IsEngaged transitions to TRUE
    Then Vehicle.Body.Alarm.IsActive becomes TRUE
    And the PanicAlarm feature requests Body.Horn.IsActive = TRUE at priority HIGH

  # --- REQ-PANIC-007 ---
  Scenario: Re-engage while already running is a no-op
    Given the panic switch is engaged
    And Vehicle.Body.Alarm.IsActive is TRUE
    When Body.Switches.Panic.IsEngaged is set to TRUE again
    Then Vehicle.Body.Alarm.IsActive is not re-published
    And the pulse loop continues uninterrupted at the existing cadence

  # --- REQ-PANIC-009 ---
  Scenario: Disengaging panic while hazard remains engaged restores hazard control
    Given the hazard switch is engaged
    And both indicators are signaling at priority HIGH due to hazard
    When Body.Switches.Panic.IsEngaged transitions to TRUE
    Then the PanicAlarm feature claims both indicators at priority HIGH (latest-wins on tie)
    When Body.Switches.Panic.IsEngaged transitions to FALSE
    Then the PanicAlarm feature releases both indicators
    And Hazard's still-engaged claim resumes control of both indicators

  # --- REQ-PANIC-010, REQ-PANIC-011 ---
  Scenario: Successful unlock command cancels a running panic alarm
    Given the panic switch is engaged
    And Vehicle.Body.Alarm.IsActive is TRUE
    When a successful authenticated unlock publishes FeedbackRequest = "unlock"
    Then Vehicle.Body.Alarm.IsActive becomes FALSE
    And Body.Switches.Panic.IsEngaged is self-published as FALSE

  # --- REQ-PANIC-012 ---
  Scenario: A "lock" feedback does NOT cancel an active panic alarm
    Given the panic switch is engaged
    And Vehicle.Body.Alarm.IsActive is TRUE
    When the central-lock bus publishes FeedbackRequest = "lock"
    Then Vehicle.Body.Alarm.IsActive remains TRUE
