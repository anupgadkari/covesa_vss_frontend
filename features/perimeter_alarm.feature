Feature: Perimeter Alarm (anti-intrusion alarm on unauthorised door open)
  As the body controller platform
  I must escalate from a soft warning chime to a full horn-and-light alarm
  whenever a door is opened while the cabin is locked and no authorised
  unlock has happened first
  so that an attacker cannot enter the vehicle silently and a legitimate
  driver who entered with a mechanical-blade key has a brief window to
  authenticate before the exterior alarm fires.

  # -------------------------------------------------------------------------
  # Implementation model
  # -------------------------------------------------------------------------
  # The PerimeterAlarm feature subscribes to:
  #   • Body.Doors.Row{1,2}.{Left,Right}.IsOpen   — trigger source
  #   • Cabin.LockStatus                          — arming gate
  #   • Cabin.LockStatus.LastRequestor            — disarm-source identity
  #   • Body.Switches.Panic.IsEngaged             — disarm via panic press
  #
  # On the FALSE→TRUE edge of Cabin.LockStatus into LOCKED / DOUBLE_LOCKED
  # the feature starts a 20 s pre-arm timer.  Until the timer elapses,
  # door-open events are ignored.  Once armed, a door-open spawns a
  # three-phase pulse task:
  #
  #   1. Chime warning (0..12 s).      Pulse Body.Chime.IsActive only.
  #                                    Vehicle.Body.Alarm.IsActive stays
  #                                    FALSE so HMI/telematics know this
  #                                    is a warning, not a real alarm.
  #
  #   2. Full alarm    (12..42 s).     Vehicle.Body.Alarm.IsActive = TRUE.
  #                                    Pulse direction indicators (both),
  #                                    dome, exterior puddle lamps, AND
  #                                    Body.Horn.IsActive in unison.
  #
  #   3. Lights only   (42..312 s).    Release horn.  Indicators / dome /
  #                                    puddle continue pulsing until the
  #                                    full 5 min total elapses, then the
  #                                    pulse loop self-exits.
  #
  # All pulses use the same 1 Hz cadence (400 ms ON, 600 ms OFF) PanicAlarm
  # uses, so the two are visually indistinguishable while active.
  # -------------------------------------------------------------------------

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-PERI-001: When Cabin.LockStatus transitions from a non-armable
  #               state (UNLOCKED / DRIVER_UNLOCKED) into an armable state
  #               (LOCKED / DOUBLE_LOCKED), the PerimeterAlarm feature
  #               SHALL start a 20 s pre-arm timer.  During the pre-arm
  #               window, door-open events SHALL be silently ignored — no
  #               chime, no horn, no lights, no Vehicle.Body.Alarm.IsActive
  #               publish.  The pre-arm window absorbs the common case of
  #               "lock the car, then realise you forgot your phone."
  #
  # REQ-PERI-002: A LOCKED ↔ DOUBLE_LOCKED transition SHALL NOT restart
  #               the pre-arm timer.  Both states are armable; the timer
  #               only resets when the cabin returns to a non-armable
  #               state and is later re-locked.
  #
  # REQ-PERI-003: When Cabin.LockStatus transitions from an armable state
  #               back to a non-armable state, the PerimeterAlarm feature
  #               SHALL clear its pre-arm timer.  A subsequent re-lock
  #               starts a fresh 20 s window.
  #
  # REQ-PERI-004: When any of Body.Doors.Row{1,2}.{Left,Right}.IsOpen
  #               transitions FALSE→TRUE while Cabin.LockStatus is LOCKED
  #               or DOUBLE_LOCKED AND the pre-arm window has elapsed,
  #               the PerimeterAlarm feature SHALL begin the chime
  #               warning phase.
  #
  # REQ-PERI-005: During the 12 s chime warning phase, the PerimeterAlarm
  #               feature SHALL pulse Body.Chime.IsActive at 1 Hz
  #               (400 ms ON, 600 ms OFF).  It SHALL NOT request horn,
  #               direction indicators, dome, or puddle lamps.  It SHALL
  #               NOT publish Vehicle.Body.Alarm.IsActive — the status
  #               flag stays at its prior value (default FALSE) so HMI /
  #               telematics can distinguish "warning chime" from "real
  #               intrusion alarm."
  #
  # REQ-PERI-006: At the end of the 12 s chime phase, if no disarm event
  #               has occurred, the PerimeterAlarm feature SHALL:
  #                 • publish Body.Chime.IsActive = FALSE,
  #                 • publish Vehicle.Body.Alarm.IsActive = TRUE, and
  #                 • begin pulsing direction indicators (both),
  #                   Cabin.Lights.IsDomeOn, both Body.Lights.Puddle.*,
  #                   and Body.Horn.IsActive at 1 Hz, all on the same
  #                   ON/OFF edges (same cadence as PanicAlarm), all
  #                   claimed via their domain arbiters at priority HIGH.
  #
  # REQ-PERI-007: 30 s after the chime phase ends (i.e., 42 s after the
  #               original door-open), the PerimeterAlarm feature SHALL
  #               release its Horn arbiter claim.  Direction indicators,
  #               dome, and puddle lamps SHALL continue to pulse.
  #
  # REQ-PERI-008: 5 minutes after the chime phase ends (i.e., 312 s after
  #               the original door-open), the PerimeterAlarm feature
  #               SHALL release all remaining arbiter claims (lighting,
  #               courtesy, puddle) and publish Body.Chime.IsActive =
  #               FALSE.  Vehicle.Body.Alarm.IsActive is NOT auto-cleared
  #               on natural expiry — the next lock/unlock cycle clears
  #               it via the disarm path or via REQ-PERI-009.
  #
  # REQ-PERI-009: When Cabin.LockStatus transitions to UNLOCKED or
  #               DRIVER_UNLOCKED AND Cabin.LockStatus.LastRequestor is
  #               one of the externally-authenticated unlock sources
  #               (KeyfobRke, KeyfobPeps, PassiveEntry, PhoneApp, PhoneBle,
  #               NfcCard, NfcPhone), the PerimeterAlarm feature SHALL
  #               immediately disarm: abort the pulse task, release all
  #               arbiter claims, publish Body.Chime.IsActive = FALSE, and
  #               publish Vehicle.Body.Alarm.IsActive = FALSE.  Disarm
  #               works at any phase — chime, full alarm, or lights-only.
  #
  # REQ-PERI-010: A thumb-pad / sill-knob / HMI-direct unlock (any source
  #               NOT in the external-auth list above) SHALL NOT disarm a
  #               running alarm.  These sources are not authenticated
  #               against a paired device; treating them as disarm
  #               candidates would let an attacker who reached the
  #               thumb-pad bypass the alarm.
  #
  # REQ-PERI-011: When Body.Switches.Panic.IsEngaged transitions to TRUE
  #               while a PerimeterAlarm sequence is running (any phase),
  #               the PerimeterAlarm feature SHALL immediately disarm
  #               (same teardown path as REQ-PERI-009).  This lets the
  #               user grab the panic button as a "stop everything"
  #               override even without a successful unlock.
  #
  # REQ-PERI-012: Disarm during the chime warning phase SHALL NOT trigger
  #               any horn / indicator / dome / puddle output.  A
  #               legitimate driver who authenticates within the 12 s
  #               window must produce zero exterior alarm activity.
  #
  # REQ-PERI-013: Additional door-open edges while a sequence is already
  #               running SHALL be ignored.  The alarm does NOT restart,
  #               extend, or re-publish Vehicle.Body.Alarm.IsActive on
  #               subsequent door opens within the same active sequence.
  #
  # REQ-PERI-014: PerimeterAlarm and PanicAlarm SHALL be mutually
  #               exclusive at the actuator level.  Both claim the same
  #               lighting / horn arbiter slots at priority HIGH.  In
  #               practice only one runs at a time — a panic-button press
  #               disarms a running PerimeterAlarm via REQ-PERI-011, and
  #               a successful unlock cancels PanicAlarm via its own
  #               FeedbackRequest = "unlock" watcher.  No bus-level
  #               contention.
  #
  # REQ-PERI-015: All pulses (chime, horn, indicators, dome, puddle)
  #               SHALL use the cadence 400 ms ON / 600 ms OFF, identical
  #               to PanicAlarm, so the two are visually / audibly
  #               indistinguishable to a bystander while active.
  #
  # REQ-PERI-016: When Vehicle.LowVoltageSystemState transitions to ON
  #               or START while a sequence is in flight (chime, full
  #               alarm, or lights-only phase), the PerimeterAlarm
  #               feature SHALL immediately disarm: abort the pulse
  #               task, release all arbiter claims, publish
  #               Body.Chime.IsActive = FALSE, and publish
  #               Vehicle.Body.Alarm.IsActive = FALSE.  Cranking or
  #               running the engine proves the operator has matched
  #               the immobiliser; we kill the alarm rather than have
  #               the owner drive away with the horn blaring.
  #
  # REQ-PERI-017: ACC alone SHALL NOT disarm the alarm.  ACC can be
  #               reached by twisting a stolen mechanical-blade key or
  #               jiggling a pried ignition cylinder; only ON / START
  #               (which require immobiliser pass on a modern vehicle)
  #               cancels.
  #
  # REQ-PERI-018: The set of recognised external lock requestors —
  #               i.e. the requestor identities that take the alarm
  #               from DISARMED to PRE_ARMED on a fresh lock cycle —
  #               SHALL include `KeyfobRke`, `KeyfobPeps`,
  #               `ThumbPadLock`, `AutoRelock`, `WalkAwayLock`,
  #               `PhoneApp`, `PhoneBle`, `NfcCard`, `NfcPhone`, and
  #               `SlamLock`.  See `slam_lock.feature` for the
  #               provenance of `SlamLock` events on US slam-lock-
  #               allowed vehicle lines.  The set SHALL NOT include
  #               `AutoLock` or `DoorTrimButton` — neither represents
  #               the user actively walking away from the vehicle.
  #
  # REQ-PERI-019: `SlamLock` SHALL NOT be a member of either the auth-
  #               unlock disarm set or the internal-unlock tampering-
  #               trigger set.  This means a thief pressing the trim
  #               Lock button during an active chime cannot use the
  #               EU slam-lock-protect inversion's `SlamLock` unlock
  #               event to silently disarm the alarm.  Defended by
  #               `slam_lock_intruder_chime_survives_*` regression
  #               tests in `perimeter_alarm.rs`.
  # -------------------------------------------------------------------------

  Background:
    Given the Lighting domain arbiter is running
    And the Horn domain arbiter is running
    And the Courtesy domain arbiter is running
    And the Puddle domain arbiter is running
    And the PerimeterAlarm feature is running

  # --- REQ-PERI-001 ---
  Scenario: Door open during the 20 s pre-arm window is silently ignored
    Given Cabin.LockStatus has just transitioned to "LOCKED"
    And less than 20 seconds have elapsed since that transition
    When the driver opens the Row1.Left door
    Then Vehicle.Body.Alarm.IsActive remains FALSE
    And Body.Chime.IsActive does NOT pulse
    And Body.Horn.IsActive does NOT pulse
    And neither direction indicator pulses

  # --- REQ-PERI-001, REQ-PERI-004 ---
  Scenario: Door open just after the pre-arm window starts the chime phase
    Given Cabin.LockStatus has been "LOCKED" for at least 20 seconds
    When the driver opens the Row1.Left door
    Then Body.Chime.IsActive begins pulsing at 1 Hz (400 ms ON / 600 ms OFF)
    And Vehicle.Body.Alarm.IsActive remains FALSE during the chime phase

  # --- REQ-PERI-002 ---
  Scenario: LOCKED → DOUBLE_LOCKED does not restart the pre-arm timer
    Given Cabin.LockStatus has been "LOCKED" for 15 seconds
    When Cabin.LockStatus transitions to "DOUBLE_LOCKED"
    And 6 more seconds elapse
    And the driver opens the Row1.Left door
    Then Body.Chime.IsActive begins pulsing
    # Total armable time = 15 + 6 = 21 s, past the pre-arm window.

  # --- REQ-PERI-003 ---
  Scenario: Unlock-then-relock resets the pre-arm window
    Given Cabin.LockStatus has been "LOCKED" for 15 seconds
    When Cabin.LockStatus transitions to "UNLOCKED"
    And Cabin.LockStatus transitions back to "LOCKED"
    And 6 seconds elapse since the second lock
    And the driver opens the Row1.Left door
    Then Body.Chime.IsActive does NOT pulse
    # Pre-arm timer was cleared on the unlock; only 6 s elapsed since re-lock.

  # --- REQ-PERI-005 ---
  Scenario: Chime warning phase publishes only the chime signal
    Given Cabin.LockStatus has been "LOCKED" for at least 20 seconds
    When the driver opens the Row1.Left door
    And the chime phase has been running for 5 seconds
    Then Body.Chime.IsActive is pulsing at 1 Hz
    And Body.Horn.IsActive has not been published TRUE
    And Body.Lights.DirectionIndicator.Left.IsSignaling has not been published TRUE
    And Cabin.Lights.IsDomeOn has not been published TRUE
    And Body.Lights.Puddle.Left.IsOn has not been published TRUE
    And Vehicle.Body.Alarm.IsActive remains FALSE

  # --- REQ-PERI-006 ---
  Scenario: Chime escalates to full alarm after 12 seconds
    Given Cabin.LockStatus has been "LOCKED" for at least 20 seconds
    When the driver opens the Row1.Left door
    And 13 seconds elapse
    Then Body.Chime.IsActive has been published FALSE
    And Vehicle.Body.Alarm.IsActive is TRUE
    And Body.Horn.IsActive is pulsing at priority HIGH
    And both direction indicators are pulsing at priority HIGH
    And Cabin.Lights.IsDomeOn is pulsing at priority HIGH
    And both puddle lamps are pulsing at priority HIGH

  # --- REQ-PERI-007 ---
  Scenario: Horn releases 30 s after chime ends; lights continue
    Given a PerimeterAlarm sequence is in the full-alarm phase
    When 31 seconds have elapsed since the chime ended
    Then the PerimeterAlarm feature has released its Horn arbiter claim
    And Body.Horn.IsActive has settled FALSE
    And the direction indicators continue pulsing
    And Vehicle.Body.Alarm.IsActive remains TRUE

  # --- REQ-PERI-008 ---
  Scenario: Pulse loop self-exits 5 minutes after the chime ends
    Given a PerimeterAlarm sequence is in the lights-only phase
    When 5 minutes have elapsed since the chime ended
    Then the PerimeterAlarm feature has released its Lighting, Courtesy and Puddle claims
    And Body.Chime.IsActive is FALSE
    And the direction indicators have come to rest at FALSE

  # --- REQ-PERI-009 ---
  Scenario: Authenticated unlock during full alarm disarms immediately
    Given a PerimeterAlarm sequence is in the full-alarm phase
    When Cabin.LockStatus transitions to "UNLOCKED"
    And Cabin.LockStatus.LastRequestor becomes "PassiveEntry"
    Then Vehicle.Body.Alarm.IsActive becomes FALSE
    And Body.Chime.IsActive is FALSE
    And the PerimeterAlarm feature has released all arbiter claims

  # --- REQ-PERI-010 ---
  Scenario: Thumb-pad unlock does NOT disarm a running alarm
    Given a PerimeterAlarm sequence is in the full-alarm phase
    When Cabin.LockStatus transitions to "UNLOCKED"
    And Cabin.LockStatus.LastRequestor becomes "ThumbPadLock"
    Then Vehicle.Body.Alarm.IsActive remains TRUE
    And the alarm continues pulsing

  # --- REQ-PERI-011 ---
  Scenario: Panic-button press during full alarm disarms immediately
    Given a PerimeterAlarm sequence is in the full-alarm phase
    When Body.Switches.Panic.IsEngaged transitions to TRUE
    Then Vehicle.Body.Alarm.IsActive becomes FALSE
    And the PerimeterAlarm feature has released all arbiter claims

  # --- REQ-PERI-012 ---
  Scenario: Authenticated unlock during chime phase produces no exterior output
    Given the chime warning phase is running
    When Cabin.LockStatus transitions to "UNLOCKED"
    And Cabin.LockStatus.LastRequestor becomes "KeyfobRke"
    Then Body.Chime.IsActive is FALSE
    And Vehicle.Body.Alarm.IsActive is FALSE
    And Body.Horn.IsActive has never been published TRUE during this sequence
    And neither direction indicator has been published TRUE during this sequence

  # --- REQ-PERI-013 ---
  Scenario: Additional door opens during a running sequence are ignored
    Given a PerimeterAlarm sequence is in the full-alarm phase
    When the driver opens the Row1.Right door
    Then Vehicle.Body.Alarm.IsActive remains TRUE
    And the pulse cadence does not restart or extend

  # --- REQ-PERI-014 ---
  Scenario: Panic press while perimeter alarm runs cleanly hands off control
    Given a PerimeterAlarm sequence is in the full-alarm phase
    When Body.Switches.Panic.IsEngaged transitions to TRUE
    Then PerimeterAlarm releases its Lighting and Horn arbiter claims
    And PanicAlarm may now claim the same actuators without contention

  # --- REQ-PERI-016 ---
  Scenario: Ignition ON during chime cancels silently
    Given the chime warning phase is running
    When Vehicle.LowVoltageSystemState transitions to "ON"
    Then Body.Chime.IsActive is FALSE
    And Vehicle.Body.Alarm.IsActive is FALSE
    And Body.Horn.IsActive has never been published TRUE during this sequence

  # --- REQ-PERI-016 ---
  Scenario: Ignition ON during full alarm disarms immediately
    Given a PerimeterAlarm sequence is in the full-alarm phase
    When Vehicle.LowVoltageSystemState transitions to "ON"
    Then Vehicle.Body.Alarm.IsActive becomes FALSE
    And the PerimeterAlarm feature has released all arbiter claims

  # --- REQ-PERI-017 ---
  Scenario: ACC during chime does NOT disarm
    Given the chime warning phase is running
    When Vehicle.LowVoltageSystemState transitions to "ACC"
    Then Body.Chime.IsActive continues pulsing
    And the chime will still escalate to full alarm at the 12 s mark

  # --- REQ-PERI-015 ---
  Scenario: All pulse outputs share identical 1 Hz cadence
    Given a PerimeterAlarm sequence is in the full-alarm phase
    When the sequence is observed across one full pulse period
    Then Body.Horn.IsActive ON edges align with the direction indicator ON edges
    And the dome and puddle ON edges align with the same ON edge
    And the OFF window is 600 ms and the ON window is 400 ms
