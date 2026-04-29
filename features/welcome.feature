Feature: Welcome (PEPS approach courtesy lighting)
  As the body controller platform
  I must illuminate the exterior puddle lamps and interior dome light
  when an authenticated PEPS device enters the vehicle's LF coverage
  so that the approaching user has visibility while reaching for the door.

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-WEL-001: When any paired key fob or BLE phone transitions from a
  #              "no LF" zone (OutOfRange, RfRange) into ANY LF coverage
  #              zone (Approach + the proximity zones), the feature SHALL
  #              claim Body.Lights.Puddle.Left.IsOn,
  #              Body.Lights.Puddle.Right.IsOn, and
  #              Cabin.Lights.IsDomeOn = TRUE via the courtesy arbiter
  #              at MEDIUM priority.
  #
  # REQ-WEL-002: The hold duration SHALL default to 30 s; configurable
  #              via the platform's vehicle-line calibration once that
  #              field exists.  Tests use shorter durations via
  #              `Welcome::with_hold(...)`.
  #
  # REQ-WEL-003: The hold SHALL release early when Vehicle.LowVoltageSystemState
  #              transitions to "ON" or "START" — the driver is in the
  #              seat and courtesy lighting is no longer useful.
  #
  # REQ-WEL-004: The hold SHALL release early when all paired devices
  #              leave LF coverage (back to OutOfRange / RfRange).  If
  #              the user walks away before the timer expires, the
  #              lights go off.
  #
  # REQ-WEL-005: Multiple devices entering LF serially SHALL NOT extend
  #              the hold.  The first arrival latches a single deadline;
  #              later arrivals within the window are no-ops.  This
  #              prevents two people walking up sequentially from
  #              doubling the courtesy duration.
  #
  # REQ-WEL-006: A device transitioning to RfRange (which is NOT LF
  #              coverage) SHALL NOT arm Welcome.  Only Approach and the
  #              proximity zones count as LF entry edges.
  # -------------------------------------------------------------------------

  Background:
    Given the courtesy arbiter is running
    And the Welcome feature is running with a short hold for testing

  # --- REQ-WEL-001 ---
  Scenario: Fob entering Approach arms courtesy lights
    Given paired fob 1 is in OutOfRange
    When paired fob 1 moves to Approach
    Then Body.Lights.Puddle.Left.IsOn becomes TRUE
    And Body.Lights.Puddle.Right.IsOn becomes TRUE
    And Cabin.Lights.IsDomeOn becomes TRUE

  # --- REQ-WEL-002 ---
  Scenario: Lights release after the hold expires
    Given paired fob 1 is in Approach
    And Body.Lights.Puddle.Left.IsOn is TRUE
    When the welcome hold elapses
    Then Body.Lights.Puddle.Left.IsOn becomes FALSE

  # --- REQ-WEL-003 ---
  Scenario: Ignition ON releases courtesy lights early
    Given paired fob 1 is in Approach
    And Body.Lights.Puddle.Left.IsOn is TRUE
    When Vehicle.LowVoltageSystemState transitions to "ON"
    Then Body.Lights.Puddle.Left.IsOn becomes FALSE

  # --- REQ-WEL-004 ---
  Scenario: All devices leaving LF release courtesy lights
    Given paired fob 1 is in Approach
    And Body.Lights.Puddle.Left.IsOn is TRUE
    When paired fob 1 moves to OutOfRange
    Then Body.Lights.Puddle.Left.IsOn becomes FALSE

  # --- REQ-WEL-005 ---
  Scenario: A second device arriving mid-hold does NOT extend the deadline
    Given paired fob 1 entered Approach at the start of the hold
    When 50% of the hold elapses
    And paired fob 2 also enters Approach
    Then the original deadline is unchanged
    And the lights still release at the original deadline

  # --- REQ-WEL-006 ---
  Scenario: Fob transitioning to RfRange (no LF) does not arm Welcome
    Given paired fob 1 is in OutOfRange
    When paired fob 1 moves to RfRange
    Then Body.Lights.Puddle.Left.IsOn remains its initial value
