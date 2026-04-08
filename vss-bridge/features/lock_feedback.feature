Feature: Lock Feedback Flash
  As the body controller platform
  I must briefly flash both direction indicators
  when the door lock state changes
  so that the driver receives visual confirmation that the vehicle
  has locked or unlocked.

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-LKF-001: When any Body.Doors.*.IsLocked signal transitions (either
  #              TRUE→FALSE or FALSE→TRUE), the feature SHALL request both
  #              DirectionIndicator.Left.IsSignaling = TRUE and
  #              DirectionIndicator.Right.IsSignaling = TRUE via the
  #              Lighting arbiter at priority LOW (1).
  #
  # REQ-LKF-002: The indicator flash SHALL last 500 ms. After 500 ms the
  #              feature SHALL request both indicators = FALSE at priority
  #              LOW (1), releasing arbitration ownership.
  #
  # REQ-LKF-003: Lock feedback requests SHALL use priority LOW (1). Both
  #              Hazard (HIGH/3) and Turn Indicator (MEDIUM/2) override
  #              lock feedback in the Lighting arbiter.
  #
  # REQ-LKF-004: The lock feedback feature SHALL subscribe to door lock
  #              state change events (Body.Doors.*.IsLocked), which are
  #              STATE_UPDATE signals from the Safety Monitor — not switch
  #              inputs.
  #
  # REQ-LKF-005: The lock feedback feature SHALL NOT distinguish between
  #              lock and unlock events — both produce the same flash.
  #
  # REQ-LKF-006: If a second lock state change occurs during an active
  #              flash, the flash timer SHALL restart (extend the flash,
  #              not stack a second one).
  #
  # REQ-LKF-007: The lock feedback feature SHALL have no dependency on any
  #              other feature module.
  # -------------------------------------------------------------------------

  Background:
    Given the vehicle low-voltage system is in state "ON"
    And the Lighting domain arbiter is running
    And the Lock Feedback feature is running

  # --- REQ-LKF-001, REQ-LKF-002 ---
  Scenario: Door locked triggers a brief indicator flash
    Given all doors are unlocked
    When the driver door transitions to locked
    Then the Lock Feedback feature requests both indicators = TRUE at priority LOW
    And after 500 ms the Lock Feedback feature requests both indicators = FALSE at priority LOW

  # --- REQ-LKF-005 ---
  Scenario: Door unlocked also triggers a flash
    Given the driver door is locked
    When the driver door transitions to unlocked
    Then the Lock Feedback feature requests both indicators = TRUE at priority LOW
    And after 500 ms the Lock Feedback feature requests both indicators = FALSE at priority LOW

  # --- REQ-LKF-003 ---
  Scenario: Lock feedback flash suppressed by active hazard
    Given the hazard switch is engaged
    And both indicators are signaling at priority HIGH
    When a door lock state change occurs
    Then the Lock Feedback feature's LOW request is suppressed by the Lighting arbiter
    And both indicators continue signaling due to Hazard's HIGH priority

  # --- REQ-LKF-003 ---
  Scenario: Lock feedback flash suppressed by active turn signal
    Given the turn stalk is in position LEFT
    And the left indicator is signaling at priority MEDIUM
    When a door lock state change occurs
    Then the Lock Feedback feature's LOW request for the left indicator is suppressed
    And the right indicator flashes briefly at priority LOW

  # --- REQ-LKF-006 ---
  Scenario: Rapid lock-unlock restarts the flash timer
    Given all doors are unlocked
    When the driver door transitions to locked
    And within 200 ms the driver door transitions to unlocked
    Then the flash timer restarts from the second event
    And the total flash duration from the second event is 500 ms

  # --- REQ-LKF-002 ---
  Scenario: Flash releases arbiter ownership after timeout
    When a door lock state change triggers a flash
    And 500 ms elapses
    Then the Lock Feedback feature publishes both indicators = FALSE at priority LOW
    And the Lighting arbiter's winner for both indicator signals is cleared
