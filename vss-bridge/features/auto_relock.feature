Feature: Auto Relock
  As the body controller platform
  I must automatically re-lock the vehicle
  if it was unlocked but no door was opened within a configurable
  timeout (default 45 seconds)
  so that the vehicle is not left inadvertently unlocked.

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-ARL-001: When all doors transition from locked to unlocked (any
  #              unlock source), the feature SHALL start a relock timer
  #              (default 45 seconds).
  #
  # REQ-ARL-002: If no Body.Doors.*.IsOpen signal transitions to TRUE
  #              before the relock timer expires, the feature SHALL
  #              request LOCK via the DoorLock arbiter with requestor
  #              AutoRelock.
  #
  # REQ-ARL-003: If any Body.Doors.*.IsOpen signal transitions to TRUE
  #              before the relock timer expires, the relock timer SHALL
  #              be cancelled. No automatic relock occurs.
  #
  # REQ-ARL-004: The relock timer SHALL be cancelled if the doors are
  #              re-locked by any other source before the timer expires
  #              (e.g., the driver presses the keyfob lock button).
  #
  # REQ-ARL-005: The relock timeout SHALL be configurable (compile-time
  #              constant or runtime configuration). Default: 45 seconds.
  #
  # REQ-ARL-006: The Auto Relock feature SHALL subscribe to:
  #              - Body.Doors.*.IsLocked (STATE_UPDATE from Safety Monitor)
  #              - Body.Doors.*.IsOpen (STATE_UPDATE from Safety Monitor)
  #              - Vehicle.Safety.CrashDetected (STATE_UPDATE, see ARL-009)
  #              These are state signals, not switch inputs.
  #
  # REQ-ARL-007: The Auto Relock feature SHALL have no dependency on any
  #              other feature module. It does not know which feature
  #              triggered the original unlock.
  #
  # REQ-ARL-008: If the vehicle is unlocked and the relock timer is
  #              running, and a second unlock event arrives (e.g., from
  #              a different requestor), the timer SHALL restart.
  #
  # REQ-ARL-009: If Vehicle.Safety.CrashDetected transitions to TRUE at
  #              any time, the Auto Relock feature SHALL immediately and
  #              permanently cancel the relock timer (if running) and
  #              enter a DISABLED state. The feature SHALL NOT start any
  #              new relock timers until the next vehicle power cycle
  #              (LowVoltageSystemState OFF → ON). Rationale: the 10s
  #              arbiter lockout expires before the 45s relock timer,
  #              so relying on arbiter rejection alone would allow a
  #              post-crash relock — a safety hazard for occupants and
  #              first responders.
  #
  # REQ-ARL-010: The Auto Relock feature SHALL additionally subscribe
  #              to Vehicle.Safety.CrashDetected (STATE_UPDATE from
  #              Safety Monitor) to implement REQ-ARL-009.
  # -------------------------------------------------------------------------

  Background:
    Given the vehicle low-voltage system is in state "ON"
    And the DoorLock arbiter is running
    And the Auto Relock feature is running
    And the relock timeout is 45 seconds

  # --- REQ-ARL-001, REQ-ARL-002 ---
  Scenario: No door opened after unlock — vehicle re-locks
    Given all four doors are locked
    When the vehicle is unlocked (by any source)
    And 45 seconds elapse without any door being opened
    Then the Auto Relock feature requests LOCK via the DoorLock arbiter
    And the requestor is AutoRelock

  # --- REQ-ARL-003 ---
  Scenario: Door opened before timeout — relock cancelled
    Given all four doors are locked
    When the vehicle is unlocked
    And the driver opens the driver door within 45 seconds
    Then the relock timer is cancelled
    And the Auto Relock feature does NOT request LOCK

  # --- REQ-ARL-003 ---
  Scenario: Door opened and closed — no relock
    Given all four doors are locked
    When the vehicle is unlocked
    And the driver opens and then closes the driver door within 45 seconds
    Then the relock timer was cancelled when the door opened
    And the Auto Relock feature does NOT request LOCK
    And the vehicle remains unlocked (driver's responsibility)

  # --- REQ-ARL-004 ---
  Scenario: Re-locked by another source before timeout — timer cancelled
    Given all four doors are locked
    When the vehicle is unlocked
    And 20 seconds elapse without any door being opened
    And the driver presses the keyfob lock button (KeyfobRke)
    Then the relock timer is cancelled (doors already locked)
    And the Auto Relock feature does NOT request LOCK

  # --- REQ-ARL-008 ---
  Scenario: Second unlock restarts the timer
    Given all four doors are locked
    When the vehicle is unlocked (first unlock)
    And 30 seconds elapse
    And a second unlock event occurs (e.g., PhoneApp remote unlock)
    Then the relock timer restarts from 0
    And the vehicle will auto-relock 45 seconds after the second unlock
    (if no door is opened)

  # --- REQ-ARL-001 ---
  Scenario: Partial unlock does not start timer
    Given Row1.Left is unlocked and all other doors are locked
    Then the Auto Relock feature does NOT start the relock timer
    And the feature waits until all doors report unlocked

  # --- REQ-ARL-009 ---
  Scenario: Crash during relock timer — timer permanently cancelled
    Given all four doors are locked
    And the vehicle is unlocked (relock timer starts)
    And 10 seconds elapse
    When Vehicle.Safety.CrashDetected transitions to TRUE
    Then the relock timer is immediately cancelled
    And the Auto Relock feature enters the DISABLED state
    And the Auto Relock feature does NOT request LOCK — not now, not later

  # --- REQ-ARL-009 ---
  Scenario: Crash before any unlock — feature disables
    Given the vehicle is running normally (no crash)
    When Vehicle.Safety.CrashDetected transitions to TRUE
    Then the Auto Relock feature enters the DISABLED state
    And no future unlock event will start a relock timer

  # --- REQ-ARL-009 ---
  Scenario: Feature re-enables after power cycle (OFF → ON)
    Given Vehicle.Safety.CrashDetected was TRUE (feature is DISABLED)
    When Vehicle.LowVoltageSystemState transitions to "OFF"
    And then Vehicle.LowVoltageSystemState transitions to "ON"
    Then the Auto Relock feature re-enters the ENABLED state
    And future unlock events will start the relock timer normally

  # --- REQ-ARL-009 ---
  Scenario: Feature re-enables after power cycle (OFF → ACC → ON)
    Given Vehicle.Safety.CrashDetected was TRUE (feature is DISABLED)
    When Vehicle.LowVoltageSystemState transitions to "OFF"
    And then Vehicle.LowVoltageSystemState transitions to "ACC"
    And then Vehicle.LowVoltageSystemState transitions to "ON"
    Then the Auto Relock feature re-enters the ENABLED state
    And ACC between OFF and ON does not block the recovery

  # --- REQ-ARL-002 ---
  Scenario: Auto Relock as a DoorLock arbiter requestor
    Given the vehicle was unlocked 45 seconds ago and no door was opened
    When the Auto Relock feature submits LOCK to the DoorLock arbiter
    Then the arbiter records the requestor as AutoRelock
    And the NVM diagnostic entry shows AutoRelock as the lock source

  # --- REQ-ARL-005 ---
  Scenario: Configurable timeout
    Given the relock timeout is configured to 30 seconds
    And all four doors are locked
    When the vehicle is unlocked
    And 30 seconds elapse without any door being opened
    Then the Auto Relock feature requests LOCK via the DoorLock arbiter
