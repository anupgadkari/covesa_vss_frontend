Feature: Auto Relock
  As the body controller platform
  I must automatically re-lock the vehicle
  if it was unlocked but no door was opened within a configurable
  timeout (default 45 seconds)
  so that the vehicle is not left inadvertently unlocked.

  The set of monitored doors is determined by the variant calibration
  (DoorConfig). A 2-door coupe only monitors Row1. A vehicle with
  removable doors (Bronco, Wrangler) dynamically excludes any door
  whose IsRemoved signal is TRUE.

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-ARL-001: When all *present* doors transition from locked to unlocked
  #              (any unlock source), the feature SHALL start a relock timer
  #              (default 45 seconds). "Present" means the door exists in the
  #              variant DoorConfig AND (if removable) is not currently removed.
  #
  # REQ-ARL-002: If no Body.Doors.*.IsOpen signal transitions to TRUE on any
  #              *present* door before the relock timer expires, the feature
  #              SHALL request LOCK via the DoorLock arbiter with requestor
  #              AutoRelock.
  #
  # REQ-ARL-003: If any *present* door's Body.Doors.*.IsOpen signal transitions
  #              to TRUE before the relock timer expires, the relock timer
  #              SHALL be cancelled. No automatic relock occurs.
  #
  # REQ-ARL-004: The relock timer SHALL be cancelled if the doors are
  #              re-locked by any other source before the timer expires
  #              (e.g., the driver presses the keyfob lock button).
  #
  # REQ-ARL-005: The relock timeout SHALL be a Tier 2 (vehicle-line)
  #              calibration parameter. Default: 45 seconds.
  #
  # REQ-ARL-006: The Auto Relock feature SHALL subscribe to IsLocked and
  #              IsOpen signals only for doors present in the variant
  #              DoorConfig. A 2-door coupe SHALL NOT subscribe to Row2
  #              signals that do not exist.
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
  #
  # REQ-ARL-011: On vehicles with removable doors (DoorConfig.removable =
  #              true), the feature SHALL subscribe to Body.Doors.*.IsRemoved
  #              for each present door. A door whose IsRemoved = TRUE SHALL
  #              be excluded from both unlock detection and open detection.
  #              If all non-removed doors are unlocked, the timer starts.
  #              If a removed door is re-installed (IsRemoved → FALSE), it
  #              SHALL be included in monitoring from that point forward.
  #
  # REQ-ARL-012: The Auto Relock feature SHALL be disabled via Tier 4
  #              (dealer config) DealerConfig.auto_relock_enabled. When
  #              disabled, the feature does not start timers or request
  #              locks. It SHALL re-enable at runtime if the dealer config
  #              is updated (via UDS 0x2E) without requiring a power cycle.
  # -------------------------------------------------------------------------

  # ═══════════════════════════════════════════════════════════════════════
  # Standard 4-door sedan scenarios
  # ═══════════════════════════════════════════════════════════════════════

  Background:
    Given the vehicle low-voltage system is in state "ON"
    And the DoorLock arbiter is running
    And the Auto Relock feature is running
    And the relock timeout is 45 seconds

  # --- REQ-ARL-001, REQ-ARL-002 ---
  Scenario: No door opened after unlock — vehicle re-locks (4-door)
    Given the variant has 4 doors (sedan)
    And all four doors are locked
    When the vehicle is unlocked (by any source)
    And 45 seconds elapse without any door being opened
    Then the Auto Relock feature requests LOCK via the DoorLock arbiter
    And the requestor is AutoRelock

  # --- REQ-ARL-003 ---
  Scenario: Door opened before timeout — relock cancelled
    Given the variant has 4 doors (sedan)
    And all four doors are locked
    When the vehicle is unlocked
    And the driver opens the driver door within 45 seconds
    Then the relock timer is cancelled
    And the Auto Relock feature does NOT request LOCK

  # --- REQ-ARL-003 ---
  Scenario: Door opened and closed — no relock
    Given the variant has 4 doors (sedan)
    And all four doors are locked
    When the vehicle is unlocked
    And the driver opens and then closes the driver door within 45 seconds
    Then the relock timer was cancelled when the door opened
    And the Auto Relock feature does NOT request LOCK
    And the vehicle remains unlocked (driver's responsibility)

  # --- REQ-ARL-004 ---
  Scenario: Re-locked by another source before timeout — timer cancelled
    Given the variant has 4 doors (sedan)
    And all four doors are locked
    When the vehicle is unlocked
    And 20 seconds elapse without any door being opened
    And the driver presses the keyfob lock button (KeyfobRke)
    Then the relock timer is cancelled (doors already locked)
    And the Auto Relock feature does NOT request LOCK

  # --- REQ-ARL-008 ---
  Scenario: Second unlock restarts the timer
    Given the variant has 4 doors (sedan)
    And all four doors are locked
    When the vehicle is unlocked (first unlock)
    And 30 seconds elapse
    And a second unlock event occurs (e.g., PhoneApp remote unlock)
    Then the relock timer restarts from 0
    And the vehicle will auto-relock 45 seconds after the second unlock
    (if no door is opened)

  # --- REQ-ARL-001 ---
  Scenario: Partial unlock does not start timer
    Given the variant has 4 doors (sedan)
    And Row1.Left is unlocked and all other doors are locked
    Then the Auto Relock feature does NOT start the relock timer
    And the feature waits until all present doors report unlocked

  # ═══════════════════════════════════════════════════════════════════════
  # 2-door coupe scenarios
  # ═══════════════════════════════════════════════════════════════════════

  # --- REQ-ARL-006 ---
  Scenario: 2-door coupe — only Row1 doors are monitored
    Given the variant has 2 doors (coupe)
    And both doors (Row1.Left and Row1.Right) are locked
    When the vehicle is unlocked
    And 45 seconds elapse without either door being opened
    Then the Auto Relock feature requests LOCK via the DoorLock arbiter
    And only Row1.Left and Row1.Right lock signals were monitored
    And no Row2 signals were subscribed to

  # --- REQ-ARL-006 ---
  Scenario: 2-door coupe — door opened cancels timer
    Given the variant has 2 doors (coupe)
    And both doors are locked
    When the vehicle is unlocked
    And the driver opens Row1.Left within 45 seconds
    Then the relock timer is cancelled

  # ═══════════════════════════════════════════════════════════════════════
  # Removable door scenarios (Bronco / Wrangler)
  # ═══════════════════════════════════════════════════════════════════════

  # --- REQ-ARL-011 ---
  Scenario: All doors present — standard behavior
    Given the variant has 4 removable doors
    And no doors are currently removed (all IsRemoved = FALSE)
    And all four doors are locked
    When the vehicle is unlocked
    And 45 seconds elapse without any door being opened
    Then the Auto Relock feature requests LOCK via the DoorLock arbiter

  # --- REQ-ARL-011 ---
  Scenario: Rear doors removed — only front doors monitored for unlock
    Given the variant has 4 removable doors
    And Row2.Left.IsRemoved = TRUE and Row2.Right.IsRemoved = TRUE
    And Row1.Left and Row1.Right are locked
    When Row1.Left and Row1.Right are unlocked
    Then the relock timer starts (all non-removed doors are unlocked)
    And 45 seconds elapse without Row1.Left or Row1.Right being opened
    Then the Auto Relock feature requests LOCK via the DoorLock arbiter

  # --- REQ-ARL-011 ---
  Scenario: Rear doors removed — front door opened cancels timer
    Given the variant has 4 removable doors
    And Row2.Left.IsRemoved = TRUE and Row2.Right.IsRemoved = TRUE
    And all non-removed doors are unlocked (relock timer running)
    When the driver opens Row1.Left
    Then the relock timer is cancelled

  # --- REQ-ARL-011 ---
  Scenario: Removed door does not count as "opened"
    Given the variant has 4 removable doors
    And Row2.Right.IsRemoved = TRUE
    And all non-removed doors (Row1.Left, Row1.Right, Row2.Left) are unlocked
    And the relock timer is running
    Then the IsOpen state of Row2.Right is ignored
    And the relock timer continues based on Row1.Left, Row1.Right, and Row2.Left only

  # --- REQ-ARL-011 ---
  Scenario: Door re-installed mid-timer — included in monitoring
    Given the variant has 4 removable doors
    And Row1.Right.IsRemoved = TRUE
    And Row1.Left, Row2.Left, Row2.Right are unlocked (timer running)
    When Row1.Right.IsRemoved transitions to FALSE (door re-installed)
    Then Row1.Right is now included in open monitoring
    And if Row1.Right.IsOpen transitions to TRUE, the timer is cancelled

  # ═══════════════════════════════════════════════════════════════════════
  # Crash and power cycle scenarios (apply to all variants)
  # ═══════════════════════════════════════════════════════════════════════

  # --- REQ-ARL-009 ---
  Scenario: Crash during relock timer — timer permanently cancelled
    Given all present doors are locked
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

  # ═══════════════════════════════════════════════════════════════════════
  # Configuration and traceability scenarios
  # ═══════════════════════════════════════════════════════════════════════

  # --- REQ-ARL-002 ---
  Scenario: Auto Relock as a DoorLock arbiter requestor
    Given the vehicle was unlocked 45 seconds ago and no door was opened
    When the Auto Relock feature submits LOCK to the DoorLock arbiter
    Then the arbiter records the requestor as AutoRelock
    And the NVM diagnostic entry shows AutoRelock as the lock source

  # --- REQ-ARL-005 ---
  Scenario: Configurable timeout (Tier 2 vehicle-line calibration)
    Given the vehicle-line calibration sets auto_relock_timeout_secs to 30
    And all present doors are locked
    When the vehicle is unlocked
    And 30 seconds elapse without any door being opened
    Then the Auto Relock feature requests LOCK via the DoorLock arbiter

  # --- REQ-ARL-012 ---
  Scenario: Feature disabled via dealer config
    Given the dealer has set auto_relock_enabled = false (DID 0xF190)
    And all present doors are locked
    When the vehicle is unlocked
    Then the Auto Relock feature does NOT start the relock timer
    And no automatic relock occurs regardless of elapsed time

  # --- REQ-ARL-012 ---
  Scenario: Feature re-enabled via dealer config at runtime
    Given the dealer has set auto_relock_enabled = false
    And the vehicle is running (no restart)
    When the dealer updates auto_relock_enabled = true via UDS 0x2E
    And the M7 pushes the updated config to the A53
    Then the Auto Relock feature begins monitoring for unlock events
    And the next unlock will start the relock timer normally
