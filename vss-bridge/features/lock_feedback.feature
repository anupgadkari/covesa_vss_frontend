Feature: Lock Feedback Flash
  As the body controller platform
  I must flash both direction indicators in a distinct pattern
  when the door lock state changes
  so that the driver receives clear visual confirmation of whether
  the vehicle has locked or unlocked — even when hazard or turn
  indicators are already active.

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-LKF-001: When any Body.Doors.*.IsLocked signal transitions from
  #              FALSE to TRUE (unlock → lock), the feature SHALL play the
  #              LOCK pattern: one long flash (~600 ms ON) on both direction
  #              indicators, then release.
  #
  # REQ-LKF-002: When any Body.Doors.*.IsLocked signal transitions from
  #              TRUE to FALSE (lock → unlock), the feature SHALL play the
  #              UNLOCK pattern: two quick flashes (~150 ms ON, 100 ms OFF,
  #              150 ms ON) on both direction indicators, then release.
  #
  # REQ-LKF-003: Lock feedback SHALL OVERLAY on any active hazard or turn
  #              indicator signaling. While the lock/unlock pattern is
  #              playing, it temporarily takes control of both indicators.
  #              After the pattern completes, the underlying hazard or turn
  #              signal resumes without interruption.
  #
  # REQ-LKF-004: To achieve overlay behavior, lock feedback requests SHALL
  #              use priority HIGH (3) — the same as Hazard. This ensures
  #              the pattern is never suppressed by an active turn or
  #              hazard signal. The feature self-limits by releasing
  #              priority after the pattern completes.
  #
  # REQ-LKF-005: After the lock/unlock pattern completes, the feature
  #              SHALL release arbiter ownership by publishing both
  #              indicators = FALSE at priority HIGH. The arbiter then
  #              falls back to the next-highest pending request (hazard
  #              or turn), restoring the underlying signal.
  #
  # REQ-LKF-006: The lock feedback feature SHALL subscribe to door lock
  #              state change events (Body.Doors.*.IsLocked), which are
  #              STATE_UPDATE signals from the Safety Monitor — not switch
  #              inputs.
  #
  # REQ-LKF-007: If a second lock state change occurs during an active
  #              pattern, the current pattern SHALL be interrupted and
  #              the new pattern (lock or unlock) SHALL start immediately.
  #
  # REQ-LKF-008: The lock feedback feature SHALL have no dependency on any
  #              other feature module.
  #
  # REQ-LKF-009: The lock feedback feature owns the blink PATTERN (on/off
  #              durations for the feedback sequence) but does NOT own the
  #              underlying LED blink cadence for hazard/turn signaling.
  #              The lock feedback pattern is a deliberate timed sequence
  #              distinct from the 1-2 Hz regulatory cadence.
  # -------------------------------------------------------------------------

  Background:
    Given the vehicle low-voltage system is in state "ON"
    And the Lighting domain arbiter is running
    And the Lock Feedback feature is running

  # --- REQ-LKF-001 ---
  Scenario: Door locked — one long flash
    Given all doors are unlocked
    When the driver door transitions to locked
    Then the Lock Feedback feature plays the LOCK pattern:
      | Step | Indicators | Duration |
      | 1    | ON         | 600 ms   |
      | 2    | OFF        | release  |
    And both indicators are requested at priority HIGH during the pattern

  # --- REQ-LKF-002 ---
  Scenario: Door unlocked — two quick flashes
    Given the driver door is locked
    When the driver door transitions to unlocked
    Then the Lock Feedback feature plays the UNLOCK pattern:
      | Step | Indicators | Duration |
      | 1    | ON         | 150 ms   |
      | 2    | OFF        | 100 ms   |
      | 3    | ON         | 150 ms   |
      | 4    | OFF        | release  |
    And both indicators are requested at priority HIGH during the pattern

  # --- REQ-LKF-003, REQ-LKF-004 ---
  Scenario: Lock flash overlays on active hazard signaling
    Given the hazard switch is engaged
    And both indicators are signaling due to hazard at priority HIGH
    When a door lock state change occurs (lock event)
    Then the Lock Feedback feature temporarily takes both indicators at priority HIGH
    And the LOCK pattern (one long flash) plays over the hazard signaling
    And after the pattern completes, hazard signaling resumes

  # --- REQ-LKF-003, REQ-LKF-004 ---
  Scenario: Unlock flash overlays on active left turn signal
    Given the turn stalk is in position LEFT
    And the left indicator is signaling at priority MEDIUM
    When a door unlock state change occurs
    Then the Lock Feedback feature takes both indicators at priority HIGH
    And the UNLOCK pattern (two quick flashes) plays on both indicators
    And after the pattern completes, the left turn signal resumes at priority MEDIUM

  # --- REQ-LKF-005 ---
  Scenario: Underlying signal resumes after lock feedback releases
    Given the hazard switch is engaged
    And hazard is actively signaling both indicators
    When a lock event triggers the LOCK feedback pattern
    And the pattern completes (600 ms)
    Then the Lock Feedback feature publishes both indicators = FALSE at priority HIGH
    And the Lighting arbiter falls back to Hazard's pending HIGH request
    And both indicators resume hazard signaling

  # --- REQ-LKF-005 ---
  Scenario: No underlying signal — indicators turn off after pattern
    Given no hazard or turn signal is active
    And all indicators are off
    When a lock event triggers the LOCK feedback pattern
    And the pattern completes
    Then the Lock Feedback feature releases at priority HIGH
    And both indicators turn off (no pending request to fall back to)

  # --- REQ-LKF-007 ---
  Scenario: Lock during active unlock pattern — pattern restarts
    Given a door unlock event triggered the UNLOCK pattern
    And the UNLOCK pattern is mid-sequence (first flash ON)
    When a second event occurs (door transitions to locked)
    Then the UNLOCK pattern is interrupted
    And the LOCK pattern (one long flash) starts immediately

  # --- REQ-LKF-007 ---
  Scenario: Rapid lock-lock does not stack patterns
    Given a door lock event triggered the LOCK pattern
    When a second door lock event occurs during the pattern
    Then the LOCK pattern restarts from the beginning
    And only one pattern plays at a time

  # --- REQ-LKF-001, REQ-LKF-002 ---
  Scenario: Multiple doors change state simultaneously
    Given all four doors are unlocked
    When all four doors transition to locked simultaneously (PEPS lock-all)
    Then only one LOCK pattern plays (not four stacked patterns)
    And the first lock event triggers the pattern; subsequent events within
    the pattern window are absorbed
