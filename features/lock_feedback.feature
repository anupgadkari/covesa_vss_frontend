Feature: Lock / Unlock Feedback Flash
  As the body controller platform
  I must flash both direction indicators in a distinct timed pattern
  when an external-origin lock or unlock event occurs
  so that the driver receives clear visual confirmation of the vehicle
  state — even when hazard or turn indicators are already active.

  # -------------------------------------------------------------------------
  # Implementation model
  # -------------------------------------------------------------------------
  # This feature subscribes to Body.Doors.CentralLock.FeedbackRequest
  # (published by RKE, WalkAwayLock, ThumbPadLock, AutoRelock) and plays
  # timed flash patterns on both direction indicators via the Lighting domain
  # arbiter at priority HIGH.
  #
  # Flash unit = 100 ms OFF lead-in → 900 ms ON.
  # Gap between unlock flash units = 300 ms OFF.
  # -------------------------------------------------------------------------

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-LKF-001: When FeedbackRequest = "lock" is received, the feature SHALL
  #              play the LOCK pattern: [100 ms OFF] [900 ms ON] → release.
  #
  # REQ-LKF-002: When FeedbackRequest = "unlock" is received, the feature SHALL
  #              play the UNLOCK pattern:
  #              [100 ms OFF] [900 ms ON] [300 ms OFF] [100 ms OFF] [900 ms ON]
  #              → release.
  #
  # REQ-LKF-003: When FeedbackRequest = "trunk_unlock" is received, the feature
  #              SHALL play the UNLOCK pattern and additionally arm a trunk-close
  #              latch: when Body.Trunk.IsOpen subsequently transitions to false,
  #              a LOCK pattern SHALL be played automatically.
  #
  # REQ-LKF-004: Lock feedback SHALL overlay on any active hazard or turn
  #              indicator signaling. Both indicators are claimed at priority HIGH.
  #              After the pattern completes the claims are released, allowing the
  #              underlying hazard or turn signal to resume uninterrupted.
  #
  # REQ-LKF-005: If a new FeedbackRequest arrives while a pattern is playing,
  #              the current task SHALL be aborted, all arbiter claims released,
  #              and the new pattern SHALL start immediately from the beginning.
  #
  # REQ-LKF-006: Unknown FeedbackRequest values SHALL be silently ignored
  #              with a warning log.
  #
  # REQ-LKF-007: The lead-in OFF pulse ensures the flash has a visible start
  #              edge even when the indicators are already illuminated.
  # -------------------------------------------------------------------------

  Background:
    Given the Lighting domain arbiter is running
    And the Lock Feedback feature is running
    And no hazard or turn signal is active

  # --- REQ-LKF-001 ---
  Scenario: Lock feedback — single flash unit
    When FeedbackRequest "lock" is published
    Then the Lock Feedback feature plays the LOCK pattern on both indicators:
      | Phase       | Indicators | Duration |
      | Lead-in OFF | OFF        | 100 ms   |
      | Flash ON    | ON         | 900 ms   |
      | Release     | released   | —        |
    And both indicators are claimed at priority HIGH during the pattern

  # --- REQ-LKF-002 ---
  Scenario: Unlock feedback — two flash units with gap
    When FeedbackRequest "unlock" is published
    Then the Lock Feedback feature plays the UNLOCK pattern on both indicators:
      | Phase         | Indicators | Duration |
      | Lead-in OFF   | OFF        | 100 ms   |
      | Flash 1 ON    | ON         | 900 ms   |
      | Gap OFF       | OFF        | 300 ms   |
      | Lead-in 2 OFF | OFF        | 100 ms   |
      | Flash 2 ON    | ON         | 900 ms   |
      | Release       | released   | —        |
    And both indicators are claimed at priority HIGH during the pattern

  # --- REQ-LKF-003 ---
  Scenario: Trunk unlock — unlock pattern then arm trunk-close latch
    When FeedbackRequest "trunk_unlock" is published
    Then the UNLOCK pattern plays (same as REQ-LKF-002)
    And the feature arms an internal trunk-close flag
    When Body.Trunk.IsOpen transitions to false
    Then the LOCK pattern plays automatically
    And the trunk-close flag is cleared

  # --- REQ-LKF-003 ---
  Scenario: Trunk closes without prior trunk_unlock — no feedback
    Given no FeedbackRequest "trunk_unlock" has been received
    When Body.Trunk.IsOpen transitions to false
    Then no flash pattern is played

  # --- REQ-LKF-004 ---
  Scenario: Lock flash overlays on active hazard signaling
    Given the hazard switch is engaged and both indicators are at priority HIGH
    When FeedbackRequest "lock" is published
    Then the Lock Feedback feature claims both indicators at priority HIGH
    And the LOCK pattern plays over the existing hazard signaling
    And after the pattern completes and claims are released, hazard signaling resumes

  # --- REQ-LKF-005 ---
  Scenario: New request preempts in-progress pattern
    Given FeedbackRequest "unlock" was published and the pattern is mid-sequence
    When FeedbackRequest "lock" is published before the unlock pattern finishes
    Then the unlock pattern is aborted immediately
    And all arbiter claims are released
    And the LOCK pattern starts from the beginning
    And only one ON event occurs (the single lock flash)

  # --- REQ-LKF-006 ---
  Scenario: Unknown FeedbackRequest value is ignored
    When FeedbackRequest "activate_missiles" is published
    Then no flash pattern is played
    And a warning is logged

  # --- REQ-LKF-007 ---
  Scenario: Lead-in OFF creates visible start edge when indicators already lit
    Given both indicators are currently ON due to hazard
    When FeedbackRequest "lock" is published
    Then the indicators first go OFF for 100 ms (the lead-in)
    And then go ON for 900 ms (the flash)
    And the start edge is visible even though indicators were already lit
