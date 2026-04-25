Feature: Thumb-Pad Lock
  As the body controller platform
  I must lock all doors when the driver (or front passenger) holds the
  capacitive thumb pad on the outside door handle for 500 ms
  so that the user can lock the vehicle on exit without using the key fob.

  # -------------------------------------------------------------------------
  # Implementation model
  # -------------------------------------------------------------------------
  # Monitors two capacitive pad signals (Row 1 only — no Row 2 pads):
  #   Body.Doors.Row1.Left.Handle.Outside.LockPad.IsPressed
  #   Body.Doors.Row1.Right.Handle.Outside.LockPad.IsPressed
  #
  # Debounce = 500 ms hold-to-fire (fires at 500 ms, not on release).
  # A release before 500 ms cancels the pending lock.
  # A new press while debouncing resets the 500 ms window.
  # Each pad is independent: either pad alone is sufficient.
  # Publishes FeedbackRequest = "lock" alongside LockAll.
  # -------------------------------------------------------------------------

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-TPL-001: Pressing and holding the Row 1 Left OR Row 1 Right outside
  #              handle thumb pad continuously for 500 ms SHALL issue a LockAll
  #              command via the DoorLockArbiter.
  #
  # REQ-TPL-002: The feature SHALL publish FeedbackRequest = "lock" alongside
  #              the LockAll command.
  #
  # REQ-TPL-003: Releasing the pad before 500 ms have elapsed SHALL cancel
  #              the pending lock. No command is issued.
  #
  # REQ-TPL-004: If the pad is pressed again while a debounce is in progress,
  #              the 500 ms window SHALL restart from the new press time.
  #
  # REQ-TPL-005: After firing, the feature SHALL require a new press to fire
  #              again. Holding the pad continuously past 500 ms SHALL NOT
  #              cause repeated lock commands.
  #
  # REQ-TPL-006: Each pad (left and right) is independent. Either pad alone
  #              is sufficient to trigger a lock. Both pads held simultaneously
  #              result in a single lock command (whichever debounce completes
  #              first fires; the other's state is also cleared).
  #
  # REQ-TPL-007: Row 2 door handles have no thumb pads. The feature SHALL
  #              only monitor Row 1 Left and Row 1 Right pads.
  # -------------------------------------------------------------------------

  Background:
    Given the DoorLock domain arbiter is running
    And the Thumb-Pad Lock feature is running
    And all doors are unlocked

  # --- REQ-TPL-001, REQ-TPL-002 ---
  Scenario: Left pad held for 500 ms — vehicle locks
    When the Row 1 Left thumb pad is pressed
    And 500 ms elapse with the pad continuously held
    Then a LockAll command is issued via the DoorLockArbiter
    And FeedbackRequest "lock" is published

  # --- REQ-TPL-001, REQ-TPL-002 ---
  Scenario: Right pad held for 500 ms — vehicle locks
    When the Row 1 Right thumb pad is pressed
    And 500 ms elapse with the pad continuously held
    Then a LockAll command is issued via the DoorLockArbiter
    And FeedbackRequest "lock" is published

  # --- REQ-TPL-003 ---
  Scenario: Pad released before 500 ms — no lock
    When the Row 1 Left thumb pad is pressed
    And 200 ms elapse
    And the Row 1 Left thumb pad is released
    And an additional 600 ms elapse
    Then no LockAll command is issued

  # --- REQ-TPL-004 ---
  Scenario: New press resets debounce window
    When the Row 1 Left thumb pad is pressed
    And 300 ms elapse
    And the Row 1 Left thumb pad is released
    And the Row 1 Left thumb pad is pressed again immediately
    And 500 ms elapse
    Then a LockAll command is issued
    And the command is issued approximately 500 ms after the second press, not 800 ms after the first

  # --- REQ-TPL-005 ---
  Scenario: No re-fire without releasing and re-pressing
    When the Row 1 Left thumb pad is pressed
    And 500 ms elapse (lock fires)
    And an additional 1000 ms elapse with pad still held
    Then only one LockAll command is issued

  # --- REQ-TPL-006 ---
  Scenario: Both pads held — single lock command fires
    When the Row 1 Left thumb pad is pressed
    And the Row 1 Right thumb pad is pressed simultaneously
    And 500 ms elapse
    Then exactly one LockAll command is issued

  # --- REQ-TPL-006 ---
  Scenario: Right pad fires while left pad has been recently released
    When the Row 1 Left thumb pad is pressed and released before 500 ms
    And the Row 1 Right thumb pad is separately pressed and held for 500 ms
    Then a LockAll command is issued (from the right pad)
    And no spurious command is issued from the cancelled left pad

  # --- REQ-TPL-007 ---
  Scenario: Row 2 doors have no thumb pads — no monitoring
    Given the feature is running
    Then no subscription exists for Row 2 door handle signals
    And only Row 1 Left and Row 1 Right pad signals are monitored
