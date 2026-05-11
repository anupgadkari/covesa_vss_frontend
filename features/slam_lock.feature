Feature: Slam-lock & slam-lock-protect (interior trim Lock with door open)
  As the body controller platform
  I must apply market-appropriate behaviour when a user presses the
  interior trim Lock button while a door is open
  so that US lines support "walk-away with hands full" slam-locking
  while EU lines defend against accidental keys-in-vehicle and
  occupant-trapping events.

  # -------------------------------------------------------------------------
  # Implementation model
  # -------------------------------------------------------------------------
  # The behaviour splits in two features sharing the FeatureId
  # `SlamLock`:
  #
  #   * `door_trim_button.rs` — dispatches the LOCK request.  Subscribes
  #     to the four `Body.Doors.Row*.IsOpen` signals so it can branch
  #     three ways at the moment of trim Lock press:
  #         * all closed         → LockAll as DoorTrimButton
  #         * any open + cal=true (EU)  → LockAll as DoorTrimButton
  #         * any open + cal=false (US) → LockAll as SlamLock
  #
  #   * `slam_lock.rs` — fires only on EU lines.  Observes the same
  #     trim-Lock edge with door state, dispatches the corresponding
  #     unlock as SlamLock:
  #         * driver-side trim + two_stage=true  → UnlockDriver
  #         * driver-side trim + two_stage=false → UnlockAll
  #         * passenger-side trim                → UnlockAll  (bypass)
  #
  # On the bus the EU flow is a deterministic two-event sequence:
  #
  #     EventNum N    LOCKED                      requestor=DoorTrimButton
  #     EventNum N+1  UNLOCKED | DRIVER_UNLOCKED  requestor=SlamLock
  #
  # The US flow is a single event:
  #
  #     EventNum N    LOCKED                      requestor=SlamLock
  # -------------------------------------------------------------------------

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-SLP-001: The vehicle line cal `vehicle_line.slam_lock_protect`
  #              SHALL be a boolean read at boot and applied via the
  #              reboot loop.  Default `true` — defensive against
  #              keys-in-vehicle and accidental-occupant-lock events.
  #              Plant-level commitment: assembly lines ship to one
  #              market or the other, not both, so the value is fixed
  #              for an entire vehicle line.
  #
  # REQ-SLP-002: When the user presses an interior trim Lock button
  #              (Row1 Left or Right) while ALL doors are closed,
  #              `DoorTrimButton` SHALL dispatch `LockAll` via the
  #              DoorLockArbiter with `feature_id = DoorTrimButton`,
  #              regardless of the cal value.  This is the standard
  #              interior lock — neither slam-lock semantic applies.
  #
  # REQ-SLP-003: When the user presses an interior trim Lock button
  #              while ANY door is open AND
  #              `vehicle_line.slam_lock_protect = false` (US),
  #              `DoorTrimButton` SHALL dispatch `LockAll` with
  #              `feature_id = SlamLock` instead of `DoorTrimButton`.
  #              The `SlamLock` requestor is in PerimeterAlarm's
  #              `EXTERNAL_LOCK_REQUESTORS`, so the resulting bus event
  #              arms the alarm — when the user closes the door, the
  #              cabin is already locked and the 20 s pre-arm window
  #              starts.
  #
  # REQ-SLP-004: When the user presses an interior trim Lock button
  #              while ANY door is open AND
  #              `vehicle_line.slam_lock_protect = true` (EU),
  #              `DoorTrimButton` SHALL dispatch `LockAll` with
  #              `feature_id = DoorTrimButton` (the standard interior
  #              identity).  The `SlamLock` feature SHALL then dispatch
  #              the corresponding unlock per REQ-SLP-005..007.
  #
  # REQ-SLP-005: Under EU cal, when the trim Lock pressed is on the
  #              **driver side** (`Row1.Left` under LHD, `Row1.Right`
  #              under RHD) AND `dealer.two_stage_unlock = true`, the
  #              `SlamLock` feature SHALL dispatch `UnlockDriver` via
  #              the DoorLockArbiter with `feature_id = SlamLock`.
  #              This puts the cabin into `DRIVER_UNLOCKED` —
  #              passenger doors stay locked.
  #
  # REQ-SLP-006: Under EU cal, when the trim Lock pressed is on the
  #              **driver side** AND `dealer.two_stage_unlock = false`,
  #              the `SlamLock` feature SHALL dispatch `UnlockAll` via
  #              the DoorLockArbiter with `feature_id = SlamLock`.
  #
  # REQ-SLP-007: Under EU cal, when the trim Lock pressed is on the
  #              **passenger side** (the Row 1 door opposite the driver
  #              side per `vehicle_line.driver_door_side`), the
  #              `SlamLock` feature SHALL dispatch `UnlockAll`
  #              regardless of `dealer.two_stage_unlock`.  Mirrors
  #              PassiveEntry's REQ-PE-009 / REQ-PE-011 passenger-side
  #              bypass: a passenger seat occupant pressing lock while
  #              their door is open is unambiguous "unlock everything."
  #
  # REQ-SLP-008: RHD support — `vehicle_line.driver_door_side = Right`
  #              swaps the meaning of "driver side" and "passenger side"
  #              in REQ-SLP-005..007.  `Row1.Right` becomes the driver
  #              door, `Row1.Left` the passenger.
  #
  # REQ-SLP-009: The `SlamLock` requestor SHALL be a member of
  #              PerimeterAlarm's `EXTERNAL_LOCK_REQUESTORS` list (so a
  #              US slam-lock event arms the alarm) and SHALL NOT be a
  #              member of either `EXTERNAL_AUTH_SOURCES` (so EU
  #              inversion's unlock cannot disarm a running alarm) or
  #              `INTERNAL_UNLOCK_SOURCES` (so it cannot trigger a
  #              tampering chime).  This split is the security guarantee
  #              against an intruder using the trim-lock-during-chime
  #              path to silently cancel an alarm — see
  #              `slam_lock_intruder_chime_survives_*` regression tests.
  #
  # REQ-SLP-010: Every successful inversion under EU cal SHALL publish
  #              `Body.Doors.CentralLock.FeedbackRequest = "unlock"` so
  #              `LockFeedback` plays the standard 2-flash unlock
  #              pattern.
  # -------------------------------------------------------------------------

  Background:
    Given the DoorLockArbiter is running
    And the DoorTrimButton feature is running
    And the SlamLock feature is running

  # --- REQ-SLP-002 ---
  Scenario: Trim lock with all doors closed dispatches as DoorTrimButton
    Given all four doors are closed
    When the driver presses the Row1.Left trim Lock button
    Then Body.Doors.CentralLock.Command becomes "lock_all"
    And Cabin.LockStatus.LastRequestor becomes "DoorTrimButton"

  # --- REQ-SLP-003 ---
  Scenario: US — trim lock with door open dispatches as SlamLock
    Given vehicle_line.slam_lock_protect is FALSE
    And the Row1.Left door is open
    When the driver presses the Row1.Left trim Lock button
    Then Body.Doors.CentralLock.Command becomes "lock_all"
    And Cabin.LockStatus.LastRequestor becomes "SlamLock"
    # The user closes the door physically; cabin is already locked.
    # PerimeterAlarm sees SlamLock in EXTERNAL_LOCK_REQUESTORS and arms.

  # --- REQ-SLP-004, REQ-SLP-005 ---
  Scenario: EU — driver-side trim lock + two-stage unlock inverts to UnlockDriver
    Given vehicle_line.slam_lock_protect is TRUE
    And dealer.two_stage_unlock is TRUE
    And vehicle_line.driver_door_side is Left
    And the Row1.Left door is open
    When the driver presses the Row1.Left trim Lock button
    Then Cabin.LockStatus.EventNum bumps with LockStatus = "LOCKED" and LastRequestor = "DoorTrimButton"
    And Cabin.LockStatus.EventNum bumps again with LockStatus = "DRIVER_UNLOCKED" and LastRequestor = "SlamLock"
    And Body.Doors.CentralLock.FeedbackRequest = "unlock" is published

  # --- REQ-SLP-006 ---
  Scenario: EU — driver-side trim lock + two-stage off inverts to UnlockAll
    Given vehicle_line.slam_lock_protect is TRUE
    And dealer.two_stage_unlock is FALSE
    And vehicle_line.driver_door_side is Left
    And the Row1.Left door is open
    When the driver presses the Row1.Left trim Lock button
    Then Cabin.LockStatus.LastRequestor cycles "DoorTrimButton" → "SlamLock"
    And the second event has LockStatus = "UNLOCKED"

  # --- REQ-SLP-007 ---
  Scenario: EU — passenger-side trim lock always inverts to UnlockAll
    Given vehicle_line.slam_lock_protect is TRUE
    And vehicle_line.driver_door_side is Left
    And the Row1.Right door is open
    When the passenger presses the Row1.Right trim Lock button
    Then the inversion's LockStatus is "UNLOCKED"
    # Passenger-side bypass — `dealer.two_stage_unlock` does not apply.

  # --- REQ-SLP-008 ---
  Scenario: RHD — driver and passenger sides swap
    Given vehicle_line.slam_lock_protect is TRUE
    And dealer.two_stage_unlock is TRUE
    And vehicle_line.driver_door_side is Right
    And the Row1.Right door is open
    When the driver presses the Row1.Right trim Lock button
    Then the inversion's LockStatus is "DRIVER_UNLOCKED"
    # On RHD, Row1.Right is the driver side and respects two-stage.
    Given the Row1.Left door is open
    When the (passenger) presses the Row1.Left trim Lock button
    Then the inversion's LockStatus is "UNLOCKED"
    # On RHD, Row1.Left is the passenger side — always full UnlockAll.

  # --- REQ-SLP-009 ---
  Scenario: SlamLock cannot disarm a running alarm (US slam-lock path)
    Given the perimeter alarm is in the chime phase
    When a (LOCKED, SlamLock, EventNum=N) event is published on the bus
    Then Vehicle.Body.Alarm.State remains "ACTIVATED"
    And the chime continues to escalate to the full alarm at 12 s

  # --- REQ-SLP-009 ---
  Scenario: SlamLock cannot disarm a running alarm (EU slam-lock-protect path)
    Given the perimeter alarm is in the chime phase
    When a (LOCKED, DoorTrimButton, EventNum=N) event is published
    And a (UNLOCKED, SlamLock, EventNum=N+1) event is published immediately after
    Then Vehicle.Body.Alarm.State remains "ACTIVATED"
    And the chime continues to escalate to the full alarm at 12 s

  # --- REQ-SLP-010 ---
  Scenario: Inversion publishes unlock feedback for LockFeedback
    Given vehicle_line.slam_lock_protect is TRUE
    And the Row1.Left door is open
    When the driver presses the Row1.Left trim Lock button
    Then Body.Doors.CentralLock.FeedbackRequest = "unlock" is published
    And LockFeedback flashes the indicators twice
