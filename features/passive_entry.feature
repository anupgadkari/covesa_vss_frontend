Feature: Passive Entry (PEPS unlock-on-handle-pull)
  As the body controller platform
  I must unlock the vehicle when the driver pulls an outside door handle
  while a paired PEPS device is in the matching proximity zone
  so that the user does not need to press a key fob button to enter.

  # -------------------------------------------------------------------------
  # Implementation model
  # -------------------------------------------------------------------------
  # The PassiveEntry feature subscribes to four outside-handle-pull
  # signals (Body.Doors.Row*.*.Handle.Outside.IsPulled) and to all paired-
  # device zone signals.  On a FALSE→TRUE handle-pull edge:
  #   1. Identify which paired devices are currently in the door's
  #      proximity zone (LeftFront for Row1.Left; RightFront for the
  #      other three).
  #   2. Generate a 16-byte nonce and publish it to BOTH
  #      Body.PEPS.LfChallenge (fobs respond) and
  #      Body.PEPS.BleChallenge (phones respond).
  #   3. Wait up to 150 ms for any candidate to publish a verifiable
  #      AES-128 response.  First match wins.
  #   4. Dispatch UnlockDriver (stage 1) or UnlockAll (stage 2) via the
  #      DoorLockArbiter, plus FeedbackRequest = "unlock" for the lock-
  #      feedback flash pattern.
  # -------------------------------------------------------------------------

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-PE-001: When Body.Doors.Row1.Left.Handle.Outside.IsPulled
  #             transitions FALSE → TRUE and at least one paired PEPS
  #             device is in the LeftFront zone, the feature SHALL
  #             dispatch a door-lock command (UnlockDriver in two-stage
  #             mode, UnlockAll otherwise) via the DoorLockArbiter.
  #
  # REQ-PE-002: The feature SHALL publish FeedbackRequest = "unlock" on
  #             every successful authenticated unlock so LockFeedback
  #             plays its 2-flash unlock pattern.
  #
  # REQ-PE-003: When a handle is pulled but no paired device is in the
  #             matching proximity zone, the feature SHALL NOT dispatch
  #             any lock command.
  #
  # REQ-PE-004: Approach zone (RSSI-only) does NOT satisfy passive entry
  #             — the device must be in a challenge-response-capable
  #             proximity zone (LeftFront, RightFront, etc.).
  #             Otherwise an attacker amplifying RSSI from far range
  #             could spoof presence without producing a valid AES
  #             response.
  #
  # REQ-PE-005: Two-stage unlock honours dealer.two_stage_unlock (same
  #             flag as RKE).  When enabled (default), the first
  #             successful pull on a door dispatches UnlockDriver; a
  #             second pull within TWO_STAGE_WINDOW_SECS (3 s) on any
  #             door from the same paired device dispatches UnlockAll.
  #
  # REQ-PE-006: Wrong-key devices in the proximity zone (unpaired fobs,
  #             devices the bridge wasn't provisioned with) SHALL be
  #             ignored — they are not on the paired-devices list and
  #             never receive a challenge.  No false-positive unlocks.
  #
  # REQ-PE-007: BLE phones in proximity zones SHALL be authenticated via
  #             the BLE challenge channel (Body.PEPS.BleChallenge); the
  #             feature publishes both LF and BLE challenges in
  #             parallel since it doesn't know which device type is
  #             present.
  #
  # REQ-PE-008: Authentication time-out SHALL be 150 ms.  No verifiable
  #             response within the window means the handle pull is
  #             ignored (no door state change).
  # -------------------------------------------------------------------------

  Background:
    Given the door-lock arbiter and door-lock plant model are running
    And the PEPS plant model is running with instant response stagger
    And the PassiveEntry feature is running

  # --- REQ-PE-001, REQ-PE-002 ---
  Scenario: Handle pull with paired fob in LeftFront zone unlocks driver door
    Given paired fob 1 is in the LeftFront zone
    When the driver pulls the Row1.Left outside handle
    Then PassiveEntry dispatches UnlockDriver via the DoorLockArbiter
    And Body.Doors.CentralLock.FeedbackRequest is "unlock"

  # --- REQ-PE-003 ---
  Scenario: Handle pull with no paired device in any zone is a no-op
    Given no paired devices are positioned in any zone
    When the driver pulls the Row1.Left outside handle
    Then PassiveEntry does NOT dispatch any lock command

  # --- REQ-PE-004 ---
  Scenario: Handle pull with fob in Approach (RSSI-only) does not unlock
    Given paired fob 1 is in the Approach zone
    When the driver pulls the Row1.Left outside handle
    Then PassiveEntry does NOT dispatch any lock command

  # --- REQ-PE-005: stage-2 escalation via the OTHER side handle ---
  # Cabin-state gate (Cabin.LockStatus = DRIVER_UNLOCKED after stage-1)
  # silently skips a repeat pull on the driver handle — the user
  # already has access through that door.  To unlock all, pull a
  # passenger or rear handle (both bypass two-stage and dispatch
  # UnlockAll directly).
  Scenario: Two-stage unlock — passenger pull after stage-1 unlocks all
    Given paired fob 1 is in the LeftFront zone
    And dealer.two_stage_unlock is enabled
    When the driver pulls the Row1.Left outside handle
    Then PassiveEntry dispatches UnlockDriver
    Given paired fob 1 is in the RightFront zone
    When the passenger pulls the Row1.Right outside handle
    Then PassiveEntry dispatches UnlockAll

  # --- REQ-PE-006 ---
  Scenario: Unpaired fob in proximity zone is ignored
    Given unpaired fob 5 is in the LeftFront zone
    When the driver pulls the Row1.Left outside handle
    Then PassiveEntry does NOT dispatch any lock command

  # --- REQ-PE-007 ---
  Scenario: Paired BLE phone in LeftFront zone unlocks driver door
    Given paired phone 1 is in the LeftFront zone
    When the driver pulls the Row1.Left outside handle
    Then PassiveEntry dispatches UnlockDriver via the DoorLockArbiter

  # --- REQ-PE-008: Two-stage disabled — single pull unlocks all doors.
  #               When the dealer cal turns off two-stage unlock, every
  #               successful PEPS pull dispatches UnlockAll directly.
  Scenario: Two-stage disabled — single pull unlocks all doors
    Given paired fob 1 is in the LeftFront zone
    And dealer.two_stage_unlock is disabled
    When the driver pulls the Row1.Left outside handle
    Then PassiveEntry dispatches UnlockAll
    And Body.Doors.CentralLock.FeedbackRequest is "unlock"

  # --- REQ-PE-009: Passenger-side handle pull bypasses two-stage.  When
  #               the user is approaching from the passenger side and
  #               touching the passenger handle, leaving doors locked
  #               is hostile UX — go straight to UnlockAll regardless
  #               of dealer cal.
  Scenario: Passenger-side handle pull always unlocks all (bypasses two-stage)
    Given paired fob 1 is in the RightFront zone
    And dealer.two_stage_unlock is enabled
    When the passenger pulls the Row1.Right outside handle
    Then PassiveEntry dispatches UnlockAll

  # --- REQ-PE-010 / REQ-PE-011: RHD support.  On RHD vehicles
  #               (`dealer.driver_door_side = Right`), Row1.Right is
  #               the driver door and Row1.Left is the passenger door.
  #               The two-stage and passenger-side-bypass rules apply
  #               relative to the driver-door cal, not to physical
  #               position.
  #
  # REQ-PE-010: RHD stage-2 escalation via the passenger (LHS on RHD).
  # Mirror of REQ-PE-005 for RHD.
  Scenario: RHD two-stage unlock — passenger pull after stage-1 unlocks all
    Given the vehicle is RHD
    And dealer.two_stage_unlock is enabled
    And paired fob 1 is in the RightFront zone
    When the driver pulls the Row1.Right outside handle
    Then PassiveEntry dispatches UnlockDriver
    Given paired fob 1 is in the LeftFront zone
    When the passenger pulls the Row1.Left outside handle
    Then PassiveEntry dispatches UnlockAll

  # REQ-PE-011: RHD passenger-side bypass — pulling Row1.Left (the
  #             passenger door on RHD) unlocks all doors directly.
  Scenario: RHD passenger-side handle pull always unlocks all (bypasses two-stage)
    Given the vehicle is RHD
    And dealer.two_stage_unlock is enabled
    And paired fob 1 is in the LeftFront zone
    When the passenger pulls the Row1.Left outside handle
    Then PassiveEntry dispatches UnlockAll
