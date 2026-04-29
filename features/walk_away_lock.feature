Feature: Walk-Away Lock
  As the body controller platform
  I must lock the vehicle automatically when the driver and all passengers
  walk away with their PEPS devices
  so that the vehicle is secured without requiring an explicit key-fob press.

  # -------------------------------------------------------------------------
  # Implementation model
  # -------------------------------------------------------------------------
  # Monitors zone signals for 4 keyfobs and 2 BLE phones.
  # Zone hierarchy (closest → furthest):
  #   Cabin / TrunkInside / Trunk / Hood / LeftFront / RightFront
  #   → Approach → RfRange → OutOfRange
  #
  # "In approach" means zone ∈ {Approach, LeftFront, RightFront,
  #   Hood, Trunk, TrunkInside, Cabin}.
  # "Outside approach" means zone ∈ {RfRange, OutOfRange}.
  #
  # Armed state: a device becomes armed when it enters the approach zone.
  # The feature fires when every armed device has subsequently left.
  # After firing, all armed states are cleared; the cycle restarts on
  # the next approach entry.
  #
  # Walk-Away Lock does NOT apply to NFC cards (touch-to-unlock at handle;
  # users are physically at the vehicle when NFC reads).
  # -------------------------------------------------------------------------

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-WAL-001: When at least one PEPS device (keyfob or BLE phone) has
  #              been detected in the approach zone and all such armed devices
  #              subsequently leave the approach zone, the feature SHALL issue
  #              a LockAll command via the DoorLockArbiter.
  #
  # REQ-WAL-002: The feature SHALL publish FeedbackRequest = "lock" alongside
  #              the LockAll command so that LockFeedback plays a lock pattern.
  #
  # REQ-WAL-003: Devices that have never entered the approach zone SHALL NOT
  #              prevent the lock from firing when armed devices depart.
  #
  # REQ-WAL-004: If at least one armed device is still inside the approach zone
  #              when another armed device departs, the lock SHALL NOT fire.
  #
  # REQ-WAL-005: After the lock fires, all armed states SHALL be reset.
  #              The feature re-arms on the next approach entry.
  #
  # REQ-WAL-006: A device transitioning directly from its initial default state
  #              to an outside-approach zone without ever entering approach
  #              SHALL NOT trigger a lock.
  #
  # REQ-WAL-007: Walk-Away Lock SHALL track keyfobs 1–4 and BLE phones 1–2.
  #              NFC cards are excluded.
  # -------------------------------------------------------------------------

  Background:
    Given the DoorLock domain arbiter is running
    And the Walk-Away Lock feature is running
    And all doors are unlocked
    And no PEPS device is in the approach zone

  # --- REQ-WAL-001, REQ-WAL-002 ---
  Scenario: Single fob enters and leaves approach zone — vehicle locks
    When keyfob 1 zone changes to "Approach"
    And keyfob 1 zone changes to "OutOfRange"
    Then a LockAll command is issued via the DoorLockArbiter
    And FeedbackRequest "lock" is published

  # --- REQ-WAL-001 ---
  Scenario: Fob passes through LeftFront zone then leaves
    When keyfob 1 zone changes to "LeftFront"
    And keyfob 1 zone changes to "RfRange"
    Then a LockAll command is issued
    And FeedbackRequest "lock" is published

  # --- REQ-WAL-004 ---
  Scenario: Two fobs in approach — only first leaves, no lock
    Given keyfob 1 zone changes to "Approach"
    And keyfob 2 zone changes to "Approach"
    When keyfob 1 zone changes to "OutOfRange"
    Then no LockAll command is issued
    And no FeedbackRequest is published

  # --- REQ-WAL-001 ---
  Scenario: Two fobs — both leave approach, lock fires
    Given keyfob 1 zone changes to "Approach"
    And keyfob 2 zone changes to "Approach"
    When keyfob 1 zone changes to "OutOfRange"
    And keyfob 2 zone changes to "RfRange"
    Then a LockAll command is issued
    And FeedbackRequest "lock" is published

  # --- REQ-WAL-003 ---
  Scenario: Unarmed devices do not block lock when armed device departs
    Given only keyfob 1 has entered the approach zone
    And keyfobs 2–4 and phones 1–2 have never entered approach
    When keyfob 1 zone changes to "RfRange"
    Then a LockAll command is issued
    And FeedbackRequest "lock" is published

  # --- REQ-WAL-006 ---
  Scenario: Device never entered approach — no lock on initial outside-zone update
    When keyfob 1 zone changes directly to "OutOfRange" without prior approach entry
    Then no LockAll command is issued
    And no FeedbackRequest is published

  # --- REQ-WAL-005 ---
  Scenario: Armed state resets after lock fires — second cycle works independently
    Given keyfob 1 entered and left approach (lock fired, armed states cleared)
    When keyfob 1 enters approach again
    And keyfob 1 leaves approach
    Then a second LockAll command is issued

  # --- REQ-WAL-001, REQ-WAL-007 ---
  Scenario: BLE phone alone triggers walk-away lock
    When BLE phone 1 zone changes to "Approach"
    And BLE phone 1 zone changes to "OutOfRange"
    Then a LockAll command is issued
    And FeedbackRequest "lock" is published

  # --- REQ-WAL-007 ---
  Scenario: Mixed fob and phone — all must depart before lock fires
    Given keyfob 1 zone changes to "Approach"
    And BLE phone 1 zone changes to "Approach"
    When keyfob 1 zone changes to "OutOfRange"
    Then no LockAll command is issued
    When BLE phone 1 zone changes to "RfRange"
    Then a LockAll command is issued
