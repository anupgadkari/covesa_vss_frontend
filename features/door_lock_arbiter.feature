Feature: Door Lock Arbiter
  As the body controller platform
  I must serialize and arbitrate all door lock/unlock/double-lock
  requests from multiple features through a one-deep command queue
  so that the lock motor is never driven with conflicting commands
  and crash-safety rules are enforced.

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-DLK-001: The DoorLock arbiter SHALL accept lock, unlock, and
  #              double-lock requests from any authorized feature:
  #              KeyfobPeps, KeyfobRke, AutoLock, DoorTrimButton,
  #              PhoneApp, PhoneBle, NfcCard, NfcPhone, CrashUnlock,
  #              and any future lock requestor registered in the
  #              allow-list.
  #              KeyfobPeps = passive entry (proximity), KeyfobRke =
  #              manual button press. PhoneApp = cloud/internet remote,
  #              PhoneBle = BLE digital key. NfcCard = physical NFC
  #              card tapped on reader. NfcPhone = NFC key provisioned
  #              on a phone, tapped on reader. Each is tracked as a
  #              separate requestor for diagnostic traceability.
  #
  # REQ-DLK-002: The DoorLock arbiter SHALL maintain a one-deep command
  #              queue: one ACTIVE operation (being executed by the lock
  #              motor) and one PENDING operation (queued). When the
  #              active operation completes, the pending operation is
  #              promoted to active and dispatched.
  #
  # REQ-DLK-003: A lock/unlock motor operation takes approximately
  #              300 ms to complete. The arbiter SHALL NOT dispatch a
  #              new command while an operation is active. It SHALL wait
  #              for the completion acknowledgement before dispatching
  #              the pending request.
  #
  # REQ-DLK-004: The Classic AUTOSAR Locking SWC SHALL acknowledge
  #              completion of each lock operation by publishing an
  #              incrementing event number, the last requestor
  #              (FeatureId), and the updated lock status for all four
  #              doors. The Rust arbiter SHALL listen for this ACK to
  #              know when the active operation has finished.
  #
  # REQ-DLK-005: The Classic AUTOSAR Locking SWC (NOT the Rust arbiter)
  #              SHALL persist the last 5 requestors and lock statuses
  #              in NVM for diagnostic readout. The Rust arbiter has no
  #              NVM responsibility.
  #
  # REQ-DLK-006: When a new request arrives and a pending request
  #              already exists in the queue, the new request SHALL
  #              REPLACE the pending request (latest wins), subject to
  #              the crash-unlock exceptions in REQ-DLK-007/008.
  #
  # REQ-DLK-007: An unlock request originating from CrashUnlock SHALL
  #              NOT be replaced in the pending queue by any subsequent
  #              request. Once a crash unlock is queued (or active), it
  #              runs to completion.
  #
  # REQ-DLK-008: After a CrashUnlock request is accepted (queued or
  #              dispatched), the arbiter SHALL reject ALL new requests
  #              for 10 seconds. This crash lockout window prevents
  #              flying debris or spurious switch signals from
  #              re-locking the vehicle during a collision.
  #
  # REQ-DLK-009: When no operation is active (idle state), a new
  #              request SHALL be dispatched immediately (promoted
  #              directly to active) without queuing.
  #
  # REQ-DLK-010: If the active operation's ACK reports a failure for
  #              one or more doors (motor jam, wiring fault), the
  #              arbiter SHALL still promote the pending request. Error
  #              reporting is the Classic AUTOSAR SWC's responsibility.
  #
  # REQ-DLK-011: The arbiter SHALL support three lock commands:
  #              UNLOCK, LOCK, and DOUBLE_LOCK. Double-lock engages
  #              the deadlock mechanism (superlocking / anti-theft).
  #
  # REQ-DLK-012: The DoorLock arbiter SHALL validate each request
  #              against its static allow-list of (FeatureId, Priority)
  #              tuples, same as other domain arbiters. Unauthorized
  #              requests are rejected.
  # -------------------------------------------------------------------------

  Background:
    Given the vehicle low-voltage system is in state "ON"
    And the DoorLock arbiter is running
    And no lock operation is in progress (arbiter is idle)

  # --- REQ-DLK-009 ---
  Scenario: Idle arbiter dispatches request immediately
    Given the arbiter queue is empty and no operation is active
    When the KeyfobPeps feature requests UNLOCK
    Then the arbiter dispatches the UNLOCK command to the Locking SWC immediately
    And the request becomes the active operation

  # --- REQ-DLK-002, REQ-DLK-003 ---
  Scenario: Request during active operation is queued
    Given the KeyfobPeps feature's UNLOCK is the active operation (motor running)
    When the AutoLock feature requests LOCK
    Then the LOCK request is placed in the pending queue
    And the arbiter does NOT dispatch LOCK until the active UNLOCK completes

  # --- REQ-DLK-002, REQ-DLK-004 ---
  Scenario: Pending request dispatched after ACK
    Given the KeyfobPeps feature's UNLOCK is active
    And the AutoLock feature's LOCK is pending
    When the Locking SWC publishes completion ACK (event N, all doors unlocked)
    Then the arbiter promotes the AutoLock LOCK to active
    And the arbiter dispatches the LOCK command to the Locking SWC

  # --- REQ-DLK-006 ---
  Scenario: Newer request replaces pending request
    Given the KeyfobPeps feature's UNLOCK is active
    And the AutoLock feature's LOCK is pending
    When the KeyfobRke feature requests DOUBLE_LOCK
    Then the KeyfobRke DOUBLE_LOCK replaces AutoLock's LOCK in the pending slot
    And only the KeyfobRke DOUBLE_LOCK will execute after the active operation completes

  # --- REQ-DLK-006 ---
  Scenario: Multiple rapid requests — only last one survives in queue
    Given the KeyfobPeps feature's UNLOCK is active
    When the AutoLock feature requests LOCK
    And then the PhoneApp feature requests UNLOCK
    And then the KeyfobRke feature requests LOCK
    Then only the KeyfobRke LOCK remains in the pending slot
    And the AutoLock and PhoneApp requests were replaced

  # --- REQ-DLK-007 ---
  Scenario: Crash unlock in queue cannot be replaced
    Given the KeyfobPeps feature's LOCK is active
    And the CrashUnlock feature has queued an UNLOCK (pending)
    When the AutoLock feature requests LOCK
    Then the CrashUnlock UNLOCK remains in the pending slot
    And the AutoLock LOCK is rejected

  # --- REQ-DLK-007 ---
  Scenario: Crash unlock as active operation runs to completion
    Given the CrashUnlock feature's UNLOCK is the active operation
    When the KeyfobRke feature requests LOCK
    Then the KeyfobRke LOCK is rejected (crash lockout active)
    And the CrashUnlock UNLOCK continues to completion

  # --- REQ-DLK-008 ---
  Scenario: 10-second lockout after crash unlock
    Given the CrashUnlock feature's UNLOCK has completed
    When the DoorTrimButton feature requests LOCK within 10 seconds
    Then the LOCK request is rejected
    And the arbiter logs "crash lockout active, request rejected"

  # --- REQ-DLK-008 ---
  Scenario: Lockout expires after 10 seconds
    Given the CrashUnlock feature's UNLOCK completed 10 seconds ago
    When the KeyfobRke feature requests LOCK
    Then the LOCK request is accepted and dispatched normally

  # --- REQ-DLK-010 ---
  Scenario: Partial failure — pending still dispatched
    Given the KeyfobPeps feature's UNLOCK is active
    And the AutoLock feature's LOCK is pending
    When the Locking SWC ACK reports Row2.Right failed to unlock (motor jam)
    Then the arbiter still promotes AutoLock's LOCK to active
    And the Locking SWC handles the failure reporting and DTC

  # --- REQ-DLK-011 ---
  Scenario: Double-lock request
    Given all four doors are locked (single lock)
    When the KeyfobRke feature requests DOUBLE_LOCK
    Then the arbiter dispatches the DOUBLE_LOCK command
    And the Locking SWC engages the deadlock mechanism

  # --- REQ-DLK-001 ---
  Scenario: Multiple features can request locks
    Given the arbiter is idle
    When the following features submit requests in sequence:
      | Feature        | Command     |
      | KeyfobPeps     | UNLOCK      |
      | AutoLock       | LOCK        |
      | DoorTrimButton | LOCK        |
      | KeyfobRke      | DOUBLE_LOCK |
      | PhoneApp       | UNLOCK      |
      | CrashUnlock    | UNLOCK      |
    Then each request is processed through the one-deep queue
    And no two commands are active simultaneously

  # --- REQ-DLK-004, REQ-DLK-005 ---
  Scenario: ACK carries event number and requestor
    Given the KeyfobPeps feature's UNLOCK is active
    When the Locking SWC completes the operation
    Then the ACK contains:
      | Field        | Value              |
      | EventNumber  | N (incremented)    |
      | Requestor    | KeyfobPeps         |
      | Row1Left     | unlocked           |
      | Row1Right    | unlocked           |
      | Row2Left     | unlocked           |
      | Row2Right    | unlocked           |
    And the Locking SWC persists this to NVM (last 5 entries)

  # --- REQ-DLK-012 ---
  Scenario: Unauthorized feature rejected
    Given the arbiter is idle
    When the DRL feature attempts to request LOCK (not in allow-list)
    Then the request is rejected
    And no command is dispatched

  # --- REQ-DLK-008 ---
  Scenario: Crash during active lock — unlock queued then lockout starts
    Given the AutoLock feature's LOCK is active (motor running)
    When a crash is detected and CrashUnlock requests UNLOCK
    Then the CrashUnlock UNLOCK is placed in the pending queue
    And after the active LOCK completes, CrashUnlock UNLOCK dispatches
    And the 10-second lockout window starts from CrashUnlock dispatch
