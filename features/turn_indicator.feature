Feature: Turn Indicator
  As the body controller platform
  I must activate the correct direction indicator
  when the driver moves the turn signal stalk
  so that the vehicle signals its intended direction of travel.

  # -------------------------------------------------------------------------
  # Requirements
  # -------------------------------------------------------------------------
  # REQ-TURN-001: When Body.Switches.TurnIndicator.Direction transitions to
  #               "LEFT", the feature SHALL request
  #               DirectionIndicator.Left.IsSignaling = TRUE at priority
  #               MEDIUM (2) via the Lighting arbiter, and SHALL release
  #               any existing TurnIndicator claim on
  #               DirectionIndicator.Right.IsSignaling.
  #
  # REQ-TURN-002: When Body.Switches.TurnIndicator.Direction transitions to
  #               "RIGHT", the feature SHALL request
  #               DirectionIndicator.Right.IsSignaling = TRUE at priority
  #               MEDIUM (2) via the Lighting arbiter, and SHALL release
  #               any existing TurnIndicator claim on
  #               DirectionIndicator.Left.IsSignaling.
  #
  # REQ-TURN-003: When Body.Switches.TurnIndicator.Direction transitions to
  #               "OFF", the feature SHALL release any TurnIndicator claims
  #               on both DirectionIndicator.Left.IsSignaling and
  #               DirectionIndicator.Right.IsSignaling. A released claim
  #               lets any lower-priority or concurrent claim from another
  #               feature (e.g. Hazard) resume; if no other claim exists,
  #               the arbiter publishes the default-off value.
  #
  # REQ-TURN-004: The turn indicator feature SHALL subscribe to the physical
  #               stalk input (Body.Switches.TurnIndicator.Direction), NOT
  #               the actuator outputs.
  #
  # REQ-TURN-005: Turn indicator requests SHALL use priority MEDIUM (2).
  #               The Hazard feature (HIGH/3) and Lock Feedback (HIGH/3,
  #               overlay) can temporarily override. Lock Feedback
  #               self-releases after its brief pattern, allowing the
  #               turn signal to resume.
  #
  # REQ-TURN-006: The turn indicator feature SHALL NOT set blink timing.
  #
  # REQ-TURN-007: The turn indicator feature SHALL have no dependency on
  #               any other feature module.
  #
  # REQ-TURN-008: The turn indicator feature SHALL only process stalk
  #               inputs when Vehicle.LowVoltageSystemState is "ON" or
  #               "START". When ignition transitions to any other state
  #               (OFF, LOCK, ACC), the feature SHALL immediately release
  #               any TurnIndicator claims on both
  #               DirectionIndicator.Left.IsSignaling and
  #               DirectionIndicator.Right.IsSignaling.
  #               Rationale: turn signals require ignition ON per vehicle
  #               electrical architecture — the turn signal relay is
  #               powered from the ignition-switched bus.
  #
  # REQ-TURN-009: (Auto lane change / comfort blink / tip-to-signal)
  #               The feature SHALL count complete flash cycles (on+off)
  #               while the stalk is held. When
  #               Body.Switches.TurnIndicator.Direction transitions from
  #               "LEFT" or "RIGHT" to "OFF":
  #               (a) If fewer than `lane_change_flash_count` (default 3)
  #                   flashes have completed, the feature SHALL maintain
  #                   the arbiter claim and continue signaling until the
  #                   total reaches `lane_change_flash_count`.
  #               (b) If `lane_change_flash_count` or more flashes have
  #                   already completed, the feature SHALL release its
  #                   arbiter claim immediately (no additional flashes).
  #               A "flash" is one complete on+off cycle of the physical
  #               lamps, counted by observing the BlinkRelay lamp
  #               feedback signal's falling edge (on→off transition).
  #
  # REQ-TURN-010: When `lane_change_flash_count` is 0, comfort blink
  #               SHALL be disabled and the feature SHALL immediately
  #               release the arbiter claim when the stalk returns to OFF
  #               (REQ-TURN-003 original behavior).
  #
  # REQ-TURN-011: The comfort blink countdown SHALL be immediately
  #               cancelled (arbiter claim released, lamps stop) when:
  #               (a) Vehicle.LowVoltageSystemState transitions away from
  #                   ON/START (REQ-TURN-008 takes precedence), or
  #               (b) Body.Switches.TurnIndicator.Direction transitions
  #                   to the opposite direction (the new direction
  #                   activates immediately and the old side is released).
  #
  # REQ-TURN-012: The comfort blink countdown SHALL NOT be cancelled by
  #               hazard switch engagement. If the hazard feature engages
  #               at priority HIGH during comfort blink, the arbiter
  #               suppresses the MEDIUM comfort claim while hazard is
  #               active. When hazard disengages, any remaining comfort
  #               flashes may resume (arbiter claim still held).
  # -------------------------------------------------------------------------

  Background:
    Given the vehicle low-voltage system is in state "ON"
    And the Lighting domain arbiter is running
    And the Turn Indicator feature is running

  # --- REQ-TURN-001 ---
  Scenario: Turn stalk moved to LEFT
    Given the turn stalk is in position OFF
    When the driver moves the turn stalk to LEFT
    Then Body.Switches.TurnIndicator.Direction becomes "LEFT"
    And the Turn feature requests DirectionIndicator.Left.IsSignaling = TRUE at priority MEDIUM
    And the Turn feature releases its claim on DirectionIndicator.Right.IsSignaling

  # --- REQ-TURN-002 ---
  Scenario: Turn stalk moved to RIGHT
    Given the turn stalk is in position OFF
    When the driver moves the turn stalk to RIGHT
    Then Body.Switches.TurnIndicator.Direction becomes "RIGHT"
    And the Turn feature requests DirectionIndicator.Right.IsSignaling = TRUE at priority MEDIUM
    And the Turn feature releases its claim on DirectionIndicator.Left.IsSignaling

  # --- REQ-TURN-009a ---
  Scenario: Quick tap enters comfort blink to reach configured flash count
    Given the turn stalk is in position LEFT
    And the left direction indicator is signaling
    When the driver returns the turn stalk to OFF
    Then the left direction indicator continues signaling during comfort blink countdown

  # --- REQ-TURN-009a ---
  Scenario: Quick tap completes exactly the configured number of total flashes
    Given the turn stalk is in position LEFT
    And the left direction indicator is signaling
    When the driver returns the turn stalk to OFF
    And 3 complete flash cycles elapse
    Then the Turn feature releases its claim on DirectionIndicator.Left.IsSignaling
    And the Turn feature releases its claim on DirectionIndicator.Right.IsSignaling

  # --- REQ-TURN-009a ---
  Scenario: Comfort blink still active before total flash count reached
    Given the turn stalk is in position LEFT
    And the left direction indicator is signaling
    When the driver returns the turn stalk to OFF
    And 2 complete flash cycles elapse
    Then the left direction indicator continues signaling during comfort blink countdown

  # --- REQ-TURN-009b ---
  Scenario: Long hold releases immediately when stalk returns to OFF
    Given the turn stalk is in position LEFT
    And the left direction indicator is signaling
    When 3 complete flash cycles elapse
    And the driver returns the turn stalk to OFF
    Then the Turn feature releases its claim on DirectionIndicator.Left.IsSignaling
    And the Turn feature releases its claim on DirectionIndicator.Right.IsSignaling

  # --- REQ-TURN-001, REQ-TURN-002, REQ-TURN-011b ---
  Scenario: Turn stalk changes directly from LEFT to RIGHT
    Given the turn stalk is in position LEFT
    When the driver moves the turn stalk to RIGHT
    Then the Turn feature releases its claim on DirectionIndicator.Left.IsSignaling
    And the Turn feature requests DirectionIndicator.Right.IsSignaling = TRUE at priority MEDIUM

  # --- REQ-TURN-011b ---
  Scenario: Comfort blink cancelled by opposite stalk direction
    Given the turn stalk is in position LEFT
    And the left direction indicator is signaling
    When the driver returns the turn stalk to OFF
    And 1 complete flash cycle elapses
    And the driver moves the turn stalk to RIGHT
    Then the Turn feature releases its claim on DirectionIndicator.Left.IsSignaling
    And the Turn feature requests DirectionIndicator.Right.IsSignaling = TRUE at priority MEDIUM

  # --- REQ-TURN-011a ---
  Scenario: Comfort blink cancelled by ignition OFF
    Given the turn stalk is in position LEFT
    And the left direction indicator is signaling
    When the driver returns the turn stalk to OFF
    And 1 complete flash cycle elapses
    And Vehicle.LowVoltageSystemState transitions to "OFF"
    Then the Turn feature releases its claim on DirectionIndicator.Left.IsSignaling
    And the Turn feature releases its claim on DirectionIndicator.Right.IsSignaling

  # --- REQ-TURN-005 ---
  Scenario: Turn signal suppressed while hazard is active
    Given the hazard switch is engaged
    And both indicators are signaling at priority HIGH
    When the driver moves the turn stalk to LEFT
    Then the Turn feature's MEDIUM claim on DirectionIndicator.Left.IsSignaling is recorded by the arbiter
    And the Turn feature releases its claim on DirectionIndicator.Right.IsSignaling
    And both indicators continue signaling because Hazard's HIGH claims win arbitration

  # ===========================================================================
  # Ignition gating (REQ-TURN-008)
  # ===========================================================================

  # --- REQ-TURN-008 ---
  Scenario: Turn stalk ignored when ignition is OFF
    Given the vehicle low-voltage system is in state "OFF"
    When the driver moves the turn stalk to LEFT
    Then the Turn feature does NOT request any indicator change

  # --- REQ-TURN-008 ---
  Scenario: Turn stalk ignored when ignition is ACC
    Given the vehicle low-voltage system is in state "ACC"
    When the driver moves the turn stalk to LEFT
    Then the Turn feature does NOT request any indicator change

  # --- REQ-TURN-008 ---
  Scenario: Active turn signal deactivated when ignition turns OFF
    Given the turn stalk is in position LEFT
    And the left direction indicator is signaling at priority MEDIUM
    When Vehicle.LowVoltageSystemState transitions to "OFF"
    Then the Turn feature releases its claim on DirectionIndicator.Left.IsSignaling
    And the Turn feature releases its claim on DirectionIndicator.Right.IsSignaling

  # --- REQ-TURN-008 ---
  Scenario: Active turn signal deactivated when ignition goes to ACC
    Given the turn stalk is in position RIGHT
    And the right direction indicator is signaling at priority MEDIUM
    When Vehicle.LowVoltageSystemState transitions to "ACC"
    Then the Turn feature releases its claim on DirectionIndicator.Left.IsSignaling
    And the Turn feature releases its claim on DirectionIndicator.Right.IsSignaling

  # --- REQ-TURN-008 ---
  Scenario: Turn signal resumes when ignition returns to ON
    Given the turn stalk is in position LEFT
    And Vehicle.LowVoltageSystemState was "ACC" (turn inactive)
    When Vehicle.LowVoltageSystemState transitions to "ON"
    Then the Turn feature requests DirectionIndicator.Left.IsSignaling = TRUE at priority MEDIUM
    And the Turn feature releases its claim on DirectionIndicator.Right.IsSignaling
