---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 064 Use Cases

This sprint hardens the encoder pipeline against five failure modes found by
the 2026-07-02 stand session and CR-02/CR-03 of the 2026-07-01 code review.
None of these are new product-facing features — they are reliability
extensions of existing use cases in `docs/usecases.md`. Each SUC below names
its closest parent UC and states the guarantee this sprint adds.

---

## SUC-001: Operator Queries Diagnostic/Config State Without Side Effects
Parent: UC-005 (Query Encoder Positions) — extends the query-without-mutation
principle to `DBG IRQGUARD`, `RF`, `OL`, `OA`.

- **Actor**: Bench operator or test harness, via USB serial.
- **Preconditions**: Firmware running, command channel connected.
- **Main Flow**:
  1. Operator sends the bare (no-argument) form of a stateful command
     (`DBG IRQGUARD`, `RF`, `OL`, `OA`).
  2. The command's ArgSchema parse result distinguishes "no token supplied"
     from "token supplied."
  3. The handler reports the current value without changing it.
- **Postconditions**: State (IRQ guard, radio channel, OTOS linear/angular
  scalar) is unchanged. The reply reflects the pre-query value.
- **Acceptance Criteria**:
  - [ ] Bare `DBG IRQGUARD` does not change `irqGuard()`.
  - [ ] Bare `RF` reports the current channel and does not persist or retune.
  - [ ] Bare `OL` / `OA` report the current scalar and do not zero it.
  - [ ] A regression test proves query-without-mutation for the shared
        parsing mechanism (`parseSchema`) and, where reachable in
        HOST_BUILD, for the affected handlers.

---

## SUC-002: Robot Survives Full-Speed Reversal Commands Without Encoder Wedge
Parent: UC-001 (Drive Robot at Continuous Speed) — adds a reliability
guarantee against the reversal-transient wedge trigger.

- **Actor**: Python host / firmware control loop.
- **Preconditions**: Robot driving at or near full speed.
- **Main Flow**:
  1. Host (or the velocity PID responding to a streamed target flip) commands
     a full-speed reversal.
  2. `Motor::setSpeed` bounds the change in written PWM per call instead of
     writing the full swing in one I2C transaction.
  3. The written PWM converges to the requested value over a small number of
     control ticks.
- **Postconditions**: Encoder readback keeps tracking wheel motion (no
  persistent frozen reading induced by the reversal write itself); normal
  BVC-profiled motion (D/G/RT) is unaffected byte-for-byte.
- **Acceptance Criteria**:
  - [ ] A single `setSpeed()` call cannot move the written PWM by more than
        the slew cap, except `pct == 0` (stop, which stays immediate).
  - [ ] The cap exceeds the largest per-tick |ΔPWM| produced by a
        BVC-profiled ramp, so normal motion never engages the clamp.

---

## SUC-003: Robot Survives Mid-Motion Command Preemption Without Encoder Wedge
Parent: UC-003 (Drive Robot a Specific Distance) — adds a reliability
guarantee for D-command preemption.

- **Actor**: Python host issuing a new `D` while a prior `D`/`G`/`RT` is still
  moving the wheels.
- **Preconditions**: Wheels rotating under an active command.
- **Main Flow**:
  1. Host sends a new `D` before the previous command finishes.
  2. Firmware determines the drivetrain is not at rest.
  3. The encoder baseline is rebaselined in software from the last
     tick-cached reading; the hardware atomic-read burst is skipped.
  4. The hardware atomic re-prime happens later, the next time the
     drivetrain is genuinely at rest.
- **Postconditions**: The encoder readback is not corrupted by the reset; the
  new `D` command's distance tracking starts from a valid, non-jumped
  baseline.
- **Acceptance Criteria**:
  - [ ] `resetEncoders()` while moving does not invoke the hardware atomic
        reset path (verified via a reset-kind counter in sim).
  - [ ] `resetEncoders()` while at rest still performs the hardware atomic
        re-prime, unchanged from today.
  - [ ] The encoder baseline after a mid-motion reset does not jump by more
        than the pre-reset cached reading's own noise floor.

---

## SUC-004: Host Diagnoses Encoder Wedge State From Telemetry
Parent: UC-006 (Query and Zero Dead-Reckoning Odometry) — extends the
telemetry surface with a wedge-visibility guarantee.

- **Actor**: Python host / Test GUI consuming TLM.
- **Preconditions**: STREAM or SNAP telemetry active.
- **Main Flow**:
  1. The wedge detector observes N consecutive identical raw encoder
     readings for a wheel, independent of command-boundary target==0 resets
     or the prior arming-grace gate.
  2. The per-wheel latch state is included in every TLM frame (`wedge=`).
  3. Host correlates `wedge=` with `mode=` to distinguish a genuine in-motion
     wedge from benign idle quiescence.
- **Postconditions**: TLM continuously reflects the true wedge-latch state;
  the existing `EVT enc_wedged` line is preserved unchanged.
- **Acceptance Criteria**:
  - [ ] A wheel frozen from the start of a new command (never "moves" this
        episode) is detected within `kWedgeThreshold` ticks.
  - [ ] A wheel frozen through a target==0 command boundary keeps
        accumulating its streak instead of resetting at the boundary.
  - [ ] `wedge=<L>,<R>` appears in every TLM/SNAP frame.

---

## SUC-005: Robot Recovers Automatically From a Transient Wedge When Idle
Parent: UC-004 (Stop Robot Immediately) — extends idle-state recovery.

- **Actor**: Firmware control loop (autonomous — no host action required).
- **Preconditions**: A wheel's wedge latch is set; the drivetrain is at rest.
- **Main Flow**:
  1. `Drive::tickUpdate` observes a latched wedge while the drivetrain is at
     rest.
  2. Firmware attempts one hardware atomic re-prime for the affected
     wheel(s).
  3. If the transient latch clears, the wedge state re-arms to healthy.
- **Postconditions**: A transient (non-persistent) latch self-heals without
  operator intervention. A genuinely persistent latch (per
  `docs/knowledge/2026-07-01-encoder-wedge-boundary-latch-flavor.md`) stays
  latched — only a full power cycle clears it — and is not repeatedly
  hammered with re-prime attempts.
- **Acceptance Criteria**:
  - [ ] Auto re-prime is attempted at most once per latch episode.
  - [ ] Auto re-prime never fires while the drivetrain is not at rest.

---

## SUC-006: Robot Tolerates Transient I2C Read Failures Without Pose Corruption
Parent: UC-005 (Query Encoder Positions) — extends with fault tolerance
(CR-03).

- **Actor**: Firmware control loop.
- **Preconditions**: An I2C transaction to the encoder register fails.
- **Main Flow**:
  1. An encoder read path (`collectEncoder`, `readEncoderAtomic`,
     `readEncoderMmFSettle`, `requestEncoder`) detects a non-OK I2C return
     code.
  2. The path holds the last known-good value instead of fabricating a
     position from a zeroed response buffer.
  3. `resetEncoder()`'s median-of-3 + readback-verify sequence never commits
     an offset derived purely from failed reads.
- **Postconditions**: No spurious encoder jump reaches the outlier filter,
  the EKF, or distance-stop logic as a result of a transient I2C failure.
- **Acceptance Criteria**:
  - [ ] A sim-injected read failure for N ticks does not move the fused pose
        beyond the tolerance used by existing pose-stability tests.
  - [ ] `resetEncoder()`'s committed offset is never derived solely from a
        failed read.

---

## SUC-007: Robot Recovers Odometry Tracking After Being Hand-Repositioned While Idle
Parent: UC-006 (Query and Zero Dead-Reckoning Odometry) — extends with
outlier-filter recovery (CR-02).

- **Actor**: Bench operator (rolls/lifts the idle robot by hand) and the
  Python host issuing the next motion command.
- **Preconditions**: Robot idle (no active drive command); operator changes
  wheel position by hand.
- **Main Flow**:
  1. While idle, `Drive::_runOutlierFilter` refreshes its baseline from the
     live encoder reading every tick (no rejection gate at rest), so the
     baseline tracks the hand-rolled position continuously.
  2. If a large divergence instead happens *during* an active command, three
     consecutive rejected samples trigger a rebaseline to the fresh reading
     rather than freezing forever.
  3. Operator issues a new motion command (e.g., `TURN`).
- **Postconditions**: Odometry tracks the new command correctly; the outlier
  filter never freezes permanently.
- **Acceptance Criteria**:
  - [ ] Sim test: jump the plant encoders while idle, then command a `TURN`
        — odometry tracks the turn.
  - [ ] Sim test: force 3+ consecutive large deltas during an active command
        — the filter rebaselines instead of freezing forever.
