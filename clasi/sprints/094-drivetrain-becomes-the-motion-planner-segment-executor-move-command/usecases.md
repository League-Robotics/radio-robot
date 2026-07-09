---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 094 Use Cases

This sprint gives the Drivetrain a motion planner and the Communicator one
new command to drive it with. The use cases below describe the new
`MOVE`-driven motion surface, the preserved `S`/`STOP` escape hatch (now
with graceful-stop semantics), the new pull-based telemetry read, and the
hardware verification gate the sprint's flash/timing risk requires.
Absolute-heading motion (TURN/GOTO) is out of scope — see
architecture-update.md Step 1/Out of Scope.

## SUC-001: Operator drives one relative motion with MOVE
Parent: UC (bench operation / hardware verification)

- **Actor**: Bench operator (human, or a test harness standing in for one)
  issuing wire commands over serial or the radio relay.
- **Preconditions**: Robot booted, on the stand, `DEVICE:` banner sent.
- **Main Flow**:
  1. Operator sends `MOVE <distance> <direction> <finalHeading>` (optionally
     with `v=`/`a=`/`j=`/`w=`/`wa=`/`wj=` limit overrides).
  2. The command handler builds a `Motion::Segment` and posts it to
     `bb.segmentIn`.
  3. `Drivetrain::tick()` drains the segment into its ring and runs
     `Motion::SegmentExecutor`, which sequences PRE_PIVOT → TRANSLATE →
     TERMINAL_PIVOT phases as the segment's fields require (degenerate
     phases skipped).
  4. Each phase's Ruckig-solved body twist is converted to wheel targets
     via kinematics and the ratio governor, then staged onto the bound
     motors.
  5. On completion, the executor auto-arms a graceful decel-to-zero and the
     Drivetrain goes idle.
- **Postconditions**: The robot has moved the commanded relative distance/
  heading and come to a smooth stop — no terminal reverse-creep.
- **Acceptance Criteria**:
  - [ ] `MOVE <mm> 0 0` drives straight and stops gracefully (no terminal
        pivot).
  - [ ] `MOVE 0 0 <heading>` performs a pure in-place turn (`distance=0`).
  - [ ] `MOVE <mm> 0 <heading>` translates then pivots to the final
        heading at the end.
  - [ ] A `MOVE` posted during slack takes effect on the very next
        mandatory tick (no multi-hop mailbox latency).
  - [ ] Multiple `MOVE`s queued back-to-back are not dropped (FIFO drain,
        not latest-wins).
  - [ ] `j=`/`wj=` overrides produce visibly smoother (S-curve, not
        trapezoid) accel/decel edges than the boot default when set higher.

## SUC-002: Operator overrides with the direct S/STOP escape hatch
Parent: UC (bench operation / hardware verification)

- **Actor**: Bench operator.
- **Preconditions**: Robot idle, or mid-`MOVE`.
- **Main Flow**:
  1. Operator sends `S <left> <right>` — direct wheel velocities, no
     planning.
  2. `Drivetrain::tick()` drains `bb.driveIn` FIRST, ahead of
     `bb.segmentIn`; a WHEELS command clears the segment ring and switches
     to direct-wheel mode immediately.
  3. Operator sends `STOP`.
  4. `Drivetrain` clears the ring (if any) and arms the executor's graceful
     decel-to-zero (not an instant brake) from the current sampled state,
     then idles.
- **Postconditions**: The escape hatch always wins over any in-flight
  segment; `STOP` produces a smooth, bounded-time stop with no reverse
  spin.
- **Acceptance Criteria**:
  - [ ] `S` issued mid-`MOVE` immediately preempts the segment (ring
        cleared, direct wheel targets take effect next tick).
  - [ ] `STOP` issued mid-`MOVE` or mid-`S` decelerates smoothly to rest
        (no terminal reverse-creep — regression check vs. 093's fix).
  - [ ] `STOP`'s wire reply is unchanged (`OK stop`) even though its
        physical effect is now a graceful decel, not an instant brake.

## SUC-003: Host reads measured drivetrain state with TLM
Parent: UC (host auto-detection / telemetry)

- **Actor**: `robot_radio` / TestGUI host software, or a human at a
  terminal.
- **Preconditions**: Robot booted and running (any Drivetrain mode).
- **Main Flow**:
  1. Host sends `TLM`.
  2. The handler reads `bb.drivetrain`/`bb.motors[]` (populated each pass
     from measured `hardware_.state(port)`, never a commanded target) and
     replies synchronously through the command's own reply channel.
- **Postconditions**: Host has a fresh, measured snapshot of encoder
  position/velocity and the executor's active/idle flag, with no
  loop-originated push output involved.
- **Acceptance Criteria**:
  - [ ] `TLM`'s `enc=`/`vel=` values are measured (track real wheel motion),
        not the currently-commanded setpoint.
  - [ ] `TLM`'s active/idle flag reflects whether a segment or direct-wheel
        command is currently driving the wheels.
  - [ ] Issuing `TLM` produces no other wire traffic (no loop-originated
        `EVT`/periodic output is reintroduced).

## SUC-004: Stakeholder/team-lead verifies the flash and timing budget survive re-linking Ruckig
Parent: UC (build/deploy verification)

- **Actor**: Programmer (ticket executor), team-lead (validates the gate).
- **Preconditions**: The Drivetrain's `Motion::SegmentExecutor` is wired
  into the live tick path (Ruckig is back in the linked, in-use firmware
  image, unlike 093's build where it was fully stripped).
- **Main Flow**:
  1. Build firmware before Ruckig is wired live; run
     `arm-none-eabi-size build/MICROBIT`; record `text`/`data`/`bss`.
  2. Build firmware after; run `arm-none-eabi-size build/MICROBIT` again;
     record the same figures.
  3. Confirm the image still fits flash with the same headroom
     characteristics the pre-093 (last Ruckig-in-use) build had.
  4. Independently confirm `Hal::Motor::apply()`/`NezhaMotor::apply()` still
     perform no immediate I2C write (staging-only), so the flip-flop's
     timing is provably unchanged by the Drivetrain's new direct-staging
     write path.
- **Postconditions**: The sprint's top risk (flash budget) is measured, not
  assumed; the I2C flip-flop's load-bearing timing assumption is verified,
  not merely asserted.
- **Acceptance Criteria**:
  - [ ] Before/after `arm-none-eabi-size` figures are recorded in the
        ticket that wires the executor into the live loop.
  - [ ] The image fits flash with margin comparable to the pre-093
        Ruckig-in-use build.
  - [ ] A code-level check (review or a host test) confirms
        `NezhaMotor::setVelocity()`/`setDutyCycle()` remain staging-only —
        no I2C write outside `tick()`'s `COLLECT_DUE`-gated dispatch.

## SUC-005: Bench operator confirms the whole surface on real hardware (HITL)
Parent: UC (operational verification, `.claude/rules/hardware-bench-testing.md`)

- **Actor**: Stakeholder / bench operator, on the stand.
- **Preconditions**: Firmware flashed per SUC-004's gate; robot on the
  stand, wheels off the ground.
- **Main Flow**:
  1. Confirm encoders are alive and increment with commanded direction.
  2. Confirm `S` spins wheels directly; `STOP` decelerates gracefully.
  3. Confirm a straight `MOVE`, a translate-then-terminal-pivot `MOVE`, and
     a pure in-place `MOVE` each execute and drain to a graceful stop with
     no terminal reverse-creep.
  4. Confirm `j=`/`wj=` overrides produce visibly smoother edges.
  5. Confirm `TLM` reports measured `enc=`/`vel=` throughout.
  6. Feed the RX watchdog (`send_fast PING` ~200ms) for the duration of the
     session (per the bench-testing rule's transport guidance).
- **Postconditions**: The sprint's behavior is seen working on real
  hardware, not only in sim — the standing gate every HAL/motor/
  command-surface-touching sprint must pass.
- **Acceptance Criteria**:
  - [ ] Every step above is confirmed on the stand and reported back
        (pass/fail per step, not a single "it worked").
  - [ ] Any failure blocks sprint close — this ticket is HITL and is not
        satisfied by sim tests alone.
