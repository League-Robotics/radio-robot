---
status: complete
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 079 Use Cases

This is a firmware-infrastructure sprint: the "actors" are the bench
operator driving the dev-bench command surface over serial/radio, and the
firmware developer/test author who relies on the tick-model contract while
extending the firmware. No end-user-facing behavior changes (the wire
protocol is frozen) — these use cases capture what the tick-model rewrite
must continue to make true, and newly makes true, at the bench.

## SUC-001: Bench operator drives two motors at full sample cadence

- **Actor**: Bench operator (serial or radio).
- **Preconditions**: Firmware built with `ROBOT_DEV_BUILD=1`; robot on the
  stand (wheels free); two motor ports in use (e.g. the default drive pair,
  1/2).
- **Main Flow**:
  1. Operator commands both wheels (`DEV DT VW ...` or two `DEV M <n> VEL`).
  2. The HAL's brick flip-flop cycles only the two in-use ports.
  3. Operator polls `DEV M <n> STATE` repeatedly and observes `vel=`
     converging and updating at ~80-90 Hz per motor (2 ports in use).
- **Postconditions**: No unconditional busy-wait blocks the control loop;
  comms remain responsive throughout (statements dispatched within ~1 ms of
  arrival, not up to ~32 ms as before).
- **Acceptance Criteria**:
  - [ ] `DEV M <n> STATE` position/velocity update at the cadence table in
        the design sketch (~11-13 ms period, 2 ports in use).
  - [ ] A statement sent while a motor's settle window is open still
        dispatches (and replies) within about one loop pass, not after a
        multi-ms block.

## SUC-002: Bench operator runs the coupled-rig test with only the addressed ports cycling

- **Actor**: Bench operator running the ratio-governor coupled-rig bench
  test (`DEV DT PORTS 3 4`).
- **Preconditions**: Ports 1/2 have never been addressed since boot.
- **Main Flow**:
  1. Operator binds and drives ports 3/4 only.
  2. The HAL activates in-use tracking for ports 3/4 on their first
     addressed command and never samples ports 1/2.
- **Postconditions**: `DBG I2CLOG` (or equivalent bus-traffic inspection)
  shows zero transactions addressed to ports 1/2's motor slots; ports 3/4
  cycle evenly.
- **Acceptance Criteria**:
  - [ ] Idle, never-addressed ports generate zero bus traffic.
  - [ ] A later `DEV M 1 ...` command activates port 1 sampling from that
        point on (sticky, no auto-deactivation of 3/4).

## SUC-003: Bench operator reverses a motor and the 078 armor still holds at the new cadence

- **Actor**: Bench operator.
- **Preconditions**: 078's reversal-latch armor is merged; motor cruising at
  a nonzero velocity.
- **Main Flow**:
  1. Operator issues an opposite-sign `DEV M <n> VEL`.
  2. The write path zero-writes immediately and holds through the 100 ms
     dwell, exactly as 078 specifies, now driven by the flip-flop's collect
     slices instead of a blocking per-tick read.
- **Postconditions**: No direct sign-flip write ever reaches
  `writeRawDuty()`; dwell timing is unaffected by the faster cadence
  (dwell is wall-clock-timed, not tick-counted).
- **Acceptance Criteria**:
  - [ ] A commanded reversal produces a zero-write, then (after ~100 ms) the
        new-direction duty, ramped from zero by the leaf's slew cap — same
        as 078's stand pass, re-confirmed at the new cadence.

## SUC-004: A silent host trips the watchdog and the robot goes safe quickly

- **Actor**: Bench operator (or an unattended test harness that stops
  sending).
- **Preconditions**: The serial-silence watchdog window has elapsed with no
  statement arriving.
- **Main Flow**:
  1. `main.cpp`'s watchdog check fires.
  2. Every motor and the Drivetrain are neutralized via the one audited
     broadcast path; `EVT dev_watchdog` is emitted.
- **Postconditions**: Firing latency is bounded by one bus transaction
  (~0.9 ms) plus up to one port's worth of pass latency — not by a blocked
  multi-port sweep.
- **Acceptance Criteria**:
  - [ ] Watchdog-fire-to-neutral-write latency is measured on the stand and
        is within the accepted ~1 cm of motion bound (design sketch decision
        2), materially better than today's up-to-~32 ms case.

## SUC-005: Bench operator steals authority from the Drivetrain via an independent motor command

- **Actor**: Bench operator running a coupled-rig test (Drivetrain bound to
  ports 3/4; port 1 is an independent, unbound load motor).
- **Preconditions**: Drivetrain is `active()` on ports 3/4.
- **Main Flow**:
  1. Operator issues `DEV M 1 DUTY ...` (an unbound port) — Drivetrain
     authority is untouched.
  2. Operator issues `DEV M 3 VEL ...` (a bound port) — Drivetrain drops to
     `standby()`; port 4 (the other bound wheel) is left exactly as it was,
     not neutralized.
- **Postconditions**: `DEV DT STATE`'s `active=` flag reflects the
  authority drop only when a *bound* port was directly commanded.
- **Acceptance Criteria**:
  - [ ] `isBoundPort()` semantics are unchanged from today's 077-007 fix
        (verified via host tests, not just re-read).
  - [ ] `DEV DT STOP` neutralizes exactly the bound pair (not a global
        broadcast) and drops authority.

## SUC-006: Bench operator rebinds the Drivetrain and gets accurate capability-aware reporting

- **Actor**: Bench operator.
- **Main Flow**:
  1. Operator sends `DEV DT PORTS <left> <right>`.
  2. The binding lands in `DrivetrainConfig`; the Drivetrain's cached
     wheel-capabilities refresh from the newly-bound motors.
  3. Operator polls `DEV DT CAPS`/`DEV DT STATE`.
- **Postconditions**: `onboard_position` and other capability-derived
  fields reflect the *new* pair immediately; `DEV DT STATE`'s `ports=`
  reflects the new binding.
- **Acceptance Criteria**:
  - [ ] Wire reply shape (`OK DEV DT ports=<left>,<right>`) is unchanged.
  - [ ] The binding persists across `DEV STOP` and a watchdog neutral event
        (resets only on reboot) — unchanged from today.

## SUC-007: Firmware developer traces a wire line to its parsed command using consistent vocabulary

- **Actor**: Firmware developer debugging a statement-dispatch issue.
- **Main Flow**:
  1. Developer reads `docs/protocol-v2.md`, `communicator.h`,
     `command_processor.h`, and `.claude/rules/naming-and-style.md`.
  2. Every place that means "one wire line" says "statement"; every place
     that means an internal `msg::*Command`/`<Producer>To<Consumer>Command`
     edge says "command."
- **Postconditions**: No ambiguity between the two meanings anywhere the
  sweep touched.
- **Acceptance Criteria**:
  - [ ] `CommunicatorToCommandProcessorStatement` is the edge name
        end-to-end (type, comments, this sprint's own docs).
  - [ ] Naming rule 4 reads `<Producer>To<Consumer><Payload>` with payload
        ∈ {Command, Statement}.

## SUC-008: Test author verifies HAL scheduling logic without hardware

- **Actor**: Firmware test author.
- **Main Flow**:
  1. Author writes a host test against the `HOST_BUILD` `I2CBus` scripted
     fake.
  2. The test scripts request/collect timing, throttle windows, and
     dwell interaction, and asserts on `NezhaHal`'s in-use tracking and
     flip-flop sequencing — all off-hardware.
- **Postconditions**: The real `NezhaMotor`/`NezhaHal` register/flip-flop/
  throttle/dwell logic runs under test, not a hand-rolled mock of it.
- **Acceptance Criteria**:
  - [ ] `uv run python -m pytest` collects and passes the new HAL host
        tests.
  - [ ] `I2CBus::clear()`'s 7-bit-address convention is explicitly asserted
        (regression guard against the off-by-one-bit trap noted in the
        architecture doc).

## SUC-009: Firmware developer confirms an abandoned collect's readback is cleanly overwritten

- **Actor**: Firmware developer running the stand A/B gate.
- **Preconditions**: Firmware deployed to the robot on the stand.
- **Main Flow**:
  1. Developer runs the lazy-timer A/B: settle-window traffic vs. none,
     comparing latch rate (diagnosed from `TLM`/`DEV STATE` encoder
     constancy, not `EVT`).
  2. Developer inspects `DBG I2CLOG` around an intentionally-abandoned
     collect to confirm the next request's readback is not corrupted.
- **Postconditions**: The stand pass records a verdict either way; a
  positive finding blocks sprint close until resolved.
- **Acceptance Criteria**:
  - [ ] The A/B is run and its result (latch rate with vs. without
        settle-window traffic) is recorded in the ticket.
