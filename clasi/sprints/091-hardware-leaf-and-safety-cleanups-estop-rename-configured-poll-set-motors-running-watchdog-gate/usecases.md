---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 091 Use Cases

This is a firmware-infrastructure/safety sprint: the "actors" are the bench
operator driving the dev-bench command surface over serial/radio, and the
firmware developer who relies on `MainLoop`'s naming and `NezhaHardware`'s
scheduling contract while maintaining the firmware. The wire protocol gains
exactly one new key (`DEV M <n> CFG polled=`) and one new reply
(`ERR nodev` on an unpolled port); no existing verb or reply shape changes.

## SUC-001: Firmware developer calls the emergency-stop entry point by its
conventional name

Parent: UC-004 (Stop Robot Immediately)

- **Actor**: Firmware developer/maintainer.
- **Preconditions**: `Rt::MainLoop` owns the watchdog-fire/DEV-STOP
  immediate-neutralize bypass, currently named `emergencyNeutralize()`.
- **Main Flow**:
  1. Developer greps for the loop's emergency-stop entry point.
  2. Finds `estop()` — the short, conventional term — with no remaining
     `emergencyNeutralize` identifier anywhere in the tree.
- **Postconditions**: Behavior is byte-for-byte unchanged (same bypass of
  `bb.driveIn`/`bb.motorIn`/`bb.hardwareBroadcastIn`, same call sites, same
  timing); only the name changed.
- **Acceptance Criteria**:
  - [ ] `grep -rn emergencyNeutralize` (source + tests + docs) returns
        nothing.
  - [ ] `estop()` is called from exactly the same two sites
        (`serviceWatchdogs()`'s fire branch; any DEV-STOP-adjacent path
        that called the old name) with unchanged arguments/timing.
  - [ ] `uv run python -m pytest tests/sim` unchanged (309 passed / 2
        xfailed).

## SUC-002: Robot's normal drive pair polls at full, undegraded cadence for
the life of the boot

Parent: UC-001 (Drive Robot at Continuous Speed) — also serves UC-002/003

- **Actor**: Field/bench operator driving the robot's normal two wheels.
- **Preconditions**: Firmware built with `ROBOT_DEV_BUILD=1`; boot config's
  drive pair (`LEFT_PORT`/`RIGHT_PORT`) is polled from construction, not
  earned by a first command.
- **Main Flow**:
  1. Robot boots; the flip-flop sequencer is already cycling the drive
     pair's two ports before any command ever arrives (no "cold start,
     zero traffic until first touch" gap, and no dependency on command
     order).
  2. Operator drives normally for an extended session, at any point
     touching an unrelated bench port (see SUC-003) and then returning to
     normal driving.
  3. The drive pair's sample cadence is unaffected by step 2 — it never
     degrades to a 3- or 4-port round-robin because of a bench excursion
     earlier in the same boot.
- **Postconditions**: The drive pair's poll membership is a constant,
  config-established fact for the whole boot session — it can grow (via
  SUC-003's explicit opt-in) but the drive pair itself is never crowded
  out.
- **Acceptance Criteria**:
  - [ ] `NezhaHardware`'s poll mask contains the boot-config drive pair
        from construction, before any `apply()`/`tick()` call.
  - [ ] Enabling polling on a third port (SUC-003) does not remove the
        drive pair from the schedule or change its rotation order.

## SUC-003: Bench operator explicitly enables poll-schedule membership for
a standalone coupled-rig port

Parent: N/A — cross-cutting bench capability; supports the same
diagnostic/characterization intent as UC-005 (Query Encoder Positions),
just for ports outside the Drivetrain's bound pair

- **Actor**: Bench operator running PID-hold or ratio-governor
  characterization on the coupled bench rig (`tests/bench/
  pid_hold_speed.py`, `tests/bench/ratio_governor_curve.py`;
  `docs/protocol-v2.md` §16).
- **Preconditions**: Two independent motors are physically wired to ports
  the boot config does not poll by default (e.g. 3/4, or 4 while the
  Drivetrain is bound to 2/3).
- **Main Flow**:
  1. Operator sends `DEV M <n> CFG polled=true` for each extra port before
     addressing it.
  2. Operator drives/characterizes those ports (`DEV M <n> VEL`/`DUTY`,
     optionally `DEV DT PORTS <n> <m>`) exactly as today.
  3. Encoder readback and the embedded velocity PID work normally on the
     newly-polled ports, at the same cadence the pre-091 command-derived
     scheme gave them.
- **Postconditions**: The extra port(s) stay polled until explicitly
  disabled (`DEV M <n> CFG polled=false`) or reboot — a visible, deliberate
  config action, never an incidental side effect of an ordinary motor
  command.
- **Acceptance Criteria**:
  - [ ] `DEV M <n> CFG polled=true` on a previously-unpolled port brings it
        into the flip-flop's rotation from the next `tick()`.
  - [ ] `pid_hold_speed.py`/`ratio_governor_curve.py`'s bench-setup
        preambles are updated to send this CFG key for the ports they use
        standalone, and `docs/protocol-v2.md` documents the key.

## SUC-004: Operator addressing an unpolled port gets a clear rejection
instead of a silently non-functional accept

Parent: UC-005 (Query Encoder Positions) — the capability-rejection
pattern this reuses

- **Actor**: Bench operator or firmware developer.
- **Preconditions**: A port is not in the current poll mask (boot default,
  never opted in per SUC-003).
- **Main Flow**:
  1. Operator sends `DEV M <n> VEL 50` (or `DUTY`/`POS`) to an unpolled
     port.
  2. Firmware rejects the command before it ever reaches `bb.motorIn[]`.
- **Postconditions**: `ERR nodev` is returned — the same wire shape and
  device-presence convention `OI`/`OZ`/`OR`/`OV` already use with no
  odometer present — instead of the pre-091 silent accept that would never
  actually close the loop (the port never gets ticked, so `VEL`'s PID never
  runs).
- **Acceptance Criteria**:
  - [ ] `DEV M <n> DUTY|VEL|POS` on an unpolled port returns
        `ERR nodev <mode>` and posts nothing to `bb.motorIn[]`.
  - [ ] `NEUTRAL`/`RESET`/`STATE`/`CAPS`/`CFG` on the same port are
        unaffected (never gated on poll membership).
  - [ ] A sim test (`test_dev_command_outbox.py`'s harness) proves both the
        rejection and that `CFG polled=true` then makes the identical
        command succeed.

## SUC-005: An idle robot ignores comms silence; a driving robot still
safes on comms silence

Parent: UC-004 (Stop Robot Immediately)

- **Actor**: Field operator (host software) and the robot's own safety
  watchdog.
- **Preconditions**: The serial-silence watchdog is armed with its normal
  window (`DEV WD`).
- **Main Flow (idle)**:
  1. Motors are stopped/neutral (no Drivetrain-governed motion, no
     standalone `DEV M` motion verb outstanding).
  2. Host goes silent past the window.
  3. Nothing happens — no neutralize, no `EVT dev_watchdog`.
- **Main Flow (driving)**:
  1. A live drive command (Drivetrain-governed, or a standalone `DEV M`
     motion verb on any port) is active.
  2. Host goes silent past the window.
  3. The SAME pass that detects the expired window neutralizes every motor
     (`estop()`, per SUC-001) and emits `EVT dev_watchdog`, exactly as
     before this sprint.
- **Postconditions**: The watchdog's fire is gated on commanded
  motors-running state (`bb.drivetrain.active || any(bb.motors[i].
  active)`), never on measured encoder motion; the feed/fire-once/
  `DEV WD`-settable-window mechanics are unchanged.
- **Acceptance Criteria**:
  - [ ] Idle + silence past the window → no neutralize, no `EVT
        dev_watchdog` (new sim test).
  - [ ] Driving + silence past the window → same-pass neutralize + `EVT
        dev_watchdog` (existing sim tests keep passing unmodified).
  - [ ] The radio-path HITL bench (this sprint's issue also asked for one)
        is explicitly deferred to a fresh `clasi/issues/` item — not run
        this sprint (relay dongle unplugged) and not treated as a blocking
        acceptance criterion.
