---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 077 Use Cases

None of docs/usecases.md's UC-001..UC-019 cover a bench/firmware-developer
debug surface — they describe production motion (S/T/D/G/VW), sensor, and
discovery use cases reached through the full Planner/Superstructure stack,
which this sprint does not touch (that stack lives entirely in `source_old/`
after the park). The use cases below describe a new capability layer: a
minimal dev-loop firmware that lets a bench operator drive individual motors
and a differential Drivetrain directly, with no planner in the way. They have
no existing parent UC; if this dev surface persists past the greenfield
rebuild, promoting a UC-020 "Bench Debug Individual Motors and Drivetrain via
DEV Commands" entry to docs/usecases.md is a reasonable follow-up, but that
promotion is out of scope for this sprint.

## SUC-001: Stand up a fresh, buildable firmware tree with the old tree parked
Parent: None (infrastructure prerequisite for all other SUCs)

- **Actor**: Firmware developer
- **Preconditions**: `source/` and `tests/` contain the live (pre-rebuild)
  firmware and test tree; `codal.json` points `application` at `source`.
- **Main Flow**:
  1. Developer renames `source/` to `source_old/` and `tests/` to `tests_old/`
     (pure `git mv`, history preserved).
  2. Developer creates a new, minimal `source/` (stub `main.cpp`) and a new
     `tests/` skeleton (three domains: `sim/`, `bench/`, `playfield/`, plus
     `unit/`, `tools/`).
  3. `codal.json` keeps `"application": "source"` — it now points at the new
     tree by construction.
  4. `build.py` is adjusted so `gen_default_config.py` / `check_config_sync.py`
     are skipped or no-op cleanly while `source/robot/` does not exist yet,
     but `gen_messages.py` still runs and still targets `source/messages/`.
  5. Developer runs `python build.py --clean`; it produces a hex from the new
     tree.
- **Postconditions**: `source_old/`/`tests_old/` are untouched and independently
  buildable by flipping `codal.json`'s `application` field back. The new tree
  builds from ticket 1 onward — every later ticket in this sprint keeps this
  true.
- **Acceptance Criteria**:
  - [ ] `git log --follow source_old/hal/real/Motor.cpp` (or equivalent) shows
        the pre-rename history intact.
  - [ ] `python build.py --clean` succeeds against the new `source/` at the
        end of every ticket in this sprint.
  - [ ] Setting `codal.json` `application` to `source_old` and rebuilding
        still succeeds (rollback path proven at least once).
  - [ ] `uv run python -m pytest` collects only the new `tests/` tree; nothing
        under `tests_old/` or `source_old/` is collected.

## SUC-002: Generate accurate wire messages for motor and drivetrain control
Parent: None (data-contract prerequisite for SUC-003/SUC-004)

- **Actor**: Firmware developer
- **Preconditions**: `protos/motor.proto` under-specifies `MotorConfig`
  (missing port identity, PID gains, slew/min-duty) and
  `MotorCapabilities` (missing per-mode booleans); `source/messages/` does not
  yet exist in the new tree.
- **Main Flow**:
  1. Developer rewrites `protos/motor.proto`: `MotorCommand` gains
     `reset_position`; `MotorConfig` gains `port`, `vel_gains`, `min_duty`,
     `slew_rate`; `MotorCapabilities` gains one bool per control mode
     (`duty_cycle`, `voltage`, `velocity`, `position`, `has_encoder`).
  2. Developer checks `drivetrain.proto`/`gripper.proto`/`ports.proto`/
     `sensors.proto` against `source_old` reality, annotates the
     now-deprecated `DrivetrainConfig.vel_gains`/`min_wheel` fields, and keeps
     `sync_gain` as the ratio governor's knob.
  3. `scripts/gen_messages.py` regenerates `source/messages/*.h` from the
     updated protos; `--emit-inventory` refreshes the traceability doc.
- **Postconditions**: `source/messages/motor.h` exposes the fields ticket 3's
  `NezhaMotor` needs (port, per-mode capability booleans, PID/slew config)
  with no further proto changes required for the rest of the sprint.
- **Acceptance Criteria**:
  - [ ] `python scripts/gen_messages.py` runs clean against the updated protos
        and regenerates `source/messages/motor.h` (and siblings) with no
        manual post-edits.
  - [ ] `msg::MotorConfig` carries `port`, `vel_gains`, `min_duty`,
        `slew_rate`, `travel_calib`, `fwd_sign`.
  - [ ] `msg::MotorCapabilities` carries one bool per control mode
        (`duty_cycle`, `voltage`, `velocity`, `position`) plus `has_encoder`.
  - [ ] `docs/design/message-inventory.md` reflects the updated motor
        message set after `--emit-inventory`.

## SUC-003: Address and command any Nezha motor by port, capability-gated
Parent: None (new bench/dev capability layer)

- **Actor**: Bench operator (via `DEV M` commands over serial/relay)
- **Preconditions**: `Hal::Motor` faceplate header exists (from SUC-002's
  message types); the robot's Nezha V2 board is wired with up to four motor
  channels.
- **Main Flow**:
  1. Operator sends `DEV M <port> DUTY <duty>`; the motor spins at that PWM
     fraction and `DEV M <port> STATE` reports a matching `applied`.
  2. Operator sends `DEV M <port> VEL <velocity>`; the motor's embedded PID
     (running inside `NezhaMotor::tick()`) closes the loop and STATE's
     `velocity` converges toward the target.
  3. Operator sends `DEV M <port> VOLT <voltage>`; because Nezha's
     `capabilities().voltage` is false, `apply()` rejects it with
     `ERR unsupported` before any register write occurs.
  4. Operator sends `DEV M <port> RESET`; the motor's encoder zeroes via
     `reset_position`, using the exact split-phase 0x46 request/collect
     sequencing carried over from `source_old/hal/real/Motor.cpp`.
- **Postconditions**: Every one of the four Nezha ports can be commanded and
  observed independently; the wedge-latch-sensitive encoder sequencing is
  byte-for-byte unchanged from `source_old`.
- **Acceptance Criteria**:
  - [ ] `NezhaMotor` is instantiable on any port 1..4 (not a hardcoded L/R
        pair); the dev build brings up a `NezhaMotor` on all four ports.
  - [ ] `apply()` rejects any command mode not present in `capabilities()`
        (proven live by `DEV M <port> VOLT` returning `ERR unsupported`).
  - [ ] `NezhaMotor::requestEncoder()`/`collectEncoder()` (or the ticket's
        equivalently named split-phase pair) issue the identical 0x46
        write-then-read byte sequence, in the identical two-phase-per-tick
        arrangement, as `source_old/hal/real/Motor.cpp`.
  - [ ] Encoder plumbing and raw register verbs (0x46/0x47/0x60/0x5D/etc.) are
        private to `NezhaMotor`; nothing above the faceplate touches them.
  - [ ] Bench: `DEV M <port> DUTY 30` spins the wheel and `STATE.position`
        climbs; `DEV M <port> VEL 120` converges with a plausible step
        response.

## SUC-004: Hold a commanded wheel-speed ratio across two motors under load
Parent: None (new bench/dev capability layer)

- **Actor**: Bench operator (via `DEV DT` commands)
- **Preconditions**: Two `NezhaMotor` instances exist on bound ports (default
  the robot's drive pair; the coupled bench rig uses ports 3 and 4, which are
  mechanically linked so driving one loads the other).
- **Main Flow**:
  1. Operator sends `DEV DT VW <v_x> 0 0`; `Drivetrain::tick()` converts the
     twist to left/right wheel velocity targets via `BodyKinematics` and both
     motors run at (approximately) equal speed.
  2. Operator hand-drags one wheel (or the coupled rig's linked motor bogs it
     down); the ratio governor lowers the shared speed ceiling so both
     wheels slow together, holding the commanded ratio instead of letting the
     unloaded wheel run away.
  3. Operator disables the governor (`sync_gain` = 0) and repeats; the
     measured wheel-speed ratio is now allowed to drift, demonstrating the
     governor's effect by its absence.
- **Postconditions**: `Drivetrain` never runs a PID itself — it only computes
  kinematics and adjusts velocity-target ceilings; each `Motor`'s embedded PID
  still does all closed-loop work.
- **Acceptance Criteria**:
  - [ ] `Drivetrain::setTwist`/`setWheelTargets`/`setNeutral` are the only
        primitive setters; `apply()` unpacks `DrivetrainCommand` onto them.
  - [ ] `Drivetrain::tick()` returns a `DrivetrainToMotorCommand` (never
        writes to a motor directly) containing one `MotorCommand` per wheel.
  - [ ] The ratio governor operates on velocity **targets**, never duty
        cycle, and is observable via `DEV DT STATE`.
  - [ ] Bench (coupled rig, ports 3+4): commanding an unequal left/right
        curve and loading the faster wheel causes the governor to lower BOTH
        targets so the measured ratio holds within tolerance; with the
        governor disabled, the ratio visibly drifts.

## SUC-005: Drive the bench debug surface over the standard wire protocol
Parent: None (new bench/dev capability layer)

- **Actor**: Bench operator / host tooling (`NezhaProtocol.send()` over
  serial or the relay's `!GO` data plane)
- **Preconditions**: The new `source/` tree builds and flashes; no
  planner/production motion command family exists in this firmware — DEV is
  the only command family beyond bare liveness.
- **Main Flow**:
  1. Host tooling boots the connection and calls `PING`/`VER`/`HELP` (free
     from the copied command infrastructure) to confirm the device is alive
     and identify itself, exactly as it does against any other v2-protocol
     firmware.
  2. Operator issues `DEV M …` and `DEV DT …` commands; each produces the
     standard `OK <verb> <body> [#id]` / `ERR <code> <detail> [#id]` replies
     so existing host-side reply parsing works unchanged.
  3. `DEV STATE` and `DEV STOP` give a one-shot whole-firmware snapshot and
     emergency neutral, respectively.
- **Postconditions**: The dev-loop firmware is indistinguishable, at the wire
  level, from any other v2 firmware for liveness and reply-taxonomy purposes;
  only the command vocabulary is smaller.
- **Acceptance Criteria**:
  - [ ] `PING`, `VER`, `HELP`, `ECHO`, `ID` all work, re-registered in a new
        `system_commands.cpp` (ported bodies, not copied file).
  - [ ] Every `DEV …` handler replies using the standard `OK`/`ERR` taxonomy
        (`CommandProcessor::replyOK`/`replyErr`/`replyOKf`/`replyErrf`).
  - [ ] `docs/protocol-v2.md` gains a "Development commands" section
        documenting the full `DEV` vocabulary.
  - [ ] Host: `tests/bench/dev_exercise.py` scripts the SUC-003/004 flows
        through `NezhaProtocol.send()` over both serial and the relay `!GO`
        data plane.

## SUC-006: Auto-neutralize on comms silence
Parent: None (safety invariant — non-negotiable per runaway history)

- **Actor**: Bench operator (absence of action is the trigger)
- **Preconditions**: A `DEV M`/`DEV DT` motion command is active.
- **Main Flow**:
  1. Operator stops sending commands (serial disconnect, dropped relay link,
     or simply idle).
  2. The dev loop's serial-silence watchdog (default ~1 s, settable) detects
     the gap and drives all motors to neutral and the drivetrain to idle.
- **Postconditions**: No motor keeps running commanded motion once comms
  silence exceeds the watchdog window, regardless of which command family
  (single-motor or drivetrain) was last active.
- **Acceptance Criteria**:
  - [ ] Watchdog default window is ~1 s and is settable (a `DEV`/config verb
        or equivalent, not hardcoded only).
  - [ ] `DEV M …` motion deactivates drivetrain mode and vice versa
        (`DEV DT …` reactivates it) — only one authority drives the motors at
        a time.
  - [ ] Bench: stop sending commands mid-motion; motors reach neutral within
        the configured window, observed on the stand.

## SUC-007: Reorganize the test tree into sim/bench/playfield domains
Parent: None (infrastructure prerequisite for SUC-008)

- **Actor**: Firmware developer / test engineer
- **Preconditions**: `tests/` mixes `simulation/`, `sim/`, `field/`, `bench/`,
  `testgui/`, `calibrate/`, `_infra/`, `unit/`, `tools/` with unclear tier
  boundaries; `tests_old/` (SUC-001) holds the parked originals.
- **Main Flow**:
  1. Developer builds a new `tests/` skeleton with exactly three domain
     directories that are never combined: `sim/` (old `sim/` + `simulation/`
     merged, skeleton only this sprint), `bench/`, `playfield/` (renamed from
     `field/`), plus kept categories `unit/` and `tools/`.
  2. `testgui/` and the empty `calibrate/` shell are dropped; old `_infra` sim
     shims are left behind (a fresh sim harness is later-ticket work).
  3. `velocity_chart.py` is rewired to drive over `DEV DT`/`DEV M` and lands
     in `tests/bench/`; `plot_square.py`/`world_goto_chart.py` land in
     `tests/playfield/` carried over verbatim with a "parked" header note.
  4. New `dev_exercise.py`, `pid_hold_speed.py`, `ratio_governor_curve.py`
     are added to `tests/bench/`.
  5. `pyproject.toml`'s pytest config points `testpaths`/`norecursedirs` at
     the new tree only, excluding `tests_old/` and `source_old/`.
- **Postconditions**: `uv run python -m pytest` collects only the new tree;
  the three domains stay on their own machines/rigs and are never conflated.
- **Acceptance Criteria**:
  - [ ] `tests/CLAUDE.md` is rewritten to describe the three-domain
        structure and points to `.claude/rules/`.
  - [ ] `tests/sim/{unit,system}` exist as skeleton + conftest only.
  - [ ] `tests/bench/` contains `dev_exercise.py`, `pid_hold_speed.py`,
        `ratio_governor_curve.py`, and the reinvigorated `velocity_chart.py`.
  - [ ] `tests/playfield/` contains `plot_square.py` and
        `world_goto_chart.py`, each carrying a "parked" header note.
  - [ ] `uv run python -m pytest` succeeds and collects zero tests from
        `tests_old/` or `source_old/`.

## SUC-008: Validate the dev system on real hardware, including a coupled rig
Parent: None (acceptance gate for the whole sprint)

- **Actor**: Stakeholder / bench operator
- **Preconditions**: All prior SUCs are implemented; the robot (`tovez`) is
  on the stand with four motors wired; a bench rig exists with two motors
  mechanically linked on ports 3 and 4.
- **Main Flow**:
  1. Operator builds with `python build.py --clean` and flashes with
     `mbdeploy deploy robot --hex …` (ROLE-checked, never a blind copy).
  2. Operator runs the bench sequence from the issue's Verification section:
     per-motor DUTY/VEL/VOLT/RESET, `DEV DT VW` with hand-drag, watchdog
     silence.
  3. Operator runs `tests/bench/pid_hold_speed.py` (motor 3 holds a VEL
     target while motor 4 steps through load duties) and
     `tests/bench/ratio_governor_curve.py` (`DEV DT PORTS 3 4`, curve command,
     governor on vs. off).
  4. Operator runs the reinvigorated `tests/bench/velocity_chart.py`
     interactively while hand-loading wheels.
- **Postconditions**: The sprint's exit gate (issue's Verification section)
  is met on real hardware, not simulation or unit tests alone.
- **Acceptance Criteria**:
  - [ ] All bullet points under the issue's "Verification" section pass on
        the stand.
  - [ ] `pid_hold_speed.py` PASS: motor-3 measured velocity stays inside
        tolerance and recovers within a bounded settle time after each load
        step, with applied duty visibly rising as load increases.
  - [ ] `ratio_governor_curve.py` PASS: with the governor on, the measured
        wheel-speed ratio holds the commanded ratio within tolerance; with
        `sync_gain=0`, the ratio visibly drifts (governor-off control).
  - [ ] `dev_exercise.py` passes over both direct serial and the relay `!GO`
        data plane.
