---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 088 Use Cases

Parent use cases are drawn from `docs/usecases.md` (the project's master use
case list) where an existing UC applies. Several SUCs below are internal
(developer/CI-facing) with no user-visible behavior; those are marked
`Parent: N/A` rather than forced onto an unrelated master UC.

## SUC-001: Straight-line and turn-in-place drive commands move the robot correctly

Parent: UC-001 (Drive Robot at Continuous Speed), UC-002 (Drive Robot for
Timed Duration), UC-003 (Drive Robot a Specific Distance)

- **Actor**: Python host / stakeholder on the bench
- **Preconditions**: Robot on the stand, wheels off the ground. Firmware
  flashed with corrected per-port `fwd_sign`.
- **Main Flow**:
  1. Host sends `S`, `T`, or `D` with equal-magnitude, same-signed left/right
     arguments (a "straight" command).
  2. Firmware applies the per-port `fwd_sign` from boot config to each
     drive-pair motor.
  3. Both wheels roll in the same physical direction (both forward, or both
     reverse, matching command sign).
  4. Both encoders increment with the same sign.
- **Postconditions**: The robot drives straight (no self-fighting yaw); a
  spin command (opposite-signed L/R) still produces opposite wheel motion.
- **Acceptance Criteria**:
  - [ ] `D +<d> +<d>` on the stand: both wheels turn the same physical
        direction, both encoders increment positive.
  - [ ] The corrected `fwd_sign` is baked via `scripts/gen_boot_config.py`
        from `data/robots/tovez*.json`, not hand-edited into
        `boot_config.cpp`, and survives a clean rebuild.
  - [ ] A spin/turn command (opposite-signed L/R) is unaffected.

## SUC-002: Firmware liveness commands (PING, VER) reply reliably

Parent: N/A — protocol liveness surface (`docs/protocol-v2.md`'s system
command family); not modeled as its own UC in `docs/usecases.md`.

- **Actor**: Python host
- **Preconditions**: Robot connected over serial or radio.
- **Main Flow**:
  1. Host sends `PING`. Firmware replies `OK pong t=<ms>`.
  2. Host sends `VER`. Firmware replies `OK ver fw=<version> proto=<n>`.
- **Postconditions**: Both liveness verbs reply within the normal command
  round-trip time; neither times out.
- **Acceptance Criteria**:
  - [ ] `VER` returns a well-formed `OK ver ...` reply on serial.
  - [ ] `VER` returns a well-formed `OK ver ...` reply on radio.
  - [ ] Root cause of the prior no-reply behavior is identified and
        documented (even if the fix turns out to be small), since static
        reading during planning found no defect in the shared dispatch path
        `VER` shares with `PING`.

## SUC-003: HELP reflects the live registered command surface

Parent: N/A — protocol liveness surface; not modeled as its own UC in
`docs/usecases.md`.

- **Actor**: Python host / stakeholder inspecting the robot interactively
- **Preconditions**: Robot running the dev build (`ROBOT_DEV_BUILD=1`).
- **Main Flow**:
  1. Host sends `HELP`.
  2. Firmware enumerates the live `CommandProcessor` descriptor table
     (system + dev + telemetry + motion + config + pose + otos families).
  3. Firmware replies `OK help <space-separated verbs>`.
- **Postconditions**: The reply lists every verb actually registered and
  dispatchable in the current build, with no hand-maintained list to drift.
- **Acceptance Criteria**:
  - [ ] In the dev build, `HELP` lists every registered verb, not just the
        five liveness verbs.
  - [ ] Adding or removing a command family changes `HELP`'s output with no
        edit to the `HELP` handler itself.
  - [ ] On the stand: `HELP` over the real link returns the full, accurate
        verb set.

## SUC-004: Robot announces its identity on connect and on HELLO

Parent: UC-018 (Device Discovery) — this SUC corrects UC-018's stale
description (which references a removed `Announcer` class and a periodic
broadcast that this design does not implement); `docs/usecases.md` should be
updated to match during a future consolidation pass.

- **Actor**: Python host
- **Preconditions**: Robot firmware boots with both serial and radio
  channels configured.
- **Main Flow — boot**:
  1. Firmware brings up serial and radio (`Communicator::begin()`).
  2. Before the main loop starts, firmware emits
     `DEVICE:NEZHA2:robot:<name>:<serial>` as the first line on both serial
     and radio.
- **Main Flow — HELLO**:
  1. Host sends `HELLO` on either channel.
  2. Firmware re-emits the same banner on the channel `HELLO` arrived on.
- **Postconditions**: The host's existing parsers (`serial_conn.py`,
  `connection.py`, `testgui/transport.py`) classify the device as a
  direct/robot device from the banner; `config/devices.json`'s cached
  `DEVICE:NEZHA2:robot:tovez:...` entries keep matching unchanged.
- **Acceptance Criteria**:
  - [ ] The banner is the first line out on serial at boot.
  - [ ] The banner is the first line out on radio at boot (understanding
        radio is fire-and-forget — it only reaches a host if a relay is
        already listening; a missed boot radio banner is not a failure).
  - [ ] `HELLO` re-emits the banner on the arriving channel.
  - [ ] On the stand: the banner appears on serial at connect; over the
        radio/relay path a `HELLO` re-request returns the banner.

## SUC-005: "Statement" terminology is fully removed with no behavior change

Parent: N/A — internal identifier/naming hygiene, no externally visible
behavior.

- **Actor**: Developer / future maintainer reading or extending the code
- **Preconditions**: Sprint 087's runtime tier (`Rt::Blackboard`,
  `CommandRouter`, `Communicator`) uses "Statement" throughout.
- **Main Flow**:
  1. Every `statement`/`Statement` identifier in `source/` and `host/` is
     renamed under the new vocabulary: wire-inbound raw lines are
     `command`s (e.g. `CommunicatorToCommandProcessorStatement` →
     `CommunicatorToCommandProcessorCommand`); internal typed
     representations remain `message`s (`msg::*`, unaffected).
  2. `.claude/rules/naming-and-style.md` rule 4 is rewritten to state the
     command/message split and drop the `Statement` payload type.
  3. Firmware and host build; the full test suite passes with no behavior
     change (pure rename).
- **Postconditions**: `grep -rn "[Ss]tatement" source/ host/` returns
  nothing outside `tests_old/`/`source_old/`/archived sprint docs.
- **Acceptance Criteria**:
  - [ ] No `statement`/`Statement` identifier remains in `source/` or
        `host/` (including comments describing the removed concept).
  - [ ] `CommunicatorToCommandProcessorStatement`, `statementsIn`,
        `hasStatement()`/`takeStatement()` are renamed under the new
        vocabulary throughout every call site (`main.cpp`, `blackboard.h`,
        `command_router.{h,cpp}`, `communicator.{h,cpp}`, the command-family
        `.cpp`/`.h` files, `radio.h`, `nezha_motor.cpp`).
  - [ ] `.claude/rules/naming-and-style.md` rule 4 no longer contains a
        `Statement` payload type and documents the command/message split.
  - [ ] `uv run python -m pytest` and the firmware build are green after the
        rename; behavior is unchanged.

## SUC-006: A test can exercise a command on a chosen channel and observe the reply on that channel

Parent: UC-019 (Radio Relay Mode) — this SUC is the test-infrastructure
counterpart: proving in the sim harness that the serial/radio distinction is
real, not merely wired to one shared sink.

- **Actor**: Test author / pytest suite
- **Preconditions**: `tests/_infra/sim/sim_api.cpp`'s `sim_command()`
  hardcodes `returnPath = SERIAL` and both of `CommandRouter`'s reply
  channels resolve to the same sink.
- **Main Flow**:
  1. A test calls a channel-aware harness entry point (e.g.
     `sim_command_on(h, line, channel, reply, size)`), selecting SERIAL or
     RADIO.
  2. The harness posts a `CommunicatorToCommandProcessorCommand` with
     `returnPath` set to the requested channel.
  3. `CommandRouter`'s two reply sinks are kept distinct in the harness;
     the reply is captured from the sink matching the requested channel.
- **Postconditions**: A test can assert a reply arrived on the channel it
  was sent on, distinct from the other channel's sink.
- **Acceptance Criteria**:
  - [ ] A test can select SERIAL vs RADIO as the return channel for one
        command.
  - [ ] The reply is observable on the chosen channel's own sink, and NOT
        on the other channel's sink.
  - [ ] `host/robot_radio/io/sim_conn.py` exposes the new entry point via
        ctypes, matching the existing `sim_command()` binding pattern.

## SUC-007: Every registered command has smoke coverage on both channels

Parent: covers the full registered command surface — UC-001 through
UC-019 collectively (every command family), plus SUC-006 above (its
prerequisite).

- **Actor**: Developer / CI (`uv run python -m pytest`)
- **Preconditions**: SUC-006's channel-aware harness exists. The full
  registered command table is enumerable (per SUC-003's `HELP` mechanism).
- **Main Flow**:
  1. For each registered command (system + dev + telemetry + motion +
     config + pose + otos), a smoke test sends it via SERIAL, then via
     RADIO.
  2. Each send asserts a well-formed reply (`OK ...` or a defined `ERR ...`
     — reachability, not hardware presence, for hardware-gated verbs like
     OTOS `OI`/`OL`/`OA` on `SimMotor`).
  3. A completeness meta-test enumerates the live table and fails if any
     registered verb lacks a smoke test (or vice versa).
- **Postconditions**: The command surface is provably dispatchable on both
  channels; a future command-family addition without a matching smoke test
  fails CI.
- **Acceptance Criteria**:
  - [ ] One smoke-test function per registered command, each exercising
        both channels.
  - [ ] The completeness meta-test passes today and would fail if a
        registered verb had no smoke test.
  - [ ] `uv run python -m pytest` runs the suite green as part of the
        standard gate.

## SUC-008: Motion and configuration commands have behavioral (non-smoke) sim coverage

Parent: UC-001, UC-002, UC-003, UC-004 (Stop Robot Immediately), UC-014
(Tune Calibration Parameters at Runtime), UC-015 (Drive to Relative XY
Position)

- **Actor**: Developer / CI
- **Preconditions**: Smoke coverage (SUC-007) proves reachability; this SUC
  proves actual behavior of the subsystems backing motion/config commands.
- **Main Flow**:
  1. For each motion verb (`S T D R TURN RT G STOP`) and config surface
     (`SET`/`GET`, `DEV` config subcommands), a behavioral sim test drives
     the command through `Drivetrain`, `Hal::Motor`/`NezhaMotor` (via
     `SimMotor`), `PoseEstimator`, `Planner`, or `Configurator` as
     appropriate and asserts the expected subsystem-level effect (e.g. a
     `D` command's encoders converge to the commanded distance within
     tolerance in the simulated plant).
  2. Existing tests in `tests/sim/unit/` (e.g. `test_motion_commands.py`,
     `test_drivetrain.py`, `test_configurator.py`) are extended or
     supplemented to close gaps, not rewritten wholesale.
- **Postconditions**: Every motion/config command has both a smoke test
  (SUC-007) and a behavioral test proving its effect on the simulated
  plant.
- **Acceptance Criteria**:
  - [ ] Every motion command has a behavioral sim test beyond its smoke
        test.
  - [ ] Every config command (`SET`/`GET`, `DEV *CFG`) has a behavioral sim
        test proving the config value takes effect.
  - [ ] `uv run python -m pytest` is green.

## SUC-009: On-the-stand bench verification proves motion and config commands function on real hardware

Parent: UC-001, UC-002, UC-003, UC-004, UC-005 (Query Encoder Positions),
UC-014, UC-019

- **Actor**: Stakeholder / operator on the bench (HITL)
- **Preconditions**: Robot mounted on the stand, wheels off the ground.
  SUC-001 (wheel-direction fix) and SUC-005 (statement rename) have landed.
  Firmware deployed via `mbdeploy deploy --build`.
- **Main Flow**:
  1. For each motion verb (`S T D R TURN RT G STOP`), the operator issues
     the command over serial (and, where practical, over the radio relay)
     and confirms the wheels move as commanded and the encoders increment
     in the expected direction, roughly proportional to the command.
  2. For each config verb (`SET`/`GET`, `DEV` config subcommands), the
     operator confirms the change takes effect (readback matches, or
     behavior visibly changes).
  3. Commands that cannot be fully validated on the stand (OTOS absolute
     position without real translation, camera `SI` pose-inject,
     playfield-frame `G`/goto) are exercised at smoke/dispatch level only,
     with a written note of why and what would fully validate them.
  4. Results are captured as a bench checklist/log committed to the sprint
     directory.
- **Postconditions**: A written record exists of which commands were fully
  bench-verified vs. smoke-only-on-stand, and why.
- **Acceptance Criteria**:
  - [ ] Every motion verb drives the wheels with encoders incrementing as
        expected, verified over the real link.
  - [ ] Every config verb's effect is observable, verified over the real
        link.
  - [ ] Radio-relay path is exercised for at least the commands practical
        to test that way, not serial-only.
  - [ ] A short written record (bench checklist/log) is captured in the
        sprint directory distinguishing fully-verified from smoke-only
        commands.
