---
id: '003'
title: Shared HAL command-edge types and Drivetrain reshape
status: done
use-cases:
- SUC-004
- SUC-005
- SUC-006
depends-on:
- '002'
github-issue: ''
issue: tick-model-command-flow-and-the-command-board-design-sketch.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Shared HAL command-edge types and Drivetrain reshape

## Description

Introduce the new shared HAL command-edge header and reshape
`Subsystems::Drivetrain` to the held-output pattern with its own port
binding and authority state, per `architecture-update.md`'s "The
command-edge types" and "Authority arbitration" sections. This ticket does
**not** touch `NezhaHal`/`NezhaMotor` (ticket 004) or `dev_commands.cpp`
(ticket 005) — it only builds the new types and reshapes `Drivetrain`
itself, so it lands ahead of both consumers.

**New header** — `source/hal/capability/hal_command.h` (headers-only, no
`.cpp`, matching the `capability/` constraint):
- `Hal::AddressedMotorCommand { uint32_t port; msg::MotorCommand command; }`
- `Hal::CommandProcessorToHalCommand { bool allPorts; uint8_t count;
  AddressedMotorCommand addressed[2]; }`
- `Hal::DrivetrainToHalCommand { AddressedMotorCommand wheel[2]; }`

These live in `Hal::capability`, not `subsystems/drivetrain.h`, specifically
so `NezhaHal` (ticket 004) never has to `#include "subsystems/drivetrain.h"`
— see the architecture doc's Design Rationale 1 for the full dependency-
direction argument. Do not relitigate that placement in this ticket.

**Drivetrain reshape** (`source/subsystems/drivetrain.{h,cpp}`):
- `msg::DrivetrainConfig` gains `left_port`/`right_port` (`uint32`, new
  proto fields 40/41 in `protos/drivetrain.proto`, regenerated via
  `scripts/gen_messages.py`).
- `msg::DrivetrainCommand` gains `optional bool standby = 6;` (new proto
  field), a side-channel riding beside the oneof exactly like
  `MotorCommand.feedforward`/`reset_position`.
- New `struct DrivetrainPorts { uint32_t left; uint32_t right; };` and
  `DrivetrainPorts ports() const` (reads `config_.get_left_port()`/
  `get_right_port()`).
- New `bool active_` member (replaces `DevLoopState::drivetrainActive`,
  which ticket 005 will delete); `bool active() const`.
- `setTwist()`/`setWheelTargets()`/`setNeutral()` each gain one line,
  `active_ = true;`.
- New `void standby()` — the one audited "relinquish authority" method:
  `active_ = false;` only, does **not** touch `mode_`/`neutralMode_`. A
  caller that also wants `mode_ == NEUTRAL` sends that via the same
  command's oneof arm alongside `standby=true` — `apply()` processes the
  oneof first, then the `standby` side-channel, so both effects compose in
  one call (see the architecture doc's worked example for why this exactly
  reproduces today's `neutralizeDrivetrain()`/steal-authority semantics).
- `tick()` becomes `void` (drop the `DrivetrainToMotorCommand` return);
  builds and holds a `Hal::DrivetrainToHalCommand` internally (using
  `config_.get_left_port()`/`get_right_port()` for `wheel[].port`); new
  `bool hasCommand() const` / `Hal::DrivetrainToHalCommand takeCommand()`.
  `tick()` sets `hasCommand_ = true` unconditionally whenever it runs
  (main.cpp only calls it when `active()`, per ticket 005's loop).
- The old `DrivetrainToMotorCommand` struct/return type is deleted.
- `governRatio()`/`commandedWheelTargets()`/`state()`/`capabilities()`/
  `setMotorCapabilities()` are otherwise **unchanged** — this is a reshape
  of the output/authority mechanism, not the control law.

## Acceptance Criteria

- [x] `hal/capability/hal_command.h` exists with the three types above,
      headers-only, no new `.cpp`.
- [x] `protos/drivetrain.proto` has `left_port`/`right_port` on
      `DrivetrainConfig` and `standby` on `DrivetrainCommand`; regenerated
      messages compile (`scripts/gen_messages.py` run, `messages/drivetrain.h`
      updated — do not hand-edit the generated file).
- [x] `Drivetrain::ports()`/`active()`/`standby()` exist with the exact
      semantics above; `setTwist`/`setWheelTargets`/`setNeutral` set
      `active_ = true`.
- [x] `Drivetrain::tick()` is `void`; `hasCommand()`/`takeCommand()` exist
      and yield `Hal::DrivetrainToHalCommand` addressed via
      `config_.left_port`/`right_port`.
- [x] `DrivetrainToMotorCommand` is deleted (no remaining references).
- [x] A command with `{control_kind=NEUTRAL(mode), standby=true}` results in
      `mode_==NEUTRAL && active_==false` after `apply()` (host test).
- [x] A command with `{control_kind=NONE, standby=true}` results in
      `active_==false` with `mode_`/targets **unchanged** (host test —
      this is the authority-steal case, and must NOT reset the last
      commanded target, matching today's exact quirk).
- [x] Ratio governor, kinematics, `state()`, `capabilities()` behavior is
      unchanged (existing Drivetrain tests, if any, still pass; if none
      exist yet, this ticket adds baseline coverage per the Testing plan).
- [x] `main.cpp`/`dev_commands.cpp` do **not** yet call the new
      `ports()`/`active()`/`standby()`/`hasCommand()`/`takeCommand()` API —
      that wiring is ticket 005's job. This ticket may leave `main.cpp`
      temporarily calling the old `Drivetrain::tick(now, l, r)` returning
      shape **only if** it still compiles; if `tick()`'s signature change
      breaks the build, make the minimal `main.cpp` adjustment needed to
      keep both `ROBOT_DEV_BUILD` forks building (do not do the full loop
      reshape here — flag any such minimal adjustment clearly in the PR/
      commit so ticket 005 knows what it's inheriting).

## Implementation Plan

**Approach**: proto schema changes first (regenerate, confirm the tree
still builds with the new fields unused), then `hal_command.h` (new,
additive), then the `Drivetrain` reshape (the bulk of the ticket), then
whatever minimal `main.cpp`/`dev_commands.cpp` touch-up is needed to keep
the tree building (full wiring is ticket 005, not this ticket).

**Files to create**:
- `source/hal/capability/hal_command.h`.

**Files to modify**:
- `protos/drivetrain.proto` — `left_port`/`right_port`, `standby`.
- `source/messages/drivetrain.h` — regenerated (do not hand-edit; run the
  generator).
- `source/subsystems/drivetrain.h` — `DrivetrainPorts`, `active()`,
  `ports()`, `standby()`, `hasCommand()`/`takeCommand()`, `tick()` signature,
  delete `DrivetrainToMotorCommand`.
- `source/subsystems/drivetrain.cpp` — implementations; `active_ = true` in
  the three setters; `apply()`'s standby side-channel dispatch.
- `source/main.cpp` — only the minimal touch-up needed to keep building
  (see Acceptance Criteria note).

**Testing plan**:
- Existing tests: `uv run python -m pytest`; `just build` both forks.
- New tests (host-level, structs in/out, no fakes needed — Drivetrain has
  no hardware dependency): construct a `Drivetrain`, drive it through
  `configure()`/`apply()`/`tick()` with plain `msg::` structs, and assert:
  `ports()` reflects a configured `left_port`/`right_port`; `active()`
  toggles correctly for TWIST/WHEELS/NEUTRAL/standby-with-neutral/
  standby-alone per the acceptance criteria above; `hasCommand()/
  takeCommand()` clears after take; the held `DrivetrainToHalCommand`'s
  `wheel[].port` matches the configured binding; ratio-governor output is
  numerically unchanged from before this ticket (regression guard) for a
  representative TWIST and WHEELS case.

**Documentation updates**: none required (architecture doc already covers
this); a short doc comment in `drivetrain.h` pointing at
`architecture-update.md`'s "Authority arbitration" section is good practice.
