---
id: '001'
title: Scaffold new source/, park old tree, copy dependency-clean infra, condition
  build tooling
status: open
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: greenfield-rebuild-faceplate-hal-in-a-fresh-source-old-tree-parked.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Scaffold new source/, park old tree, copy dependency-clean infra, condition build tooling

## Description

Rename the live `source/` and `tests/` trees to `source_old/`/`tests_old/`
(pure `git mv`, history preserved), stand up a minimal new `source/` that
builds, and copy in only the infrastructure that has zero dependency on
`state/`/`subsystems/`/`robot/`/`hal/`. This ticket produces the first hex
from the new tree and is the foundation every later ticket in this sprint
builds on. `tests_old/` itself is renamed here, but the new `tests/` skeleton
is ticket 006's job — do not create `tests/` content in this ticket beyond
what `git mv` leaves behind (i.e., nothing; `tests/` will not exist between
this ticket and ticket 006 landing, which is fine — no build step depends on
`tests/` existing).

This ticket does **not** touch `source_old/` or `tests_old/` beyond the
single rename commit. Programmers must not edit files inside those
directories for the rest of the sprint.

## Acceptance Criteria

- [ ] `git mv source source_old` is one commit; `git log --follow
      source_old/hal/real/Motor.cpp` (or any other pre-existing file) shows
      history intact across the rename.
- [ ] `git mv tests tests_old` is one commit (may be the same commit as the
      `source` rename or a separate one — programmer's choice; document
      which in the PR/commit message).
- [ ] A new `source/main.cpp` stub exists; `codal.json`'s `"application":
      "source"` is left unchanged (already correct — no edit needed, but
      verify it resolves to the new tree, not `source_old`).
- [ ] Copied verbatim into the new `source/`, verified dependency-clean
      (no include chain reaches `state/`, `subsystems/`, `robot/`, or `hal/`
      in the old tree):
  - `source/com/` — `SerialPort.{h,cpp}`, `Radio.{h,cpp}`, `RadioChannel.h`,
    `I2CBus.{h,cpp}`, `Communicator.{h,cpp}` (confirmed dependency-clean:
    only includes `MicroBit.h`/its own siblings).
  - `source/commands/` — `CommandProcessor.{h,cpp}`, `ArgParse.{h,cpp}`.
    Do NOT copy `SystemCommands.*`, `ConfigCommands.*`, `DebugCommands.*`,
    `MotionCommand(s).*`, `SimCommands.*` (all robot-coupled). `CommandQueue.h`
    is copied only if a concrete need for it emerges in ticket 5 (DEV
    dispatches immediately per the locked decision) — default to NOT
    copying it; note the decision either way.
  - `source/types/` — `CommandTypes.h`, `ArgSchema.h`, `Protocol.h`,
    `ValueSet.h`. Do NOT copy `Config.h` (the legacy `RobotConfig` blob — the
    new world configures via `msg::` types only).
  - `source/kinematics/` — `IKinematics.h`, `BodyKinematics.{h,cpp}`.
    `BodyKinematics.h` depends on `Pose2D.h`, which in the old tree lives at
    `source/hal/capability/Pose2D.h` (not under `kinematics/`) — copy just
    this one small value-type header alongside `BodyKinematics.h` into
    `source/kinematics/Pose2D.h` (do not pull in the rest of
    `hal/capability/`). `BodyKinematics.h`'s existing identifiers
    (`vWheelMax`, `vx_mmps`, etc.) are pre-existing verbatim-copied code and
    are exempt from the unit-suffix naming rule until touched
    (`.claude/rules/naming-and-style.md` rule 5 / the issue's Step 0 style
    note) — do not rename them as part of this copy.
- [ ] A new `source/commands/system_commands.cpp` (+ `.h` if warranted)
      re-registers liveness commands: `PING`, `VER`, `HELP`, `ECHO`, `ID`.
      Port the handler bodies from `source_old/commands/SystemCommands.cpp`
      (do not copy the whole file — that file also carries `HELLO`, `SNAP`,
      `ZERO`, `HALT`, etc. which are out of scope this sprint). New file,
      Google-style naming (`handlePing`, `handleVer`, ... — lowerCamelCase
      functions per `.claude/rules/coding-standards.md`).
- [ ] `build.py` conditioned: the `gen_default_config.py` and
      `check_config_sync.py` calls are skipped (or become no-ops) whenever
      `source/robot/` does not exist; `gen_messages.py` continues to run
      unconditionally, still targeting `source/messages/`. The conditioning
      test should be structural (does `source/robot/` exist?), not a version
      flag, so it self-heals once a later sprint adds `source/robot/` back.
- [ ] `python build.py --clean` succeeds and produces `MICROBIT.hex` from the
      new tree.
- [ ] Setting `codal.json`'s `application` to `source_old` and rebuilding
      still succeeds — the rollback path is exercised at least once and
      documented as working.
- [ ] `compile_commands.json` is regenerated and clangd is restarted after
      the tree move (known "squiggles" gotcha — see
      `.clasi/knowledge/squiggles-cdb-application-switch.md`); verify with
      `clangd --check` or equivalent that `source/` files no longer show
      phantom errors.
- [ ] No programmer edit touches any file under `source_old/` or
      `tests_old/` after the rename commit(s).

## Testing

- **Existing tests to run**: None from `tests/` — it does not exist yet
  after the rename (ticket 006 recreates it). `tests_old/` is expected
  broken against the new tree and is explicitly not chased this ticket.
- **New tests to write**: None (no test tree exists yet this ticket).
  Verification is build-based: `python build.py --clean` producing a hex is
  the acceptance gate.
- **Verification command**: `python build.py --clean` (from repo root).
  Also verify rollback: temporarily set `codal.json` `application` to
  `source_old`, run `python build.py --clean`, confirm success, then restore
  `application` to `source`.
