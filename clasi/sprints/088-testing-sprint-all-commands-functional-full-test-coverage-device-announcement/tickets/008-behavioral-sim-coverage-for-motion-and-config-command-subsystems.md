---
id: '008'
title: Behavioral sim coverage for motion and config command subsystems
status: open
use-cases: [SUC-008]
depends-on: ['006', '007']
github-issue: ''
issue: rebuild-test-suite-and-verify-commands-functional.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Behavioral sim coverage for motion and config command subsystems

## Description

Beyond the per-command smoke suite (ticket 007), the stakeholder wants
"expand out and rebuild all the other tests" for the subsystems that back
motion and configuration commands: `Drivetrain`, `Hal::Motor`/`NezhaMotor`
(via `SimMotor`), `PoseEstimator`, `Planner`, `Communicator`,
`CommandRouter`/`Configurator`. Smoke coverage (ticket 007) proves a
command dispatches and gets a well-formed reply; this ticket proves the
command actually does the right thing to the simulated plant (e.g. a `D`
command's encoders converge to the commanded distance, a `SET` config
value is observably applied). Bounded per the issue: extend/supplement
existing coverage where a gap exists — this is not a from-scratch rebuild
of `tests_old/`.

## Implementation Plan

**Approach**: Audit existing `tests/sim/unit/` coverage against every
motion verb (`S T D R TURN RT G STOP`) and config surface (`SET`/`GET`,
`DEV *CFG`) for a genuine behavioral assertion (drives the simulated
plant and checks an effect), not just "got a reply." Existing files
already cover much of this ground — `test_motion_commands.py`,
`test_motion_commands_goto.py`, `test_motion_commands_arc_turn.py`,
`test_drivetrain.py`, `test_configurator.py`, `test_config_registry.py`,
`test_otos_commands.py`, `test_pose_commands.py` — extend/supplement
these rather than rewriting. Cross-reference against ticket 007's smoke
list to find any motion/config command with smoke-only coverage and no
behavioral test, and close each such gap.

**Files to create/modify**: extensions to the existing `tests/sim/unit/*.py`
files listed above; a new file only where no existing file is a natural
home for a genuinely missing behavioral case.

**Testing plan**: this ticket's product IS test content — verify via
`uv run python -m pytest` staying green, and record which commands
gained new/extended coverage.

**Documentation updates**: none required.

## Acceptance Criteria

- [ ] Every motion command (`S R T D TURN RT G STOP`) has a behavioral
      sim test beyond its smoke test, asserting an actual effect on the
      simulated plant (e.g. encoders/pose converge as expected within
      tolerance).
- [ ] Every config command (`SET`/`GET`, `DEV *CFG` subcommands) has a
      behavioral sim test proving the configured value takes effect
      (readback matches, or dependent behavior visibly changes).
- [ ] Gaps are filled by extending existing test files where a natural
      home exists, not by wholesale rewrite.
- [ ] `uv run python -m pytest` is green.

## Testing

- **Existing tests to run**: full `uv run python -m pytest`, with
  particular attention to the extended files listed above.
- **New tests to write**: behavioral test additions/extensions as scoped
  above, closing any smoke-only gap identified against ticket 007's list.
- **Verification command**: `uv run python -m pytest`.
