---
id: 008
title: Behavioral sim coverage for motion and config command subsystems
status: done
use-cases:
- SUC-008
depends-on:
- '006'
- '007'
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

- [x] Every motion command (`S R T D TURN RT G STOP`) has a behavioral
      sim test beyond its smoke test, asserting an actual effect on the
      simulated plant (e.g. encoders/pose converge as expected within
      tolerance).
- [x] Every config command (`SET`/`GET`, `DEV *CFG` subcommands) has a
      behavioral sim test proving the configured value takes effect
      (readback matches, or dependent behavior visibly changes).
- [x] Gaps are filled by extending existing test files where a natural
      home exists, not by wholesale rewrite.
- [x] `uv run python -m pytest` is green.

## Completion Notes (2026-07-07)

**Gap audit.** Surveyed `tests/sim/unit/` against every motion/config verb.
Most of the surface was already thoroughly covered behaviorally
(`test_motion_commands.py`, `test_motion_commands_arc_turn.py`,
`test_motion_commands_goto.py`, `test_motion_verbs_full_sequence.py`,
`test_config_registry.py`'s `SET tw=`/`SET rotSlip=` downstream-effect
tests, `test_otos_commands.py`). Six genuine gaps were identified and
closed, all by extending existing files (no new test file needed):

1. **`fwd_sign` (088-002 depth).** `test_gen_boot_config_fwd_sign.py`'s own
   docstring explicitly disclaims proving behavior "against the simulator"
   (`Hal::SimMotor` never reads `config_.fwd_sign` at all — confirmed by
   inspection of `source/hal/sim/sim_motor.cpp`; only the real
   `Hal::NezhaMotor` leaf consumes it). Closed with a new scenario in
   `nezha_flipflop_harness.cpp` (`scenarioFwdSignNegatesEncoderPositionSign`,
   already compiled/run by `test_nezha_flipflop.py`): two standalone
   `NezhaMotor` objects differing only in `fwd_sign` (+1/-1), fed the
   IDENTICAL scripted raw encoder bytes, must report opposite-sign
   `position()` — the sim-side, real-HAL proof of the mirror-mounted
   wheel-direction fix beyond 002's generator/config-value-only check.
2. **`S` holding commanded wheel speed.** The existing S test only checked
   forward progress, not that `sim.vel()` actually converges to and holds
   the commanded value. Added
   `test_s_wheel_speeds_converge_to_and_hold_the_commanded_value`
   (`test_motion_commands.py`).
3. **Per-wheel same-sign/opposite-sign at the wire-verb layer.**
   `test_plant_correctness.py` already proved this at the raw `DEV M DUTY`
   layer; no test proved it for the actual `D`/`RT` wire verbs via
   `sim.true_wheel_travel()`. Added
   `test_d_straight_drive_moves_both_wheels_the_same_sign`
   (`test_motion_commands.py`) and
   `test_rt_spin_moves_the_two_wheels_in_opposite_directions`
   (`test_motion_commands_arc_turn.py`).
4. **`SET sTimeout=` propagation.** Only round-trip tested; no test proved
   the SET value actually retunes S's streaming-drive watchdog firing time.
   Added `test_set_stimeout_changes_the_streaming_drive_watchdog_firing_time`
   (`test_motion_commands.py`).
5. **`DEV DT CFG trackwidth=` propagation.** The DEV-plane config surface
   (separate code path from `SET`) had no behavioral test at all. Added
   `test_dev_dt_cfg_trackwidth_visibly_changes_arc_geometry`
   (`test_config_registry.py`), mirroring the existing `SET tw=` test.
6. **`DEV STOP`.** Smoke-only (ack text only) — no test proved it actually
   neutralizes a driving motor. Added
   `test_dev_stop_neutralizes_a_driving_motor` (`test_protocol_roundtrips.py`).

`minSpeed` (`PlannerConfig.min_speed`) was checked and found to have no
downstream consumer anywhere in `source/` (dead/reserved config field,
same as `DrivetrainConfig.fwd_sign_l/r`) — its existing round-trip-only
coverage is correct and complete; no further test was added for it.

**Result:** `uv run python -m pytest tests/sim -q` → 303 passed, 4 xfailed
(baseline was 297 passed, 4 xfailed — 6 new passing tests, no regressions).

## Testing

- **Existing tests to run**: full `uv run python -m pytest`, with
  particular attention to the extended files listed above.
- **New tests to write**: behavioral test additions/extensions as scoped
  above, closing any smoke-only gap identified against ticket 007's list.
- **Verification command**: `uv run python -m pytest`.
