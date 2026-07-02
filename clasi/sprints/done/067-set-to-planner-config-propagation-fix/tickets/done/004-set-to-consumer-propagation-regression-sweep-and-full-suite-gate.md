---
id: '004'
title: SET-to-consumer propagation regression sweep and full-suite gate
status: done
use-cases:
- SUC-003
depends-on:
- '001'
- '002'
- '003'
github-issue: ''
issue: set-config-not-propagated-to-planner.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# SET-to-consumer propagation regression sweep and full-suite gate

## Description

Tickets 001-003 fix the three distinct propagation bugs this sprint's audit
found (Planner's owned-value `_cfg`, Drive's `_drvCfg` shadow for
`tw`/`lag.otos`, and the missing EKF noise-update path for `ekfRHead`).
This ticket adds the recurrence guard the issue's own acceptance criteria
demand: a table-driven sweep test that `SET`s each motion-critical key this
sprint's audit identified as STALE and asserts the owning consumer observes
the new value, so a future regression that reintroduces a stale config copy
anywhere in this key set fails a specific, named test — not a test that
happens to pass regardless of underlying behavior (which is exactly what
`test_rt_slip.py` was doing before Ticket 001 fixed it).

The audit's STALE rows (from `architecture-update.md`'s full audit table)
that this sweep must cover: `rotSlip`, `tw`, `vWheelMax`, `rotGainPos`,
`rotGainNeg`, `turnGate`, `ctrlPeriod`, and `ekfRHead`.

Each measurement must be isolated — a fresh `Sim()` instance or a
successful (reply-checked) `ZERO enc`, never a bare `ZERO` — per the exact
false-positive pattern Ticket 001 found and fixed in `test_rt_slip.py`
(`parseZero()` in `source/commands/SystemCommands.cpp` rejects a bare
`ZERO` with `ERR badarg`; an unchecked reply lets encoder state accumulate
across sequential measurements within one test, faking a slip/behavior
change that isn't real).

This ticket also serves as the sprint's final acceptance gate: the full
default pytest suite must be green (baseline 2506 passed / 0 failed, per
`architecture-update.md`'s Step 4-5 item 4) after all three fix tickets and
this sweep test are in place.

See `architecture-update.md` Step 4-5 item 4 and `usecases.md` SUC-003 for
the full design and acceptance criteria this ticket implements.

## Acceptance Criteria

- [x] New sim test file (e.g.
      `tests/simulation/unit/test_set_config_propagation_sweep.py` or
      similar, per project test-naming convention) with a table-driven
      case for each of: `rotSlip`, `tw`, `vWheelMax`, `rotGainPos`,
      `rotGainNeg`, `turnGate`, `ctrlPeriod`, `ekfRHead`.
      Implemented as `tests/simulation/unit/test_067_004_set_propagation_sweep.py`
      (explicit functions, not `@pytest.mark.parametrize`, matching this
      directory's established style) with 10 test functions: `rotSlip`,
      `tw` (two tests — see TWIST-shadow-site note below), `vWheelMax`,
      `rotGainPos`, `rotGainNeg`, `turnGate`, `ctrlPeriod`, `lag.otos`
      (bonus — already covered by 067-002's own test, included here too so
      this file is a complete single-file record of every STALE key), and
      `ekfRHead`.
- [x] Each case: `SET`s the key to a value distinct from its boot default,
      exercises the one sim-observable behavior that depends on it (e.g.
      RT arc for `rotSlip`/`tw`/`rotGainPos`/`rotGainNeg`/`turnGate`; the
      EKF-predict trackwidth/lag compensation for `tw`/`lag.otos`-adjacent
      behavior already covered by Ticket 002; the OTOS heading-correction
      weighting for `ekfRHead`), and asserts the observed behavior differs
      from the boot-default behavior.
      One exception, documented in-file: `vWheelMax`'s Planner-side
      consumer (`_startPreRotate`'s `omegaMax` clamp) was found, by
      reverting Planner.h/.cpp to the pre-067-001 value-copy `_cfg` and
      re-running the exact same scenario, to have NO independently
      observable behavioral signature today — two other already-live
      downstream clamps (`BodyVelocityController`'s `yawRateMax` ramp-target
      clamp and its wheel-speed saturation, both formula-identical or
      tighter) always bind first, for every reachable `vWheelMax` value.
      The sweep's `vWheelMax` test therefore exercises `BodyVelocityController`'s
      already-live saturation end-to-end (proving `SET vWheelMax` reaches a
      live consumer) rather than isolating `_startPreRotate`'s specific,
      also-fixed-but-currently-non-binding clamp; this is called out
      explicitly in the test file so a future maintainer does not mistake
      it for narrow coverage of that one code path.
- [x] Every measurement is isolated: fresh `Sim()` per value under test, OR
      a `ZERO enc` (never bare `ZERO`) with the reply checked
      (`assert "OK" in reply`) between measurements within the same test.
      Every sweep test uses a fresh `Sim()` per value (no shared-instance
      `ZERO enc` pattern was needed).
- [x] `tests/simulation/unit/test_rt_slip.py`'s three existing tests are
      confirmed to still pass for the right reason (already fixed in
      Ticket 001; this ticket only re-verifies as part of the full-suite
      gate, no further changes expected here).
      Re-run explicitly alongside the new sweep file: all 3 pass.
- [x] Full default pytest suite green: `uv run python -m pytest` reports
      2506 passed (baseline) plus this sprint's new tests, 0 failed. This
      is the sprint's closing acceptance gate.
      Result: **2520 passed, 0 failed** (2510 baseline going into this
      ticket — 2506 + 2 (Ticket 002) + 2 (Ticket 003) — plus 10 new tests
      from this ticket's sweep file).

## Testing

- **Existing tests to run**: full default suite via
  `uv run python -m pytest` (this IS the ticket's primary deliverable —
  confirming the baseline plus new tests are green).
- **New tests to write**: the table-driven sweep test described above,
  covering all eight STALE keys from the audit.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Write one new table-driven (or explicitly enumerated, if the
project's sim test style prefers explicit functions over
`@pytest.mark.parametrize`) test module that exercises each STALE key from
the audit table in `architecture-update.md`, using the isolation pattern
Ticket 001 established for `test_rt_slip.py` (fresh `Sim()` or
reply-checked `ZERO enc`). This ticket depends on 001, 002, and 003 because
it tests behavior those tickets introduce — it cannot pass (and should not
be started) before all three land.

**Files to create**:
- A new test file under `tests/simulation/unit/` covering the sweep (name
  per existing test-file conventions in that directory — check sibling
  files like `test_rt_slip.py`, `test_059_config_routing.py` for the
  established pattern before naming).

**Files to modify**: none expected beyond the new test file — this ticket
should not require further source changes if Tickets 001-003 are complete
and correct. If the sweep surfaces a gap Tickets 001-003 missed, treat that
as a signal to revisit those tickets rather than papering over it in the
test.

**Testing plan**:
- Implement the sweep test per the acceptance criteria above.
- Run the full default suite (`uv run python -m pytest`) and confirm the
  final green count matches or exceeds the 2506-passed / 0-failed baseline
  recorded in `architecture-update.md`.
- This is the sprint's closing gate — do not mark this ticket (or the
  sprint) done until the full suite is confirmed green.

## Implementation Notes (as executed)

**TWIST-shadow-site carried over from Ticket 002's execution.** Ticket
002's programmer found a second `_drvCfg.get_trackwidth()` shadow read in
`Drive::tickAction()`'s TWIST inverse-kinematics case
(`source/subsystems/drive/Drive.cpp`, then lines ~273-275), structurally
identical to the `tickUpdate()` shadow 002 fixed, but out of that ticket's
scope. Decision: **fixed it**, not documented-only — it is the exact same
bug class (a `msg::DrivetrainConfig` snapshot populated once at boot by
`Robot::Robot()`'s `drive.configure(toDriveConfig(config))`, never
refreshed again because `tw` is not `"drive"`-annotated), and it sits on
the vehicle for essentially every motion command (`Planner::tick()` always
packs a `DrivetrainCommand{TWIST}`, verb_id=1 — RT, D, T, VW, and G's
PURSUE/PRE_ROTATE phases all route through it). The fix reads
`_robCfg.trackwidthMm` directly, matching Ticket 002's fix one function up
and the already-correct `rotationalSlip` read one line below the original
site. A dedicated regression test
(`test_tw_changes_twist_inverse_kinematics`, using a fire-and-forget `_VW`
command to isolate this exact read from Planner's separate, threshold-based
RT-arc stop condition) was written and confirmed to FAIL against the
pre-fix code (wheel speed frozen at ~28 mm/s regardless of `SET tw`) and
PASS against the fix (wheel speed scales with `tw`: 64→12, 128→28, 256→60,
500→112 mm/s).

**vWheelMax finding.** While building the `vWheelMax` sweep case, reverting
Planner.h/.cpp to the pre-067-001 value-copy `_cfg` and re-running the same
G/PRE_ROTATE scenario produced numerically IDENTICAL results to the fixed
build at every `vWheelMax` value tried (40 through 1200 mm/s). Root cause:
`BodyVelocityController::advance()` (Planner's own internal `_bvc`, used to
profile PRE_ROTATE's spin) clamps its ramp target to `yawRateMax` (70
deg/s ≈ 78 mm/s-equivalent wheel speed) before ramping, and separately
saturates the post-kinematics wheel speeds via the formula-identical,
already-live `_cfg.vWheelMax` (`BodyVelocityController::_cfg` is `const
RobotConfig&`, confirmed LIVE by the audit). Both bind at least as tightly
as `_startPreRotate`'s own clamp for every value tried, at and above the
default (400 mm/s already exceeds yawRateMax's ceiling) and below it
(BVC's own saturation reproduces the correct value independent of
Planner's clamp). This does not change Ticket 001's fix — converting
`Planner::_cfg` to a live reference is still correct and necessary for
`vWheelMax` along with every other field — but it means this one field's
specific consumer has no currently isolable behavioral signature. The
sweep's `vWheelMax` test is written and documented accordingly: it proves
`SET vWheelMax` reaches a live consumer end-to-end (via BVC's saturation),
not that it isolates `_startPreRotate`'s own clamp specifically.

**Full-suite result**: 2520 passed, 0 failed (`uv run python -m pytest`).

**Documentation updates**: none — `architecture-update.md` already
documents this change in full (Step 4-5 item 4, `usecases.md` SUC-003). No
wire-protocol change, no `RobotConfig` schema change.
