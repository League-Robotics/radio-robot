---
id: '010'
title: Tier-1 fault-knob matrix + lag-on validation
status: done
use-cases:
- SUC-012
depends-on:
- '007'
- 008
github-issue: ''
issue: motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Tier-1 fault-knob matrix + lag-on validation

## Description

Exercise the sim's existing fault knobs (`motor_lag`, `enc_slip`,
`stiction`, `trackwidth`, `scrub`) against `source/drive/` through the
now-live adapter (tickets 007/008), with `motor_lag` at 120-140ms as the
default for every tracker/replan scenario — the zero-lag path is reserved
for golden-TLM bit-exactness only.

## Acceptance Criteria

- [x] Sim test matrix covers `motor_lag`(120-140ms)/`enc_slip`/
      `stiction`/`trackwidth`-error/`scrub`, each run against at least
      one arc and one pivot segment through the live adapter.
- [x] `enc_slip`/scale faults are checked against `true_pose` convergence
      (NEVER `bb.fusedPose`, per the plant-model convention — grep/
      review-verifiable in the new test file).
- [x] `stiction` is checked for terminal walk-in with no premature
      `DONE_STOP` and no reversal (a dedicated no-reversal assertion,
      mirroring ticket 005's own terminal-machine regression test, now
      through the full adapter+plant stack).
- [x] `trackwidth` error is checked for cross-gain (`k_c`) correction of
      the resulting radius error.
- [x] An infeasible ask under fault conditions produces a typed `ERR`
      with the queue untouched (no hang, no silent wrong answer).
- [x] The zero-lag sim path is explicitly EXCLUDED from this matrix's
      tracker/replan scenarios — a comment/assertion in the test file
      documents why (reserved for golden-TLM bit-exactness only).
- [x] No scenario in the matrix reproduces the 2026-07-11 false-green
      (zero-lag-only validation) failure class — an explicit note in
      completion notes cross-references that incident.
- [x] `uv run python -m pytest` passes.

## Testing

- **Existing tests to run**: `uv run python -m pytest`.
- **New tests to write**: the fault-knob matrix itself (see Acceptance
  Criteria).
- **Verification command**: `uv run pytest`

## Implementation Plan

**Approach**: the sim's fault knobs already exist (`motor_lag`/
`enc_slip`/`stiction`/`trackwidth`/`scrub` on the `Sim` class) — this
ticket is pure test-writing against the now-live adapter; no production
code change is expected. If a fault-knob combination surfaces a real
defect, do NOT fix it inline here — file it and reopen the relevant
ticket (004/005/007), matching this sprint's own acceptance-tickets-
don't-silently-patch-code precedent (M11/M12).

**Files to create**: fault-matrix test file(s) under `tests/sim/system/`
or `tests/sim/unit/` (programmer's judgment on exact location, matching
`tests/CLAUDE.md`'s domain split).

**Testing plan**: the matrix itself is the testing plan.

**Documentation updates**: none.

## Completion notes

New file: `tests/sim/unit/test_fault_knob_matrix.py` (19 tests, pure
test-writing against the now-live wire adapter -- no `source/` change; no
real defect surfaced).

**Matrix structure**: `test_fault_matrix_admits_and_converges` is a
`pytest.mark.parametrize` product of 5 fault knobs (`motor_lag`/`enc_slip`/
`stiction`/`trackwidth`/`scrub`) x 2 segment kinds (a genuine curved arc,
500mm/60deg; a 90deg pivot) = 10 base cases, each proving: admitted over
the wire (`segment` `CommandEnvelope`, real admission -- never
`sim.post_segment()`'s bypass), reaches a terminal idle state within a
generous wall-clock budget (the "no hang" proxy), and `sim.true_pose()`
lands within a fault-tolerant bound of the ideal end pose (finite, no
runaway). `motor_lag` is ALWAYS on (120-140ms, `_MANDATORY_MOTOR_LAG =
130.0`) for every one of these 10 cases, including the 8 rows testing a
DIFFERENT knob -- baked in structurally via the shared `_apply_faults()`
helper's own default, not opted into per test.

On top of the base matrix, 4 deeper checks, one per AC bullet naming a
specific mechanism:
- `test_enc_slip_true_pose_convergence` / `test_enc_scale_error_true_pose_
  convergence` (arc + pivot each, 4 tests): checked ONLY against
  `sim.true_pose()` -- this Sim ctypes ABI (`tests/_infra/sim/firmware.py`)
  has no `fused_pose()` accessor at all, so "never `bb.fusedPose`" is
  structurally enforced, not just a convention. Measured (this exact
  plant/adapter pairing, motor_lag=130): 15% enc_slip arc pos_err ~31.6mm/
  h_err ~1.84deg, pivot ~0mm/0.24deg; -10% enc_scale arc pos_err ~29mm/
  h_err ~1.7deg, pivot near 0mm/<1deg -- the sim's always-live OTOS/EKF
  fusion (`SimOdometer::connected()` is unconditionally `true`, unlike the
  real bench's I2C-flip-flop-gated OTOS) pulls the control loop's own
  state estimate back toward ground truth even though the REPORTED
  encoder is corrupted, and this test proves that correction survives a
  full tracked segment, not just a raw drive (`test_otos_fusion_live.py`'s
  own scope).
- `test_stiction_terminal_walk_in_no_reversal_no_premature_done` +
  `test_stiction_terminal_walk_in_no_reversal_fresh_run`: a forward
  (zero-curvature) segment, mirroring ticket 005's own
  `scenarioTerminalWalkInBandsNeverNegative` no-reversal regression
  (`drive_policy_harness.cpp`) through the full adapter+plant stack.
  Checked via MEASURED velocity (`sim.vel()`, floor=15mm/s -- identical
  shape to `test_bare_loop_move_and_tlm.py`'s own established
  `_run_and_check_no_reverse_creep()`), deliberately NOT raw PWM/duty:
  direct experiment against this exact pairing showed `sim.pwm()` dipping
  a few percent negative near settling under stiction (the low-level
  velocity PID legitimately braking a still-coasting wheel toward a
  lower/zero target) while measured velocity never once reversed -- a
  duty-level check would have false-positived on ordinary terminal
  braking chatter the reversal-dwell armor already absorbs, not the
  wedge-hazard "wheel setpoint flips sign" this AC bullet is actually
  about. "Not premature": `idle_at` (first `sim.active()==False`) is
  bounded below (>1.0s, vs. an instant/degenerate completion) AND the
  true-pose sampled at that exact tick is within tolerance of the goal
  (measured ~11-26mm across stiction magnitudes).
- `test_trackwidth_error_cross_gain_bounds_radius_error`: same-test
  baseline-vs-mismatch comparison (a second, independently-scoped `Sim()`
  instance for the zero-mismatch control run). Measured: baseline (128mm,
  matches firmware config) pos_err ~24.7mm; trackwidth in {98, 108, 148,
  158}mm (up to +/-23% mismatch) all land ~22-27mm -- essentially FLAT
  despite the mismatch, which is exactly what a working cross-gain (k_c)
  correction looks like (an uncorrected mismatch of this size would scale
  the radius error with the trackwidth delta).
- `test_infeasible_ask_under_fault_conditions_typed_err_queue_untouched`:
  `Drive::Verdict::EXIT_UNREACHABLE` (50mm arc, 400mm/s exit speed --
  `drive_admission_harness.cpp`'s own `scenarioExitUnreachable()` numbers)
  sent with `enc_slip`/`stiction`/`trackwidth` all simultaneously active
  -> `ERR_RANGE`, `field=1`, `bb.chainTail`/`segmentIn` unchanged, AND a
  subsequent feasible segment is still admitted and drives (queue
  genuinely still usable, not just superficially untouched). Companion to
  `test_binary_channel.py`'s own pristine-zero-fault admission-rejection
  test -- this one's point is specifically that active fault knobs never
  turn a clean rejection into a hang or a silent wrong answer.

**Mandatory motor_lag / 2026-07-11 false-green cross-reference**: every
tracker/replan scenario in this file runs with 120-140ms actuation lag
(`_MANDATORY_MOTOR_LAG = 130.0`), applied by every `_apply_faults()` call
unless explicitly overridden (no test in this file overrides it). This is
the direct fix for the 2026-07-11 false-green failure class referenced by
this ticket's own AC: a v2 validation pass that ran exclusively at the
sim's zero-lag default went green in sim while the same behavior
false-tripped on real hardware (a related same-day sim-plant-gain-
calibration fix is documented in `test_bare_loop_move_and_tlm.py`'s own
"2026-07-11" MOVE-200 note). No scenario in this matrix reproduces that
failure class: `test_every_scenario_runs_with_hardware_realistic_motor_lag`
mechanically, grep/review-verifiably enforces it by parsing this file's
own source (excluding its own body, to sidestep a self-reference trap in
its docstring/example text) and failing if `set_motor_lag()` is ever
called from anywhere other than `_apply_faults()`'s own single call site,
or if `_MANDATORY_MOTOR_LAG`/any explicit override ever falls outside
[120, 140]. The sim's zero-lag DEFAULT path is reserved for golden-TLM
bit-exactness comparison only (a determinism concern, orthogonal to this
file's control-accuracy-under-fault concern) and is never exercised by
this file at all.

**No real `source/drive/`/adapter defect surfaced.** Every fault
combination tried (including several exploratory magnitudes beyond what
shipped in the final assertions -- stiction 10-60, trackwidth +/-30mm,
symmetric and asymmetric enc_slip) converged cleanly, admitted/rejected
correctly, and never reversed a measured wheel velocity. No exception was
thrown.

### Verify

- `uv run python -m pytest tests/sim/unit/test_fault_knob_matrix.py -q`,
  run 4x back to back: 19 passed every time (no flakiness observed).
- Full suite, BLOCKING (`uv run python -m pytest -q`): **1486 passed, 2
  skipped, 4 xfailed, 1 xpassed, ZERO failures** (baseline, confirmed by
  re-running with this ticket's new file removed: 1467 passed + this
  ticket's 19 new tests = 1486; the 2 skipped/4 xfailed/1 xpassed are
  pre-existing and untouched by this ticket).

### Files changed

- `tests/sim/unit/test_fault_knob_matrix.py` (new, 19 tests)
- This ticket file (acceptance criteria, status, completion notes)
