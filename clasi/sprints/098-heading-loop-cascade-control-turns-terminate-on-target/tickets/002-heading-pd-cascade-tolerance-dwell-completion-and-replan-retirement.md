---
id: '002'
title: Heading PD cascade, tolerance/dwell completion, and replan retirement
status: done
use-cases:
- SUC-001
- SUC-002
depends-on:
- '001'
github-issue: ''
issue:
- heading-loop-cascade-control-turns-terminate-on-target.md
- real-robot-motion-calibration-undershoot.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Heading PD cascade, tolerance/dwell completion, and replan retirement

## Description

The sprint's core behavioral change. Implement the outer heading PD cascade
for PRE_PIVOT/TERMINAL_PIVOT in `Motion::SegmentExecutor`
(`segment_executor.cpp`), replace their `STOP_ROTATION`-arc-threshold +
ride-the-tail completion with a tolerance+dwell gate, and retire
`maybeReplanPivot()`'s sub-gross EXTEND branch to a no-op for these two
phases (the gross-divergence reanchor branch stays live as stall
protection). TRANSLATE and BLEND are untouched — this ticket is scoped to
the rotational channel's PRE_PIVOT/TERMINAL_PIVOT phases only.

Reference: `architecture-update.md` M3/M4/M5, Decision 1 (why this lands
inline in the existing class, not a new one), Decision 3 (tolerance/dwell
as file-local constants), Open Question 2 (dwell-vs-STOP_TIME budget —
this ticket resolves it as a concrete assertion, item below).

Depends on 001 — needs `config_.heading_kp`/`config_.heading_kd` to read.

## Acceptance Criteria

**The cascade**

- [x] Each tick, for PRE_PIVOT/TERMINAL_PIVOT only (NOT BLEND, NOT
      TRANSLATE): sample desired `(theta_desired, omega_desired)` from
      `rotational_` at `rotationalElapsed(now)`; derive `theta_measured` as
      the encoder-differential heading relative to the phase's OWN
      baseline — `((encRight.position.val - encLeft.position.val) -
      baseline_.encDiff0) / trackwidth_` — matching the same
      relative-to-phase-start convention `baseline_.encDiff0` already
      establishes for the existing divergence-replan math; derive
      `omega_measured` as `(encRight.velocity.val - encLeft.velocity.val) /
      trackwidth_`, falling back to the plan-sampled `omega_desired` when
      either wheel's `velocity.has` is false (mirrors
      `maybeReplanPivot()`'s existing reanchor-seed fallback exactly).
- [x] Commanded `omega = omega_desired + config_.heading_kp *
      (theta_desired - theta_measured) + config_.heading_kd *
      (omega_desired - omega_measured)`, replacing the raw
      `rotational_.sample(...).velocity` currently returned for these two
      phases.

**Tolerance/dwell completion**

- [x] New file-local `constexpr` constants in `segment_executor.cpp`:
      `kHeadingTol = 0.00873f` (~0.5°, `[rad]`), `kHeadingRateTol =
      0.0175f` (~1°/s, `[rad/s]`), `kHeadingDwellMs = 150` (150 ms — within
      the issue's suggested 100-200 ms range), documented with the same
      style of derivation comment as the existing `kDivergenceThreshold`
      family — labeled explicitly as a first-cut, code-edit-iterable
      constant (architecture-update.md Decision 3), NOT a `PlannerConfig`
      field.
- [x] Completion for PRE_PIVOT/TERMINAL_PIVOT: `|rotationalTarget_ -
      theta_measured| < kHeadingTol` AND `|omega_measured| <
      kHeadingRateTol`, held continuously for `>= kHeadingDwellMs`,
      REPLACES `STOP_ROTATION`'s role for these two phases —
      `STOP_ROTATION` is no longer appended to `stops_[]` in
      `beginPrePivot()`/`beginTerminalPivot()`. `STOP_TIME` stays appended,
      unchanged, as the independent stall/non-convergence backstop.
- [x] The dwell timer resets whenever the AND condition goes false, and is
      (re)initialized at phase start (`beginPrePivot()`/
      `beginTerminalPivot()`).
- [x] Once the gate fires, the phase completes through the SAME
      `stopping_`/`advancePhase()` machinery already in place (no second
      completion pathway) — verified by the no-reverse-creep item below.
- [x] **Dwell-vs-STOP_TIME budget** (Open Question 2): a sim assertion
      proves the added dwell (≤200 ms) does not cause the `STOP_TIME`
      safety net's own nominal-duration budget (`beginPrePivot()`/
      `beginTerminalPivot()`'s `nominal * 2.0f + 2000.0f` formula) to be
      exhausted before the tolerance+dwell gate can fire, for a
      representative SLOW (low-ceiling) turn.

**Replan retirement (SUC-002)**

- [x] `maybeReplanPivot()`'s sub-gross (`kRotDivergenceThreshold`,
      EXTEND-only) branch becomes a no-op for PRE_PIVOT/TERMINAL_PIVOT
      specifically. The gross-divergence (`kRotGrossDivergenceThreshold`,
      reanchor) branch is UNCHANGED and still live for these phases.
      BLEND's own replan suppression (already disabled via
      `phaseReplanDeadline_ == chain instant`) is untouched.
      `maybeReplanTranslate()` (linear channel) is untouched.

**Sim acceptance — `tests/sim/unit/segment_executor_harness.cpp`**

- [x] New scenario: the PD correction term is nonzero and in the
      correcting direction when a deliberately-lagging or
      deliberately-leading plant variant is driven against the executor
      (proves the loop is actually closing, not a no-op).
- [x] New scenario: tolerance+dwell completion does NOT fire prematurely
      mid-cruise (while `|target error|` or `|rate|` still exceeds
      tolerance) and DOES fire once both conditions hold for the dwell
      window.
- [x] New scenario (SUC-002, dwell-vs-budget): the dwell-vs-STOP_TIME
      budget assertion itemized above, for a representative slow turn.
- [x] New scenario (SUC-002, stall-protection): holding one wheel's
      encoder reading artificially fixed (a simulated stall) against a
      nonzero PRE_PIVOT or TERMINAL_PIVOT command proves the
      gross-divergence reanchor STILL FIRES within the same ~2-pass budget
      as today.
- [x] New scenario (SUC-002, replan retirement): under NOMINAL tracking lag
      (the kind the pre-sprint code's sub-gross EXTEND branch WOULD have
      fired on) proves that branch no longer fires for PRE_PIVOT/
      TERMINAL_PIVOT post-sprint.
- [x] The EXISTING `scenarioNoReverseCreepInTerminalDecelTrace` regression
      scenario (094-001's named regression gate) is re-run UNMODIFIED and
      stays green — the literal-`0.0f` snap on rotational convergence and
      the "sampled omega never changes sign" invariant both still hold
      with the PD cascade live.
- [x] Every other existing `segment_executor_harness.cpp` scenario
      (straight segment, translate-then-terminal-pivot, pure in-place
      turn, auto-decel-stays-idle, stop-mid-TRANSLATE) stays green;
      tolerances re-verified — note any tolerance changes explicitly in
      this ticket's completion notes (the old dead-time-projected-firing
      widened tolerances may no longer apply to PRE_PIVOT/TERMINAL_PIVOT
      now that they use the tolerance+dwell gate instead of that firing
      path).
- [x] TRANSLATE-phase behavior (`maybeReplanTranslate()`, `STOP_DISTANCE`
      completion) is provably untouched — the existing TRANSLATE scenarios
      pass unmodified, no new coverage needed.
- [x] Full `uv run python -m pytest` stays green, no regression from the
      pre-ticket baseline (the sim plant has ~no tracking asymmetry, so
      this must be a no-op-to-improvement per architecture-update.md's own
      Impact note — a sim regression here means the gains/constants need
      adjustment, not that the mechanism is wrong).
- [x] `just build-sim` and `just build-clean` both succeed.

## Testing

- **Existing tests to run**: full `uv run python -m pytest`; explicit
  focus on `tests/sim/unit/segment_executor_harness.cpp`'s existing 6
  scenarios.
- **New tests to write**: the 6 new scenarios itemized above (PD
  correction direction, no-premature-completion, dwell-vs-STOP_TIME
  budget, stall-still-fires, nominal-lag-no-longer-fires, plus the
  no-reverse-creep re-confirmation of the existing scenario).
- **Verification command**: `uv run python -m pytest`.

## Implementation Plan

**Approach**: Surgical edit inside `segment_executor.cpp`'s existing
rotational tick branch (the `else` branch of `tick()`, currently shared by
PRE_PIVOT/TERMINAL_PIVOT) and `maybeReplanPivot()`; no new files, no new
classes (architecture-update.md Decision 1). Factor the measured-heading/
measured-rate derivation into a small private helper if useful for both the
PD term and the completion gate (both need `theta_measured`) — implementer's
judgment on exact factoring; the acceptance criteria constrain BEHAVIOR,
not internal shape.

**Files to modify**: `source/motion/segment_executor.h` (new dwell-timer
member, new tolerance/dwell constants), `source/motion/segment_executor.cpp`
(the PD law, the completion gate, `maybeReplanPivot()`'s narrowed scope,
`beginPrePivot()`/`beginTerminalPivot()`'s stop-set no longer appending
`STOP_ROTATION`), `tests/sim/unit/segment_executor_harness.cpp` (new
scenarios).

**Files to create**: none.

**Testing plan**: as above — sim-only ticket, no firmware/hardware
verification (that is ticket 003's job).

**Documentation updates**: none beyond in-code comments —
`architecture-update.md` already documents the design; this ticket
implements it, matching this file's own existing convention of carrying
the "why" in doc comments (as it already does for the divergence replan/
dead-time/graceful-decel machinery this ticket edits).

## Completion Notes

- **Sign convention (verified, not assumed)**: `theta_measured` is
  computed with NO `omegaSign` multiplication — `((encRight.position.val -
  encLeft.position.val) - baseline_.encDiff0) / trackwidth_` — deliberately
  differing from `rotationProgress()`'s `STOP_ROTATION` geometry (which
  DOES multiply by `omegaSign`). Verified algebraically: at
  `rotationProgress()`'s own FIRED boundary (`signedArc >= cond.a`), for
  BOTH signs of `omegaSign`, `diff / trackwidth_ == rotationalTarget_`
  exactly — so `theta_measured` (no `omegaSign` factor) lands in the SAME
  signed frame as `rotationalTarget_` and `rotational_.sample().position`
  (0 at phase start, growing toward `rotationalTarget_`, signed like the
  target itself). This is documented in `measuredHeading()`'s doc comment
  in `segment_executor.cpp`. Re-multiplying by `omegaSign` would have been
  the natural-looking but WRONG mirror of the divergence-replan code and
  would have made the P-term diverge instead of converge for
  negative-direction turns — caught by hand-deriving the boundary case
  before writing any code, not by trial and error.
- **`maybeReplanPivot()`'s `stops_[]` dependency**: M4 removing
  `STOP_ROTATION` from `stops_[]` for PRE_PIVOT/TERMINAL_PIVOT silently
  would have disabled `maybeReplanPivot()` ENTIRELY (its `rotCond` lookup
  scans `stops_[]` for a `STOP_ROTATION` entry and returns early if none is
  found) — including the gross-divergence stall-protection branch M5 says
  must stay live. Fixed by reconstructing the same `StopCondition` locally
  (`fabsf(rotationalTarget_) * arcScale_`, exactly what `beginPrePivot()`/
  `beginTerminalPivot()` used to append) instead of scanning `stops_[]`.
  Caught by tracing the call graph before editing, confirmed by the
  stall-protection sim scenario passing.
- **PD gated to `!stopping_`**: the outer heading PD cascade is applied
  only while the phase is actively converging (`!stopping_`); once the
  terminal graceful decel-to-zero is armed (`stopping_ == true`, via
  either `stop()` or the `STOP_TIME` backstop's `promptHalt` path), the
  phase rides that decel's own tail UNCORRECTED, exactly like before this
  ticket. This is an implementer's-judgment reading of the architecture's
  own stated cascade boundary ("regulates PRE_PIVOT/TERMINAL_PIVOT's
  commanded angular rate... instead of playing the Ruckig plan's own
  velocity sample straight through") applied to the sub-state the ticket
  text does not explicitly disambiguate. Justification: `maybeReplanPivot()`
  itself already carries the identical `!stopping_` guard for the same
  reason; and empirically (see below), a live P-term correcting residual
  error DURING the terminal decel is exactly the mechanism that produces a
  slow, small-magnitude, sustained commanded reversal — the architecture's
  own Risks section names this precise failure mode as the sprint's
  highest-priority hazard. Gating to `!stopping_` keeps the REQUIRED
  `scenarioNoReverseCreepInTerminalDecelTrace` regression's guarantee
  completely intact and un-diluted by the new PD path (that scenario also
  uses `Kp=Kd=0` so it would have passed either way — the gating choice is
  belt-and-suspenders for any FUTURE bench-tuned nonzero-gain scenario that
  hits `stop()` or the `STOP_TIME` backstop mid-turn).
- **Real finding — a live P/D-loop CAN command a small, slow, sustained
  reversal near final convergence even in sim, and it is caused by
  overshoot, not by the `stopping_` tail**: while building the
  dwell-vs-STOP_TIME-budget scenario with representative gains
  (`heading_kp=2.0`, `heading_kd=0.3`) against a REAL (non-stalled,
  non-lagging) plant, the FIRST version of that scenario failed on
  `everReversedOmega`. Traced with temporary instrumentation (not
  committed): the D-term, evaluated against `tests/sim/unit/
  segment_executor_harness.cpp`'s EXISTING `PlantState` helper — which sets
  `MotorState.velocity.has = true` but never populates `.velocity.val`
  (always reads back `0.0f`) — degenerates into a CONSTANT `kd *
  omega_desired` feedforward boost every tick (since `omega_measured` is
  always the fabricated `0.0f`, never the real rate), overdriving the
  plant ~30% past the plan's own velocity ceiling. That overdrive
  accumulates a genuine ~1-2° overshoot PAST the target (not a numerical
  residual), and because the overshoot is LARGER than `kHeadingTol`
  (0.00873 rad), the tolerance+dwell gate cannot fire immediately — the
  phase has to wait out a slow (~0.5s time-constant) pure-P walk-back from
  ABOVE the target, commanding a small (~0.02-0.04 rad/s peak, i.e.
  ~3mm/s per wheel), 1+ second SUSTAINED negative `omega` the entire time.
  This is a TEST-FIXTURE artifact (confirmed: the SAME config against a
  NEW `VelocityAwarePlant` helper added this ticket, which DOES report
  real `.velocity.val`, converges cleanly with zero reversal — see
  `scenarioDwellDoesNotExhaustStopTimeBudget`'s comment). No production
  code changed as a result. Flagging prominently for ticket 003 (hardware
  acceptance) anyway: (1) confirm real `Hal::Motor`/`MotorState` velocity
  reporting is never `has=true` with a stale/zero `.val` (the exact
  precondition that produced this artifact) — if it ever is, this same
  overdrive mechanism would reproduce on the real robot; (2) even with
  honest velocity feedback, ANY nonzero `heading_kp` correcting a REAL
  overshoot will, by definition, briefly command the opposite sign near
  final convergence — ticket 003's own "zero commanded reversal beyond the
  armor's own window" bench check is exactly the right instrument to
  quantify whether this is negligible (sub-deadband, as the magnitude
  above suggests) or needs damping/tuning attention before playfield use.
- **No existing scenario tolerances were changed.** All 6 pre-098-002
  scenarios (`scenarioStraightSegmentSkipsBothPivots`,
  `scenarioTranslateThenTerminalPivot`,
  `scenarioPureInPlaceTurnSkipsTranslate`,
  `scenarioAutoDecelToZeroOnceConvergedStaysIdle`,
  `scenarioStopMidTranslateAbandonsRemainingPhases`,
  `scenarioNoReverseCreepInTerminalDecelTrace`) pass byte-for-byte
  unmodified — none needed a tolerance adjustment. This is expected: the
  three straight/TRANSLATE-only scenarios never touch the rotational
  tolerance/dwell path at all, and the three PRE_PIVOT-involving scenarios
  all run with `heading_kp = heading_kd = 0.0f` (`generousConfig()`'s
  default, untouched by this ticket), which degenerates M3's cascade to
  exactly the old open-loop passthrough and leaves M4's tolerance+dwell
  gate to fire the moment the (already dead-time-projected-accurate) plant
  lands at/near target — no old dead-time-projected-firing widening was
  ever depended on by these six scenarios' own asserted tolerances.
- **`uv run python -m pytest` count**: baseline is 1280 passed, 5 xfailed
  (measured both pre-ticket and, when the environment cooperates,
  post-ticket — see below). `tests/sim/unit/segment_executor_harness.cpp`
  is compiled and run as ONE pytest test
  (`test_segment_executor_harness_compiles_and_passes`) regardless of how
  many scenarios its `main()` calls internally, so this ticket's 5 new
  scenarios do not change the pytest-level PASS COUNT — they enrich what
  that one test proves (6 → 11 internal scenarios, all green). Verified via
  `uv run python -m pytest tests/sim tests/unit -q` (896 passed, the
  ticket's own deterministic gate per `tests/CLAUDE.md`) and, separately,
  `uv run python -m pytest -q --ignore=tests/testgui` (1280 passed, 5
  xfailed, matching the stated baseline exactly).
- **Pre-existing, UNRELATED flakiness discovered in `tests/testgui/`**:
  running the bare `uv run python -m pytest -q` (which collects
  `tests/testgui/` per `pyproject.toml`'s `testpaths`) intermittently
  crashes the whole process with a native `Bus error`/`Segmentation fault`
  inside `host/robot_radio/testgui/transport.py`'s background
  `_tick_loop` thread — a lingering Qt/thread-teardown race, not a Python
  exception. CONFIRMED unrelated to this ticket: reproduced identically
  (`git stash`, same crash, same class of fault) on the clean pre-098-002
  commit (`ed4fcba2`) with zero code changes; also does not reproduce when
  `tests/testgui/test_set_origin.py` (the module the crash trace pointed
  at) is run in isolation, with or without this ticket's changes — it is a
  full-suite-ordering/thread-lifecycle flake in the GUI test domain, not a
  segment_executor regression. Not fixed here (well outside this ticket's
  scope); worth a `clasi/issues/` entry for a future session so it does
  not re-waste investigation time.
