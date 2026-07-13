---
id: '004'
title: Tracker + IK/saturate/clamp cascade
status: done
use-cases: [SUC-004]
depends-on: ['003']
github-issue: ''
issue: motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Tracker + IK/saturate/clamp cascade

## Description

Implement `source/drive/tracker.{h,cpp}`: the pure per-tick control law
that converts one tick's reference-vs-measured error into a wheel-velocity
command. This is responsibility 4 from `architecture-update.md` Step 2 —
"how a reference-vs-measured error becomes a wheel-velocity command" —
deliberately separated from responsibility 5 (policy/terminal decision,
ticket 005): this ticket answers HOW, never WHEN or WHAT STATUS.

Cascade order (fixed, per the issue): reference sample -> exact arc-frame
error projection -> P-only Kanayama trims (clamped) -> IK ->
curvature-preserving saturation -> one-sided forward-arc wheel clamp ->
wheel velocity setpoints.

## Acceptance Criteria

- [x] Error projection (`eAlong`/`eCross`/`eTheta`) uses EXACT arc-frame
      projection against the sampled `RefState` (never a linearized
      approximation).
- [x] Trim law implements the issue's Kanayama form exactly:
      `v_cmd = v_ref + clamp(k_s*e_along, ±trimVMax)`;
      `omega_cmd = omega_ref + clamp(k_theta*e_theta + k_c*v_ref*e_cross,
      ±trimOmegaMax)`; pivot mode (`|v_ref| < minSpeed`): `v_cmd` is a
      LITERAL `0.0f` (not merely near-zero), `omega_cmd = omega_ref +
      k_theta*e_theta` (matches sprint 098's proven heading loop).
      **Sign reconciliation (see completion notes)**: implemented against
      the issue's own "errors reference−measured" convention (negated
      relative to `arc_math`'s locked `measured−reference` `ArcError`) —
      the literal `measured−reference` sign is provably unstable
      (verified analytically); the reconciled sign is what actually
      matches sprint 098's proven loop and produces a convergent
      controller (verified by the closed-loop test below).
- [x] No `k_d`/derivative term anywhere (`k_d = 0`, not shipped, per the
      issue's explicit "encoder omega-hat is stale staggered noise"
      rationale) — grep-verifiable absence of a derivative term in
      `tracker.{h,cpp}`.
- [x] No integral term in any trim (the P-only outer loop rule) —
      grep-verifiable absence of any accumulator/integrator field in
      `tracker.h`.
- [x] IK reuses `arc_math`/`types`' copied `BodyKinematics`-equivalent
      `inverse()`; saturation is curvature-preserving (ports
      `BodyKinematics::saturate()`'s existing "scale both wheels by the
      same factor so the faster wheel sits exactly at the ceiling"
      contract).
- [x] The one-sided forward-arc wheel clamp: on a forward arc, NEITHER
      wheel's commanded velocity is ever negative — this is STRUCTURAL,
      not tuned. A property/fuzz test asserts this across a wide range of
      trim/error inputs (including deliberately-saturating ones).
- [x] `trimSaturated` is reported `true` exactly when a trim was clamped
      — verified against a scenario deliberately constructed to saturate
      each of `trimVMax`/`trimOmegaMax` independently.
- [x] A closed-loop test (a minimal, ticket-scoped Python or C++ plant
      stub is acceptable if ticket 006's real plant model has not landed
      yet — document this explicitly as "superseded once ticket 006
      lands" in completion notes) shows the tracker's output converging
      an arc's and a pivot's tracked error toward zero given a
      converging plant.
- [x] `uv run python -m pytest` passes.

## Testing

- **Existing tests to run**: `uv run python -m pytest`; ticket 002's grep
  isolation test.
- **New tests to write**: trim-law clamp-behavior tests; pivot-mode
  literal-zero-`v` test; the one-sided-clamp structural property test;
  the `trimSaturated` exact-true-iff-clamped test; a minimal closed-loop
  convergence smoke test.
- **Verification command**: `uv run pytest`

## Implementation Plan

**Approach**: this is a pure function of `(RefState, measured BodyState/
WheelState, Limits)` -> `WheelVelocities` + diagnostics (`eAlong`/
`eCross`/`eTheta`/`vTrim`/`omegaTrim`/`trimSaturated`). No `StepState`,
no `Status` decision (that is ticket 005's job — do not let this ticket's
scope creep into deciding when to replan or stop). Keep the cascade in
one clearly-staged function matching the issue's own stated order.

**Files to create**:
- `source/drive/tracker.h`, `source/drive/tracker.cpp`
- `tests/sim/unit/drive_tracker_harness.cpp` + `test_drive_tracker.py`

**Testing plan**: unit harness covering the trim law's clamp behavior,
the pivot-mode rule, the structural one-sided-clamp guarantee (property/
fuzz test), and `trimSaturated`'s exact-true-iff-clamped behavior.

**Documentation updates**: none.

## Completion Notes

- **Sign convention reconciliation (load-bearing finding).** The issue's
  own "Control laws and numbers" section headlines the trim law "errors
  reference−measured" — the OPPOSITE sign from `arc_math.h`'s already
  committed, test-locked `ArcError` convention (`measured − reference`,
  ticket 100-002). Applying the trim-law formulas literally to
  `arc_math`'s sign is not a style difference: linearizing the closed
  loop around a straight reference (`d(eTheta)/dt = omegaCmd`,
  `d(eCross)/dt = v_ref*eTheta`) gives a system whose trace equals
  `+trackKTheta` for any positive gain fed `arc_math`'s raw `eTheta` —
  unconditionally unstable, independent of `k_c`'s sign. Sprint 098's
  actual, hardware-proven heading loop
  (`motion/segment_executor.cpp`: `omega = desired.velocity +
  heading_kp * (desired.position - thetaMeasured) + ...`) computes its
  proportional term on `(reference − measured)`, confirming the issue's
  prose, not the literal symbol names taken at `arc_math`'s sign, is the
  intended contract. `track()` negates `arc_math`'s `projectOntoPose()`
  result before applying the trim law; `TrackerOutput.eAlong/eCross/
  eTheta` still report `arc_math`'s native `(measured − reference)`
  convention unchanged (what `TrackRecord` expects). See `tracker.h`'s
  class comment ("Reconciled sign convention") and `tracker.cpp`'s
  inline comments for the full derivation. The closed-loop convergence
  test empirically confirms the reconciled sign converges (both arc and
  pivot); the literal, unreconciled sign was verified (during
  development) to diverge.
- **`arc_math.{h,cpp}` extended, not changed.** `tracker.cpp` needs the
  exact tangent/normal projection against an ALREADY-sampled reference
  pose (`RefState.x/y/theta`), not a fresh `(anchor, kappa, s)` triple —
  `track()`'s own inputs have no anchor/kappa. Added
  `Drive::projectOntoPose(reference, measured)` to `arc_math.{h,cpp}` and
  refactored `projectOntoArc()` to call it internally (bit-identical
  behavior; ticket 100-002's own arc_math tests are unaffected — this is
  additive, no existing signature changed).
- **`types.h`'s `Limits` extended.** Added `trackKS`/`trackKTheta`/
  `trackKCross`/`minSpeed` (the `PlannerConfig.track_k_s/track_k_theta/
  track_k_cross/min_speed` wire fields this ticket's cascade consumes) —
  anticipated explicitly by ticket 100-003's own doc comment ("land with
  tickets 004/005, the modules that actually consume them"). Also closed
  the loop on the pre-existing `trimOmegaMax` doc comment's open question
  about the issue table's "1.0 arc / 2.0 pivot" split: as transcribed,
  the pivot formula carries NO clamp at all (matches 098), so the
  pivot-specific 2.0 rad/s table value is not wired to any clamp by this
  ticket.
- **`WheelState` not consumed by `track()`.** The ticket's own cascade
  overview lists `(RefState, measured BodyState/WheelState, Limits)` as
  the wider step()-level inputs, but the trim law as specified (P-only,
  no `k_d`) never reads wheel velocity — only `measured.pose` is used.
  `track()`'s signature is `(RefState, BodyState, Limits, trackwidth)`;
  `WheelState` remains part of `StepInput` for ticket 005's own
  (non-tracker) purposes.
- **Closed-loop plant stub is ticket-scoped**, per this ticket's own
  allowance — a minimal first-order-lag/Euler-integrate stub
  (`drive_tracker_harness.cpp`'s `PlantState`/`stepPlant`), superseded
  once ticket 100-006's real plant model lands. The arc-mode gain set
  (`k_c = 1.5e-5`) is deliberately overdamped/slow (issue's own
  `zeta >= 1.3` claim, confirmed by the linearization above: poles at
  ~-0.1/s and ~-5.9/s at `v_ref = 200mm/s`), so the arc closed-loop test
  runs 20s of simulated time and asserts a 5x error reduction, not a
  fast settle — this is a stability/convergence smoke test, not a tuned
  settling-time gate.
