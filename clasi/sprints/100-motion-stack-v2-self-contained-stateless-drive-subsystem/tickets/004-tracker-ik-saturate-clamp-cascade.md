---
id: '004'
title: Tracker + IK/saturate/clamp cascade
status: open
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

- [ ] Error projection (`eAlong`/`eCross`/`eTheta`) uses EXACT arc-frame
      projection against the sampled `RefState` (never a linearized
      approximation).
- [ ] Trim law implements the issue's Kanayama form exactly:
      `v_cmd = v_ref + clamp(k_s*e_along, ±trimVMax)`;
      `omega_cmd = omega_ref + clamp(k_theta*e_theta + k_c*v_ref*e_cross,
      ±trimOmegaMax)`; pivot mode (`|v_ref| < minSpeed`): `v_cmd` is a
      LITERAL `0.0f` (not merely near-zero), `omega_cmd = omega_ref +
      k_theta*e_theta` (matches sprint 098's proven heading loop).
- [ ] No `k_d`/derivative term anywhere (`k_d = 0`, not shipped, per the
      issue's explicit "encoder omega-hat is stale staggered noise"
      rationale) — grep-verifiable absence of a derivative term in
      `tracker.{h,cpp}`.
- [ ] No integral term in any trim (the P-only outer loop rule) —
      grep-verifiable absence of any accumulator/integrator field in
      `tracker.h`.
- [ ] IK reuses `arc_math`/`types`' copied `BodyKinematics`-equivalent
      `inverse()`; saturation is curvature-preserving (ports
      `BodyKinematics::saturate()`'s existing "scale both wheels by the
      same factor so the faster wheel sits exactly at the ceiling"
      contract).
- [ ] The one-sided forward-arc wheel clamp: on a forward arc, NEITHER
      wheel's commanded velocity is ever negative — this is STRUCTURAL,
      not tuned. A property/fuzz test asserts this across a wide range of
      trim/error inputs (including deliberately-saturating ones).
- [ ] `trimSaturated` is reported `true` exactly when a trim was clamped
      — verified against a scenario deliberately constructed to saturate
      each of `trimVMax`/`trimOmegaMax` independently.
- [ ] A closed-loop test (a minimal, ticket-scoped Python or C++ plant
      stub is acceptable if ticket 006's real plant model has not landed
      yet — document this explicitly as "superseded once ticket 006
      lands" in completion notes) shows the tracker's output converging
      an arc's and a pivot's tracked error toward zero given a
      converging plant.
- [ ] `uv run python -m pytest` passes.

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
