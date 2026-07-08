---
id: '002'
title: Odometer owns reset translation and per-pass fusability
status: in-progress
use-cases:
- SUC-002
depends-on:
- '001'
github-issue: ''
issue: odometer-owns-reset-and-fusability.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Odometer owns reset translation and per-pass fusability

## Description

Move the `SetPose → Pose2D → OdometerCommand` translation
(`main_loop.cpp` ~178-188) and the `odometerResetThisPass` fusability
decision (~172-195, ~202) from `MainLoop::tick()` into `Hal::Odometer` as
`applySetPose(const msg::SetPose&)` and `fusableThisPass()`. Depends on
ticket 001 only by file-serialization (both touch `main_loop.cpp`), not by
logic.

**This is the sprint's one load-bearing ticket.** It MUST preserve the
live-debugged fix that skips OTOS fusion for exactly the one pass a reset
is applied (`OI`/`OZ`/`OR`/`OV`/`SI`), or the EKF fabricates a false
innovation against a stale, pre-reset OTOS reading (reproduced live via SI:
`fusedPose()` dragged back toward the pre-reset reading while
`encoderPose()` landed correctly — see `main_loop.cpp`'s own comment at the
current `odometerResetThisPass` block, and architecture-update.md
Decision 2). The loop still drains `bb.otosCommandIn`/`bb.otosSetPoseIn`
and applies them to the odometer (unchanged — the loop legitimately knows
a reset happened because it just applied one); only the translation logic
and the "is this pass fusable" decision move.

`fusableThisPass()` is a **read-and-clear, single-caller query** — it
mirrors this codebase's existing `hasEvent()/takeEvent()` one-shot-signal
convention rather than relying on `Hal::Odometer::tick()` (pure virtual,
no shared base body) to clear a flag on every leaf's behalf. Document this
single-caller contract explicitly in the method's own doc comment — do not
leave it implicit.

## Acceptance Criteria

- [ ] `Hal::Odometer` gains `void applySetPose(const msg::SetPose& pose)` —
      a concrete method (built on the existing `setPose()` primitive,
      mirroring how `apply()`/`configure()` are already built on
      primitives) performing the `SetPose → Pose2D → OdometerCommand`
      translation currently inlined at `main_loop.cpp` ~178-188.
- [ ] `Hal::Odometer` gains `virtual bool fusableThisPass()` — documented
      explicitly as a READ-AND-CLEAR, single-caller query (called at most
      once per pass, by the loop's own `poseEstimator_.tick()` gate;
      calling it twice in the same pass is NOT supported and the doc
      comment must say so). Returns `false` for exactly the one call
      immediately following a reset applied via `applySetPose()`/
      `apply(const msg::OdometerCommand&)` THIS pass (covering all four
      reset actions — `INIT`/`ZERO`/`RESET_TRACKING`/`SET_POSE` — not only
      `SET_POSE`); `true` otherwise.
- [ ] `main_loop.cpp`'s loop-local `odometerResetThisPass` bool is removed;
      the `poseEstimator_.tick()` call site's gate becomes
      `(bb.otosValid && odometer->fusableThisPass()) ? &bb.otos : nullptr`.
      `bb.otosValid`'s own computation is UNCHANGED by this ticket (still
      the existing `odometer != nullptr` check — ticket 003 folds
      `fusableThisPass()` into it).
- [ ] `Hal::SimOdometer` and `Hal::OtosOdometer` both correctly participate
      in the new contract (inherit the concrete base behavior; verify at
      implementation time that neither leaf's own `apply()`/`setPose()`
      override bypasses the base's reset-flag bookkeeping).
- [ ] **Load-bearing**: the SI/OZ/OR/OV regression tests in `tests/sim` are
      RUN and shown green BOTH immediately before this change (baseline)
      and immediately after (not inferred from reading the diff) — record
      both results in the ticket's own completion notes.
- [ ] `encoderPose()`/`fusedPose()` values after an SI/OZ/OR/OV sequence are
      bit-for-bit identical to pre-ticket behavior for the same test
      script.
- [ ] `uv run python -m pytest tests/sim` is green overall.

## Implementation Plan

**Approach**:
1. Add a private/protected flag (e.g. `resetAppliedThisPass_`) to
   `Hal::Odometer`, set `true` by `applySetPose()` and by
   `apply(const msg::OdometerCommand&)`'s existing dispatch (covering
   `INIT`/`ZERO`/`RESET_TRACKING`/`SET_POSE` — all four currently
   participate in `odometerResetThisPass` via `bb.otosCommandIn`, so all
   four must set the flag, not only `SET_POSE`).
2. Add `virtual bool fusableThisPass()` with a base implementation that
   reads the flag, clears it, and returns its negation. Do NOT rely on
   `tick()` to clear it (pure virtual, no shared base body — every leaf
   would have to remember to cooperate, a silent-breakage footgun).
3. Add `void applySetPose(const msg::SetPose& pose)`: build the
   `Pose2D`/`OdometerCommand` exactly as `main_loop.cpp` does today, then
   call `apply(cmd)` (reusing existing dispatch) rather than duplicating
   the reset-flag-setting logic.
4. Update `main_loop.cpp`: replace the inline `SetPose → Pose2D →
   OdometerCommand` construction with `odometer->applySetPose(pose)`;
   replace the loop-local `odometerResetThisPass` bool with
   `odometer->fusableThisPass()` read at the exact same call site
   (`poseEstimator_.tick()`'s gate), preserving the same evaluation order
   relative to the reset-drain code above it.
5. Do NOT touch `bb.otosValid`'s own computation in this ticket.

**Files to modify**: `source/hal/capability/odometer.h`,
`source/hal/sim/sim_odometer.{h,cpp}`,
`source/hal/otos/otos_odometer.{h,cpp}`, `source/runtime/main_loop.cpp`.

**Documentation updates**: none beyond the new methods' own doc comments
(the single-caller/read-once contract for `fusableThisPass()` MUST be
documented there, not only in this ticket).

## Testing

- **Existing tests to run**: the full `tests/sim` suite, with explicit
  attention to whichever test files exercise SI/OZ/OR/OV (grep
  `tests/sim/unit/` for `otosSetPoseIn`/`otosCommandIn`/`SI`/`OZ`/`OR`/`OV`
  — e.g. `dev_loop_pose_estimator_harness.cpp`/
  `test_pose_estimate_tolerance.py` and any `otos_commands` test). Run
  once BEFORE the change (baseline) and once AFTER — both green, not just
  "green after."
- **New tests to write**: none required if existing coverage already
  exercises the one-pass-skip window; if it does not, add one that stages
  a reset then asserts the very next pass's fusion gate is `false`, and
  the pass after that is `true` again.
- **Verification command**: `uv run python -m pytest tests/sim`
