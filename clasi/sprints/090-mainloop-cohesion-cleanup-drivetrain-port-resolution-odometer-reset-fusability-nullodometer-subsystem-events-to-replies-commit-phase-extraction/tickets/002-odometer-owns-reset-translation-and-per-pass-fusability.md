---
id: '002'
title: Odometer owns reset translation and per-pass fusability
status: done
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

- [x] `Hal::Odometer` gains `void applySetPose(const msg::SetPose& pose)` —
      a concrete method (built on the existing `setPose()` primitive,
      mirroring how `apply()`/`configure()` are already built on
      primitives) performing the `SetPose → Pose2D → OdometerCommand`
      translation currently inlined at `main_loop.cpp` ~178-188.
- [x] `Hal::Odometer` gains `virtual bool fusableThisPass()` — documented
      explicitly as a READ-AND-CLEAR, single-caller query (called at most
      once per pass, by the loop's own `poseEstimator_.tick()` gate;
      calling it twice in the same pass is NOT supported and the doc
      comment must say so). Returns `false` for exactly the one call
      immediately following a reset applied via `applySetPose()`/
      `apply(const msg::OdometerCommand&)` THIS pass (covering all four
      reset actions — `INIT`/`ZERO`/`RESET_TRACKING`/`SET_POSE` — not only
      `SET_POSE`); `true` otherwise.
- [x] `main_loop.cpp`'s loop-local `odometerResetThisPass` bool is removed;
      the `poseEstimator_.tick()` call site's gate becomes
      `(bb.otosValid && odometer->fusableThisPass()) ? &bb.otos : nullptr`.
      `bb.otosValid`'s own computation is UNCHANGED by this ticket (still
      the existing `odometer != nullptr` check — ticket 003 folds
      `fusableThisPass()` into it).
- [x] `Hal::SimOdometer` and `Hal::OtosOdometer` both correctly participate
      in the new contract (inherit the concrete base behavior; verify at
      implementation time that neither leaf's own `apply()`/`setPose()`
      override bypasses the base's reset-flag bookkeeping).
- [x] **Load-bearing**: the SI/OZ/OR/OV regression tests in `tests/sim` are
      RUN and shown green BOTH immediately before this change (baseline)
      and immediately after (not inferred from reading the diff) — record
      both results in the ticket's own completion notes.
- [x] `encoderPose()`/`fusedPose()` values after an SI/OZ/OR/OV sequence are
      bit-for-bit identical to pre-ticket behavior for the same test
      script.
- [x] `uv run python -m pytest tests/sim` is green overall.

## Completion Notes

**Implementation.** `Hal::Odometer` (`source/hal/capability/odometer.h`)
gains: a private `resetAppliedThisPass_` flag, set `true` by every one of
`apply()`'s four action arms (`INIT`/`ZERO`/`RESET_TRACKING`/`SET_POSE`) —
bookkeeping lives in the base class's own non-virtual `apply()`, so it runs
regardless of what a leaf's `init()`/`resetTracking()`/`setPose()` override
does, and cannot be bypassed; `applySetPose(const msg::SetPose&)`, the
`SetPose → Pose2D → OdometerCommand` translation ported verbatim from
`main_loop.cpp`, dispatching through `apply()` (not calling `setPose()`
directly) so the flag-setting is not duplicated; and `virtual bool
fusableThisPass()`, a read-and-clear query documented explicitly as a
single-caller contract (mirrors `hasEvent()`/`takeEvent()`), returning
`!resetAppliedThisPass_` and clearing the flag on every call.

`main_loop.cpp`'s `MainLoop::tick()` no longer builds the `OdometerCommand`
by hand (`odometer->applySetPose(bb.otosSetPoseIn.take())` replaces the
inline translation) and no longer tracks a loop-local
`odometerResetThisPass` bool — the `poseEstimator_.tick()` gate is now
exactly `(bb.otosValid && odometer->fusableThisPass()) ? &bb.otos :
nullptr`, per the acceptance criterion. This is `fusableThisPass()`'s one
sanctioned call site. `bb.otosValid`'s own computation (the `odometer !=
nullptr` check at COMMIT) is untouched, as scoped — the short-circuit
`&&` is safe because `hardware_.odometer()` returns a fixed pointer for
the lifetime of the process (verified: `Hardware::odometer()` defaults to
`nullptr`, `NezhaHardware::odometer()` always returns `&otosOdometer_`,
`SimHardware::odometer()` always returns `&odometer_` — no leaf ever
flips between null/non-null across passes), so `bb.otosValid == true`
implies `odometer != nullptr` on every pass, not just the pass it was
committed on.

`Hal::SimOdometer`/`Hal::OtosOdometer` required NO code changes: neither
overrides `apply()` (not virtual — cannot be overridden) or
`fusableThisPass()`; both correctly inherit the base's concrete reset-flag
bookkeeping with no way to bypass it. Verified by grep
(`grep -n "apply(\|fusableThisPass" source/hal/sim/sim_odometer.h
source/hal/otos/otos_odometer.h` — no matches besides a doc-comment
reference) and by the SI/OZ/OR/OV tests below staying green against both
leaves (the `sim` pytest fixture exercises `Hal::SimOdometer`;
`Hal::OtosOdometer` has no host-reachable pytest fixture but shares the
identical base-class code path).

**Load-bearing regression proof (SI/OZ/OR/OV).**

Focused run — `tests/sim/unit/test_pose_commands.py`,
`tests/sim/unit/test_otos_commands.py`,
`tests/sim/unit/test_otos_commands_nodev.py`,
`tests/sim/unit/test_config_pose_set_otos_surface.py` (covers SI, OI/OZ/OR/OV):

- BEFORE (baseline, pre-change): `64 passed in 15.37s`
- AFTER (post-change): `64 passed in 15.89s`

The key load-bearing assertion is
`test_si_reanchors_both_encpose_and_the_fused_pose_exactly`
(`tests/sim/unit/test_pose_commands.py`), which asserts `pose=` (fused,
EKF-owned) reads back the EXACT SI value (`"1000,500,900"`), not a
partial drag toward it — this is only true if OTOS fusion is skipped for
the one pass SI's reset lands on. It passed identically before and after,
proving the one-pass-skip window is preserved bit-for-bit.

**Full suite.**

- BEFORE (baseline, pre-change): `308 passed, 2 xfailed in 99.81s (0:01:39)`
- AFTER (post-change): `308 passed, 2 xfailed in 95.12s (0:01:35)`

Identical pass/xfail counts before and after — no regressions, no new
tests needed (existing SI coverage already exercises the one-pass-skip
window per the ticket's own "New tests to write: none required" note).

**Deviations from the plan:** none. `Hal::SimOdometer`/`Hal::OtosOdometer`
files were listed as "files to modify" in the plan but needed no edits —
confirmed at implementation time (per the plan's own "verify... at
implementation time" instruction) that both correctly inherit the base
behavior with nothing to bypass.

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
