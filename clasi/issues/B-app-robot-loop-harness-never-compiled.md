---
status: pending
priority: medium
---

# The base's integration coverage is a corpse, and the suite's green overstates what is tested

## Description

2026-08-02 post-130 review, **F9 (MAJOR)**. The post-mortem calls the underlying
habit out as a root cause: *"'Known pre-existing failure' is a claim, not a
fact."*

`src/tests/sim/unit/app_robot_loop_harness.cpp` includes `motion/move_queue.h`
and `motion/state_estimator.h` — both **deleted two sprints ago**.
`src/tests/sim/unit/test_app_robot_loop.py:110` xfails it as "does not COMPILE
at all" (~28 errors), non-strict, and it has been silently green since
2026-07-26.

Two other stale xfails now **xpass**.

Four sprint-130 tickets reported "no regressions" against a baseline of "2 known
failures" that turned out to be tests which had **never compiled** — a missing
`-I src` flag. Nobody read the error. (Fixed at `87cbdb2d` during close; this
issue is the larger hole it exposed.)

## What is actually uncovered

The harness's coverage was never relocated. It exclusively covered:

- boot-NACK behavior,
- routing arbitration,
- **the position-rebaseline policy** — which is
  [[A-position-rebaseline-destroys-the-pose]], a CRITICAL defect whose *only*
  test lives in this corpse and asserts the wire epoch rather than the pose,
- most flag derivations.

Meanwhile `motion_tests` runs **zero tests by design**
(`src/motion/CMakeLists.txt:83-98`), so `body_kinematics` and `odometry` — the
code F1 lives in — have no standalone coverage either.

## What to do

1. **Delete the corpse harness** and rebuild its scenarios on
   `TestSim::SimHarness`, where the telemetry half already moved.
2. Give the rebaseline policy a **pose-sanity** test across the boundary (assert
   the observable — continuous heading and x/y — not the wire counter). That is
   this issue's highest-value single output.
3. **Make surviving xfails strict**, so an xpass fails loudly instead of hiding.
4. Wire `motion_tests` to actually run `body_kinematics`/`odometry` coverage.
5. One-line fix while in the neighborhood: `composition_root_parity_harness.cpp:53-58`
   passes `snprintf` a `double` for `%s` and a `char*` for `%g` — undefined
   behavior exactly when the parity check fails and tries to explain itself
   (midpoint finding #7, verified still swapped today).

## Verification

- No test in the suite is xfailed for "does not compile"; a compile failure
  fails.
- Every scenario the deleted harness covered either has a live home or is
  explicitly recorded as dropped, with a reason.
- Surviving xfails are `strict=True`.
- `motion_tests` runs a non-zero number of tests.
- The parity harness's failure message is correct — verify by forcing a failure.

## Related

- `docs/code_review/2026-08-02-post-130-wheels-solid-review.md` F9, P1.
- `docs/code_review/2026-08-02-sprint-130-midpoint.md` finding 7.
- `docs/post-mortem/2026-08-02-sprint-130-wheels-solid.md` root cause 1.
