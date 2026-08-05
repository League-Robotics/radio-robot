---
status: pending
priority: medium
---

# Planner residuals, and the last two includes between `src/motion` and `src/firm`

## Description

2026-08-02 post-130 review, Part 3 (P2–P6 plus the separability scorecard). The
headline planner defect, F10, is filed separately as
[[A-tour2-146-degree-turn-still-undershoots-after-130-010]] — this issue is the
remainder.

Context: the standalone suite builds and passes **8/8 in ~2 s with no Python and
no firmware**. The separability story is fundamentally sound; what follows is
the last mile plus small correctness residue.

## Separability — grep-verified, two includes from zero

Exactly **three** includes cross into `src/firm`:

1. `planner.h:35` → `types/robot_state.h` (the sanctioned boundary, itself
   `<cstdint>`-clean)
2. `tests/test_support.h:13` → same
3. `body_kinematics.{h,cpp}` → `messages/common.h`, for the `msg::BodyTwist3`
   array overloads **only**

No `Devices::*`, `App::*`, `Config::*`, clock, bus, or telemetry type anywhere.

**Path to zero:**
- Relocate `robot_state.h` to a neutral shared root, so a future
  `git subtree split` of `src/firm` does not take the planner's one include with
  it. A `mv` plus include edits.
- Give `BodyKinematics` a motion-owned twist triple — one header, ~4 call sites.

## The real coupling is the build, not the code

The planner source list is duplicated in **~14 places**: its own CMake,
`src/sim/CMakeLists.txt:135-143`, the root glob's exclusion regexes, and
**eleven** pytest harnesses hardcoding the four `.cpp` paths. A missing entry is
a **link** error, not a compile error — which is exactly as confusing as it
sounds, and has bitten before.

**Fix:** one shared source-list constant (or glob) for the pytest tier; glob the
planner directory the way the ARM build already does.

## Correctness residue

- **P2 — the 130-010 Angle completion gate is a no-op for Wheels-kind Moves.**
  The gate tests `profileVelocity_ ≤ settleRestOmega` (`planner.cpp:552-558`),
  but the Wheels branch never writes `profileVelocity_` and `activateNext()`
  zeroes it, so `|0| ≤ floor` is trivially true. The comment sells a guarantee
  the code does not provide. Tours use Twist, so TOUR_2 is unaffected — but the
  next Wheels-based tour would be.
- **P3 — `estop()` does not invalidate the ledger carry.** A
  completes-into-pending-Stop window (`planner.cpp:705-714`) lets a post-estop
  Move adopt pre-estop baselines. One line: `carryValid_ = false`.
- **P4 — `move(replace=true)` leaves `lifecycle_` stale until the next tick**,
  while `estop()` got the immediate-consistency treatment. The two preemption
  paths disagree on observability.
- **P5 — Wheels Moves ramp under linear ceilings only**, never
  `alphaMax`/`alphaDecel`. Consistent with "direct wheel commands" but
  undocumented at the limits level.
- **P6 — ctypes mirror drift.** `planner_harness.py` still declares
  `MovePhase.SETTLE = 4` (deleted in 130-008), and `plannerTrim()`
  (`capi.cpp:58`) keeps its stale ABI name — it contains no trim; 130-005
  deleted `Motion::WheelTrim`. The `PlannerLimits` mirror itself is verified
  coherent (18 fields, offsets checked), but **`RobotState`'s mirror has no
  per-field offset guard** — a same-size mid-struct swap corrupts silently. Give
  it the offset table `PlannerLimits` already has.

## Test gaps worth closing in the same pass

No Angle scenario above 90° exists in `planner_tests`; no chained
leg→large-turn→leg under lag; no transient-misprediction-vs-latch case. All
three are what [[A-tour2-146-degree-turn-still-undershoots-after-130-010]]
needs, so build them here and let that issue use them.

## Verification

- `grep -rn "src/firm\|firm/" src/motion --include=*.h --include=*.cpp` returns
  nothing (or only the one sanctioned neutral-root include).
- The planner source list exists once for the pytest tier; deleting a `.cpp`
  from disk produces one clear failure, not eleven.
- P2–P4 each have a behavioral test.
- `RobotState`'s ctypes mirror has an offset guard; a deliberate mid-struct
  swap fails it.
- `motion_tests` still passes standalone in ~2 s with no Python.

## Related

- `docs/code_review/2026-08-02-post-130-wheels-solid-review.md` Part 3.
- `docs/code_review/2026-08-02-sprint-130-midpoint.md` findings 12, 15.
