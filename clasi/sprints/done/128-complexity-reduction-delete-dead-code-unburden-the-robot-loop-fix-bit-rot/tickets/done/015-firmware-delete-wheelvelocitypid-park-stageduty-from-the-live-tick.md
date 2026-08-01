---
id: '015'
title: 'Firmware: delete WheelVelocityPid; park stageDuty() from the live tick'
status: done
use-cases:
- SUC-002
depends-on:
- '014'
github-issue: ''
issue: delete-wheel-velocity-pid-and-decide-the-duty-stage.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware: delete WheelVelocityPid; park stageDuty() from the live tick

**Worktree note**: implement in the git worktree checked out for
`sprint/128-complexity-reduction-delete-dead-code-unburden-the-robot-loop-fix-bit-rot`.
All paths below are relative to that worktree.

**Decision — SETTLED, plan of record (Eric approved, not an open
question)**: **PARK** the duty stage. Stop calling `stageDuty()` from the
live tick. Keep the `WheelPid` class and its existing ctest tiers intact
(do not delete the class). Add a `src/motion/DESIGN.md` note naming the
parked intent and its owner. `WheelVelocityPid` (a fully separate,
zero-instantiation class) is deleted unconditionally, independent of the
park/adopt call.

**Build-source-list requirement**: this ticket deletes
`wheel_velocity_pid.{h,cpp}` from `src/motion`. The build source lists
exist in **FOUR** places (root `CMakeLists.txt`, `src/motion/CMakeLists.txt`,
and the pytest `_APP_SOURCES` lists) — missing one is a link error, not a
compile error. Update all four explicitly.

## Description

`Motion::WheelVelocityPid` (`wheel_velocity_pid.{h,cpp}`) has zero
instantiations anywhere in `src/firm` — `main.cpp:70-75`'s own comment
confirms the pair that used it is gone. `Motion::WheelPid`/
`Planner::stageDuty()` (`planner.cpp:248-337`) runs a full per-wheel PID
every cycle (~21 evaluations/s) whose output `main.cpp:408-412` says is
"computed every tick and DISCARDED" — only `Motion::WheelTrim`
(velocity-domain) actually reaches the wheels. Three generations of the
wheel velocity-control law currently coexist; only one reaches the
wheels, and an engineer debugging misbehaving wheels needs a one-line
answer for which to suspect.

## Acceptance Criteria

- [x] `wheel_velocity_pid.{h,cpp}` are deleted, along with their entries
      in `src/motion/CMakeLists.txt` and the other three build-source-list
      locations (root `CMakeLists.txt`, both affected pytest
      `_APP_SOURCES` lists). [Actual scope: root `CMakeLists.txt` needed
      no edit — it globs `src/motion/*.cpp` via `RECURSIVE_FIND_FILE`, so
      deleting the file removes it from the ARM build automatically. The
      pytest source lists turned out to be ~14 files, not 2 — every one
      updated; see report.]
- [x] `stageDuty()` is no longer called from the live tick — the call
      site in `planner.cpp`/`main.cpp` (wherever it is actually invoked
      per-cycle) is removed. The `WheelPid` class itself, and its
      existing ctest tiers, are UNCHANGED — do not delete the class.
      [`stageDuty()` moved from private to a public, explicitly-callable
      method so `wheel_pid_test.cpp`/`planner_duty_scenarios_test.cpp`
      keep exercising it directly — see report for why.]
- [x] `main.cpp:408-412`'s "computed every tick and DISCARDED" comment is
      updated to reflect the parked state (it is no longer computed every
      tick at all, since the call is removed) — or removed if it no
      longer applies once the call site is gone.
- [x] `src/motion/DESIGN.md` gains a one-paragraph "wheel control
      generations" note: `WheelVelocityPid` (deleted), `WheelPid`/duty
      stage (PARKED — stopped calling from the live tick, class + ctest
      tiers kept warm, intent and owner named), `WheelTrim` (live, the
      loop that actually reaches the wheels).
- [x] The two historical doc-comment mentions of `WheelVelocityPid` in
      `device_config.h`/`nezha_motor.h` are updated (they currently
      reference a class that no longer exists).
- [x] `grep -rn "WheelVelocityPid" src/` returns nothing.
- [x] The park decision is visible IN CODE (the call removed), not left
      implicit or only stated in DESIGN.md prose.
- [x] `just build-clean`, `motion_tests` + planner ctest suite pass;
      bench smoke unchanged (the loop now spends ~21 fewer PID
      evaluations/s — a timing improvement, not a behavior change, since
      the output was discarded either way). [No physical hardware bench
      deployment performed — see report for rationale.]

## Testing

- **Existing tests to run**: `motion_tests`, planner ctest suite
  (`WheelPid`'s own ctest tiers must still pass — they are kept, only the
  live-tick call is removed), firmware pytest tiers, full
  `just build-clean`.
- **New tests to write**: none required — `WheelPid`'s existing ctest
  coverage already validates the class; removing its live-tick call site
  needs no new test, only confirmation the removal doesn't break
  anything that (incorrectly) depended on the discarded computation
  happening.
- **Verification command**: `just build-clean`, then
  `cmake --build src/motion/build --target motion_tests`, then
  `uv run python -m pytest src/tests -q`.

## Implementation Notes

- **Approach**: delete `WheelVelocityPid` first (unconditional, no
  decision attached), then remove the `stageDuty()` live-tick call site,
  then update the two stale doc comments and add the `src/motion/DESIGN.md`
  note.
- **Files to modify/delete**: delete `src/motion/wheel_velocity_pid.{h,cpp}`;
  edit `src/motion/planner/planner.cpp` or `src/firm/main.cpp` (wherever
  the live `stageDuty()` call site actually is — confirm exact location
  before editing), root `CMakeLists.txt`, `src/motion/CMakeLists.txt`,
  affected pytest `_APP_SOURCES` lists, `device_config.h`, `nezha_motor.h`,
  `src/motion/DESIGN.md`.
- **Documentation updates**: `src/motion/DESIGN.md`'s "wheel control
  generations" note (this ticket's own acceptance criterion) — this is a
  deliverable, not optional cleanup.
