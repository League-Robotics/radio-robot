---
id: '002'
title: Drive live-OTOS indirection
status: open
use-cases:
- SUC-002
depends-on:
- '001'
github-issue: ''
issue: otos-not-used-frozen-pose-ekf-rejects-everything.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Drive live-OTOS indirection

## Description

`subsystems::Drive` holds `IOdometer& _otos` (`source/subsystems/drive/
Drive.h:123`), a C++ reference bound ONCE at construction to whatever
`hal.otos()` returned at that moment (`Robot.cpp`'s `drive(...)` initializer,
passing `hal.otos()`). A bound reference cannot be re-seated: once
`DBG OTOS BENCH` swaps the HAL's active pointer (ticket 001 makes this swap
real in sim; it was already real in firmware), `Drive::tickUpdate()`'s STEP 5
(`Drive.cpp:134-183`, the SOLE live OTOS-read-and-fuse path since the
ordered-tick cutover, sprint 060) keeps calling methods on the STALE object
forever. This is the direct, code-review-confirmed reason "bench mode shows
the same frozen-OTOS signature": the swap mechanism works, but the one
consumer that matters for fusion and telemetry never sees it.

`Robot::otosCorrect()` already solves this identical problem correctly — it
re-resolves `hal.otos()` fresh on every call (a fix shipped in ticket
031-002) — but that function has had zero live callers since the 060
ordered-tick cutover deleted the legacy loop that called it. This ticket
ports the SAME live-indirection pattern into the path that actually runs,
by giving `Drive` a `Hardware&` instead of a boot-bound `IOdometer&`.

This ticket depends on 001: the new sim test below toggles `DBG OTOS BENCH`
mid-session and needs 001's real `SimHardware` pointer-swap substrate to
prove the fix (against pre-001 `SimHardware`, toggling bench mode has no
effect on `otos()`'s return value at all, so this ticket's own test would be
unable to distinguish "Drive re-reads the pointer" from "the pointer never
changed").

See `architecture-update.md` Step 3 "Module: Drive OTOS consumption", Step 5
item 3, Design Rationale Decision 1 (why `Hardware&`, not a narrower
one-method seam); `usecases.md` SUC-002.

## Acceptance Criteria

- [ ] `Drive`'s constructor (`Drive.h:56-60`, `Drive.cpp:26-40`) changes its
      6th parameter from `IOdometer& otos` to `Hardware& hal`; the member
      `IOdometer& _otos` (`Drive.h:123`) is replaced by `Hardware& _hal`; the
      initializer list changes `_otos(otos)` to `_hal(hal)`.
      `#include "hal/capability/IOdometer.h"` in `Drive.h` is replaced by
      `#include "hal/Hardware.h"`.
- [ ] Every STEP-5 read call site in `Drive::tickUpdate()` resolves through
      `_hal.otos()` fresh, not a cached reference:
      `_otos.is_initialized()` (`Drive.cpp:137`) → `_hal.otos().is_initialized()`;
      `_otos.readTransformed(...)` (`Drive.cpp:148`) → `_hal.otos().readTransformed(...)`;
      `_otos.readStatus(...)` (`Drive.cpp:159`) → `_hal.otos().readStatus(...)`;
      `_otos.readVelocityTransformed(...)` (`Drive.cpp:163`) →
      `_hal.otos().readVelocityTransformed(...)`.
- [ ] `Drive::capabilities()`'s `caps.onboard_position = _otos.is_initialized();`
      (`Drive.cpp:476`) is updated to `_hal.otos().is_initialized()` — the
      only other `_otos.` reference in the file (grep-confirmed: `_otos.` has
      exactly 5 occurrences in `Drive.cpp` before this change, all updated).
- [ ] `Robot.cpp`'s `drive(...)` construction call passes `hal` instead of
      `hal.otos()` (`Robot.h` already declares `Hardware& hal;` as a member,
      so this is a bare identifier swap, no new member needed).
- [ ] **All THREE ctypes test-harness files that construct `subsystems::Drive`
      directly** are updated the same way — `hal.otos()` → `hal` in the
      `drive(...)` initializer:
      `tests/_infra/sim/drive_api.cpp` (`DriveHandle`, ~line 59),
      `tests/_infra/sim/bus_drain_api.cpp` (`BusDrainHandle`, ~line 74),
      `tests/_infra/sim/planner_api.cpp` (~line 61). NOTE: this corrects a
      factual gap in `architecture-update.md`'s Migration Concerns, which
      states the sole production call site is `Robot.cpp` and implies test
      call sites are unaffected because they construct `Drive` "only
      indirectly through Robot/SimHandle" — that is true of
      `tests/_infra/sim/sim_api.cpp`'s `SimHandle` (which owns a `Robot`),
      but these three OTHER harness files construct `subsystems::Drive`
      directly with `hal.otos()` as an argument and DO need this one-line
      update each, confirmed by grep during ticket planning
      (`grep -rn "subsystems::Drive" tests/_infra/sim/*.cpp`).
- [ ] A new sim test (built on ticket 001's bench-otos substrate): drive,
      toggle `DBG OTOS BENCH 1` mid-session, tick one control period, and
      confirm the live fusion/telemetry path's `otos=`/fused-pose behavior
      reflects the bench sensor's simulated motion starting the VERY NEXT
      tick — not the frozen value the pre-fix boot-bound reference would
      keep returning. This test MUST FAIL against pre-fix `Drive` (confirm
      by temporarily reverting the constructor change locally during
      implementation, or by reasoning from the diff, per this codebase's
      practice of citing a concrete before/after where a regression test's
      power matters — see e.g. ticket 073-001's Testing section for the
      expected rigor).
- [ ] Toggling `DBG OTOS BENCH 0` (back off) similarly restores the
      previously active sensor's readings on the live path on the next tick.
- [ ] No behavior change for any session that never toggles bench mode — the
      default-bound sensor (`hal.otos()` at the moment of the first STEP-5
      read, which for a session that never calls `setOtosBench` is the same
      object the old boot-bound reference would have captured) behaves
      identically. Confirmed by the existing `test_otos_warn_persistence.py`
      suite (all 3 tests) passing unmodified.
- [ ] Full suite (`uv run python -m pytest`) passes at the 2672 baseline (+
      ticket 001's net additions) + this ticket's net new test count, zero
      unexplained failures. The `data/robots` drift noted in the sprint's
      hard contract is environmental — do not chase or touch it.

## Testing

- **Existing tests to run**: `tests/simulation/unit/test_otos_warn_persistence.py`
  (all three, confirming the live-indirection change is behavior-neutral
  when bench mode is never touched), any `Drive`-level test exercised
  through `drive_api.cpp`/`planner_api.cpp`/`bus_drain_api.cpp`'s ctypes
  surface (these will hard-fail to COMPILE if their `drive(...)`
  initializer isn't updated — treat a compile failure in any of the three
  as a signal this ticket's file list is incomplete, not as an unrelated
  break), full suite.
- **New tests to write**: the mid-session bench-toggle test described above
  (SUC-002's acceptance criteria) — likely alongside or in the same file as
  ticket 001's bench-tracks-motion test, since it reuses the same setup
  (`DBG OTOS BENCH 1` + drive + tick) and adds the "toggle mid-session,
  confirm the LIVE path switches" assertion on top.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Change `Drive`'s constructor signature and member first
(`Drive.h`), then fix every call site inside `Drive.cpp` that the compiler
flags (STEP 5's four reads plus `capabilities()`'s one read — five total).
Fix `Robot.cpp`'s one construction-argument line. Then fix the three
ctypes-harness construction sites the compiler will ALSO flag (these are
easy to miss if only grepping "production" call sites, per the correction
noted in Acceptance Criteria — build the host-sim target early to surface
all of them at once rather than iterating file-by-file from a doc citation).
Add the new bench-toggle-mid-session test last, once the substrate compiles
end-to-end.

**Files to create/modify**:
- `source/subsystems/drive/Drive.h` — constructor signature, `_hal` member,
  include swap.
- `source/subsystems/drive/Drive.cpp` — five `_otos.` call sites → `_hal.otos().`.
- `source/robot/Robot.cpp` — `drive(...)` construction argument.
- `tests/_infra/sim/drive_api.cpp`, `tests/_infra/sim/bus_drain_api.cpp`,
  `tests/_infra/sim/planner_api.cpp` — `drive(...)` construction argument
  (three files, one line each).
- New/extended test file under `tests/simulation/unit/` for the mid-session
  toggle assertion.

**Testing plan**: build the host-sim shared library FIRST and let the
compiler enumerate every call site needing the one-line fix (catches
anything this ticket's manual grep missed); then run
`test_otos_warn_persistence.py` to confirm no regression when bench mode is
untouched; then the new toggle test; then the full suite.

**Documentation updates**: `Drive.h`'s class comment
(`source/subsystems/drive/Drive.h:1-13`) and the constructor's own comment
gain a one-line note that OTOS is resolved live through `Hardware` every
tick (not bound at construction), cross-referencing `Robot::otosCorrect()`'s
existing header comment that explains why (already-correct prior art, per
`architecture-update.md`'s Sprint Changes Summary item 3).
