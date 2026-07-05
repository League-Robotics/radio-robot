---
id: '001'
title: Port Hal::EkfTiny -- 3-state EKF core
status: open
use-cases: [SUC-001]
depends-on: []
github-issue: ''
issue: plan-revive-testgui-against-the-new-tree-simulator.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Port Hal::EkfTiny -- 3-state EKF core

## Description

Port `source_old/state/EKFTiny.{h,cpp}` (the parked, sprint-050 TinyEKF
wrapper) into a new `Hal::EkfTiny` in the new `source/` tree, trimmed from
the old **5-state** (x, y, theta, v, omega) filter down to a **3-state**
(x, y, heading) filter. This is the numeric core `Subsystems::PoseEstimator`
(ticket 002) wraps; it has no device I/O and no CODAL dependency.

`libraries/tinyekf` is **already vendored** and already on the root
`CMakeLists.txt` include path (`include_directories(${PROJECT_SOURCE_DIR}/libraries/tinyekf)`)
-- this is not a new external dependency, just a new consumer of one the
project already carries.

**Deliberately dropped from the old 515-line class** (architecture-update.md
Decision 2 -- do not reintroduce any of this without a fresh, acceptance-bar-
driven reason):
- The velocity/omega state sub-block and its `updateVelocity()` channel.
  `twist=` (TLM) is populated from directly-measured/derived rates elsewhere
  (ticket 004), not filtered EKF state.
- Mahalanobis chi-squared gating on any channel.
- P-inflation gate-recovery and the rejection-streak counters
  (`rejHeadStreak()`/`rejPosStreak()`/`rejectedCount()`).
- Any test-harness accessor that only exists to support the old class's
  Python-oracle parity test (`tests_old/`) -- not this sprint's scope to
  re-validate.

**Keep, adapted to the 3-state shape and Google/project naming (lowerCamelCase
methods, no unit-suffixed identifiers, units in `// [unit]` comment tags):**
- `predict(dCenter, dTheta, thetaBefore, dt)` -- the arc-segment motion model,
  minus the velocity sub-block.
- `updatePosition(xOtos, yOtos)` -- the 2-observation (M=2) position channel,
  analytic 2x2 S-inverse (same numerical approach as the old class, no
  Cholesky `ekf_update` invert needed for a 2x2).
- `updateHeading(thetaOtos)` -- scalar (1-DOF) heading channel, wrap-safe
  innovation (`y = wrapPi(thetaMeas - x[2])`), applied manually (no
  `ekf_update` call), same as the old class.
- `setPose(x, y, theta)` -- overwrite state with a known pose (used by
  ticket 002's construction/reset path), sane diagonal P-prior instead of
  zeroing P.
- Plain accessors: `x()`, `y()`, `theta()`, `pDiag(idx)` (idx in [0..2] now,
  not [0..4]).
- `init(qXy, qTheta, rOtosXy, rOtosTheta)` -- 4 noise parameters, not 8 (only
  the position/heading channels this class implements).

## Acceptance Criteria

- [ ] `Hal::EkfTiny` (`source/estimation/ekf_tiny.h` / `.cpp`) compiles with
      no `#include "MicroBit.h"` and no I2C dependency -- includes only
      `<math.h>`, `<stdint.h>`, `<tinyekf.h>`.
- [ ] State vector is 3-wide (x, y, heading); `EKF_N`/`EKF_M` (or equivalent
      TinyEKF macros) reflect a 3-state, 2-observation shape -- no `v`/`omega`
      state, no `updateVelocity()` method exists on this class.
- [ ] No Mahalanobis/chi-squared gating, rejection-streak counter, or
      P-inflation logic exists on this class (grep-verifiable: no
      `rejStreak`, no `chiSquare`, no gate-recovery constant).
- [ ] All method and parameter names are lowerCamelCase (never PascalCase),
      no unit-suffixed identifier (`xEntry`/`thetaObs`, not `x_mm`/`theta_rad`)
      -- units live in leading `// [unit]` comment tags per
      `.claude/rules/coding-standards.md`.
- [ ] A synthetic predict-only unit test sequence (no corrections) matches a
      hand-computed arc-integration reference within floating-point
      tolerance.
- [ ] A synthetic predict+correct unit test sequence demonstrably pulls the
      estimate toward a deliberately-offset position/heading observation
      (proves the correction step runs, not a no-op) -- e.g. drive the filter
      several ticks with a constant encoder-derived arc, then apply
      `updatePosition`/`updateHeading` with a synthetic observation offset by
      a known amount, and assert the post-update state moved toward that
      observation (not away from it, not unchanged).
- [ ] `tests/_infra/sim/CMakeLists.txt`'s source list is noted as needing
      `source/estimation/ekf_tiny.cpp` added (ticket 003/004 land the actual
      CMakeLists.txt edit once the sim harness's file is touchable per this
      sprint's dependency gate -- see Implementation Plan).

## Implementation Plan

### Approach

1. Read `source_old/state/EKFTiny.{h,cpp}` in full; identify exactly the
   position (M=2) and heading (scalar) update paths and the arc-segment
   predict step -- these are the only paths this ticket ports.
2. Write `source/estimation/ekf_tiny.h` / `.cpp` (a new top-level directory,
   sibling to `source/kinematics/` -- pure math, no `Hal`/`Subsystems` device
   ownership, matching that directory's existing precedent; class name
   `EkfTiny`, NOT namespaced under `Hal::` despite the ticket title's shorthand
   -- confirm against `source/kinematics/body_kinematics.h`'s un-namespaced
   class convention before finalizing, and note the final namespace/location
   choice in the class's own file-header comment for the next ticket).
3. Use **only** `msg::Pose2D`/`msg::BodyTwist3`/`msg::PoseEstimate`
   (`source/messages/common.h`) for any pose-shaped type this class's public
   surface needs -- never `source/kinematics/pose2d.h`'s parallel,
   unit-suffixed `Pose2D`/`BodyTwist3` family (see architecture-update.md
   "Grounding," fact 3, for why that second family exists and must not be
   reused here).
4. Write host-side unit tests (see Testing below) proving the predict and
   predict+correct behaviors independently of any encoder/odometer wiring
   (that wiring is ticket 002's job).

### Files to create

- `source/estimation/ekf_tiny.h`
- `source/estimation/ekf_tiny.cpp`
- A new host-compiled test harness under `tests/sim/unit/` (matching the
  existing `tests/sim/unit/*_harness.cpp` ad hoc-compile convention sprints
  078/079/081 already use for pre-CMake acceptance) -- e.g.
  `tests/sim/unit/ekf_tiny_harness.cpp`.

### Files to modify

- None. This ticket is additive only -- no existing file changes.

### Testing plan

- New standalone-compiled harness (no CMake dependency yet, matching
  `tests/sim/unit/*_harness.cpp`'s existing pattern): predict-only sequence,
  predict+correct sequence (see Acceptance Criteria).
- Verification command: compile and run the new harness directly (see an
  existing `*_harness.cpp` for the exact g++/clang++ invocation this project
  uses for that ad hoc tier); no `uv run pytest` involvement for this ticket
  (pure C++, no Python surface yet).

### Documentation updates

- None required this ticket (no wire-visible change; `docs/protocol-v2.md` is
  untouched until ticket 004). Note in the class's own file-header comment
  that it is a simplified derivative of `source_old/state/EKFTiny.*` and list
  exactly what was dropped (velocity channel, gating, P-inflation) so a future
  reader does not mistake the omission for an oversight.
