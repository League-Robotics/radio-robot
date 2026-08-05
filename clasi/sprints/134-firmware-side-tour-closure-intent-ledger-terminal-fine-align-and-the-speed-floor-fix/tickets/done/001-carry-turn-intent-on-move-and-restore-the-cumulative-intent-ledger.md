---
id: '001'
title: Carry turn intent on Move and restore the cumulative-intent ledger
status: done
use-cases:
- SUC-002
depends-on: []
github-issue: ''
issue: A-turn-baseline-ledger-ignores-the-preceding-legs-heading-drift.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Carry turn intent on Move and restore the cumulative-intent ledger

## Description

**Source of truth: `docs/bench-reports/motion-planning-lab-2026-08-04.md` §5.1.
Read it before starting. Do not re-derive its numbers — cite them.**

Today `Motion::Planner`'s cumulative-baseline ledger re-anchors every turn to
wherever the robot happened to stop, so a heading residual is never repaid — it
silently becomes the new truth. Measured on the sequential planner square tour:
**+1.5 / +5.3 / +6.3 / +10.6° cumulative drift, 64.1 mm closure.**

### Why the ledger is like this (read the history before changing it)

`App::RobotLoop::handleMove()` (`src/firm/app/robot_loop.cpp:272-286`) applies
the rotation-calibration inversion to `Motion::Move::threshold` *at ingestion*,
before the planner ever sees the Move:

```cpp
const float corrected = (m.threshold - offset) / gain;
m.threshold = (corrected > 0.0f) ? corrected : m.threshold;
```

So by the time the ledger reads `threshold`, it is an actuation-sized command,
not "what this Move was asked to turn". Commit **`af3ca435` (130-010)** hit
exactly this and had to abandon intent-carry for measured-carry. Its comment
still stands in `planner.cpp:645-659` and says so explicitly. **Read
`af3ca435` in full before editing** — `git show af3ca435`.

### The fix: carry intent as its own field

Report §5.1 sanctions two routes: "Move the calibration to the actuation side,
**or carry intent as its own field.**" **This ticket takes the second** — see
sprint.md Design Rationale **D1** for the full justification. The short version:
relocating the calibration costs four constants through the entire
config→`PlannerLimits` pipeline (~16 files) *and* changes what the profiler is
asked to turn, while the intent field touches ~5 files, changes no commanded
value, and adds no constant.

Add a field to `Motion::Move` recording the caller's **requested** threshold,
set it in `handleMove()` *before* the calibration rewrite, and point the Angle
branch of the ledger at it:

```cpp
carryHeading_ = active_.baselineHeading + angularDirection(m) * m.<requested>;
```

`Move` then has two readers that legitimately want different things: the
**profiler** wants the command (`threshold`), the **ledger** wants the intent.

### Naming

Per `.claude/rules/naming-and-style.md`: name the quantity, no units in the
identifier, units in a leading `// [unit]` comment tag. Suggested
`requestedThreshold` (mirroring `threshold`'s own multi-kind comment:
`// [ms] Time / [mm] Distance / [rad] Angle`), documented as "what the CALLER
asked for, before any upstream rewrite of `threshold`". Final name is yours; it
must not encode a unit and must not start with an uppercase letter.

### Do not regress what 130-010 fixed

`af3ca435` also fixed an **Angle profile-complete undershoot** — gating the
`boundary<=0` profile-complete branch on `profileVelocity_` reaching the same
rest floor `settleReached()` uses (`planner.cpp:581-586`). **Keep that.** Its
guard test is
`planner_noise_test.cpp::testAngleDoesNotUndershootAtCompletionUnderSevereTrackingLag`
and it must still pass.

### Do not add a rotation fudge constant

The linked issue's own instruction #4, and it is load-bearing: `tovez_nocal.json`
already carried rotation constants fitted to a sim artifact (1.006 / +12.1°)
that injected ~12.5° of under-rotation into every real turn. **Another
correction layer is exactly how that happened the first time.** This ticket
adds no constant.

### What this ticket does NOT close

The linked issue also demands (a) attribution of the unexplained residual
remainder and (b) an explanation of TOUR_2's 146° sign flip. Neither is in
scope; `completes_issue: false` is set deliberately. Do not change it.

## Acceptance Criteria

- [x] `Motion::Move` carries the caller's requested threshold as a distinct
      field, set in `handleMove()` **before** the calibration rewrite
- [x] The Angle branch of the completion ledger carries
      `baselineHeading + angularDirection(m) * <requested>` instead of
      `pose_.heading()`
- [x] The Distance branch is **unchanged** (a straight leg intends zero heading
      change; it carries its baseline forward)
- [x] The stale comment at `planner.cpp:645-659` is rewritten to describe the
      new mechanism — it currently documents the opposite behaviour
- [x] `planner_harness.py:149`'s `Move` ctypes mirror gains the field in the
      same position (append at the end of both)
- [x] 130-010's Angle profile-complete rest-floor gate is preserved, and
      `testAngleDoesNotUndershootAtCompletionUnderSevereTrackingLag` passes
- [x] No new configuration constant is introduced
- [x] A ctest demonstrates residual repayment: a chained turn whose predecessor
      left a heading residual targets the **cumulative** heading, not a fresh
      re-anchor

## Completion Notes

**Field name**: `Motion::Move::requestedThreshold`, appended after `vRight`.
`<= 0` means UNSET and the ledger falls back to `threshold` — the same
fail-open convention `settleRestOmega <= 0` already uses in the Angle
profile-complete gate. That is what keeps every direct caller (the ctests,
the sim harnesses, the ctypes bench harness) sane: none of them has an
ingestion-side rewrite in front of it, so for them command **is** intent.
It is deliberately *not* bit-identical to 130-010's behaviour for those
callers — an unset Move now projects an exact `baseline + threshold`
instead of a completion-tick `pose_.heading()` reading that measured ~0.93°
behind true rest. That gap is the defect, not a side effect.

`handleMove()` sets the field for **every** `Kind`, not just Angle: it means
the same thing on every axis and a Kind-conditional assignment would be a
trap for the next reader.

**Verification that the new ctest bites**: `carryHeading_` was temporarily
reverted to `pose_.heading()` and `planner_scenarios_test` rebuilt — the new
`testAngleChainRepaysRequestedIntentResidual` failed at 169.07° against the
expected 180.0° (the injected 10° shortfall, plus 0.93° of the
completion-tick pose lag described above). Restored and re-run: 8/8 ctests
pass.

**The repayment needs no new mechanism.** With intent carried,
`activateNext()`'s ledger adoption puts the next Move's `baselineHeading` at
the cumulative target, and `measure()`'s existing
`m.threshold - (heading - baseline) * dir` therefore starts a debt-carrying
turn short by exactly the residual. No constant was added.

**Test placement deviation**: the repayment ctests went into
`planner_scenarios_test.cpp`, not `planner_lifecycle_test.cpp` — the
cumulative-carry tests (`testSameAxisChainExactAndCarried`,
`testOrthogonalChainExact`) already live there, and lifecycle_test pins
states/events rather than ledger arithmetic. Two tests were added: the
repayment case and `testAngleChainWithoutRequestedIntentCarriesTheCommand`,
which pins the unset-field fallback so a zero-default can never silently
make a legacy caller's turn target its own activation baseline.

**Firmware ARM build not run.** `uv run python build.py --clean
--robot-debug` unconditionally runs `dotconfig version bump`
(`build.py:121-131`), which `.claude/rules/git-commits.md` forbids during
ticket work on a sprint branch (`close_sprint` owns the one bump per
sprint); and ticket 002's in-flight `drive.cpp`/`drive.h` edits were live in
the same working tree, so a firmware build would have compiled another
ticket's WIP. Compile coverage of the exact changed sources came instead
from the standalone `planner_tests` build and from `src/tests/sim`, whose
harnesses compile `robot_loop.cpp`/`planner.cpp` from source into a fresh
per-test binary on every run. **Ticket 004 must still do a clean build** —
`planner_types.h` is a shared header
(`.clasi/knowledge/clean-build-after-shared-header-change.md`).

**`test_tour_closure_gate` not run.** It lives in `src/tests/testgui`, not in
this ticket's verification command, and needs `src/sim/build/
libfirmware_host.dylib` rebuilt — which would have baked in ticket 002's
half-finished speed-floor change and made the number unattributable. It is a
known-failing sim gate whose corner behaviour sign-flips vs hardware
(report §7), so its verdict is not actionable here anyway. Ticket 003's full
run is the right place to record its movement.

## Implementation Plan

**Files to modify** (expected ~5):

1. `src/motion/planner/planner_types.h` — add the field to `struct Move`
   (append, after `vRight`, to keep the ctypes mirror append-only)
2. `src/firm/app/robot_loop.cpp` — `handleMove()`: set it from the wire value
   before the calibration block at `:272-286`
3. `src/motion/planner/planner.cpp` — the Angle carry at `:643-660`; rewrite
   the doc comment
4. `src/tests/bench/planner_harness.py` — `Move` ctypes mirror at `:149`
5. `src/motion/planner/tests/planner_lifecycle_test.cpp` (or
   `planner_noise_test.cpp`) — the new repayment ctest

**Watch for**: every other construction site of `Motion::Move` (tests, sim
harnesses, `capi.cpp`). A default-initialised new field means an
uninitialised-intent Move carries `0`, which would make the ledger target the
baseline. Choose a default that keeps a caller who never sets it behaving
sanely, and grep the whole repo for `Motion::Move` construction sites.

**Build**: `uv run python build.py --clean --robot-debug`. A change to
`planner_types.h` is a shared-header change — a clean build is required, not
optional (a stale incremental build makes encoders read a manufactured zero
that looks exactly like a dead bus).

## Testing

- **Existing tests to run**: `planner_noise_test`, `planner_lifecycle_test`
  (ctest); `src/tests/sim`; `src/tests/unit`. Compare failures **by identity**
  against the master baseline of ~8 failed / ~1994 passed
  (`test_move_protocol`, `test_rebaseline_pose_sanity`,
  `test_gen_boot_config_planner` ×2, `test_gen_boot_config_robot_groups`,
  `test_gui_button_acceptance` ×2, `test_tour_closure_gate`).
- **New tests to write**: a planner ctest showing a chained Angle Move targets
  the cumulative intent heading after a predecessor left a residual.
- **`test_tour_closure_gate` may move.** It is a **sim** gate, and per report §7
  the sim's corner behaviour **sign-flips** vs hardware (−1.4° under-rotation in
  sim vs +1.55° over-rotation on `tovez`). **Record what it does; do not tune
  firmware to satisfy it.** Bench is the arbiter for corners.
- **Verification command**:
  `uv run python -m pytest src/tests/sim src/tests/unit -q`
