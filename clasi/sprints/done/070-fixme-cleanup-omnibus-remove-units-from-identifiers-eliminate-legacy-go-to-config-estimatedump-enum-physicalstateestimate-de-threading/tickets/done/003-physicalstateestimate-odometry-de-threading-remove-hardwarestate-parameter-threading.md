---
id: '003'
title: 'PhysicalStateEstimate/Odometry de-threading: remove HardwareState parameter
  threading'
status: done
use-cases:
- SUC-004
- SUC-005
depends-on: []
github-issue: ''
issue: physicalstateestimate-remove-hardwarestate-param-threading.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# PhysicalStateEstimate/Odometry de-threading: remove HardwareState parameter threading

## Description

`PhysicalStateEstimate` (and its wrapped implementation `Odometry`) currently
take a `HardwareState&`/`ActualState&` on nearly every method, even though
each method only reads or writes a small, specific sub-piece of that
200+-byte blob. Issue 3 asks to stop threading the whole state object and
instead make inputs (encoder readings), config (trackwidth, rotational slip,
EKF noise), and outputs (the three pose-estimate snapshots) explicit.

**Decision 3 (approved at the stakeholder gate) is the key design constraint
for this ticket**: reading the actual call graph (not just the issue text)
found that `resetPose`/`zero` are called from **two independent, live
places targeting two different `PoseEstimate` storage locations**:
1. `Drive.cpp:327` — targets `Drive`'s private `_hw` (the message-contract /
   ordered-tick path).
2. `SystemCommands.cpp` (`handleSI`, `handleZero`) — targets `Robot`'s own
   top-level `state.actual` directly (the legacy `SI`/`OV`/`ZERO pose`
   command path). `handleSI` deliberately *also* stages a
   `drive.apply(SetPose)` so `Drive::_hw` gets independently re-anchored too
   — the existing code comment says this dual-write is deliberate, not an
   oversight.

A single "bind once at construction" injection point (the issue's literal
second option) would silently collapse these two intentionally-distinct
destinations onto one, which is **not** behavior-preserving given
`LoopTickOnce`'s one-directional, once-per-tick `Drive::_hw → Robot::
state.actual` sync. This ticket therefore uses **explicit per-call output-
reference parameters** on every observation/reset method instead — see
`architecture-update.md` Decision 3 and the 4c diagram for the full call-site
topology and why the naive reading of the issue would have broken behavior.

Config (trackwidth, rotational slip, EKF noise) is genuinely single-
destination (`Drive` is the only caller), so it uses a "set once, refreshed
live" shape instead: a new `setKinematics(trackwidthMm, rotationalSlip)`
call, invoked every tick from `Drive::tickUpdate()` (matching today's
every-tick freshness exactly) — this preserves sprint 067's live-`SET`-
reaches-the-estimator guarantee. `setCtx` is deleted outright — it was
already a documented no-op, and no replacement injection point is needed
since every remaining method is now explicit per-call.

`Odometry::correct()` (the pre-EKF complementary filter) is confirmed dead
(zero callers, per sprint 067's audit) and is **left untouched**, still
taking `HardwareState&` — it is unreachable via any `PhysicalStateEstimate`
method and out of this ticket's acceptance-criteria scope (Decision 6,
mirroring sprint 067 Decision 5's "document dead things, don't fix them"
precedent).

See `architecture-update.md` Step 5 ("PhysicalStateEstimate de-threading"),
Decisions 3 and 6, and the 4c call-site-topology diagram; `usecases.md`
SUC-004 (live `SET` of kinematics/noise) and SUC-005 (byte-identical
three-pose telemetry).

## Acceptance Criteria

- [x] `source/state/PhysicalStateEstimate.h`/`.cpp` — new `void
      setKinematics(float trackwidthMm, float rotationalSlip);` (forwards to
      `Odometry::setKinematics`).
- [x] `addOdometryObservation(HardwareState&, float, float, uint32_t)` →
      `addOdometryObservation(float encLeftMm, float encRightMm, uint32_t
      now_ms, PoseEstimate& encoderOut, PoseEstimate& fusedOut)`.
- [x] `addOtosObservation(HardwareState&, ...)` →
      `addOtosObservation(float x_otos, float y_otos, float theta_otos_rad,
      float v_otos_mmps, float omega_otos_rads, float vy_otos_mmps, uint32_t
      now_ms, PoseEstimate& opticalOut, PoseEstimate& fusedOut)`.
- [x] `resetPose(HardwareState&, int32_t, int32_t, int32_t)` →
      `resetPose(float encLeftMm, float encRightMm, int32_t x_mm, int32_t
      y_mm, int32_t h_cdeg, PoseEstimate& encoderOut, PoseEstimate&
      fusedOut)`.
- [x] `zero(HardwareState&)` → `zero(float encLeftMm, float encRightMm,
      PoseEstimate& encoderOut, PoseEstimate& fusedOut)`.
- [x] `static getPose(const HardwareState&, ...)` → `static getPose(const
      PoseEstimate& fused, int32_t&, int32_t&, int32_t&)`.
- [x] `getVelocity`, `encoderEstimate`, `opticalEstimate`, `fusedEstimate`,
      and `setCtx` are **deleted** from `PhysicalStateEstimate` (confirmed
      zero callers anywhere in `source/` or `tests/_infra/`).
- [x] `source/control/Odometry.h`/`.cpp` — every signature above mirrored
      (it is the wrapped implementation); new `_trackwidthMm`/
      `_rotationalSlip` member fields and `setKinematics()` added;
      `predict()`'s trackwidth/slip parameters removed (read from the new
      members instead); `setCtx` deleted.
- [x] `Odometry::correct()` is left untouched, still taking `HardwareState&`
      — confirmed still zero callers; not modified by this ticket (Decision
      6).
- [x] `source/subsystems/drive/Drive.cpp` — `tickUpdate()` calls
      `_est.setKinematics(...)` every tick (feeding the trackwidth/rotSlip
      read that today is passed as observation parameters); the
      `addOdometryObservation`/`addOtosObservation`/`resetPose` call sites
      (including the `SetPose` command handler) pass `_hw.encMm[]`/
      `_hw.encoder`/`_hw.optical`/`_hw.fused` explicitly. No change to
      `Drive.h`'s public API.
- [x] `source/robot/Robot.cpp` — the `estimate.setCtx(&otos, &state.actual);`
      line (line 130) is deleted; the dead `otosCorrect()`'s
      `addOtosObservation` call is updated to the new signature so it still
      compiles (`&state.actual.optical`, `&state.actual.fused`).
- [x] `source/commands/SystemCommands.cpp` — `handleZero`'s `estimate.zero(
      ...)` and `handleSI`'s `estimate.resetPose(...)` calls updated to pass
      `robot->state.actual.encMm[1]/[0]` and `&robot->state.actual.encoder`/
      `&robot->state.actual.fused` explicitly. No change to `SI`/`ZERO`'s
      wire grammar or reply text.
- [x] `source/superstructure/Planner.cpp::getPoseFloat` —
      `PhysicalStateEstimate::getPose(*_hwState, xi, yi, hi)` →
      `PhysicalStateEstimate::getPose(_hwState->fused, xi, yi, hi)`.
- [x] No `PhysicalStateEstimate` method takes a `HardwareState&`/
      `ActualState&` parameter (`grep -n "HardwareState&\|ActualState&"
      source/state/PhysicalStateEstimate.h` returns nothing).
- [x] New unit test coverage for `Odometry::setKinematics()`/
      `PhysicalStateEstimate::setKinematics()` as a live-update regression
      (mirrors sprint 067's own methodology: fresh `Sim()`/`ZERO enc`, `SET
      tw=`/`SET rotSlip=` changes the next tick's `predict()` output by the
      expected amount).
- [x] New/updated unit test coverage for the narrowed `getPose(const
      PoseEstimate&, ...)` signature.
- [x] `tests/_infra/golden_tlm_capture.json` requires no regeneration (no
      TLM field/format change) — `encpose=`/`otos=`/`pose=` TLM fields
      byte-identical.
- [x] Immediately after `SET` of `tw`/`rotSlip`/any `ekfQ*`/`ekfR*` key
      mid-mission, the fused pose/velocity read back identically to their
      pre-`SET` values (no reset-to-origin regression — mirrors sprint 067's
      own acceptance criterion).
- [x] Full test suite green (`uv run python -m pytest`), including
      `test_sim_otos_lever_arm.py`, `test_ekf_odometry_commands_coverage.py`,
      and the `SI`/`ZERO` command tests.

## Testing

- **Existing tests to run**: `test_sim_otos_lever_arm.py`,
  `test_ekf_odometry_commands_coverage.py`, the `SI`/`ZERO`/`OV` command
  tests, any existing `Drive`/`Odometry` unit tests, full default suite.
- **New tests to write**:
  - `SET tw=<x>` / `SET rotSlip=<x>` live-update regression test asserting
    `Odometry::predict()`'s next-tick output changes by the expected amount
    (fresh `Sim()`/`ZERO enc`, per sprint 067's methodology).
  - Narrowed `getPose(const PoseEstimate&, ...)` signature coverage.
  - No-reset-on-`SET` test: drive to a non-origin pose, `SET` a kinematics or
    noise key, assert fused pose/velocity unchanged immediately after.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Narrow every `PhysicalStateEstimate`/`Odometry` method
signature from "take the whole `HardwareState&`" to "take exactly the inputs
read and exactly the `PoseEstimate&` output(s) written," per-call rather than
bound once at construction (Decision 3 — required because `resetPose`/`zero`
have two independent live destinations). Route trackwidth/rotational slip
through a new `setKinematics()` setter called every tick from `Drive`
(single destination, so "set once, refreshed live" is correct there). Delete
`setCtx` and the four confirmed-zero-caller forwarders outright. Update the
five call-site families (`Drive.cpp`, `Robot.cpp`, `SystemCommands.cpp`,
`Planner.cpp`) to the new signatures. Leave `Odometry::correct()` untouched
(dead, out of scope — Decision 6).

Sequenced last in this sprint (no file-level dependency on tickets 001/002)
because it is the largest, highest-attention change and benefits from the
smaller tickets' green-suite confidence first.

**Files to modify**:
- `source/state/PhysicalStateEstimate.h`, `PhysicalStateEstimate.cpp`
- `source/control/Odometry.h`, `Odometry.cpp`
- `source/subsystems/drive/Drive.cpp`
- `source/robot/Robot.cpp`
- `source/commands/SystemCommands.cpp`
- `source/superstructure/Planner.cpp` (`getPoseFloat`)

**Testing plan**: run the `Odometry`/`PhysicalStateEstimate` unit tests and
the OTOS/EKF-coverage tests in isolation first, then `SI`/`ZERO`/`OV`
command tests, then the full suite. A `--clean` sim build is required first,
since these are ARM-target-and-sim-shared source files (project knowledge:
stale incremental builds on `/Volumes` — build banners lie).

**Documentation updates**: none required — no `docs/` file describes
`PhysicalStateEstimate`'s method signatures at this level of detail (the
architecture-update.md itself, already written, is the record of this
change).
