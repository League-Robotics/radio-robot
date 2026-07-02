---
id: '006'
title: Gate OTOS fusion on warn-bit persistence with sim ABI support
status: done
use-cases:
- SUC-004
depends-on: []
github-issue: ''
issue: otos-warn-bit-fusion-spin-on-placement-regression.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Gate OTOS fusion on warn-bit persistence with sim ABI support

## Description

CR-06 (high). `Robot::otosCorrect` (`source/robot/Robot.cpp:168-295`)
documents a two-tier D9 gate (READABLE — is there a usable reading at all;
HEALTHY — is it good enough to fuse), but a 2026-06-17 change collapsed it to
`bool healthy = poseOk;`, with the legitimate rationale that benign,
*transient* WARNING bits shouldn't drop fusion entirely. The implementation
lost the transient-vs-persistent distinction: a robot with
`warnOpticalTracking` set *persistently* (lifted, on the stand, freshly
placed) now has its frozen pose and near-zero velocity fused every tick.
`EKFTiny`'s own gate-recovery (`EKFTiny.cpp:217-250`, `:408-437`) rejects the
frozen observation temporarily but then force-snaps fused position/heading
to it after 10 consecutive rejections — reopening the exact "spin on
placement" failure the original D9 gate (027-005) existed to prevent.

Fix: restore the persistence distinction entirely upstream of `EKFTiny`
(which stays unchanged — its gate-recovery mechanism is independently useful
for genuinely transient sensor noise elsewhere and is not the defect here).
`Robot::otosCorrect` tracks consecutive warn/clean ticks and blocks
`addOtosObservation` once a warn streak persists past a threshold, re-
admitting after a run of clean ticks. `SimOdometer` gains a
"warn-bit-set-but-readable" state (currently only `setLift`/`setReadFailure`
model the fully-unreadable case) so the gate is testable in the sim tier,
following the existing `setReadFailure`/`sim_set_otos_read_failure` pattern
exactly. See `architecture-update.md` Step 4-5 item 6 and Design Rationale
Decision 5 for the full design, including the chosen K/N constants and why
they are fixed (not config-tunable).

Independent of tickets 001-005 (disjoint files: `Robot.{h,cpp}`,
`SimOdometer.{h,cpp}`, sim ABI). No functional dependency.

## Acceptance Criteria

- [x] `source/robot/Robot.h` gains three new private members:
      `uint8_t _otosWarnStreak = 0;`, `uint8_t _otosCleanStreak = 0;`,
      `bool _otosFusionBlocked = false;`, plus `static constexpr uint8_t
      kOtosWarnPersistK = 3;` and `static constexpr uint8_t
      kOtosCleanReadmitN = 5;` — alongside the existing
      `_otosInvalidStartMs`/`_otosLostEmitted` (unaffected).
- [x] `Robot::otosCorrect()`'s existing unreadable-path branch (`!healthy`)
      is unchanged. Immediately after it (i.e. only once the reading is
      readable), a WARNING bit (`otosStatus != 0`, with HARD errors already
      excluded by `readable`) increments `_otosWarnStreak` and sets
      `_otosFusionBlocked = true` once the streak exceeds
      `kOtosWarnPersistK`; a clean tick (`otosStatus == 0`) resets
      `_otosWarnStreak` and, if blocked, increments `_otosCleanStreak`,
      clearing `_otosFusionBlocked` once it reaches `kOtosCleanReadmitN`.
- [x] When `_otosFusionBlocked` is true, `otosCorrect()` returns before
      calling `addOtosObservation()`. Raw telemetry
      (`state.actual.optical.pose`, `otos.valid`) is unaffected — it is
      already written earlier in the function, before this gate.
- [x] `source/state/EKFTiny.{h,cpp}` is **not** modified by this ticket.
- [x] `source/hal/sim/SimOdometer.h`/`.cpp` gain `setWarnOptical(bool on)`
      (mirrors `setLift`). `readStatus()` reports `out = 0x02` (
      `warnOpticalTracking`), `return true` (readable) when
      `_warnOptical` is true and `_lift`/`_readFailure` are false. `tick()`
      skips the odometry-accumulator update and zeros `_velV`/`_velOmega`/
      `_accAx`/`_accAy` while `_warnOptical` is true, modeling "frozen pose,
      near-zero velocity" while wheels keep turning (encoders, driven
      independently by true wheel velocity, are unaffected).
      `readTransformed()`/`readVelocityTransformed()` are otherwise
      unchanged (still return `true`/readable). Default `_warnOptical =
      false` — no behavior change for any existing test.
- [x] `tests/_infra/sim/sim_api.cpp` gains `sim_set_otos_warn(void* h, int
      on)` (mirrors `sim_set_otos_read_failure`). `tests/_infra/sim/
      firmware.py` gains `Sim.set_otos_warn(on: bool)` (mirrors
      `set_otos_read_failure`).
- [x] New sim test (in or alongside `tests/simulation/unit/
      test_fusion_validation.py`): with the warn bit persistently set
      (`sim.set_otos_warn(True)`) and wheels commanded to spin, fused pose
      tracks encoder-derived odometry — no snap to the frozen OTOS pose,
      even after 10+ ticks (past `EKFTiny`'s own gate-recovery threshold).
- [x] New sim test: a 1-2 tick warn blip (`set_otos_warn(True)` then
      `False)` within `kOtosWarnPersistK`) does not interrupt fusion — fused
      pose continues to track the (otherwise healthy) OTOS reading normally.
- [x] New sim test: after a persistent-warn block, `kOtosCleanReadmitN`
      consecutive clean ticks re-admit fusion.
- [x] `tests/simulation/unit/test_dbg_otos_commands.py` and
      `test_n8_n9_sensor_freshness.py` stay green (raw telemetry visibility
      unchanged).
- [x] Full default sim suite green.

## Implementation Note — live-path scope correction (found during implementation)

The architecture-update and this ticket's plan target `Robot::otosCorrect()`
as *the* OTOS→EKF fusion call site. Reading the call graph
(`source/robot/LoopTickOnce.cpp`) during implementation showed this is
**stale post-060-005**: `Robot::otosCorrect()` has zero callers anywhere in
`source/` or `tests/_infra/sim/` — the legacy loop that called it was
deleted in the 060 ordered-tick cutover. The sole live OTOS→EKF fusion path
(both real firmware `run_blocks()`/`run_test()` and the sim's `sim_tick()`,
both via `loopTickOnce()` → `robot.drive.tickUpdate()`) is
`subsystems::Drive::tickUpdate()`'s STEP 5, in `source/subsystems/drive/
Drive.cpp` — an entirely separate `addOtosObservation()` call site with no
warn-bit gating of its own.

Gating only `Robot::otosCorrect()` per the letter of the ticket's file list
would have been a fix to dead code with zero effect on the sim tests, HITL
behaviour, or the "spin on placement" regression this ticket exists to
close. Confirmed by temporarily bypassing the `Drive.cpp` gate and re-running
the new tests: `test_persistent_warn_blocks_fusion_no_snap` failed hard
(112 mm fused/encoder drift vs. a 20 mm threshold, reproducing the exact
snap-to-frozen-pose symptom) — proving the `Drive.cpp` gate, not the
`Robot.cpp` one, is what the acceptance tests actually exercise and what
fixes the regression.

Both are gated, as the ticket's literal file list plus the dispatch
instructions' explicit "gate the live path (both if both are live)"
directive requires:

- `source/robot/Robot.{h,cpp}` — implemented exactly per the acceptance
  criteria above. Currently unreachable (dead code) pending any future
  re-wiring that restores a caller; kept for API parity / documentation and
  because the ticket's ACs specify it explicitly.
- `source/subsystems/drive/Drive.{h,cpp}` — **new, beyond the ticket's
  original file list** — the operative live-path fix. Same constants
  (`kOtosWarnPersistK=3`, `kOtosCleanReadmitN=5`), same member names and
  state-machine shape as `Robot::otosCorrect()`'s gate, added as a private
  `Drive::_updateOtosFusionGate(bool warnBit)` helper invoked from STEP 5
  after a successful `readTransformed()`. Raw telemetry (`_hw.otos.valid`/
  `lastUpdMs`) is written unconditionally, matching `Robot::otosCorrect()`'s
  contract of telemetry-visibility-unaffected-by-fusion-gating.

## Implementation Plan

**Approach**: A small, self-contained persistence-counter state machine
inside `Robot::otosCorrect`, gating the single existing call to
`addOtosObservation`. No `EKFTiny` change. Sim ABI addition follows the
`setReadFailure` template exactly (same shape, same file touch pattern) per
sprint 064's established precedent for this class of fault-injection
addition.

**Files to modify**:
- `source/robot/Robot.h` — three new members + two new `constexpr`
  thresholds.
- `source/robot/Robot.cpp` — `otosCorrect()`: insert the persistence-gate
  logic between the existing unreadable-path branch and the
  `addOtosObservation()` call.
- `source/hal/sim/SimOdometer.h` — `setWarnOptical(bool)`, `_warnOptical`
  member, `readStatus()` inline update.
- `source/hal/sim/SimOdometer.cpp` — `tick()`: skip accumulation, zero
  velocity/accel, when `_warnOptical`.
- `tests/_infra/sim/sim_api.cpp` — `sim_set_otos_warn`.
- `tests/_infra/sim/firmware.py` — `Sim.set_otos_warn()`.

**Testing plan**:
- Three new sim tests as listed in Acceptance Criteria (persistent-warn
  block, transient-blip no-op, clean-streak re-admission), likely added to
  `tests/simulation/unit/test_fusion_validation.py` alongside existing D9
  -adjacent coverage.
- Run `test_dbg_otos_commands.py`, `test_n8_n9_sensor_freshness.py`, and the
  full default sim suite to confirm no regression in raw telemetry
  visibility or existing OTOS-readable-path behavior.

**Documentation updates**: `architecture-update.md` already documents this
change (Step 4-5 item 6, Design Rationale Decision 5, Open Question 1). No
wire-protocol change — `setWarnOptical`/`sim_set_otos_warn` are sim-only
ABI, not reachable from the robot wire protocol.
