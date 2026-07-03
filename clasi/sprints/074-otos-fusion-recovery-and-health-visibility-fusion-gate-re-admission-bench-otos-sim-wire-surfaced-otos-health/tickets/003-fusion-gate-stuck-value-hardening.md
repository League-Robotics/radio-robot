---
id: '003'
title: Fusion-gate stuck-value hardening
status: open
use-cases:
- SUC-003
depends-on:
- '002'
github-issue: ''
issue: otos-not-used-frozen-pose-ekf-rejects-everything.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fusion-gate stuck-value hardening

## Description

The issue's HITL recording shows TLM `otos=` frozen for an entire session
while `ekf_rej` climbed almost every tick. `Drive::_updateOtosFusionGate`
(CR-06, sprint 065, `Drive.cpp:412-430`) already re-admits fusion after a
clean-tick run — that gate is proven correct and is NOT the bug (already
regression-tested by `test_otos_warn_persistence.py`). The actual gap: the
gate's only "is this healthy" input is the OTOS chip's self-reported STATUS
byte (`_otos.readStatus(otosStatus)`, `Drive.cpp:159`, now `_hal.otos().
readStatus(...)` per ticket 002). It has no check that the pose VALUE is
actually changing. A reading that is readable, reports a clean STATUS byte
(`otosStatus == 0`), and simply stops updating sails through undetected and
gets fused into the EKF every tick — the only mechanism that explains BOTH
symptoms at once: a blocked gate would stop `ekf_rej` from climbing (fusion
skipped means nothing to reject), so a climbing `ekf_rej` alongside a frozen
`otos=` implies the frozen reading kept being fed to the EKF, which kept
rejecting it.

Fix: add a per-tick boolean — "the newly read pose is unchanged from the
previous tick's pose (within a small epsilon, position AND heading) AND the
robot shows encoder-evidenced motion this tick" — and OR it into the
existing `warnBit` passed to `_updateOtosFusionGate(bool warnBit)`
(`Drive.cpp:160`). The function body, its streak counters
(`_otosWarnStreak`/`_otosCleanStreak`), and the re-admission threshold
(`kOtosCleanReadmitN`) are UNCHANGED — this is strictly an additional input
to an already-correct, already-tested state machine, not a new gate and not
a rewrite (Design Rationale Decision 3).

"Encoder-evidenced motion" is available cheaply and already computed every
tick: `_hw.vel[0]`/`_hw.vel[1]` (per-wheel velocity, mm/s, populated by
`controlTick()` in STEP 1+2, which runs before STEP 5 in the same
`tickUpdate()` call — `Drive.cpp:76-87` vs. `134-183`). This avoids
introducing a second encoder-delta bookkeeping mechanism alongside the one
`Odometry`/`_est.addOdometryObservation()` already owns.

See `architecture-update.md` Step 3 "Module: Drive OTOS consumption" (the
`_updateOtosFusionGate` boundary), Step 5 item 4, Design Rationale Decision
3 (why extend the existing gate, not add a parallel one), Open Question 4
(threshold values are HIL-tunable starting points, not yet bench-validated);
`usecases.md` SUC-003.

## Acceptance Criteria

- [ ] `Drive` (`Drive.h`, near the existing OTOS gate members at lines
      164-166) gains: `float _prevOtosX = 0.0f, _prevOtosY = 0.0f,
      _prevOtosH = 0.0f;` and `bool _prevOtosValid = false;` (previous
      tick's successfully-read OTOS pose, for tick-to-tick comparison — NOT
      persisted across a read failure; see below). Three new named
      constants alongside `kOtosWarnPersistK`/`kOtosCleanReadmitN`:
      `kOtosStuckPosEpsMm` (position epsilon, mm), `kOtosStuckHeadEpsRad`
      (heading epsilon, rad), `kOtosStuckEncMotionMmps` (per-wheel velocity
      threshold above which the robot is considered "commanded to move,"
      mm/s). Starting values proposed at implementation time (e.g. 0.5mm,
      ~0.01 rad, 5mm/s) — flagged in `architecture-update.md` Open Question
      4 as HIL-tunable, not yet bench-validated; document the chosen values
      and rationale inline as a comment, matching this file's existing
      practice for `kOtosWarnPersistK`/`kOtosCleanReadmitN`.
- [ ] Inside STEP 5's `poseOk` branch (`Drive.cpp:149-181`), after reading
      `p`/`headingRad` via `_hal.otos().readTransformed(...)` and BEFORE
      overwriting `_prevOtos*`, compute: `bool encMotion =
      (fabsf(_hw.vel[0]) > kOtosStuckEncMotionMmps) ||
      (fabsf(_hw.vel[1]) > kOtosStuckEncMotionMmps);` and `bool otosStuck =
      _prevOtosValid && encMotion && (fabsf(p.x - _prevOtosX) <
      kOtosStuckPosEpsMm) && (fabsf(p.y - _prevOtosY) < kOtosStuckPosEpsMm)
      && (fabsf(p.h - _prevOtosH) < kOtosStuckHeadEpsRad);`. Update
      `_prevOtosX/Y/H = p.x/p.y/p.h; _prevOtosValid = true;` unconditionally
      whenever a read succeeds (whether or not `otosStuck` fired this tick —
      the comparison is always tick-to-tick, not against the first-ever
      value).
- [ ] The call `_updateOtosFusionGate(!statusOk || (otosStatus != 0));`
      (`Drive.cpp:160`) becomes `_updateOtosFusionGate(!statusOk ||
      (otosStatus != 0) || otosStuck);`. No other line inside
      `_updateOtosFusionGate`'s own body changes.
- [ ] On a READ FAILURE (`poseOk == false`, `Drive.cpp:176-179`),
      `_prevOtosValid` is left UNCHANGED (not reset to false, not updated to
      a bogus value) — matching the existing `_hw.otos.valid` "preserve
      last-known-good" convention on the same branch, and ensuring a
      transient read failure doesn't spuriously arm or disarm the
      staleness check on the next successful read.
- [ ] A robot that is legitimately stationary (both `_hw.vel[]` below
      `kOtosStuckEncMotionMmps`) with an unchanging OTOS reading is NEVER
      flagged `otosStuck`, regardless of how long the value has been
      static — `encMotion` gates the whole check. This must be exercised by
      a dedicated test (see Testing) — do not rely on inference from the
      other tests.
- [ ] New sim test: `sim.set_otos_fusion(True)`, `sim.set_otos_pose(x, y,
      h)` ONCE (a fixed value — `readStatus()` stays clean, `readTransformed()`
      returns this same static value every subsequent tick per `SimOdometer`'s
      documented injected-pose behavior when `enableSimModel` is off), then
      drive (`VW 200 0`) for well past `kOtosWarnPersistK` control ticks.
      Assert: fused pose tracks the encoder estimate (mirrors
      `test_persistent_warn_blocks_fusion_no_snap`'s drift assertion, < 20mm),
      and `sim.get_ekf_rej_count()` stops climbing once the block engages
      (compare the count over two later equal-length tick windows — the
      second window's delta must be ~0, unlike the first).
- [ ] Same test, recovery phase: stop (`X`) — this drops `encMotion` to
      false, which disarms the staleness check regardless of the frozen
      injected pose, so `_otosWarnStreak` resets and `_otosCleanStreak`
      accumulates every stopped tick (mirrors
      `test_clean_streak_readmits_fusion_after_block`'s exact phase-2/3
      structure) — then inject a large OTOS offset via `sim.set_otos_pose(enc_x
      + 200, 0, 0)` once `kOtosCleanReadmitN` clean ticks have re-admitted
      fusion, and confirm the fused pose is subsequently pulled toward the
      injected offset (same `fused_pull > 50.0` style assertion as the
      existing STATUS-bit re-admission test).
- [ ] Existing `test_otos_warn_persistence.py`'s three tests pass UNMODIFIED
      — the STATUS-bit path is untouched; the new check is an additional,
      independently-gated input to the same `warnBit`/state machine.
- [ ] Full suite (`uv run python -m pytest`) passes at the running baseline
      (2672 + tickets 001/002's net additions) + this ticket's net new test
      count, zero unexplained failures. The `data/robots` drift noted in
      the sprint's hard contract is environmental — do not chase or touch
      it.

## Testing

- **Existing tests to run**: `tests/simulation/unit/test_otos_warn_persistence.py`
  (all three, by name, confirming zero interference with the STATUS-bit
  path), full suite.
- **New tests to write**: the stuck-value block + re-admission test
  described above (likely a new file, e.g.
  `tests/simulation/unit/test_otos_stuck_value_gate.py`, mirroring
  `test_otos_warn_persistence.py`'s structure and docstring style closely
  enough that a future reader can see the two gates are siblings), plus a
  narrow "stationary robot with a frozen OTOS reading is never flagged"
  test (drive to a stop, hold the SAME injected OTOS pose across many
  ticks with zero encoder motion, assert fusion is never blocked and
  `ekf_rej` behaves as it would with a perfectly healthy sensor).
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Add the new members/constants to `Drive.h` first. Implement
the `encMotion`/`otosStuck` computation and the `_prevOtos*` update inside
STEP 5's existing `poseOk` branch in `Drive.cpp`, taking care that the
comparison uses the PREVIOUS tick's stored value (read before the update,
not after). Widen the single `_updateOtosFusionGate(...)` call site with an
`||`. Do not touch `_updateOtosFusionGate`'s own body. Write the new test
file mirroring `test_otos_warn_persistence.py`'s phase structure, adapting
only the "how the warn condition is induced" step (`sim.set_otos_pose(...)`
once instead of `sim.set_otos_warn(True)`).

**Files to create/modify**:
- `source/subsystems/drive/Drive.h` — new members/constants.
- `source/subsystems/drive/Drive.cpp` — STEP 5 staleness computation +
  widened gate call.
- New test file `tests/simulation/unit/test_otos_stuck_value_gate.py`.

**Testing plan**: run the new test file in isolation first, then
`test_otos_warn_persistence.py` by name (regression-of-siblings check),
then the full suite. Manually sanity-check the chosen epsilon/threshold
constants against a plain `VW`/`RT` drive in sim (no injection) to confirm
normal driving never spuriously trips `otosStuck` against a healthy
(non-frozen) `SimOdometer` reading — this is the "no false positive during
ordinary operation" check the HIL follow-up (Open Question 4) will later
validate against real sensor noise.

**Documentation updates**: `Drive.cpp`'s STEP 5 comment block gains a note
describing the staleness check alongside the existing CR-06 STATUS-bit
comment, and a cross-reference to `otos_health=`'s `fusionBlocked` bit
(ticket 004) as the wire-visible symptom of either warn source.
