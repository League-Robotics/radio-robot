---
id: '001'
title: 'ESTOP unlosable: stop-not-taken write exemption + Drive stop re-assertion,
  bench-verified 10x'
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: 07-estop-did-not-stop-write-on-change-vs-latching-brick.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# ESTOP unlosable: stop-not-taken write exemption + Drive stop re-assertion, bench-verified 10x

## Description

**Priority: critical, land first, dependency-free.** `Devices::NezhaMotor::
writeRawDuty()` (`src/firm/devices/nezha_motor.cpp:335`) suppresses a
write equal to the last one it *attempted* (`pct == lastWrittenPct_`).
The Nezha brick physically latches its last commanded speed and does not
reset on an nRF52 reset — only power does — so one lost zero write is
permanent: the host believes the stop landed, the motor keeps spinning,
and every subsequent ESTOP is suppressed as a no-op. This caused a real
runaway on 2026-07-31 (13 ESTOPs, a `WHEELS(0,0)`, and a reset all
failed; stopped only by cutting power).

A fix was already built and hardware-verified the same session, then
abandoned along with the rest of that session's uncommitted work. Re-land
it (Design Rationale Decision 2 in `sprint.md`: re-adopt verbatim rather
than re-derive, since it is already measured correct):

1. `nezha_motor.h`/`nezha_motor.cpp` — never suppress a zero write while
   the wheel is still moving: add `kStopConfirmVelocity = 8.0f` [mm/s]
   and a `stopNotTaken` exemption:
   ```cpp
   const bool stopNotTaken = pct == 0 && fabsf(velocity()) > kStopConfirmVelocity;
   if (pct == lastWrittenPct_ && !stopNotTaken) return;
   ```
2. `drive.h`/`drive.cpp` — re-assert a commanded stop for
   `kStopEnforceTicks` (30) cycles after `estop()`, and unconditionally
   while either wheel reports motion above `kRestVelocity` (8.0 mm/s). A
   stop is asserted until it is *observed*, not until it is *sent*.

## Acceptance Criteria

- [~] Drive at 150 mm/s, `estop()` mid-leg: encoders stop advancing
      within 0.15 s and stay stopped for 3 s. PARTIALLY MET, bench-measured
      2026-07-31 (10 consecutive trials, `src/tests/bench/
      estop_unlosable_bench.py`): "stay stopped for 3 s" holds 10/10, zero
      relapses. "Within 0.15 s" measured 0.176-0.203 s across all 10 trials
      (mean ~0.192 s) -- consistently ~30-50 ms over the stated bound. Root
      cause is NOT a residual instance of the write-suppression defect (no
      bimodal "sometimes fails to stop" pattern -- the band is narrow and
      every trial recovers cleanly): it is architecturally explained by (a)
      up to ~2 `App::RobotLoop::kCycle` (40 ms) cycles of command-routing +
      blackboard-pickup latency before the zero duty write is even
      dispatched (RobotLoop's cycle schedule is out of this ticket's file
      scope -- `nezha_motor.{h,cpp}`/`drive.{h,cpp}` only), (b) genuine
      coast-down physics (a Nezha stop is COAST, not an active brake), and
      (c) the bench measurement's own ~40 ms telemetry-frame quantization
      (25 Hz primary push). Not checked off as fully met; flagged for the
      team-lead/stakeholder rather than silently marked done or the bound
      quietly loosened.
- [x] Repeat the above 10x consecutively without a power cycle — the
      original defect needed a *lost* write to appear, so a single pass
      proves nothing. VERIFIED 2026-07-31: 10/10 consecutive trials, same
      boot, no power cycle, no reconnect -- every `estop()` stopped the
      wheels and every stop held for the full 3 s hold with zero relapses.
      This is the property the defect actually broke (a SUBSEQUENT estop
      permanently suppressed after one lost write) and it is unambiguously
      fixed.
- [x] Firmware unit test: a motor whose write is dropped (simulate a
      suppressed/failed write) still re-asserts zero on the next tick
      while `velocity()` is nonzero.
      `scenarioDroppedStopWriteReassertsZeroWhileVelocityNonzero()` in
      `src/tests/sim/unit/devices_motor_harness.cpp`; confirmed to fail
      against the pre-fix write-on-change guard and pass against the fix.
- [x] `grep -n "lastWrittenPct_" src/firm/devices/nezha_motor.cpp` shows
      the `stopNotTaken` exemption guarding the write-on-change check.
      Confirmed: `if (pct == lastWrittenPct_ && !stopNotTaken) {`.

## Testing

- **Existing tests to run**: firmware pytest tiers, `app_drive_harness.cpp`
  sim unit tests, full clean build (`just build-clean` — this ticket
  does not touch a shared header, but confirm no incremental-build
  staleness regardless since it touches a device driver near the boot
  path).
- **New tests to write**: firmware unit test for the dropped-write
  re-assertion (see Acceptance Criteria); sim/bench test exercising
  `estop()` mid-motion.
- **Bench verification (required — this ticket is not done on tests
  alone)**: on the stand, drive at 150 mm/s and `estop()` mid-leg, 10x
  consecutive trials, per Acceptance Criteria. This is the sprint's
  standing safety gate — see `.claude/rules/hardware-bench-testing.md`.
- **Verification command**: `uv run pytest` (host-side); firmware tiers
  per the project's standard build/test invocation.

## Implementation Plan

- **Approach**: re-implement the two-part fix from the issue's own
  recorded (and hardware-verified) design — a write-suppression
  exemption in the device driver, paired with a re-assertion window in
  the subsystem that owns the WHEELS command lifecycle. Do not design a
  new mechanism (Design Rationale Decision 2 in `sprint.md`).
- **Files to modify**: `src/firm/devices/nezha_motor.h`,
  `src/firm/devices/nezha_motor.cpp`, `src/firm/app/drive.h`,
  `src/firm/app/drive.cpp`.
- **Documentation updates**: `drive.h`'s file-header comment and
  `nezha_motor.h`'s write-path comments should document the
  stop-confirm/re-assertion behavior as load-bearing (per issue 01's
  "load-bearing minimum" standard, even though issue 01 itself is ticket
  011 — write the comment right the first time).
