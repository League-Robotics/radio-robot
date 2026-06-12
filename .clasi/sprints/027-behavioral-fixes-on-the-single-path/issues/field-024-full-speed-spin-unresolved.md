---
status: in-progress
sprint: '027'
tickets:
- 027-006
---

# Field test of sprint 024: robot still does a full-speed spin (root cause unresolved)

Captured at the close of sprint 024 so the finding isn't lost. The 024 code
(PRE_ROTATE bounding, watchdog re-role, EKF heading fusion, gate recovery, slip)
is implemented and all 1434 host/dev tests pass, but the **on-field behavior is
not fixed** and needs more design before another sprint.

## What was verified on hardware (2026-06-11)
- 024 firmware flashed + booted clean: `GET ekfRHead` is recognized (024-only key);
  OTOS alive; EKF fused pose tracks OTOS at rest.
- Driving `G` to a colored box (`square_run.py --boxes black-N`) produced a
  **full-speed spin** that ended with the robot jammed into the boards. Same class
  of failure the sprint set out to fix.

## What is NOT the cause (verified — do not re-chase these)
- **Encoders are healthy.** `tests/dev/enc_watch.py` was run 3× and both wheels
  count cleanly (e.g. L 0→211, R 0→207; second run L 193→405, R 230→417). The
  `enc=0` seen in the square_run log is NOT a frozen encoder. See
  [[dont-reflexively-blame-frozen-encoders]].
- **PRE_ROTATE termination does stop the motors.** `MotionController.cpp` ~line 839
  (the TIME-net branch of the `!still_running` block) calls
  `_mc.stop(); _bvc.reset(); _mode = IDLE`. So "command ends without stopping
  motors" is not it.

## The real open anomalies (for the design round)
1. **SNAP telemetry reported `enc=0` AND `mode=IDLE` while the robot was physically
   spinning at full speed.** The STREAM/TLM `.enc` path reads fine (enc_watch), so
   this is a SNAP-specific discrepancy — investigate SNAP vs STREAM TLM frame
   building (note 024-005 changed `buildTlmFrame`). `mode=IDLE` while the motors
   spin is the key smell.
2. **The bench program abandons the autonomous `G` without stopping it.**
   `square_run.py` declared "reached" off a (wrong) fused pose and exited; the
   firmware `G` runs autonomously, so the robot kept spinning until manually X'd.
   Either the host must `X` on exit / on its own arrival decision, or the firmware
   pre-rotate genuinely ran away — needs to be disentangled with live ground truth.
3. **Why did the pre-rotate spin at full speed at all?** The G pre-rotate gates on
   the fused heading; characterize the actual fused heading during a hardware
   pre-rotate (with the camera as truth) to see whether the gate ever closes and
   whether the 024 ramp/TIME-net behaved as designed on real hardware.

## Tooling added this session
- `tests/dev/enc_selftest.py` — per-wheel live encoder probe (reads ENC off the
  TLM stream during commanded motion; reports per-wheel and flags the
  jammed-wheel/idle-silence confounds). Use the stand (wheels free).

Related: [[wild-spin-root-cause-prerotate-no-timeout]], the sprint-024 architecture
update, and the sim2real review defects (D8 pursuit, D9 OTOS validity, D10 telemetry).

## Resolution (027-006, 2026-06-11)

### Lead B — host abandons G without X

**Closed by 027-002 (bench runaway wrapper).** `square_run.py` (the program that
triggered the field-024 failure) wraps its drive loop in `BenchRun` (line 211),
which sends `X` on exit/exception/timeout.  Verified in 027-006: the `BenchRun`
import and context manager are present in `tests/bench/square_run.py`.

Manual Ctrl-C test (robot stops mid-run): **DEFERRED — stakeholder field test.**

### Lead A — SNAP TLM discrepancy (enc=0, mode=IDLE while spinning)

**Deferred to sprint 028 — D10 work required.** Investigated in 027-006.

**Finding:** SNAP and STREAM share the **exact same** `buildTlmFrame()` call path
(`Robot::handleSnap` → `robot->buildTlmFrame()` vs `Robot::telemetryEmit` →
`buildTlmFrame()`).  Both read the same `state.inputs` struct and
`motionController.mode()`.  The 024-005 commit only added the `ekf_rej` field
and expanded the TLM buffer from 128→160 bytes; it made no structural change to
how either SNAP or STREAM reads state.

**Root cause of the field anomaly (not a code bug):** SNAP fires via
`cmd.dequeueOne()` at the START of the `loopTickOnce()` tick body, BEFORE
`driveAdvance()` has run in that tick.  This is a tick-ordering limitation of the
cooperative loop architecture (see `LoopTickOnce.cpp`: dequeueOne → watchdog →
halt → driveAdvance → odometry → ... → telemetry).  In the field-024 incident,
the sequence was:

1. PRE_ROTATE completed and set mode=IDLE + zeroed enc via `distanceDrive()`.
2. The transition to the next phase was enqueued but `driveAdvance()` hadn't
   yet run in the new tick.
3. SNAP was dispatched (dequeueOne) before `driveAdvance()` fired → saw
   mode=IDLE, enc=0 while the physical wheels were still spinning (hardware
   inertia + the enqueued command not yet started).

This is not fixable with a one-liner.  Properly sequencing SNAP to always
reflect post-driveAdvance state requires D10 work: per-command epoch markers or
seq numbers so the host can correlate SNAP frames to motion phases.  Cross-
reference: **sprint 028, ticket 028-001**.

A sim test (`host_tests/test_snap_tlm.py`) was added confirming that SNAP
correctly reports live state (non-IDLE mode, non-zero enc) after `driveAdvance()`
has advanced — the normal case when SNAP is not racing a transition boundary.
