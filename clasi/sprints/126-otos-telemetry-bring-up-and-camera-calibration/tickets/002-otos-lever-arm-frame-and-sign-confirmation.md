---
id: '002'
title: OTOS lever-arm frame and sign confirmation
status: open
use-cases: [SUC-002]
depends-on: ['001']
github-issue: ''
issue: otos-telemetry-bring-up-and-camera-calibration.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# OTOS lever-arm frame and sign confirmation

## Description

Determine whether the OTOS pose reported in telemetry is the robot's
**centre of rotation** or the raw **chip location**, and confirm the
configured lever arm (`data/robots/tovez.json`'s `geometry.odometry_offset_mm`
= {x: -47.7, y: 3.5, yaw_rad: 0.0}) is correct in magnitude AND sign
against whichever frame it turns out to be.

Note for context (do not treat as the answer — the issue is explicit that
this must be settled by measurement, not by reading code): a reading of
`RealOtos::tick()` (`src/firm/devices/otos.cpp`) shows it already applies
a `sensorToCentre()` transform using the configured offset before caching
the pose, which is consistent with telemetry already reporting the
robot-centre frame. This ticket exists to prove or disprove that with a
real rotation, not to take the code's word for it.

Add `--mode lever-arm` to `otos_calibration_bench.py` (the script started
in ticket 001):

1. Preflight (lights, camera bring-up) exactly as ticket 001's mode does
   — reuse the same helper, do not duplicate it.
2. From a tether-safe rest orientation, capture a camera-truth rest pose.
3. Command **one** in-place rotation (magnitude of your choosing, large
   enough to be unambiguous — e.g. 90 deg — direction chosen to respect
   the tether rule for a single turn: east→west through north, never
   through south).
4. Capture camera-truth rest pose after settle.
5. Read the OTOS pose trajectory across the turn (multiple telemetry
   frames spanning the rotation, not just before/after).
6. Test the two hypotheses against that trajectory:
   - **Centre-frame hypothesis**: the reported (x, y) barely moves (the
     robot's centre of rotation is approximately stationary during an
     in-place turn); heading changes by the turn's camera-measured delta.
   - **Chip-frame hypothesis**: the reported (x, y) traces roughly a
     circular arc of radius `|odometry_offset_mm|` (≈47.8 mm, from
     `sqrt(47.7² + 3.5²)`) centred near the robot's centre of rotation.
   Whichever hypothesis the trajectory matches settles the frame.
7. If centre-frame: confirm the reported (x, y) stayed within a stated,
   small bound of stationary (state the bound and the observed value).
   If chip-frame: fit the observed arc's radius and centre-offset
   direction against the configured `odometry_offset_mm`'s magnitude and
   sign, and report the fit.
8. Print a clear frame/lever-arm report: which frame, the supporting
   trajectory data, and whether magnitude/sign are confirmed correct or a
   mismatch is found. **Do not correct `odometry_offset_mm` in this
   ticket** even if a mismatch is found — report it; correction is out of
   this sprint's scope (see sprint.md Scope). If a mismatch is found,
   flag it clearly in the completion notes as a candidate follow-up issue
   for the stakeholder.
9. `estop()` (never `stop()`) in a `finally` block.

## Acceptance Criteria

- [ ] `otos_calibration_bench.py --mode lever-arm` exists, reusing ticket
      001's preflight/camera helpers.
- [ ] A single in-place rotation is captured with camera truth at rest
      before/after and multiple OTOS telemetry samples spanning the turn.
- [ ] The script determines, from that trajectory, whether telemetry
      reports centre-frame or chip-frame, and prints the determination
      with supporting numbers.
- [ ] The configured `odometry_offset_mm` magnitude and sign are checked
      against the determined frame and the result (confirmed or
      mismatched) is printed explicitly — no silent correction is made.
- [ ] Every connection path calls `estop()` (never `stop()`) in a
      `finally` block.
- [ ] Run against the real robot on the playfield; raw results included in
      the ticket's completion notes.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim -q`.
- **New tests to write**: None — hardware/playfield measurement script,
  not pytest-collected.
- **Verification command**: `uv run python src/tests/bench/otos_calibration_bench.py --port <robot-port> --mode lever-arm`
