---
id: 008
title: Bench + playfield bring-up and calibration (HITL)
status: done
use-cases:
- SUC-003
- SUC-004
- SUC-005
depends-on:
- 046-007
github-issue: ''
issue: ''
completes_issue: true
hardware-in-the-loop: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 046-008: Bench + playfield bring-up and calibration (HITL)

## Description

Final integration, calibration, and camera verification of the mecanum robot.
This ticket has two phases:

- **Programmer deliverable**: extend `tests/bench/playfield_camera_run.py`
  with a strafe leg.
- **Team-lead HITL**: measure geometry, calibrate per-wheel mmPerDeg and OTOS
  scalars, run the playfield camera verification, and update the robot JSON
  with all calibrated values.

All three motion primitives (forward, turn, strafe) must be camera-verified.

## Programmer Deliverable

### Extend tests/bench/playfield_camera_run.py with a strafe leg

Read the existing `playfield_camera_run.py` to understand the test structure
(how it sequences commands, waits, and reads camera ground-truth). Add a
strafe leg after the existing forward + turn legs:

```python
# Strafe leg: command STRAFE 150 for 2 seconds, then stop.
# Camera assertion: measured lateral displacement > 150mm; forward drift < 15% of lateral.
relay.send("STRAFE 150 t=2.0")
time.sleep(2.5)
relay.send("STOP")
# Read camera pose: compare final_x/y to initial_x/y.
# Assertion: |delta_y| > 150mm * 0.8 (80% of expected — generous for first calibration pass)
# Assertion: |delta_x| < |delta_y| * 0.15 (less than 15% forward drift during strafe)
```

Use the OTOS pose at the robot's frame origin (camera ground-truth) for the
displacement assertion. Follow the existing assertion pattern (e.g. pytest
`assert` or the script's existing check mechanism).

The test must still pass for the differential robot (the strafe leg should be
gated on `drivetrain_type == "mecanum"` in the script, or skipped if the
active robot config is differential).

## Team-lead HITL Deliverables

### 1. Geometry measurement

Use a ruler or calipers to measure:
- `half_track_mm`: half the lateral distance between wheel contact patches
  (left edge of FL contact to the centerline).
- `half_wheelbase_mm`: half the fore-aft distance between front and rear
  wheel contact patches.
- `wheel_diameter_mm`: mecanum wheel diameter at the roller contact.

Update `data/robots/<5char>.json` with measured values.

### 2. Per-wheel mm_per_wheel_deg calibration

For each wheel that has an encoder:
1. Command `VW 0 0` (stopped); zero encoders (`EZ` or equivalent).
2. Drive the wheel forward a measured distance (500 mm on a ruler).
3. Read the encoder: `mmPerDeg = actual_mm / encoder_degrees`.
4. Update `mm_per_wheel_deg_<fr|fl|br|bl>` in the robot JSON.

If only some wheels are encodered (check in T4), use those and set the others
to the same value as the measured wheels (OTOS is the primary odometry source).

### 3. OTOS scalar calibration (linear and angular)

Follow the same procedure used for `tovez` (existing calibration protocol):
- **Linear scalar**: drive forward 1000 mm (commanded via `D 1000`); measure
  actual travel with the camera; compute `linear_scale = commanded / actual`.
  Update `calibration.otos_linear_scale`.
- **Angular scalar**: turn 360 degrees in place (`TURN 360`); measure actual
  rotation with the camera; compute `angular_scale = actual / commanded`.
  Update `calibration.otos_angular_scale`.

### 4. Playfield camera verification

With the calibrated robot JSON, run:

```bash
uv run python tests/bench/playfield_camera_run.py
```

The script executes: forward leg → turn leg → **strafe leg** (new).
Camera assertions must pass for all three.

### 5. Commit calibrated robot JSON

Once all calibration values are measured and the camera test passes, commit
the updated `data/robots/<5char>.json` with a message like:
`calibrate: mecanum robot <5char> — geometry, mmPerDeg, OTOS scalars`.

Also restore `data/robots/active_robot.json` to point at `tovez.json` for the
default (classroom) build, and document the mecanum switch procedure in a
comment in `active_robot.json`.

## Files to Modify (programmer)

- `tests/bench/playfield_camera_run.py` (add strafe leg)

## Files to Modify (team-lead, during HITL)

- `data/robots/<5char>.json` (fill in measured geometry, mmPerDeg, OTOS scalars)

## Acceptance Criteria

<!-- PROGRAMMER DELIVERABLE (code only, no hardware required) -->
- [x] `tests/bench/playfield_camera_run.py` includes strafe leg after forward + turn
      legs: sends `STRAFE 150 t=2.0`, samples `SNAP vy=` during motion, asserts
      `|delta_y| > 150mm*0.80` and `|delta_x| < |delta_y|*0.15`; leg is gated on
      `drivetrain_type == "mecanum"` (skips cleanly for differential). (`py_compile`
      clean; sim suite 2230 passed; `data/robots/` untouched.) ← code merged 2026-06-23

<!-- HITL DELIVERABLES — pending hardware, to be completed by team-lead -->
- [ ] `tests/bench/playfield_camera_run.py` runs without error on the mecanum robot
      (team-lead confirms all three legs execute).
- [ ] Forward leg: camera-measured displacement within 10% of commanded (1m run).
- [ ] Turn leg: camera-measured rotation within 5 degrees of commanded.
- [ ] Strafe leg: camera-measured lateral displacement > 80% of commanded; forward
      drift < 15% of lateral displacement.
- [ ] `SNAP` after strafe: `vy=` field is non-zero and has the expected sign.
- [ ] Robot JSON `data/robots/<5char>.json` has no MEASURE/CALIBRATE placeholders in
      the geometry and calibration sections — all values are real measurements.
- [ ] `uv run --with pytest python -m pytest tests/simulation -q` still reports `2093 passed`
      (differential sim unaffected even with mecanum JSON as active robot for the HITL session).
- [ ] Switching `active_robot.json` back to `tovez.json` and rebuilding:
      - `DefaultConfig.cpp` diff is additive-constant lines only.
      - Sim suite stays 2093 passed.
      - Golden-TLM oracle unchanged.

## Testing

- **Regression gate (post-HITL)**: `uv run --with pytest python -m pytest tests/simulation -q`
- **HITL gate**: `uv run python tests/bench/playfield_camera_run.py` (camera assertions pass).
- **Verification command**: `uv run --with pytest python -m pytest tests/simulation -q`

## Implementation Notes

- The `playfield_camera_run.py` strafe leg should use `STRAFE 150 t=2.0` as the
  command (time-bounded, 2 seconds). This is safe even if OTOS-distance stop is
  not yet calibrated. The camera reads the actual displacement after the move.
- If the camera shows significant forward drift during strafe (more than 15%),
  suspect `fwd_sign_*` errors or geometry measurement errors (halfTrackMm or
  halfWheelbaseMm off). The kinematics are correct; the parameters need refinement.
- The `STRAFE dist=` feature (OTOS y-pose stop) should be bench-verified here: after
  calibrating the OTOS linear scalar, send `STRAFE 150 dist=300` and confirm the
  robot stops at approximately 300mm lateral displacement (camera-verified).
- Once calibrated, record the final robot JSON values in a commit so they are
  preserved for future sprints.
