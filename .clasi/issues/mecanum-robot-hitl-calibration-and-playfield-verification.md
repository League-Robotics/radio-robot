---
title: "Mecanum robot HITL calibration and playfield camera verification"
status: pending
hardware-in-the-loop: true
blocked-on: "a physical mecanum robot (togov) connected on the bench + the playfield camera"
origin: "Carved out of sprint 046 ticket 008 — the code deliverable (strafe leg) shipped; the HITL half was deferred (no mecanum robot on the bench at close time)."
---

# Mecanum robot HITL calibration and playfield camera verification

Sprint 046 delivered the mecanum drivetrain code and the compile-time
differential/mecanum switch (both firmware variants build clean; merged to
master). The remaining work from ticket 046-008 is **hardware-in-the-loop**
calibration of a real mecanum robot, which could not be done at close time
because no mecanum robot was connected to the bench (`mbdeploy: no devices
found`). This issue tracks that follow-up so it is not lost.

## Prerequisites
- A physical mecanum robot (config `data/robots/togov.json`) flashed with the
  mecanum firmware (`active_robot.json` → `togov.json`, then
  `python3 build.py --fw-only --clean` → flash via `mbdeploy deploy <robot>`).
- The aprilcam playfield camera available and calibrated.

## Deliverables (from ticket 046-008 "Team-lead HITL")
1. **Geometry measurement** → `togov.json`: `half_track_mm`, `half_wheelbase_mm`,
   `wheel_diameter_mm` (ruler/calipers).
2. **Per-wheel `mm_per_wheel_deg_<fr|fl|br|bl>`**: drive each encodered wheel a
   measured 500 mm, `mmPerDeg = actual_mm / encoder_degrees`.
3. **OTOS scalars**: `otos_linear_scale` (drive `D 1000`, camera-measure travel),
   `otos_angular_scale` (`TURN 360`, camera-measure rotation).
4. **Provisional `fwd_sign_*` re-verification**: the values committed in 046-008
   (FR+1/FL-1/BR+1/BL-1) were set from bench observation and need confirmation on
   real hardware (per-wheel direction check via `source/WheelTestMain.cpp` —
   build with `WHEEL_TEST_MAIN 1`).
5. **Playfield camera verification**: `uv run python tests/bench/playfield_camera_run.py`
   — forward, turn, AND the new **strafe** leg must all pass their camera assertions
   (forward ≤10% err, turn ≤5°, strafe lateral >80% of commanded, forward drift <15%).
6. **`SNAP` after strafe**: confirm `vy=` is non-zero with the expected sign.
7. **Commit** the calibrated `togov.json` (real values, no MEASURE/CALIBRATE
   placeholders), then restore `active_robot.json` → `tovez.json` (classroom default).

## Acceptance
All ticket 046-008 HITL acceptance criteria met on the real mecanum robot, with
the calibrated `togov.json` committed and the differential default restored
(sim suite green, golden-TLM unchanged).
