---
id: '001'
title: OTOS presence, units, and liveness bring-up
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: otos-telemetry-bring-up-and-camera-calibration.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# OTOS presence, units, and liveness bring-up

## Description

The OTOS driver, telemetry frame, and boot config are already built and
wired (`Devices::RealOtos`, `TLMFrame`'s `otos*` fields,
`gen_boot_config.py`'s `otos_boot_config_values()`) — verified on hardware
2026-07-29, `STATUS` already reports `otos=1`. What is not known is what
the reported tuple's units are and whether it actually tracks motion; this
ticket establishes both from a measured move, not from source reading.

Create `src/tests/bench/otos_calibration_bench.py`, the shared script this
whole sprint grows one mode at a time (see sprint.md's Architecture
Design Rationale). This ticket adds its skeleton plus a `--mode units`
run:

1. **Preflight**: check playfield lights (`192.168.1.122`, `output: true`
   — `.claude/rules/playfield-testing.md`), open camera / create playfield
   / start detection, confirm tag 100 (robot centre) and tag 1 (field
   origin) are visible.
2. Confirm robot has reached `READY` with `connL=1`/`connR=1` and `STATUS`
   reports `otos=1`; confirm telemetry flag bit 0 is set.
3. Capture a camera-truth rest pose (median-of-7, per
   `.claude/rules/playfield-testing.md`'s segment-boundary convention).
4. Read the OTOS pose tuple at rest over several telemetry frames; confirm
   it holds steady (bounded jitter) while the robot is not moving.
5. Command **one** known straight-line move (tether-safe by construction —
   straight moves never wrap the cable). Capture the camera-truth pose
   again at rest after the move (post-settle, per the same convention).
6. Read the OTOS pose delta over the same interval. Compare the OTOS
   delta's raw magnitude to the camera-measured distance in both mm and cm
   interpretations; whichever interpretation matches the camera distance
   (after accounting for the already-applied `otos_linear_scale`, since
   the chip's on-board scale register is set at `begin()` — the read-back
   value already reflects the committed 1.067) settles the unit.
7. Print a clear units/liveness report: raw units, at-rest jitter bound,
   moving-vs-camera delta comparison, PASS/FAIL against acceptance
   criteria 1 and 2 of the issue.
8. `estop()` (never `stop()`) in a `finally` block on every connection
   opened, per `.claude/rules/playfield-testing.md`.

Reuse `square_tour.py`'s lights-preflight and camera-fix helpers and
`tlm_log.py`'s `otos_*` telemetry-column capture rather than
reimplementing either — do not fork new copies of that logic.

If `otos=0` is seen at any point, first check `connL`/`connR` and whether
`READY` was emitted (per the issue's "blocking hazard" section) before
concluding the chip is missing — a wedged external I2C bus produces the
identical symptom and needs a full power cycle, not debugging as a sensor
fault.

## Acceptance Criteria

- [x] `otos_calibration_bench.py` exists with a `--mode units` entry
      point, reusing `square_tour.py`/`tlm_log.py` helpers (not forked
      copies).
- [x] Script confirms `STATUS otos=1` and telemetry flag bit 0 set on a
      `READY` robot with `connL=1`/`connR=1`, and prints the result.
- [x] Script measures one known straight-line move, compares camera truth
      to the OTOS delta, and prints an explicit units conclusion (mm or
      cm) with the supporting numbers — not asserted from source.
- [x] Script confirms and prints that the OTOS pose holds steady at rest
      (bounded jitter) and changes under motion.
- [x] Every connection path calls `estop()` (never `stop()`) in a
      `finally` block.
- [x] Run against the real robot on the playfield; results (raw output,
      not paraphrased) included in the ticket's completion notes.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim -q`
  (confirm no new regressions beyond the 11 known pre-existing failures).
- **New tests to write**: None — this is a hardware/playfield measurement
  script, not a unit-testable code path. `otos_calibration_bench.py` is a
  bench tool, same category as `tlm_log.py`/`square_tour.py` (not
  pytest-collected, per `src/tests/CLAUDE.md`).
- **Verification command**: `uv run python src/tests/bench/otos_calibration_bench.py --port <robot-port> --mode units`

## Completion Notes

**Safety change made before this run** (per stakeholder instruction, following
the prior session's west-rail incident): added
`clampTimeoutToClearance()`/`clearanceAlongDirectionCm()` to
`otos_calibration_bench.py`, shared by every move the script commands. Each
move's `timeout` is now clamped so that worst-case travel
(`timeout x commanded speed`) cannot exceed the camera-confirmed clearance to
the nearest field boundary in the direction of travel, computed from the
pre-move camera fix — not from the intended stop condition, which is exactly
what can fail to fire. If the clamp would leave less time than the commanded
leg needs to complete, the script FAILs out and refuses to run the leg rather
than attempting it. Verified in isolation before the hardware run: at the
robot's actual pose (x=+40.8cm, heading ~178deg/west), a `--leg 300 --cruise
150` move clamped the old `9000ms` request down to `6189ms`
(worst-case travel bounded to 928mm, vs. the pre-fix formula's ~1.4m).

**Hardware run** (`/dev/cu.usbmodem21141112`, playfield, lights confirmed ON
at 192.168.1.122 before the run — they had gone dark again since the prior
session):

```
configured odometry_offset_mm: x=-47.70 y=+3.50 mm (from data/robots/tovez.json)
=== otos_calibration_bench --mode units (ticket 126-001) ===
connected: port=/dev/cu.usbmodem21141112 mode=direct
geofence ARMED: stops the robot within 15 cm of the field edge, and on tag loss
liveness over 28 frames: otos_present=True conn_left=True conn_right=True
PASS: STATUS otos=1, connL=1, connR=1 -- telemetry flags bit 0 set on a READY robot (issue acceptance criterion 1)
  camera fix 'units: rest before move': x=+40.5cm y=-0.7cm yaw=-176.8deg (7/7 samples)
at-rest OTOS jitter over 29 bursts: dx=0.00 dy=0.00 raw units, dheading=0.000 deg
timeout safety clamp: requested=9000ms camera-confirmed clearance=928mm -> clamped=6189ms (worst-case travel 928mm)
commanding straight move: v_x=+150 mm/s stop_distance=300 mm timeout=6189 ms (direction=forward, chosen for field margin; straight moves never wrap the tether)
  enqueue ack: AckEntry(corr_id=1, ok=True, err_code=0) (OK)
  camera fix 'units: rest after move': x=+11.6cm y=-0.6cm yaw=+177.5deg (7/7 samples)

camera-measured travel: 289.4 mm (commanded leg 300 mm)
OTOS raw delta magnitude: 305.01 raw units (x: 48.0->353.0, y: -3.0->-5.0)
  mm hypothesis (raw IS mm):        ratio raw/camera_mm       = 1.054
  cm hypothesis (raw IS cm, x10=mm): ratio (raw*10)/camera_mm = 10.539
CONCLUSION: OTOS pose tuple units = mm (residual 5.4% vs. camera-measured travel, measured from this move, not asserted from source)

motion-vs-rest: OTOS delta (305.01) exceeds 3x the at-rest jitter bound (0.00) -- pose DOES track motion

=== SUC-001 acceptance ===
  AC1 presence (otos=1, flags bit0, connL=1, connR=1): PASS
  AC2 liveness+units (tracks motion, holds at rest, units stated): PASS
```

**Conclusion**: OTOS pose tuple units are **mm**, measured (5.4% residual
against camera-measured travel), not asserted from source. At-rest jitter is
0.00 raw units over 29 bursts — pose holds perfectly steady when not moving
and tracks motion cleanly under it. Robot ended the move at camera x=+11.6cm,
well clear of the west rail (field limit -67.15cm, margin 15cm) — the timeout
clamp left ~7cm of unused slack (928mm clearance budgeted, ~289mm actually
travelled) rather than the ~1.4m of blind runway the old formula allowed.

`uv run python -m pytest src/tests/sim -q`: 423 passed, 1 skipped, 1 xfailed
(no new regressions).
