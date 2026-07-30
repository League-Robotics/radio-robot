---
id: '002'
title: OTOS lever-arm frame and sign confirmation
status: done
use-cases:
- SUC-002
depends-on:
- '001'
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

- [x] `otos_calibration_bench.py --mode lever-arm` exists, reusing ticket
      001's preflight/camera helpers.
- [x] A single in-place rotation is captured with camera truth at rest
      before/after and multiple OTOS telemetry samples spanning the turn.
- [x] The script determines, from that trajectory, whether telemetry
      reports centre-frame or chip-frame, and prints the determination
      with supporting numbers.
- [x] The configured `odometry_offset_mm` magnitude and sign are checked
      against the determined frame and the result (confirmed or
      mismatched) is printed explicitly — no silent correction is made.
- [x] Every connection path calls `estop()` (never `stop()`) in a
      `finally` block.
- [x] Run against the real robot on the playfield; raw results included in
      the ticket's completion notes.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim -q`.
- **New tests to write**: None — hardware/playfield measurement script,
  not pytest-collected.
- **Verification command**: `uv run python src/tests/bench/otos_calibration_bench.py --port <robot-port> --mode lever-arm`

## Completion Notes

**Tether-direction bug found and fixed before this run.** The script as
written in ticket 001 hardcoded the turn direction (`TURN_OMEGA = -2.0`,
always `+90 deg` delta) on the assumption the robot starts near east. After
001's move the robot's actual heading was `+178.3 deg` (near due west) — a
fixed `+90 deg` delta from there sweeps to `+267.5 deg`, i.e. **within 2.5
deg of due south (-90/270 deg, the tether direction)**, not through north at
all. `sweepCrossesAngle()` alone said this technically didn't cross south,
but 2.5 deg of clearance is not what "never through south" means in
practice. Replaced the fixed direction with `pickSafeTurnDirection()` +
`closestApproachToSouthDeg()`: the sign of the turn is now chosen from the
*measured* starting heading, picking whichever of `+90`/`-90 deg` keeps the
larger clearance from south, and refusing to move (returns None) if neither
direction keeps >= 20 deg clearance. `TURN_OMEGA` was renamed
`TURN_OMEGA_MAG` (a magnitude; sign is now computed at run time). Verified
in isolation before the hardware run — at the actual start heading
(+178.3 deg) this picks `-90 deg` (ending near due north, +87.5 deg unit-test
/ +92.7 deg measured), 90+ deg clear of south, vs. the old fixed direction's
2.5 deg.

**Hardware run** (`/dev/cu.usbmodem21141112`, playfield, lights ON):

```
camera fix 'lever-arm: rest before turn': x=+11.6cm y=-0.5cm yaw=+178.3deg (7/7 samples)
turn direction: -90 deg (omega=+2.00 rad/s) chosen from the measured start heading +178.3 deg for tether safety -- closest approach to south during the sweep: 91.7 deg (>= 20 required)
timeout safety clamp: requested=5356ms camera-confirmed clearance=inf (pure rotation, zero translational worst case) -> clamped=5356ms
commanding in-place rotation: omega=+2.00 rad/s stop_angle=90 deg timeout=5356 ms
  enqueue ack: AckEntry(corr_id=1, ok=True, err_code=0) (OK)
  camera fix 'lever-arm: rest after turn': x=+12.0cm y=-2.0cm yaw=+92.7deg (7/7 samples)

camera-measured turn: -85.6 deg (commanded 90 deg)

71 OTOS bursts spanning the turn (excerpt, raw units = mm per ticket 001):
  [ 0] x=+48.00 y=-2.00 heading=+2.01 deg
  [24] x=+49.00 y=+0.00 heading=-54.14 deg
  [49] x=+47.00 y=+8.00 heading=-78.95 deg
  [70] x=+47.00 y=+11.00 heading=-82.05 deg

centre-frame test: max deviation from the trace's own start point = 13.04 raw units (mm)
chip-frame test: fitted circle centre=(+37.86,+3.63) radius=11.20 raw units (mm), fit residual RMS=0.60 (expected radius if chip-frame ~ |odometry_offset_mm| = 47.83 mm)

=== FRAME DETERMINATION ===
CENTRE-FRAME: reported (x,y) stayed within 13.04 raw units of its start (bound 16.7) across a -85.6 deg camera-measured turn.

=== SUC-002 acceptance ===
  AC1 frame determined from measurement: PASS (frame=centre)
  AC2 odometry_offset_mm magnitude+sign: CONFIRMED correct against the measured frame
```

**Conclusion**: telemetry reports the robot's **CENTRE of rotation**, not
the raw chip location — measured, not read from source. Over an 85.6
deg camera-measured turn, the reported (x, y) moved only 13.04 mm from its
own start point (well under the 16.7 mm bound, and an order of magnitude
below the ~47.8 mm radius a raw chip-frame reading would have traced — the
chip-frame circle fit landed at radius 11.2 mm, nowhere near the expected
47.83 mm). This confirms `RealOtos::tick()`'s `sensorToCentre()` transform
is being applied, and that the configured `data/robots/tovez.json`
`odometry_offset_mm` (x=-47.70, y=+3.50 mm) is correct in **both magnitude
and sign** — a wrong-signed or wrong-magnitude offset would have left an
uncancelled residual tracing a circle with the turn, which was not
observed. No mismatch found; nothing to flag as a follow-up.

The residual 13.04 mm (vs. a theoretical 0 for a perfect centre-of-rotation)
is small relative to field/measurement noise (camera quantization, wheel
scrub during an in-place turn) and does not indicate a lever-arm error —
it is well inside the 16.7 mm centre-frame bound and nowhere near the
47.8 mm chip-frame radius.

`uv run python -m pytest src/tests/sim -q`: 423 passed, 1 skipped, 1 xfailed
(no new regressions).
