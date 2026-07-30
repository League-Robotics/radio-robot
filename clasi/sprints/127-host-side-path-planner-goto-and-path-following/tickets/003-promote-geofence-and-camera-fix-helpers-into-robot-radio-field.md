---
id: '003'
title: Promote Geofence and camera-fix helpers into robot_radio/field/
status: open
use-cases:
- SUC-003
depends-on: []
github-issue: ''
issue: sprint-127-host-side-path-planner-goto-path-following.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Promote Geofence and camera-fix helpers into robot_radio/field/

## Description

Design issue T2. Today `Geofence`/`GeofenceViolation`/`checkPlayfieldLights`/
`captureFix` live only in `src/tests/bench/square_tour.py`
(confirmed: `grep -rln "class Geofence" src` matches exactly that one
file), and `otos_calibration_bench.py` reaches them via a `sys.path`
hack (`otos_calibration_bench.py:59-65`:
`sys.path.insert(0, str(_THIS_DIR))` then
`from square_tour import Geofence, GeofenceViolation,
checkPlayfieldLights`). `robot_radio/field/` already exists and is
"Live" per `robot_radio/DESIGN.md` (currently just `playfield.py`) — this
ticket moves the geofence/camera-fix helpers into it as a second module,
not a new package, and repoints both existing callers. This unblocks
ticket 004 (`WorldPose`'s re-anchor logic) and ticket 006 (the planner
loop's geofence check), which both need one canonical
`Geofence`/camera-fix implementation to depend on, not two.

**In transit, fix one real bug**: `Geofence.captureFix`'s per-axis median
averages raw yaw linearly (`square_tour.py:289-293`), which is
wrap-unsafe near ±π. Switch it to the circular mean
`testkit/camera.read_camera_pose` already implements
(`atan2(mean(sin), mean(cos))`) — move-not-rewrite for x/y, fix-in-place
for yaw.

**Files**:
- New: `src/host/robot_radio/field/geofence.py` — `Geofence`,
  `GeofenceViolation`, `checkPlayfieldLights`, `captureFix`,
  `captureFixWithRetry`, moved verbatim from `square_tour.py` except the
  yaw-averaging fix described above.
- Modify: `src/host/robot_radio/field/__init__.py` — export the new names
  alongside the existing `Playfield`/`Tag`/`Feature`.
- Modify: `src/tests/bench/square_tour.py` — delete the local
  `Geofence`/`GeofenceViolation`/`checkPlayfieldLights`/`captureFix`
  definitions; `from robot_radio.field import Geofence,
  GeofenceViolation, checkPlayfieldLights` (or `from robot_radio.field.geofence
  import ...`, whichever `field/__init__.py` ends up exporting). No
  behavior change at any of `square_tour.py`'s own call sites.
- Modify: `src/tests/bench/otos_calibration_bench.py` — delete the
  `sys.path.insert`/`from square_tour import ...` block (lines ~59-65);
  import from `robot_radio.field` instead.
- Grep the rest of the tree for any other importer of
  `square_tour.Geofence`/`square_tour.checkPlayfieldLights` before
  deleting the originals — the two call sites above are the only ones
  known today, but confirm rather than assume (Architecture Open Question
  3).

**Coding standards**: no units in any identifier introduced or touched in
transit (e.g. `margin`/`lostGrace` stay quantity-named with `# [cm]`/
`# [s]` tags, matching the existing style — do not accidentally rename
into a units-suffixed form while moving). lowerCamelCase
functions/variables, UpperCamelCase types (`Geofence`, `GeofenceViolation`
already conform).

This ticket touches no hardware directly — it is a code move plus one bug
fix, verified by re-running the two scripts it repoints, not a new bench
or playfield session. `.claude/rules/playfield-testing.md`'s obligations
don't newly apply here (this ticket doesn't drive the robot), but note
for the record: the geofence/camera-fix code this ticket moves is exactly
what implements those obligations for every script that calls it — do not
change its `estop()`-not-`stop()` behavior or its fail-closed-on-lost-tag
behavior while moving it.

## Acceptance Criteria

- [ ] `grep -rn "class Geofence" src` finds exactly one definition, under
      `robot_radio/field/`.
- [ ] `otos_calibration_bench.py` contains no `sys.path.insert` call.
- [ ] `square_tour.py` imports `Geofence`/`GeofenceViolation`/
      `checkPlayfieldLights`/`captureFix` from `robot_radio.field` and
      defines none of them locally.
- [ ] `captureFix`'s yaw averaging uses the circular mean (matches
      `testkit/camera.read_camera_pose`'s formula), not a linear median of
      raw yaw.
- [ ] A grep for any other importer of the old
      `square_tour.Geofence`/`square_tour.checkPlayfieldLights` path
      turns up none (or, if one is found, it is repointed in this same
      ticket).
- [ ] `square_tour.py --sim` and `otos_calibration_bench.py`'s existing
      modes still run without import errors after the move.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim -q`
  (no sim tests exercise `Geofence` directly today, but this confirms no
  import-time regression across the package); a smoke run of
  `uv run python src/tests/bench/square_tour.py --sim` (exercises the
  Geofence import path end to end against the sim backend, no hardware
  needed).
- **New tests to write**: a small unit test for `captureFix`'s corrected
  circular-mean yaw averaging (e.g. a synthetic set of tag readings
  straddling ±π that a linear median would get wrong and the circular
  mean gets right) — `src/tests/unit/` or alongside
  `field/geofence.py` per the existing `robot_radio` test layout.
- **Verification command**:
  `uv run python -m pytest src/tests/sim -q && uv run python src/tests/bench/square_tour.py --sim`
