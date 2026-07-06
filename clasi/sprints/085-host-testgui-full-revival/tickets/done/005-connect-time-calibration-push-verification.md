---
id: '005'
title: Connect-time calibration push verification
status: done
use-cases:
- SUC-006
depends-on: []
github-issue: ''
issue: host-testgui-full-revival.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Connect-time calibration push verification

## Description

`calibration.push.calibration_commands(config)`
(`host/robot_radio/calibration/push.py`) and `_push_robot_calibration`
(`host/robot_radio/testgui/__main__.py` lines ~1580–1631) already build and
send the full calibration sequence on Connect and on every robot-combo
change while connected: `SET ml=`/`SET mr=`/`SET tw=`/`SET rotSlip=`, `OI`,
`OL`, `OA`, and optional `SET odomOffX/Y/Yaw=`. An uncalibrated robot
("tovez nocal") pushes the documented neutral sentinels (`rotSlip=0` →
firmware's `PoseEstimator::effectiveSlip()` maps it to 1.0; `OL`/`OA` scale
1.0), so a nocal robot always runs geometry-pure regardless of what
`DefaultConfig.cpp` bakes in at compile time. A reply containing `NODEV`
(expected for `OI`/`OL`/`OA` against real hardware, which has no OTOS
driver) is already counted and logged, not treated as a failure.

This code predates the greenfield rebuild and has not been exercised
against real sprint-084 `SET`/OTOS verbs. Planning-time grounding already
confirmed one specific risk does *not* apply: `docs/protocol-v2.md` §7's
invariant table states `rotSlip` must be in `[0.5, 1.0]`, which looks like
it would reject `SET rotSlip=0` — but direct read of
`source/commands/config_commands.cpp`'s `validateCandidate` (line ~307)
confirms `(slip == 0.0f) || (slip >= 0.5f && slip <= 1.0f)` explicitly
allows the `0.0f` sentinel. This ticket's job is to confirm that holds in
practice (a real `SET`/`GET` round trip), not to fix a bug that turned out
not to exist.

## Acceptance Criteria

- [x] `tests_old/testgui/test_calibration_push_on_connect.py` is ported to
      `tests/testgui/`, updated for any API drift, and passes under
      `QT_QPA_PLATFORM=offscreen` against the real ctypes firmware sim
      (skipped if the sim lib is not built, matching the file's existing
      `pytestmark`).
- [x] Connecting with an uncalibrated robot config ("tovez nocal") results
      in `GET rotSlip` reading back `0.000` after connect (the sentinel is
      pushed and accepted, not rejected with `ERR badval`).
- [x] Connecting with a calibrated robot config pushes and reads back that
      robot's actual `ml`/`mr`/`tw`/`rotSlip` values via `GET`.
  - [x] `OI`/`OL`/`OA` against the sim ack normally (the sim implements
        the OTOS command surface per 084 ticket 008); the `NODEV`-tolerant
        code path is exercised or explicitly documented as verified
        (e.g. by a fake transport returning `ERR nodev` for one of these
        three, confirming the push loop logs and continues rather than
        aborting).
- [x] Robot-combo change while connected re-triggers the same push
      sequence and is confirmed to overwrite the previously-active robot's
      values.
- [x] Any genuine bug surfaced by a real round trip is fixed here and
      documented.

## Implementation notes (2026-07-06) -- real bug found and fixed

Ported to `tests/testgui/test_calibration_push_on_connect.py` (10 tests:
4 Qt-free `calibration_commands()` unit tests, 2 Qt-free push-loop
resilience tests, 4 real-GUI/real-sim tests).

**Real bug (the one this sprint's brief flagged from 085-002's finding):**
`calibration_commands()` (`host/robot_radio/calibration/push.py`) pushed
`SET odomOffX=`/`odomOffY=`/`odomYaw=` whenever
`config.geometry.odometry_offset_mm` was non-zero — true for BOTH real
robot configs (`data/robots/tovez.json` and `tovez_nocal.json`,
`x=-47.7, y=3.5`). `config_commands.cpp`'s registered `SET` key table
(architecture-update.md (084) Decision 2's closed 15-key surface —
`tw, ml, mr, pid.*, rotSlip, ekf*, minSpeed, sTimeout`) has never included
`odomOffX`/`odomOffY`/`odomYaw`, so every Connect with either real config
hit `ERR badkey` on those three commands (silently tolerated by the push
loop's own `ERR`-counting, so Connect itself did not fail — but the values
were genuinely rejected, not applied).

**Fix chosen: Option (a) — drop the odom-offset push entirely** from
`calibration_commands()` (not Option (b), tolerate-and-skip). Rationale:
the OTOS lever-arm has no real hardware driver anywhere in this program,
and OTOS pose is otherwise configured entirely via `OI`/`OL`/`OA` (scale
calibration) and `OV`/`SI` (pose seeding) — never via `SET`. The
`odomOffX/Y/Yaw` push was dead weight pushing values firmware has never
been able to apply, not a feature silently regressed by 084's closed key
table (084 Decision 2's 15-key surface was never asked to include it).
`config.geometry.odometry_offset_mm` itself is untouched in the schema —
`calibration_commands()` is simply no longer one of its consumers.
Option (b) (tolerate/skip with a warning) was rejected: it would keep
generating known-dead wire traffic on every Connect and every robot-combo
change indefinitely, for a value with no code path to ever reach firmware.

**Known related issue, NOT fixed (out of this ticket's file scope):**
`host/robot_radio/io/cli.py`'s `_push_calibration()` (the standalone CLI
`push-cal` command path) has an independent, textually-duplicated copy of
the same `SET odomOffX/Y/Yaw` push (lines ~193-206) — it is a separate
function, not a caller of `calibration_commands()`, so this ticket's fix
does not reach it. Flagged here for a future ticket; out of scope per this
ticket's stated files (`calibration/push.py` + `_push_robot_calibration`
in `__main__.py`).

New coverage beyond the pre-rebuild file: a `calibration_commands()`
regression test asserting no `odomOff*`/`odomYaw` command is ever produced;
two push-loop resilience tests (`ERR nodev` tolerated and doesn't abort the
loop; a genuine `ERR` is counted/logged and also doesn't abort); a
real-sim regression connecting with the REAL `data/robots/tovez_nocal.json`
(the actual file that exhibited the bug) confirming no `badkey`/`REJECTED`
appears in the log; and a real end-to-end robot-combo-switch test (real
`robot_combo`/`list_robots()`/`set_active_robot()` wiring, real
`data/robots/{tovez_nocal,tovez}.json`) confirming `GET rotSlip` reads
`0.000` then `0.920` after switching robots while connected. The combo test
temporarily rewrites and restores (in `finally`) the real, git-tracked
`data/robots/active_robot.json` pointer — verified unchanged after the
run.

Full `tests/testgui` suite: 186 passed (up from 176 pre-ticket).

## Testing

- **Existing tests to run**: full `tests/testgui` suite (regression).
- **New tests to write**: port `test_calibration_push_on_connect.py`.
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run pytest
  tests/testgui -q`
