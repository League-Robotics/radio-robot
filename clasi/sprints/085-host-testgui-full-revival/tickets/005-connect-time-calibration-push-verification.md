---
id: "005"
title: "Connect-time calibration push verification"
status: open
use-cases: [SUC-006]
depends-on: []
github-issue: ""
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

- [ ] `tests_old/testgui/test_calibration_push_on_connect.py` is ported to
      `tests/testgui/`, updated for any API drift, and passes under
      `QT_QPA_PLATFORM=offscreen` against the real ctypes firmware sim
      (skipped if the sim lib is not built, matching the file's existing
      `pytestmark`).
- [ ] Connecting with an uncalibrated robot config ("tovez nocal") results
      in `GET rotSlip` reading back `0.000` after connect (the sentinel is
      pushed and accepted, not rejected with `ERR badval`).
- [ ] Connecting with a calibrated robot config pushes and reads back that
      robot's actual `ml`/`mr`/`tw`/`rotSlip` values via `GET`.
  - [ ] `OI`/`OL`/`OA` against the sim ack normally (the sim implements
        the OTOS command surface per 084 ticket 008); the `NODEV`-tolerant
        code path is exercised or explicitly documented as verified
        (e.g. by a fake transport returning `ERR nodev` for one of these
        three, confirming the push loop logs and continues rather than
        aborting).
- [ ] Robot-combo change while connected re-triggers the same push
      sequence and is confirmed to overwrite the previously-active robot's
      values.
- [ ] Any genuine bug surfaced by a real round trip is fixed here and
      documented.

## Testing

- **Existing tests to run**: full `tests/testgui` suite (regression).
- **New tests to write**: port `test_calibration_push_on_connect.py`.
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run pytest
  tests/testgui -q`
