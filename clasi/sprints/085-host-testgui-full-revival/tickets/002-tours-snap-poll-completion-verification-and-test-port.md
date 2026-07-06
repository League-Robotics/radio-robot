---
id: "002"
title: "Tours: SNAP-poll completion verification and test port"
status: done
use-cases: [SUC-001]
depends-on: ["001"]
github-issue: ""
issue: host-testgui-full-revival.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Tours: SNAP-poll completion verification and test port

## Description

`_TourRunner` (`host/robot_radio/testgui/__main__.py` lines ~1254–1372) and
`commands.TOURS` (`TOUR_1`/`TOUR_2`, hardcoded `D`/`RT` wire-string
sequences) already implement the full tour feature: it sends each step via
`transport.command()`, then polls completion with a fire-and-forget `SNAP`
and reads `mode=I` off the cached `TLMFrame` in `_state["last_tlm"]` (not
`transport.command("SNAP")`, whose corr-id-less reply never reaches the
reply queue — see the class's own docstring). This code predates the
greenfield rebuild and has never run against a real sprint-084 firmware/sim
(per `architecture-update.md` Grounding fact 1) — sprint 084's `D`/`RT`
verbs and `mode=` machine are what it was always waiting on.

This ticket ports the three tours-related historical test files from
`tests_old/testgui/` to `tests/testgui/`, updates them for the current API
if anything drifted, and — critically — actually runs Tour 1 and Tour 2
against the sim for the first time since 084 closed, fixing whatever a real
run surfaces. Completing with zero production-code changes (beyond a real
bug fix, if one is found) is an acceptable, expected outcome — the
acceptance bar is that the ported tests actually pass against the sim, not
merely that they exist (see `architecture-update.md` Migration Concerns,
"Risk of verification finds nothing to fix").

## Acceptance Criteria

- [x] `tests_old/testgui/test_tour_idle_detection.py`,
      `test_tour_stop.py`, and `test_tour1_geometry.py` are ported to
      `tests/testgui/`, updated for any API drift since they were written,
      and pass under `QT_QPA_PLATFORM=offscreen`.
- [x] Tour 1 (`commands.TOUR_1`) runs to completion against the sim
      (`SimTransport`) with no step timing out, and the robot's fused pose
      ends near world origin (the tour is a closed geometric loop).
- [x] Tour 2 (`commands.TOUR_2`) likewise runs to completion.
- [x] `_wait_for_idle`'s stale-frame rejection (a cached `TLMFrame`
      timestamped before the current step began must not end the wait
      early) is exercised and holds against the real `mode=` machine.
- [x] Stopping a running tour re-enables the tour buttons synchronously
      (not dependent on the `finished` signal being delivered during the
      blocking `thread.wait()` — see the existing
      `testgui-tour-stop-reactivation.md` root-cause doc).
- [x] Any bug a real run surfaces (e.g. a `SNAP`-poll timing constant that
      needs retuning against actual 084 mode-machine latency — flagged as
      Open Question 1 in `architecture-update.md`) is fixed in this ticket
      and the fix is documented in this ticket's file, not silently folded
      in.

## Testing

- **Existing tests to run**: full `tests/testgui` suite (regression); the
  three newly-ported files specifically.
- **New tests to write**: the three ported files above, adapted as needed;
  no net-new test file (unlike ticket 003).
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run pytest
  tests/testgui -q`

## Findings (real-run verification against sprint-084 firmware/sim)

**Result: zero production-code changes.** `_TourRunner`'s SNAP-poll
`mode=I` completion-detection design (fire-and-forget `transport.send("SNAP")`
+ read `state["last_tlm"]`, stale-frame rejected by `ts >= t_start`) works
correctly against the real sprint-084 `mode=` machine exactly as designed —
no timing-constant retuning was needed (`SPINUP_S`/`POLL_S`/
`SNAP_REPLY_TIMEOUT_S`/`MOVE_TIMEOUT_S` are unchanged). Both tours ran to
completion, every one of the 26 combined per-step idle-waits (13 steps ×
2 tours) succeeded with no timeout, and the stop-button reactivation is
synchronous against a live running tour (new test:
`test_stopping_a_running_tour_reenables_buttons_synchronously` in
`test_tour1_geometry.py`).

**Investigated: a real, but out-of-scope, motion-accuracy characteristic.**
The ported `test_tour1_geometry.py` originally (`tests_old/`) asserted
strict per-waypoint/final-heading tracing, `xfail(strict=True)`,
root-caused to `source_old`'s `rotationalSlip=0.92` baked into the compiled
firmware in a way GUI robot selection could never reach. Direct
investigation this ticket found:

- That specific bug is FIXED in this tree: the active robot
  (`data/robots/tovez_nocal.json`) pushes `SET rotSlip=0` on Connect (the
  documented no-correction sentinel), which the firmware's
  `PoseEstimator::effectiveSlip()` maps to `1.0` (no inflation) — confirmed
  by direct read of `config_commands.cpp`/`pose_estimator.cpp`.
- However, per-waypoint/heading tracing STILL does not hold exactly, for a
  different, already-documented, already out-of-scope reason:
  `handleRT` (`source/commands/motion_commands.cpp`) is explicitly
  open-loop and slip-uncorrected by design ("minus its
  rotational-slip/coast-anticipation refinement ... coast-anticipation is
  not part of this ticket's [084-003] acceptance bar" — its own doc
  comment), and `tests/sim/unit/test_motion_commands_arc_turn.py`
  independently measures ~4-5° coast overshoot per isolated `RT 9000`
  (its own tolerance is ±10°). Chained across a 6-7 turn tour this
  compounds into tens of degrees of final-heading drift.
- This was reproduced identically (same order of magnitude) via TWO
  independent harnesses — the real GUI/`SimTransport`/`QThread` stack, and
  a raw single-threaded `tests/_infra/sim` `firmware.Sim` script with no
  GUI/threading involved at all — ruling out a tour-plumbing/GUI-polling
  bug. It is a firmware motion-control-accuracy characteristic, explicitly
  deferred by ticket 084-003's own architecture decision, not a defect in
  this ticket's scope (SNAP-poll completion verification).

Per this ticket's own acceptance criteria (position-only "ends near world
origin", not exact waypoint tracing), the ported `test_tour1_geometry.py`
was rewritten to assert fused-pose position closure with a documented,
measured-and-margined tolerance (300 mm), and to drop the strict
per-waypoint/heading assertion (documented in the file's own module
docstring, not silently dropped). Measured fused-pose distance from origin,
across repeated runs, real 1x sim pacing:

| Tour | Measured (mm) | Assertion tolerance |
|---|---|---|
| Tour 1 | ~20-40 | 300 mm |
| Tour 2 | ~95-175 | 300 mm |

**Note for a future ticket (085-005, calibration push):** `_push_robot_calibration()`'s
`SET odomOffX=`/`odomOffY=`/`odomYaw=` pushes are rejected
(`ERR badkey`) by the current `config_commands.cpp` — observed on every
Connect during this ticket's verification runs, unrelated to tours (the
push already treats `ERR`/`NODEV` as non-fatal and continues). Flagged here
for ticket 005's scope, not fixed in this ticket.
