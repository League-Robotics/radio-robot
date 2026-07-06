---
id: "003"
title: "Camera GOTO: pure-pursuit verification and new test"
status: open
use-cases: [SUC-003]
depends-on: ["001"]
github-issue: ""
issue: host-testgui-full-revival.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Camera GOTO: pure-pursuit verification and new test

## Description

`_GotoRunner` (`host/robot_radio/testgui/__main__.py` lines ~1374–1490)
already implements the camera-in-the-loop pure-pursuit loop: each iteration
reads the freshest cached camera truth pose (`_state["last_truth"]`),
checks `commands.goto_reached`, and if not yet arrived, sends `SI` (via
`operations.build_setpose_command`, re-anchoring pose to camera truth) then
`G <x> <y> <speed>` (re-aiming at the fixed target) — throttled by
`POLL_S`. On arrival, timeout, or explicit stop, `_safe_stop()` sends the
top-level `STOP` verb.

Unlike every other ticket in this sprint, **camera GOTO has no historical
test at all.** Grepping every `tests_old/testgui/*.py` and
`tests/testgui/*.py` file for `_GotoRunner`/`GotoRunner` during planning
found it referenced only in `test_tour_stop.py` (which tests the
`_stop_goto` button-re-enable path, not the pursuit loop itself) and in
`__main__.py` — `goto_distance`/`goto_reached` (the pure geometry helpers)
are tested in the already-ported `test_commands.py`, but the runner's
actual convergence behavior has never been exercised. This ticket writes
that test from scratch (see `architecture-update.md` Grounding fact 4 and
Design Rationale Decision 2, which is why this is its own ticket rather
than folded into Tours).

This ticket also serves as the practical confirmation of
`architecture-update.md` Decision 1: the top-level `STOP` verb
`_safe_stop()` sends is a real, sprint-084-implemented verb
(`source/commands/motion_commands.cpp`'s `handleStop`) that clears
`Subsystems::Planner`'s active goal — it must **not** be changed to
`DEV DT STOP`, which would not cancel an in-flight `G` and would regress
GOTO's abort/timeout paths (084's own Open Question 3: no arbitration
between `DEV DT` and `Planner`-issued motion). This ticket's new test
asserts on that cancel-in-progress-goal behavior directly against the sim,
rather than relying on source-reading alone.

## Acceptance Criteria

- [ ] A new `tests/testgui/test_goto.py` exercises `_GotoRunner.run()`
      end-to-end: synthetic camera truth poses are fed into
      `_state["last_truth"]` (with fresh timestamps, mirroring how
      `_on_truth_thread` populates it in production), the runner is started
      against a `SimTransport`, and the loop is asserted to converge —
      terminating once within `eps` of the target and issuing a `STOP`.
  - [ ] The test also covers: a stale/missing truth pose does not crash the
        loop (it logs and waits, per the existing `run()` logic).
  - [ ] The test also covers: an explicit `stop()` call mid-pursuit halts
        the loop and issues `STOP`, without waiting for arrival or timeout.
- [ ] `_safe_stop()`'s bare `STOP` send is confirmed (via the sim) to
      actually cancel the Planner's active `G` goal — i.e. after `STOP`,
      the robot does not continue pursuing the last `G` target on
      subsequent ticks. **No change to use `DEV DT STOP` is made** — see
      Description above and `architecture-update.md` Decision 1.
- [ ] Stopping a running GOTO re-enables the GOTO button synchronously
      (mirrors `_stop_tour`'s fix, per `testgui-tour-stop-reactivation.md`).
- [ ] Any genuine bug the new test surfaces (e.g. a throttle/timeout
      constant needing adjustment against real 084 `G`/`SI` timing) is
      fixed in this ticket, documented here.

## Testing

- **Existing tests to run**: full `tests/testgui` suite (regression),
  including ticket 001/002's changes.
- **New tests to write**: `tests/testgui/test_goto.py` (net-new — no
  legacy equivalent to port, see Description).
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run pytest
  tests/testgui -q`
