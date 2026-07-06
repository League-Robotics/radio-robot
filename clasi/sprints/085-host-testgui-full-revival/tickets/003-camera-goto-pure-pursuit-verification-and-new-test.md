---
id: '003'
title: 'Camera GOTO: pure-pursuit verification and new test'
status: done
use-cases:
- SUC-003
depends-on:
- '001'
github-issue: ''
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

- [x] A new `tests/testgui/test_goto.py` exercises `_GotoRunner.run()`
      end-to-end: synthetic camera truth poses are fed into
      `_state["last_truth"]` (with fresh timestamps, mirroring how
      `_on_truth_thread` populates it in production), the runner is started
      against a `SimTransport`, and the loop is asserted to converge —
      terminating once within `eps` of the target and issuing a `STOP`.
  - [x] The test also covers: a stale/missing truth pose does not crash the
        loop (it logs and waits, per the existing `run()` logic).
  - [x] The test also covers: an explicit `stop()` call mid-pursuit halts
        the loop and issues `STOP`, without waiting for arrival or timeout.
- [x] `_safe_stop()`'s bare `STOP` send is confirmed (via the sim) to
      actually cancel the Planner's active `G` goal — i.e. after `STOP`,
      the robot does not continue pursuing the last `G` target on
      subsequent ticks. **No change to use `DEV DT STOP` is made** — see
      Description above and `architecture-update.md` Decision 1.
- [x] Stopping a running GOTO re-enables the GOTO button synchronously
      (mirrors `_stop_tour`'s fix, per `testgui-tour-stop-reactivation.md`).
- [x] Any genuine bug the new test surfaces (e.g. a throttle/timeout
      constant needing adjustment against real 084 `G`/`SI` timing) is
      fixed in this ticket, documented here.

## Testing

- **Existing tests to run**: full `tests/testgui` suite (regression),
  including ticket 001/002's changes.
- **New tests to write**: `tests/testgui/test_goto.py` (net-new — no
  legacy equivalent to port, see Description).
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run pytest
  tests/testgui -q`

## Implementation Notes (post-hoc)

`tests/testgui/test_goto.py` (7 tests) is split into three parts, because the
acceptance criteria need genuinely different fidelity levels and
`_GotoRunner` is a `QObject` nested inside `_build_main_window()` with no
import seam (the same constraint every other worker in `__main__.py` is
under):

- **Part A** (4 tests, Qt-free, no sim lib needed) — inline reimplements
  `run()`/`_safe_stop()`'s exact control flow (verified line-for-line against
  `__main__.py`) and drives it with directly-controlled synthetic
  `state["last_truth"]` tuples — Decision 2's own "convergence-under-
  synthetic-truth" strategy. Covers: convergence + `STOP` on arrival; a
  stale truth pose (older than `TRUTH_MAX_AGE_S`) is never treated as fresh
  and the loop times out cleanly instead of crashing; a missing truth pose
  behaves identically; an explicit `stop()` flag halts the loop immediately
  (not waiting for a 30 s timeout) and still issues `STOP`.
- **Part B** (1 test, requires the compiled sim lib) — sends the *exact*
  wire commands `_GotoRunner` uses (`G ...` then the bare `STOP`
  `_safe_stop()` sends) directly against a real `SimTransport`
  (sprint-084 `source/` firmware), and watches telemetry `mode=` settle to
  and stay `'I'` afterward. This is the direct, real-firmware confirmation
  of Decision 1: `mode=` is `Subsystems::Planner::state().mode`'s sole
  wire representation (`telemetry_commands.cpp`), so a `mode` that stayed
  `'G'` after `STOP` would prove the goal was NOT cancelled (the exact
  regression a `DEV DT STOP` substitution would cause, per 084 Open
  Question 3). Observed: `mode` reaches `'G'` after `G 6000 0 200`, then
  settles to and stays `'I'` for all of the last 10 telemetry samples
  observed after the bare `STOP`.
- **Part C** (2 tests, requires the compiled sim lib) — drives the REAL
  `_GotoRunner`/`QThread`/`SimTransport` stack through the actual GUI
  widgets (mirrors `test_tour1_geometry.py`'s pattern), for the
  highest-fidelity confirmation: (1) clicking `goto_btn` with target
  (900, 0) mm, eps 80 mm, speed 250 mm/s converges and re-enables the
  button — **measured final distance to target: 24.6 mm** (well inside
  eps); (2) clicking Operations' `STOP` mid-pursuit (target (8000, 0), far
  enough it cannot have arrived) re-enables `goto_btn` synchronously,
  right inside the click — no additional event-loop spin needed — mirroring
  `_stop_tour`'s identical fix and complementing `test_tour_stop.py`'s
  existing deterministic fake-widget version of the same assertion
  (`test_stop_goto_reenables_button_synchronously`).

**Genuine bug found and fixed**: the GOTO `x`/`y`/`eps`/`speed` `QSpinBox`es
(`_make_goto_spin` in `__main__.py`) had **no `objectName`** — unlike the
tour buttons (`tour_btn_*`) and Sim Errors spinboxes (`sim_err_*`), which
made it impossible to drive a real end-to-end GOTO test through the actual
widgets (`findChild` had nothing to find). This is exactly the kind of gap
Grounding fact 4 predicted ("camera GOTO has no historical test at all") —
no historical test ever needed this seam because no historical test existed.
Fixed by adding `goto_spin_{x,y,eps,speed}` object names — a test-seam-only
change, zero behavior change. No throttle/timeout constant needed
retuning: `POLL_S`/`TRUTH_MAX_AGE_S`/`TIMEOUT_S` all held against real
sprint-084 `G`/`SI`/`STOP` timing without adjustment.

Full `tests/testgui` suite: 161 passed (regression, including tickets
001/002). Full default suite (`uv run python -m pytest`): 408 passed.
