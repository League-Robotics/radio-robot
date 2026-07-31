---
id: '001'
title: REPL ack-ring fix + halt_now() sweep across host call sites
status: open
use-cases: [SUC-004]
depends-on: []
github-issue: ''
issue:
- repl-confirm-reads-deleted-ack-slot-scan-the-ack-ring.md
- halt-now-call-sites-must-use-estop-and-never-swallow-failure.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# REPL ack-ring fix + halt_now() sweep across host call sites

**Worktree note**: implement in the git worktree checked out for
`sprint/128-complexity-reduction-delete-dead-code-unburden-the-robot-loop-fix-bit-rot`
(created via `acquire_execution_lock`). All paths below are relative to
that worktree, not the `master` checkout used for planning.

## Description

Two P0 host-side defects, fixed together because they touch the same
file (`io/repl.py`) and the same safety idiom:

1. `io/repl.py`'s `RogoSession.confirm()` reads `TLMFrame.ack`, a scalar
   field deleted by 124-008 — every motion-issuing verb (`twist`, `stop`,
   `config`, `raw`) crashes the whole REPL process with an
   `AttributeError` on the first ack-bearing frame.
2. Six-plus call sites across `io/cli.py`, `io/repl.py`, `io/calibrate.py`,
   `io/robot_mcp.py`, and `robot/cutebot.py` call the PLANNED `stop()`
   where the context means "halt now" (measured: `stop()` mid-leg rode
   out an entire 400 mm leg, 39.8 cm / 5.9 s; `estop()` halted in
   2.9 cm / 0.10 s), several swallowing the call's own failure in a bare
   `except Exception: pass`.

## Acceptance Criteria

- [ ] `confirm()` rewritten to scan `TLMFrame.acks` (the ring) for a
      matching `corr_id` instead of the deleted scalar `ack` field; a
      frame with an empty ring times out to `None` rather than raising.
- [ ] `pump()`'s stale docstring (still describing "your single ack
      slot") is fixed.
- [ ] A new shared `halt_now(proto, log=print)` helper exists (in
      `src/host/robot_radio/robot/halt.py`), modeled on
      `field/geofence.py::Geofence._halt()`: retries `estop()` 3x, logs
      `ERROR: estop() failed 3x -- ROBOT MAY STILL BE MOVING` and raises
      on total failure.
- [ ] Every site in the issue's own table (`io/robot_mcp.py:560`,
      `io/cli.py:540,555,576,675-679,765-769`, `io/repl.py:129-133`
      (`RogoSession.close()`), `io/calibrate.py:501-505,908-916`) calls
      `halt_now()` instead of `proto.stop()`.
- [ ] `robot/cutebot.py::speed()`'s `GeneratorExit` handler uses the same
      helper; `rotate()`'s guaranteed `NameError` (undefined name
      `robot` at line 180) is fixed or removed in the same touch.
- [ ] `field/geofence.py::Geofence._halt()` delegates to `halt_now()`
      (keeping its `GeofenceViolation` wrapper).
- [ ] `src/tests/CLAUDE.md`'s stale guidance (telling authors to use
      `stop()` in Ctrl-C handlers) is updated to name `estop()`/
      `halt_now()`.
- [ ] `grep -rn "proto\.stop()\|robot\.stop()\|_robot\.stop()" src/host/robot_radio/{io,robot}`
      finds only sites whose surrounding comment states a genuinely
      planned, sequenced stop.
- [ ] No halt call anywhere in `src/host` is wrapped in a bare
      `except Exception: pass` without a log line.
- [ ] `grep -n "\.ack\b" src/host/robot_radio/io/repl.py` returns nothing.

## Testing

- **Existing tests to run**: full `uv run python -m pytest` (repl and
  CLI-adjacent unit tests must still pass with the rewritten `confirm()`
  and the new `halt_now()` call sites).
- **New tests to write**:
  - A unit test constructing a `TLMFrame` with a populated `acks` ring,
    asserting `confirm()` matches by `corr_id`, and a second test with an
    empty ring asserting a timeout to `None` instead of a raise.
  - A unit test for `halt_now()`: succeeds on the first `estop()` call;
    retries up to 3x on failure; raises (and logs) after 3 failures.
- **Verification command**: `uv run python -m pytest src/tests/unit -k "repl or halt" -q`,
  then on the bench (RADIOBRIDGE relay or direct USB, confirm ROLE via
  `mbdeploy list`): `rogo repl` against the bench robot — `twist 150 0 0 800`,
  `stop`, `config pid.kp=...`, and `raw` each print a `corr_id=N OK` (or a
  real error string) and the REPL survives all of them; Ctrl-C during
  `rogo drive` halts wheels within ~0.1 s (watch encoders in telemetry).

## Implementation Notes

- **Approach**: fix `confirm()`'s ring-scan first (it is the smaller,
  more isolated change and unblocks using `rogo repl` for any later
  ticket's own bench verification in this sprint); then add `halt.py`
  and sweep the call sites.
- **Files to modify**: `src/host/robot_radio/io/repl.py`,
  `src/host/robot_radio/io/cli.py`, `src/host/robot_radio/io/calibrate.py`,
  `src/host/robot_radio/io/robot_mcp.py`, `src/host/robot_radio/robot/cutebot.py`,
  `src/host/robot_radio/field/geofence.py`, `src/tests/CLAUDE.md`.
- **Files to create**: `src/host/robot_radio/robot/halt.py`.
- **Documentation updates**: none beyond `src/tests/CLAUDE.md`'s guidance
  fix named above — this ticket does not touch a `DESIGN.md`.
