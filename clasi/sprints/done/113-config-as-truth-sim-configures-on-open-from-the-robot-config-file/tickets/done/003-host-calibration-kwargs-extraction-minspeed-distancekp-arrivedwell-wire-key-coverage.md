---
id: '003'
title: 'Host: calibration_kwargs() extraction + minSpeed/distanceKp/arriveDwell wire-key
  coverage'
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on: []
github-issue: ''
issue: config-as-truth-sim-configure-on-open.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host: calibration_kwargs() extraction + minSpeed/distanceKp/arriveDwell wire-key coverage

## Description

Two independent, small fixes to the host's existing Tier-1 (already-live-wire)
config-push code, bundled because both touch the same two files:

**1. DRY extraction.** `calibration_commands()`
(`src/host/robot_radio/calibration/push.py`) currently does two things at
once: (a) decide *which* fields to push from a `RobotConfig` and (b) format
them as text `SET key=value` command strings for `SerialConnection.send()`
(the hardware/CLI path). Ticket 005 needs (a) *without* (b) — `SimLoop`
should call `NezhaProtocol.set_config(**kwargs)` directly with the resolved
kwargs, not round-trip through text parsing. Extract (a) into a new function
`calibration_kwargs(config) -> dict[str, float | int]` returning the flat
wire-key kwargs dict (`{"ml": ..., "pid.kp": ..., "headingKp": ..., ...}`);
`calibration_commands()` becomes a thin wrapper: call `calibration_kwargs()`,
then format each item as `f"SET {key}={_format(value)}"` with the same
timeouts it already assigns per key. **Must be behavior-preserving** —
`calibration_commands()`'s existing callers (`cli.py`, `turn_shape.py`,
`__main__.py`'s manual robot-select, and every existing test that asserts on
its returned text-command list) see byte-identical output.

**2. Missing wire-key coverage.** `config.proto`'s `PlannerConfigPatch`
already curates `min_speed` (field 1), `arrive_dwell` (field 20), and
`distance_kp` (field 21) as live-tunable — real firmware and the sim both
already apply them via `RobotLoop::handleConfig()`/`Pilot::
applyPlannerPatch()`. But `protocol.py`'s `_PLANNER_KEYS` dict (the Python
flat-key vocabulary `NezhaProtocol.set_config()`/`.config()` consult) only
maps `minSpeed`/`headingKp`/`headingKd` — `distanceKp` and `arriveDwell` have
no host-side key at all, and `calibration_kwargs()`/`calibration_commands()`
never pushes `minSpeed` even though a key exists for it. This means these
three already-live fields silently never reach either the sim or real
hardware today. Add the missing `_PLANNER_KEYS` entries and add all three to
`calibration_kwargs()`'s pushed set (reading `config.control.min_speed`/
`distance_kp`/`arrive_dwell`, following the exact same "push only when
present" pattern the existing `headingKp`/`headingKd` entries use).

## Acceptance Criteria

- [x] `src/host/robot_radio/robot/protocol.py`'s `_PLANNER_KEYS` gains
      `"distanceKp": "distance_kp"` and `"arriveDwell": "arrive_dwell"`
      (mirrors the existing `"minSpeed": "min_speed"` entry's shape exactly).
      `_ALL_SET_KEYS` picks these up automatically (it's derived from
      `_PLANNER_KEYS`).
- [x] `src/host/robot_radio/calibration/push.py` gains
      `calibration_kwargs(config) -> dict[str, float]`, covering exactly the
      same field set `calibration_commands()` currently builds (`ml`, `mr`,
      `tw`, `rotSlip`, `pid.kp/ki/kff/iMax/kaw`, `headingKp`, `headingKd`)
      **plus** the three new keys (`minSpeed`, `distanceKp`, `arriveDwell`,
      each pushed only when `config.control.min_speed`/`distance_kp`/
      `arrive_dwell` is not `None`, same "push only when present" rule as
      `headingKp`). `OI`/`OL`/`OA` (OTOS) stay OUT of
      `calibration_kwargs()` — they are not `SET key=value` verbs at all
      (see `calibration_commands()`'s own docstring on `otos_config()`
      being a separate mechanism) and have no place in a flat kwargs dict;
      `calibration_commands()` keeps building them directly, unchanged.
- [x] `calibration_commands(config)` is rewritten to call
      `calibration_kwargs(config)` and format each item into the existing
      `(command, read_timeout)` tuple list, THEN append the unchanged
      `OI`/`OL`/`OA` sequence — producing an **identical** list to today's
      output for every existing test fixture (verified by the regression
      test below). Per-key `read_timeout` values are preserved exactly
      (200ms for `SET` keys, 500/200 for `OI`/`OL`/`OA` as today).
- [x] No change to `calibration_commands()`'s public signature or return
      shape.

## Testing

- **Existing tests to run**: `src/tests/testgui/test_calibration_push_on_connect.py`
  (the primary regression surface — asserts on `calibration_commands()`'s
  output shape/content) and any other test importing
  `calibration_commands`/`calibration/push.py` (grep for
  `from robot_radio.calibration.push import` across `src/tests/`) — must
  pass unchanged.
- **New tests to write**: a direct unit test for `calibration_kwargs()`
  (no sim lib, no Qt needed) asserting: (a) it returns the same field set
  `calibration_commands()`'s pre-refactor text list implied, on both
  `tovez.json`-shaped and `tovez_nocal.json`-shaped `RobotConfig` fixtures;
  (b) `minSpeed`/`distanceKp`/`arriveDwell` are present when the source
  config has `control.min_speed`/`distance_kp`/`arrive_dwell` set, absent
  when not; (c) `calibration_commands()`'s output is unchanged before/after
  the refactor (a snapshot-style comparison against the pre-refactor
  expected list is the most direct proof).
- **Verification command**:
  `uv run python -m pytest src/tests/testgui/test_calibration_push_on_connect.py -v`,
  then the full suite.

## Files to touch

- `src/host/robot_radio/robot/protocol.py` (`_PLANNER_KEYS` two new entries)
- `src/host/robot_radio/calibration/push.py` (`calibration_kwargs()` new,
  `calibration_commands()` refactored to a thin wrapper)
- New/extended: a unit test file for `calibration_kwargs()` under
  `src/tests/` (co-locate near `test_calibration_push_on_connect.py` or as a
  new `src/tests/unit/test_calibration_kwargs.py` — no sim/Qt dependency).

## Depends On

None — independent of the proto/SimHarness work in tickets 001/002.
