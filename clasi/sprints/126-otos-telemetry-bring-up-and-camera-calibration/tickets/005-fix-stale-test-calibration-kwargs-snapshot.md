---
id: '005'
title: Fix stale test_calibration_kwargs snapshot
status: open
use-cases: []
depends-on: ['003', '004']
github-issue: ''
issue: otos-telemetry-bring-up-and-camera-calibration.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix stale test_calibration_kwargs snapshot

## Description

`src/tests/unit/test_calibration_kwargs.py`'s
`test_calibration_commands_tovez_json_snapshot` pins the exact
`calibration_commands()` output for `data/robots/tovez.json`, including
`SET rotSlip=0.92` and the `OL`/`OA` (OTOS linear/angular scale, wire
register units) lines. This snapshot is **already stale independent of
this sprint** — the file currently pins `rotSlip=0.92` while
`tovez.json`'s live `rotational_slip` is `0.9117` (a mismatch predating
this sprint, from an earlier calibration session). Tickets 003 and 004
may additionally change `otos_linear_scale`/`otos_angular_scale`, which
would shift the snapshot's `OL`/`OA` lines too. Per the issue: any
`tovez.json` edit this sprint makes touches this same snapshot test, so
fixing it belongs in this sprint.

1. Read the current `data/robots/tovez.json` (post tickets 003/004 — this
   ticket runs after both, win or no-op).
2. Recompute the expected `calibration_commands()` output by hand (or by
   calling the real function against the current file) for both
   `tovez.json` and `tovez_nocal.json`.
3. Update `test_calibration_commands_tovez_json_snapshot`'s expected list
   to match: the `rotSlip` line to the file's current `rotational_slip`
   (`0.9117` unless a later ticket changed it), and the `OL`/`OA` lines to
   whatever ticket 003/004 landed (unchanged 67/-13 if neither ticket
   corrected its scale, or the new register values if one or both did).
4. Confirm `test_calibration_commands_tovez_nocal_json_snapshot` is still
   accurate (it reads a different file, `tovez_nocal.json`, untouched by
   this sprint — should need no change, but verify rather than assume).
5. Run the full sim suite and confirm the known-failure count drops from
   11 to 10 (the remaining 10 are the unrelated `testgui` sprint-108
   sim-mode tour-1 fault-baseline failures — do not attempt to fix those,
   out of scope).

Do not touch `calibration_kwargs()`/`calibration_commands()`
(`robot_radio/calibration/push.py`) themselves — this ticket fixes a
stale pinned expectation, not the code under test.

## Acceptance Criteria

- [ ] `test_calibration_commands_tovez_json_snapshot`'s expected command
      list matches the current `tovez.json` (correct `rotSlip`, and
      correct `OL`/`OA` reflecting whatever tickets 003/004 landed).
- [ ] `test_calibration_commands_tovez_nocal_json_snapshot` verified still
      correct (or fixed, if it turns out to have independently drifted).
- [ ] No production code (`robot_radio/calibration/push.py`) is changed by
      this ticket.
- [ ] `uv run python -m pytest src/tests/sim -q` shows exactly the known,
      reduced pre-existing-failure set (10, `testgui`-only) — zero
      failures in `test_calibration_kwargs.py`.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim -q`
  (full suite), plus targeted:
  `uv run python -m pytest src/tests/unit/test_calibration_kwargs.py -v`.
- **New tests to write**: None — this ticket corrects an existing
  snapshot's pinned expectation.
- **Verification command**: `uv run python -m pytest src/tests/sim -q`
