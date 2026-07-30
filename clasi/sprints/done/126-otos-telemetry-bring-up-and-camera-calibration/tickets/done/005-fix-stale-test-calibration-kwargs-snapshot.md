---
id: '005'
title: Fix stale test_calibration_kwargs snapshot
status: done
use-cases: []
depends-on:
- '003'
- '004'
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

- [x] `test_calibration_commands_tovez_json_snapshot`'s expected command
      list matches the current `tovez.json` (correct `rotSlip`, and
      correct `OL`/`OA` reflecting whatever tickets 003/004 landed).
- [x] `test_calibration_commands_tovez_nocal_json_snapshot` verified still
      correct (or fixed, if it turns out to have independently drifted).
- [x] No production code (`robot_radio/calibration/push.py`) is changed by
      this ticket.
- [x] `uv run python -m pytest src/tests/sim -q` shows exactly the known,
      reduced pre-existing-failure set (10, `testgui`-only) — zero
      failures in `test_calibration_kwargs.py`.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim -q`
  (full suite), plus targeted:
  `uv run python -m pytest src/tests/unit/test_calibration_kwargs.py -v`.
- **New tests to write**: None — this ticket corrects an existing
  snapshot's pinned expectation.
- **Verification command**: `uv run python -m pytest src/tests/sim -q`

## Completion Notes

Both drifts confirmed and fixed in one pass, by calling the real
`calibration_commands()` against the live `data/robots/tovez.json` rather
than hand-deriving:

- `rotSlip`: `0.92` -> `0.9117` (pre-existing drift, predates sprint 126 —
  confirmed by `git log` on `tovez.json`: `rotational_slip` was last
  touched by a pre-126 commit, not by tickets 003/004).
- `OL`: `67` -> `28`. 126-003's `otos_linear_scale` correction (1.067 ->
  1.0275) changes the `scale_to_int8()`-encoded register value:
  `(1.0275-1)*1000 = 27.5` -> rounds to 28. `OA` unchanged at `-13`:
  126-004 confirmed `otos_angular_scale` (0.987) correct as committed, no
  edit, so `(0.987-1)*1000 = -13` is unchanged.

`test_calibration_commands_tovez_nocal_json_snapshot` re-run against the
live `tovez_nocal.json`: reproduces the existing pinned list exactly
(`rotSlip=1`, `OL 0`, `OA 0`) — confirmed still accurate, no edit needed
(that profile is untouched by this sprint, all three fields are
uncalibrated sentinels).

**Is this snapshot worth keeping, or is it a pure change-detector?** Worth
keeping. It is not simply mirroring the JSON back at itself — it pins the
output of real formatting/encoding logic exercised against a real shipped
profile: the `%.6f` vs `%g` vs plain-int branching per wire key
(`_SIX_DECIMAL_KEYS`), the field selection and ORDER `calibration_kwargs()`
produces, and critically the `scale_to_int8()` non-trivial encoding of
`otos_linear_scale`/`otos_angular_scale` into the `OL`/`OA` register
values — none of which a reader of `tovez.json` alone could reconstruct
without re-deriving the encoding. A regression in any of those (wrong
decimal count, wrong key order, a broken `scale_to_int8`) would be caught
here and nowhere else in the unit suite. The two drifts this ticket fixed
were both legitimate upstream data changes, not test bugs — exactly the
failure mode this kind of snapshot is supposed to catch, working as
intended.

No production code (`robot_radio/calibration/push.py`) touched.

`uv run python -m pytest src/tests/unit/test_calibration_kwargs.py -v`:
6 passed.

`uv run python -m pytest src/tests/sim -q`: 423 passed, 1 skipped, 1
xfailed (0 failures — matches ticket 001/002's pre-005 baseline of 423
passed exactly, confirming the reduced known-failure set with zero
`test_calibration_kwargs.py` failures).
