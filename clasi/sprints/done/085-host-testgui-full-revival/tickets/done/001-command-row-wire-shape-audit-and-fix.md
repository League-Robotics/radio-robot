---
id: '001'
title: Command-row wire-shape audit and fix
status: done
use-cases:
- SUC-002
depends-on: []
github-issue: ''
issue: host-testgui-full-revival.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Command-row wire-shape audit and fix

## Description

`host/robot_radio/testgui/commands.py`'s `COMMANDS` schema drives the
interactive `S`/`T`/`D`/`R`/`TURN`/`RT`/`G` command rows. A direct
comparison against `docs/protocol-v2.md` §10 (the firmware's own
documented, sprint-084-implemented ranges) during sprint 085 planning found
two genuine range mismatches — the UI currently allows the operator to
enter values the firmware will reject with `ERR range ...`:

- `TURN`'s `eps` field: UI allows 0–180° (0–18000 cdeg); firmware
  (`source/commands/motion_commands.cpp`, §10 `### TURN`) only accepts
  10–1800 cdeg (0.1°–18°).
- `RT`'s `deg` field: UI allows ±3600° (±360000 cdeg); firmware (§10
  `### RT`) only accepts ±180000 cdeg (±1800°, up to 5 full turns).

Every other row's range already matches the firmware exactly (verified
during planning, see `architecture-update.md` Grounding fact 2): `S`/`T`/
`D`/`R`'s velocity fields (±1000 mm/s), `D`'s distance (1–10000 mm), `R`'s
radius (±10000 mm), `TURN`'s heading (any UI input wraps onto ±180° before
centidegree conversion, matching the firmware's ±18000 cdeg exactly), `T`'s
duration (1–30000 ms), and `G`'s x/y (±10000 mm) and speed (1–1000 mm/s).

This ticket fixes the two mismatches and records the full audit (as a code
comment and as this ticket's acceptance evidence) so the next person to
touch `commands.py` has the range table in one place instead of having to
re-derive it from `docs/protocol-v2.md`.

This ticket is foundational for tickets 002 (Tours) and 003 (Camera GOTO),
which read `commands.py`'s `TOURS`/`goto_distance`/`goto_reached` — not
because those tickets are blocked by this one technically (the hardcoded
`TOUR_1`/`TOUR_2` wire strings are already within range), but because
fixing the base command layer first is the lowest-risk sequencing for the
sprint.

## Acceptance Criteria

- [x] `commands.py`'s `TURN` row's `eps` param spec has `max: 18` (degrees;
      18° × 100 = 1800 cdeg, matching the firmware ceiling). `min` stays 0
      (the omit-if-zero sentinel for "use firmware default of 300 cdeg").
- [x] `commands.py`'s `RT` row's `deg` param spec has `min: -1800`,
      `max: 1800` (degrees; ±1800° × 100 = ±180000 cdeg, matching the
      firmware ceiling).
- [x] A code comment above `COMMANDS` (or inline on each row) records the
      firmware-range citation (`docs/protocol-v2.md` §10 section name) each
      row's `min`/`max` was checked against, so future edits have the
      reference in one place.
- [x] `tests/testgui/test_commands.py` (already ported, sprint 083) gains
      test cases asserting the corrected `TURN.eps` and `RT.deg` bounds,
      and a table-driven test (or equivalent) confirming every row's
      declared range is within the corresponding `docs/protocol-v2.md` §10
      range — so a future accidental range widening is caught by CI, not
      by another manual audit.
- [x] No other row's `min`/`max`/`default` changes — this ticket is a
      two-field fix, not a schema rewrite.

## Testing

- **Existing tests to run**: `QT_QPA_PLATFORM=offscreen uv run pytest
  tests/testgui/test_commands.py -q` (must stay green); full
  `tests/testgui` suite as a regression check.
- **New tests to write**: extend `test_commands.py` with the corrected
  `TURN.eps`/`RT.deg` bound assertions and the range-vs-firmware
  table-driven check described above.
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run pytest
  tests/testgui -q`
