---
id: '009'
title: 'rogo CLI: rename unit-suffixed identifiers in the console-script entry point'
status: open
use-cases:
- SUC-007
depends-on:
- '003'
- '004'
- '005'
- '006'
github-issue: ''
issue: remove-units-from-identifier-names-host-python.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# rogo CLI: rename unit-suffixed identifiers in the console-script entry point

## Description

`host/robot_radio/io/cli.py` implements the `rogo` console-script — the
single largest file in this sprint's scope (170 rename-eligible
occurrences). It sits at the top of the host dependency graph, importing
from `robot/` (including `Cutebot`, ticket 003), `sensors/color.py`
(ticket 004), `config/robot_config.py` (excluded, ticket 011 confirms),
`calibration/helpers.py` (ticket 005), and `nav/camera_goto.py` (ticket
006) — it must be renamed **after** every subpackage it imports from, or
call sites would be renamed into not-yet-renamed functions and break
immediately (`architecture-update.md` Step 5 "Why").

Renames (per Step 5): `read_ms` → `read_timeout`; `left_mm_per_deg`/
`right_mm_per_deg` → matching ticket 005's calibration naming choice
exactly (do not diverge); `watchdog_ms`/`resend_ms`/`t_ms` → bare names
with `# [ms]`; `x_mm`/`y_mm`/`h_deg`/`angle_deg` → bare names with
`# [unit]`.

**Wire-key pairing to leave untouched**: `io/cli.py` builds pairs like
`("minWheelMms", getattr(ctrl, "min_wheel_mms", None))` — both halves of
this pairing are already-excluded surfaces (the string is a `SET`/`SIMSET`
wire key; `min_wheel_mms` is a pydantic attribute name on `RobotConfig`,
excluded wholesale per ticket 011's Exclusion Table confirmation). Neither
half is renamed by this ticket.

Total scope: 170 occurrences, 1 file.

## Hard Contract (applies to this and every sprint 076 ticket)

- **Pure rename — no behavioral change.** Every `rogo` subcommand behaves
  identically to pre-076; output formatting and exit codes unchanged.
- **Every renamed declaration carries a `# [unit]` comment.**
- **Wire keys and pydantic attribute names are STABLE**: any
  `("wireKeyString", getattr(ctrl, "pydantic_attr_name", ...))`-style
  pairing in this file keeps both halves exactly as-is.
- **Full suite green throughout**: `uv run python -m pytest -q` remains
  **2682 passed, 0 failed**.
- **Cross-cutting kwargs**: `read_ms` → `read_timeout` and every other
  cross-cutting rename decided in tickets 001–006 must be used
  consistently in this file — do not invent an alternative spelling.
- **Manual verification required**: no automated CLI-invocation test
  exists for `rogo` today (`tests/simulation/unit/test_cli.py` covers
  internal logic, not the console-script invocation itself) — run `rogo
  help` and a representative smoke command manually pre/post-rename and
  confirm identical output.
- **Ignore environmental `data/robots` drift.**

## Acceptance Criteria

- [ ] `read_ms` → `read_timeout` with `# [ms]`, matching ticket 001/002's
      decided name.
- [ ] `left_mm_per_deg`/`right_mm_per_deg` renamed to **exactly** the name
      ticket 005 chose in `calibration/push.py` (grep ticket 005's landed
      code first, do not choose independently).
- [ ] `watchdog_ms`/`resend_ms`/`t_ms` → bare names with `# [ms]`.
- [ ] `x_mm`/`y_mm`/`h_deg`/`angle_deg` → bare names with `# [unit]`
      (`mm`, `deg` respectively).
- [ ] Every wire-key/pydantic-attribute pairing this file builds (e.g. the
      `minWheelMms`/`min_wheel_mms` pair) is untouched — both halves
      byte-identical to pre-076.
- [ ] `rogo help` and at least one representative subcommand round-trip
      identically pre/post-sprint (manual verification, documented in the
      PR/commit description since no automated CLI-invocation test
      exists).
- [ ] `tests/simulation/unit/test_cli.py` passes unchanged (per
      `usecases.md` SUC-007).
- [ ] Hard Contract above holds.

## Testing

- **Existing tests to run**: `tests/simulation/unit/test_cli.py`.
- **New tests to write**: none required — pure rename. (No automated
  CLI-invocation test exists; manual verification substitutes, per the
  Hard Contract above.)
- **Verification command**: `uv run python -m pytest -q` (confirm 2682
  passed, 0 failed), plus a manual `rogo help` and one representative
  subcommand invocation.

## Implementation Plan

**Approach**: Read `io/cli.py` in full first to enumerate every
unit-suffixed identifier and every wire-key/pydantic-attribute pairing,
since this is the largest single file in the sprint and the most
consumer-facing.

1. Confirm ticket 005's exact chosen name for
   `left_mm_per_deg`/`right_mm_per_deg` before starting (grep
   `calibration/push.py`'s landed code) — use the identical name here.
2. Rename `read_ms` → `read_timeout`, `watchdog_ms`/`resend_ms`/`t_ms`,
   and `x_mm`/`y_mm`/`h_deg`/`angle_deg` throughout the file, adding
   `# [unit]` comments.
3. Locate every wire-key/pydantic-attribute pairing (grep for
   `getattr(ctrl,` and any `SET`/`SIMSET`/`GET` string literal builder);
   confirm neither half is touched.
4. Grep the whole file for every renamed identifier's old name to confirm
   no call site was missed.
5. Run `tests/simulation/unit/test_cli.py`, then the full suite.
6. Manually run `rogo help` and one representative subcommand (e.g. a
   dry-run or status command that doesn't require live hardware) and
   confirm output is unchanged from a pre-rename run.

**Files to create/modify**:
- `host/robot_radio/io/cli.py` — the only file this ticket touches.

**Testing plan**: Run `tests/simulation/unit/test_cli.py`, then
`uv run python -m pytest -q` and confirm the 2682 baseline holds. Manually
invoke `rogo help` and a representative subcommand.

**Documentation updates**: None in this ticket.
