---
id: '004'
title: 'Sensor modules: rename unit-suffixed identifiers in host-side sensor reading
  and classification'
status: open
use-cases:
- SUC-003
depends-on:
- '002'
github-issue: ''
issue: remove-units-from-identifier-names-host-python.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sensor modules: rename unit-suffixed identifiers in host-side sensor reading and classification

## Description

`host/robot_radio/sensors/otos.py`, `color.py`, `calibration.py`,
`motion_monitor.py`, and `odom_tracker.py` read and classify onboard
sensors (line, color, OTOS, encoder-derived odometry) from the host side.
This subsystem depends only on `robot/protocol.py` (via `parse_tlm`,
ticket 002 — already renamed) and `io/serial_conn.py` (ticket 001), **not**
on the `robot/` object-model layer (ticket 003) — independent of that
ticket's rename order (`architecture-update.md` Step 2).

Renames (per Step 5): `sensors/odom_tracker.py` (`x_mm`/`y_mm`/
`trackwidth_mm` → bare names with `# [mm]`); `sensors/otos.py`;
`sensors/color.py` (`h_deg` → `hue  # [deg]`, `brightness_pct` →
`brightness  # [%]`); `sensors/calibration.py`; `sensors/motion_monitor.py`.

`sensors/odometry.py` has zero unit-suffix hits (already clean, Step 1) and
is in this ticket's review scope only because it is imported by files that
do have hits — no edit expected. `robot/protocol.py` is consumed here (via
`parse_tlm`) but not re-renamed — that was ticket 002's concern.

Total scope: 121 rename-eligible occurrences (Step 3).

## Hard Contract (applies to this and every sprint 076 ticket)

- **Pure rename — no behavioral change.** Sensor readings and
  classifications must be bit-for-bit identical to pre-076 for the same
  input stream.
- **Every renamed declaration carries a `# [unit]` comment.**
- **Wire tokens are STABLE**: this subsystem parses `robot.protocol`'s
  already-unchanged `kv` dict keys (`enc`, `pose`, `otos`, `encpose`,
  `otos_health`, `line`, `color`) — do not touch any string literal that
  matches one of these wire tokens inside `sensors/`.
- **Full suite green throughout**: `uv run python -m pytest -q` remains
  **2682 passed, 0 failed**.
- **Cross-cutting kwargs**: any call into `robot/protocol.py`
  (`parse_tlm`, etc.) using a ticket-002-renamed keyword argument must
  already use the converged name; fix any stale one found in this ticket's
  files here.
- **Ignore environmental `data/robots` drift.**

## Acceptance Criteria

- [ ] `sensors/odom_tracker.py`: `x_mm`/`y_mm`/`trackwidth_mm` → bare names
      with `# [mm]`.
- [ ] `sensors/otos.py`: all unit-suffixed identifiers renamed with
      `# [unit]` comments matching the file's existing unit vocabulary.
- [ ] `sensors/color.py`: `h_deg` → `hue` with `# [deg]`; `brightness_pct`
      → `brightness` with `# [%]`.
- [ ] `sensors/calibration.py`, `sensors/motion_monitor.py`: all
      unit-suffixed identifiers renamed with `# [unit]` comments.
- [ ] `sensors/odometry.py` is confirmed to remain clean (zero
      unit-suffixed identifiers) — no edit expected.
- [ ] No wire-token string literal (`"enc"`, `"pose"`, `"otos"`,
      `"encpose"`, `"otos_health"`, `"line"`, `"color"`, etc.) is altered
      anywhere in this ticket's files — diff-confirm byte-identical to
      pre-076.
- [ ] Sensor-related unit tests in `tests/simulation/unit/` (per
      `usecases.md` SUC-003) pass with unchanged numeric assertions.
- [ ] Hard Contract above holds.

## Testing

- **Existing tests to run**: sensor-related unit tests in
  `tests/simulation/unit/` (grep for `Otos`/`ColorClassifier`/
  `OdomTracker`/`MotionMonitor` imports to enumerate the exact files).
- **New tests to write**: none required — pure rename.
- **Verification command**: `uv run python -m pytest -q` (confirm 2682
  passed, 0 failed).

## Implementation Plan

**Approach**: Rename file-by-file; this subsystem has no internal
dependency on ticket 003 (robot object model), so it can proceed
immediately once ticket 002 lands.

1. `sensors/odom_tracker.py` — rename `x_mm`/`y_mm`/`trackwidth_mm` and any
   other unit-suffixed field/local.
2. `sensors/otos.py` — rename all unit-suffixed identifiers.
3. `sensors/color.py` — rename `h_deg` → `hue`, `brightness_pct` →
   `brightness`, and any other unit-suffixed identifier.
4. `sensors/calibration.py`, `sensors/motion_monitor.py` — rename
   remaining unit-suffixed identifiers.
5. Confirm `sensors/odometry.py` needs no edit.
6. Grep each renamed identifier across this file set to confirm no
   internal call site was missed, and confirm every wire-token string
   literal this subsystem reads is untouched.
7. Run sensor-related unit tests, then the full suite.

**Files to create/modify**:
- `host/robot_radio/sensors/otos.py`
- `host/robot_radio/sensors/color.py`
- `host/robot_radio/sensors/calibration.py`
- `host/robot_radio/sensors/motion_monitor.py`
- `host/robot_radio/sensors/odom_tracker.py`
- `host/robot_radio/sensors/odometry.py` — reviewed only, no edit expected.

**Testing plan**: Run sensor-related unit tests individually, then
`uv run python -m pytest -q` and confirm the 2682 baseline holds.

**Documentation updates**: None in this ticket.
