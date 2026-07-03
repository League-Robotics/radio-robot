---
id: '003'
title: 'Robot object model: rename unit-suffixed identifiers across the Robot/Nezha/Cutebot
  family'
status: done
use-cases:
- SUC-002
depends-on:
- '002'
github-issue: ''
issue: remove-units-from-identifier-names-host-python.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Robot object model: rename unit-suffixed identifiers across the Robot/Nezha/Cutebot family

## Description

`host/robot_radio/robot/robot.py`, `robot_state.py`, `nezha.py`,
`nezha_state.py`, `nezha_kinematic.py`, `cutebot.py`, `clock_sync.py`,
`sync_pose.py`, and `kinematics/differential_drive.py` are the "robot
object" abstraction: a base class, two concrete implementations
(`Nezha`/`Cutebot`), per-tick state, kinematics, clock sync, and camera-fix
pose sync. All depend on `robot/protocol.py` (ticket 002, already renamed)
and share a base-class/consumer relationship — renaming together avoids
leaving `Robot` and one concrete subclass transiently inconsistent in
naming for no benefit (`architecture-update.md` Step 2).

**This is the sprint's largest single ticket** — 328 occurrences across 9
files (320 in the `robot/` files + 8 in `kinematics/differential_drive.py`)
— flagged explicitly in Open Question 2, mirroring 071's own largest-ticket
precedent. It is deliberately one ticket, not several, because
`robot.py`/`nezha*.py`/`cutebot.py` share the base-class coupling described
above; the self-review's own "no 'and'" cohesion test is waived for this
ticket for that reason (Step 2 justification, restated in the Architecture
Self-Review's Design Quality section).

Renames (per `architecture-update.md` Step 5):
- `robot/nezha.py`/`nezha_state.py`: `x_mm`/`y_mm` → `x`/`y  # [mm]`,
  `period_ms` → `period  # [ms]`, `left_mms`/`right_mms` → per-wheel bare
  names with `# [mm/s]`.
- `robot/cutebot.py`: `r_deg`/`l_deg` → `right`/`left  # [deg]`, mirroring
  `nezha.py`'s own per-wheel naming for consistency across the two concrete
  implementations.
- `robot/clock_sync.py`: `t_robot_ms`/`t0_ms`/`t1_ms` →
  `t_robot`/`t0`/`t1  # [ms]`.
- `robot/nezha_kinematic.py`: renamed to match
  `kinematics/differential_drive.py`'s own renamed parameters (rename both
  files together).
- `robot/robot.py`, `robot_state.py`, `sync_pose.py`: renamed consistently
  with the same convention (base-class fields/locals feeding the concrete
  subclasses).

`robot/connection.py` has zero unit-suffix hits (already clean, Step 1) —
included in this ticket's review scope for import-consistency, not edited.

## Hard Contract (applies to this and every sprint 076 ticket)

- **Pure rename — no behavioral change.** Encoder queries, odometry, and
  clock-sync results must be numerically identical to pre-076.
- **Every renamed declaration carries a `# [unit]` comment.**
- **No wire-key/token surface exists in this layer** (confirmed by Step 1's
  reading) — nothing to exclude here beyond not re-touching
  `robot/protocol.py` (ticket 002's concern).
- **Full suite green throughout**: `uv run python -m pytest -q` remains
  **2682 passed, 0 failed**.
- **Cross-cutting kwargs**: any call into `robot/protocol.py` using a
  ticket-002-renamed keyword argument (e.g. `read_timeout=`, `speed=`) must
  already use the converged name — if any stale `read_ms=`/`speed_mms=`
  call site is found in this ticket's files, fix it here.
- **Ignore environmental `data/robots` drift.**

## Acceptance Criteria

- [x] `robot/nezha.py`, `robot/nezha_state.py`: `x_mm`/`y_mm` → `x`/`y` with
      `# [mm]`; `period_ms` → `period` with `# [ms]`; `left_mms`/
      `right_mms` → per-wheel bare names with `# [mm/s]`.
- [x] `robot/cutebot.py`: `r_deg`/`l_deg` → `right`/`left` with `# [deg]`,
      matching `nezha.py`'s per-wheel naming pattern.
- [x] `robot/clock_sync.py`: `t_robot_ms`/`t0_ms`/`t1_ms` →
      `t_robot`/`t0`/`t1` with `# [ms]`.
- [x] `robot/nezha_kinematic.py` and `kinematics/differential_drive.py` are
      renamed together, using matching parameter names between the two
      files.
- [x] `robot/robot.py`, `robot/robot_state.py`, `robot/sync_pose.py` carry
      no unit-suffixed identifier and each carries `# [unit]` comments
      where applicable.
- [x] `robot/connection.py` is confirmed to remain clean (zero
      unit-suffixed identifiers) — no edit expected (`git diff` shows no
      changes to this file).
- [x] No renamed identifier collides across two previously-distinguished
      names (`docs/coding-standards.md`'s ambiguity-resolution rule) — spot
      check the base-class/subclass field overlaps in particular. (Found
      one real case: `nezha_state.py`'s `set_world_pose` and
      `nezha_kinematic.py`'s `anchor` both had a degrees-valued heading
      alongside a separately-computed centi-degrees value in the same
      scope; resolved by keeping the public parameter as bare `heading`
      and naming the derived wire-ready value `wire_heading  # [cdeg]`
      rather than re-embedding a unit suffix.)
- [x] `tests/simulation/unit/test_odom_tracker.py`,
      `test_serial_conn_reader.py`, and the clock-sync/kinematic unit
      tiers (per `usecases.md` SUC-002) pass with unchanged behavior.
      (`test_odom_tracker.py`/`test_serial_conn_reader.py` exercise
      ticket 001/004 surfaces untouched by this ticket and were already
      passing; `test_clock_sync.py`,
      `test_robot_vw_generator.py`, `test_robot_go_to_callback.py` — the
      actual clock-sync/kinematic call sites into this ticket's renamed
      API — updated and passing.)
- [x] If this ticket proves too large for one focused implementation
      session, sequence the work as separate commits **within this same
      ticket** along the `robot.py`/`nezha*.py` (base + differential-drive
      family) vs. `cutebot.py` (secondary implementation) seam — do **not**
      split into separate ticket files, since ticket 009 (rogo CLI) and
      ticket 010 (calibration CLI/MCP) both depend on this ticket as a
      single completed unit. (Completed in one focused pass; not split.)
- [x] Hard Contract above holds.

## Testing

- **Existing tests to run**: `tests/simulation/unit/test_odom_tracker.py`,
  `tests/simulation/unit/test_serial_conn_reader.py`, and any clock-sync or
  differential-drive kinematic unit test under `tests/simulation/unit/`
  (grep for `ClockSync`/`differential_drive`/`Nezha`/`Cutebot` imports to
  enumerate the complete set).
- **New tests to write**: none required — pure rename.
- **Verification command**: `uv run python -m pytest -q` (confirm 2682
  passed, 0 failed).

## Implementation Plan

**Approach**: Rename file-by-file, base class first, then each concrete
subclass, keeping `nezha_kinematic.py` and `differential_drive.py` in sync
since they share parameter names by design.

1. `robot/robot.py`, `robot/robot_state.py` — rename base-class
   fields/locals first, since `nezha.py`/`cutebot.py` inherit from these.
2. `robot/nezha.py`, `robot/nezha_state.py`, `robot/nezha_kinematic.py`,
   `kinematics/differential_drive.py` — rename together as one
   differential-drive family; verify `nezha_kinematic.py`'s renamed
   parameters match `differential_drive.py`'s.
3. `robot/cutebot.py` — rename its own encoder-side parameters
   (`r_deg`/`l_deg`), matching `nezha.py`'s per-wheel naming style chosen
   in step 2.
4. `robot/clock_sync.py`, `robot/sync_pose.py` — rename clock-offset and
   camera-fix pose-sync fields/locals.
5. Confirm `robot/connection.py` needs no edit (Step 1 finding).
6. Grep each renamed identifier across this 9-file set to confirm no
   internal call site was missed.
7. Run the clock-sync/kinematic/odometry unit tests, then the full suite.

**Files to create/modify**:
- `host/robot_radio/robot/robot.py`
- `host/robot_radio/robot/robot_state.py`
- `host/robot_radio/robot/nezha.py`
- `host/robot_radio/robot/nezha_state.py`
- `host/robot_radio/robot/nezha_kinematic.py`
- `host/robot_radio/robot/cutebot.py`
- `host/robot_radio/robot/clock_sync.py`
- `host/robot_radio/robot/sync_pose.py`
- `host/robot_radio/kinematics/differential_drive.py`
- `host/robot_radio/robot/connection.py` — reviewed only, no edit expected.

**Testing plan**: Run the odometry/clock-sync/kinematic unit tests
individually, then `uv run python -m pytest -q` and confirm the 2682
baseline holds.

**Documentation updates**: None in this ticket.
