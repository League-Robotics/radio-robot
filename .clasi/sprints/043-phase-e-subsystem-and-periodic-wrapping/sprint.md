---
id: '043'
title: "Phase E \u2014 Subsystem and periodic wrapping"
status: roadmap
branch: sprint/043-phase-e-subsystem-and-periodic-wrapping
use-cases: []
issues:
- migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 043: Phase E — Subsystem and periodic wrapping

## Goals

Wrap Drive, Gripper, and sensor clusters as subsystems with `periodic()` /
`updateInputs()` methods, and establish the `source/subsystems/` directory layout (§5).
`loopTickOnce` calls them in the same order it does today. Bodies moved verbatim; no
behavior changes.

Depends on: Sprint 042 (Phase D) — `Superstructure` must exist before subsystems can be
called from a known periodic orchestration point.

## Problem

Drive control, gripper, and sensor reads are currently called directly from `loopTickOnce`
and `Robot` with no subsystem abstraction. There is no `updateInputs()` convention — each
subsystem reads its device at scattered call sites. This makes the §5 directory layout
(with `subsystems/drive/`, `subsystems/gripper/`, `subsystems/sensors/`) unreachable
until the subsystem objects exist.

## Solution

Following the "Phase E — subsystem/periodic" entry in the migration sequence and §5
directory layout:

- Create `source/subsystems/` with subdirs: `drive/Drive.{h,cpp}`,
  `gripper/Gripper.{h,cpp}`, `sensors/{Line,Color,Ports}.{h,cpp}`.
- Each subsystem wraps its device handle(s) and exposes `updateInputs()` (reads device
  state into a local inputs slice — the pattern for the TLM logging contract in Phase F)
  and `periodic()` (drives the subsystem each cycle).
- `loopTickOnce` calls subsystems in the same order it currently calls the underlying
  devices — no reordering, no new behavior.
- `Gripper` is optional (`has_gripper=false` / `GripperIONull` null-object pattern as
  noted in issue "What does NOT apply" section).
- Move bodies verbatim from their current scattered locations into the subsystem wrappers.
- The `inputs` slice per subsystem is the logging seam pre-cut for Phase F (no TLM
  repoint yet).

## Key Deliverables

- `source/subsystems/drive/Drive.{h,cpp}` wrapping drive motor control.
- `source/subsystems/gripper/Gripper.{h,cpp}` with optional `GripperIONull`.
- `source/subsystems/sensors/LineSensor.{h,cpp}`, `ColorSensor.{h,cpp}`,
  `Ports.{h,cpp}` (or equivalent groupings).
- Each subsystem has `updateInputs()` and `periodic()`.
- `loopTickOnce` calls subsystems in the original order; net behavior identical.
- All behavior-preservation fences (wedge-hardening, goto-bounds, incident-scenarios,
  watchdog-exemption) still green.

## Scope

### In Scope

- `source/subsystems/` directory with Drive, Gripper, sensor subsystems.
- `updateInputs()` / `periodic()` convention on each subsystem.
- `GripperIONull` null-object for `has_gripper=false`.
- `loopTickOnce` update to call subsystems (same order as today).
- Pre-cut inputs slices (structs) per subsystem.

### Out of Scope

- TLM reader repoint from `HardwareState` to subsystem inputs slices (Phase F).
- `RobotState.h` split / old header deletion (Phase F).
- Any `updateInputs()` logging enforcement ("no subsystem prints" rule enforced in
  Phase F).
- New subsystem behavior or error handling.

## Architecture Notes

- The cooperative `loopTickOnce` replaces FRC's `CommandScheduler`/`SubsystemBase`;
  `periodic()` is the per-tick hook, called in a fixed order.
- `updateInputs()` is the seam that lets Phase F's TLM logging contract ("every subsystem
  writes its inputs slice in `updateInputs`, no subsystem prints") be enforced cleanly.
- Gripper is already optional in the firmware (`has_gripper` config field); the
  `GripperIONull` null-object makes this explicit rather than guarded by conditionals.
- Zero-heap: all subsystem instances are value members or static; no dynamic allocation.

## Definition of Done (Phase E — from issue migration sequence)

- [ ] `source/subsystems/drive/Drive.{h,cpp}` compiles with `updateInputs()` + `periodic()`.
- [ ] `source/subsystems/gripper/Gripper.{h,cpp}` compiles with `GripperIONull` variant.
- [ ] Sensor subsystems compile with `updateInputs()` + `periodic()`.
- [ ] `loopTickOnce` calls subsystems in original order; no reordering.
- [ ] All behavior-preservation fences still green (`test_033_005_wedge_hardening.py`,
      `test_goto_bounds.py`, `test_incident_scenarios.py`, `test_watchdog_exemption.py`).
- [ ] Simulation tier green (≥ 1954 tests): `uv run --with pytest python -m pytest -q`
- [ ] `defaultRobotConfig()` field-pin diff empty.
- [ ] Golden-TLM frame canary unchanged.
- [ ] Vendor-confinement grep gate passes (Phase E scope).
- [ ] No new heap allocation or fibers introduced.

## Tickets

| # | Title | Depends On |
|---|-------|------------|

Tickets execute serially in the order listed.
