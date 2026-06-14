---
id: '036'
title: Stateful Robot object and Playfield object (host module)
status: done
branch: sprint/036-stateful-robot-object-and-playfield-object-host-module
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
- SUC-008
- SUC-009
- SUC-010
- SUC-011
- SUC-012
issues:
- plan-stateful-robot-object-playfield-object-for-the-host-module.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 036: Stateful Robot object and Playfield object (host module)

## Goals

Give the host `robot_radio` module two first-class objects:

1. An evolved `Nezha` that owns its own state (queryable as `robot.state`),
   supports callback-driven G/turn for tick-level feedback, VW and S generators
   for open-ended streaming, a one-shot `refresh()` for idle queries, and a
   `update_world_pose()` convenience for camera-fix injection.
2. A new `Playfield` class wrapping the AprilCam daemon that provides live tag
   lookup, static map-feature lookup, pixel-world coordinate conversion (with the
   correct A1-centred y-up convention), and persistent path/symbol drawing.

Both classes are delivered with unit tests and the open playfield-tour demo is
ported to use them end-to-end.

## Problem

The host module splits robot control across loosely coupled pieces (`Nezha`,
`NezhaState`, `NezhaKinematic`, ad-hoc bench scripts) with no single queryable
robot state, no uniform callback model for drive commands, and no first-class
camera/world-geometry object. The `playfield_random_tour.py` demo bypasses all
module abstractions and talks to the serial relay and AprilCam daemon directly.

## Solution

Four locked decisions (from `.clasi/issues/plan-stateful-robot-object-playfield-object-for-the-host-module.md`):

1. Evolve `Nezha` in place — no new facade. Add the state model, callback/generator
   drive methods, `refresh()`, and `update_world_pose()` directly, keeping all
   existing signatures backwards-compatible (`on_tick=None` preserves today's
   blocking behaviour).
2. `Playfield` wraps `DaemonControl` (not the high-level `aprilcam.Playfield`).
   Live tags from `get_tags()`; static features from `playfield.json`/`where_is()`;
   pixel-world via the daemon homography with the display.py-verified transform.
3. State updates during commands only. Drive-loop ticks update `robot.state`; idle
   queries call `refresh()` (one-shot SNAP). No background telemetry thread.
4. Scope boundary: two classes + unit tests + port the demo. `navigator.py`,
   `odometry.py`, and all other bench scripts are untouched.

## Success Criteria

- `uv run --with pytest python -m pytest host/tests/ -q` — all new and existing
  host tests green (regression guard for Navigator/CLI paths).
- Existing wire-layer tests pass: `test_protocol_v2.py`, `test_nezha_drive.py`.
- Bench smoke (robot on stand, safe to drive; `VER` confirms live firmware):
  ported demo runs, camera + odometry tracks draw on the live view, robot reaches
  each rectangle, and a bounds-excursion aborts cleanly (callback returns False).

## Scope

### In Scope

- `host/robot_radio/robot/nezha.py` — state object, `go_to` callback form, new
  `turn`, `vw` generator, `refresh`, `update_world_pose`, back-compat properties.
- `host/robot_radio/robot/robot_state.py` — extend/confirm state dataclass.
- `host/robot_radio/robot/robot.py` — keep ABC consistent with safe defaults.
- `host/robot_radio/field/__init__.py`, `host/robot_radio/field/playfield.py` —
  new `Playfield` subpackage with `Tag`, `Feature` dataclasses.
- `host/robot_radio/__init__.py` / `robot/__init__.py` — export new public types.
- `host/tests/test_robot_state.py`, `test_robot_go_to_callback.py`,
  `test_robot_vw_generator.py`, `test_playfield.py` — four new test files.
- `host_tests/playfield_tour/playfield_random_tour.py` — rewrite to use `Nezha`
  and `Playfield`.

### Out of Scope

- `host/robot_radio/nav/navigator.py`, `odometry.py` — untouched.
- All other bench scripts and host_tests except the one demo.
- Firmware changes of any kind.
- Live colored-object detection via `ObjectRecord`/`SquareDetector` (later add-on).
- Background telemetry thread.

## Test Strategy

Unit tests drive `Nezha` via `SimConnection` (the in-process firmware sim at
`host/robot_radio/io/sim_conn.py`) — no hardware required. `Playfield` tests mock
`DaemonControl` and load a fixture `playfield.json`.

Bench acceptance gate: robot on stand, `VER` confirms firmware, then run the ported
demo end-to-end. Per project memory, bench verification is the standing acceptance
gate for any sprint touching host motion.

Run: `uv run --with pytest python -m pytest host/tests/ -q`
(bare `uv run pytest` fails on missing serial — do not use).

## Architecture Notes

- `_apply_tlm` stays the single state-writing hook; the new `state` field is
  populated there.
- `on_tick=None` branch in `go_to`/`turn` is the old blocking path — zero diff
  to callers that don't pass a callback.
- `Playfield.world_to_pixel` / `pixel_to_world` replicate `display.py:411-447`
  exactly (A1-centred, y-up, `raw = [x+origin_x, origin_y-y, 1]`).
- `vw()` generator models `protocol.stream_drive` — VW re-send is its own keepalive
  within the firmware watchdog window. `GeneratorExit` → STOP + STREAM 0.
- New subpackage `host/robot_radio/field/` is namespaced to avoid colliding with
  `aprilcam.Playfield`.

## GitHub Issues

(None yet.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Extend RobotState and wire back-compat Nezha properties | — |
| 002 | Add refresh() and update_world_pose() to Nezha; wire _apply_tlm to state | 001 |
| 003 | Callback-driven go_to and turn with _run_until_done loop | 001, 002 |
| 004 | Add vw() body-velocity generator to Nezha | 001 |
| 005 | New Playfield subpackage: tags, objects, pixel-world, paths | — |
| 006 | Port playfield_random_tour.py demo to Nezha and Playfield | 001, 002, 003, 005 |
| 007 | Fix SerialConnection relay handshake: HELLO classify + !GO data-plane | — |
| 008 | Fix SerialConnection reader routing: ID reply and SNAP/TLM reply | — |

Tickets execute serially in the order listed. T004 and T005 may be developed
in parallel with T002/T003 since they have no shared dependencies, but all must
complete before T006. T007 and T008 are independent comms-layer bug fixes
surfaced during bench validation; they have no dependency on the state-model
tickets.
