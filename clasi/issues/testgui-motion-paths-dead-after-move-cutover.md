---
status: resolved
---

# TestGUI motion paths dead after the MOVE cutover (S managed + Turn on both transports; unmanaged dead on serial)

## Description

Post-115-117, TestGUI's motion buttons are broken: managed S-drive and Turn
do nothing on BOTH transports; unmanaged S works only in Sim (serial
unmanaged is a silent hasattr no-op). Stakeholder-reported 2026-07-22.

## Cause

- Serial managed path: `binary_bridge.translate_command()` is a permanent
  dead stub (legacy_verbs/render modules deleted long ago) — returns ERR,
  sends nothing (binary_bridge.py:86-102, 209-210).
- Sim managed path: `_run_motion_async` imports `planner.tour`, whose module
  body reads deleted `telemetry_pb2.ACK_STATUS_*` (tour.py:126) →
  AttributeError kills the worker silently; beyond that, `SimLoop.move()`
  still builds the deleted flat arc-Move fields (sim_loop.py:606-610).
- Serial unmanaged: `_HardwareTransport` has no `run_unmanaged`; the GUI's
  hasattr guard silently no-ops (__main__.py:767,772).

## Proposed fix (OOP, tiers A+B; tours stay dormant)

Port the single-leg button paths to the new protocol: `SimLoop.move()` →
new Move schema (distance leg = MoveTwist(v_x)+stop distance; turn leg =
MoveTwist(omega)+stop angle, host-picked yaw rate); `SimTransport`
D/RT/SEG dispatch → direct `self._loop.move(...)`, no planner.tour import;
`_HardwareTransport` gains `run_unmanaged` + a D/RT/SEG → move_twist
dispatch (or direct button wiring). Distinct move ids for completion acks.

## Resolution (2026-07-22, OOP fix)

Implemented tiers A+B exactly as proposed; tours (`planner.tour`) left
dormant.

- **`src/host/robot_radio/io/sim_loop.py`** — `SimLoop.move()` rebuilt
  against the current `Move` schema (`envelope.proto`): a velocity variant
  (`MoveTwist{v_x,v_y,omega}`, the default, or `MoveWheels{v_left,v_right}`
  when both are given) + exactly one stop condition
  (`stop_time`/`stop_distance`/`stop_angle`, via
  `protocol._build_move_stop_kwargs()`, reused not reimplemented) +
  required `timeout` (`ValueError` if not `>0`). Distinct incrementing
  `id`s via the existing `_next_corr_id()` counter when omitted.
- **`src/host/robot_radio/testgui/transport.py`** —
  `SimTransport._run_motion_async()` no longer imports `planner.tour`;
  parses `D`/`RT` itself (`_parse_d_step()`/`_parse_rt_step()`, module-
  level, shared with the hardware dispatch below) and calls
  `self._loop.move(...)` directly, worker-thread exceptions caught and
  logged. `_HardwareTransport` gains `run_unmanaged()` (calls
  `NezhaProtocol.move_twist()`) and `_dispatch_managed_move()` (`D`/`RT`/
  `SEG 0 <cdeg>` → `move_twist()`/`move_wheels()`, intercepted in
  `send()`/`command()` ahead of `binary_bridge.translate_command()`'s dead
  stub; `D`'s `left==right` → twist, `left!=right` → wheels). Move.timeout
  sized as `expected_duration × 3`, floored at 2000ms
  (`_MOVE_TIMEOUT_FACTOR`/`_MOVE_MIN_TIMEOUT`). `_UNMANAGED_SPEED`/
  `_UNMANAGED_YAW_RATE` hoisted to module level so both backends share one
  cruise-rate source.
- **`src/host/robot_radio/testgui/binary_bridge.py`** —
  `_LEGACY_UNAVAILABLE_REPLY` reworded to name protocol-v4
  (`docs/protocol-v4.md`) and note that D/RT/SEG are now handled upstream
  in `transport.py`, never reaching this stub.
- **`src/host/robot_radio/testgui/__main__.py`** — `_TourRunner.run()`'s
  `from robot_radio.planner.tour import parse_tour, run_tour` moved inside
  a guarded `try`/`except Exception` (was above the method's own `try`
  block, so the module's own `AttributeError` at import time propagated
  out of the worker thread uncaught — `finished()` never emitted, the tour
  button never re-enabled, nothing logged). Now logs one `[TOUR] ...
  planner.tour is dormant this sprint ...` line and returns cleanly; tour
  buttons are still non-functional (dormant, unchanged, per scope) but now
  fail **visibly**.

**Tests**: `src/tests/testgui/test_sim_loop.py` (+9, against the real
compiled sim: twist/wheels Move variants advance true pose/heading,
timeout/stop-condition/wheels validation, id assignment),
`src/tests/testgui/test_transport.py` (+6, against the real compiled sim:
`command("D 150 150 <mm>")`/`"RT <cdeg>"`/`"SEG 0 <cdeg>"` drive the plant,
in-flight rejection, malformed-line error logging), new
`src/tests/testgui/test_hardware_transport_managed_move.py` (20 tests, fake
connection double, no hardware/sim needed: envelope construction for
D-twist/D-wheels/RT/SEG, badarg/ack-timeout/nak paths, `run_unmanaged()`).
Full suite: 1275 passed, 13 skipped, 10 xfailed, 1 xpassed, 0 failed.

**Live hardware** (`/dev/cu.usbmodem2121102`, robot on the stand,
transport-level, not GUI-click): `command("D 150 150 300")` → `OK move`,
envelope `move{twist{v_x:150} distance:300 timeout:6000 replace:true}`,
encoders baseline (0,0) → settled (340,312) mm both wheels.
`command("RT 9000")` → `OK move`, envelope
`move{twist{omega:2.0} angle:1.5708 timeout:2356}`, heading landed at
99.46° after a commanded 90° turn (~10% over — well inside the known
overshoot band below, not chased). `run_unmanaged(distance_mm=200)` →
envelope `move{twist{v_x:150} distance:200 timeout:4000}`, encoders
(206,428)→(446,653) (~240mm/~225mm travelled). STOP issued after every
step; port cleanly released on disconnect (`lsof` confirmed clear
afterward).

## Related

- angle-stop-overshoot-61-73-percent-on-hardware.md — GUI turns will show
  the same overshoot; expected until the trajectory-controller arc.
