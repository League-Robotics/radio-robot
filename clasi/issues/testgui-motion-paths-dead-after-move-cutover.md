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

## Resolution — tours revived (2026-07-22, follow-up OOP fix)

The prior resolution above left `planner.tour` (and the Tour 1/Tour 2
buttons) dormant on purpose (module body raised `AttributeError` at import,
referencing deleted `telemetry_pb2.ACK_STATUS_DONE`/the depth-3 `AckEntry`
ring). Stakeholder clicked a Tour button and got the dormant-tour log line;
this pass ports `planner/tour.py` (and its minimal call path) onto the same
protocol v4 wire dialect the single-leg fix above already speaks.

- **`src/host/robot_radio/planner/tour.py`** — `MoveTransport.move()`'s
  kwargs updated to the current `Move` schema (mirrors `SimLoop.move()`
  exactly: `v_x`/`v_y`/`omega` or `v_left`/`v_right`, `stop_time`/
  `stop_distance`/`stop_angle`, required `timeout`, `replace`, `id`). The
  module-level `_STATUS_DONE`/deleted-enum crash is gone. `_move_kwargs_for_leg()`
  rebuilt: a "distance" leg → `MoveTwist(v_x)` + `stop_distance`; a "turn"
  leg → `MoveTwist(omega)` + `stop_angle`, `omega` sourced from
  `PlannerParams.omega_max` (2.0 rad/s default, matching
  `testgui/transport.py`'s own `_UNMANAGED_YAW_RATE` — duplicated, not
  imported, per the `[Presentation] -> [Domain]` dependency direction this
  module's own header already established). `Move.timeout` sized
  `expected_duration × 3`, floored at 2000ms (same constants/rationale as
  the single-leg fix's own `_move_timeout_for()`). Each leg gets a distinct
  `Move.id` from a fixed offset (`_TOUR_MOVE_ID_BASE = 1 << 20`) far above
  any session-scoped envelope `corr_id` counter, so a leg's own COMPLETION
  ack (echoing `Move.id`) can never be confused with a DIFFERENT command's
  ENQUEUE ack (echoing the auto-assigned envelope `corr_id`) landing in the
  wire's single ack slot — this distinction (enqueue vs. completion ack use
  different correlation keys, `docs/protocol-v4.md` section 7.2) is the
  actual defect the old ack-ring code never had to think about.
  `_drain_and_poll()`/`_wait_for_move_terminal()` poll for a frame whose
  `ack` (`AckEntry`, valid iff `ack_fresh`) matches a leg's own `Move.id`;
  `_outcome_for_terminal_frame()` reads that frame's `fault_move_timeout`
  flag (bit 15) to distinguish a stop-condition completion from a timeout
  ending, since the completion ack's own `ack_err` is unconditionally 0
  either way (section 7.3, AS-BUILT). Chaining unchanged in shape: the
  first leg sends `replace=True`, every later leg `replace=False` while the
  one before it is still active (real `MoveQueue` enqueue behind the active
  command, not a preempt) — same one-leg lookahead the pre-cutover code had.
- **`src/host/robot_radio/robot/protocol.py`** — `NezhaProtocol.move()`
  added, mirroring `SimLoop.move()`'s kwargs exactly (a thin dispatcher
  onto the pre-existing `move_twist()`/`move_wheels()`). Needed because
  `planner.tour`'s `MoveTransport.move()` protocol is called on whatever
  `.protocol` a transport exposes — `SimTransport.protocol` is a `SimLoop`
  (already had `.move()`), but `_HardwareTransport.protocol` is a
  `NezhaProtocol`, which had no generic `.move()` at all before this fix —
  so hardware tours could not have worked even with `tour.py` itself fixed.
- **`src/host/robot_radio/testgui/__main__.py`** — `_TourRunner.run()`'s
  guarded `planner.tour` import comment updated to reflect that the import
  now succeeds; the guard itself is left in place (future-proofing against
  a later breakage), and the happy path (progress narration + closure
  summary via `log_line`, already wired to the GUI log pane through
  `_WorkerBridge`) resumes automatically — no functional change needed
  there.
- **`src/host/robot_radio/testgui/commands.py`** — stale comment on the
  `TOUR_1`/`TOUR_2` import guard updated (the guard itself, and its
  empty-list fallback, is kept as defense-in-depth, but now resolves to the
  real 13/15-leg geometry instead of tripping the fallback).

**Tests revived** (previously skip-marked for the same `AttributeError`):
`src/tests/testgui/test_commands.py::TestTours` (6 tests, pure data, no
Qt/sim), `src/tests/testgui/test_sim_transport_tour1.py`'s two skipped
tests (direct-twist tour-shaped drive, and the full
`run_tour(TOUR_1)`-against-the-real-compiled-sim completion test — the
"Tour 1 completes in Sim" gate), and
`src/tests/testgui/test_tour1_geometry.py` (4 GUI/`QThread`-level tests,
un-skipped whole-file; `_FakeTwistTransport.move()`/`_make_frame()` ported
to the current `MoveTransport`/`TLMFrame`/`AckEntry` shapes — one
synthesized frame per queued completion, never discarded until matched,
mirroring the depth-3-ring-era fake's own "list only grows" semantics so
the one-leg lookahead's SECOND `move()` call — sent before the first leg's
completion is even polled for — doesn't have its own ack silently
discarded as a side effect of the first leg's earlier poll). Full suite:
1287 passed, 1 skipped, 9 xfailed, 2 xpassed (both pre-existing, unrelated,
non-strict flakes — `test_two_compatible_distance_legs_..._at_tour_level`
is a documented reorder-experiment-coupled flake, see
`cycle-order-reorder-experiment-ab-before-hardware.md`;
`test_otos_fused_traces_still_flat_pending_098` is a pre-existing
sprint-098-deferred OTOS-fusion gap), 0 failed.

**Headless Tour 1 in Sim** (`SimTransport`/`SimLoop`, the real compiled
firmware, `run_tour(TOUR_1)` end to end): all 13 legs `completed`, closure
position_delta ≈276mm / heading_delta ≈4.3° from start (0,0,0) to end
(276,10,12.6°) — well inside the ~500mm bench-observed range this tour
(not a tightly-closed loop by design) has always shown.

**Live hardware** (`/dev/cu.usbmodem2121102`, robot on the stand, via
`NezhaProtocol.move()`/`run_tour()`, not a GUI click): a 2-leg mini tour
(80mm straight + 30° turn) both legs `completed`; `stop()` issued and the
port cleanly disconnected afterward.

## Related

- angle-stop-overshoot-61-73-percent-on-hardware.md — GUI turns will show
  the same overshoot; expected until the trajectory-controller arc.
