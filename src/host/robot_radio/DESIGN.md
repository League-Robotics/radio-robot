# robot_radio (`src/host/robot_radio`)

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-21 · **Status:** in-flux

---

## 1. Purpose

`robot_radio` is the host-side Python package: everything that talks to
the robot (or a simulated one) from a laptop — transports, the wire
protocol adapter, per-robot config loading, calibration, sensor
decoding, the `rogo` CLI, an MCP server, and the PySide6 TestGUI. It is
the host half of the host/robot split described in
[`../../firm/DESIGN.md`](../../firm/DESIGN.md) §1: the firmware is a
pure velocity/deadman follower, and everything that used to plan
motion — a tour of legs, a path, a world-frame navigation goal — lived,
and still lives, on this side of the wire.

**This subsystem currently has two eras of code living in one tree, and
that is the single fact every reader needs before touching anything
here.** Sprint 115 (gut-to-minimal-firmware S1) deleted the firmware's
entire motion stack (`Motion::Executor`, `App::Pilot`,
`App::HeadingSource`) but made a deliberate, recorded decision (115's
Design Rationale, Decision 6) **not** to delete the host-side code built
against it — `planner/`, `path/`, `nav/`, and the TestGUI's tour/turn
modules stay in the tree, expected to go dormant/broken, with actual
deletion deferred to a separate future sprint. The result is not a clean
directory-level split; several nominally "live" directories (`robot/`,
`io/`, `sensors/`, `calibration/`) themselves contain a mix of
current, wire-compatible code and dead code still calling
firmware-side commands that no longer exist. §2 and §3 spell out exactly
which functions in which files are which — do not assume a whole
directory is safe to call into just because it is not `planner/`,
`path/`, or `nav/`.

## 2. Orientation

One row per top-level directory. **Live** = calls only
`move`/`stop`/`config` (the current `CommandEnvelope` oneof —
[`../../firm/DESIGN.md`](../../firm/DESIGN.md) §4; `move_twist()`/
`move_wheels()` are the host builders for the `move` arm, replacing the
116-001-deleted `twist()` — see §5's `Exposes` bullet) or has no firmware
wire dependency at all. **Dormant** = calls a firmware verb or message
that no longer exists (a pre-102 text verb, a pre-116 bare `twist`/
G-command/`SI`/`O*`-family binary or text verb) and would raise/return an
error against the real robot today. **Mixed** = the directory contains
both, file-by-file or even function-by-function.

| Directory | Status | Notes |
|---|---|---|
| `robot/` | **Mixed** | `protocol.py`'s `NezhaProtocol` (the actual wire adapter) is live and current — its own docstring states the firmware "has no text-plane command parser at all." 116-007 (MOVE protocol cutover): `move_twist()`/`move_wheels()` replace `twist()` (deleted — its wire arm, `Twist`, is `reserved`, not reused) as the live motion-command builders, alongside `stop()`/`config()`/`otos_config()`/`estimator_config()`/`set_config()`/`set_config_binary()` (117 ticket 003 added `estimator_config()`, `App::StateEstimator`'s own live fusion-weight-tuning surface — see §5's `Exposes` entry for the full detail). `connection.py` (port resolution, session cache) is live. `nezha.py`'s `Nezha` wrapper — what every entry point actually constructs — is mostly dead: only `stop()`/`set_config()`/`set_config_binary()` map onto surviving `NezhaProtocol` methods; `go_to()`, `turn()`, `speed*()`, `zero_encoders()`, `ping()`, `get_id()`, `get_config()`, `grip()`, `vw()`, `stream_tlm()`, `snap()`, OTOS scalar accessors, and `port_read/write*()` all call methods `NezhaProtocol` no longer has. `sync_pose.py` (builds a pruned `SI` text arm) and `_legacy_tlm_text.py` (an explicitly self-described frozen legacy parser) are dormant. `clock_sync.py`'s `ClockSync` class itself has no live caller wired up today (its own usage example and `testkit/safety.py`'s `SafeRun` preflight both drive it through the dead `Nezha.ping()` wrapper); 117 (SUC-056) made the firmware's `PING` reply carry `t=<ms>`, closing the WIRE half of `ClockSync`'s activation gap (proven at the sim level), but a live round trip through `NezhaProtocol.send()` hits a separate, pre-existing host-side gap — see §6. `cutebot.py` is a separate hardware family (a different robot, different protocol), unaffected by the gut either way. |
| `io/` | **Mixed** | `serial_conn.py` (transport), `repl.py`, `sim_config.py`, `sim_loop.py` are live and current — `repl.py`'s own docstring is explicitly post-gut-aware ("exactly three CommandEnvelope arms... every verb here maps onto one of those three"). `cli.py` (the `rogo` entry point) is split: `repl`/`stop`/`binary stop` subcommands are live; its legacy subcommands (`drive`, `turn`, `turnto`, `go`, `goto`, `rot`, `ang`, `port`, `pwm`, `grip`, `enc`, `opos`, `ez`, `line`, `color`, `pose`) route through the dead half of `nezha.py` and are broken today. `calibrate.py` drives raw text commands (`TN...`, `OA...`) — dormant. `robot_mcp.py` registers ~30 MCP tools; the majority (`go`, `goto`, `navigate_to`, `visit_tags`, `approach`, `follow_path`, `grab_at`, `release_at`, `plan_path`, `preview_path`, `otos_*`, `read_pose_fused`, `tune`, `reload_nav`, `reset_camera`) are built on the dormant `nav`/`path`/`sensors.otos` machinery; `connect`/`disconnect`/`status`/`stop`/`list_serial_ports`/`probe_devices` and the pure-camera tools are live — but `connect` unconditionally calls the dead `push_calibration()` text path on every connect (see `calibration/` row). `preview.py` is an explicit stub (unimplemented, not dormant). |
| `config/` | **Live** | `robot_config.py` — pydantic loader for `data/robots/*.json`. Pure Python, no wire dependency, no imports from elsewhere in this package. |
| `calibration/` | **Mixed** | `helpers.py` (pure scale-encoding math) is live as a library. `angular.py`/`linear.py` (extracted from the now-archived `host_scripts/calibrate_*.py`) use a raw-pyserial text handshake — dormant. `push.py`'s `push_calibration()` default code path sends literal `SET`/`OI`/`OL`/`OA` text lines — dead — but the SAME calibration data has a second, live route: its `calibration_kwargs()` helper feeds `NezhaProtocol.set_config()`/`otos_config()` (binary) directly, used by `io/sim_loop.py` and `testgui/binary_bridge.py`. `sim_boot_config.py`'s own docstring confirms 115-003 deleted its `PlannerConfig` half; `motor_boot_config_for()` is "the sole survivor." `fit_sim_error_model.py` is sim-only bench tooling reading the legacy text TLM parser through an explicitly-flagged exception path. |
| `sensors/` | **Mixed** | `odom_tracker.py` (v2/binary `update_from_tlm()` primary path — live; a deprecated `parse_so()` fallback explicitly marked "NOT on the v2 hot path"), `color.py` (reads the current binary frame's packed `color` field — live), `cam_tracker.py` and `motion_monitor.py` (pure camera/pose-stream analysis, no wire dependency — live as libraries). `otos.py` — despite the name suggesting it is THE live OTOS surface — is entirely dormant: every method sends one of the pruned text verbs (`O`, `OI`, `OZ`, `OR`, `OC`, `OP`, `OV`, `OL`, `OA`). The actual live OTOS surface is `NezhaProtocol.otos_config()` in `robot/protocol.py`, a completely different, binary `CONFIG{otos}` code path this module does not use. `calibration.py`'s `apply`/`load_and_apply` (wire-push) are dormant (its own docstring: the `GET`/`CFG` config read-back arm is "permanently reserved," no live read-back path exists); `load()` (pure JSON) is fine. |
| `controllers/` | **Live, but unused** | `base.py` (ABC), `pid.py` (pure-math `PID` class). Every real consumer (`planner/heading.py`, `planner/tour.py`, `nav/navigator.py`) is dormant; `io/robot_mcp.py` only references the module for `importlib.reload()` bookkeeping. Inert but not broken — reusable if a live consumer ever needs it again. |
| `kinematics/` | **Live, but unused/orphaned** | `differential_drive.py` (wraps `wpimath`, pure math). Its only importer, `robot/nezha_kinematic.py`, is itself unreferenced by anything except `robot/__init__.py`'s lazy re-export and `README.md` — no live code path actually calls it. It structurally duplicates `src/firm/kinematics/body_kinematics.h`'s `inverse()`/`forward()` equations; a host-side twin of the firmware math with no current caller. |
| `field/` | **Live** | `playfield.py` — AprilCam/AprilTag playfield/geofence model (`aprilcam.client.control.DaemonControl`). No firmware wire dependency at all; orthogonal to the gut entirely. |
| `media/` | **Live** | `movie.py` — camera/video capture tooling. No firmware wire dependency. |
| `planner/` | **Mixed — `tour.py` LIVE, the rest still dormant** | `tour.py`'s `TOUR_1`/`TOUR_2`/`parse_tour()`/`run_tour()` were ported (2026-07-22, `testgui-motion-paths-dead-after-move-cutover.md`) onto protocol v4's `Move`/single-ack-slot wire shape — the pre-port `AttributeError`-at-import state this row used to describe (a reference to the deleted `telemetry_pb2.ACK_STATUS_DONE`) is fixed. Verified empirically: `from robot_radio.planner.tour import TOUR_1, TOUR_2` imports cleanly (re-checked 2026-07-23), and `run_tour()` is called directly by both `src/tests/testgui/test_tour_closure_gate.py`'s gate tests and `test_gui_button_acceptance.py`'s managed-motion tests. `executor.py`'s `StreamingExecutor` and `heading.py`/`model.py`/`profile.py` remain genuinely dormant — `tour.py` itself no longer routes through them (one `Move` per leg via `MoveTransport.move()`, not `StreamingExecutor`/`profile.py`'s setpoint-sequence path); nothing in the shipped app constructs a `StreamingExecutor` outside `src/tests/bench/`'s own scripts. |
| `path/` | **Dormant (orphaned)** | Pure geometry (`arc.py`/`bezier.py`/`builder.py`/`catmull_rom.py`/`obstacle.py`/`patterns.py`/`sampled_path.py`) with no firmware wire dependency of its own — not itself broken, but its only real callers are the dormant `io/robot_mcp.py` MCP tools (`navigate_to`/`follow_path`/`plan_path`/`preview_path`). |
| `nav/` | **Dormant, by stakeholder decision** | `navigator.py`'s own docstring: "Navigator is a route planner: it sequences firmware G commands... the sole steering loop" — `navigate()`/`follow_path()` call `Robot.go_to()`, which is one of `nezha.py`'s dead methods. `camera_goto.py` feeds `cli.py`'s already-dormant `goto`/`rot`/`ang` subcommands. `nav_params.py` tunes a Stanley controller `navigator.py` itself says predates the gut (already deleted pre-gut in favor of the now-also-dead G-command path). `pose_align.py` calls the dormant `Otos.align_to()`. The one exception: `pose.py` (`Pose`/`Waypoint` frozen dataclasses, zero logic) is a plain coordinate type reused by several genuinely live modules (`robot/nezha_state.py`, `robot/robot_state.py`, `sensors/odometry.py`, `sensors/otos.py`) — importing it is not itself a sign of dormancy. |
| `testgui/` | **Mixed** | Live: `drive.py`, `canvas.py`, `live_view.py`, `telemetry_panel.py`, `recorder.py`, `traces.py`, `camera_prefs.py`, `sim_prefs.py`, `binary_bridge.py` (a translation shim converting legacy text verbs to binary `CommandEnvelope` calls — see §3), and `__main__.py`'s "Unmanaged" direct-twist/stop controls (explicitly labeled "direct twist... no planner, no heading loop" in its own comments). Dormant, by file name: `commands.py` (37 lines of first-party dormancy commentary, quoting sprint 115's own Decision 6), `transport.py` (imports `planner.executor.TwistTransport`/`planner.tour.run_tour`/`parse_tour`), `__main__.py`'s tour-button block, `turn_control.py` (a TCP socket sending a pruned `"SEG pivot"` text command), `turn_graphs.py` (visualization for tour/turn-driven telemetry — passive, no live driver), and `turn_shape.py` (its capture functions call `SimLoop.move()`, which builds a `CommandEnvelope{move: Move{...}}` — `envelope_pb2.Move` does not exist in the current schema, confirmed empirically). |
| `testkit/` | **Mixed** | `camera.py` (tag-averaging) and `dash.py` (generic dashboard) are wire-independent and live. `pose.py`'s `FirmwarePose` reads `SNAP` telemetry — one of the arms pruned by 104-002 — dormant; its `CameraPose` (aprilcam-based) is fine. `safety.py`'s `SafeRun` preflight sends `PING` (dormant — maps to the dead `Nezha.ping()`); its `stop()`-on-exit guarantee still works (`stop()` is live). `target.py`'s "sim" branch explicitly raises `NotImplementedError`; "bench"/"production" branches inherit the OTOS/SNAP dormancy above. |

## 3. Constraints and Invariants

- **Dormancy here is a recorded, deliberate stakeholder decision, not an
  oversight — do not "clean up" by deleting dormant code without a
  separate, explicit decision to do so.** Sprint 115's Design Rationale
  (Decision 6) chose to let `planner/`/`path/`/`nav/` and the TestGUI
  tour/turn modules go dormant in place rather than delete them, because
  sprint 116's MOVE protocol is expected to revive most of this
  machinery against a new wire shape. Treat every file this document
  marks dormant as "parked, not gone."
- **A directory being "live" does not mean every function in it is
  callable.** `robot/nezha.py`, `sensors/otos.py`, `calibration/push.py`,
  and `io/robot_mcp.py` are the sharpest traps: each sits in a
  nominally-live directory but its *default* or *most obviously named*
  entry point calls a firmware verb that no longer exists. Before adding
  a new caller of anything in this package, check §2's per-file notes,
  not just which directory the file lives in.
- **`testgui/binary_bridge.py` is the one sanctioned text→binary
  translation shim** — its own docstring: "Firmware is binary-only plus
  a 6-verb text rump (HELP/HELLO/PING/ID/VER/STOP)... every motion/
  config/telemetry text verb... gets ERR unknown if sent as literal
  text." Any NEW code that still thinks in terms of a text verb
  (`SET`/`OI`/`OL`/`TN...`) must go through this shim's translation
  layer or `NezhaProtocol`'s binary methods directly — never emit a bare
  text line and expect the firmware to answer it. This is the same
  "legacy text clients go through a host-side translator proxy, never
  through firmware text parsing" stakeholder decision recorded in
  [`../../firm/DESIGN.md`](../../firm/DESIGN.md) §4.
- **`io/robot_mcp.py`'s `connect` tool pushes calibration via the dead
  text path on every connection, silently.** `push_calibration()`'s
  default branch (no `push_calibration` method on `NezhaProtocol`) falls
  through to raw `SET`/`OI`/`OL`/`OA` text lines the firmware cannot
  parse — this does not raise, it just has no effect. Anyone debugging
  "why didn't my calibration take" via the MCP `connect` tool should
  check this path before suspecting the robot. The LIVE calibration-push
  route is `calibration/push.py`'s `calibration_kwargs()` feeding
  `NezhaProtocol.set_config()`/`otos_config()` directly (what
  `io/sim_loop.py` and `testgui/binary_bridge.py` actually use).
- **`nav/pose.py`'s `Pose`/`Waypoint` are plain data types, not
  motion-stack code** — several genuinely live modules import them for
  the coordinate type alone. Do not treat "imports something from
  `nav/`" as itself proof a module is dormant; check what specifically
  is imported.

## 4. Design

**Why the split by "where it runs" mirrors `src/tests/`'s domain split.**
This package's live surface is deliberately narrow and close to the
wire (`protocol.py`, `serial_conn.py`, `repl.py`, `sim_loop.py`,
`robot_config.py`) — the same minimalism sprint 115 imposed on the
firmware side. Everything built for a richer motion abstraction
(multi-leg tours, splines, closed-loop world-frame navigation) is one
layer up and, for now, inert. This is not a redesign of this package;
it is the host-side shadow of the firmware gut, recorded here rather
than rediscovered by whoever next tries to call `Nezha.go_to()`.

**Why `NezhaProtocol` (in `robot/`) rather than a `protocol.py` under
`io/`.** The wire adapter lives beside the `Robot`/`Nezha` class
hierarchy it backs, not beside the transport (`io/serial_conn.py`) it is
built on — `protocol.py` is a protocol-*level* concept (what a
`CommandEnvelope`/`ReplyEnvelope` round trip looks like), while `io/`
owns byte-level transport concerns (serial framing, the sim ctypes
bridge). `Robot`/`Nezha` are the callers that turn "twist" into
`NezhaProtocol.twist()`.

**Two calibration-push routes are a known duplication, not a bug to
merge blindly.** `push.py`'s dead text route and `calibration_kwargs()`'s
live binary route both exist because nothing has yet gone back and
removed the dead one — merging them requires confirming every caller of
the dead route (chiefly `io/robot_mcp.py`'s `connect`) is updated to the
live one first, not just deleting the dead function.

**`planner/tour.py`'s three-generation history.** Sprint 107 ticket 002
(SUC-033) moved `TOUR_1`/`TOUR_2` out of `testgui/commands.py`, where they
had lived as raw firmware wire strings (`D`/`RT`) for the since-deleted
`Motion::SegmentExecutor` — the GEOMETRY those strings encode is a real,
tuned asset, so this module owns the data and parses it into typed legs
(keeping the `[Presentation] -> [Domain]` dependency direction,
`architecture-update.md` Decision 3). 109-008 moved `run_tour()` off a
host-computed `profile.py` setpoint sequence streamed through
`planner.executor.StreamingExecutor` and onto ONE `Move` command per leg
(`transport.move()`), relying on firmware's own bounded-queue + boundary-
velocity carry (one-leg lookahead, SUC-003) to sequence legs and on each
`Move`'s own completion EVENT (not a host-timed settle delay or polled
`fault_bits`) to end a leg — closing `tour1-freeze-investigation-
2026-07-15.md`, where the old streaming path's raw `fault_bits` poll froze
a whole tour on a transient, firmware-self-recovered blip. That "one Move
per leg, one-leg lookahead, event-driven completion" SHAPE survived
unchanged through the 2026-07-22 port (`testgui-motion-paths-dead-after-
move-cutover.md`) that brought the module back to life onto protocol v4's
`Move` shape (see the `planner/` row above) — only the WIRE mechanics
changed: `MoveTransport.move()`'s kwargs became the current `Move` schema
(`v_x`/`omega`/`stop_distance`/`stop_angle`/`timeout`/`replace`/`id`, not
the deleted sprint-109 arc shape), and the old `AckStatus` taxonomy
(`DONE`/`TRIVIAL`/`SUPERSEDED`/`FLUSHED`/`TIMEOUT`/`SOLVE_FAIL`) plus
depth-3 ack ring were replaced by `Telemetry`'s single ack slot
(`ack_corr`/`ack_err`) — since 120, additionally backed by a bounded
ack RING (`acks`, depth 4; see §5's own `wait_for_ack()` note below) —
which carries EITHER a command's enqueue ack OR a `Move`'s own completion
ack (`docs/protocol-v4.md` §7.2) — `tour.py`'s own
`_drain_and_poll()`/`_outcome_for_terminal_frame()` poll and read that
slot keyed on `Move.id`, never the enqueue envelope's `corr_id`.
`planner.executor.StreamingExecutor`/`planner.profile`
themselves were untouched by either move — only TOURS (this module)
changed path; they remain the dormant half of `planner/` (see the
`planner/` row above).

## 5. Interfaces

### Exposes

- **`robot.protocol.NezhaProtocol`** — the live wire adapter:
  `move_twist()`/`move_wheels()`/`stop()`/`config()`/`otos_config()`/
  `estimator_config()`/`set_config()`/`set_config_binary()` (116-007:
  `move_twist()`/`move_wheels()` replace the deleted `twist()` as the live
  motion-command builders — the bounded `Move` arm's twist/wheels velocity
  variants),
  plus `send()`/`send_fast()`/`read_lines()`/
  `wait_for_ack()`/`read_binary_tlm_frames()`/
  `read_pending_binary_tlm_frames()`. This is the authoritative host-side
  surface for the current firmware's `CommandEnvelope`/`ReplyEnvelope`
  round trip — see [`../../firm/messages/DESIGN.md`](../../firm/messages/DESIGN.md)
  for the wire shape it encodes/decodes.

  **`wait_for_ack()` is ring-aware since 120
  (bench-single-ack-slot-observability-collapses-at-40ms.md).**
  `NezhaProtocol.wait_for_ack(corr_id, timeout)` delegates to
  `SerialConnection.wait_for_ack()` (`io/serial_conn.py`), which polls
  `drain_binary_tlm()` and scans each drained `ReplyEnvelope{tlm:
  Telemetry}` frame's bounded `acks` ring (`telemetry.proto`, depth 4) —
  not the single scalar `ack_corr`/`ack_err`/`flags`-bit-5 slot the pre-120
  implementation scanned — via the module-private
  `_match_ack_in_frames()`. Matching policy (the one genuinely open
  question this sprint's Architecture Step 7 left to this ticket):
  returns on the FIRST `(frame, ring-entry)` pair whose `corr_id` matches,
  scanning frames in arrival order and, within one frame, ring entries in
  wire order (oldest-pushed first) — chosen because a match is an exact
  `corr_id` equality check, not a "freshest wins" precedence judgment, so
  entry order only matters in the (unexpected, not observed in practice)
  case of the same `corr_id` appearing twice. No freshness bit gates a
  ring match — unlike the scalar pair (whose value persists, stale or
  not, until the next `ack()` call, needing `ack_fresh` to disambiguate),
  a `corr_id` present in the ring was genuinely pushed by
  `App::Telemetry::ack()` at some point; there is nothing to
  disambiguate. Returns the matched raw `telemetry_pb2.AckEntry` ring
  entry (`SerialConnection.wait_for_ack()`) or this module's own
  `AckEntry` dataclass via `AckEntry.from_ring_entry()`
  (`NezhaProtocol.wait_for_ack()`) — reading the enclosing FRAME's own
  scalar `ack_corr`/`ack_err` instead would be wrong whenever a different,
  later command's ack has since become that frame's own "freshest ack".
  `TLMFrame.acks` (`robot/protocol.py`) exposes the full decoded ring,
  always populated (independent of `ack_fresh`), for a caller that wants
  to inspect it directly rather than go through `wait_for_ack()`'s own
  single-`corr_id` search — see `src/tests/bench/
  ack_ring_rapid_fire_bench.py` for the hardware-verified N=5 rapid-fire
  proof this ticket's own acceptance criteria required. The scalar
  `ack_corr`/`ack_err`/`flags` bit 5 and `TLMFrame.ack`/`ack_fresh` keep
  their exact pre-120 meaning, read by `AckEntry.from_telemetry()`
  (unchanged, still used by `TLMFrame.from_pb2()`) — this is an additive
  capability, not a replacement.

  **`estimator_config()`** (117
  ticket 003) is a new live-tuning surface, mirroring `otos_config()`'s own
  "one envelope, one patch, fire-and-poll" builder shape exactly
  (`weight_heading_otos`/`weight_omega_otos`/`staleness_ms` →
  `CommandEnvelope{config: ConfigDelta{estimator: EstimatorConfigPatch{...}}}`)
  — but UNLIKE `otos_config()`, its patch is never persisted on the robot
  side (`RobotLoop::handleConfig`'s `ESTIMATOR` branch applies it live,
  never writes `persistedTuning_`/flash — see
  [`../../firm/config/DESIGN.md`](../../firm/config/DESIGN.md) §3).
- **`robot.connection.make_robot()`** — port resolution + session cache,
  the shared construction path `io/cli.py` and `io/robot_mcp.py` both use.
- **`config.robot_config.get_robot_config()`** — resolves
  `data/robots/active_robot.json`/`ROBOT_CONFIG` to a validated pydantic
  model; see [`../../firm/config/DESIGN.md`](../../firm/config/DESIGN.md)
  §5 for the firmware-side consumer of the same JSON files.
- **`io.sim_loop.SimLoop`** — loads `src/sim/`'s dylib, drives it via
  `twist()`/`stop()`; see [`../../sim/DESIGN.md`](../../sim/DESIGN.md).
  **`configure_from_robot()`** (113, extended 119 ticket 001) is a
  three-tier push over one shared `SimConfigConn`: Tier 1
  (`calibration_kwargs()` → `NezhaProtocol.set_config()`, the live
  `SET`-key-equivalent binary plane), Tier 2 (`motor_boot_config_for()` →
  `sim_configure_motor()`, the boot-only motor fields with no live wire
  arm), and Tier 3 (`estimator_kwargs()` →
  `NezhaProtocol.estimator_config()`, the `EstimatorConfigPatch` fusion
  weights + `Motion::VelocityShaper` accel/jerk ceilings). Tier 3 closes
  `kill-the-silent-off-shaping-config-boundary.md`: every
  `configure_from_robot()` caller (TestGUI, bench scripts, tests) now
  inherits shaping/anticipation by default instead of running silently OFF
  until a caller separately remembered to push it — the TestGUI's own
  connect-time `_push_estimator_config()` push (`testgui/__main__.py`)
  becomes redundant-but-harmless (idempotent acks) once this landed.
- **`rogo` console script** (`io/cli.py:main`) — the live subset is
  `repl`, `stop`, `binary stop`; see §2 for which subcommands are
  currently broken.
- **`robot_mcp` MCP server** (`io/robot_mcp.py`) — the live subset is
  `connect`/`disconnect`/`status`/`stop`/`list_serial_ports`/
  `probe_devices` plus the pure-camera tools; see §2 for which tools
  call into dormant machinery.
- **TestGUI** (`testgui/__main__.py`, `just testgui`) — the live subset
  is direct twist/stop ("Unmanaged"), telemetry display, camera
  preferences, and sim connect; the tour/turn buttons are dormant (§2).

### Consumes

- **`src/firm/messages/` (via `robot/pb2/`, generated by
  `scripts/gen_pb2.py`)** — the compiled Python protobuf bindings for
  the wire schema; see [`../../protos/DESIGN.md`](../../protos/DESIGN.md).
- **`src/sim/`** — the simulator dylib, via `io/sim_loop.py`; see
  [`../../sim/DESIGN.md`](../../sim/DESIGN.md).
- **`data/robots/*.json`** — per-robot calibration, via
  `config/robot_config.py`.
- **AprilCam** (`aprilcam.client.control.DaemonControl`) — `field/`,
  `media/`, and camera-dependent `testkit/`/`testgui/` modules.

## 6. Open Questions / Known Limitations

- **117 (SUC-056): `PING`'s reply now carries `t=<ms>`, closing the wire
  side of `ClockSync`'s activation gap — but the host's own
  `SerialConnection.send()` has a SEPARATE, pre-existing gap that still
  blocks a live round trip through it.** The firmware's text-plane
  `PING` handler (`Comms::pumpTransport()`, `src/firm/app/comms.cpp`)
  now replies `OK pong t=<ms>` — the robot's own clock at reply time —
  closing `docs/protocol-v4.md` §2.4's former AS-BUILT divergence.
  `robot/clock_sync.py`'s `ClockSync.ping_burst(send_fn)` already
  tolerated and parsed this exact shape (`_parse_pong_t()`). It is
  proven to activate against the firmware's actual (compiled, not
  hand-typed) reply format at the sim/unit level
  (`src/tests/sim/unit/test_clock_sync_activation.py`, 117 ticket 001).
  **Found while verifying this, flagged rather than silently worked
  around:** `io/serial_conn.py`'s `SerialConnection.send()` appends a
  `" #<corr_id>"` suffix to EVERY command it sends (`corr_suffix =
  f" #{corr_id}"`, `cmd = f"{message}{corr_suffix}\n"`) — so
  `NezhaProtocol.send("PING")` actually puts `"PING #7"` on the wire, not
  `"PING"`. `Comms::pumpTransport()`'s text-plane check is an EXACT
  `std::strcmp(line, "PING")` (no trimming beyond `SerialPort::
  readLine()`'s own trailing-newline strip) — a corr-id-suffixed line
  does not match, falls through to the `*B`-armor check, fails that too,
  and increments `malformedCount_` with **no reply at all** (not even a
  bare `OK pong`). This is not new or caused by 117 — `send()`'s own
  docstring already warned "a text line sent through it reaches no live
  firmware handler," and grepping the tree turns up no existing caller
  of `NezhaProtocol.send("PING", ...)`/`send("HELLO", ...)` against a
  real connection today. A live/bench `ClockSync.ping_burst()` round
  trip therefore needs a corr-id-suffix-free send path (e.g. a small
  `send_fast()` + `read_lines()` pairing, or a `SerialConnection.send()`
  fix) before it can work off the sim harness — flagged here as a real,
  separate gap, not fixed as part of this ticket (out of its scope: the
  firmware-side wire contract and the sim-level proof are what SUC-056
  asks for).
- **Sprint 116's MOVE protocol is the expected path back to life for
  most of `planner/`/`path/`/`nav/`** — but it has not been executed
  yet (as of this review). Until it lands, treat every dormant entry in
  §2 as broken today, not "probably fine."
- **`robot/nezha.py`'s module docstring documents the deleted v2 text
  command set** (`S`/`T`/`D`/`G`/`TURN`/`GRIP`/`GET`/`SET`) and has not
  been updated to reflect which of its own methods still work — a
  reader trusting the docstring over the method bodies will be misled.
  Not fixed as part of this review (source code, out of this
  documentation task's scope).
- **Whether to delete the dead half of `robot/nezha.py`, `sensors/
  otos.py`'s text methods, and `calibration/push.py`'s default text
  route, versus rewriting them onto the live binary primitives, is an
  open call** — both are plausible follow-ups once sprint 116 clarifies
  what the next wire surface actually needs.
- **`kinematics/differential_drive.py` and `controllers/pid.py` have no
  live caller today** — worth revisiting whether they are still the
  right shape once a live consumer reappears (likely alongside sprint
  116), or whether they should be deleted as unused.
