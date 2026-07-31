# Craftsmanship + Correctness Review — host-side path planning + sim

**Date:** 2026-07-30 · **Reviewer:** programmer-agent (review-only pass) ·
**Branch:** `sprint/127-host-side-path-planner-goto-and-path-following`
(WIP from a parallel session) · **Scope:** every `.py` under
`src/host/robot_radio/{nav,path,pathplan,planner,controllers,kinematics,
calibration}` (~7.6k lines) plus `src/sim` and the packages those import
across a live boundary (`field/`, `robot/protocol.py`, `io/sim_loop.py`).
Guidelines: `docs/code_review/GUIDELINES.md`.

## Verdict

**Merge-with-findings** for sprint 127's own WIP (`pathplan/world_pose.py`,
committed; `pathplan/solver.py`, uncommitted but complete and tested) —
both are genuinely close to the textbook bar: pure functions, explicit
units, wrap-safe residuals reused rather than reforked, and a curvature
slew limit whose derivation is fully shown and traced to a measured
number. But the review's primary question — "how many independent
`drive-to-a-point` stacks exist, and which is the owner?" — has a
sharper and worse answer than the 2026-06-11 review's "three, no owner":
**one of the three (`nav/`) is not actually live** — it is dead code
still wired into the CLI and the MCP tool surface as if it worked,
confirmed at runtime to raise `AttributeError` the moment it tries to
drive a wheel, because the ASCII-verb `NezhaProtocol` methods it calls
were deleted when the wire moved to protocol v5. This is not a fresh
discovery — `clasi/issues/later/nezha-facade-and-midlayer-dead-verb-
residue.md` already names most of it — but this review found the same
rot one layer further into this scope than that issue tracks
(`calibration/linear.py`/`angular.py`'s own ASCII verbs over a raw
serial connection), and it matters directly to sprint 127's own
"exactly one way in" question: **the count of live goto stacks today is
not three, it is effectively zero-live/one-dead/one-differently-scoped/
one-in-progress**, and sprint 127's own Out-of-Scope section defers
deleting the dead one to a later consolidation sprint it explicitly
names. That deferral is a reasonable sprint-scoping call, not a defect
in this sprint's own diff — but the tree the *next* engineer opens still
has five sibling packages implying five ways to move the robot, and this
report's job is to say plainly which of those five are real.

---

## Findings

### 1. CRITICAL — `nav/camera_goto.py` and `nav/navigator.py` are dead code, still exposed as live CLI commands and MCP tools

**Category:** accretion / interface-bleed
**Evidence:**
- `src/host/robot_radio/nav/camera_goto.py:283,296,304` (`go_to_world_camera`) and `nav/camera_goto.py` (`spin_to_yaw_camera`) call `proto.drive(...)` where `proto` is asserted (`io/cli.py:738`, `isinstance(proto, NezhaProtocol)`) to be a real `NezhaProtocol`.
- `src/host/robot_radio/robot/protocol.py:823` — `NezhaProtocol`'s docstring: "Binary wire-protocol adapter for the P4 single-loop Nezha firmware... one method per firmware command group (`move`/`stop`/`config`)." A runtime check confirms it: `'drive' in dir(NezhaProtocol)` → `False`, and neither `go_to`, `stream`, nor `wait_for_evt_done` exist either (full live surface: `config, estimator_config, estop, is_open, mode, move, move_twist, move_wheels, otos_config, ..., read_pending_binary_tlm_frames, send, send_fast, set_config, set_config_binary, stop, tlmNow/Off/On, wait_for_ack, wheels`).
- `nav/navigator.py:211` (`Navigator.navigate()`) calls `self._robot.go_to(...)`, which is `robot/nezha.py:406-414` — itself calling `self._proto.go_to(...)` / `self._proto.stream(80)` / `self._proto.wait_for_evt_done("G", ...)`, none of which exist on `NezhaProtocol`.
- **Failing scenario**: `rogo goto 30 10` (`io/cli.py:cmd_goto`) or the MCP tool `navigate`/`follow_path` (`io/robot_mcp.py:229,271`, wired to `nav_mod.Navigator` at line 122) is invoked against the actual hardware connection `connection.make_robot()` builds today (`robot/connection.py:316`: `robot = Nezha(NezhaProtocol(conn))` — confirmed this is the live construction path, not a hypothetical one). The very first attempt to drive raises `AttributeError: 'NezhaProtocol' object has no attribute 'drive'` (camera_goto path) or `'...' has no attribute 'go_to'` (Navigator path). `rogo goto`'s own `try/finally` catches nothing — it propagates, hits `finally: proto.stop()` (harmless, since `stop()` does exist), and the command dies with a Python traceback instead of doing anything.
- This is a **known, already-filed** issue — `clasi/issues/later/nezha-facade-and-midlayer-dead-verb-residue.md` (filed after ticket 104-002, status `pending`) names `nav/camera_goto.py`/`nav/navigator.py` explicitly as calling "`NezhaProtocol.drive()`/`Robot.go_to()` directly... none of which exist on the P4 wire," and states plainly: "`nav/`... should be treated as non-functional against a P4 firmware robot." This review independently re-derived and confirmed the same conclusion at the interpreter level (see the `dir()` check above) rather than taking the issue's word for it, and additionally verified that the crash is real, not merely a documentation claim.
- **New scope beyond the filed issue**: the issue's audit did not cover `calibration/linear.py`/`calibration/angular.py` (this review's assigned scope). Both send raw ASCII verbs (`ser.write_line("STOP")`, `linear.py:95` builds and sends a `T`-style command string) over `calibration/_conn_helpers.RelaySerial`/`DirectSerial` — a deliberately separate raw-pyserial connection, not `NezhaProtocol` (documented, legitimately, in `robot/_legacy_tlm_text.py`'s own header as an exception for DTR-timing control). But the ASCII verbs themselves (`T`/`STOP`/etc.) are the exact same retired wire grammar the filed issue describes as gone from the P4 firmware ("no bare-command REPL shape any more" — `hardware-bench-testing.md`). Sending them over a raw serial connection instead of through the dead `NezhaProtocol.drive()` method doesn't make the firmware understand them — it just fails one layer further down (silently, at the wire, rather than loudly with an `AttributeError`), which is arguably worse. **This extends the filed issue's own scope**: `calibration/linear.py`/`calibration/angular.py`'s *interactive calibration* drive logic is very likely equally non-functional against current firmware, even though the issue's file list doesn't name it.
- **Design answer**: `nav/` should not be silently imported as if live. Until the filed issue's "future sprint (105+)" redesign lands, at minimum: (a) `nav/camera_goto.py`/`nav/navigator.py`'s motion-issuing methods should fail loudly and immediately (an explicit `NotImplementedError` at the top of `go_to_world_camera`/`Navigator.navigate`/`follow_path`/`visit_tags`, not a deep `AttributeError` three call-frames down mid-loop) so the MCP tool surface (`navigate`, `follow_path`, `visit_tags`) stops advertising a capability that crashes; (b) the same treatment for `calibration/linear.py`'s/`angular.py`'s drive primitives, and a note added to the filed issue naming this file pair as additional affected scope. This is a discussion item for the stakeholder per the filed issue's own "future sprint" framing — not something this review is asking to fix inline.

### 2. MAJOR — `planner.executor.StreamingExecutor` is dead code whose own "no adapter needed in production" claim is false

**Category:** accretion / correctness (stale documented invariant)
**Evidence:**
- `src/host/robot_radio/planner/executor.py:94-101` — `TwistTransport` Protocol requires `.twist(v_x, omega, duration)`. Docstring: "A real `NezhaProtocol` instance already satisfies this Protocol as-is -- no adapter needed in production." Runtime check: `'twist' in dir(NezhaProtocol)` → `False`; only `move_twist` exists. The claim is false today (it may have been true before a rename).
- `grep -rn "StreamingExecutor("` across the live tree (`src/host`, `src/tests`) returns **zero** production call sites — only its own module and its own unit test construct it. `planner/tour.py` (the actually-used, testgui-wired module — `TOUR_1`/`TOUR_2`, imported by `testgui/__main__.py:1918`, `testgui/commands.py:62`, six files under `src/tests/testgui`, `fake_otos_tour_bench.py`, `turn_prediction_capture.py`) imports only `RunOutcome`/`RunState`/`TickResult` from `executor.py` — the enum/dataclass shells, not `StreamingExecutor` itself. `run_tour()` implements its own per-leg `Move`-based loop against a `MoveTransport` Protocol (`.move()`, matching the live `NezhaProtocol.move()`) instead.
- **Consequence**: roughly 300 lines of `executor.py` (the `StreamingExecutor` class, ~145-450), documented with an elaborate "ten binding requirements" traceability map to `host-planner-design-lessons-from-drive-v2-review.md`, are unreachable from any production code path, and the one sentence in its own docstring asserting production-readiness is stale and wrong.
- **Design answer**: either (a) `StreamingExecutor` is genuinely superseded by `tour.py`'s own `Move`-based loop and should be deleted along with its docstring's binding-requirement claims (the guidelines' "gut the old thing decisively" — §3), or (b) if it is meant to be `pathplan.planner`'s (ticket 006, not yet started) actual transport — since a continuously-replacing twist-based outer loop is exactly the shape `StreamingExecutor` was built for — that intent should be stated explicitly in sprint 127's Architecture section (it currently is not; ticket 006's own description sketches a bespoke loop, not a reuse of `executor.py`). Silently carrying a fully-built, disconnected, doc-heavy class forward is the "half-retired code is worse than either state" pattern the guidelines call out by name.

### 3. MAJOR — `path/arc.py`'s `compute_arc` is unexported dead code solving the exact problem ticket 005 rewrote from scratch

**Category:** accretion / placement
**Evidence:**
- `src/host/robot_radio/path/arc.py:13-76` (`compute_arc`) computes "the unique circular arc through `start_pos` that is tangent to the robot's heading at start and also passes through `target_pos`" — the identical geometric problem `pathplan/solver.py:218` (`solveArcToPoint`) solves for ticket 005, six weeks/one sprint later, with a from-scratch derivation (re-derived tangent-circle identities, re-derived degenerate-case handling).
- `path/__init__.py:18-22` exports `build_path`, `SampledPath`, `catmull_rom`, `plan_path`/`build_safe_spline`, `four_leaf_waypoints` — **not** `arc`/`compute_arc`. `grep -rn "compute_arc"` finds only `path/arc.py`'s own definition and the archived `src/archive/tests_old/simulation/unit/test_imports_smoke.py`. No live caller anywhere.
- The two implementations are not identical in shape (`arc.py` returns per-wheel arc distances + signed radius; `solver.py` returns a twist `v_x`/`omega`/`arcLength`), so this is not a pure duplicate-and-delete — but the tangent-circle derivation itself (the hard part: bearing computation, degenerate/behind-target handling) is the same problem solved twice, unknowingly, four modules apart.
- **Design answer**: whoever picks up ticket 006/008 (or a future maintainer touching either) should be pointed at this file before assuming `solveArcToPoint` is the first attempt at this geometry. At minimum, `path/arc.py` should either be deleted (dead, unexported, unused) or explicitly noted in `pathplan/solver.py`'s own header as "related, but superseded/differently-shaped" so the duplication is acknowledged rather than accidental.

### 4. MAJOR — `path/catmull_rom.py`'s `find_lookahead_target`/`circle_intersections` are the exact primitive ticket 008 needs, dead and undiscovered

**Category:** accretion / placement
**Evidence:**
- `src/host/robot_radio/path/catmull_rom.py:63-86` (`find_lookahead_target`) already implements "walk forward from `start_idx` and return the first exit from the lookahead circle" over a raw polyline — precisely SUC-008's own Main Flow ("pick the point on the path a lookahead distance ahead"). `circle_intersections` (line 46) is its supporting primitive, correctly handling the `da == db` degenerate case (`t = 0.5 if abs(da - db) < 1e-9 else da / (da - db)`, no division-by-zero).
- `grep -rn "find_lookahead_target\|circle_intersections"` finds zero callers anywhere in the tree, including the archived smoke test — genuinely dead, and not even exported from `path/__init__.py` (only bare `catmull_rom` is).
- Sprint 127's own sprint.md (ticket 008 description) frames the lookahead target-picker as new work over `SampledPath` (currently a plain, method-free dataclass — confirmed at `path/sampled_path.py:8-47`, no lookahead query of any kind), with no mention of this existing primitive.
- **Design answer**: ticket 008, when picked up, should evaluate adapting `find_lookahead_target` (rewritten to walk a `SampledPath`'s `points`/`headings` instead of a bare list-of-tuples) rather than re-deriving circle-intersection geometry a second time. This is exactly the kind of reuse the guidelines ask reviewers to surface before a ticket starts, not after.

### 5. MAJOR — `nav/navigator.py` and `nav/camera_goto.py`'s halt paths use `stop()`, not `estop()`, on every failure branch

**Category:** explicitness (would be CRITICAL if the code were live; downgraded because Finding 1 establishes it currently cannot reach the wire at all)
**Evidence:**
- `nav/camera_goto.py`: `go_to_world_camera`'s pose-loss branch (`if p is None: proto.stop(); continue`), its timeout branch (`proto.stop(); print("WARNING: hit ... timeout")`), and the end of every forward-burst segment (`proto.stop(); time.sleep(0.15)`) all call the **planned** stop, never `estop()`.
- `nav/navigator.py:663,685,698,729` (`Navigator.approach()`) calls `self._robot.stop()` on `lost_robot_tag`, on arrival, on timeout, and in its `except` handler — again, never `estop()`.
- Per `.claude/rules/playfield-testing.md`'s own measured table and `robot/nezha.py:179-204`'s own docstrings: `stop()` is a queued command that "waits behind whatever is already in flight" (measured: a full 39.8 cm leg still completes before it takes effect); `estop()` is required for every "halt now" path, explicitly including "a fence detected correctly and stopped nothing" as the cost of getting this wrong once already. Losing the camera tag mid-approach is precisely the scenario that rule exists for.
- Contrast: `nav/camera_goto.py`'s `spin_to_yaw_camera` gets this right in two places (`proto.estop()` at both the on-target and timeout exits, with a comment explaining why `estop()` specifically is needed) — so the correct pattern was known and used elsewhere in the *same file*, just not applied consistently to `go_to_world_camera`.
- **Why MAJOR not CRITICAL**: Finding 1 establishes that `proto.drive(...)` itself raises before any of these halt branches can matter on hardware today. This is a real defect in the code as written, but its blast radius is currently zero because the surrounding function cannot run. It should be fixed in the same pass that resolves Finding 1, not independently.

### 6. MINOR — `kinematics/differential_drive.py` carries a silent CW-positive convention against the rest of the project's CCW-positive convention

**Category:** interface-bleed
**Evidence:**
- `kinematics/differential_drive.py:9-10`: "yaw/omega: CW-positive radians (not CCW)," with explicit negation (`-omega` at line 20, `-chassis.omega` at line 29) to convert to/from WPILib's own CCW-positive `ChassisSpeeds`.
- Every other reviewed module in this scope is CCW-positive: `nav/pose.py:25` ("East = 0, counter-clockwise positive"), `pathplan/solver.py`/`world_pose.py` (both explicitly note they match `Pose.heading`'s CCW convention), and `.claude/rules/playfield-testing.md`'s camera-yaw convention.
- Its only current caller, `robot/nezha_kinematic.py`, is itself named in `clasi/issues/later/nezha-facade-and-midlayer-dead-verb-residue.md` as part of the same dead-verb layer as `nezha.py` — so this sign inversion is inert today, not live-wrong.
- **Design answer**: no action needed now (no live caller), but flag it for whoever next reaches for a differential-drive wheel-speed conversion (e.g. if `pathplan`'s planner loop, ticket 006, ever needed host-side wheel speeds instead of a firmware-side twist) — this module is a landmine for a CCW-convention caller that doesn't read its docstring closely.

### 7. NOTE — sprint 127's own naming rationale mischaracterizes `planner/`'s liveness

**Category:** style / accretion (documentation accuracy)
**Evidence:**
- `clasi/sprints/.../sprint.md`'s Solution section: "New package `src/host/robot_radio/pathplan/` (`planner/` is taken by trajectory profiles from the pre-gut era..." — this reads as "legacy, low-relevance." In fact `planner/tour.py`/`heading.py`/`model.py`/`profile.py` are heavily live: imported by `testgui/__main__.py`, `testgui/commands.py`, six `src/tests/testgui/*` files, and two bench scripts; `TOUR_1`/`TOUR_2` are the square-tour geometry testgui's own button-acceptance tests exercise today (`test_tour_closure_gate.py`, `test_sim_transport_tour1.py`).
- This doesn't change the naming decision (avoiding a `planner`/`pathplan` collision was correct regardless), but the stated reason ("pre-gut era," implying dead) is inaccurate — `planner/` is a live, differently-scoped stack (fixed/parameterized tour execution + heading correction) that happens not to overlap with `gotoWorld`'s "arbitrary point" scope, not legacy code nobody uses.
- **No action implied** — flagging only so a future reader doesn't infer `planner/` is safe to delete or ignore from this sentence alone.

### 8. NOTE — `pathplan.solver.solveArcToPoint`'s returned `arcLength` is for the unclamped arc, not the clamped trajectory actually driven

**Category:** correctness (explicitly acknowledged design tradeoff, not a bug)
**Evidence:** `pathplan/solver.py:259-264` documents this precisely: when the curvature slew limit (`_clampOmegaStep`) reduces `omega`, the returned `arcLength` is still computed from the *unclamped* tangent-circle solution, so the `Move`'s `stop_distance` will not exactly match the trajectory the clamped `omega` actually produces. The module's own docstring correctly identifies why this is acceptable (ticket 006's continuous re-solve/replace corrects the drift on the very next cycle) rather than a latent bug. Flagging as a NOTE because it is exactly the kind of thing the guidelines ask reviewers to name even when accepted — it is a real, if bounded, geometric inconsistency, and ticket 007's convergence-gate acceptance criteria should be aware that a *single* solve's `arcLength` is not a ground-truth distance under clamping.

---

## Package table

| Package | Purpose | Callers (live tree, excluding `src/archive`) | Verdict |
|---|---|---|---|
| `nav/` | Camera-feedback go-to-point (`camera_goto.py`), route planner + two-phase approach controller (`navigator.py`), shared `Pose`/`Waypoint`/`heading_error` value types (`pose.py`) | `pose.py`: widely used (`sensors/`, `robot/robot_state.py`, `robot/nezha_state.py`, `robot/nezha.py`, `pathplan/`) — **live, foundational**. `camera_goto.py`/`navigator.py`: `io/cli.py` (`rogo goto`/`turnto`), `io/robot_mcp.py` (`navigate`/`follow_path`/`visit_tags`/`approach`/`grab_at`/`release_at` MCP tools) | **`pose.py` LIVE; `camera_goto.py`/`navigator.py` DEAD** (confirmed via runtime `AttributeError` against current `NezhaProtocol`; already tracked in `clasi/issues/later/nezha-facade-and-midlayer-dead-verb-residue.md`) |
| `path/` | Pure curve/path geometry: `SampledPath`, Bezier/Catmull-Rom builders, obstacle-avoiding spline, cloverleaf pattern, arc-tangent-circle math | `io/robot_mcp.py` (`build_path`, `plan_path` MCP tools); `path/obstacle.py` uses `catmull_rom`; nothing yet imports `SampledPath`'s lookahead capability (none exists) | **LIVE (builder/obstacle/patterns, exposed as MCP tools) + DORMANT (`SampledPath`, awaiting ticket 008) + DEAD (`arc.py`, `catmull_rom.find_lookahead_target`/`circle_intersections` — unexported, uncalled)** |
| `pathplan/` | Sprint 127's designated `gotoWorld`/`gotoRobot` owner: `WorldPose` (SE(2) re-anchor tracker), `solveArcToPoint` (pure arc solver) | `world_pose.py`: committed, ticket 004 done, unit-tested. `solver.py`: **uncommitted** (`git status`: `??`), ticket 005 still `open` in CLASI, but functionally complete with 16 passing tests | **WIP — `world_pose.py` done/solid; `solver.py` implemented-but-unlanded (parallel-session work, not yet committed or marked done); the planner loop itself (ticket 006) does not exist yet** |
| `planner/` | Fixed/parameterized square-tour execution (`tour.py`, `TOUR_1`/`TOUR_2`), heading-correction outer loop (`heading.py`), trapezoidal profile generator (`profile.py`), live-tunable params (`model.py`); `executor.py`'s `StreamingExecutor` | `testgui/__main__.py`, `testgui/commands.py`, 6+ `src/tests/testgui/*.py`, `fake_otos_tour_bench.py`, `turn_prediction_capture.py` — heavily live | **LIVE (`tour.py`/`heading.py`/`model.py`/`profile.py`) — a different-scoped stack (fixed tours, not arbitrary-point goto) — but `executor.py`'s `StreamingExecutor` class specifically is DEAD (zero production callers, stale "no adapter needed" claim)** |
| `controllers/` | Shared `PID` + `normalize_angle` leaf utility | `planner/tour.py`, `planner/heading.py`, `nav/navigator.py`, `io/robot_mcp.py` | **LIVE — genuine shared leaf, correctly used by multiple stacks, not a duplicated implementation** |
| `kinematics/` | `DifferentialDriveKinematics` (wpimath wrapper, CW-positive convention) | `robot/nezha_kinematic.py` only | **Effectively DEAD today** — its one caller is part of the same dead-verb `Nezha` layer `clasi/issues/later/nezha-facade-and-midlayer-dead-verb-residue.md` already flags |
| `calibration/` | Interactive OTOS linear/angular scale calibration (`linear.py`/`angular.py`), config-push-on-connect (`push.py`), sim boot-config derivation (`sim_boot_config.py`), sim-vs-hardware error-model fit (`fit_sim_error_model.py`) | `push.py`/`sim_boot_config.py`: heavily live (`testgui/__main__.py`, `io/robot_mcp.py`, `move_accuracy_bench.py`, extensive `src/tests` coverage). `linear.py`/`angular.py`: reachable only via `io/calibrate.py` | **`push.py`/`sim_boot_config.py`/`helpers.py` LIVE and well-tested. `linear.py`/`angular.py`'s interactive drive logic is very likely equally non-functional against current firmware as `nav/`** (sends the same retired ASCII verbs over a deliberately separate raw-serial connection — see Finding 1) |

## `src/sim` — one entry point, confirmed

`SimLoop` (`src/host/robot_radio/io/sim_loop.py`) is the single Sim wrapper used by both `testgui/` (`turn_shape.py`, `transport.py`, `sim_prefs.py`) and `src/tests/` (bench scripts, testgui tests, `src/tests/sim/*`) — a `grep` across the live tree for `SimLoop(` and for `ctypes.CDLL`/`firmware_host`-style raw ABI access found no second bespoke entry point in this scope. This matches the project's own established convention (no second finding needed here — see "What's good").

## Duplicated pose estimation — reconciliation is explicit where it exists

`pathplan.world_pose.WorldPose` deliberately tracks **two** independent transforms (`worldFromOdom`, encoder-anchored; `worldFromOdomOtos`, OTOS-anchored) rather than fusing them, and is explicit about why: the divergence between them (`encoderOtosDivergence()`) is a first-class, intentional output, not an accident of not-having-decided. Both are re-anchored from the same camera fix at the same instant (`reanchor()`), so there is exactly one authority (the camera) and two trackers whose disagreement is measured, not silently averaged. This is the "reconciliation is explicit" model the guidelines ask for (§1). All wrap-sensitive residuals (`solveTransform`'s rotation, `encoderOtosDivergence`'s heading) go through `nav.pose.heading_error` — reused, not reforked, and I traced the sign convention through `solveTransform`/`_extrapolate`/`encoderOtosDivergence` and found it internally consistent (verified against `heading_error`'s own docstring: "signed angular difference *b − a*").

Outside `pathplan/`, no second live world-frame pose estimator exists to reconcile against — `nav/navigator.py`'s camera-only reads and `planner/heading.py`'s encoder-or-OTOS selection (`otos_untrusted`-gated, one source at a time, never both) don't compute a persistent world-frame transform the way `WorldPose` does, so there's no "three pose estimators" situation live today, only `WorldPose` (new) and two dead/differently-scoped precedents.

## Waypoint/segment math — wrap and degeneracy audit

- `pathplan/solver.py`: distance and bearing degeneracies both guarded (`_MIN_DISTANCE`, `_MIN_BEARING`) before any division; heading only ever enters `cos`/`sin` (periodic, no wrap needed) or `atan2` (self-wrapping). No division-by-zero path found.
- `pathplan/world_pose.py`: `Transform2`/`solveTransform`/`_extrapolate` all use `heading_error` for any residual; verified sign convention consistent end-to-end (see above).
- `path/catmull_rom.py`: `circle_intersections`/`find_lookahead_target` both guard the `da == db` case explicitly (`abs(da - db) < 1e-9`) before dividing — correct, though dead (Finding 4).
- `path/arc.py`: guards the on-heading degenerate case (`abs(cross) < 1e-6`) before dividing by `cross` — correct, though dead/unexported (Finding 3).
- `controllers/pid.py:normalize_angle`: correct (while-loop wrap to `(-pi, pi]`), but note it is O(n) in the magnitude of its input; every live caller passes it an already-small residual today, so this is a NOTE, not a finding.

## What's good

- `pathplan/world_pose.py` is genuinely close to the textbook bar: every non-obvious decision (why two transforms, why frame-age extrapolation instead of `ClockSync`, why `heading_error` and never raw subtraction) is explained in the module or class docstring at the point it matters, not in a separate design doc that will drift. The re-anchor/extrapolate/divergence split maps cleanly onto the three things it's responsible for.
- `pathplan/solver.py`'s curvature slew limit derivation is a model of "no magic timing constants without provenance" (guidelines §4): every constant traces to a measured number (ticket 001's Edge-B figure, `hil_drive.py`'s measured `aDecel`), with the arithmetic shown inline so a future reader can re-derive it rather than trust it.
- `planner/executor.py`'s binding-requirement-to-mechanism map (its own header comment) is an unusually good instance of "comments state constraints the code can't show" — even though the class itself turned out to be unused (Finding 2), the documentation discipline that produced it is worth keeping as the house style.
- `field/geofence.py`'s promoted `captureFix` correctly switched from a linear median of raw yaw to the circular mean (`atan2(mean(sin), mean(cos))`) — a real wrap-safety bug fixed in transit, exactly as ticket 003 claimed, and I confirmed the fix is in place rather than taking the ticket's word for it.
- `src/sim`'s single-entry-point discipline (`SimLoop` for both `testgui` and `src/tests`) holds in this scope — no second bespoke sim API was found.
- `planner/tour.py`'s `MoveTransport` Protocol (structural typing over `NezhaProtocol`, not a concrete import) is the right shape for testability, and it is one of the few Protocol boundaries in this scope whose "a real X already satisfies this" claim I checked and found **true** (`NezhaProtocol.move()` genuinely matches), unlike `executor.py`'s `TwistTransport` (Finding 2).
