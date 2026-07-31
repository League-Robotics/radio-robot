# Craftsmanship Review — Consolidated Summary

**Date:** 2026-07-30
**Scope:** `src/firm`, `src/motion`, all packages under `src/host/robot_radio`
(`robot`, `io`, `config`, `sensors`, `field`, `media`, `nav`, `path`,
`pathplan`, `planner`, `controllers`, `kinematics`, `calibration`, `testgui`,
`testkit`), and `src/sim` — roughly 75k lines of product code. Reviewed
against [`docs/code_review/GUIDELINES.md`](GUIDELINES.md), which this review
adopts as the operative standard for the first time. Branch:
`sprint/127-host-side-path-planner-goto-and-path-following` (WIP from a
parallel session), reviewed as-is.

| Part | Scope | Verdict |
|---|---|---|
| [01-firm.md](01-firm.md) | `src/firm` (~12.6k lines) | **Merge-with-findings** — tree is close to textbook in most places; one CRITICAL (I2C errata guard shipped OFF) and a cluster of MAJORs around an undocumented architecture cutover (`Motion::Planner` replacing `MoveQueue`) |
| [02-motion.md](02-motion.md) | `src/motion` (~8.2k lines) | **Rework**, on documentation/ownership — live math (`profile.cpp`, `shape.cpp`) is careful and correct; the tree is carrying three dead/half-dead motion-decision generations and `DESIGN.md` describes an architecture that no longer runs |
| [03-host-core.md](03-host-core.md) | `robot/`, `io/`, `config/`, `sensors/`, `field/`, `media/` (~17k lines) | **Rework, not merge** — the halt path (`field/geofence.py`) is excellent, but the driver facade and CLI/REPL surfaces above it call protocol methods deleted by the v4→v5 cutover and never call `estop()` |
| [04-host-planning.md](04-host-planning.md) | `nav/`, `path/`, `pathplan/`, `planner/`, `controllers/`, `kinematics/`, `calibration/`, `src/sim` (~7.6k lines) | **Merge-with-findings** for sprint 127's own WIP (`pathplan/world_pose.py`, `solver.py`) — both near textbook; but `nav/` is dead code still exposed as live CLI/MCP surface, and two solved-twice geometry primitives sit undiscovered |
| [05-testgui-testkit.md](05-testgui-testkit.md) | `testgui/`, `testkit/` (~13.4k lines) | **Rework required before trusted on hardware again** — the `Transport` ABC is well built, but the GUI's STOP button and arrow-key drive are both silently no-ops against real hardware |

---

## Overall verdict

This is not a codebase in trouble — the load-bearing safety and correctness
disciplines the guidelines care most about (`field/geofence.py`'s halt
retry-then-raise, `App::Telemetry`'s single-assembly frame, `Devices::
NezhaMotor`'s never-latch-before-confirmed writes, `pathplan/world_pose.py`'s
explicit dual-transform reconciliation, `transport.py`'s one-ABC-three-backends
shape) are genuinely close to the "textbook" bar the guidelines set, and every
part reviewer found them independently, in different trees, without
coordinating. The problem is what sits *above* those disciplined cores: this
project has now been through enough decisive architecture cutovers — protocol
v4→v5, `MoveQueue`→`Planner`, the testgui legacy-verb deletion, three
successive `nav`/`pathplan`/`planner` goto stacks — that the "gut decisively"
half of GUIDELINES §3 has stopped keeping pace with the "cut over" half. Each
cutover correctly replaced the old mechanism at its center, but left the old
mechanism's *callers* — CLI commands, MCP tools, GUI buttons, design docs —
pointed at the deleted surface instead of walking outward to fix or delete
them. The result is a tree where a stakeholder or a fresh agent reading
`DESIGN.md`, `CLAUDE.md`, or the CLI's own `--help` would form a materially
wrong picture of what actually runs, and where several "obviously safe"
actions (click STOP, press an arrow key, run `rogo goto`, run `rogo calibrate
distance`) either raise immediately or do nothing at all on first contact
with real hardware. None of the five parts found this by inference — every
instance below is a traced call site with a confirmed missing method or a
confirmed unreachable code path, not a suspicion.

---

## Cross-cutting themes

**(a) The protocol v4→v5 cutover was completed in firmware but never
followed through the host mid-layer.** Sprint 124's binary-only wire rewrite
deleted the ASCII verb surface (`ping`/`go_to`/`turn`/`distance`/`stream`/
`snap`/`otos_*`/`drive`) from `NezhaProtocol` cleanly — but three independent
host layers were never repointed and still call the deleted names as if they
existed: the `Nezha` driver facade (03, CRITICAL #1, ~25 dead calls),
`io/repl.py`'s ack-confirmation path reading a deleted `TLMFrame.ack` field
(03, CRITICAL #2), `io/calibrate.py`'s two live calibration commands (03,
CRITICAL #5), and `nav/camera_goto.py`/`nav/navigator.py`'s `proto.drive()`/
`proto.go_to()` calls reached from `rogo goto`/`turnto` and the MCP
`navigate`/`follow_path`/`visit_tags` tools (03, CRITICAL #3; 04, CRITICAL
#1 — the same rot found independently from the CLI-entry side and the
package-ownership side). `testgui`'s parallel legacy-verb deletion
(104-002) left the identical shape one layer over: `binary_bridge.py`'s
dispatch is an unconditional dead stub for `STOP`/`STREAM`/`SNAP`/`ZERO`,
and `testgui/drive.py`'s whole arrow-key-drive feature targets a `DEV *`
verb family with no binary arm on either sim or hardware (05, CRITICAL #1,
#2). This is one systemic gap wearing five costumes, not five unrelated bugs.

**(b) Halt-path discipline is inconsistent everywhere except the one place
it was built with the rule in mind.** `field/geofence.py::_halt()` is cited
by three of the five parts as the correct template (retries `estop()`,
raises loudly on total failure, never swallows). Everywhere else, "halt now"
sites reach for the planned `stop()` instead: six-plus call sites in
`cli.py`/`repl.py`/`calibrate.py`/`robot_mcp.py` (03, CRITICAL #4), every
failure branch of `nav/navigator.py::approach()` and half of
`camera_goto.py::go_to_world_camera` (04, MAJOR — note its sibling function
`spin_to_yaw_camera`, *in the same file*, gets this right and even documents
why), and the entirety of `testgui`'s STOP button / `_safe_stop()` /
`_set_origin()`, which never call `estop()` at all (05, CRITICAL #1). Several
of these wrap the halt call in `except Exception: pass`, so a halt that fails
looks identical to one that worked — the exact "a fence detected correctly
and stopped nothing" failure this project has already paid for once.

**(c) The documented firm↔motion boundary is dead; its replacement is
undocumented.** `Motion::WheelSink` is described by `CLAUDE.md` and
`docs/design/design.md` as *the* one narrow seam between the firmware base
and the motion library — both firm- and motion-side reviewers independently
found it is not called anywhere in the live loop (01, MAJOR #2; 02, MAJOR
#1), because `Motion::Planner` now writes directly into two plain
`RobotState` floats instead. Neither `design.md` §5 nor `src/firm/app/
DESIGN.md` nor `src/motion/DESIGN.md` mentions `Motion::Planner` or
`src/motion/planner/` at all (01, MAJOR #3; 02, MAJOR #5) — the larger,
actually-live half of `src/motion` by every measure (13 source files vs. 6,
9 test executables vs. 3) is simply missing from the documentation tree that
exists to describe this exact boundary.

**(d) "Multiple pose estimators, no owner" recurred independently on both
sides of the firm/host split.** On the firmware side: `Odometry`,
`StateEstimator` (computed every cycle, provably unconsumed), and the
planner's own `PoseTracker` are three independent ZOH/OTOS-blend
implementations, and `RobotState::pose` itself has two writers with
contradictory doc-comment contracts that only don't collide today by
accident of call order (02, MAJOR #2, #3). On the host side: `Nezha.
_apply_tlm`, `NezhaState._apply_tlm`, and `NezhaKinematic._update_odometry`
independently re-derive the same pose from raw telemetry with no
reconciliation (03, MAJOR #2). Both reviewers cited the same precedent
unprompted — GUIDELINES §1's own history callout, "three independent
go-to-point stacks and three pose estimators with no owner" — meaning the
pattern this document was written to name has reproduced itself twice more
since that finding was recorded.

**(e) Shipped-experiment residue sits in the composition root.**
`main.cpp` unconditionally disables the documented "non-negotiable" nRF52
I2C errata guard with a comment admitting it is a temporary bench experiment
"NOT a candidate for shipping as-is" (01, CRITICAL) — every build compiled
from this tree today ships with the wedge-protection mechanism's root cause
unsuppressed. Adjacent to it, an entire `PlannerLimits` tuning struct is
hardcoded as C++ literals in `main()` rather than sourced from
`data/robots/*.json`, contradicting the project's own fail-closed
config-as-truth convention and risking the "one robot's gearboxes become
every robot's" failure this project has already hit once (01, MAJOR #4).

**(f) Work the active sprint is about to do has already been done once,
unknowingly, and left undiscovered.** `path/arc.py::compute_arc` solves the
identical tangent-circle geometry problem sprint 127 ticket 005's
`pathplan/solver.py::solveArcToPoint` re-derived from scratch (04, MAJOR
#3); `path/catmull_rom.py::find_lookahead_target`/`circle_intersections`
already implement the exact lookahead-target primitive ticket 008's Main
Flow describes as new work, correctly handling the degenerate case ticket
008 will need to handle again if it doesn't find this file first (04, MAJOR
#4). Neither is exported or called anywhere; both would have saved
re-derivation time had the tickets' authors known to look.

---

## Top findings across the codebase

Every CRITICAL and MAJOR from all five parts, ranked severity-first. (9
CRITICAL, 22 MAJOR — 31 rows.)

| # | Sev | Category | Finding | File:line | Part |
|---|---|---|---|---|---|
| 1 | CRITICAL | correctness | Composition root ships with the documented "non-negotiable" I2C errata guard turned OFF | `src/firm/main.cpp:201-213` | 01 §1 |
| 2 | CRITICAL | correctness/accretion | `Nezha` driver facade (constructed for the active robot config) calls ~25 `NezhaProtocol` methods that no longer exist | `src/host/robot_radio/robot/nezha.py` (many lines) | 03 §1 |
| 3 | CRITICAL | correctness | `io/repl.py`'s command-confirmation path reads `TLMFrame.ack`, a field deleted by 124-008 — every REPL motion verb crashes | `io/repl.py:105-106` | 03 §2 |
| 4 | CRITICAL | correctness | `nav/camera_goto.py` (the live `goto`/`turnto` implementation) calls `NezhaProtocol.drive()`, which does not exist | `nav/camera_goto.py:149,295` | 03 §3 |
| 5 | CRITICAL | explicitness/correctness | No call site in `cli.py`/`repl.py`/`robot_mcp.py`/`calibrate.py` ever calls `estop()`; 6+ "halt now" sites use planned `stop()`, several with the failure swallowed | `io/cli.py`, `io/repl.py`, `io/calibrate.py`, `io/robot_mcp.py:560` | 03 §4 |
| 6 | CRITICAL | correctness/accretion | `io/calibrate.py`'s two live commands (`rogo calibrate distance`/`turns`) call `NezhaProtocol` methods that don't exist | `io/calibrate.py:400-420,744` | 03 §5 |
| 7 | CRITICAL | accretion/interface-bleed | `nav/camera_goto.py` and `nav/navigator.py` are dead code, still exposed as live CLI commands and MCP tools (`navigate`/`follow_path`/`visit_tags`) | `nav/camera_goto.py`, `nav/navigator.py:211` | 04 §1 |
| 8 | CRITICAL | correctness/explicitness | GUI STOP button sends `"STOP"` into a permanently-dead `binary_bridge` stub, discards the reply, and logs success — does not stop a real robot | `testgui/operations.py:557-614`, `binary_bridge.py` | 05 §1 |
| 9 | CRITICAL | correctness/accretion | Arrow-key drive (`KeyboardDriver`) is fully dead in both Sim and hardware modes | `testgui/drive.py` | 05 §2 |
| 10 | MAJOR | interface-bleed/accretion | `Motion::WheelSink` is a dead interface; the real firm/motion actuation crossing is an undocumented `RobotState` field written by whichever of `Planner`/`Drive` runs last | `src/firm/app/drive.h:125-132`, `src/motion/planner/planner.cpp:1230-1233` | 01 §2 |
| 11 | MAJOR | accretion | `docs/design/design.md` §5 and `src/firm/app/DESIGN.md` describe an architecture (`MoveQueue`, `Drive::setDuty()`) the code no longer has | `docs/design/design.md` §5 | 01 §3 |
| 12 | MAJOR | accretion | `PlannerLimits` tuning hardcoded as C++ literals in the composition root, bypassing config-as-truth | `src/firm/main.cpp:341-433` | 01 §4 |
| 13 | MAJOR | accretion | `MoveQueue`/`WheelSink`/`StopCondition`/`VelocityShaper` fully dead in production; `DESIGN.md` still calls `MoveQueue` "the one consumer that ties everything together" | `src/motion/DESIGN.md:120-131` | 02 §1 |
| 14 | MAJOR | correctness | `RobotState::pose` has two writers (`Odometry::integrate()`, `Planner::update()`) with contradictory doc-comment contracts | `src/firm/types/robot_state.h:164-169`, `planner.cpp:1247-1252` | 02 §2 |
| 15 | MAJOR | accretion | Pose/estimation logic independently implemented three times (`Odometry`, `StateEstimator`, `PoseTracker`) — the 2026-06-11 "three pose estimators" finding recurring inside one tree | `odometry.{h,cpp}`, `state_estimator.{h,cpp}`, `planner/estimation.{h,cpp}` | 02 §3 |
| 16 | MAJOR | accretion | Three generations of the wheel velocity-control law coexist; the live one's duty output (`WheelPid`/`stageDuty()`) is explicitly discarded by the composition root's own comment | `planner/wheel_pid.h`, `main.cpp:408-412` | 02 §4 |
| 17 | MAJOR | accretion | `DESIGN.md` doesn't mention `src/motion/planner/` at all — the larger, actually-live half of the tree | `src/motion/DESIGN.md` §2 | 02 §5 |
| 18 | MAJOR | interface-bleed | `nezha.py`, `cli.py`, `robot_mcp.py` reach past `NezhaProtocol`/`Robot`'s public surface into private `_proto`/`_conn` fields, even where a public equivalent exists | `robot/nezha.py:326,365,...`, `io/cli.py:655,741` | 03 §6 |
| 19 | MAJOR | interface-bleed/placement | Three independent, unreconciled pose/state trackers inside `robot/` (`Nezha._apply_tlm`, `NezhaState._apply_tlm`, `NezhaKinematic._update_odometry`) | `nezha.py:587-633`, `nezha_state.py:124-165`, `nezha_kinematic.py:106-140` | 03 §7 |
| 20 | MAJOR | accretion | `sensors/calibration.py` sends retired `K`-command wire family by its own docstring's admission; `cam_tracker.py`/`odom_tracker.py` are unreferenced duplicates of `odometry.py` | `sensors/calibration.py`, `sensors/cam_tracker.py` | 03 §8 |
| 21 | MAJOR | correctness (fail-closed) | `VisionConfig.robot_tag_id` defaults to `1` (the fixed field-origin tag) instead of failing closed — every real robot config already overrides it | `config/robot_config.py:77` | 03 §9 |
| 22 | MAJOR | accretion/correctness | `planner.executor.StreamingExecutor` is dead code (zero production callers); its own "no adapter needed in production" docstring claim is false | `planner/executor.py:94-101` | 04 §2 |
| 23 | MAJOR | accretion/placement | `path/arc.py::compute_arc` is unexported dead code solving the identical geometry problem ticket 005's `solveArcToPoint` re-derived from scratch | `path/arc.py:13-76` | 04 §3 |
| 24 | MAJOR | accretion/placement | `path/catmull_rom.py::find_lookahead_target`/`circle_intersections` are the exact primitive ticket 008 needs, dead and undiscovered | `path/catmull_rom.py:63-86` | 04 §4 |
| 25 | MAJOR | explicitness | `nav/navigator.py::approach()` and `camera_goto.py::go_to_world_camera` halt on every failure branch with `stop()`, not `estop()` (downgraded from CRITICAL only because Finding 7 makes the code path unreachable today) | `nav/navigator.py:663-729`, `nav/camera_goto.py` | 04 §5 |
| 26 | MAJOR | placement/interface-bleed | `turn_shape.py` — the specific file GUIDELINES cites as wrongly placed — is still in `testgui/`, with three divergent sim-capture paths inside it | `testgui/turn_shape.py` | 05 §3 |
| 27 | MAJOR | accretion | Half of `binary_bridge.py` (~250/632 lines) is unreachable dead code by its own docstring's admission | `testgui/binary_bridge.py:112-119,312-496` | 05 §4 |
| 28 | MAJOR | accretion | `commands.py`'s COMMANDS schema is dead twice over — the panel is hidden, and 5 of 7 rows fall through to the dead `binary_bridge` stub anyway | `testgui/__main__.py:606-738` | 05 §5 |
| 29 | MAJOR | placement | `testkit/` is bench/test tooling sitting in the importable host package, with no current non-archived consumer | `src/host/robot_radio/testkit/` | 05 §6 |
| 30 | MAJOR | placement | `_build_main_window()` is one ~3028-line function covering connection, tour, and GOTO state as nested closures instead of controller classes | `testgui/__main__.py:308-3336` | 05 §7 |
| 31 | MAJOR | placement | "Test S/T/US/UT" buttons embed a full edit-compile-hot-reload-and-verify build pipeline inside the shipped GUI | `testgui/__main__.py:3209-3316` | 05 §8 |

---

## What's good

- **`field/geofence.py::Geofence._halt()`** — retries `estop()` three times,
  raises loudly with the underlying exception chained on total failure,
  checks at true 10 Hz from inside the drive loop. The one halt path in the
  whole review that should be copied everywhere else, not re-derived (03).
- **`pathplan/` sprint-127 WIP** (`world_pose.py`, `solver.py`) — explicit
  dual-transform reconciliation instead of silent fusion, a curvature slew
  limit whose derivation traces to a measured number, wrap-safe residuals
  reused rather than reforked (04).
- **`App::Telemetry`** — the single best instance in the codebase of
  "assembled once, from primary sources, immediately before the boundary,"
  with a bit-layout comment that names the exact defect class it protects
  against (01).
- **`Devices::NezhaMotor::writeRawDuty()`** and **`Devices::
  MicroBitI2CBus::waitForClearance()`** — never latch success before
  confirmation, never spin on a shortfall; both textbook instances of
  GUIDELINES §5 (01).
- **`transport.py`'s `Transport` ABC** — one abstract interface, three
  backends, no branching on backend type anywhere else in the GUI; correct
  threading discipline (QueuedConnection bridges) throughout the package (05).
- **`profile.cpp`'s discrete-exact braking accounting** and **`shape.cpp`'s
  ratio lock** — dense, careful control math with measured-failure-mode
  justification for every non-obvious branch (02).
- **`camera_prefs.py`** — a textbook fix of the "three call sites pick a
  camera three different ways" failure mode, centralized into one
  `select_camera()` helper (05).

---

## Closing note

Per GUIDELINES §7, these findings are discussion items, not work orders.
Nothing in this review or its five parts authorizes refactoring, deletion,
or repair of any of the above — disposition (what gets fixed, what gets
formally retired with a design-doc update, what gets filed as a deferred
issue, and in what order) is a conversation to have with the stakeholder,
not a queue to start working through.
