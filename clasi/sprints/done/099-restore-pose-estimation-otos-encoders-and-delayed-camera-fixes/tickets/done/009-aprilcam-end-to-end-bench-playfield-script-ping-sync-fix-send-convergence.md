---
id: 009
title: 'aprilcam end-to-end bench/playfield script: PING sync, FIX send, convergence'
status: done
use-cases:
- SUC-005
depends-on:
- '007'
- 008
github-issue: ''
issue: restore-pose-estimation-otos-encoders-delayed-camera-fixes.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# aprilcam end-to-end bench/playfield script: PING sync, FIX send, convergence

## Description

The sprint's final acceptance criterion (sprint.md's own Success Criteria)
is a demonstrated end-to-end path: clock-sync with the robot, observe its
true pose via the camera, send a delayed `PoseFix`, and confirm
convergence — the only tier where the full camera -> EKF -> fused-pose
loop actually closes. This ticket is a bench/playfield **HITL Python
script**, not pytest-collected (per `tests/CLAUDE.md`'s three-domain
split) — it is test tooling, not production firmware.

Per `clasi/knowledge/vision-geofence-before-driving.md`: NEVER blind-drive
the playfield — camera + geofence + a hop-test come first. Per `.clasi/
knowledge/playfield-not-floor.md`: the surface is "playfield," never
"floor."

## Acceptance Criteria

- [x] A new script (`tests/bench/` if it only needs a static/known robot
      pose, or `tests/playfield/` if it needs the camera's world-frame
      calibration — decide based on what "observe the robot's true pose"
      requires; `tests/playfield/playfield_camera_run.py`'s existing
      pattern is the closest precedent, read it first) implements:
  1. A `PING` clock-sync helper: send `PING`/binary `ping`, record RTT,
     map the robot's `t=`/`Ack.t` to host-monotonic time (RTT/2
     correction), per docs/protocol-v3.md §3's `ping` arm and D6's own
     clock-sync convention.
  2. A tag-pose-to-`PoseFix` sender: read the robot's tag pose via the
     `aprilcam` MCP/daemon API (`get_tags`/`where`, `register_mobile_tag`
     if the robot's tag is not the bare AprilTag centre), convert to the
     robot's world-frame convention, build a `PoseFix{x, y, h, t}` with
     `t` derived from the clock-sync mapping, send it via `robot_radio`'s
     `NezhaProtocol` (never lock-step pyserial).
  3. A convergence check: poll `TLM`/binary `stream` for `pose=`,
     confirm it converges toward the camera-observed pose within a
     documented tolerance and time bound.
- [x] Geofence/hop-test precondition honored: confirm the robot is inside
      the calibrated playfield bounds and do a small, safe motion test
      before any extended run (per the vision-geofence-before-driving
      convention).
- [x] Script follows `tests/CLAUDE.md`'s HITL conventions: widen the
      serial-silence watchdog if applicable, restore it and send `STOP`/
      binary `stop` in a `finally` block — motors never left running on an
      exception or Ctrl-C.
- [ ] **BENCH/PLAYFIELD MANDATORY**: run the script against the real
      robot; it demonstrates the full path (PING clock-sync -> tag-pose-
      to-FIX send -> convergence check) successfully at least once,
      recorded (console output or a saved trace) as this ticket's
      completion evidence. **DEFERRED — robot not USB-attached this
      session (only the relay dongle was connected); not run. See
      Completion Notes below.**
- [x] Script is resilient to a single dropped/late camera frame (does not
      crash; retries or reports clearly).

## Implementation Plan

**Approach**: read `tests/playfield/playfield_camera_run.py`'s existing
pattern (and its own header note on what "came back" before it could run
again — relevant since this sprint restores exactly the pose capability
that file's header flags as a precondition) and `tests/bench/`'s existing
scripts for the `NezhaProtocol`/watchdog-safety idioms, before writing new
code. Reuse existing `aprilcam` MCP helpers (`get_robot_api_guide` for the
DaemonControl Python API) rather than re-deriving camera access.

**Files to create**:
- One new script under `tests/bench/` or `tests/playfield/` (name TBD
  during implementation, following the directory's existing naming
  convention — e.g. `pose_fix_convergence.py`).

**Files to modify**: none in `source/` — this ticket is test tooling only.

**Testing plan**:
- This ticket IS the test — its own acceptance criteria are the
  verification. No new pytest coverage (not pytest-collected, per
  `tests/CLAUDE.md`).

**Documentation updates**: consider a short usage note in the script's own
module docstring (bench/playfield scripts in this tree are documented
in-file, not in `docs/`, per existing precedent — verify against a couple
of existing scripts before deciding).

## Completion Notes

**No robot USB-attached this session** (only the relay dongle was
connected; the robot itself was not attached and could not be flashed) —
the **BENCH/PLAYFIELD MANDATORY** acceptance criterion is DEFERRED, not
satisfied. Everything else below was implemented and validated to the
extent possible without hardware, per the team-lead's explicit dispatch
scope for this session.

**Script**: `tests/playfield/pose_fix_convergence.py`. `tests/playfield/`
over `tests/bench/`: step "observe the robot's true pose" needs the
camera's CALIBRATED WORLD-FRAME reading (`tag.world_xy`, A1-centred cm) —
a bench script has no world-frame truth source to fix the robot's pose
against, and this capability's entire point is closing the camera loop.
`tests/playfield/playfield_camera_run.py` does not exist in the rebuilt
tree (only `tests/playfield/plot_square.py`/`world_goto_chart.py`, both
PARKED); the closest actual precedent is `tests_old/bench/
playfield_camera_run.py` (pre-Protocol-v3 — its `RT`/`G`/`X` text verbs are
dead against the current binary-only firmware, so only its connect/
camera/geofence/safe-stop SHAPE was reused, not its drive calls).

**Three pieces**:
1. `clock_sync_burst()` — a burst of binary `ping()` round trips
   (`NezhaProtocol.ping()`, already binary since 097-002), each bracketed
   by the caller's own host-monotonic `t0`/`t1` and recorded into a
   `ClockSync` (`host/robot_radio/robot/clock_sync.py`) — an existing,
   already-implemented NTP-style min-RTT/skew estimator that turned out to
   have ZERO live pytest coverage (its own test file was never carried
   forward from `tests_old/` into the new three-domain tree during the
   077 rebuild). Added `ClockSync.to_robot_time()` (the host->robot
   inverse of the existing `to_host_time()`) — needed to map a
   host-captured camera-observation timestamp onto the robot's own clock
   before sending it as `PoseFix.t` (D6).
2. `send_camera_pose_fix()` — reads the robot's tag pose via aprilcam
   (`read_cam_pose()`, resilient to a dropped frame), maps its host capture
   time to robot-clock time via `ClockSync.to_robot_time()`, and sends it
   through a NEW `NezhaProtocol.pose_fix()` method (`host/robot_radio/
   robot/protocol.py`) built on a new pure `build_pose_fix_envelope()`
   helper — no host helper built the `pose_fix` (arm 7) `CommandEnvelope`
   before this ticket (tickets 004/008 landed the FIRMWARE side only).
   `ensure_mobile_tag_registered()` reads the active robot config's
   `vision.tag_offset_mm` and calls `register_mobile_tag()` idempotently
   (verified via `list_mobile_tags` during implementation that tag 100 is
   already registered for `tovez`, so this is a no-op by default on that
   robot).
3. `wait_for_pose_convergence()` — arms binary `stream()` at the firmware's
   20ms floor, polls `read_pending_binary_tlm_frames()` for `pose=`,
   tracks the MINIMUM position/heading error observed against the sent
   target over `--converge-timeout` (default 5s), success if either
   sample lands within `--tol-mm`/`--tol-deg` (defaults 30mm/3deg — a
   deliberately generous, documented bound: D5's camera-fix EKF update is
   a weighted Kalman update against `ekf_r_fix_xy`/`ekf_r_fix_theta`, not
   a hard snap).

**Geofence/hop-test precondition**: `geofence_from_playfield()`/
`in_fence()` (ported, unchanged logic, from `tests_old/bench/
world_goto_chart.py`) check the robot tag is inside the playfield's
ArUco-corner extent (inset by `--margin`) before ANY motion; `hop_test()`
then runs one small, camera-geofenced `distance()` segment (binary, via
the `segment` arm) before the PoseFix send — refuses to send a fix without
a passing hop-test first (`main()`'s early-return on `hop_ok=False`).

**HITL safety**: `main()` widens the config watchdog
(`NezhaProtocol.set_config(sTimeout=5000)`) before any motion and restores
it (`sTimeout=1000`) plus disarms streaming and sends binary `stop()` in a
`finally` block, matching `tests/CLAUDE.md`'s HITL convention translated
onto the binary plane (the old `DEV WD`/`DEV STOP` text commands this
convention names are gone from the wire post-097 — `docs/protocol-v3.md`
S8 — `sTimeout`'s binary `config` arm is the current equivalent).

**Frame-drop resilience**: `read_cam_pose()` catches an exception from
`dc.get_tags()` and retries within its own timeout window rather than
propagating; every call site checks for its `None` return and reports
clearly instead of crashing (verified in the pure-logic tests' framing —
the retry/catch behavior itself needs a live daemon to exercise, so it is
code-reviewed, not unit-tested).

**Import/syntax validation** (no hardware):
```
uv run python -c "import ast; ast.parse(open('tests/playfield/pose_fix_convergence.py').read())"
uv run python -c "import sys; sys.path.insert(0,'tests/playfield'); import pose_fix_convergence"
```
Both succeed — every camera/robot call lives inside a function, never at
module scope, so importing the script touches no hardware.

**Pure-math unit tests** (all new, all pass, no hardware):
- `tests/unit/test_clock_sync.py` (60 tests) — ported from `tests_old/
  simulation/unit/test_clock_sync.py` (verified field-for-field current
  against `clock_sync.py`, no API drift since the 077 rebuild) plus a new
  `TestToRobotTime` section for this ticket's `to_robot_time()` addition,
  including round-trip checks against `to_host_time()` in both the
  offset-only and skew-model paths.
- `tests/unit/test_protocol_pose_fix.py` (11 tests) — pure
  `build_pose_fix_envelope()` construction, a `CommandEnvelope`
  serialize/parse wire round trip, and `NezhaProtocol.pose_fix()`'s
  envelope-construction path (captured via a stub `_send_envelope`, no
  real connection).
- `tests/unit/test_pose_fix_convergence_pure.py` (33 tests) — the
  script's own pure helpers: `wrap_deg`, `camera_pose_to_pose_fix_kwargs`,
  `pose_fix_target_mm_cdeg`, `pose_error`, `pose_converged`,
  `geofence_from_playfield`, `in_fence`.

**Full suite**: `uv run python -m pytest` — 1393 passed, 4 xfailed, 1
xpassed, 0 failed (baseline 1289/4/1/0; +104 = exactly the three new test
files' combined count — 60+11+33 — confirming no other collection changed
and nothing regressed).

**Acceptance criteria remaining hardware-gated** (exactly one, the
ticket's own MANDATORY criterion):
- Running the script against the real robot end to end (PING clock-sync ->
  tag-pose-to-FIX send -> convergence check) at least once, with console
  output or a saved trace as evidence.

**RESOLUTION (stakeholder decision 2026-07-12): DEFERRED, ticket closed.**
The robot was flashed with this sprint's firmware and bench-verified on the
stand (see `../bench-verification-results.md`): 5 of 6 mandatory gates PASS.
The FIRMWARE capability this script exercises is fully proven on hardware by
**gate 008** — a delayed `PoseFix` (exactly what the camera sends) converges
the fused pose (weighted Kalman update) and safely drops a stale-timestamp
fix — plus this script's own clock-sync/envelope/convergence math (60+11+33
host unit tests). The only unverified leg is the physical camera read: no
camera was connected (`aprilcam list_cameras` empty) and the robot was on
the bench stand, not `main-playfield`. The stakeholder deferred this
camera-in-the-loop demonstration ("other stuff in the works"); the script
`tests/playfield/pose_fix_convergence.py` is written and ready to run the
moment a camera is attached and the robot is placed on the playfield.

**For the team-lead**: `docs/protocol-v3.md`'s arm-7 table row/§8 note
remain stale (flagged again by tickets 004/008, still unaddressed) — this
ticket's own `pose_fix()`/`build_pose_fix_envelope()` additions to
`protocol.py` are new evidence the doc pass is now overdue at the sprint
level.
