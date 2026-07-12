---
id: '009'
title: 'aprilcam end-to-end bench/playfield script: PING sync, FIX send, convergence'
status: open
use-cases: [SUC-005]
depends-on: ['007', '008']
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

- [ ] A new script (`tests/bench/` if it only needs a static/known robot
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
- [ ] Geofence/hop-test precondition honored: confirm the robot is inside
      the calibrated playfield bounds and do a small, safe motion test
      before any extended run (per the vision-geofence-before-driving
      convention).
- [ ] Script follows `tests/CLAUDE.md`'s HITL conventions: widen the
      serial-silence watchdog if applicable, restore it and send `STOP`/
      binary `stop` in a `finally` block — motors never left running on an
      exception or Ctrl-C.
- [ ] **BENCH/PLAYFIELD MANDATORY**: run the script against the real
      robot; it demonstrates the full path (PING clock-sync -> tag-pose-
      to-FIX send -> convergence check) successfully at least once,
      recorded (console output or a saved trace) as this ticket's
      completion evidence.
- [ ] Script is resilient to a single dropped/late camera frame (does not
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
