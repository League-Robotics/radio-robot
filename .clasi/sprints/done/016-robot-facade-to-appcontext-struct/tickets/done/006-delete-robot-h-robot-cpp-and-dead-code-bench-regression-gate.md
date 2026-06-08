---
id: '006'
title: Delete Robot.h/Robot.cpp and dead code; bench regression gate
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
depends-on:
- '005'
github-issue: ''
issue: replace-robot-facade-with-appcontext-struct.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Delete Robot.h/Robot.cpp and dead code; bench regression gate

## Description

After T001–T005, no file outside `Robot.h`/`Robot.cpp` references `Robot`.
This ticket removes the old files and dead code, then runs the full bench
regression gate to confirm the refactor is behavior-neutral.

This is the final integration ticket. Its acceptance criteria include the
standing bench-verification gate (all drive verbs, sensors, telemetry on the
physical robot on the stand).

### Files to delete

- `source/robot/Robot.h`
- `source/robot/Robot.cpp`

Before deleting, run a final grep to confirm no references remain:
```
grep -r 'Robot' source/ --include='*.h' --include='*.cpp' | grep -v AppContext
```
Any remaining hit must be resolved before deletion.

### Dead code to remove (already absent from AppContext, but confirm Robot.cpp is gone)

The following were present in `Robot.h`/`Robot.cpp` only — they disappear with
the file deletion:
- `Robot::EncoderReading` struct
- `Robot::Pose` struct
- `Robot::controlCollect` (synchronous stub)
- `Robot::noteActivity`
- `Robot::setPose` / `Robot::getPose` (confirmed zero callers in T001 grep)
- `Robot::_gripperPresent` field
- `Robot::_lastOtosMs` / `Robot::kOtosSlowMs` constants

No source edits are needed to remove these — they vanish with the file. This
step is strictly file deletion + final grep verification.

### main.cpp cleanup

If any residual Robot-related includes or dead comments remain in main.cpp
from earlier tickets, remove them. Verify `main.cpp` includes `AppContext.h`
(not `Robot.h`).

### Bench regression gate (standing acceptance gate for this sprint)

After a clean firmware build and flash, verify using `uv run rogo ...`:

**Liveness preflight**:
- PING confirms robot responds.
- ID response: `caps=` lists `otos,line,color,gripper,portio` correctly.

**Drive verbs** (robot is on stand, safe to drive all commands):
- `S 200 200` — both wheels spin; `STOP` halts.
- `T 200 200 2000` — drives 2 s; `EVT done T` received.
- `D 200 200 300` — drives ~300 mm; `EVT done D` received; no spasm
  (encoder-reset workaround preserved).
- `G 100 0 200` — navigates; `EVT done G` received.
- `VW 200 0` — drives; keepalive lapses; `EVT safety_stop` fires.

**Gripper**:
- `GRIP 90` — servo moves to 90°.
- `GRIP` — returns `deg=90`.
- `GRIP 200` — clamped to 180°; `GRIP` returns `deg=180`.

**Zeroing**:
- `ZERO enc` — encoders reset; confirmed via `SNAP` enc=0,0.
- `ZERO pose` — pose reset; TLM shows `pose=0,0,0`.

**OTOS verbs** (with OTOS present):
- `OR` (reset tracking), `OP` (raw position), `OL`/`OA` (scalars).

**Port I/O**:
- `P 1` — reads port 1 digital.
- `PA 1` — reads port 1 analog.

**Telemetry**:
- `STREAM 40` while driving — clean TLM frames received.
- Stream goes silent after idle grace period.
- `SNAP` — returns one frame synchronously (stopped or driving).

**Encoder + sensor health** (standing gate):
- TLM shows encoders incrementing while driving.
- TLM `pose=` field updates (OTOS fusion active).
- Line/color data visible in TLM with appropriate fields.
- No encoder wedge during the bench exercise.

## Acceptance Criteria

**Build and test**:
- [x] `source/robot/Robot.h` and `source/robot/Robot.cpp` are deleted.
- [x] `grep -r 'class Robot' source/` returns zero results.
- [x] `grep -r '#include "Robot.h"' source/` returns zero results.
- [x] Clean build: `python3 build.py --clean <target>` passes (clean build
      mandatory before bench flash — see project rules).
- [x] Host unit tests pass: `uv run --with pytest python -m pytest`
      (1035 passed / 8 failed — matches pre-ticket baseline; no new failures).

**Bench regression gate** (STAKEHOLDER-DEFERRED — flash and run after all three sprints land):
- [ ] PING responds; `ID caps=otos,line,color,gripper,portio` correct.
- [ ] `S`/`T`/`D`/`G`/`VW`/`STOP` all produce correct responses and behavior.
- [ ] `D` command completes distance without spasm (encoder-reset workaround
      confirmed working).
- [ ] `GRIP` set and query work; angle is clamped and tracked.
- [ ] `ZERO enc` and `ZERO pose` reset correctly.
- [ ] OTOS verbs work (or `ERR nodev` if sensor absent).
- [ ] `P` / `PA` port commands work.
- [ ] `STREAM 40` produces TLM frames while driving; stream silences when idle.
- [ ] `SNAP` returns one frame synchronously.
- [ ] Encoders, wheels, and sensors all read on the bench (standing gate).
- [x] No `Robot` type references remain in the source tree (`grep -rn "Robot\.h\|class Robot\b\|Robot::" source/` returns zero results).

## Implementation Plan

**Approach**: Delete files; verify with grep; flash clean build; run bench gate.

**Files to delete**:
- `source/robot/Robot.h`
- `source/robot/Robot.cpp`

**Files to modify**:
- `source/main.cpp` — remove any residual Robot includes/comments if present

**Files NOT to touch**: `AppContext.h/.cpp`, `LoopScheduler`, `CommandProcessor`,
`WedgeTest`, HAL files.

**Build command**: `python3 build.py --clean <target>` (mandatory clean build
before bench flash per project rules; stale incremental builds flash broken
binaries).

**Bench tool**: `uv run rogo ...` (per project conventions; do not write
throwaway probe scripts).

**Testing plan**:
- `python3 build.py --clean <target>` — clean build.
- `uv run --with pytest python -m pytest` — host unit tests.
- Bench exercise via `uv run rogo` per the acceptance criteria checklist above.

**Documentation updates**: None required for this ticket. The sprint
architecture-update.md already documents the final AppContext design.
