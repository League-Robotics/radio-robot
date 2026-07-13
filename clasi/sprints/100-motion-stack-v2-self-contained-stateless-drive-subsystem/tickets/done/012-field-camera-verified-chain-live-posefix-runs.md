---
id: '012'
title: 'Field: camera-verified chain + live PoseFix runs'
status: done
use-cases: [SUC-014]
depends-on: ['011']
github-issue: ''
issue: motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Field: camera-verified chain + live PoseFix runs

## Preconditions

Robot USB-attached beforehand to flash the bench-accepted build (ticket
011's sign-off); robot then moved to the camera-covered, geofenced
playfield (untethered) for this ticket's actual test — a distinct HITL
context from tickets 007/008/011's stand-based work. Read
`.clasi/knowledge/vision-geofence-before-driving.md` and
`.clasi/knowledge/playfield-not-floor.md` before driving — geofence
checked BEFORE driving, never assumed.

## Description

A playfield chain script (the `playfield_camera_run.py` pattern)
commands a multi-segment world-frame chain with live `PoseFix`
corrections mid-chain, demonstrating the full camera -> EKF -> tracker
loop closing end to end — the only tier where that full loop actually
closes.

## Acceptance Criteria

- [ ] A playfield chain script exists (`tests/playfield/`, per
      `tests/CLAUDE.md`'s three-domain split — HITL, not pytest-
      collected) commanding a multi-segment world-frame chain.
- [ ] The camera (aprilcam) observes the robot's true pose and sends
      `PoseFix` corrections mid-chain (sprint 099's mechanism, consumed
      via `bb.poseStepped` -> `StepInput.poseStep` per ticket 007's
      adapter wiring).
- [ ] A camera-verified chain completes without leaving the geofenced
      playfield area.
- [ ] The plan-vs-actual overlay (`RefState` polyline vs. fused pose vs.
      camera ground truth) shows the fused/camera traces converging, not
      diverging, across at least one mid-chain `PoseFix`.
- [ ] Results (overlay plots, run logs) committed under `tests/
      notebooks/out/` and referenced in completion notes.

## Testing

- **Existing tests to run**: none beyond confirming ticket 011's bench
  sign-off is still valid on this build.
- **New tests to write**: the playfield chain script itself (HITL, not
  pytest-collected).
- **Verification command**: `uv run pytest` (host-side regression check
  only — this ticket's real verification is the field session itself).

## Implementation Plan

**Approach**: follow the existing `playfield_camera_run.py` pattern
(aprilcam MCP/daemon API, `NezhaProtocol` for the robot side — never raw
`pyserial`, per prior bench-session lessons). This ticket does NOT
modify `source/drive/` or the adapter (same posture as ticket 011) — a
field-discovered defect reopens the relevant earlier ticket.

**Files to create**: `tests/playfield/` script (new, or adapted from the
existing parked `playfield_camera_run.py` — per `tests/CLAUDE.md`'s note
that playfield scripts are currently parked pending motion/odometry
restoration; sprint 099 plus this sprint together restore what they
need).

**Testing plan**: the field session itself.

**Documentation updates**: none beyond the committed run artifacts.

---
## DEFERRED to sprint 101 (2026-07-13)

Not completed in sprint 100. Bench diagnosis found the DeviceBus firmware's
**heading feedback is broken** (raw OTOS heading frozen during an open-loop
spin; fused heading garbage/resetting; OTOS re-init commands accepted-inert),
so closed-loop turn accuracy cannot be validated or tuned until that is fixed.
That debugging — and the arc/turn accuracy sweeps, camera-verified field runs,
and the parked-file cleanup that depends on field sign-off — is re-scoped into
sprint 101 (debugging). Carried forward, superseded by 101's tickets.
