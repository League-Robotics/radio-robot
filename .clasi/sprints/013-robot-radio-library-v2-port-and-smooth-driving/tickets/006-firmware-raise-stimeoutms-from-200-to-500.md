---
id: '006'
title: 'Firmware: raise sTimeoutMs from 200 to 500'
status: open
use-cases:
  - SUC-004
  - SUC-008
depends-on: []
github-issue: ''
issue: ''
completes_issue: false
---

# Firmware: raise sTimeoutMs from 200 to 500

## Description

The root cause of herky-jerky driving is the `S`/`VW` streaming watchdog default of 200 ms in `source/types/Config.h`. Over the laggy RADIORELAY, the host often cannot refresh the `S` command within 200 ms, causing the firmware to cut motors (`EVT safety_stop`) → the robot jerks to a stop → the host restarts the drive command → jerk again.

This ticket makes a single targeted firmware change: raise `sTimeoutMs` from `200` to `500` in `source/types/Config.h`. Then verify:

1. `source/control/DriveController.cpp` reads `_config.sTimeoutMs` (not a hardcoded `200`).
2. A clean firmware build succeeds.
3. The stakeholder reflashes the robot and confirms `GET sTimeout` returns `500`.

This ticket has no dependency on the host library tickets (T001–T005) and can be worked in parallel with them. However, bench verification of smooth driving (T008) depends on both this ticket and T003.

**Build + flash instructions** (reminder):
- Build: `mbdeploy build --clean` (always clean — stale incremental builds can flash broken binaries).
- Flash: mass-storage copy to the robot's USB drive.
- Verify: connect RADIORELAY, run `GET sTimeout`, confirm `CFG sTimeout=500`.

## Acceptance Criteria

- [ ] `source/types/Config.h` diff shows `sTimeoutMs` changed from `200` to `500`.
- [ ] Code review of `source/control/DriveController.cpp` confirms the streaming watchdog check uses `_config.sTimeoutMs` (not a literal `200` or `200u`).
- [ ] Firmware builds clean: `mbdeploy build --clean` exits 0 with no warnings introduced.
- [ ] (Bench — stakeholder) `GET sTimeout` returns `500` on the robot after reflash.

## Implementation Plan

**Approach**: Single-line edit; then read `DriveController.cpp` to confirm `sTimeoutMs` usage.

**Files to modify**:
- `source/types/Config.h` — change `sTimeoutMs = 200` to `sTimeoutMs = 500`.

**Files to read (verify only, do not modify unless watchdog is hardcoded)**:
- `source/control/DriveController.cpp` — confirm watchdog check reads `_config.sTimeoutMs`.

**Testing plan**:
- No host-side unit tests apply to this change.
- If any firmware unit test mocks `sTimeoutMs` and hard-codes `200`, update that test to `500`.
- Run `uv run --with pytest python -m pytest host/tests` to confirm host tests still pass (should be unchanged).
- Bench verification deferred to T008.

**Documentation**: No README change needed — T008 covers bench procedure docs.
