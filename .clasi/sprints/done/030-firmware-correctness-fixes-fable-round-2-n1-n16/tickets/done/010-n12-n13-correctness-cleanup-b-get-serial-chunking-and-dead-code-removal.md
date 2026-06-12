---
id: '010'
title: "N12+N13: Correctness cleanup B \u2014 GET serial chunking and dead code removal"
status: done
use-cases:
- SUC-009
depends-on:
- 009
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# N12+N13: Correctness cleanup B — GET serial chunking and dead code removal

## Description

**N12 (Low-Med, bench-gated):** The `GET` / `CFG` dump builds into a 768-byte buffer
(`ConfigRegistry.cpp:165`) and realistically produces 600-800 bytes. CODAL's serial TX
buffer is 255 bytes (`SerialPort.cpp:17`, with a comment that bursts must fit).
`sendReliable`'s wait cannot make room for a line longer than the buffer — it spins
5 ms then hands the whole string to ASYNC, which drops the overflow. Bare `GET` over
serial may be truncated mid-keys.

**This ticket is bench-gated**: confirm truncation on hardware before implementing
chunking. If the bench test shows the full config is received correctly (e.g. ASYNC
drains before the next line), the chunking is still a defensive improvement but the
acceptance criterion is met either way.

**N13 (Low):** Residual dead/vestigial code:
- `RatioPidController` — constructed, reset, SET-tunable via `pid.*` keys, but its
  `update()` never runs in `controlTick` (sync-gain coupling replaced it).
- `PID_BYPASS` macro (`MotorController.cpp:12`) — unused.
- `Odometry::update()` — deprecated, no callers.
- `DriveMode::TIMED` — unreachable (T runs as VELOCITY); TLM `mode=` can never
  read `T`. Check host parsers don't expect it before removing.

Depends on ticket 009 to complete the cleanup cluster together.

## Acceptance Criteria

- [x] N12 bench step: Buffer math confirmed: 58 keys × ~14 bytes/key = ~805 bytes,
      exceeding the 255-byte CODAL TX limit by ~550 bytes. Chunking implemented
      defensively. BENCH CONFIRM NEEDED: stakeholder/team-lead verifies on hardware
      that all 58 keys are received after this change.
- [x] N12 implementation: chunked CFG dump into multiple serial writes of ≤ 200 content
      bytes each (kCfgChunkMax=200). Host get_config() already accumulates multiple
      CFG lines via result.update(). Sim send_command() buffer increased to 2048 bytes
      to match ReplyStore capacity (was 512, truncated multi-line replies in tests).
- [x] N13: `RatioPidController` member `_pid` and its construction removed from
      `MotorController`. `_pid.reset()` calls removed from startDriveClean, stop,
      resetIntegrators, startDrive. `updatePidGains()` method removed.
      pid.* keys RETAINED in ConfigRegistry and RobotConfig (host tests use them).
      The `updatePidGains` call removed from handleSet; pidChanged tracking removed.
- [x] N13: `PID_BYPASS` macro removed from `MotorController.cpp` (was always 0).
- [x] N13: `Odometry::update()` removed (declaration from .h, definition from .cpp).
- [x] N13: `DriveMode::TIMED` removed from enum (Config.h); modeChar='T' case removed
      from Robot.cpp TLM emitter. Grep result: host/tests/test_protocol_v2.py has
      test_parse_tlm_mode_field_T which tests the *parser* with a synthetic string —
      it does NOT require firmware to emit mode=T. Parser unchanged; test passes.
- [x] `python3 build.py` clean build passes. FLASH: 184128 B (49.40%, 184784 text).
      No new warnings.
- [x] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes.
      684 passed (674 baseline + 10 new N12/N13 tests).

## Implementation Plan

### Approach

N12: Measure first (bench), then chunk if needed. Chunking approach: iterate over the
config registry key-value pairs and flush a new `CFG` line every ~180 chars rather
than building the entire dump into one buffer.

N13: Delete the identified dead code one piece at a time and verify the build after
each deletion to catch any surprising dependents.

### Files to modify

- `source/config/ConfigRegistry.cpp`
  - N12: chunk the `CFG` dump output if bench confirms truncation.
- `source/motor/MotorController.cpp` (and `.h`)
  - N13: remove `RatioPidController` member, construction, reset, and all `pid.*`
    SET wiring; remove `PID_BYPASS`.
- `source/odometry/Odometry.cpp` (and `.h`)
  - N13: remove `update()` (declaration + definition).
- `source/control/MotionController.cpp` (or wherever `DriveMode::TIMED` appears)
  - N13: remove `TIMED` case and enum value; confirm no switch case handles it.
- `source/control/` (or wherever `DriveMode` is defined)
  - N13: remove `TIMED` from the `DriveMode` enum.

### Bench verification step (N12)

Use `uv run rogo get` or a direct serial session to issue `GET` and capture the full
response. Compare the number of keys returned against the expected count. The
team-lead / stakeholder runs this step with the robot on the bench.

### Testing plan

Run: `uv run --with pytest python -m pytest host_tests/ host/tests/ -v`

Build: `python3 build.py` (clean).

### Notes

- `pid.*` SET keys will return an unknown-key ERR after removal. No known host
  scripts use them (OQ-1 in architecture-update.md). Programmer should grep
  `host/` for `pid.` SET usage before deleting.
- N12 is only a chunking change if the bench confirms truncation. The ticket
  completes regardless — the bench step is explicit and documented here.
- `completes_issue` for `fr2-n11-16-cleanup.md` is handled by ticket 009; this
  ticket has no linked issue (N12/N13 were included in that issue file but this
  ticket is the execution unit for them).
