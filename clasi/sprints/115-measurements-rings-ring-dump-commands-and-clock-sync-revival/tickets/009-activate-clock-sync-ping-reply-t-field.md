---
id: 009
title: 'Activate clock sync: PING reply t= field'
status: open
use-cases:
- SUC-115-003
depends-on: []
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Activate clock sync: PING reply t= field

## Description

Independent of every other ticket in this sprint (no dependency).
`src/firm/app/comms.cpp`'s `PING` handler currently replies exactly
`"OK pong"`. `src/host/robot_radio/robot/clock_sync.py` — a complete,
already-unit-tested NTP-style min-RTT host clock estimator — has been
waiting on the firmware side of this exact change: `_parse_pong_t()`
already parses `"OK pong t=<n>"` and every downstream method
(`to_host_time()`/`to_robot_time()`) is implemented and tested against
that exact reply shape. This ticket is the one-line firmware change that
activates it.

## Implementation Plan

- **Approach**: in `comms.cpp`'s `PING` handler (`t.sendReliable("OK
  pong")`), append `" t=<robot ms>"` using the loop's own current time in
  milliseconds (the same clock source `RobotLoop` already reads each
  cycle, converted to `[ms]` if the handler doesn't already have it in
  that unit — do not add a second clock read path). Reply shape must be
  EXACTLY `"OK pong t=<n>"` (space before `t=`, no trailing space/text)
  to match `_parse_pong_t()`'s existing parsing exactly — verify against
  that function's implementation before finalizing the format string.
- **Files to modify**: `src/firm/app/comms.cpp`.
- **Testing plan**: a sim/unit test confirming the `PING` reply matches
  the `"OK pong t=<n>"` pattern; a bench exercise running
  `clock_sync.py`'s `ping_burst()` over live serial and confirming it
  converges to a stable offset/skew estimate (this sprint's own gate:
  "clock sync converges over serial").
- **Documentation updates**: none — `clock_sync.py`'s own docstring
  already documents the full protocol this activates; no firmware-side
  doc exists to update beyond `comms.cpp`'s own reply-format comment (if
  any).

## Acceptance Criteria

- [ ] `PING` reply is exactly `"OK pong t=<n>"` where `<n>` is the
      robot's own clock in milliseconds.
- [ ] `clock_sync.py`'s `_parse_pong_t()` parses the live reply
      successfully with no host-side code change required.
- [ ] A ping burst over live serial on the bench converges to a stable
      offset estimate (bounded by ~½ the minimum observed RTT, per the
      module's own documented accuracy bound).
- [ ] No other `comms.cpp` reply/dispatch behavior changes — `HELLO` and
      every armored command handler are unaffected.

## Testing

- **Existing tests to run**: any existing `comms.cpp`/`PING` sim-unit
  test; `src/host/robot_radio/robot/clock_sync.py`'s own existing unit
  tests (must still pass unchanged — this ticket touches firmware only);
  full `uv run python -m pytest` suite; `just build-clean`.
- **New tests to write**: a sim/unit test asserting the exact `PING`
  reply format.
- **Verification command**: `uv run pytest`
