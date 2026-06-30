---
id: '006'
title: Bench smoke verification (safe on-stand)
status: open
use-cases:
- SUC-006
depends-on:
- 059-005
github-issue: ''
issue: message-based-subsystem-architecture.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench smoke verification (safe on-stand)

## Description

Flash the sprint-059 firmware to the tovez differential dev bot (on its bench stand)
and confirm that the new ordered-tick path behaves correctly on real hardware.

**SAFETY CONSTRAINT (project rule):** NEVER blind-drive on the playfield. On-stand
spins are exempt from the playfield/geofence rule. This ticket is limited to:
- Connectivity and HELLO/PING response
- TLM stream and SNAP telemetry correctness
- One safe on-stand rotation (RT command or TURN) to verify motion + telemetry parity

No field driving. No free-floor driving. The robot stays on the bench stand throughout.

**NOTE:** The team-lead (or a human operator) should oversee or execute the actual
bench step. The programmer agent produces the `build.py --clean` artifact and the
bench script; the bench run itself requires a human with the physical robot.

The bench script should be placed in `tests/bench/` following the pattern of
existing bench scripts.

## Acceptance Criteria

- [ ] `python build.py --clean` produces zero errors and a valid `MICROBIT.hex`
  (verify hex contains the sprint-059 ordered-tick code, not a stale incremental
  build — per the project knowledge note: always `--clean` before HITL flash).
- [ ] Flash the hex to the tovez robot.
- [ ] Verify HELLO/PING response over serial: `HELLO` returns the firmware banner;
  `PING` returns `OK PONG`.
- [ ] Verify TLM stream: `STREAM 100` starts periodic TLM frames; spot-check that
  `pose.x/y/h` and `drive.vx/vl/vr` fields are present and non-garbage.
- [ ] Verify SNAP: `SNAP` returns a one-shot TLM frame with correct field layout.
- [ ] Verify on-stand rotation: send an RT command (e.g. `RT 3600` = 360°) or TURN;
  robot spins in place; TLM heading field changes proportionally; EVT done is received.
- [ ] Verify safe stop: `X` (cancel) stops motion immediately; TLM shows near-zero velocity.
- [ ] Verify telemetry parity: the TLM frame format (field names, order, encoding)
  matches the pre-sprint baseline recorded in `test_golden_tlm.py`. If the format
  changed, update `test_golden_tlm.py` and document the change.
- [ ] No unexpected reboots, panics, or watchdog resets during the bench run.
- [ ] `uv run python -m pytest -x --tb=short -q` passes at 2380/2 plus sprint tests
  (final sweep after bench confirms no regressions from any last-minute fixes).

## Implementation Plan

### Approach

**Build**:
```bash
python build.py --clean
```
Verify the `MICROBIT.hex` is newly built (check modification time; do not flash a
stale build — this is a known project pitfall).

**Flash**: Use the robot's standard flashing procedure to put the hex on the micro:bit.
Match `active_robot.json` to the physical bot (tovez = differential, Tovez firmware).

**Bench script** (`tests/bench/059_smoke.py`):
```
connect to serial port (DTR asserted, !GO handshake per relay protocol in .clasi/knowledge/)
send HELLO → assert "radio-robot" in response
send PING → assert "PONG" in response
send STREAM 100 → collect 5 TLM frames → assert pose fields present
send SNAP → assert one TLM frame received
send X → assert robot is idle
send RT 1800 → assert EVT done RT received within 5 seconds (180° spin)
send SNAP → assert heading changed by ~π rad (within 20° tolerance)
send X → stop
```

**Parity check**: Compare SNAP TLM field layout against the golden frame in
`test_golden_tlm.py`. If format diverged, update the golden and document in the
commit message.

**If the ordered-tick path was feature-flagged in ticket 005**: run the bench with
`USE_ORDERED_TICK` enabled (rebuild). The bench smoke is the real-hardware validation
of the flag.

### Files to Create

- `tests/bench/059_smoke.py` — bench smoke script

### Files to Modify

- `test_golden_tlm.py` — update golden frame if TLM format changed (document why)

### Testing Plan

This ticket's test is the bench run itself. The programmer produces the artifacts;
the team-lead runs the bench.

```bash
# Final sim sweep (must pass before bench)
uv run python -m pytest -x --tb=short -q

# Device build
python build.py --clean

# Bench (team-lead runs this with the physical robot)
uv run python tests/bench/059_smoke.py
```

### Documentation Updates

None required beyond the bench script itself. If the TLM format changed, document
in `test_golden_tlm.py` and in the commit message.
