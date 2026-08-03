---
status: pending
---

# radio_bench_gate's kFaultCommsMalformed stays-clear checks contradict the link's own inbound loss budget

Found during sprint 128's sprint-level bench gate run (2026-07-31, robot
`tovez` freshly flashed at `7898d2fc`, relay `zavaz`, channel 0 group 10
RAW250 power 7). Gate result: **31/35, FAIL** — every failure traces to
one mechanism:

- `kFaultCommsMalformed` (a latching fault bit) gets set at some point
  during the command-heavy relay session and never clears, failing both
  the fresh-connect check (SUC-008) and the stays-clear-entire-session
  check.
- The 0x0A repro's iteration 1/10 lost both acks once (9/10 passed;
  the 10/10 aggregate check therefore fails). Same mechanism: one
  corrupted/lost inbound line.

## Evidence it is NOT a firmware regression

- Fresh boot (pyOCD reset), direct USB, `tlmOn()`: 113 frames, **zero**
  malformed faults.
- Fresh boot, relay connect (`!GO` handshake) + `tlmOn()`: 135 frames,
  **zero** malformed faults — the relay handshake alone does not latch it.
- Everything substantive passes: HELLO/PING/ID/VER over the relay,
  move_wheels with climbing encoders, planned stop, estop measurably
  shortening travel (49.5 mm vs 750 mm commanded), enqueue+completion
  acks, outbound wire quality 99.96% clean (0 unparseable) over 120 s.
- Nothing in sprint 128 touched framing, parsing, UART, or relay code.

## The contradiction

Sprint 127 ticket 002's own notes document the relay path's
"**~20% inbound-line loss budget** (`radio_bench_gate.py`)". A latching
fault bit plus dozens of inbound command lines per session means the
stays-clear check is expected to fail at any nonzero inbound corruption
rate — the check contradicts the budget the same script defends. Either
the check should count/budget malformed frames per session (like the
wire-quality phase does) instead of testing a latch, or the latch needs
a documented clear path the gate exercises, or RF conditions have
degraded since sprint 124 measured this green and that degradation is
the real finding.

## Acceptance

- Decide the intended semantics (budgeted count vs latch) with the
  stakeholder; align the gate and SUC-007/008 wording.
- If RF degradation is suspected instead, characterize inbound loss
  directly (host→robot direction) and record the number.
