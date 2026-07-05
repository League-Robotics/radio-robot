---
status: pending
---

# Flip-flop per-motor sample cadence is ~half the design target (~44–52 Hz vs ~80–90 Hz)

## Context

Sprint 079 replaced the fused blocking encoder read with the split-phase
brick flip-flop (REQUEST_DUE / COLLECT_DUE per bus slice, lazy per-device
clearance). The tick-model design sketch
(`clasi/sprints/done/079-.../` issues) estimated a per-motor sample period of
**~11–13 ms (~80–90 Hz)** with 2 ports in use.

## Measured (079-006 stand gate, clean build, 2026-07-05)

On the real robot, 2 ports in use, closed-loop motion:

- Port 1: median inter-sample **~19.07 ms (~52 Hz)**
- Port 2: median inter-sample **~22.54 ms (~44 Hz)**

Roughly **half** the design estimate. Functionally fine — the embedded PID
closes the loop cleanly at this cadence (VEL 150 converges within ~6%, no
oscillation, `vel_filt_alpha=0.3` confirmed adequate), so this is a
performance/throughput note, not a correctness bug. Recorded honestly rather
than rounded to target.

## Likely cause (hypothesis, not yet confirmed)

The 079-006 TWIM-stall fix added real `preClear=4000`/`postClear=4000` `// [us]`
clearance around the 0x46 request and the 0x60 duty write. The design's cadence
estimate predated that fix and assumed the flip-flop's own scheduling gap would
supply the settle time "for free." The mandatory ≥4 ms clearances plus the
COLLECT_DUE spin-the-remainder likely add real per-slice time the estimate
didn't budget. This wants measurement (per-phase timing via the I2CBus stats or
a scoped gdb/pyOCD pass) before any change.

## Possible directions (choose after measuring)

- Overlap other-device traffic (OTOS/line/color) into the settle windows once
  those devices join the HAL scheduler (design Case 5 — currently out of scope).
- Revisit whether both the request's `preClear` and the paired collect's spin
  are each paying the full 4 ms, or double-counting.
- Accept ~50 Hz as the real budget and correct the design doc's estimate.

## Non-goals

- Do NOT regress the TWIM-stall fix or the reversal-latch armor to chase Hz.
- Not urgent — closed-loop control is correct at the current cadence.
