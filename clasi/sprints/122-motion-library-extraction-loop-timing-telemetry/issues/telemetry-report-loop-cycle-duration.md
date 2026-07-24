---
status: in-progress
filed: 2026-07-24
filed_by: team-lead (stakeholder request)
related:
- firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
tickets:
- 122-003
- 122-004
sprint: '122'
---

# Telemetry: report how long the loop took to run

## Description (stakeholder request, 2026-07-24)

Each telemetry frame should carry the loop's own timing — "basically
`clock.now() - now`": the elapsed time since the cycle-start timestamp,
measured where the frame is staged. Two fields, both cheap, both diagnostic
gold on hardware:

- `cycle_busy` — [us] elapsed from `cycleStart` to the end of the cycle's
  WORK (measured just before the pace sleep, or at frame staging — state
  which in the field comment). This is "how long the loop took to run":
  I2C stalls, OTOS retries, and comms bursts show up here directly.
- `cycle_period` — [us] this `cycleStart` minus the previous `cycleStart`.
  The actual achieved period vs the nominal kCycle=40 ms: overruns and
  scheduling jitter show up here, and it is the true dt a host-side
  consumer should use for rate math instead of assuming 40 ms.

## Why it earns its bytes

- Base principle "the base never lies and never hides latency" — timing is
  the last hidden quantity in the frame.
- The tlm-rate question (`later/tlm-rate-15-19hz-vs-50hz-nominal-serial.md`)
  becomes answerable from the frame itself: emit-side period is now visible,
  so host-arrival shortfalls separate cleanly into firmware pacing vs link
  throughput.
- Bench sessions get overrun detection for free (a `cycle_busy` approaching
  `cycle_period` is the early warning the I2C safety-net bit was groping
  toward).

## What to do

- Add the two `uint32 [us]` fields to the telemetry proto + `Telemetry::
  Frame` staging (base-side, `robot_loop.cpp` already holds `cycleStart`);
  regenerate codecs; expose on `TLMFrame` host-side.
- Display in the TestGUI telemetry panel (one line: `loop 3.2ms / 40.0ms`);
  optionally a strip-chart tab later, not required here.
- Sim: deterministic values (virtual clock) — assert exact expected numbers
  in one unit test; hardware: eyeball-verify plausible values on the next
  bench session, no dedicated stand time needed.

## Acceptance

- Every frame carries both fields; sim test asserts exact values under the
  virtual clock; GUI shows them; wire change is backward-compatible
  (proto field addition only); no measurable emit-cost regression.
