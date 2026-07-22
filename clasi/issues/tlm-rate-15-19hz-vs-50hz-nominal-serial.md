---
status: pending
---

# Telemetry reaches ~15-19 Hz on serial vs the 50 Hz (every-cycle) nominal

## Description

The frame-v2 telemetry is emitted every 20 ms loop cycle by design, but
measured arrival at the host over USB CDC serial is ~15.4-19 Hz (measured
twice: sprint 116 gate, post-close verification 2026-07-22). No frame
LOSS signature (seq gaps ~none, drop rate 0.01% in the 600 s soak) — the
emitter appears throttled, not lossy.

## Cause

Not diagnosed. Candidates: serial write backpressure in the loop's emit
path (armored line ~207 B x 50 Hz ~= 10.4 KB/s — near 115200-baud
practical throughput with overhead), Telemetry::emit's own pacing
interacting with kPrimaryPeriod=kCycle, or host-side read batching.

## Proposed fix

Diagnose before fixing: instrument emit-side counters vs host arrivals;
check the CDC baud/config; if bandwidth-bound, either accept (~19 Hz is
still 2.5x the pre-gut 40 ms cadence... it is not; pre-gut was 25 Hz —
~19 Hz is BELOW it) or shrink the frame further / raise baud. Matters for
the estimator dataset density (one-step-ahead analysis quality scales
with sample rate).

## Related

- docs/bench-checklists/sprint-116-move-protocol.md, sprint-117-estimator-v1.md — measurements.
