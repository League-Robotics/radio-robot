---
status: pending
priority: high
---

# A worker draining the telemetry queue starved the GUI's traces and graphs

Stakeholder, 2026-07-31: *"I'm not getting my traces until the move is
finished, and my graphs are wrong."*

## Cause

The host-side unmanaged drive loop needed encoder feedback and called
`read_binary_tlm_frames()` — which **drains the same queue the GUI's
`on_telemetry` is fed from**. The background worker consumed every frame before
the traces or graphs could see it, so the display stayed dead for the whole move
and then updated all at once.

Nothing errors when this happens. The graphs simply go blank, which reads as a
telemetry problem rather than as contention.

## Fix

- Both transports cache every delivered frame on `self.latest_frame` in
  `_deliver_tlm()` before invoking `on_telemetry` — a **non-consuming** read for
  observers.
- The drive loop polls `latest_frame` (`_observe()`) instead of draining. It
  watches the same stream the GUI watches rather than competing with it.

Measured on the bench, frames delivered to `on_telemetry` at 1 s intervals
during a 300 mm move: `[14, 37, 51, ...]` — previously zero until completion.

## Follow-up

Make this structurally impossible rather than a convention. Options: a
fan-out/broadcast on the telemetry reader so every consumer gets its own view,
or an explicit `observe()` API with `read_*` marked as an exclusive drain that
warns when called while a GUI consumer is attached. Right now the only thing
preventing a recurrence is a comment.

Also note the managed path calls `suspend_telemetry_reader()` during tours,
which is a *separate* consumption story and has not been checked for the same
symptom.

Related: [[unmanaged-drive-lease-expiry-and-terminal-pivot]],
[[trace-baselines-only-refreshed-while-appending]]
