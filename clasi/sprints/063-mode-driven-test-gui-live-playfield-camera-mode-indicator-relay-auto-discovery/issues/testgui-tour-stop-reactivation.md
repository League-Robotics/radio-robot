---
status: in-progress
sprint: '063'
tickets:
- 063-007
---

# TestGUI: Tour button does not reactivate after Stop; add an obvious Tour Stop control

## Symptom

In playfield (Relay) mode, starting a Tour and then stopping it leaves the
**"Tour 1"** button permanently disabled until the next reconnect. There is
also no obvious, dedicated way to stop a running tour — only the shared
Operations **STOP** button cancels it.

## Root cause (confirmed by code reading)

Stop routes: STOP button → `_stop_all_motion` → `_stop_tour`.

`_stop_tour` calls `worker.stop()`, `thread.quit()`, `thread.wait(3000)`, then
nulls `tour_worker` / `tour_thread` / `tour_bridge` — but it **never re-enables
the tour buttons**. Re-enabling lives only in `_on_tour_finished`, which is
unreliable after an explicit stop:

1. The main thread is blocked in `thread.wait()` at the moment the worker emits
   its `finished` signal, so the queued slot can't run until after wait returns.
2. `_stop_tour` then sets `_state["tour_bridge"] = None`, dropping the only
   reference to the `_WorkerBridge` that receives `finished`; the pending queued
   `finished` → `bridge.on_finished` call is discarded (receiver gone).

Net effect: `_on_tour_finished` (the only place that re-enables the buttons)
never runs → the Tour button stays disabled.

## Desired behavior

- A visible, explicit **Stop** control for a running tour (not just the shared
  STOP).
- Stopping a tour must reliably re-enable the Tour button(s) while still
  connected. The simplest fix is to re-enable synchronously inside `_stop_tour`
  (after the join) when a transport is still present, rather than relying on the
  `finished` callback.

## Affected code

- `host/robot_radio/testgui/__main__.py` — `_stop_tour`, `_on_tour_finished`,
  `_stop_all_motion`, tour button creation/wiring.

## Notes

Introduced with the Tour feature (out-of-process work on 2026-07-01). The same
stop/re-enable weakness exists for the GOTO worker (`_stop_goto` /
`_on_goto_finished`) and should be fixed together.
