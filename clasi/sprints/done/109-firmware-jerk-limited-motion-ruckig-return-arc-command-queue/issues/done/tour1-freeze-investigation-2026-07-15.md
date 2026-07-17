---
status: done
sprint: '109'
tickets:
- 109-008
---

# Tour 1 "died/froze" after leg 1 — investigation record (2026-07-15 emergency fix)

Stakeholder reported clicking Tour 1 in the TestGUI and it died/froze right after
leg 1, at the straight->turn boundary, while running on the playfield. This was an
out-of-process emergency dispatch (three fixes, master directly, no sprint) — this
issue is the honest record of the freeze investigation specifically (fix 3 of that
dispatch), since the other two fixes (planner defaults, log-flood) each have their
own commit.

## Root-cause verdict: NOT a Qt-threading defect — found and fixed elsewhere

Investigated `_TourRunner._on_row` -> `transport.on_telemetry` (`testgui/__main__.py`
~line 1617-1630) calling `_on_telemetry_thread_v2` (~line 1750) directly and
synchronously from the TOUR worker thread, per the project memory
`pyside-queuedconnection-bare-function.md` ("queued signal -> bare function executes
on the EMITTING thread").

Audited every cross-thread `.connect(..., QueuedConnection)` in `__main__.py`
(`_rx_bridge.rx_line`, `_bridge.frame_ready`, `_bridge.truth_ready`,
`worker.log_line`, `worker.finished` — both `_TourRunner` and `_GotoRunner`): every
one targets a **bound method on a `QObject` instance** (`_bridge.on_frame_ready`,
`bridge.on_log`, etc.), never a bare function or lambda. `_on_telemetry_thread_v2`
itself (the function actually assigned to `transport.on_telemetry`) does no widget
work at all — it queues the frame into a `queue.Queue` and calls
`bridge.frame_ready.emit()`; all widget touching happens inside `on_frame_ready`,
the `@Slot()`-decorated bound method the signal is queued to.

Empirically reproduced this exact shape in isolation (background `threading.Thread`
calling `.emit()` on a `QObject.Signal` connected via explicit
`Qt.ConnectionType.QueuedConnection` to a bound method) — confirmed the slot body
does NOT run until the main thread's event loop is pumped (`QApplication.
processEvents()`), and when it runs, it runs on the main thread, never the calling
worker thread. See `tests/testgui/test_telemetry_bridge_threading.py` (added this
dispatch) for the reproduction and its own docstring for the full writeup,
including a note that the memory-note gotcha did not reproduce for a bound-method
QueuedConnection in the PySide6 version this repo pins — it most likely applies to
a lambda/partial slot with no `QObject` receiver at all, a shape that does not
appear anywhere in this call chain.

Also audited `planner/executor.py`'s `begin()`/`tick()` for a block-forever path a
fault could trigger: `begin()`'s telemetry drain retry is bounded
(`_BEGIN_DRAIN_RETRIES`), `tick()`'s own frame read is a non-blocking poll. No
unbounded wait found on the tour worker thread either.

## What actually explains the report

The stakeholder's own description — "died/froze right after leg 1, the
straight->turn boundary" — matches, almost exactly, 107-005's own bench finding
(`clasi/sprints/done/107-testgui-revival-tours-execute-and-close/tickets/done/
005-bench-tour-runs-trace-capture.md`, "Bench findings" #1): a real
`kFaultWedgeLatch` firmware fault reproducibly trips at the straight->turn boundary
with `tour.py`'s `DEFAULT_INTER_LEG_SETTLE=0.3s` (fixed this same dispatch to
1.0s, the bench-proven value — `host/robot_radio/planner/tour.py`). `run_tour()`'s
own "stop immediately, no further legs attempted" contract means the robot simply
stops driving the instant that fault fires — a robot that goes suddenly and
completely still mid-tour reads as "died" from the operator's chair, with no Qt
freeze required to explain it.

Separately, a real log-flood bug was found and fixed this same dispatch
(`testgui/binary_bridge.py`'s `render_log_line()` misrendering every bare
`TelemetrySecondary` frame as `corr_id: N` at ~4 lines/s) — this would have made
the message monitor feel sluggish/unresponsive around the same time, plausibly
compounding the "froze" impression even though it is not a deadlock either.

## Resolution (109-008, 2026-07-17): verified against the new MOVE-queue path

This ticket's acceptance criterion #5 required confirming the specific
failure mode above (`kFaultWedgeLatch` at a straight->turn boundary, plus the
old `DEFAULT_INTER_LEG_SETTLE=0.3s` timing) cannot recur on the new
`Motion::Executor`/`Move`-queue path (sprint 109 tickets 003-006), and
closing this issue with that verdict.

**Structural argument.** The freeze symptom traced above was never a real
Qt deadlock — it was `run_tour()`'s OLD `StreamingExecutor.tick()` polling
raw `Telemetry.fault_bits` every ~150ms cadence tick and stopping the WHOLE
tour the instant ANY bit was nonzero, including a transient, self-recovered
blip (`Devices::MotorArmor`'s own wedge-latch detector can assert briefly at
a stop/reversal boundary and clear on its own without the drivetrain ever
actually wedging). The new host-side `run_tour()` (`host/robot_radio/
planner/tour.py`, this ticket) has NO raw fault-bit polling of any kind: a
leg's own outcome is driven ENTIRELY by that leg's `Move` command reaching
its own terminal `AckStatus` (`DONE`/`TRIVIAL`/`SUPERSEDED`/`FLUSHED`/
`TIMEOUT`/`SOLVE_FAIL`, `Motion::Executor`'s own per-command taxonomy,
telemetry.proto's ack ring) — a transient fault bit that firmware's own
`MotorArmor` recovers from without aborting the active command has no wire
path left to stop the tour at all. This eliminates the entire class of
symptom the investigation above describes, by construction, independent of
whether a wedge-latch condition happens to fire in any given run.

**Empirical verification.** `src/tests/testgui/test_sim_transport_tour1.py`'s
`test_tour_1_runs_to_completion_with_finite_small_closure` runs the REAL
compiled firmware simulator (`libfirmware_host`) through the NEW
`run_tour()` for the full 13-leg `TOUR_1` (6 straight->turn/turn->straight
boundary crossings — the exact shape the original 2026-07-15 report
described) end to end, asserting every leg reaches `RunOutcome.COMPLETED`
(no `FAULT`, no timeout) and a finite closure. This test passed on its first
attempt during this ticket's own verification run (no fault-triggered
retries needed). `DEFAULT_INTER_LEG_SETTLE` (the other half of the original
fix) is now vestigial (`tour.py`'s own file header): the MOVE-queue path has
no host-timed gap between two QUEUED legs at all — firmware's own boundary-
velocity carry (ticket 006) sequences the transition instead.

**Verdict: RESOLVED.** The freeze symptom's own root cause (host-side
"stop the tour on any fault bit" polling) does not exist on the new path,
and the new path's own decisive test (TOUR_1 end to end, sim) demonstrates
the straight->turn boundary crossings the original report named do not
reproduce it. Closing this issue with this verdict per ticket 109-008's own
acceptance criterion.

## If this verdict is wrong — what would pin it down

This was inspection + isolated reproduction, not a live capture of the actual
freeze. If a future session reproduces a GENUINE GUI freeze (window stops
repainting, unresponsive to clicks, force-quit required) rather than "the robot
stopped moving and the log went quiet," the evidence that would actually
distinguish a real Qt deadlock from this dispatch's explanation:

- Whether the window was truly unresponsive to ANY input (menu clicks, window
  drag) — not just the tour appearing stalled — during the freeze.
- A thread dump / `py-spy dump` taken while frozen, showing the main thread
  blocked inside Qt event delivery (vs. idle/waiting normally).
- Whether the message monitor log pane was still scrolling/updating during the
  "freeze" (rules out a full GUI deadlock; consistent with a stopped tour +
  possibly-still-flooding log, i.e. this dispatch's explanation).
- The exact console output at the moment of the freeze — specifically whether a
  `[TOUR] ... stopped at leg N/M (fault)` line appears (confirms the WedgeLatch
  explanation) or whether no further log lines appear at all even after the
  robot's own fault would have fired (would point back toward a genuine GUI
  freeze and reopen this investigation).
