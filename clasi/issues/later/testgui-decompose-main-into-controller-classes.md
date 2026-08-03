---
status: pending
---

# Decompose testgui __main__.py's 3,000-line function into controller classes; move the embedded build pipeline out

**Source:** code review 2026-07-30, `05-testgui-testkit.md` MAJOR §7, §8.
**Priority:** P1 — `_build_main_window()` is 3,028 lines (23% of the whole
testgui+testkit tree) of nested closures over one shared `_state` dict. This
is where the STOP-button and arrow-key defects hid: nothing in it is
independently testable.
**Goal served:** the package already contains the correct template
(`operations.py`'s `build_panel()` + `OpsController`: widget factory vs. a
plain, Qt-decoupled, testable controller). Applying it to the remaining
state machines makes each one unit-testable headlessly — which is exactly
how the acceptance suite would have caught the dead STOP path.

## Progress 2026-07-31 (out of process)

**Done — `WorkerSession`** (`src/host/robot_radio/testgui/worker_session.py`,
10 tests). Tour and GOTO had byte-identical copies of the QThread worker
lifecycle differing only in which buttons re-enable; both are now one tested
class, and the six shadow `_state` slots (tour/goto worker/thread/bridge) are
gone. This was the piece the STOP defect actually hid in, so it went first.

`__main__.py` is now 3,209 lines (was 3,262). That is a small dent in the line
count and a large one in testability -- the point was never the line count.

**Still to do**, unchanged from below:
- `ConnectionController` — the connect/disconnect state machine. The largest
  and riskiest of the three; it touches every button's enabled state.
- `TourController` / `GotoController` — the remaining non-lifecycle logic
  (origin reset, sim-mode logging, worker construction). `WorkerSession`
  removed the shared part; what is left is genuinely per-controller.
- Moving the "Test S/T/US/UT" build pipeline to `src/tests/sim/`. Deliberately
  not done out of process: it REMOVES buttons from a GUI the stakeholder is
  actively using, which is their call, not a refactor's.

## What to do

1. Extract three controllers following the `OpsController` pattern, each
   owning its state (no shared `_state` dict), each testable without a
   `QApplication`:
   - `ConnectionController` — the connect/disconnect state machine
     (`_on_connect`/`_on_disconnect`, ~lines 2929-3207)
   - `TourController` — `_make_tour_handler`/`_stop_tour`/
     `_on_tour_finished` (~2632-2751)
   - `GotoController` — `_GotoWorker`/`_stop_goto` (~2753-2820)

```python
# Shape (mirrors operations.py):
class TourController:
    """Owns tour-run state. Qt-free: callers wire signals in build_panel()."""
    def __init__(self, transport_ref, log, on_finished): ...
    def start(self, tour, params) -> None: ...
    def stop(self) -> None: ...      # cancels AND joins the worker; then
                                     # transport.halt() -- see the halt issue

def build_tour_panel(controller: TourController) -> QWidget: ...
```

2. Move the "Test S/T/US/UT" edit-compile-hot-reload pipeline
   (`__main__.py:3209-3316` — gen scripts, `cmake --build`, dylib hot-swap,
   scripted verification) to a standalone `src/tests/sim/` script. The
   plumbing (`_sim_lib_path()`/`set_sim_lib_override()`) already lives in
   `transport.py` and stays importable. testgui loads a build; it must not
   trigger one.

3. Headless button-acceptance tests for each extracted controller (the
   project rule: GUI work is not done until
   `test_gui_button_acceptance.py`-style coverage exercises it), including
   the hardware-shaped transport mock from the STOP issue.

Sequencing note: land the STOP/halt fix first (small, safety); this
decomposition then gives it a permanent testable home.

## Acceptance

- `_build_main_window()` is under ~500 lines of widget assembly; connection/
  tour/GOTO logic lives in controller classes with their own tests.
- No `cmake`/`subprocess` build invocation reachable from the GUI.
- Full GUI acceptance suite passes; manual smoke: connect (Sim + hardware),
  run a tour, run a GOTO, STOP mid-motion.
