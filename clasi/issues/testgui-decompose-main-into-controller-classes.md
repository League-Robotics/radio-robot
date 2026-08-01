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
