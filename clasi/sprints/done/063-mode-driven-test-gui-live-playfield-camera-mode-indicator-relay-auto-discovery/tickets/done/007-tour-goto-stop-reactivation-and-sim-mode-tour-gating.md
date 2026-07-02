---
id: '007'
title: Tour/GOTO stop reactivation and Sim-mode tour gating
status: done
use-cases:
- SUC-008
- SUC-009
depends-on: []
github-issue: ''
issue:
- testgui-tour-stop-reactivation.md
- testgui-tour-sim-mode-gating.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Tour/GOTO stop reactivation and Sim-mode tour gating

## Description

Two confirmed, related bugs in the Tour/GOTO worker wiring in
`host/robot_radio/testgui/__main__.py`:

1. **Tour button never reactivates after Stop** (`testgui-tour-stop-reactivation.md`).
   Root cause (confirmed by code reading): `_stop_tour()` calls `worker.stop()`,
   `thread.quit()`, `thread.wait(3000)` — which **blocks the main thread** — then
   sets `_state["tour_bridge"] = None`. The worker's `finished` signal fires
   *during* that blocking `wait()`, so the queued slot cannot run until `wait()`
   returns; and once it returns, `_stop_tour` has already dropped the only
   Python reference to the `_WorkerBridge` (`tour_bridge`), so the bridge is
   eligible for GC and the still-pending queued `finished → bridge.on_finished`
   delivery is lost. `_on_tour_finished` — the *only* place that re-enables the
   tour buttons — never runs. The exact same weakness exists in `_stop_goto()` /
   `_on_goto_finished()` for the GOTO button.

   There is also no dedicated, visible control to stop a running tour today —
   only the shared Operations STOP button (`_stop_all_motion`) reaches
   `_stop_tour()`.

2. **Sim-mode tour gating is unclear** (`testgui-tour-sim-mode-gating.md`). Code
   reading confirms tour buttons are correctly gated on `_state["transport"]`
   being set, and are only enabled after a successful Connect — so a tour
   cannot fire pre-connect from a code standpoint. But a `SimTransport` *does*
   count as "connected", so selecting Sim + Connect + Tour 1 will happily run
   the tour against the simulator. This is confusing to an operator who reads
   it as "the tour ran without being connected to the robot." **Stakeholder
   decision: tours remain allowed in Sim mode** (useful for dry-runs), but must
   log an unambiguous `[TOUR] running in SIM mode` line so the operator is
   never confused about the target.

## Stakeholder Decisions (binding)

- Add a dedicated, visible **"Stop Tour"** button next to the Tour 1 button
  (there is currently only one named tour, so a single dedicated button is
  unambiguous — no need to repurpose the Tour 1 button's own label). Disabled
  when no tour is running; enabled while one runs. Wired to the same
  `_stop_tour()` function as the shared STOP button.
- Fix `_stop_tour()` to **synchronously re-enable** the tour buttons (and
  disable the new Stop Tour button) **after the thread join, inside
  `_stop_tour()` itself**, when a transport is still connected — do NOT rely
  on the `finished` signal / `_on_tour_finished` for the explicit-stop path.
  `_on_tour_finished` remains the mechanism for the *natural completion* path
  (worker finishes on its own, `_stop_tour()` is never called) — leave that
  path working as-is.
- Apply the identical synchronous re-enable fix to `_stop_goto()` /
  `_on_goto_finished()` for `goto_btn`. No new dedicated UI control is needed
  for GOTO — the existing shared STOP button already reaches `_stop_goto()`.
- Tours remain runnable in Sim mode. Log `[TOUR] running in SIM mode` when
  `is_sim_transport(transport)` is true, at tour start, before the existing
  "resetting to origin" log line.
- Add a regression test confirming tour buttons cannot be clicked before
  Connect (one already exists — `TestTourButton.test_tour_button_present_and_disabled`
  in `tests/testgui/test_smoke.py` — extend it to also cover the new Stop Tour
  button's initial disabled state).

## Affected Code

- `host/robot_radio/testgui/__main__.py`:
  - `_stop_tour()`, `_on_tour_finished()`, `_make_tour_handler()` /
    `_on_tour_clicked()`, tour button row construction (currently builds only
    `_tour_buttons` from `TOURS`).
  - `_stop_goto()`, `_on_goto_finished()`.
- `host/robot_radio/testgui/operations.py` — reuse the existing
  `is_sim_transport(transport)` pure helper (already defined there; do not
  duplicate the check).
- `tests/testgui/test_smoke.py` — extend `TestTourButton`.
- New test file (or extension) covering the stop/reactivation fix — see
  Testing Plan below.

## Acceptance Criteria

### Stop Tour control

- [x] A new `QPushButton` (suggested `objectName="stop_tour_btn"`, text "Stop
      Tour") exists next to the Tour 1 button in the tour row.
- [x] `stop_tour_btn` starts disabled (no tour running, and/or not connected).
- [x] `stop_tour_btn` becomes enabled exactly when a tour starts
      (`_on_tour_clicked`) and disabled again exactly when the tour stops
      (whether via explicit stop or natural completion).
- [x] Clicking `stop_tour_btn` calls `_stop_tour()`.

### Synchronous reactivation (Tour)

- [x] `_stop_tour()` re-enables all tour buttons (`Tour 1`, any future tours)
      and disables `stop_tour_btn`, synchronously, immediately after the
      `thread.quit()` / `thread.wait()` join — provided
      `_state.get("transport")` is not `None` — without depending on the
      worker's `finished` signal being delivered.
- [x] Calling `_stop_tour()` when no tour is running remains a safe no-op
      (existing behavior preserved).
- [x] `_on_tour_finished()` (natural-completion path) continues to re-enable
      the buttons correctly; no regression when a tour completes without an
      explicit stop.
- [x] After an explicit stop, the Tour 1 button is immediately re-enabled
      (this is the regression the issue reports — must be fixed).

### Synchronous reactivation (GOTO)

- [x] `_stop_goto()` re-enables `goto_btn` synchronously after the join,
      provided a transport is still connected — mirroring the Tour fix.
- [x] `_on_goto_finished()` natural-completion path is unaffected.

### Sim-mode gating

- [x] Starting a tour (`_on_tour_clicked`) with `is_sim_transport(transport)`
      True logs `[TOUR] running in SIM mode` before the "resetting to origin"
      log line.
- [x] Starting a tour with a non-Sim transport does NOT log that line.
- [x] Tours are NOT blocked in Sim mode (existing dry-run behavior preserved).
- [x] Regression test confirms tour buttons (`tour_btn_tour_1` and the new
      `stop_tour_btn`) are disabled before Connect.

### No regressions

- [x] All existing `tests/testgui/` tests pass unchanged.
- [x] `uv run python -m pytest tests/simulation -q` remains green (this
      ticket does not touch firmware/sim code, but confirm no import-order
      breakage).

## Implementation Plan

### Approach

1. **Tour row**: in the tour-button construction block (`__main__.py`, near
   `_tour_buttons: list[tuple[QPushButton, str]] = []`), after the loop that
   builds one button per `TOURS` entry, add:

   ```python
   stop_tour_btn = QPushButton("Stop Tour")
   stop_tour_btn.setObjectName("stop_tour_btn")
   stop_tour_btn.setEnabled(False)
   tour_layout.addWidget(stop_tour_btn)
   ```

   Note: do NOT append `stop_tour_btn` to `_send_buttons` — it must stay
   disabled while idle even when connected (it only enables while a tour is
   actively running), unlike the Tour 1 button which enables on connect.

2. **`_stop_tour()`**: after the existing join logic (`thread.quit()` /
   `thread.wait(3000)`) and clearing `_state["tour_worker"/"tour_thread"/"tour_bridge"]`,
   add the synchronous re-enable:

   ```python
   def _stop_tour() -> None:
       worker = _state.get("tour_worker")
       thread = _state.get("tour_thread")
       if worker is not None:
           try:
               worker.stop()
           except Exception:
               pass
       if thread is not None:
           try:
               thread.quit()
               thread.wait(3000)
           except Exception:
               pass
       _state["tour_worker"] = None
       _state["tour_thread"] = None
       _state["tour_bridge"] = None
       # Re-enable synchronously — do not rely on the worker's `finished`
       # signal, which may be undeliverable after this method returns (see
       # testgui-tour-stop-reactivation.md for the root-cause race).
       if _state.get("transport") is not None:
           for _tb, _ in _tour_buttons:
               _tb.setEnabled(True)
       stop_tour_btn.setEnabled(False)
   ```

   `_on_tour_finished()` is unchanged in its own re-enable logic (it still
   guards on `_state.get("transport")` and re-enables `_tour_buttons`), but
   should also disable `stop_tour_btn` for consistency on the natural-completion
   path:

   ```python
   def _on_tour_finished() -> None:
       ...
       if _state.get("transport") is not None:
           for _tb, _ in _tour_buttons:
               _tb.setEnabled(True)
       stop_tour_btn.setEnabled(False)
   ```

3. **`_on_tour_clicked` (inside `_make_tour_handler`)**: enable
   `stop_tour_btn` alongside disabling the tour buttons, and log the Sim-mode
   line first:

   ```python
   def _on_tour_clicked() -> None:
       transport = _state.get("transport")
       if transport is None:
           _append_log("[WARN] Not connected")
           return
       if _state.get("tour_thread") is not None:
           _append_log("[WARN] A tour is already running")
           return
       from robot_radio.testgui.operations import is_sim_transport
       if is_sim_transport(transport):
           _append_log("[TOUR] running in SIM mode")
       _append_log(f"[TOUR] {name} starting — resetting to origin")
       _set_origin()
       for _tb, _ in _tour_buttons:
           _tb.setEnabled(False)
       stop_tour_btn.setEnabled(True)
       ... # unchanged worker/thread/bridge construction
   ```

   Wire the click: `stop_tour_btn.clicked.connect(_stop_tour)`.

4. **`_stop_goto()`**: apply the identical synchronous re-enable for
   `goto_btn`:

   ```python
   def _stop_goto() -> None:
       ...  # existing worker.stop() / thread.quit() / thread.wait() / clear state
       if _state.get("transport") is not None:
           goto_btn.setEnabled(True)
   ```

   `_on_goto_finished()` keeps its existing re-enable (unchanged; it already
   does this correctly for the natural-completion path per the existing code).

5. Verify ordering: `stop_tour_btn` must be defined (in scope) before
   `_stop_tour()` and `_on_tour_clicked()` reference it — since these are all
   closures inside the same function body, define `stop_tour_btn` right after
   the tour-button loop, ahead of the `_stop_tour` / `_on_tour_finished` /
   `_make_tour_handler` definitions (matching the existing top-to-bottom
   ordering in the file, ~line 428 onward through ~line 1198).

### Files to modify

- `host/robot_radio/testgui/__main__.py`
- `tests/testgui/test_smoke.py` (extend `TestTourButton`)

### Testing Plan

`host/robot_radio/testgui/__main__.py`'s internals are closures with no test
seam (`_build_main_window()` returns only `(window, app)`). Follow the
established project pattern for this constraint (see
`tests/testgui/test_set_origin.py`): re-implement the exact production control
flow inline in the test using fake worker/thread doubles, so the *logic* is
verified deterministically without real `QThread` timing.

- **Existing tests to run**: `QT_QPA_PLATFORM=offscreen uv run python -m pytest
  tests/testgui/ -q` (must stay green); `uv run python -m pytest
  tests/simulation -q` (must stay green — no sim/firmware code touched, sanity
  check only).
- **New tests to write** (new file, e.g. `tests/testgui/test_tour_stop.py`, or
  extend `test_smoke.py`):
  - `test_stop_tour_button_present_and_disabled` — `stop_tour_btn` exists via
    `findChild(QPushButton, "stop_tour_btn")`, is disabled before any tour runs.
  - `test_stop_tour_reenables_buttons_synchronously` — construct a minimal
    `_state` dict plus fake `worker`/`thread` doubles (`worker.stop()` a no-op,
    `thread.quit()`/`thread.wait()` no-ops returning immediately — mirroring
    `_FakeTransport`-style doubles already used in `test_set_origin.py`) and a
    fake `_tour_buttons` list of real (or mock) `QPushButton`s set to disabled.
    Re-implement `_stop_tour()`'s logic inline (per `test_set_origin.py`'s
    established pattern) and assert the tour buttons are `setEnabled(True)`
    and `stop_tour_btn` is `setEnabled(False)` *immediately* after the call
    returns — i.e. the re-enable does not depend on any signal being processed
    afterward.
  - `test_stop_goto_reenables_button_synchronously` — same pattern for
    `goto_btn` / `_stop_goto()`.
  - `test_tour_click_logs_sim_mode_line` — construct a fake `SimTransport`-like
    object (or import the real `SimTransport` from
    `robot_radio.testgui.transport` and check `is_sim_transport()` returns
    True for it) and a fake non-sim transport; re-implement the relevant slice
    of `_on_tour_clicked()`'s logging logic (Sim-mode check + log line) and
    assert the `[TOUR] running in SIM mode` line appears only for the Sim
    transport case.
  - Extend `TestTourButton.test_tour_button_present_and_disabled` (or add a
    sibling test) in `test_smoke.py` to also assert
    `findChild(QPushButton, "stop_tour_btn")` exists and
    `not stop_tour_btn.isEnabled()` before Connect.
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run python -m pytest
  tests/testgui/ -q`

### Documentation updates

- Update the `_stop_tour` / `_on_tour_finished` / `_stop_goto` /
  `_on_goto_finished` docstrings in `__main__.py` to describe the synchronous
  re-enable and why it no longer depends on the `finished` signal for the
  explicit-stop path (cross-reference `testgui-tour-stop-reactivation.md` in a
  comment, matching the existing style of comments referencing `_WorkerBridge`).
- Update the tour-button row comment to mention the new `stop_tour_btn`.
