---
id: '001'
title: Fix graph-tab data persistence across view switches
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: testgui-graphs-not-persistent-on-view-switch.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix graph-tab data persistence across view switches

## Description

Reported symptom: switching the TestGUI's four live graph tabs (wheel
speed, wheel position, heading, distance) away and back corrupts the
earlier graph's history — "the existing series is deleted and then
repopulated with wrong data."

**Sprint-planning-time finding — start from here, don't assume a fix
location.** Reading `src/host/robot_radio/testgui/turn_graphs.py`'s
current implementation did NOT turn up an obvious structural cause:
`TurnGraphPanel` owns exactly one, persistent `TurnTraceRecorder`
(`self.recorder`) and four independent `_GraphCanvas` instances, each
with its OWN `matplotlib` `Figure`/`Axes` and its own fixed, DISJOINT
series-key list (`WHEEL_SPEED`/`WHEEL_POS`/`HEADING`/`DISTANCE` share no
keys). `QTabWidget.currentChanged` and the 150 ms dirty-repaint `QTimer`
both route through `_redraw_current()`, which only calls `redraw()` on
the CURRENTLY-selected canvas — `_GraphCanvas.redraw()` does `ax.clear()`
then re-plots strictly from `recorder.series.get(key)` for its own key
list. The recorder itself is only ever cleared by the explicit
"Clear traces" button (`TurnGraphPanel.clear()`) — nothing in the
tab-switch path touches it. This reads as CORRECT, not buggy.

Two live possibilities, and this ticket's first job is to determine
which is true (do not skip straight to "assume fixed, close ticket"):
1. The repro requires exercising a real, continuous data stream (the
   sprint-planning-time read was static code inspection, not a live
   repro) — some interaction between the 150 ms timer, a tab switch
   landing mid-append, and matplotlib's own canvas backend could still
   produce the reported corruption under real timing. Build a real
   automated repro before concluding "no bug here."
2. The issue's own "Notes / where to look" section names `canvas.py` —
   on inspection, that file holds the playfield/avatar canvas
   (`CanvasController`), NOT the turn graphs. If the actual, currently-
   shipped graph-tab UI the operator was using is a DIFFERENT
   implementation than `turn_graphs.py` (e.g. an older or parallel one),
   find it. Grep the full `testgui/` tree and `__main__.py`'s own wiring
   for anything constructing graph tabs before concluding `turn_graphs.py`
   is the only candidate.

## Acceptance Criteria

- [x] An automated, headless Qt test reproduces (or, with a documented
      attempt, fails to reproduce) the exact switch-away/switch-back
      corruption sequence from the issue's own repro steps, using a real,
      continuous telemetry-frame stream (not a static/one-shot dataset).
- [x] If the bug reproduces: root cause is identified and fixed at its
      actual location (not a guessed one); the fix is such that each
      graph's data is per-series-owned and a tab switch only changes what
      is DISPLAYED, never what is stored.
- [x] If the bug does NOT reproduce in `turn_graphs.py`: the ticket
      explicitly records (a) that a real repro attempt was made, not just
      static reading, (b) whether a second graph-tab implementation was
      found and checked, and (c) the conclusion — do not close silently
      on "couldn't repro" without this record. (N/A — the bug DID
      reproduce in `turn_graphs.py`; see Findings below.)
- [x] A regression test exists that will fail if this bug (or an
      equivalent one) is reintroduced — added to `src/tests/testgui/`
      regardless of which of the above two outcomes applies.
- [x] No change to `TurnTraceRecorder`'s accumulation semantics (Qt-free,
      independently testable) unless the root cause is actually inside
      it — prefer a fix in the Qt-layer redraw/switch path if that's
      where the bug lives. (The root cause WAS inside
      `TurnTraceRecorder.add_tlm()` — see Findings; the Qt layer
      (`TurnGraphPanel`/`_GraphCanvas`) needed no change.)
- [x] Full `src/tests/testgui/` suite stays green.

## Findings (2026-07-17)

**Reproduced: yes**, in `turn_graphs.py` — no second graph-tab
implementation exists. `canvas.py` (named in the issue's "Notes / where to
look") was inspected and confirmed to hold `CanvasController`, the
playfield/avatar canvas — an unrelated code path with its own separate
`TraceModel`, not a second implementation of the four turn/drive graphs.
Grepping the full `testgui/` tree and `__main__.py`'s own wiring turned up
exactly one graph-tab constructor (`TurnGraphPanel` at `__main__.py:1176`).

**Root cause**: `TurnTraceRecorder.add_tlm()` (Qt-free) auto-froze
recording after >1 s of no wheel motion (`_stopped = True`), and — before
this fix — called `self.clear()` (wiping ALL four graphs' entire series,
not just the active tab's) the moment a subsequent telemetry frame showed
motion again. This wipe fires purely from the telemetry stream's own
idle/resume pattern, independent of which tab is selected. But because
`TurnGraphPanel._redraw_current()` only ever repaints the CURRENTLY active
tab (via `QTabWidget.currentChanged` and the 150 ms dirty timer), an
operator watching the wheel-speed tab while an idle-then-resume happens on
a DIFFERENT tab never sees the wipe happen — they only discover the lost
history when they switch back. That is exactly the reported "switch away,
switch back, existing series deleted and repopulated with wrong data"
symptom, even though the proximate trigger is a resume-from-idle
telemetry frame rather than the tab switch itself. Confirmed by a
standalone repro script (feed 10 wheel-speed frames while on the
wheel-speed tab -> switch to Heading -> feed an idle frame + a
resume-motion frame while still on Heading -> switch back to wheel-speed):
`vel_l` went from 10 points to 1 after the switch-back, before the fix.

**Fix**: `add_tlm()` no longer calls `clear()` on resume-from-idle; it
just clears the `_stopped` freeze flag and continues appending to the same
series. Only the explicit "Clear traces" button
(`TurnGraphPanel.clear()` -> `TurnTraceRecorder.clear()`) now discards
data. No Qt-layer change was needed — `TurnGraphPanel`/`_GraphCanvas`'s
tab-switch/redraw logic was already correct, matching the
sprint-planning-time static read.

**Regression tests** (`src/tests/testgui/test_turn_graphs_persistence.py`,
new file):
- `test_tab_switch_during_idle_resume_preserves_wheel_speed_history` — the
  Qt-level repro above, as a pytest.
- `test_recorder_add_tlm_resumes_without_clearing_after_idle_freeze` —
  Qt-free unit test directly on `TurnTraceRecorder`.
Both were confirmed to FAIL against the pre-fix code (via `git stash` on
just `turn_graphs.py`) and PASS against the fix.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/testgui/`
  (full suite, confirm no regression from whatever fix lands).
- **New tests to write**: a headless Qt test that (1) constructs a
  `TurnGraphPanel` (or whatever the real widget turns out to be), (2)
  feeds it a stream of distinguishable synthetic `TLMFrame`s while on the
  wheel-speed tab, (3) switches to another tab, feeds more distinguishable
  frames, (4) switches back to wheel-speed, and (5) asserts the
  wheel-speed tab's underlying series data (not just that it renders
  without crashing) contains ALL frames fed while it was both the active
  and an inactive tab, in the correct order, with no corruption.
- **Verification command**: `uv run python -m pytest src/tests/testgui/
  -k "graph or persistence or turn_graphs"`.

## Implementation Plan

**Approach**: Reproduce first, with a real automated test exercising a
continuous stream — the sprint-planning-time static read found the
current `turn_graphs.py` structure plausible-correct, so guessing at a
fix without a real repro risks "fixing" a non-bug or missing the actual
one. If a second graph-tab implementation is found, that changes the fix
location entirely — check before assuming.

**Files to investigate first (read, don't assume)**:
- `src/host/robot_radio/testgui/turn_graphs.py` (`TurnTraceRecorder`,
  `TurnGraphPanel`, `_GraphCanvas`)
- `src/host/robot_radio/testgui/canvas.py` (confirmed at planning time to
  be the playfield/avatar canvas, not graphs — re-check only if the
  above doesn't pan out)
- `src/host/robot_radio/testgui/__main__.py`'s own wiring of whichever
  graph-tab widget is actually instantiated for the live GUI

**Files to modify**: determined by the repro's findings — not
predictable at planning time.

**Testing plan**: as above — a real, continuous-stream repro test first,
then the fix, then a permanent regression test.

**Documentation updates**: none anticipated (`src/firm/` untouched by
this ticket; no `DESIGN.md` obligation applies). If a host-side design
doc for `testgui/` exists, note the finding there.
