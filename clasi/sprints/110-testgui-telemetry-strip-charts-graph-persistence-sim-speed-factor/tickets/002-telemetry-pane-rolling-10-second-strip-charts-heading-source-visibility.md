---
id: '002'
title: Telemetry-pane rolling 10-second strip charts + heading-source visibility
status: in-progress
use-cases:
- SUC-002
depends-on:
- '001'
github-issue: ''
issue: testgui-telemetry-strip-charts.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Telemetry-pane rolling 10-second strip charts + heading-source visibility

## Description

Add a second, tabbed set of the same four graphs (wheel speed, wheel
position, heading, distance) in the telemetry section's currently-unused
right-hand space — tabbed the same way as the playfield-mode tabs. Unlike
the existing top graphs (full-run history), each of these is a **rolling
10-second strip chart**: once 10 seconds of data have accumulated, the
oldest points scroll off the left edge, always showing at most the last
10 seconds.

**Reuse, don't duplicate.** Per the issue's own instruction: this is a
WINDOWED VIEW over the same telemetry stream the top graphs
(`TurnTraceRecorder`, fixed by ticket 001) already accumulate — not a
second data source, not a second recorder. Implement the strip chart as
a redraw-time filter (trailing-10-second slice of `recorder.series[key]`)
over the SAME shared recorder the top graphs use, so the two views can
never disagree about what was actually recorded and no telemetry frame
is processed twice.

**Also close a real wire-decode gap.** Sprint 109 added four new fields
to `telemetry.proto`'s primary `Telemetry` message — `queue_depth`,
`active_id`, `exec_state` (enum `ExecutorState`: IDLE/RUNNING/
RAMP_TO_REST/STOPPING), and `heading_source` (enum `HeadingSourceStatus`:
OTOS/ENCODER) — but `TLMFrame.from_pb2()`
(`src/host/robot_radio/robot/protocol.py`) does not decode any of them
into the parsed `TLMFrame` the GUI actually consumes (confirmed by
reading `from_pb2()` directly at planning time — its own docstring's
"fields left at default" enumeration predates these four and doesn't
mention them). `heading_source` visibility (which sensor — OTOS or
encoder-fallback — is currently heading-truth) is a **standing
stakeholder requirement**; this is the first TestGUI work to touch the
telemetry pane since that field shipped, so close the gap now:
1. Add all four fields to the `TLMFrame` dataclass and decode them in
   `from_pb2()`, following that method's own existing "adapter, not a
   redesign" convention (additive fields, `None` default when absent,
   matching every other optional field's own pattern).
2. Surface `heading_source` VISIBLY in the telemetry pane (e.g. an "OTOS"/
   "ENCODER (fallback)" indicator) — this is the hard, stakeholder-driven
   acceptance criterion.
3. `queue_depth`/`active_id`/`exec_state` are decoded into `TLMFrame` at
   the same time (cheap — same frame, same adapter method) but are NOT
   required to be visibly displayed by this ticket; note in the ticket
   whether you displayed them anyway or left them decoded-but-unshown.

## Acceptance Criteria

- [ ] A new tabbed widget (playfield-mode-tab styling) occupies the
      telemetry section's previously-unused right-hand space, with four
      tabs: wheel speed, wheel position, heading, distance.
- [ ] Each tab shows a continuously-scrolling window of at most the last
      10 seconds of that series — verified by a windowing test (points
      older than 10 s excluded from the strip-chart view; the SAME points
      remain present in the unaffected top-graph recorder/view).
- [ ] The strip charts read from the SAME `TurnTraceRecorder` the (by-now
      fixed, ticket 001) top graphs use — no second recorder, no second
      telemetry-consumption path.
- [ ] `queue_depth`, `active_id`, `exec_state`, `heading_source` are all
      added to the `TLMFrame` dataclass and decoded in `from_pb2()`,
      following its existing additive-field/optional-default convention.
- [ ] `heading_source` is visibly surfaced in the telemetry pane (OTOS vs.
      encoder-fallback) — the hard, stakeholder-driven bar.
- [ ] Older firmware that doesn't set these fields (or older `TLMFrame`
      construction paths) degrades gracefully — fields read `None`/a
      documented default, no crash.
- [ ] Full `src/tests/testgui/` suite stays green; `TLMFrame.from_pb2()`'s
      own existing decode tests are extended, not replaced.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/testgui/`
  (full suite); any existing `TLMFrame.from_pb2()`/`protocol.py` decode
  tests (must stay green — this is a purely additive change to that
  method).
- **New tests to write**: `TLMFrame.from_pb2()` decode test asserting
  `queue_depth`/`active_id`/`exec_state`/`heading_source` are correctly
  populated from a synthetic `telemetry_pb2.Telemetry` message, and that
  they default sensibly when absent; a strip-chart windowing test
  (feed >10 s of synthetic frames, assert the strip-chart view only shows
  the trailing 10 s while the top-graph recorder still has everything);
  a `heading_source` visibility test (feed an OTOS-then-ENCODER
  transition, assert the telemetry-pane indicator updates).
- **Verification command**: `uv run python -m pytest src/tests/testgui/
  -k "telemetry or strip_chart or heading_source"`.

## Implementation Plan

**Approach**: Build the strip-chart tabs as a thin windowing layer over
the existing `TurnTraceRecorder`/`_GraphCanvas` infrastructure (post-
ticket-001 fix) rather than a parallel implementation — a new
`_GraphCanvas` subclass or wrapper that filters `recorder.series[key]` to
`now - 10s` at redraw time is the expected shape, not a new recorder
class. Decode the four new `TLMFrame` fields as a small, independent,
purely-additive change to `protocol.py`, landed either before or
alongside the UI work.

**Files to modify**:
- `src/host/robot_radio/testgui/telemetry_panel.py` (new tab set, layout
  into the unused right-hand space)
- `src/host/robot_radio/testgui/turn_graphs.py` (reuse `_GraphCanvas`/
  `TurnTraceRecorder`; add the 10-second windowing behavior — as a new
  canvas variant or a `redraw(recorder, window_s=10.0)`-style parameter,
  whichever fits the existing class shape more cleanly)
- `src/host/robot_radio/robot/protocol.py` (`TLMFrame` dataclass fields +
  `from_pb2()` decode for the four new fields)

**Testing plan**: as above.

**Documentation updates**: none in `src/firm/` (this ticket is host-only;
the four wire fields it decodes already shipped in sprint 109, no wire-
schema change here). If a host-side `testgui`/`protocol` design doc
exists, note the newly-decoded fields there.
