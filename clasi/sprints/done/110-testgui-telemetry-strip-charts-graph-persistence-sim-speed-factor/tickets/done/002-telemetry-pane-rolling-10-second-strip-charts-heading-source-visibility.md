---
id: '002'
title: Telemetry-pane rolling 10-second strip charts + heading-source visibility
status: done
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

- [x] A new tabbed widget (playfield-mode-tab styling) occupies the
      telemetry section's previously-unused right-hand space, with four
      tabs: wheel speed, wheel position, heading, distance.
- [x] Each tab shows a continuously-scrolling window of at most the last
      10 seconds of that series — verified by a windowing test (points
      older than 10 s excluded from the strip-chart view; the SAME points
      remain present in the unaffected top-graph recorder/view).
- [x] The strip charts read from the SAME `TurnTraceRecorder` the (by-now
      fixed, ticket 001) top graphs use — no second recorder, no second
      telemetry-consumption path.
- [x] `queue_depth`, `active_id`, `exec_state`, `heading_source` are all
      added to the `TLMFrame` dataclass and decoded in `from_pb2()`,
      following its existing additive-field/optional-default convention.
- [x] `heading_source` is visibly surfaced in the telemetry pane (OTOS vs.
      encoder-fallback) — the hard, stakeholder-driven bar.
- [x] Older firmware that doesn't set these fields (or older `TLMFrame`
      construction paths) degrades gracefully — fields read `None`/a
      documented default, no crash.
- [x] Full `src/tests/testgui/` suite stays green; `TLMFrame.from_pb2()`'s
      own existing decode tests are extended, not replaced.

## Findings / Completion Notes (2026-07-17)

**Decode gap closure**: `TLMFrame` (protocol.py) gained four new fields
(`queue_depth`, `active_id`, `exec_state`, `heading_source`), decoded
unconditionally in `from_pb2()` (telemetry.proto declares them as plain
proto3 scalars with no `has_*` gate — the SAME "always present" treatment
`active`/`acks`/`fault_bits`/`event_bits` already get). `exec_state`/
`heading_source` are kept as raw enum ints (matching `AckEntry.status`'s
own raw-int convention), not decoded into a host-side string — a caller
compares against `telemetry_pb2.EXEC_*`/`HEADING_SOURCE_STATUS_*`
directly. Older firmware / a bare `TLMFrame()` not built via `from_pb2()`
stays at the dataclass's own `None` default (not a crash); a `Telemetry()`
built with no explicit value for these fields decodes to the proto3
zero-value default (0 for each), the same graceful-degrade shape `active`
already has.

**`queue_depth`/`active_id`/`exec_state` — decoded but NOT displayed.**
Per this ticket's own item 3, these three are decoded into `TLMFrame` (same
frame, same adapter method, cheap) but left UNSHOWN in the telemetry pane
this ticket — only `heading_source` (the stakeholder-mandated visibility
requirement) got a panel row. Surfacing executor queue/state visibility is
left as a future ticket's own UI work if wanted.

**`heading_source` visibility**: new "heading src" row in the telemetry
panel grid (`telemetry_panel.py`), styled loudly (amber background, bold
text, "ENCODER (fallback)" text) whenever `heading_source ==
HEADING_SOURCE_STATUS_ENCODER` — the non-gyro state is visually impossible
to miss, not just a plain text value like every other row. `None`
(undecoded/older firmware) is NOT treated as fallback — renders `—` like
every other absent field.

**Strip charts**: `turn_graphs.py` gained `TurnTraceRecorder.latest_t()`
(Qt-free "now" reference) and `StripChartCanvas` (a `_GraphCanvas`
subclass that filters `recorder.series[key]` to the trailing `window`
(default 10s) at redraw time — a pure windowing filter, never a second
recorder or a second `add_tlm`/`add_camera` call site).
`build_telemetry_panel()` gained an optional `recorder` parameter;
`__main__.py` passes `graph_panel.recorder` — the SAME recorder
`TurnGraphPanel` (the top graphs) already owns and feeds — so the two
views can never disagree about what was recorded. The four strip-chart
tabs occupy telemetry panel grid column 3 (previously pure horizontal-slack
padding with nothing in it), styled the same plain `QTabWidget` as
`TurnGraphPanel`'s own tab bar (no separate "playfield-mode" widget class
exists to reuse; visual parity achieved via the same widget type).

**New tests**: `src/tests/testgui/test_telemetry_strip_charts.py` (new) —
`latest_t()` unit tests, `StripChartCanvas` windowing tests (feed 30
frames/15s of synthetic history, assert the canvas's own plotted lines
only carry the trailing 10s while `recorder.series` still has all 30
points), and `build_telemetry_panel(recorder=...)` wiring tests (the panel
reads the caller-supplied recorder, not a private one). Extended
`src/tests/unit/test_protocol_binary_client.py` (decode tests for the four
new fields) and `src/tests/testgui/test_telemetry_panel.py`
(`heading_source` formatting + widget-level OTOS→ENCODER→OTOS transition
test) — none of the pre-existing tests in either file were replaced, only
added to.

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
