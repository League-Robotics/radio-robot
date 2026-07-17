---
id: '110'
title: 'TestGUI: telemetry strip charts, graph persistence, sim speed factor'
status: closed
branch: sprint/110-testgui-telemetry-strip-charts-graph-persistence-sim-speed-factor
worktree: false
use-cases:
- SUC-001
- SUC-002
- SUC-003
issues:
- testgui-telemetry-strip-charts.md
- testgui-graphs-not-persistent-on-view-switch.md
- testgui-speedup-factor-broken-at-high-values.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 110: TestGUI: telemetry strip charts, graph persistence, sim speed factor

## Goals

Three independent TestGUI (PySide6, `src/host/robot_radio/testgui/`) usability
fixes/additions, all discovered during and after sprint 109's TestGUI Sim-mode
revival: (1) the four live graphs (wheel speed, wheel position, heading,
distance) lose or corrupt their accumulated history when the operator
switches between graph tabs; (2) a second, rolling 10-second strip-chart
view of the same four series is wanted in the telemetry section's
currently-unused right-hand space, for at-a-glance recent behavior without
scrolling the full-run graphs; (3) the Sim speed-up selector (1×/2×/5×/
10×/20×) stutters at 10× and does not work at all at 20×, instead of
scaling the simulation rate smoothly. None of these touch firmware —
this is a host/Python + Qt sprint, and no `src/firm/` `DESIGN.md` update
is expected from any ticket (flagged explicitly if that assumption turns
out wrong during implementation).

## Problem

1. **Graph persistence** (`testgui-graphs-not-persistent-on-view-switch.md`):
   reported symptom is that switching tab away from and back to a graph
   corrupts its history — "the existing series is deleted and then
   repopulated with wrong data." Sprint-planning-time code reading of
   `src/host/robot_radio/testgui/turn_graphs.py`'s `TurnGraphPanel`/
   `_GraphCanvas` (the current graph-tab implementation) did **not**
   reproduce an obvious structural cause: each of the four tabs owns its
   own `_GraphCanvas` (own `Figure`/`Axes`, disjoint series-key lists —
   `WHEEL_SPEED`/`WHEEL_POS`/`HEADING`/`DISTANCE` do not overlap), all four
   read from one shared, persistent `TurnTraceRecorder` that is only
   cleared by the explicit "Clear traces" button — `_redraw_current()`
   (fired by `QTabWidget.currentChanged` and by the 150 ms dirty-repaint
   timer) only ever touches the CURRENTLY selected canvas's own axes,
   never the recorder. This sprint's first ticket must therefore start
   with **reproduction**, not an assumed fix location — either the bug is
   in a code path not yet found (the issue also names `canvas.py`, which
   turned out on inspection to hold the playfield/avatar canvas, not the
   turn graphs — a second, distinct code path worth checking), or the
   originally-observed build predates a since-landed partial fix and the
   ticket's job is to confirm-and-regression-test rather than redesign.
2. **Telemetry strip charts** (`testgui-telemetry-strip-charts.md`): the
   telemetry section has unused space to the right; the ask is a second,
   tabbed set of the same four graphs, windowed to the last 10 seconds,
   scrolling as new data arrives — reusing the existing series/plotting
   infrastructure rather than a second, divergent implementation.
   Sprint 109 also added four new telemetry-frame fields relevant to this
   pane specifically: `queue_depth`, `active_id`, `exec_state`, and
   `heading_source` (`telemetry.proto`, already on the wire) — none of
   which `TLMFrame.from_pb2()` (`src/host/robot_radio/robot/protocol.py`)
   currently decodes into the parsed frame the GUI consumes. Surfacing
   `heading_source` (OTOS vs. encoder-fallback) in the telemetry pane is a
   standing stakeholder requirement (visibility of which sensor is
   currently heading-truth) — this sprint is the first TestGUI work to
   touch the telemetry pane since that field landed, so closing this wire
   gap and surfacing it is in scope here, not deferred again.
3. **Sim speed-up factor** (`testgui-speedup-factor-broken-at-high-values.md`):
   the GUI only offers five discrete multipliers (1×/2×/5×/10×/20×,
   `__main__.py`'s `sim_speed_combo`). 2× and 5× work; 10× is
   "herky-jerky" and 20× is broken. Sprint-planning-time reading of
   `src/host/robot_radio/io/sim_loop.py`'s `_tick_loop()` shows the pacing
   model: one iteration steps `cycles = max(1, int(speed_factor))` sim
   cycles via `sim_step()`, then sleeps to fit the WHOLE iteration inside
   one `_CYCLE_DURATION_S` (50 ms) budget — the design intent (per that
   method's own comment) is that once N cycles' compute exceeds the 50 ms
   budget, the loop "free-runs" at full compute speed with no sleep, which
   is stated as the desired behavior for a fast tour. That per-iteration
   pacing model looks intentional and correct for the PHYSICS side; the
   reported stutter/breakage is more likely on the CONSUMPTION side — at
   `cycles=10` or `cycles=20`, one iteration now delivers up to 10-20
   telemetry frames in a burst (via `_drain_tlm_into_queue()` →
   `on_telemetry`) instead of the usual one, and the GUI's own
   `QueuedConnection` main-thread bridge (the pattern this project already
   has a documented gotcha about — bare-function `QueuedConnection`
   targets execute on the WORKER thread, not the GUI thread; the fix is
   always a bound-method `QObject` bridge) processes each of those N
   frames as a separate queued signal delivery. This sprint's ticket must
   confirm (not assume) whether the burst-delivery-vs-bridge-processing
   rate is the actual mechanism, using ticket 109-009's own pattern —
   deterministic, non-wall-clock-paced stepping against the real compiled
   sim — to make the speed-factor behavior testable without the same
   real-time-jitter flakiness that sprint 109 diagnosed and eliminated for
   tour completion.

## Solution

Three independently-shippable tickets, ordered so the persistence fix
lands before the strip-chart ticket builds a second consumer of the same
graph infrastructure (per the issue's own explicit coordination note),
with the speed-factor ticket independent of both (no shared code path,
can execute in any position relative to the other two — sequenced last
here purely for topical grouping, not a dependency).

1. **Graph persistence fix** (ticket 001): reproduce the reported
   corruption first, using a headless Qt test (`src/tests/testgui/`,
   `pytest-qt` or the project's existing headless-Qt test pattern — check
   for precedent before introducing a new one) that switches tabs and
   asserts each canvas's rendered/backing data is unchanged by a switch
   away and back. If the bug reproduces in `turn_graphs.py`, fix at the
   root cause found (not a guessed one). If it does NOT reproduce in the
   current `turn_graphs.py` code path, investigate whether the issue's own
   `canvas.py` reference is a second, separate graph implementation this
   sprint-planning pass missed, and report which is the case in the
   ticket's own findings before closing it — this ticket must not close
   on "couldn't repro, assumed fixed" without checking for a second code
   path.
2. **Telemetry-pane rolling strip charts** (ticket 002, depends on 001):
   add a second `QTabWidget` of four graphs (wheel speed, wheel position,
   heading, distance) in the telemetry section's unused right-hand space,
   tabbed the same way as playfield-mode tabs, each a rolling 10-second
   window of the SAME underlying series the top graphs already record
   (reuse `TurnTraceRecorder`/`_GraphCanvas` — a windowed view over the
   existing accumulator, not a second data source or a second recorder).
   Also plumbs `heading_source` (and, since they ride the same frame at
   no extra cost, `queue_depth`/`active_id`/`exec_state`) from
   `telemetry.proto` into `TLMFrame.from_pb2()` and surfaces
   `heading_source` visibly in the telemetry pane (OTOS vs. encoder-
   fallback indicator), closing the standing stakeholder visibility
   requirement for this sensor.
3. **Sim speed-up factor fix** (ticket 003, independent): confirm the
   burst-delivery-vs-`QueuedConnection`-bridge hypothesis above (or find
   the real mechanism if it's wrong) using a deterministic sim-stepping
   test harness modeled on ticket 109-009's pattern, then fix so all five
   offered multipliers (up to 20×) scale the effective sim rate smoothly
   without stutter or breakage.

## Success Criteria

- [ ] Switching away from and back to any of the four graph tabs preserves
      that graph's complete, correct accumulated history — verified by an
      automated headless test, not just manual inspection.
- [ ] A second, tabbed set of the same four graphs exists in the
      telemetry section's previously-unused right-hand space, each a
      rolling window showing at most the last 10 seconds of data,
      scrolling as new data arrives.
- [ ] `heading_source` is visibly surfaced in the telemetry pane
      (OTOS vs. encoder-fallback), wired from `telemetry.proto`'s
      `heading_source` field through `TLMFrame` to the GUI.
- [ ] All five Sim speed-up multipliers (1×, 2×, 5×, 10×, 20×) advance
      the simulation smoothly and proportionally, with no stutter and no
      breakage at the higher end — verified by a deterministic
      (non-wall-clock-flaky) test.
- [ ] No firmware (`src/firm/`) changes are required by any ticket in this
      sprint; if implementation reveals one is needed, it is flagged to
      the team-lead rather than made silently (this sprint's own
      `DESIGN.md` obligation is N/A unless that flag is raised).
- [ ] The existing `src/tests/testgui/` suite (1191 tests green on master
      per the team-lead's own dispatch note) stays green throughout.

## Scope

### In Scope

- Reproduction, root-cause, and fix for graph-tab-switch data corruption
  (`turn_graphs.py`'s `TurnGraphPanel`/`_GraphCanvas`, or wherever the
  real bug is found to live).
- A new rolling 10-second strip-chart tab set in the telemetry section,
  reusing the existing series/plotting infrastructure.
- Wiring `queue_depth`/`active_id`/`exec_state`/`heading_source` from
  `telemetry.proto` into `TLMFrame.from_pb2()`, and surfacing
  `heading_source` visibly in the telemetry pane.
- Diagnosis and fix for the Sim speed-up factor's stutter (10×) and
  breakage (20×), including a deterministic test harness for it.
- Any headless/Qt test infrastructure needed to make the above three
  verifiable without manual GUI interaction or wall-clock timing
  dependence.

### Out of Scope

- Any firmware (`src/firm/`) change — flagged explicitly to the team-lead
  if a ticket's implementer concludes one is actually required.
- New telemetry-pane content beyond `heading_source` (and, opportunistically,
  `queue_depth`/`active_id`/`exec_state` if cheaply plumbed alongside it) —
  no broader telemetry-pane redesign.
- Adding speed-up multipliers beyond the existing five, or making the
  selector continuous/free-entry — fixing the existing five to work
  correctly is this sprint's bar, not expanding the control's range.
- Any change to the playfield-mode tabs' own implementation beyond
  confirming (ticket 001) whether they are or are not the actual site of
  the persistence bug.

## Test Strategy

- **Headless Qt tests** (`src/tests/testgui/`): tab-switch persistence
  regression test (ticket 001); strip-chart windowing test — verify a
  window boundary (>10 s old points excluded, recent points present)
  independent of the top graphs' own unbounded history (ticket 002);
  `heading_source`/`queue_depth`/`active_id`/`exec_state` decode test on
  `TLMFrame.from_pb2()` (ticket 002).
- **Deterministic sim-stepping tests** (ticket 003): model directly on
  ticket 109-009's own pattern — drive the real compiled sim
  (`src/sim/build/libfirmware_host`) via `SimLoop`/`sim_ctypes.cpp` with
  explicit `step(cycles)` calls (`start_tick_thread=False` or equivalent),
  not wall-clock pacing, so the speed-factor fix is verified by counting
  actual sim cycles advanced / frames delivered per unit of test time,
  not by racing a real timer.
- **Full regression**: `uv run python -m pytest src/tests/testgui/` (and
  the full suite) must stay green after each ticket.

## Architecture

This sprint is host/Python + Qt only — no new subsystem, no firmware
change, no wire-schema addition (the four telemetry fields ticket 002
wires up already exist on `telemetry.proto`, landed by sprint 109). The
write-up below stays proportionate to that: a lighter pass than a
firmware sprint's, but still naming the modules touched, their
boundaries, and the risk each ticket carries.

**Step 1 — Understand the problem.** Recapped in Problem/Solution above:
three independent TestGUI defects/gaps, discovered/motivated by sprint
109's changes to the TestGUI's world (MOVE-queue tours, new telemetry
fields, deterministic sim stepping as the proven pattern for eliminating
wall-clock-driven flakiness).

**Step 2 — Identify responsibilities.** Three responsibility groups, each
already cleanly separated in the existing code and staying that way:
1. *Graph-tab data lifecycle* (`turn_graphs.py`'s `TurnTraceRecorder`/
   `TurnGraphPanel`/`_GraphCanvas`) — owns per-series accumulation and
   which axes get redrawn on a tab switch; changes only with tab-switch/
   redraw policy, not with what data is recorded.
2. *Telemetry-pane content* (`telemetry_panel.py` + the new strip-chart
   tabs + `TLMFrame`'s wire-to-Python field decode) — owns what's
   displayed in the telemetry section and which wire fields reach it;
   changes with what the pane shows, not with how graphs redraw.
3. *Sim pacing* (`sim_loop.py`'s `_tick_loop()` + whatever GUI-side
   telemetry-consumption path turns out to be the actual bottleneck) —
   owns how fast simulated time advances relative to wall time; changes
   only with pacing/throughput policy.
   These three responsibility groups do not share code paths with each
   other (confirmed by inspection at planning time), which is why the
   three tickets can be sequenced almost independently — the only real
   coupling is ticket 002 reusing ticket 001's (by-then-fixed) graph
   infrastructure rather than building a second, parallel one.

**Step 3 — Subsystems and modules.**

| Module | Purpose (one sentence) | Boundary | Use cases served |
|---|---|---|---|
| `turn_graphs.py` (`TurnTraceRecorder`/`TurnGraphPanel`/`_GraphCanvas`) | Accumulates and displays the four live time-series graphs. | Owns per-series data buffers and per-tab redraw; Qt-free `TurnTraceRecorder` is independently testable from the `QWidget` layer. | SUC-001 |
| Telemetry strip-chart tabs (new, `telemetry_panel.py` or a sibling module) | Shows a rolling last-10-second window of the same four series in the telemetry section. | A windowed VIEW over `TurnTraceRecorder`'s existing data, not a second recorder; owns only the windowing/scroll behavior and its own tab set. | SUC-002 |
| `TLMFrame.from_pb2()` (`robot/protocol.py`) | Decodes a wire `telemetry.proto` `Telemetry` message into the GUI's parsed frame shape. | Adapter only — bends the wire shape to the existing `TLMFrame` dataclass fields, per its own documented convention; this sprint adds four more fields to that adapter, no redesign. | SUC-002 |
| `sim_loop.py`'s `_tick_loop()` + GUI telemetry-consumption bridge | Paces simulated time relative to wall time at a configurable speed factor, and delivers the resulting telemetry to the GUI. | `_tick_loop()` owns cycle-stepping and pacing; the GUI-side `QueuedConnection` bridge (existing pattern, see Design Rationale) owns cross-thread delivery to Qt widgets. Ticket 003 must determine which side (or both) needs to change. | SUC-003 |

No component/dependency-graph Mermaid diagram: none of the three modules
above changes its dependency DIRECTION relative to the others (each
ticket's change is internal to one module's own responsibility), and
none of this sprint's work touches `src/firm/`, so the firmware
dependency diagram in `src/firm/DESIGN.md` is unaffected. No
entity-relationship diagram: no persisted data model changes (the GUI
holds in-memory series buffers only).

**Step 5 — What Changed / Why / Impact / Migration.**

*What changed:* a fix to graph-tab redraw/data lifecycle (exact diff
depends on ticket 001's own reproduction findings); a new rolling
strip-chart tab set in the telemetry pane, plus four new `TLMFrame`
fields; a fix to sim-speed-factor pacing and/or GUI-side telemetry
consumption at high multipliers.

*Why:* three independently-reported usability defects/gaps, two of them
directly motivated by sprint 109 (new telemetry fields to surface;
deterministic stepping as the now-proven pattern for testing timing-
sensitive GUI behavior without wall-clock flakiness).

*Impact on existing components:* `turn_graphs.py` (ticket 001 fix, exact
scope pending reproduction); `telemetry_panel.py` (new tabs, ticket 002);
`protocol.py`'s `TLMFrame`/`from_pb2()` (four new fields, purely
additive — existing fields/behavior unchanged, matching that method's
own "adapter, not a redesign" convention); `sim_loop.py` and/or the GUI's
`QueuedConnection` telemetry bridge (ticket 003, exact scope pending
confirmation of the burst-delivery hypothesis).

*Migration concerns:* None. No persisted data model, no wire-schema
change (the four `telemetry.proto` fields ticket 002 wires up already
shipped in sprint 109 — this sprint only catches the host-side parser up
to them), no deployment-sequencing concern (host-only change, no
firmware/host version-skew risk since older firmware simply doesn't set
the new fields and `TLMFrame`'s existing "only sensors present in the
frame are populated" convention already handles that gracefully).

**Step 6 — Design rationale.**

- *Decision: strip charts are a windowed VIEW over the existing recorder,
  not a second data source.* **Context**: the issue itself says "reuse
  the existing series/plotting infrastructure... not a separate data
  source." **Alternatives**: (a) a second `TurnTraceRecorder` instance
  feeding the strip charts independently; (b) one shared recorder, two
  views (full-history top graphs, windowed strip charts). **Why this
  choice**: (b) — avoids double-accumulating every telemetry frame,
  avoids the two views ever disagreeing about what was actually recorded,
  and is what the issue explicitly asks for. **Consequences**: the strip
  chart's windowing logic must filter the shared recorder's series by a
  trailing-10-second time slice at redraw time rather than owning its own
  bounded buffer — a redraw-time computation, not a different accumulation
  policy, keeping `TurnTraceRecorder` itself unchanged.
- *Decision: fix ticket 001 before building ticket 002 on top of the same
  infrastructure.* **Context**: the persistence issue's own notes ask for
  this coordination explicitly. **Alternatives**: (a) build the strip
  charts first, fix persistence separately; (b) fix persistence first,
  then build strip charts on the corrected foundation. **Why this
  choice**: (b) — building a second consumer (windowed tabs) of
  infrastructure known to have a data-corruption bug risks either
  propagating the same bug into the new tabs or requiring the fix to be
  re-applied twice. **Consequences**: ticket 002 has a real `depends-on`
  edge on ticket 001, unlike ticket 003 which has none.
- *Decision: ticket 003 must confirm its hypothesis with a deterministic
  test before fixing, not fix based on the sprint-planning-time reading
  alone.* **Context**: sprint-planning-time inspection of `sim_loop.py`
  produced a plausible mechanism (burst telemetry delivery vs.
  `QueuedConnection` bridge throughput) but did NOT trace the GUI-side
  consumption path deeply enough to confirm it — this is a hypothesis,
  not a diagnosis. **Alternatives**: (a) implement a fix for the
  hypothesized mechanism directly; (b) build the deterministic test
  harness first, use it to confirm (or refute and re-diagnose) the
  mechanism, then fix. **Why this choice**: (b) — ticket 109-009's own
  experience (six real bugs found by building a real reproduction harness
  before fixing anything) is directly instructive: guessing at a fix for
  a timing-shaped bug without a deterministic repro risks fixing the
  wrong thing and declaring victory on vibes. **Consequences**: ticket
  003's first deliverable is the test harness itself, not a patch.

**Step 7 — Open questions.**

1. Whether `turn_graphs.py`'s `TurnGraphPanel` is actually the site of the
   persistence bug, or whether the issue's own `canvas.py` reference
   points at a second, not-yet-found graph implementation, is unresolved
   at planning time — ticket 001's first job is to resolve this, not
   assume either answer.
2. Whether the sim speed-up factor's stutter/breakage is really the
   burst-telemetry-vs-`QueuedConnection`-bridge mechanism hypothesized
   above, or something else entirely (e.g. a GUI-side redraw timer
   competing for the main thread, or `_drain_tlm_into_queue()`'s own
   bounded-queue drop-oldest policy interacting badly with a burst), is
   unresolved at planning time — ticket 003's deterministic harness must
   establish the real mechanism before fixing it.
3. Whether `queue_depth`/`active_id`/`exec_state` are worth surfacing
   visibly in the telemetry pane alongside `heading_source` (which IS a
   standing requirement) or just decoded-and-available-but-not-yet-
   displayed is left to ticket 002's implementer — decoding all four into
   `TLMFrame` is in scope regardless (cheap, same frame, same adapter
   method), but only `heading_source`'s VISIBLE surfacing is a hard
   acceptance criterion.

## Use Cases

### SUC-001: Operator switches between graph tabs without losing history
Parent: UC (TestGUI observability)

- **Actor**: TestGUI operator
- **Preconditions**: A drive/tour has been running long enough for at
  least two of the four graph tabs to have accumulated distinct data.
- **Main Flow**:
  1. Operator views the wheel-speed graph; data accumulates.
  2. Operator switches to another graph tab (wheel position / heading /
     distance).
  3. Operator switches back to the wheel-speed graph.
- **Postconditions**: The wheel-speed graph shows its complete, correct,
  unmutated accumulated history — identical to what it would have shown
  had the operator never switched away.
- **Acceptance Criteria**:
  - [ ] An automated headless test drives this exact switch-away/switch-
        back sequence and asserts the graph's underlying data (not just
        its rendered pixels) is unchanged.

### SUC-002: Operator monitors recent behavior via telemetry-pane strip charts, including active heading source
Parent: UC (TestGUI observability)

- **Actor**: TestGUI operator
- **Preconditions**: Telemetry is streaming (real or Sim transport).
- **Main Flow**:
  1. Operator looks at the telemetry section's right-hand tab set
     (wheel speed / wheel position / heading / distance).
  2. Each tab shows a continuously-scrolling window of at most the last
     10 seconds of that series — older data scrolls off the left edge.
  3. Operator also sees which heading source (OTOS or encoder-fallback)
     is currently active, surfaced in the telemetry pane.
- **Postconditions**: The operator has an at-a-glance recent-behavior
  view without needing to scroll or interpret the full-run top graphs,
  and always knows which sensor is currently heading-truth.
- **Acceptance Criteria**:
  - [ ] A windowing test confirms points older than 10 seconds are
        excluded from the strip-chart view while still present in the
        (unaffected) full-history top-graph recorder.
  - [ ] `heading_source` is decoded from `telemetry.proto` into
        `TLMFrame` and visibly displayed in the telemetry pane.

### SUC-003: Operator runs the sim at any offered speed-up factor without stutter or breakage
Parent: UC (TestGUI Sim-mode operation)

- **Actor**: TestGUI operator (Sim transport)
- **Preconditions**: Connected via the Sim transport.
- **Main Flow**:
  1. Operator selects a speed-up factor (1×, 2×, 5×, 10×, or 20×).
  2. The simulation advances at proportionally the selected multiple of
     wall-clock rate, smoothly, at every offered multiplier.
- **Postconditions**: No multiplier stutters or fails to advance the
  simulation faster than 1×.
- **Acceptance Criteria**:
  - [ ] A deterministic (non-wall-clock-paced) test confirms cycles-
        advanced-per-unit-test-time scales correctly with the selected
        factor at all five offered multipliers, including 10× and 20×.

## GitHub Issues

(None linked yet — this sprint's issues are internal `clasi/issues/*.md`
files, listed in frontmatter `issues:`.)

## Definition of Ready

- [x] Sprint planning document is complete (sprint.md, including its
      Architecture and Use Cases sections)
- [x] Architecture review passed — recorded by the team-lead (per
      dispatch instruction), 2026-07-17.
- [x] Stakeholder has approved the sprint plan — recorded by the
      team-lead (pre-authorized), 2026-07-17.

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Fix graph-tab data persistence across view switches | — |
| 002 | Telemetry-pane rolling 10-second strip charts + heading-source visibility | 001 |
| 003 | Fix Sim speed-up factor stutter/breakage at 10×/20× | — |

Tickets execute serially in the order listed.
