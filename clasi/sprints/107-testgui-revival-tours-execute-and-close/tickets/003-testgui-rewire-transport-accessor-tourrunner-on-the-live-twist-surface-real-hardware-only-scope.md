---
id: '003'
title: 'TestGUI rewire: transport accessor + _TourRunner on the live twist surface,
  real-hardware-only scope'
status: in-progress
use-cases:
- SUC-034
depends-on:
- '002'
github-issue: ''
issue: ''
completes_issue: true
exception:
  thrown_by: programmer
  thrown_at: '2026-07-15T17:08:50.249871+00:00'
  attempted: 'Implemented all of ticket 003''s own code scope: _HardwareTransport.protocol
    (transport.py), _TourRunner.run() rewired onto planner.tour.run_tour() with parse_tour()/on_leg/row_callback/should_stop,
    removal of _wait_for_idle()/SPINUP_S/POLL_S/SNAP_REPLY_TIMEOUT_S/MOVE_TIMEOUT_S/STREAM_FRESH_S
    (grep-verified no remaining callers repo-wide), Sim-only tour-button gating in
    _on_connect() mirroring operations.py''s own is_sim_transport() pattern, and investigated+resolved
    the telemetry-drain competition (Open Question 1) by adding suspend_telemetry_reader()/resume_telemetry_reader()
    to _HardwareTransport so a tour becomes the shared binary-TLM-queue''s sole consumer
    and re-feeds frames through the existing on_telemetry Qt-bridge path. Verified
    via: uv run python -m pytest (703 passed), ast.parse on both changed files, an
    isolated (non-hardware) threading test proving suspend/resume actually pauses/resumes
    the reader loop''s queue drain, and a QT_QPA_PLATFORM=offscreen headless build
    of the full window (with legacy_render/legacy_verbs stubbed as a throwaway, out-of-band
    TEST SCAFFOLD only, no source change) confirming _build_main_window() constructs
    with no error and tour buttons start correctly disabled/tooltipped. Then attempted
    the ticket''s own mandated step: `just testgui` / `python -m robot_radio.testgui`
    for real, unstubbed -- it crashes immediately on import, before any window appears,
    with ImportError: cannot import name ''legacy_render'' from robot_radio.robot,
    raised from testgui/binary_bridge.py''s own module-level `from robot_radio.robot
    import legacy_render as render` (transport.py imports binary_bridge unconditionally
    at its own module level, so this blocks transport.py, and therefore all of __main__.py,
    from importing at all -- unrelated to whether the tour path itself still calls
    binary_bridge, which it no longer does).'
  conflict: 'architecture-update.md Step 7 Finding 1 explicitly scoped fixing testgui/binary_bridge.py
    OUT of sprint 107 ("This sprint does not claim to fix binary_bridge.py generally
    -- only the tour path is rerouted around it"), on the basis that its D/RT/R/TURN/G
    translators target a segment/replace envelope arm the current 3-arm (twist/config/stop)
    protobuf schema no longer has -- a semantic dead-arm problem. That finding did
    not account for a more severe, separate fact: commit 129cbcb3 (feat(104-002):
    delete retired legacy-translator/rogo-proxy modules, landed ~6h before this sprint
    was created) deleted robot/legacy_render.py and robot/legacy_verbs.py WHOLESALE
    with no replacement, and binary_bridge.py (never updated to match) still imports
    both at module level and depends on ~800 lines of their now-deleted surface throughout
    (tokenizing, one-arm verb dispatch, all reply rendering). The practical effect:
    binary_bridge.py cannot be imported AT ALL right now, which means transport.py
    cannot be imported, which means the entire TestGUI cannot launch -- not a narrower
    "some verbs behave wrong" problem the architecture doc anticipated, but "the GUI
    does not start," which makes AC9''s mandatory real-hardware demonstration ("clicking
    a tour button on a connected real robot actually drives it") impossible to perform
    as specified, through no defect in this ticket''s own (correctly scoped, binary_bridge-free)
    tour-path implementation. Properly fixing binary_bridge.py is a substantial, separate
    rewrite against the current 3-arm wire (comparable in scope to the ~425-line host/robot_radio/io/repl.py
    the stakeholder is visibly building out-of-process, uncommitted, in this same
    checkout right now, as an apparent from-scratch replacement for exactly this legacy-translation
    gap) -- well outside this ticket''s own Files-to-Modify list (transport.py, __main__.py)
    and directly overlapping that concurrent work, so it was deliberately not attempted
    here without a decision from the team-lead/stakeholder on how to proceed (narrow
    unblock now vs. defer bench verification vs. coordinate with the repl.py effort).'
  surface: user-visible
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# TestGUI rewire: transport accessor + _TourRunner on the live twist surface, real-hardware-only scope

## Description

`testgui/__main__.py`'s `_TourRunner` currently sends each `D`/`RT` tour
step as a literal wire string through `testgui/binary_bridge.py`'s
`translate_command()`, which builds a `segment`/`replace` envelope for it
— an arm that no longer exists on the wire (see ticket 002's own
Description). `_wait_for_idle()`'s SNAP-poll completion detection is
mechanism the new `StreamingExecutor`-based run loop does not need at all:
`run_tour()` (ticket 002) already knows synchronously when a leg finishes.

This ticket rewires `_TourRunner.run()` to call ticket 002's `planner.
tour.run_tour()` against the connected transport's underlying
`NezhaProtocol` (a new narrow accessor on `_HardwareTransport`) in place
of the old wire-string-per-step + SNAP-poll loop, and scopes tour buttons
to real-hardware transports only this sprint — `SimTransport`'s backing
library was deleted wholesale at sprint 102 ticket 005 (`git show
72d8be7e --stat`) and has no working foundation to rewire onto (see
architecture-update.md Decision 1) — a deliberate, documented scope
boundary: tour buttons stay disabled (clear tooltip) when connected via
Sim, never a crash or silent no-op. Serves SUC-034 — this is the ticket
that makes the stakeholder's own literal acceptance wording ("demonstrate
that the tours... actually execute") true on the bench rig.

## Acceptance Criteria

- [x] `_HardwareTransport` (the `SerialTransport`/`RelayTransport` base)
      gains a narrow accessor exposing its already-constructed
      `NezhaProtocol` instance. `Transport`'s existing `connect()`/
      `send()`/`command()` surface is otherwise unchanged.
- [x] `_TourRunner.run()`'s body calls `planner.tour.run_tour()` against
      that accessor — no `D`/`RT` wire string, no `binary_bridge.
      translate_command()` call, no `SNAP`-poll, anywhere in the tour code
      path.
- [x] `_wait_for_idle()` and its SNAP-poll machinery are removed from
      `_TourRunner` — nothing else in the tree calls it (grep-verified
      before removal).
- [x] `_TourRunner`'s public shape (`log_line`/`finished` Qt signals,
      `stop()`) is UNCHANGED — only its internal `run()` implementation
      changes, so no other code in `__main__.py` that references
      `_TourRunner` needs to change.
- [x] Progress narration (`[TOUR] ... leg i/N: ...`) is emitted from
      ticket 002's own per-leg outcomes, not from raw wire traffic.
- [x] Tour buttons are disabled with a clear, specific tooltip when
      connected via `SimTransport` (e.g. "Tours require a real-hardware
      connection this sprint — Sim's backing sim library was removed at
      sprint 102") — enabled (as today) for `SerialTransport`/
      `RelayTransport`.
- [x] Stop Tour still re-enables the tour buttons synchronously
      (`testgui-tour-stop-reactivation.md`'s existing contract) —
      `_TourRunner.stop()` propagates to `run_tour()`'s own stop/preempt
      hook (ticket 002).
- [x] Investigate whether `StreamingExecutor`'s continuous telemetry drain
      (inside `run_tour()`) competes with or complements the GUI's
      existing `on_telemetry` canvas/avatar-update callback path
      (architecture-update.md Step 7, Open Question 1) — resolve one way
      or the other and document the finding in this ticket's own
      Completion Notes; if they compete, either tap the existing callback
      path to feed the tour driver or drive canvas updates from the
      driver's own latest-frame state instead. **Resolved at the code
      level and verified with an isolated (non-hardware) reader-thread
      test — see Completion Notes; empirical on-bench confirmation is
      blocked, see AC below and the thrown exception.**
- [ ] Demonstrated end to end on the bench rig against real hardware
      (`.claude/rules/hardware-bench-testing.md`) as part of this ticket's
      own verification — clicking a tour button on a connected real robot
      actually drives it. **BLOCKED — see Completion Notes and the thrown
      exception: `just testgui`/`python -m robot_radio.testgui` cannot even
      launch on this branch right now (pre-existing, unrelated regression).**

## Implementation Plan

### Approach

1. `transport.py`: add a method/property to `_HardwareTransport` (e.g.
   `build_twist_transport()` or a `protocol` property returning the
   `NezhaProtocol` instance it already constructs in `connect()`) — narrow,
   read-only, no new state.
2. `__main__.py`: rewrite `_TourRunner.run()` to look up the tour's `TourLeg`
   list (via `planner.tour.parse_tour(commands.TOURS[self._name])` or
   equivalent) and call `planner.tour.run_tour(transport_handle, params,
   heading, legs, on_leg=<narration callback>)`, emitting `log_line` from
   the callback instead of per-wire-string sends. Remove
   `_wait_for_idle()`/`SPINUP_S`/`POLL_S`/`SNAP_REPLY_TIMEOUT_S`/
   `STREAM_FRESH_S` (no longer used).
3. Tour-button construction (the `for _tour_name in TOURS:` loop, ~line
   730) gains a transport-type check in its enable/disable logic — reuse
   whatever pattern `operations.is_sim_transport()` already establishes
   for other Sim-specific gating in this file.
4. Investigate telemetry-path interaction (Open Question 1) by tracing
   `_HardwareTransport`'s existing `on_recv`/`on_telemetry` wiring against
   `NezhaProtocol.read_pending_binary_tlm_frames()`'s own queue — confirm
   empirically on the bench (a running tour with the canvas visibly
   tracking, or not) rather than by inspection alone.

### Files to Modify

- `host/robot_radio/testgui/transport.py` — new `_HardwareTransport`
  accessor.
- `host/robot_radio/testgui/__main__.py` — `_TourRunner.run()` rewired;
  `_wait_for_idle()` removed; tour-button enable/disable gains a
  transport-type check.

### Testing Plan

- Ticket 004 owns the full headless GUI test rewrite (a separate ticket,
  depends on this one) — this ticket's own verification is primarily the
  bench demonstration (below), plus any quick manual/headless smoke check
  needed to iterate without a full bench cycle per edit.
- Bench verification (`.claude/rules/hardware-bench-testing.md`): connect
  via Serial/Relay, confirm tour buttons enabled; connect via Sim, confirm
  tour buttons disabled with the expected tooltip; run Tour 1 on real
  hardware end to end, confirm canvas/avatar tracks and Stop Tour
  re-enables buttons synchronously when clicked mid-run.
- `uv run python -m pytest` (existing suite, pre-ticket-004-rewrite tests
  in `tests/testgui/` remain skipped/excluded as they are today — ticket
  004 addresses that).

### Documentation Updates

- Update the tour-button tooltip text in `__main__.py` itself (already
  covered above).
- No other doc changes required.

## Completion Notes

### Implementation

- `host/robot_radio/testgui/transport.py`: `_HardwareTransport` gains a
  read-only `protocol` property returning `self._proto` (the already-
  constructed `NezhaProtocol`, or `None` before `connect()`) — no adapter,
  it satisfies `executor.py`'s `TwistTransport` structural `Protocol`
  as-is. `connect()`/`disconnect()`/`send()`/`command()` are otherwise
  byte-for-byte unchanged.
- `_TourRunner.run()` (`__main__.py`) rewritten: `parse_tour(self._steps)`
  builds the `TourLeg` list, a `PlannerParams()`/`HeadingCorrector(params,
  robot_config=get_robot_config())` pair is built fresh per run (picking
  up whichever robot is selected in the Robot combo, including its
  `geometry.otos_untrusted` heading-source choice), and `run_tour()` is
  called against `transport.protocol` with `on_leg`/`row_callback`/
  `should_stop` hooks. `_wait_for_idle()` and
  `SPINUP_S`/`POLL_S`/`SNAP_REPLY_TIMEOUT_S`/`MOVE_TIMEOUT_S`/
  `STREAM_FRESH_S` are deleted outright — grep-verified (repo-wide, plus
  `archive/`) that nothing else calls `_wait_for_idle` or reads those
  constants; the only remaining hits are two pre-existing doc-comment
  mentions in `protocol.py`/`binary_bridge.py` that name the old method
  historically (not calls) — left as-is, out of this ticket's file scope.
  `_TourRunner`'s public shape (`log_line`/`finished` signals, `stop()`,
  constructor signature) is byte-for-byte unchanged; `_make_tour_handler`
  needed no edits.
- Tour-button gating: the button-creation loop now sets a hardware-mode
  tooltip via `_tour_hw_tooltip(name)`; `_on_connect()` overrides the
  generic `_send_buttons` enable pass right after it runs, disabling every
  tour button with `_TOUR_SIM_TOOLTIP` when `is_sim_transport(transport)`,
  re-enabling with the hardware tooltip otherwise — same pattern
  `operations.py`'s `set_connected()` already uses for its own Sim-only
  gating (e.g. the Sync Pose button).

### Open Question 1 (telemetry-drain competition) — investigated and resolved

Traced the drain paths directly (not by inspection alone, though the bench
confirmation step itself is blocked — see below): `_HardwareTransport.
_reader_loop()` and `NezhaProtocol.read_pending_binary_tlm_frames()`
(`protocol.py`) both call `SerialConnection.drain_binary_tlm()` against the
SAME `_binary_tlm_queue` — one non-replayable queue, two independent
consumers. `_reader_loop()` polls every 40ms (`_TLM_DRAIN_INTERVAL_S`),
far faster than `StreamingExecutor`'s own `streaming_interval`-paced
`tick()` (~150ms default) drains during a tour, so **they do compete**,
and left unmanaged `_reader_loop()` wins almost every frame — starving
`run_tour()`'s own heading-feedback/fault-bit/overshoot checks of fresh
telemetry for nearly the whole tour.

Resolution: "drive canvas updates from the driver's own latest-frame state
instead" (the ticket's second offered option). `transport.py` gained
`suspend_telemetry_reader()`/`resume_telemetry_reader()` (a
`threading.Event` `_reader_loop()` checks each iteration, skipping its
drain entirely while set — draining-and-discarding would still steal
frames). `_TourRunner.run()` calls `suspend_telemetry_reader()` before
`run_tour()` (making the tour thread the queue's sole consumer for the
run) and `resume_telemetry_reader()` in a `finally`; its `_on_row()`
`row_callback` forwards every frame `run_tour()` drains straight to
`transport.on_telemetry` — the SAME Qt-bridge (`_pending_frames`/
`_TelemetryBridge.frame_ready`) path `_reader_loop()` normally feeds — so
the canvas/avatar keeps tracking during a tour with `_reader_loop()`
stood down.

Verified WITHOUT hardware (isolated `threading` test against a
`SerialTransport` instance with a fake `_conn`/`_proto`, run standalone —
not part of the committed test suite, a one-off verification script): the
reader thread drains repeatedly under normal operation, stops draining
entirely the instant `suspend_telemetry_reader()` is called, and resumes
draining after `resume_telemetry_reader()` — confirming the mechanism
itself works exactly as designed. **NOT yet confirmed with a live tour
against real hardware** (the AC's own "confirm empirically on the bench"
instruction) — blocked, see below.

### BLOCKED: bench verification (AC9) could not be performed

`just testgui` (`python -m robot_radio.testgui`) currently **cannot
launch at all** on this branch, for a reason entirely unrelated to this
ticket's own changes:

```
File ".../testgui/transport.py", line 132, in <module>
    from robot_radio.testgui import binary_bridge
File ".../testgui/binary_bridge.py", line 53, in <module>
    from robot_radio.robot import legacy_render as render
ImportError: cannot import name 'legacy_render' from 'robot_radio.robot'
```

Root cause: commit `129cbcb3` (`feat(104-002): delete retired
legacy-translator/rogo-proxy modules`, landed ~6 hours before this sprint
was created) deleted `robot/legacy_render.py` AND `robot/legacy_verbs.py`
wholesale (per that commit's own message: "retiring an interface means
gutting it, not preserving a legacy-client translation path") but never
updated `testgui/binary_bridge.py`, which still imports both at module
level and depends on ~800 lines of their deleted surface (tokenizing,
one-arm verb dispatch, all reply rendering) throughout. `transport.py`
imports `binary_bridge` unconditionally at its own module level (for
`_HardwareTransport`'s send/recv log-line rendering) — so `transport.py`,
and therefore the entire TestGUI (`__main__.py` imports `transport.py`),
cannot be imported at all right now, tour path included, regardless of
this ticket's own changes (which correctly no longer call
`binary_bridge.translate_command()` anywhere in the tour path — AC2 holds
on inspection).

This sprint's own `architecture-update.md` (Step 7, Finding 1) already
flagged `binary_bridge.py` as broken, but at a narrower level (the `D`/
`RT`/`R`/`TURN`/`G` translators target a `segment`/`replace` envelope arm
the current 3-arm — twist/config/stop — protobuf schema no longer has) and
explicitly scoped fixing it OUT of sprint 107 ("This sprint does not claim
to fix `binary_bridge.py` generally — only the tour path is rerouted
around it"). That scoping call is sound for the semantic dead-arm problem
it was written against, but did not anticipate — and does not cover — a
full module-level `ImportError` that prevents the module (and therefore
the whole GUI) from loading at all. A proper fix is a substantial,
separate undertaking (rewrite `binary_bridge.py`'s translation/rendering
against the current 3-arm wire, comparable in scope to the ~425-line
`host/robot_radio/io/repl.py` the stakeholder is visibly building
out-of-process, uncommitted, in this same checkout right now as an
apparent from-scratch replacement for exactly this legacy-translation
gap) — well beyond this ticket's own "Files to Modify" list
(`transport.py`, `__main__.py`) and directly overlapping that concurrent
work, so it was not attempted here.

What WAS verified without a live bench session:
- `uv run python -m pytest`: 703 passed (unchanged pass count from before
  this ticket).
- `ast.parse()` on both changed files.
- `QT_QPA_PLATFORM=offscreen`, with `robot_radio.robot.legacy_render`/
  `legacy_verbs` stubbed as an out-of-band TEST SCAFFOLD ONLY (no source
  change) purely to route around the blocker above: `_build_main_window()`
  builds the full window with no error; both tour buttons start disabled
  with the hardware tooltip, matching the un-connected initial state.
- The `transport.py` `protocol` accessor and `suspend_telemetry_reader()`/
  `resume_telemetry_reader()` mechanism verified directly (see Open
  Question 1 section above).

A `throw_ticket_exception` is filed against this ticket for the AC9 bench
gate specifically — the code changes above are complete and believed
correct per every non-hardware verification available, but the mandatory
"clicking a tour button on a connected real robot actually drives it"
demonstration cannot be performed until the GUI can be launched at all.
