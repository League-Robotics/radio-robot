---
id: "003"
title: "TestGUI rewire: transport accessor + _TourRunner on the live twist surface, real-hardware-only scope"
status: open
use-cases: [SUC-034]
depends-on: ["002"]
github-issue: ""
issue: ""
# completes_issue: Controls whether linked issues are archived when this ticket
# is moved to done. Default: true (archive when all referencing tickets are done).
# Set to false (scalar) to suppress archival for ALL linked issues on this ticket.
# Set to a mapping {filename.md: false} to suppress archival per issue filename.
# Use false for tickets that partially address a multi-sprint umbrella issue.
completes_issue: true
# exception: Written by a lower agent when it cannot proceed (see architecture §exception-protocol).
# exception:
#   thrown_by: "programmer"          # "programmer" | "sprint-planner"
#   thrown_at: "2026-05-07T14:23:00Z"
#   attempted: |
#     Description of what was attempted before giving up.
#   conflict: "architecture-update.md §3 — reason the agent is blocked"
#   surface: "internal"              # "user-visible" | "internal"
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

- [ ] `_HardwareTransport` (the `SerialTransport`/`RelayTransport` base)
      gains a narrow accessor exposing its already-constructed
      `NezhaProtocol` instance. `Transport`'s existing `connect()`/
      `send()`/`command()` surface is otherwise unchanged.
- [ ] `_TourRunner.run()`'s body calls `planner.tour.run_tour()` against
      that accessor — no `D`/`RT` wire string, no `binary_bridge.
      translate_command()` call, no `SNAP`-poll, anywhere in the tour code
      path.
- [ ] `_wait_for_idle()` and its SNAP-poll machinery are removed from
      `_TourRunner` — nothing else in the tree calls it (grep-verified
      before removal).
- [ ] `_TourRunner`'s public shape (`log_line`/`finished` Qt signals,
      `stop()`) is UNCHANGED — only its internal `run()` implementation
      changes, so no other code in `__main__.py` that references
      `_TourRunner` needs to change.
- [ ] Progress narration (`[TOUR] ... leg i/N: ...`) is emitted from
      ticket 002's own per-leg outcomes, not from raw wire traffic.
- [ ] Tour buttons are disabled with a clear, specific tooltip when
      connected via `SimTransport` (e.g. "Tours require a real-hardware
      connection this sprint — Sim's backing sim library was removed at
      sprint 102") — enabled (as today) for `SerialTransport`/
      `RelayTransport`.
- [ ] Stop Tour still re-enables the tour buttons synchronously
      (`testgui-tour-stop-reactivation.md`'s existing contract) —
      `_TourRunner.stop()` propagates to `run_tour()`'s own stop/preempt
      hook (ticket 002).
- [ ] Investigate whether `StreamingExecutor`'s continuous telemetry drain
      (inside `run_tour()`) competes with or complements the GUI's
      existing `on_telemetry` canvas/avatar-update callback path
      (architecture-update.md Step 7, Open Question 1) — resolve one way
      or the other and document the finding in this ticket's own
      Completion Notes; if they compete, either tap the existing callback
      path to feed the tour driver or drive canvas updates from the
      driver's own latest-frame state instead.
- [ ] Demonstrated end to end on the bench rig against real hardware
      (`.claude/rules/hardware-bench-testing.md`) as part of this ticket's
      own verification — clicking a tour button on a connected real robot
      actually drives it.

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
