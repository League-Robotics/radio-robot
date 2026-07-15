---
id: "002"
title: "Tour driver: planner/tour.py owns tour geometry, chains legs through the executor, closure bookkeeping"
status: open
use-cases: [SUC-033]
depends-on: ["001"]
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

# Tour driver: planner/tour.py owns tour geometry, chains legs through the executor, closure bookkeeping

## Description

`TOUR_1`/`TOUR_2` (currently `host/robot_radio/testgui/commands.py`) are
ordered lists of legacy `D`/`RT` text-verb wire strings — the tour
GEOMETRY is a real, tuned, reusable asset, but both verbs are retired
(sprint 102/103 deleted the on-robot `Motion::SegmentExecutor` and the
`segment`/`replace` envelope arms `testgui/binary_bridge.py` still targets
for them — confirmed by this sprint's own reading of `protos/
envelope.proto`: the current `CommandEnvelope.cmd` oneof carries only
`twist`/`config`/`stop`). This ticket builds the module that makes the
geometry drivable again: a new `host/robot_radio/planner/tour.py` that
OWNS `TOUR_1`/`TOUR_2`'s raw wire-string data (moved here from
`testgui/commands.py` — architecture-update.md Decision 3, corrected
during that document's own self-review to keep the dependency direction
`[Presentation]→[Domain]`, not the reverse), parses it into typed legs,
and chains each leg through a `StreamingExecutor` (ticket 001's fixes
applied), recording per-leg outcomes and the tour's own whole-run pose
closure.

This module is the single, shared per-leg execution loop BOTH the TestGUI
(ticket 003) and the bench script (ticket 005) call — no duplicated
per-leg execution/telemetry-capture logic between them (architecture-
update.md's own SUC-033 acceptance criterion, structurally enforced by
this module existing as a shared dependency of both). Serves SUC-033.

## Acceptance Criteria

- [ ] `planner/tour.py` owns `TOUR_1`/`TOUR_2`'s raw wire-string geometry
      (moved from `testgui/commands.py`, byte-for-byte — same leg
      distances/angles, same order).
- [ ] A pure parser converts a `TOUR_1`/`TOUR_2`-shaped wire-string list
      into an ordered sequence of typed legs (signed straight distance in
      mm, or signed turn angle in degrees, matching `RT`'s own sign
      convention — positive CCW/left). Unit-tested directly against
      `tour.TOUR_1`/`TOUR_2` (regression-protects the geometry itself
      against silent drift) — e.g. asserts leg count, first/last leg
      values, and total leg count matches the wire-string list's own step
      count.
- [ ] A `run_tour(transport, params, heading, legs, ...)`-shaped public
      function (exact signature implementer's call) runs each leg's
      profile (`profile_for_distance`/`profile_for_turn`) through a
      `StreamingExecutor` built from the caller-supplied `transport`/
      `params`/`heading`, in order, stopping immediately — no further legs
      attempted — on any leg outcome other than `RunOutcome.COMPLETED`,
      and reporting which leg index and what outcome caused the stop.
- [ ] Tour closure (position delta + heading delta) is computed: the
      measured pose (`TLMFrame.pose`) is captured once immediately before
      leg 1's `begin()` (the tour's own closure baseline — `App::Odometry`
      never resets across a boot session, so this is always a RELATIVE
      baseline, never an absolute zero) and once after the final leg's
      settle window; the delta between the two is returned to the caller.
- [ ] The per-leg run loop accepts an OPTIONAL per-tick row-callback hook
      (or equivalent extension point, implementer's call) so a caller that
      wants a full commanded-vs-measured trace (ticket 005's bench script)
      can capture one without `tour.py` itself knowing about CSV/JSON file
      formats, and a caller that only wants per-leg progress narration
      (ticket 003's TestGUI) can ignore it.
- [ ] `tour.py` never imports `NezhaProtocol`/`SerialConnection`/
      `SimConnection` directly — it accepts a `TwistTransport`-compatible
      object from its caller, the same pattern `executor.py` itself
      already uses.
- [ ] `testgui/commands.py`'s `TOURS: dict[str, list[str]]` becomes a read
      FROM `planner.tour.TOUR_1`/`TOUR_2` (GUI labeling only) — the
      corrected `[Presentation]→[Domain]` direction. No other field of
      `commands.py` changes.
- [ ] 100% unit-tested under `tests/unit/`, no hardware/sim dependency for
      the parsing/chaining/closure logic itself (a `FakeTransport` double,
      mirroring `tests/unit/test_planner_executor.py`'s own convention).
- [ ] Full suite (`uv run python -m pytest`) stays green.

## Implementation Plan

### Approach

1. Add `TOUR_1: list[str]`/`TOUR_2: list[str]` to `planner/tour.py`
   (verbatim move from `testgui/commands.py` — do not hand-retype the
   values; copy them exactly to avoid a transcription error in exactly the
   kind of data where one would be easy to make and hard to notice).
2. `parse_tour(wire_steps: list[str]) -> list[TourLeg]` where
   `TourLeg` is a small frozen dataclass/NamedTuple with `kind: Literal
   ["distance", "turn"]` and `value: float` (signed mm or signed degrees).
   Parses `"D <l> <r> <mm>"` (extract signed `mm` — TOUR_1/2's own D legs
   are all-positive/forward; a future negative-distance tour is not
   precluded, just untested by TOUR_1/2's own data) and `"RT <cdeg>"`
   (signed, `/100` → degrees). Any other verb in a tour list raises
   `ValueError` immediately (defense in depth — a future tour author who
   adds an unsupported step gets a clear error, not silent misparsing).
3. `run_tour()` mirrors `profiled_motion_verify.py`'s own `run_leg()`
   shape (that function is effectively promoted to production here,
   generalized to an ordered leg list): for each `TourLeg`, build
   `ProfileLimits` (straight legs: a bench-safe default `v_max`/`a_max`,
   e.g. matching `profiled_motion_verify.py`'s own `DEFAULT_V_MAX=150`/
   `DEFAULT_A_MAX=400`, honoring the ORIGINAL per-leg speed encoded in the
   `D` wire string if present — `executor.py`'s own defense-in-depth
   ceiling clamp already caps anything unsafe, so faithfully preserving
   the tour's authored speed intent costs nothing extra in safety; turn
   legs: `RT` carries no rate field, so use a bench-safe default
   `omega_max`/`alpha_max`, e.g. matching `DEFAULT_OMEGA_MAX=1.0`/
   `DEFAULT_ALPHA_MAX=3.0` — NOT `PlannerParams`' own more aggressive
   `omega_max=2.0`/`alpha_max=6.0` hard ceilings), generate the profile via
   `profile_for_distance`/`profile_for_turn`, then run it through a
   `StreamingExecutor.begin()`/manual `tick()` loop (not the blocking
   `.run()` convenience — the manual loop is what allows the optional
   per-tick row-callback hook).
4. Capture `heading.measured_heading(ex.latest_frame)` and `TLMFrame.pose`
   immediately before leg 1's `begin()` and again after the final leg's
   settle window (mirroring `profiled_motion_verify.py`'s own settle-window
   convention — the terminal `tick()` sends `stop()` and returns
   immediately, but the PLANT needs real time to actually decelerate, so
   poll telemetry briefly after the loop ends before reading final pose).
5. Update `testgui/commands.py`: replace `TOUR_1`/`TOUR_2` definitions with
   `from robot_radio.planner.tour import TOUR_1, TOUR_2`; `TOURS` dict
   unchanged in shape (`{"Tour 1": TOUR_1, "Tour 2": TOUR_2}`).

### Files to Create

- `host/robot_radio/planner/tour.py`

### Files to Modify

- `host/robot_radio/testgui/commands.py` — `TOUR_1`/`TOUR_2` become an
  import from `planner.tour`; `TOURS` unchanged in shape.

### Testing Plan

- New `tests/unit/test_planner_tour.py`: `parse_tour()` against `tour.
  TOUR_1`/`TOUR_2` directly (leg count, first/last leg values); `run_tour()`
  against a `FakeTransport` double for: (a) a clean multi-leg run —
  `COMPLETED` outcome for every leg, closure computed; (b) a leg that
  returns `FAULT`/`OVERSHOOT` mid-tour — remaining legs are NOT attempted,
  the failing leg index/outcome is reported.
- `tests/unit/test_commands.py` (existing) — add/update a case confirming
  `commands.TOURS["Tour 1"] is planner.tour.TOUR_1` (or equal, implementer's
  call) after the move.
- Full suite: `uv run python -m pytest`.

### Documentation Updates

- None required beyond this ticket's own docstrings (module-level docstring
  on `planner/tour.py` explaining its scope/boundary, matching every other
  `planner/` module's own documentation convention).
