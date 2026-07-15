---
id: '002'
title: 'Tour driver: planner/tour.py owns tour geometry, chains legs through the executor,
  closure bookkeeping'
status: done
use-cases:
- SUC-033
depends-on:
- '001'
github-issue: ''
issue: ''
completes_issue: true
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

- [x] `planner/tour.py` owns `TOUR_1`/`TOUR_2`'s raw wire-string geometry
      (moved from `testgui/commands.py`, byte-for-byte — same leg
      distances/angles, same order).
- [x] A pure parser converts a `TOUR_1`/`TOUR_2`-shaped wire-string list
      into an ordered sequence of typed legs (signed straight distance in
      mm, or signed turn angle in degrees, matching `RT`'s own sign
      convention — positive CCW/left). Unit-tested directly against
      `tour.TOUR_1`/`TOUR_2` (regression-protects the geometry itself
      against silent drift) — e.g. asserts leg count, first/last leg
      values, and total leg count matches the wire-string list's own step
      count.
- [x] A `run_tour(transport, params, heading, legs, ...)`-shaped public
      function (exact signature implementer's call) runs each leg's
      profile (`profile_for_distance`/`profile_for_turn`) through a
      `StreamingExecutor` built from the caller-supplied `transport`/
      `params`/`heading`, in order, stopping immediately — no further legs
      attempted — on any leg outcome other than `RunOutcome.COMPLETED`,
      and reporting which leg index and what outcome caused the stop.
- [x] Tour closure (position delta + heading delta) is computed: the
      measured pose (`TLMFrame.pose`) is captured once immediately before
      leg 1's `begin()` (the tour's own closure baseline — `App::Odometry`
      never resets across a boot session, so this is always a RELATIVE
      baseline, never an absolute zero) and once after the final leg's
      settle window; the delta between the two is returned to the caller.
- [x] The per-leg run loop accepts an OPTIONAL per-tick row-callback hook
      (or equivalent extension point, implementer's call) so a caller that
      wants a full commanded-vs-measured trace (ticket 005's bench script)
      can capture one without `tour.py` itself knowing about CSV/JSON file
      formats, and a caller that only wants per-leg progress narration
      (ticket 003's TestGUI) can ignore it.
- [x] `tour.py` never imports `NezhaProtocol`/`SerialConnection`/
      `SimConnection` directly — it accepts a `TwistTransport`-compatible
      object from its caller, the same pattern `executor.py` itself
      already uses.
- [x] `testgui/commands.py`'s `TOURS: dict[str, list[str]]` becomes a read
      FROM `planner.tour.TOUR_1`/`TOUR_2` (GUI labeling only) — the
      corrected `[Presentation]→[Domain]` direction. No other field of
      `commands.py` changes.
- [x] 100% unit-tested under `tests/unit/`, no hardware/sim dependency for
      the parsing/chaining/closure logic itself (a `FakeTransport` double,
      mirroring `tests/unit/test_planner_executor.py`'s own convention).
- [x] Full suite (`uv run python -m pytest`) stays green.

## Completion Notes

Implemented `host/robot_radio/planner/tour.py`:

- `TOUR_1`/`TOUR_2` moved verbatim (copy, not retyped) from
  `testgui/commands.py`.
- `TourLeg` (frozen dataclass): `kind: Literal["distance","turn"]`,
  `value: float` (signed mm / signed deg), plus an extra `speed: float |
  None` field beyond AC2's minimal description — the D wire string's own
  left/right speed (averaged; TOUR_1/2 always have left==right), `None` for
  turn legs (RT carries no rate field). This lets `run_tour()` honor the
  tour's authored per-leg speed per the Implementation Plan's own Step 3,
  which the two-field `TourLeg` shape in AC2's prose couldn't otherwise
  support — noted here since it's a deliberate reading of "implementer's
  call", not an oversight.
- `parse_tour()`: parses `"D <l> <r> <mm>"` / `"RT <cdeg>"`, raises
  `ValueError` on any other verb or malformed step.
- `run_tour(transport, params, heading, legs, *, v_max=150, a_max=400,
  omega_max=1.0, alpha_max=3.0, cadence=None, inter_leg_settle=0.3,
  final_settle=0.6, row_callback=None, on_leg=None, should_stop=None,
  clock_fn=time.monotonic, sleep_fn=time.sleep) -> TourResult`. Builds ONE
  `StreamingExecutor`, calls `begin()` fresh per leg (never `preempt()` —
  each leg's own run already ended cleanly before the next starts), stops
  immediately (no further legs) on any non-`COMPLETED` outcome.
  `should_stop`, if given, is polled once per tick (not just per leg
  boundary) so an external caller (ticket 003's `_TourRunner.stop()`) can
  interrupt mid-leg; a `True` result calls `StreamingExecutor.stop_now()`
  and reports `RunOutcome.STOPPED` for the interrupted leg.
- Two independent, optional extension points per AC4: `row_callback(tick_index,
  leg_index, leg, TickResult, TLMFrame|None)` (global tick_index across the
  WHOLE tour, matching the CSV convention ticket 005's bench script needs)
  and `on_leg(leg_index, total_legs, leg, TourLegResult)` (per-leg
  narration only, no CSV/trace knowledge needed).
- Closure: `TLMFrame.pose` (not `HeadingCorrector.measured_heading()`, which
  reads whichever source the caller's corrector is configured for) is read
  at leg 1's own `begin()`-time baseline drain (reusing `begin()`'s own
  bounded-retry logic rather than duplicating it) and again after the final
  leg's settle window. `TourClosure.position_delta`/`heading_delta` are
  `None` unless every leg reaches `COMPLETED` — an early-stopped tour never
  reaches "the final leg's settle window" AC3 defines closure against, so
  reporting a partial/best-effort closure for a faulted tour would be
  inventing a number the ticket doesn't define. `heading_delta` uses
  `controllers.pid.normalize_angle` (reused, no new angle-wrap code).
- `testgui/commands.py`: `TOUR_1`/`TOUR_2` now imported from
  `planner.tour` at module top; `TOURS` dict body unchanged. No other field
  of `commands.py` touched.
- Also added a `TestTours.test_tours_are_read_from_planner_tour` case to
  the EXISTING `tests/testgui/test_commands.py` (the ticket's Testing Plan
  names `tests/unit/test_commands.py`, which does not exist — the real,
  existing file for `commands.py` is `tests/testgui/test_commands.py`,
  confirmed against ticket 004's own description: `tests/testgui/` is not
  in `pyproject.toml`'s `testpaths` yet — ticket 004 re-adds it — so this
  new case runs today only via a direct `pytest tests/testgui/
  test_commands.py` invocation (verified: 72 passed), not via the full-suite
  gate; it will start running under the full suite once ticket 004 lands).

Testing: `tests/unit/test_planner_tour.py` (28 tests) — parser regression
tests against real `TOUR_1`/`TOUR_2` data (leg counts, first/last leg,
turn-sign preservation, malformed/unknown-verb rejection), an AST-based
import guard (no `NezhaProtocol`/`SerialConnection`/`SimConnection`
import), a clean 3-leg run with closure math (including a synthetic
position+heading drift), leg-count-preserving "no leg after the last one"
check, FAULT and OVERSHOOT mid-tour early-stop tests (remaining legs never
attempted, closure fields `None`), a `should_stop()` mid-leg preemption
test, independent `row_callback`/`on_leg` hook tests (including "neither
hook supplied" still works), a per-leg-speed-honored test, and direct
`_compute_closure()` unit tests (simple drift, `±π` wraparound via
`normalize_angle`, zero-movement, missing-pose `None` propagation).
`FakeTransport` here is a "current frame" double (simpler than
`test_planner_executor.py`'s batch queue) — tests needing staged telemetry
mutate `transport.current_frame` from inside `row_callback`/`on_leg`, which
fire synchronously and deterministically from `run_tour()`'s own call
stack, avoiding any dependency on `begin()`'s exact retry-call count.

Full suite: `uv run python -m pytest` — 703 passed (was 675 before this
ticket; +28 new). `tests/testgui/test_commands.py` run directly (not yet
collected — ticket 004): 72 passed.

No surprises requiring an exception — the only judgment call was the
`TourLeg.speed` field addition (documented above) and the "closure only on
full completion" design (also documented above), both within "implementer's
call" per the ticket's own wording.

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
