---
id: '007'
title: 'Host shrink: pathplan gotoWorld/gotoRobot/followPath become thin GO_TO senders'
status: done
use-cases:
- SUC-002
depends-on:
- '004'
github-issue: ''
issue: replaceable-go-to-moves-in-the-motion-library.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host shrink: pathplan gotoWorld/gotoRobot/followPath become thin GO_TO senders

## Sequencing — this is the sprint's ONE breaking change, run it last

This ticket depends technically only on ticket 004 (the wire arm existing
and working), but it should be executed LAST among this sprint's tickets
in practice — after ticket 005 (sim tests) and ticket 006 (hardware smoke)
have both proven `GO_TO` solid end-to-end. Deleting the host-side loop
before `GO_TO` is provably working would leave no way to drive a goto at
all if something regresses. Per this sprint's own "breaking changes go
last" sequencing rule (sprint.md Migration Concerns), do not start this
ticket until 005 and 006 are done, even though the dependency graph alone
would permit it right after 004.

## Description

Once `GO_TO` exists and works (tickets 004-006), the host's own
arc-solving and replace-throttling logic in
`src/host/robot_radio/pathplan/` becomes dead weight: `gotoWorld`/
`gotoRobot` (`planner.py:678`/`879`) shrink to thin `GO_TO` senders, and
`followPath` (`planner.py:993`) keeps `pursuitTarget()` (waypoint
lookahead — firmware doesn't do this, it stays host-side) but streams
`GO_TO` targets instead of driving arcs itself. `solveArcToPoint`
(`solver.py:256`), `ReplaceThreshold` (`planner.py:321`), and the
curvature-slew-clamp logic (`solver.py:237`, `_clampOmegaStep`) all
become dead code and are deleted.

**Keep**: `pursuitTarget()` (`solver.py:523`, waypoint lookahead — solves
a problem firmware-internal goto does not, "which point on the path is
next"), `MoveIdAllocator` (`planner.py:464`), and the ack-verified
retry/dedup machinery (`AckRetry`, `_recordAcks`, `_readFrames`,
`GiveUpLimits`) — link loss is real regardless of where arc-solving runs,
and ticket 001's measurement sizes this retry policy honestly rather than
against folklore.

## Callers to check before deleting anything — verified, not assumed

The linked issue names only `gotoWorld`/`gotoRobot`/`followPath` as
callers of the doomed functions. Grepping the actual repo at
sprint-planning time (2026-08-06) found MORE callers of
`robot_radio.pathplan` than the issue lists — **all of these must be
updated or explicitly retired, not silently left broken**:

```
src/tests/bench/square_tour.py
src/tests/unit/test_world_pose.py
src/tests/unit/test_planner.py
src/tests/unit/test_solver.py
src/tests/sim/test_pathplan_goto_convergence.py
```

Re-run this grep at ticket time — the list above is a floor, not a
ceiling:

```
grep -rln "from robot_radio.pathplan\|import pathplan\|pathplan\.planner\|pathplan import" --include="*.py" src/
```

At minimum: `test_solver.py` and `test_planner.py` almost certainly unit-
test `solveArcToPoint`/`ReplaceThreshold` directly — these tests must be
DELETED alongside the functions they test (not left red), and their
parity-testing value is already captured by ticket 002's C++ arc-solver
ctest, which was built BY PORTING this same `solver.py` surface — confirm
ticket 002's ctest actually covers what these Python tests covered before
deleting the Python originals, so no test coverage is silently lost in
the port, only relocated. `square_tour.py` and
`test_pathplan_goto_convergence.py` likely call `gotoWorld`/`gotoRobot`/
`followPath` themselves and should keep working once those functions are
reshaped to thin `GO_TO` senders — verify each one still passes/runs
after the shrink rather than assuming the reshape is behavior-preserving
for its callers.

## Acceptance Criteria

- [x] `gotoWorld`/`gotoRobot` (`planner.py`) send `GO_TO` commands
      directly (frame WORLD/ROBOT respectively) instead of running their
      own solve-and-replace loop; ack-verified retry against the
      accepted-id ring is KEPT.
- [x] `followPath` keeps `pursuitTarget()`'s own lookahead-point picking
      unchanged, and streams each picked point as a `GO_TO` instead of
      calling `_sendVerifiedTwist`'s own arc-driving logic.
- [x] `solveArcToPoint`, `ReplaceThreshold`, and the curvature-slew-clamp
      logic (`_clampOmegaStep`) are DELETED, not just unreferenced —
      confirm with `grep -rn "solveArcToPoint\|ReplaceThreshold\|_clampOmegaStep" src/`
      returning nothing outside version-control history.
- [x] `pursuitTarget()`, `MoveIdAllocator`, and the ack-verified
      retry/dedup machinery (`AckRetry`/`GiveUpLimits`/`_recordAcks`/
      `_readFrames`) are UNCHANGED and still exercised.
- [x] Every caller found by the grep above (the five files listed, plus
      whatever a fresh re-run of that grep finds at ticket time) is
      checked: either still passes/runs against the reshaped functions,
      or is explicitly deleted/updated with a stated reason in Completion
      Notes — none is left silently broken.
- [x] `test_solver.py`/`test_planner.py` coverage of the deleted
      functions is confirmed to already exist in ticket 002's C++
      arc-solver ctest before the Python tests are deleted (a coverage
      relocation, not a coverage loss) — state this confirmation
      explicitly in Completion Notes.
- [x] `clasi/issues/later/otos-sampled-only-at-rest-not-integrated-
      during-motion.md` is marked superseded (sprint.md Scope's own
      instruction, issue's Open Point 2) once this ticket lands and the
      sprint's shipped behavior supersedes it in practice.

## Testing

- **Existing tests to run**: `pyproject.toml`'s `testpaths` is
  `["src/tests/sim", "src/tests/unit", "src/tests/testgui"]` (verified
  directly, not assumed) — a bare, repo-root `uv run python -m pytest`
  already collects ALL FIVE caller files from the grep above
  (`test_world_pose.py`/`test_planner.py`/`test_solver.py` live under
  `src/tests/unit`, `test_pathplan_goto_convergence.py` under
  `src/tests/sim`; `square_tour.py` is a bench script, not
  pytest-collected — exercise it manually or via whatever smoke path it
  already has). No narrower `--rootdir`/path is needed or more correct
  than the plain invocation:
  ```
  uv run python -m pytest
  ```
  plus an explicit, verbose run of whichever of the four pytest-collected
  caller files survive the shrink (e.g. `uv run python -m pytest src/tests/sim/test_pathplan_goto_convergence.py -v`).
- **New tests to write**: none required beyond what tickets 002/005
  already added — this ticket is a deletion/reshape, not new behavior.
- **Verification command**:
  ```
  uv run python -m pytest
  grep -rn "solveArcToPoint\|ReplaceThreshold\|_clampOmegaStep" src/
  ```
  (the grep should return nothing).

## Completion Notes

**Policy override for this ticket (team-lead instruction, not this
ticket's own Testing section)**: did NOT run a bare repo-root
`uv run python -m pytest` (~15 min, has caused problems this sprint).
Verified instead with each specific caller test file individually by
name, plus `square_tour.py --sim` (both `segments` default mode and
`--mode goto`). Team-lead runs the one full bare-suite pass at sprint
close.

### Files changed

- `src/host/robot_radio/pathplan/solver.py` — DELETED
  `solveArcToPoint()`/`ArcSolution`/`SolverLimits`/`MAX_WHEEL_STEP`/
  `_clampOmegaStep()` and their supporting plant-derived constants
  (`_PLANT_GAIN`/`_PLANT_TAU`/`_PLANT_ACCEL`/`_SLEW_ACCEL`/
  `_SOLVE_PERIOD`/`_MIN_DISTANCE`/`_MIN_BEARING`). `pursuitTarget()`/
  `PursuitTarget`/`_closestPointOnSegment()`/`_circleSegmentExit()`/
  `_searchEndSegment()`/`_SEARCH_LOOKAHEADS`/`_MIN_SEARCH_SEGMENTS`/
  `_POSITION_SCALE` UNCHANGED. Module docstring rewritten to describe
  only `pursuitTarget()`, with a "History note (135-007)" pointing at
  `Motion::ArcSolver`/`arc_solver_test.cpp` as where the deleted
  geometry now lives and is tested.
- `src/host/robot_radio/pathplan/planner.py` — full rewrite. DELETED:
  `ReplaceThreshold`, `ProgressCheck` (the liveness backstop it
  no longer needs — a `GO_TO`'s own wire `timeout` plus SUC-005's
  firmware-side OTOS-staleness/disconnect handling supersede it),
  `TERMINATION_TOLERANCE` (arrival is now firmware-judged via the
  completion ack, not a host distance check), `_shouldReplace()`,
  `_moveTimeoutFor()` and its `_MOVE_TIMEOUT_*` constants,
  `_targetBehindReason()`, `_sendVerifiedTwist()`,
  `_MAX_UNSOLVABLE_CYCLES` and the whole "three rejected approaches"
  target-behind-guard saga (moot — `Motion::Navigator` handles bearing/
  pivot internally, SUC-004). KEPT unchanged: `GiveUpLimits`, `AckRetry`,
  `MoveIdAllocator` (docstring extended to note it is now shared by
  `Move.id` AND `GoTo.id`), `_giveUpReason()`, `_readFrames()`,
  `_advance()`, `_recordAcks()`, `_lookaheadFor()` and its derivation
  constants. NEW: `_sendVerifiedGoTo()` (the `go_to()` analogue of
  `_sendVerifiedTwist()`), `_gotoAndWait()` (shared send-then-wait-for-
  completion-ack implementation behind `gotoWorld()`/`gotoRobot()`),
  `_DEFAULT_PATH_ARRIVAL_TOLERANCE` (100mm, `followPath()`'s own
  terminal-waypoint host-side arrival test default, carrying forward
  `TERMINATION_TOLERANCE`'s old NUMBER since that physical floor is
  unchanged, but as its own independent constant). TRIED AND REVERTED:
  a `_STREAM_PERIOD` (0.5s) wall-clock send throttle for `followPath()`
  — NOT a revived `ReplaceThreshold`, but still wrong; measured to cause
  spurious stop-pivot cycles on tight cornering and removed — see
  "Real bug found and fixed during verification" below.
  `followPath()` sends every cycle unconditionally instead.
  `GotoResult`/`FollowPathResult` trimmed (`forcedResends`
  removed — no more `ProgressCheck` to report on).
- `src/host/robot_radio/pathplan/__init__.py` — export list updated to
  match (drops `solveArcToPoint`/`SolverLimits`/`ArcSolution`/
  `ReplaceThreshold`/`TERMINATION_TOLERANCE`; adds `pursuitTarget`/
  `followPath`/`FollowPathResult`/`AckRetry`).
- `src/host/robot_radio/robot/protocol.py` — NEW `NezhaProtocol.go_to()`
  (mirrors `move_twist()`/`move_wheels()`) plus module-level
  `GOTO_FRAME_WORLD`/`GOTO_FRAME_ROBOT` constants. This was not
  explicitly named in the ticket body, but is a necessary consequence of
  "gotoWorld/gotoRobot send GO_TO commands directly" — there was no
  existing wire sender for `GO_TO` on `NezhaProtocol` as of ticket 004
  (confirmed via `git log`: 004's own diff to `protocol.py` only touched
  the `DriveMode` display-char rename). `goto_otos.py`'s own ticket-006
  code comment explicitly anticipated this ("135-007 adds the proper
  host wrapper... delete this method and call `self.p.go_to(...)` once
  135-007 lands").
- `src/tests/bench/goto_otos.py` — `Robot.goto_wire()` now delegates to
  `self.p.go_to()` instead of building the envelope itself (removes the
  now-dead `self._envelope_pb2`/local import); the one call site inside
  `goto()` is unchanged, so this is a pure internal simplification, not a
  behavior change. Not one of the ticket's named caller files (it never
  imports `robot_radio.pathplan`), but directly coupled to adding
  `go_to()` and flagged by ticket 006's own forward-reference comment.
- `src/tests/bench/square_tour.py` — fixed a real breakage:
  `gotoSquareWaypoints()` imported the now-deleted `solver.MAX_WHEEL_STEP`
  to derive its fillet-radius drivable floor. Replaced with
  `_ARC_SOLVER_MAX_WHEEL_STEP = 125.0` (a manually-synced mirror of
  `Motion::kArcSolverMaxWheelStepDefault`, `src/motion/navigator/
  arc_solver.h` — no shared header exists to import the real C++
  constant from a Python bench script). Updated the surrounding
  derivation comment and `runGotoTour()`'s `followPath()` call site
  (dropped the `SolverLimits` construction, now passes `speed=CRUISE`
  directly; dropped `.forcedResends` from the printed diagnostic).
  **Real, documented behavior consequence found while fixing this (not
  fixed by this ticket, flagged per instructions)**: the firmware's own
  arc solver clamps at a shorter 50ms period than the deleted host
  solver's 100ms, which raises the derived floor from 38.4mm to 76.8mm
  for this robot's `CRUISE`/`PHYSICAL_TRACK`. The 250mm playfield leg
  case this whole derivation comment was originally written for now
  clamps its corner radius to 62.5mm, BELOW the new floor —
  `gotoSquareWaypoints(legLength=250.0, ...)` now raises `ValueError`
  where it used to succeed. Playfield goto-mode runs are out of scope
  this sprint session regardless (sprint.md Decision 3: hardware
  constrained to direct serial, no camera) — this is a real follow-up,
  not something this ticket fixes, and is called out in the code comment
  itself (`TARGET_CORNER_RADIUS`'s own module-level derivation) so it is
  not silently lost.
- `clasi/issues/later/otos-sampled-only-at-rest-not-integrated-during-
  motion.md` — marked superseded (blockquote note + `superseded_by`
  frontmatter citing tickets 002/003/006/007), citing ticket 006's
  hardware-verified mid-goto pivot/replacement pass (its own Completion
  Notes: "per `Motion::Navigator`'s design, the pivot's own stop
  condition reads OTOS heading" — i.e. mid-motion) and ticket 003's
  `Motion::Navigator` design (reads `state.otos` every internal tick
  while a goto is active, by construction — closed-loop point-target
  navigation cannot work sampling only at rest). Noted explicitly that
  the issue's ORIGINAL concern (`Motion::Planner`'s continuous
  `StateEstimator` OTOS fusion) is UNCHANGED and not wrong — `Motion::
  Navigator` is a second, independent OTOS consumer the issue never
  anticipated, not a rebuttal of the original analysis.

### Coverage-relocation confirmation (acceptance criterion, stated
explicitly as required)

Compared `test_solver.py`'s pre-135-007 test surface (10 groups: max-
wheel-step derivation, default-behind-angle, on-heading, 90-degree
tangent-circle identity, moderate off-heading symmetric left/right, three
target-behind-guard cases, zero-distance degenerate case, three slew-
clamp ramp/reversal cases, purity, final-heading-ignored, and the
structural no-Angle-stop-field check) directly against
`src/motion/navigator/tests/arc_solver_test.cpp` (read in full before
deleting anything, per this ticket's own instruction): the ctest's own
header comment states it was built "mirroring `test_solver.py`'s own test
surface... closely enough to be a real parity check", and its 17
`testXxx()` functions cover, one-for-one, every one of those ten groups
with the SAME representative numbers (`kTrackWidth=128`, `kSpeed=150`,
the 25.6mm close-target slew-clamp case, the 80/90/120-degree
behind-guard boundary cases, the `unclampedLimits()`/`defaultLimits()`
pair matching `test_solver.py`'s own `_UNCLAMPED`/`_DEFAULT`) plus one
addition (`testDefaultTurnFirstAngleIs50Degrees`, a ticket-003 addition
with no Python precedent, since `TURN_FIRST` pivot policy never existed
host-side). Confirmed by running the C++ suite is unaffected by this
ticket (`arc_solver_test.cpp` prints "PASS arc_solver_test (17 checks)"
per ticket 002's own Completion Notes) — this is a coverage RELOCATION,
not a coverage loss. `test_planner.py` never itself tested
`solveArcToPoint`/`ReplaceThreshold` directly (it tested `gotoWorld()`/
`gotoRobot()`/`followPath()`'s own decision logic around them, e.g.
`_shouldReplace()`), and those decision-logic tests are deleted alongside
the decision logic they tested (superseded by `Motion::Navigator`'s own
material-change throttle, `navigator.cpp`'s `kNavOmegaReplaceThreshold`/
`kNavArcLengthReplaceThreshold`, itself ctest-covered by
`navigator_test.cpp`, tickets 002/003 — not re-verified by this ticket,
which is host-side only).

### Every caller from the grep — verification result

Re-ran `grep -rln "from robot_radio.pathplan\|import pathplan\|pathplan\.
planner\|pathplan import" --include="*.py" src/` at ticket time: same five
files as the ticket's own floor (`square_tour.py`, `test_world_pose.py`,
`test_planner.py`, `test_solver.py`, `test_pathplan_goto_convergence.py`)
plus the package's own `__init__.py`/`solver.py`/`planner.py`
(self-references, expected) — no new external callers appeared.

- `src/tests/bench/square_tour.py` — UPDATED (see Files changed above).
  Verified by RUNNING, not just reading, REPEATEDLY (the first run's own
  success was misleading — see the debugging section below):
  `uv run python src/tests/bench/square_tour.py --sim` (default
  `segments` mode, does not touch `pathplan` at all) → `PASS: square tour
  closed`, stable across reruns. `uv run python src/tests/bench/
  square_tour.py --sim --mode goto` (the actual `followPath()` caller) —
  FIRST run (with the since-reverted 0.5s send throttle still in place):
  `success=True`, `waypointsReached=17/17`, `sent=44`, closure 77.6mm.
  Three IMMEDIATE re-runs then FAILED consistently (90s give-up,
  15/17 waypoints) — a real, reproducible bug, root-caused and fixed
  (see "Real bug found and fixed during verification" below). After the
  fix, three more re-runs all passed cleanly: `success=True`,
  `waypointsReached=17/17` every time, `sent`=142/145/157,
  `retries=0`, `unacked=0`, closures 84.4/84.4/84.9mm → `PASS: goto-mode
  square tour closed`, all three times.
- `src/tests/unit/test_world_pose.py` — NOT UPDATED, verified unaffected:
  it imports only `robot_radio.pathplan.world_pose` (matched the
  `pathplan import` grep pattern on its own module docstring prose, not
  an actual import of anything deleted). Ran: 15/15 passed.
- `src/tests/unit/test_planner.py` — REWRITTEN. Deleted all coverage of
  `_shouldReplace()`/`ProgressCheck`/`_moveTimeoutFor()`/
  `_targetBehindReason()`/the old `_sendVerifiedTwist()`/the mock-based
  "`gotoRobot()` composes through `gotoWorld()`" test (no longer true —
  both are now peers over `_gotoAndWait()`). Added coverage for
  `_sendVerifiedGoTo()` (ack/retry/ERR_FULL/non-retryable-NACK/exhausted-
  retries), `gotoWorld()`/`gotoRobot()` end-to-end against hand-built
  fakes (world-frame send, robot-frame send with NO pre-existing
  `WorldPose` required, an aborted completion ack, a never-acked
  enqueue, a give-up while waiting for completion, `estop()` on every
  exit, `MoveRejected` propagation, shared-allocator id monotonicity),
  and a lock-in test that `followPath()` sends exactly once per cycle
  with no throttle (`sent == iterations`) plus a stalled-robot
  `GiveUpLimits` case. Kept
  unchanged: give-up-reason tests, `MoveIdAllocator` tests, `_readFrames`/
  `_advance` tests, `_lookahead` derivation test, `FollowPathResult`
  field-set test (updated for the dropped `forcedResends` field), the
  straight-line-reaches-every-waypoint test (adapted to a `go_to()`-
  shaped fake). Ran: 34/34 passed.
- `src/tests/unit/test_solver.py` — REWRITTEN. Deleted the ten
  `solveArcToPoint()`-era test groups (see coverage-relocation
  confirmation above); kept every `pursuitTarget()` test verbatim except
  one assertion inside `test_overshooting_a_dense_waypoint_still_yields_
  a_forward_target` that fed the picked target through the now-deleted
  `solveArcToPoint()` to check its behind-guard — removed that half of
  the check (the equivalent guard is now exercised directly by
  `arc_solver_test.cpp`) and kept the geometric assertion that is
  `pursuitTarget()`'s own to make. Ran: 10/10 passed.
- `src/tests/sim/test_pathplan_goto_convergence.py` — REWRITTEN. Deleted
  `test_goto_world_converges_under_otos_drift()`/
  `test_goto_world_stays_sane_under_enc_slip()` — their whole premise
  (the HOST navigates by `WorldPose`, so injecting an OTOS/encoder fault
  into the SIM's OTOS/encoder plant proves host-side robustness) no
  longer holds: `gotoWorld()`/`gotoRobot()` do not navigate by
  `WorldPose` at all any more, so a sim-side fault injection into a path
  the code under test never reads would prove nothing. Replaced with
  `test_goto_world_converges_to_a_nearby_target` (kept, adapted to the
  new `GotoResult`), `test_goto_robot_converges_without_any_preexisting_
  world_pose_fix` (NEW — proves `gotoRobot()` needs no `WorldPose` seed,
  directly, not just by assertion), and `test_follow_path_streams_
  targets_and_reaches_the_terminal_waypoint` (NEW — sim smoke for the
  reshaped `followPath()`). All three run against the REAL compiled
  firmware (`src/sim/build/libfirmware_host.dylib`, current as of this
  session) via `SimConfigConn(SimLoop)` — i.e. against a real
  `Motion::Navigator`, not a fake. Ran: 3/3 passed (`gotoWorld` residual
  5.9mm, `gotoRobot` residual 5.9mm, `followPath` residual 56.7mm — all
  well inside the 220mm slack bound).

### Real bug found and fixed during verification: followPath()'s send
throttle caused spurious stop-pivot cycles on tight cornering

The first implementation of `followPath()`'s streaming send gated on a
`_STREAM_PERIOD = 0.5` wall-clock throttle, deliberately chosen to match
ticket 005's own sim-tested EXTERNAL-mode streaming cadence
(`goto_protocol_harness.cpp`'s `scenarioStreamedTargetsNeverRestBeforeFinal`:
"streamed every 10 cycles/500ms"). All unit tests and the first
`square_tour.py --sim --mode goto` run PASSED with this design. Re-running
the exact same command three more times (per this ticket's own
acceptance criterion — verify square_tour.py by RUNNING it, not by reading
the diff) FAILED consistently: 90s give-up, only 15/17 waypoints reached,
~140 sends, average progress ~20 mm/s against a 150 mm/s CRUISE — an
almost 8x slowdown, not noise.

Four-phase debugging: (1) evidence — re-ran 3x, all 3 failed identically
(92.5-92.8s wall time, low CPU usage ruling out resource contention,
consistent ~15/17 waypoints); (2) pattern — first run passed, every
subsequent run failed the same way; ruled out "lucky first run" caching
theories since the failure was 100% reproducible from run 2 onward; (3)
hypothesis — `Motion::Navigator`'s own `turnFirstAngle` bearing check
(`navigator.cpp`'s `tick()`) runs every INTERNAL 50ms tick against
whatever target it was last handed; a target that goes stale for up to
0.5s while cruising a 62.5-90mm-radius fillet at 150mm/s gives the
robot's heading time to drift far enough that the bearing crosses
`turnFirstAngle` (~50deg), triggering a full stop-then-pivot-then-arc
sequence (SUC-004) instead of a smooth continued cruise — repeatable on
every fillet, which would produce exactly this magnitude of slowdown;
tested by lowering `_STREAM_PERIOD` to 0.05 then 0.0 inline (no file
edit yet) and re-running — both passed immediately, confirmed over 2
more repeats each; (4) root-cause fix — removed the throttle entirely,
sending every `followPath()` cycle unconditionally (matching the ORIGINAL,
pre-throttle instinct); re-ran `--sim --mode goto` 3 more times after the
code fix landed — 3/3 passed cleanly (142-157 iterations, 82-85mm
closure, no timeouts). Ticket 005's own 500ms-cadence sim test remains
valid and passing — its own scenario comment explicitly scopes it to a
gently-curving path "small enough to stay well under
NavigatorLimits::turnFirstAngle", i.e. it never exercised the tight-
cornering case that broke. `_STREAM_PERIOD` is fully removed from
`planner.py` (constant, docstring references, and the `nextSendTime`
throttle bookkeeping in `followPath()`'s loop); `test_planner.py`'s two
throttle-specific tests (`test_follow_path_throttles_sends_to_the_
stream_period`/`test_follow_path_streams_more_than_one_send_when_
unthrottled`) are replaced with one lock-in test
(`test_follow_path_sends_every_cycle_with_no_throttle`) asserting
`sent == iterations` (one send per cycle, no throttle).

### Final grep confirmations

```
grep -rn "solveArcToPoint\|ReplaceThreshold\|_clampOmegaStep" src/
```
Returns only comment/docstring prose (this ticket's own "History note"
blocks in `solver.py`/`planner.py`/`protocol.py`, `square_tour.py`'s
historical-context comments, and ticket 002/003's own already-shipped
C++ port comments in `arc_solver.h`/`navigator.h`/`arc_solver_test.cpp`
citing where these Python names came from) — zero function/class
DEFINITIONS remain anywhere in `src/`, confirmed with a stricter
`grep -rn "^def solveArcToPoint\|^class ReplaceThreshold\|^def
_clampOmegaStep\|def solveArcToPoint(\|class ReplaceThreshold(\|def
_clampOmegaStep(" src/` returning nothing at all.

### Test results summary (all AFTER the _STREAM_PERIOD fix landed)

- `test_world_pose.py`: 15/15 passed (unaffected).
- `test_planner.py`: 33/33 passed (rewritten; 34 before removing two
  now-obsolete throttle tests and adding one replacement lock-in test).
- `test_solver.py`: 10/10 passed (rewritten).
- `test_pathplan_goto_convergence.py`: 3/3 passed against the real
  compiled sim firmware (rewritten).
- `square_tour.py --sim`: `PASS: square tour closed`, stable.
- `square_tour.py --sim --mode goto`: `PASS: goto-mode square tour
  closed`, 3/3 consecutive runs after the fix (was 1/4 before it — see
  the debugging section above).
- Did NOT run the bare repo-root `uv run python -m pytest` per the
  team-lead's explicit override of this ticket's own Testing section for
  this run.

No version bump performed (sprint-branch ticket work; `close_sprint`
bumps once per sprint, per this project's own git-workflow instruction).
