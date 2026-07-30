---
id: '007'
title: 'square_tour.py goto-mode: sim/bench/playfield convergence gate'
status: in-progress
use-cases:
- SUC-007
depends-on:
- '002'
- '006'
github-issue: ''
issue: sprint-127-host-side-path-planner-goto-path-following.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# square_tour.py goto-mode: sim/bench/playfield convergence gate

## Description

Design issue T6+T7, **re-expressed per sprint.md Architecture Decision
2**: the issue's own text proposes new files under `src/tests/sim/
system/`; that directory is scheduled for deletion by a separate, later
sprint (`square-tour-is-the-one-system-test-sim-bench-playfield.md`),
which also states plainly that new coverage belongs in a unit test or an
expansion of the square tour, not a new system test. This ticket adds a
`goto`-driven mode to the existing `src/tests/bench/square_tour.py`
instead — same 8-segment square, driven through `gotoWorld()` (ticket
006) rather than open-loop `WHEELS` commands, reusing the script's
existing `_Backend` sim/hardware split. **No new file under
`src/tests/sim/system/` may be created by this ticket.**

Depends on ticket 002 (this loop replaces the in-flight move
continuously, at rate — it must not run against unverified dedup) and
ticket 006 (the loop itself).

**Decision 4's feedback loop — this ticket's own defining requirement,
not an optional stretch goal**: ticket 006 shipped a *provisional*
termination-tolerance/give-up constant, isolated at one commented
definition site and marked `# PROVISIONAL -- pending ticket 007`. This
ticket:

1. Measures the real actuation floor — **minimum reliable move distance
   and minimum reliable turn angle** — from playfield-tier camera-verified
   data (below what distance/angle does the robot's own deadband/
   actuation lag make a commanded move unreliable, i.e. it doesn't
   reliably move at all or lands with error comparable to the move
   itself).
2. **Updates ticket 006's provisional constant** at its single definition
   site with the measured value, removing the `# PROVISIONAL` marker (or
   replacing it with a comment citing this ticket and the measurement).
3. Records the measurement itself — raw data, not just the final number —
   in this ticket's Completion Notes.

A ticket that only reports the number without writing it back into
ticket 006's constant does **not** satisfy this ticket's acceptance.

**Three tiers**:

1. **Sim** (`square_tour.py --sim --mode goto`): convergence to a stated
   closure tolerance, bounded overshoot, no stall, through the real
   firmware loop compiled into `libfirmware_host.dylib`
   (`just build-sim`), ground truth `SimLoop.get_true_pose()`. Also
   exercise `SimLoop.set_otos_drift()`/`set_enc_slip()` injection and
   confirm sane (non-catastrophic) behavior under both.
2. **Bench** (stand, wheels free): same square, `--mode goto`, encoder-
   judged closure. Robot currently reachable only via the RADIOBRIDGE
   relay (`/dev/cu.usbmodem2121302`, dongle `zavaz`) — confirm current
   ROLE with `mbdeploy list` before running, do not assume this port is
   still current.
3. **Playfield** (camera-supervised): `--mode goto` against camera-
   verified world targets, per-segment-boundary camera fixes, PNG chart,
   the minimum-reliable-distance/angle measurement above, and the
   encoder-vs-OTOS divergence rate (from ticket 004's `WorldPose`) — this
   is what feeds a re-anchor-cadence recommendation.

**Safety obligations** (`.claude/rules/playfield-testing.md`, mandatory
for the bench and playfield legs of this ticket):

- **Lights**: confirm `GET http://192.168.1.122/rpc/Switch.GetStatus?id=0`
  reports `"output": true` *before* arming any playfield run — do not
  infer from a prior session. If off,
  `GET http://192.168.1.122/rpc/Switch.Set?id=0&on=true`, then re-confirm
  with `GetStatus` (the `Set` reply's own `was_on` is not fully
  trustworthy per the standing rule).
- **Geofence checked INSIDE the ~10 Hz time-advance primitive**, never
  between segments — reuse the promoted `field.Geofence`/
  `drainFrames(proto, seconds, geofence)` idiom, do not poll only at
  segment boundaries.
- **`estop()`, never `stop()`, in a `finally` block on every connection
  path** — `stop()` is a planned stop that waits behind whatever move is
  already queued (measured elsewhere: 39.8 cm of travel on a 40 cm leg
  before a mid-leg `stop()` took effect); `estop()` clears Drive targets
  and the planner queue in the same cycle.
- **Camera fix at every segment boundary, at REST** (after the settle
  dwell, so there is no velocity lag) — not just at start and end. An
  8-leg square with turns yields many boundary fixes; log every one.

**Files**:
- Modify: `src/tests/bench/square_tour.py` — add a `--mode goto` (or
  equivalent) that drives the same 8 segments through `gotoWorld()`
  instead of `WHEELS`; reuse the existing `_Backend`, `Geofence` (now
  imported from `field/`, ticket 003), and camera-fix helpers unchanged.
- Modify: `src/host/robot_radio/pathplan/planner.py` (ticket 006's file)
  — update the provisional termination constant per the feedback-loop
  requirement above.
- New: PNG chart output (per-run, following the existing bench-script
  chart convention `square_tour.py` already has for its wheel-command
  mode).

**Coding standards**: lowerCamelCase functions/variables, UpperCamelCase
types; no units in identifiers in any new code added to `square_tour.py`
or `planner.py`.

## Acceptance Criteria

- [x] Sim: `square_tour.py --sim --mode goto` closes the square within a
      stated tolerance (specific number, not "close enough"), bounded
      overshoot, no stall.
- [x] Sim: behavior under `set_otos_drift()` and under `set_enc_slip()`
      (run separately) stays sane — no runaway, no unhandled exception,
      explicit pass/fail against a stated bound for each.
- [ ] Bench: `square_tour.py --port /dev/cu.usbmodem2121302 --mode goto`
      (or the port `mbdeploy list` currently reports) completes with
      encoder-judged closure reported numerically. **BLOCKED — not run
      this pass** (explicit scope limit: write, do not run; see
      Completion Notes).
- [ ] Playfield: camera-verified closure reported numerically; per-
      segment-boundary camera fixes logged for every boundary, not just
      start/end; PNG chart produced and attached/referenced in Completion
      Notes. **BLOCKED on field access** (robot is on the stand; moving it
      to the playfield is a separate stakeholder action).
- [ ] Minimum reliable move distance and minimum reliable turn angle are
      each reported as a specific measured number, with the raw data that
      established them in Completion Notes. **BLOCKED on field access** —
      requires the wheels loaded (playfield), not the stand (see
      Completion Notes for why a stand measurement would be invalid).
- [ ] Ticket 006's provisional termination-tolerance/give-up constant is
      updated in place with the measured value from the criterion above,
      and its `# PROVISIONAL` marker is removed or replaced with a
      citation to this ticket. **BLOCKED — depends on the criterion
      above.** `planner.py` is untouched this pass.
- [ ] Encoder-vs-OTOS divergence rate is reported from a playfield run,
      with an explicit re-anchor-cadence recommendation. **BLOCKED on
      field access** — the divergence-reporting CODE is written and
      demonstrated working (sim tier, see Completion Notes), but this
      criterion specifically asks for a playfield-run number.
- [x] `estimator.weight_heading_otos` / `weight_omega_otos` confirmed
      still `0.0` and `geometry.otos_untrusted` confirmed untouched
      (grep-checked, not assumed) — sprint success criterion 5.
- [x] No new file under `src/tests/sim/system/`.
- [ ] Lights confirmed via `Switch.GetStatus` before every playfield run;
      `estop()` (never `stop()`) confirmed in a `finally` on every path
      exercised. **BLOCKED on field access** — the lights-check/estop-in-
      finally CODE PATHS are in place (unchanged `checkPlayfieldLights()`/
      `Geofence` arming ahead of every mode branch, `estop()` in
      `gotoWorld()`'s own `finally`, in `HardwareBackend.close()`, and in
      a new SIGTERM/SIGINT handler — see Completion Notes), but nothing
      hardware/playfield ran this pass to actually exercise them.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim -q`;
  `square_tour.py`'s existing (non-`goto`) mode still passes unmodified,
  both `--sim` and on hardware.
- **New tests to write**: none new at the pytest level beyond what
  tickets 001/002/004/005/006 already added — this ticket is primarily a
  bench-script expansion plus HITL runs, per the standing
  no-new-system-tests directive.
- **Verification command**:
  `just build-sim && uv run python src/tests/bench/square_tour.py --sim --mode goto`,
  then (confirm ROLE with `mbdeploy list` first)
  `uv run python src/tests/bench/square_tour.py --port /dev/cu.usbmodem2121302 --mode goto`,
  then the same command against the playfield with camera bring-up
  completed first.

## Completion Notes (2026-07-30, SIM tier only — bench/playfield written, not run)

**Status: `in-progress`, deliberately not `done`.** This pass was scoped
by the team-lead to the SIM tier plus writing (not running) the bench and
playfield tiers: the robot is on the STAND (wheels off the ground) this
session, and moving it to the playfield is a separate, physically-
coordinated stakeholder action. Four acceptance criteria are structurally
blocked on that field access (bench run, playfield run, the actuation-
floor measurement, and the resulting `planner.py` constant update) — see
each criterion above for its specific block reason. This ticket stays
`in-progress` until those legs run.

### Files

- Modified: `src/tests/bench/square_tour.py` —
  - `--mode {segments,goto,actuation-floor}` (default `segments`,
    unchanged behavior — the pre-existing 8-segment WHEELS/MOVE tour is
    untouched and still passes both `--sim` and, per ticket 001/002's own
    hardware legs earlier this sprint, on hardware).
  - `--mode goto`: `runGotoTour()`, `gotoSquareWaypoints()`,
    `_TruePoseSampler` (sim-only ground-truth overshoot/stall sampler),
    `reportGotoBoundaryFixes()`, `writeGotoChart()`.
  - `--mode actuation-floor`: `runActuationFloorMeasurement()` — written,
    playfield-only, refuses to run without a camera geofence
    (`SystemExit`), NOT exercised this pass (see below).
  - `_installEstopSignalHandler()` — mirrors `planner_square_tour.py`'s
    own SIGTERM/SIGINT-safety pattern (127-002's own hardware-incident
    lesson: a bare SIGTERM bypasses every Python `finally`).
  - `SimBackend.__init__`/`.advance()` gained a `realTime: bool = False`
    parameter (default preserves the exact existing manual-step
    behavior). See "Why SimBackend needed a real-time mode" below.
- Modified: `src/tests/sim/test_pathplan_goto_convergence.py` — extended
  (not duplicated) per the team-lead's own instruction: factored the
  existing test's setup into `_seedAndReanchor()`/`_forwardLeftTarget()`
  helpers (behavior-preserving refactor — the original test still passes
  unmodified in substance), then added
  `test_goto_world_converges_under_otos_drift` and
  `test_goto_world_stays_sane_under_enc_slip`.
- `src/host/robot_radio/pathplan/planner.py`: **untouched**.
  `TERMINATION_TOLERANCE` stays `100.0`, still marked `# PROVISIONAL --
  pending ticket 007's measured minimum reliable move distance/turn
  angle` — there is no playfield measurement yet to write into it. Per
  this ticket's own Description ("A ticket that only reports the number
  without writing it back... does not satisfy this ticket's acceptance"),
  since there is no number yet, no write-back was attempted.
- No file under `src/tests/sim/system/` created (grep-confirmed:
  `git diff --stat` against this pass's own changes shows only the two
  files above, both outside that directory).

### Why `SimBackend` needed a real-time mode

`square_tour.py`'s existing `SimBackend` runs the sim with **no tick
thread** (`start_tick_thread=False`) and advances it by manually calling
`SimLoop.step()` once per `CYCLE_S` slice — deliberate, so the segmented
tour's timing is fully deterministic. `pathplan.planner.gotoWorld()`'s
own `_advance()` (127-006) instead polls telemetry on **wall-clock**
time and never calls `SimLoop.step()` itself — it expects the sim to be
advancing **in the background**, exactly the pattern
`test_pathplan_goto_convergence.py` (127-006) already established with
`start_tick_thread=True`. Reusing the existing manually-stepped
`SimBackend` for `--mode goto` would have left the sim frozen while
`gotoWorld()`'s poll loop spun forever waiting for telemetry that never
arrives. Fixed by adding `SimBackend(robot_json, realTime=True)`
(`main()` passes `realTime=(args.mode == "goto")`) — a real-time
tick-thread connection with a matching wall-clock `advance()` branch,
additive only; every other mode still gets the exact original manually-
stepped behavior (`realTime` defaults `False`).

### Sim tier — `square_tour.py --sim --mode goto` (500 mm leg, default)

Full run (`uv run python src/tests/bench/square_tour.py --sim --mode goto`),
raw output:

```
[corner 1] success=True reason='arrived within 100 mm (distance=92.0 mm)' iterations=31 sent=17 minDistance=92.3mm overshoot=0.0mm stalled=False
  encoder-vs-OTOS divergence: distance=0.0mm heading=+0.00deg
[corner 2] success=True reason='arrived within 100 mm (distance=91.2 mm)' iterations=50 sent=32 minDistance=90.8mm overshoot=0.0mm stalled=False
  encoder-vs-OTOS divergence: distance=0.0mm heading=-0.02deg
[corner 3] success=True reason='arrived within 100 mm (distance=94.6 mm)' iterations=36 sent=22 minDistance=94.3mm overshoot=0.0mm stalled=False
  encoder-vs-OTOS divergence: distance=0.0mm heading=-0.26deg
[corner 4] success=True reason='arrived within 100 mm (distance=97.1 mm)' iterations=44 sent=27 minDistance=97.7mm overshoot=0.0mm stalled=False
  encoder-vs-OTOS divergence: distance=1.4mm heading=-0.25deg

[sim] goto-mode square: 4/4 waypoints attempted, 4/4 converged
  WorldPose closure (start -> end, plant truth): 91.1 mm
  PASS: max overshoot 0.0 mm (bound 60 mm)
  PASS: stall detected = False (window 2.0s / eps 5mm)
  ground-truth closure (SimLoop.get_true_pose()): 91.8 mm
wrote src/tests/bench/square_tour_goto_sim.png
PASS: goto-mode square tour closed
```

**Convergence**: all 4/4 waypoints reached, each arriving 90–99 mm from
its target — inside the (still-provisional) 100 mm `TERMINATION_TOLERANCE`
as expected, since that is the stop condition. Ground-truth closure
(start pose vs. end pose, `SimLoop.get_true_pose()`, independent of the
loop's own telemetry-derived belief) 91.8 mm on this run; repeat runs
(real-time ticking means exact iteration counts vary run to run) land in
the 79–94 mm band, consistently well inside the square's own 500 mm leg
scale.

**Bounded overshoot**: `overshoot=0.0mm` at every corner, every run —
`_TruePoseSampler.analyze()`'s own bound test (`GOTO_OVERSHOOT_BOUND =
60.0 mm`) passes. Zero overshoot is the expected shape here, not a fluke:
`solveArcToPoint()` emits one constant-speed arc per call and this loop
returns the instant ground truth would cross the tolerance ring, so there
is no mechanism in a SINGLE `gotoWorld()` call for the robot to fly past
a target and have to correct back (see `_TruePoseSampler`'s own docstring
for the same point). A future path-following ticket chaining waypoints
tighter than one leg apart is where real cross-target overshoot would
first become observable.

**No stall**: `stalled=False` at every corner, every run
(`GOTO_STALL_WINDOW=2.0s` / `GOTO_STALL_EPS=5.0mm` bound).

**Chart**: `src/tests/bench/square_tour_goto_sim.png` (regenerated by the
run above) — left panel shows the actual path: a rounded-corner
quadrilateral (NOT the segmented tour's sharp pivots — `gotoWorld()`
continuously re-solves the arc, so corners are flown through, not
stopped-and-turned), closing back near the start. Right panel plots
ground-truth distance-to-target per corner: all four traces are visually
near-parallel straight-line descents from ~500 mm to ~100 mm, each
preceded by a ~0.5–1 s near-flat segment (see "sluggishness" below).

**Sluggishness check (explicitly requested at dispatch)**: corner 1 (a
near-straight leg — the robot's start heading already points roughly at
it) converged in **31 iterations** (~3.1 s at the loop's 0.1 s
`cyclePeriod`); corners 2–4 (each requiring a real reorientation) took
**50, 36, 44 iterations** (~5.0 s, 3.6 s, 4.4 s respectively) — a
**16–61% longer time-to-converge** than the straight corner. This is a
real, measured, repeatable pattern (confirmed across 3 separate runs,
corner 1 always fastest, corner 2 always slowest), and it is consistent
with — though this pass did not isolate it as the sole cause of —
ticket 005's own flagged risk: the curvature slew limit
(`MAX_WHEEL_STEP = 6.0 mm/s` per solve,
`src/host/robot_radio/pathplan/solver.py`) forces omega to ramp toward
its target value over several solve cycles rather than jump there, and a
corner requiring more total curvature change necessarily spends more
cycles ramping. **Not fixed and not silently worked around here** — this
is exactly the number the team-lead asked to see, reported as measured;
the constant itself is unchanged (still traceable to ticket 001's
measured 433.33 mm/s Edge-B hazard per 005's own Completion Notes) and
changing it is explicitly out of this ticket's judgment to make.

### Sim tier — fault injection (`test_pathplan_goto_convergence.py`)

`uv run python -m pytest src/tests/sim/test_pathplan_goto_convergence.py -v -s`
— **3 passed**, raw output:

```
test_goto_world_converges_to_a_nearby_target gotoWorld sim smoke: success=True reason='arrived within 100 mm (distance=90.2 mm)' iterations=17 sent=9
  true final pose=(201.2,27.5,0.291 rad) ground-truth residual=89.9 mm
PASSED
test_goto_world_converges_under_otos_drift gotoWorld under otos_drift(80,80,0.3rad): success=True reason='arrived within 100 mm (distance=95.3 mm)' iterations=17 sent=9
  true final pose=(195.5,25.8,0.282 rad) ground-truth residual=95.9 mm
  encoder-vs-OTOS divergence at arrival: distance=112.8mm heading=-17.2deg (the injected drift, visible ONLY in this separate diagnostic -- not in gotoWorld()'s own navigation)
PASSED
test_goto_world_stays_sane_under_enc_slip gotoWorld under enc_slip(port=1,rate=0.15,mag=25mm): success=False reason="target is behind the robot's current heading (bearing=+97.9 deg from straight ahead) -- in-place reorientation is out of scope for this loop (sprint 127 Out of Scope: terminal-theta honoring in the solver)" iterations=20 sent=19
  true final pose=(184.5,99.8,1.372 rad) ground-truth residual=108.1 mm (vs. the loop's own belief: target is behind the robot's current heading ...)
PASSED
```

**`set_otos_drift(80mm, 80mm, 0.3rad)` — zero effect on navigation, by
design, not by luck**: ground-truth residual 95.9 mm vs. the no-drift
baseline's 89.9 mm — statistically the same, both well inside the
existing 220 mm slack bound. This is a **structural** finding, not a
control-loop compensation: `gotoWorld()` navigates by
`WorldPose.worldPose()`, the ENCODER-anchored transform, and NEVER calls
`WorldPose.worldPoseOtos()` — `set_otos_drift()` only perturbs
`TLMFrame.otos_reading`, which feeds exclusively the OTOS-anchored
transform. The drift is real and measurable — `encoderOtosDivergence()`
reports 112.8 mm / -17.2 deg at arrival, essentially the injected 80/80mm
+ 0.3 rad fed through the sim's own OTOS-vs-encoder geometry — it is just
invisible to `gotoWorld()`'s own navigation, exactly matching sprint
success criterion 5 (OTOS untrusted) at the HOST layer, not just the
firmware estimator layer.

**`set_enc_slip(port=1, rate=0.15, magnitude=25mm)` — sane, but a real
finding worth flagging**: this fault corrupts what `gotoWorld()` actually
navigates by (`TLMFrame.enc_left`, which `frame.pose`/`_latestEnc` are
built from directly). No exception, no NaN pose, no runaway (ground-truth
residual 108.1 mm, well inside the 565 mm no-runaway bound) — but the
run did NOT converge to success. Instead, the accumulated slip biased the
loop's own ENCODER-DERIVED heading estimate enough to trip the solver's
target-behind guard (`bearing=+97.9deg > limits.behindAngle=90deg`),
and `gotoWorld()` gave up with an explicit, correct reason rather than
attempting an unsafe in-place turn (by design — target-behind is treated
as an immediate give-up, not a turn cue, per this loop's own module
docstring). **This is "sane" per the ticket's own bar** (explicit
give-up reason, no crash, bounded final position) but is a real,
reportable robustness edge: a large enough heading bias from a slipped
wheel can end a real `gotoWorld()` call early via the target-behind path,
even though the true position was still only moderately off. Flagging
this plainly rather than tuning the injected fault down until it produces
a cleaner "success" result.

### Sprint success criterion 5 (grep-checked, not assumed)

```
$ grep -rn "weight_heading_otos\|weight_omega_otos" data/robots/tovez.json data/robots/tovez_nocal.json data/robots/togov.json
data/robots/tovez.json:139:    "weight_heading_otos": 0.0,
data/robots/tovez.json:140:    "weight_omega_otos": 0.0,
data/robots/tovez_nocal.json:114:    "weight_heading_otos": 0.0,
data/robots/tovez_nocal.json:115:    "weight_omega_otos": 0.0,
data/robots/togov.json:128:    "weight_heading_otos": 0.0,
data/robots/togov.json:129:    "weight_omega_otos": 0.0,

$ grep -n "otos_untrusted" data/robots/tovez.json data/robots/tovez_nocal.json
data/robots/tovez.json:35:    "otos_untrusted": true,
data/robots/tovez_nocal.json:35:    "otos_untrusted": true,
```

Both still hold, unchanged by this ticket (this ticket touched no `data/
robots/*.json` file).

### Regression

`uv run python -m pytest src/tests/sim -q` → **428 passed, 1 skipped, 1
xfailed** (was 426/1/1 before this ticket's 2 new tests — no regressions,
exactly +2 for the 2 new tests added). `square_tour.py --sim` (the
original, non-`goto` mode) still reports `PASS: square tour closed`
unmodified (`path 2067.0 mm (target 2000), heading 360.8 deg (target
360), closure 6.3 mm (plant truth)`).

### What is written but NOT run this pass (bench + playfield tiers)

- **Bench** (`--mode goto` on the stand): fully implemented (reuses
  `HardwareBackend`, `_installEstopSignalHandler()`, `WorldPose` left
  un-anchored so `worldPose()` reports raw encoder dead-reckoning from
  boot — "encoder-judged closure" per the ticket's own wording). Not
  executed this pass per the team-lead's explicit scope limit for this
  pass (sim only).
- **Playfield** (`--mode goto` with the camera): fully implemented —
  `checkPlayfieldLights()` + `Geofence` arming (existing, unchanged code,
  now ahead of the mode branch so it covers `goto`/`actuation-floor` too),
  geofence checked inside `gotoWorld()`'s own `_advance()` at its stated
  ~10 Hz cadence (127-006, unmodified), `captureFixWithRetry()` at start
  and every corner boundary (`reportGotoBoundaryFixes()` for the
  printed report), PNG chart. Not executed — robot is on the stand this
  session; moving it to the playfield is a separate stakeholder action.
- **`--mode actuation-floor`** (the Decision-4 minimum-reliable-move
  measurement): fully implemented — descending distance/angle sweeps
  (`--floor-distances`/`--floor-angles`, defaults
  `150,100,80,60,50,40,30,20,15,10,5` mm /
  `45,30,20,15,10,8,6,4,2` deg), camera-fix-before/after each move,
  prints raw (commanded, measured) pairs, refuses to run without a camera
  geofence (`SystemExit` — explicitly will NOT fall back to a bench/stand
  measurement, since unloaded wheels would understate the real floor).
  **Not run**: the robot is on the stand, and a stand measurement would
  be exactly the invalid substitute the team-lead's dispatch warned
  against — see the dispatch's own "Why the actuation floor cannot be
  measured on the stand" section. No number exists yet to write into
  `planner.py`'s `TERMINATION_TOLERANCE`.

### Explicit list of criteria still blocked on field access

1. Bench `--mode goto` numeric closure — not run this pass (scope limit,
   not a hard blocker; could run on the stand without moving the robot).
2. Playfield `--mode goto` camera-verified closure + per-boundary fixes +
   chart — blocked on the robot reaching the playfield.
3. Minimum reliable move distance / turn angle raw measurement — blocked
   on the playfield (wheels must be loaded; see "Why the actuation floor
   cannot be measured on the stand" in the dispatch).
4. `planner.py`'s `TERMINATION_TOLERANCE` write-back — blocked on #3.
5. Encoder-vs-OTOS divergence rate + re-anchor-cadence recommendation
   from a PLAYFIELD run — blocked on #2 (the divergence-computation code
   itself is proven working in sim, see the otos_drift test above).
6. Lights-check/estop-in-finally "confirmed... exercised" — blocked on
   #2/#1 (the code paths exist and are unchanged/additive, but nothing
   hardware ran this pass to exercise them).
