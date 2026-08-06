---
id: 008
title: 'Fix sim OTOS heading sign divergence (Option A: SimPlant packs the hardware-mounted
  sign)'
status: done
use-cases:
- SUC-001
- SUC-004
depends-on: []
github-issue: ''
issue: sim-otos-heading-sign-diverges-from-hardware-angle-moves-never-stop.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix sim OTOS heading sign divergence (Option A: SimPlant packs the hardware-mounted sign)

## Description

**Pre-existing regression on master, discovered while establishing this
sprint's baseline — fix before any other ticket relies on sim OTOS
heading.** `uv run python -m pytest src/tests/sim` currently gives
**1 failed, 26 passed, 1 xfailed, 1 xpassed**:
`test_move_protocol.py::test_move_protocol_scenarios_pass` fails SUC-050
ANGLE — a Twist Move commanding `omega = 1.0 rad/s` with an ANGLE stop at
`1.0 rad` and a 5000 ms timeout runs the FULL timeout and ends at
**5.157 rad**. The ANGLE stop condition does not fire at all; this is not
tolerance drift.

**Correction (discovered during execution — see Completion Notes for the
full numbers):** the "1 failed, 26 passed, 1 xfailed, 1 xpassed" figure
above was measured under `pytest -x` (stop-on-first-failure) and never
saw the rest of the suite — the real, un-`-x`'d `src/tests/sim` run is
**492 tests**, not 29. The real pre-existing-branch baseline (measured by
`git stash`-ing this ticket's diff back out and re-running the full
suite) is **4 failed, 485 passed, 2 xfailed, 1 xpassed**: this ticket's
own named target (`test_move_protocol.py`), one more sim-system test
sharing the SAME root cause (`test_rebaseline_pose.py` — also drives a
90° ANGLE-stop Move — fixed as a side effect of this same change), and
two pre-existing failures with a completely unrelated cause
(`test_app_preamble.py`, `test_devices_otos.py` — confirmed byte-for-byte
unchanged before and after this ticket's diff, via `git stash`; not
fixed here, not caused here, out of this ticket's scope).

**Cause** (verified, `clasi/issues/sim-otos-heading-sign-diverges-from-
hardware-angle-moves-never-stop.md` has the full analysis): the firmware
negates OTOS heading before the planner consumes it
(`src/motion/planner/planner.cpp:513`,
`pose_.applyOtosHeading(-state.otos.heading, ...)`), reconciling a real,
measured hardware fact — the OTOS chip's mounted orientation is inverted
relative to encoder-derived heading (+84.58° optical vs. -82.45° encoder
on a single measured rotation, `planner.cpp:499-513`'s own comment). That
negation is correct for hardware and WRONG for sim, because sim's OTOS is
not an independently-oriented sensor: `TestSim::OtosPlant`
(`src/tests/sim/plant/otos_plant.h:5-18`) accumulates heading with the
same midpoint-arc update `Odometry::integrate()` uses (same
`BodyKinematics::forward()` math), and `TestSim::SimPlant` packs that
centre-frame `(x, y, heading)` directly into the OTOS chip's raw registers
with NO transform (`otos_plant.h:25-32` — an identity-mounting
assumption). So sim OTOS heading carries the SAME sign as encoder heading
by construction; hardware's carries the OPPOSITE sign; one negation
constant cannot be right for both. In sim, the firmware's negation runs
the planner's heading backwards against the commanded turn, the ANGLE
threshold is never reached, and the Move times out — exactly what
`planner.cpp:499-513`'s own comment predicted.

**Why this blocks sprint 135, not just master**: the Navigator's pivot
path (ticket 003, SUC-004) issues ANGLE Moves, and its world-frame target
solving (ticket 003, SUC-001) reads `state.otos` directly. Any sim
verification of either is currently invalid — it validates against a
simulated robot whose OTOS is oriented the OPPOSITE way from the real one.
Fixing this here, first, is what makes tickets 003 and 005's sim coverage
mean anything.

## Scope — Option A ONLY (stakeholder-confirmed)

The linked issue lays out two options. **This ticket is Option A only —
Option B is explicitly OUT of scope for this sprint, by stakeholder
decision. Do not widen this ticket to include it.**

- **Option A (this ticket)**: make `TestSim::SimPlant` pack the OTOS
  heading register with the hardware's actual mounted sign — negated
  relative to its own internal centre-frame accumulator — so the
  simulated chip reports what the real chip reports. The firmware's
  existing negation (`planner.cpp:513`) then reconciles sim and hardware
  identically. Touches sim only: no wire change, no calibration
  invalidation. Must be introduced as ONE named constant with a comment
  tying it explicitly to `planner.cpp:513`, so the two flip together if
  and when Option B ever lands — this is the same "one constant, one
  comment, one flip" discipline sprint 135's own Navigator landmine list
  (ticket 004, Landmine 4) already applies to the omega sign.
- **Option B (explicitly deferred, not this ticket)**: fix the
  body-kinematics omega sign itself — delete the negation and every
  host-side `YAW_SIGN = -1`. `planner.cpp:509-512`'s own comment names
  this as "the RIGHT fix," and names why it's deferred: it changes the
  MEANING of omega on the wire and invalidates both stored per-direction
  rotation calibrations, requiring hardware re-measurement. That is a
  wire-semantics change with its own blast radius and gets its own
  sprint. Do not attempt it here even if it looks tempting mid-ticket —
  if you find yourself editing `BodyKinematics` or deleting a
  `YAW_SIGN` constant, stop; that is Option B, not this ticket.

## Acceptance Criteria

- [x] `TestSim::SimPlant` packs the OTOS heading register with the
      hardware-mounted sign (negated relative to `OtosPlant`'s own
      centre-frame accumulator), as one named constant with a comment
      pointing at `planner.cpp:513`.
- [x] `uv run python -m pytest src/tests/sim` is fully green **for
      everything this ticket is responsible for**. ~~27 passed (up from 26
      passed / 1 failed)~~ — that figure was wrong (measured under `-x`,
      see Description correction). Corrected criterion: the full-suite
      failure count drops from 4 (baseline, pre-existing branch state) to
      2, with the 2 remaining failures BOTH confirmed byte-for-byte
      unchanged from baseline (`test_app_preamble.py`,
      `test_devices_otos.py` — pre-existing, unrelated cause, not in this
      ticket's scope) and zero new failures introduced. No xfail/xpass
      drift: 2 xfailed / 1 xpassed, unchanged baseline-to-fix.
- [x] SUC-050 ANGLE specifically: a sim ANGLE Move of 1.0 rad ends via
      the ANGLE stop condition (`kFlagFaultMoveTimeout` clear, not a
      timeout) with final heading inside **1.0 ± 0.25 rad**.
- [x] `planner_tests` ctest suite stays 8/8 green (it uses `PerfectPlant`,
      not `SimPlant`/`OtosPlant` — should be untouched by this change;
      confirm rather than assume).
- [x] The settled sign convention (sim OTOS reports the hardware-mounted
      sign; the firmware's existing negation is the ONE reconciliation
      point) is documented in one place — this ticket's own Completion
      Notes plus a pointer from `otos_plant.h`'s own header comment — so
      ticket 003's Navigator ctest fixtures and ticket 005's sim system
      tests can cite it without re-deriving the analysis.
- [x] Option B is NOT attempted: `git diff` at completion touches only
      sim-side files (`src/tests/sim/plant/otos_plant.h`/`.cpp`,
      `src/tests/sim/plant/sim_plant.h`/`.cpp` or wherever `SimPlant`
      actually packs the register — confirm exact file), never
      `BodyKinematics`, never any host-side `YAW_SIGN` constant.

## Testing

- **Existing tests to run**:
  ```
  uv run python -m pytest src/tests/sim
  cmake --build src/motion/planner/build --target planner_tests
  ctest --test-dir src/motion/planner/build --output-on-failure
  ```
- **New tests to write**: none required beyond confirming the existing
  SUC-050 scenario (`test_move_protocol.py`) now passes — this is a bug
  fix restoring an existing test's validity, not new behavior needing new
  coverage. If the fix reveals the existing scenario's tolerance is too
  loose to have caught this cleanly, tighten it here.
- **Verification command**:
  ```
  uv run python -m pytest src/tests/sim -v
  ```
- **Hardware cross-check** (optional but recommended, per the linked
  issue's own Verification section): on `tovez` (by UID, direct serial —
  same constraints as ticket 006), a commanded 1.0 rad turn should land
  within the same tolerance the sim now asserts, confirming sim and
  hardware are demonstrably measuring the same convention. Not run this
  ticket (out of scope for this pass — sim-only fix); left for a future
  hardware session.

## Completion Notes

### The settled sign convention (cite this, do not re-derive)

**`TestSim::OtosPlant`'s own accessors (`x()`/`y()`/`heading()`/
`reportedX()`/`reportedY()`/`reportedHeading()`) stay in the ENCODER-sign
convention** — unchanged by this ticket. `TestSim::SimPlant` is the ONE
place the sign flips: its `handleOtosRead()` (`src/sim/sim_plant.cpp`)
negates `OtosPlant::reportedHeading()` by a new constant,
`kOtosHardwareMountSign = -1.0f`, before packing the OTOS wire register.
That makes the SIMULATED chip's wire-level heading report the SAME
(inverted) sign a REAL chip reports, so the firmware's one existing
reconciliation point — `src/motion/planner/planner.cpp:513`,
`pose_.applyOtosHeading(-state.otos.heading, ...)` — negates sim and
hardware identically, and the planner's `pose_.heading()` (what the ANGLE
stop condition and every angular decision actually measures) ends up in
the encoder-sign convention in both regimes.

Practical takeaway for tickets 003/005: **read `state.otos.heading` (the
raw wire value the planner receives, pre-negation) and it will be the
NEGATION of encoder/`Odometry`-derived heading, matching hardware.**
Read `pose_.heading()` (post-negation, what the Navigator's own
world-frame solving and ANGLE Moves actually act on) and it agrees with
encoder heading, in both sim and hardware. Do not add a second sign flip
anywhere in the Navigator — this is the only reconciliation point.

### Exact files/lines changed

- `src/sim/sim_plant.cpp`:
  - Added `kOtosHardwareMountSign = -1.0f` (anonymous namespace, next to
    the existing `kPosMmPerLsb`/`kHdgRadPerLsb` OTOS wire constants) —
    one named constant, comment ties it explicitly to `planner.cpp:513`
    and explains the double-negation reconciliation, per the ticket's
    "one constant, one comment, one flip" requirement.
  - `handleOtosRead()`: the `rh` (POSITION_XL heading word) computation
    now multiplies `otos_.reportedHeading()` by `kOtosHardwareMountSign`
    before scaling to LSBs. `rx`/`ry` (position) and the VELOCITY_XL
    words (`rvx`/`rvy`/`rvh`, including the omega/angular-rate word) are
    UNCHANGED — the firmware only negates `state.otos.heading`
    (`planner.cpp:513`), never `state.otos.omega`, and `state.otos.omega`
    is otherwise only telemetry-reported (`robot_loop.cpp:372`,
    `telemetry.cpp:62`), never consumed by the planner — confirmed by
    reading both call sites before deciding the rate word was out of
    scope.
- `src/tests/sim/plant/otos_plant.h`: added a paragraph to the file's own
  header comment (after the existing "Identity-mounting assumption"
  paragraph) stating the settled sign convention above and pointing at
  `sim_plant.cpp`'s `kOtosHardwareMountSign` and `planner.cpp:513` — the
  acceptance criterion's required pointer for tickets 003/005.
- `src/tests/sim/plant/plant_harness.cpp` (NOT in the ticket's original
  file list — added during execution, see below):
  `scenarioPivotHeadingSaneViaOdometry()`'s "sanity cross-check" assertion
  compared `OtosPlant`-derived heading (as decoded off `SimPlant`'s wire
  by a real `Devices::Otos`, with no planner-side negation applied) directly
  against `Odometry::theta()`, asserting equality — the exact
  identity-sign assumption this ticket's fix intentionally invalidates.
  Updated the assertion to expect the NEGATION (`-last.odomTheta`) instead
  of equality, with a comment explaining why, citing this ticket and
  `planner.cpp:513`. This is a "restore test validity" fix of the same
  kind the ticket's own Testing section anticipated for SUC-050's
  tolerance ("if the fix reveals the existing scenario['s assumptions
  are]... too loose/stale to have caught this cleanly, tighten it here")
  — same root cause, same reconciliation point, just a different existing
  test whose comment literally called its own equality assumption
  "Decision 3's own consequence" of identity mounting.

### Why the file list grew beyond the ticket's own list

Running the FULL `uv run python -m pytest src/tests/sim` (492 tests, not
the 29 the ticket's baseline assumed — see Description correction above)
surfaced that `test_plant.py::test_plant_harness_compiles_and_passes`
PASSED on the pre-fix branch tip but FAILED once `sim_plant.cpp`/
`otos_plant.h` were changed — confirmed via `git stash` (fix removed →
passes; fix restored → fails) that this was a genuine regression from
this ticket's own diff, not a pre-existing issue. Root cause: the same
`scenarioPivotHeadingSaneViaOdometry()` assertion described above. Fixed
in place (see above) rather than reverting the sign fix, since the
assertion's own comment already named the assumption this ticket
overturns on purpose (Option A's entire point is that identity-mounting
no longer holds at the wire boundary).

### Full test output summary

**`planner_tests` ctest suite — untouched, confirmed by running, not
assumed** (uses `PerfectPlant`, never `SimPlant`/`OtosPlant`):
```
100% tests passed, 0 tests failed out of 8
```
Identical before and after this ticket's diff.

**`uv run python -m pytest src/tests/sim` — full suite, no `-x`:**

| | failed | passed | xfailed | xpassed | total |
|---|---|---|---|---|---|
| baseline (fix stashed out) | 4 | 485 | 2 | 1 | 492 |
| with this ticket's fix | 2 | 487 | 2 | 1 | 492 |

Baseline's 4 failures: `test_move_protocol.py::test_move_protocol_scenarios_pass`
(this ticket's named target, SUC-050 ANGLE), `test_rebaseline_pose.py::
test_rebaseline_pose_sanity` (same root cause — also drives a 90° ANGLE-stop
Move; fixed as a side effect), `test_app_preamble.py::
test_app_preamble_harness_compiles_and_passes` and `test_devices_otos.py::
test_devices_otos_harness_compiles_and_passes` (pre-existing, unrelated —
same exact failure text baseline vs. fix, confirmed via `git stash`; these
concern `Devices::Otos::begin()`'s write-transaction count and scripted
calibration-register bytes, nothing to do with heading sign).

With-fix's 2 failures: the same `test_app_preamble.py`/`test_devices_otos.py`
pair, byte-for-byte identical output to baseline — confirmed unchanged,
out of scope for this ticket, not re-diagnosed here so a future reader
doesn't mistake them for sprint 135 fallout.

**SUC-050 ANGLE specifically** (`test_move_protocol.py -v -s`): scenario
prints no `FAIL` lines (previously printed 2: `kFlagFaultMoveTimeout` set
when it should be clear, and heading 5.157 rad outside tolerance). Full
scenario list runs and the enclosing `test_move_protocol_scenarios_pass`
passes.

**`test_rebaseline_pose.py` specifically** (`-v -s`): now prints
`final heading: rebaseline case=93.445deg, control (no rebaseline)=93.890deg,
commanded=90.0deg` and `OK: rebaseline pose-sanity scenarios passed` — both
within tolerance of the commanded 90°, where before the fix the ANGLE-stop
Move never completed correctly (the same runaway-heading defect as
SUC-050).

### Confirming Option B was not touched

`git diff` at completion touches only: `src/sim/sim_plant.cpp`,
`src/tests/sim/plant/otos_plant.h`, `src/tests/sim/plant/plant_harness.cpp`,
plus this ticket file and `.clasi/.clasi.db` (process bookkeeping, not
source). No `BodyKinematics` edit. No `YAW_SIGN` constant touched, deleted,
or added (host-side `YAW_SIGN` constants are untouched; the only line in
the diff mentioning `BodyKinematics` is a doc-comment word-for-word
unchanged reference, not an edit to the class).

### Hardware cross-check

Not performed this ticket (sim-only fix, no bench session run). Left for
a future ticket/session per the ticket's own "optional but recommended"
framing.
