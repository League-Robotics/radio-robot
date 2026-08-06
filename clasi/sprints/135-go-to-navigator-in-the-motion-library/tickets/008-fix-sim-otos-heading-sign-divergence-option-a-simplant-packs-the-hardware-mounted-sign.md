---
id: '008'
title: 'Fix sim OTOS heading sign divergence (Option A: SimPlant packs the hardware-mounted sign)'
status: open
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

- [ ] `TestSim::SimPlant` packs the OTOS heading register with the
      hardware-mounted sign (negated relative to `OtosPlant`'s own
      centre-frame accumulator), as one named constant with a comment
      pointing at `planner.cpp:513`.
- [ ] `uv run python -m pytest src/tests/sim` is fully green: **27
      passed** (up from 26 passed / 1 failed), no new xfail/xpass drift.
- [ ] SUC-050 ANGLE specifically: a sim ANGLE Move of 1.0 rad ends via
      the ANGLE stop condition (`kFlagFaultMoveTimeout` clear, not a
      timeout) with final heading inside **1.0 ± 0.25 rad**.
- [ ] `planner_tests` ctest suite stays 8/8 green (it uses `PerfectPlant`,
      not `SimPlant`/`OtosPlant` — should be untouched by this change;
      confirm rather than assume).
- [ ] The settled sign convention (sim OTOS reports the hardware-mounted
      sign; the firmware's existing negation is the ONE reconciliation
      point) is documented in one place — this ticket's own Completion
      Notes plus a pointer from `otos_plant.h`'s own header comment — so
      ticket 003's Navigator ctest fixtures and ticket 005's sim system
      tests can cite it without re-deriving the analysis.
- [ ] Option B is NOT attempted: `git diff` at completion touches only
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
  hardware are demonstrably measuring the same convention.
