---
id: '003'
title: D-mode terminal completion guarantee
status: open
use-cases:
- SUC-004
depends-on:
- '001'
- '002'
github-issue: ''
issue: d-drive-terminal-instability-reversal-thrash.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# D-mode terminal completion guarantee

## Description

`d-drive-terminal-instability-reversal-thrash.md` root-causes the observed
stall -> reverse -> thrash -> lunge sequence (5 of 6 recorded `D` drives
failed that day) to the interaction of four factors, all confirmed by
config bisection in the sim against the actual firmware control code:

- **Asymptotic decel vs. strict crossing stop.** The D-mode decel hook
  caps `v_cap = sqrt(2 * aDecel * d_remaining)`, which reaches exactly
  zero AT the target — a profile shape that implicitly assumes the
  plant/controller can track an arbitrarily small commanded speed all the
  way to zero. Real motors (stiction, cogging, driver deadband) stop
  responding before that.
- **Down-only ratchet.** The hook only ever lowers the BVC target; once
  the robot is at/near the target the setpoint is pinned at ~15-20 mm/s
  forever, with no path to re-approach after a retreat, and mode `D` has
  no other terminal path except the multi-second TIME net.
- **Integrator freeze in exactly the wrong regime.** The pinned setpoint
  sits at/below `minWheelSpeed` (default 20 mm/s), where
  `VelocityController::update()` freezes the integrator
  (`source/control/VelocityController.cpp:85-96`) — no guaranteed windup
  path to break stiction forward. Bisection confirms reversal requires
  `vel.kI > 0` AND `vel.kP > 0` AND the `minWheelSpeed` freeze; with
  `minWheelSpeed = 0` there is no reversal at all.

This ticket gives the D-mode decel hook a terminal-completion guarantee via
two complementary, non-alternative fixes (architecture-update.md Decision
6 — both land here since they are two small, related changes to the same
decel hook, not independently schedulable units):

(a) **Floor the terminal `v_cap`** at `minWheelSpeed` once `d_remaining` is
inside a final-approach zone, so the profile itself never asks for a speed
the controller/plant cannot track. This reduces how often the
plant/controller enters the problematic near-zero-command regime but is
not a guarantee — it has no visibility into the actual physical breakaway
threshold of a given robot's motors.

(b) **A new stalled-short-completes terminal path** — if `d_remaining`
sits inside a new arrive tolerance (`distArriveTol`) and stops shrinking
for a stall-confirm window (`stallConfirm`), the drive completes now via a
new `MotionCommand::forceComplete(reason)` entry point (mirrors the
internal SOFT-stop teardown path a normal DISTANCE-fired stop already
takes, without requiring a `StopCondition` to have fired), tagged with an
additive `reason=` token. This is the backstop that makes correctness
independent of getting (a)'s floor value exactly right.

Letting the down-only ratchet re-approach after a retreat was considered
and explicitly rejected (architecture-update.md Decision 5): re-enabling
that mechanism directly re-exposes the same integrator-freeze/windup
dynamics that produced the initial reversal, with no guarantee of
convergence — exactly what the recorded thrash evidence looks like. The
stalled-short-completes design instead trades a small, bounded, KNOWN
worst-case under-travel (`distArriveTol`) for elimination of an unbounded
thrash — an intentional, documented trade-off (SUC-004's postcondition),
not a silent regression.

`distArriveTol` and `stallConfirm` are new real `RobotConfig` fields
(not `SIMSET`-only) per Decision 4: this is firmware behavior that changes
identically on real hardware and sim, unlike ticket 001's pure sim-plant
knobs.

Validate against ticket 001's stiction repro test: must now complete
cleanly (within `distArriveTol`, at rest, no reversal, no thrash) instead
of stalling short. Also add a control test proving the ORIGINAL
zero-stiction plant's behavior is provably unchanged — the stall-confirm
window cannot elapse before the strict crossing fires.

See `architecture-update.md` Step 3 (Planner D-mode decel hook (extended),
`RobotConfig`/`ConfigRegistry` (extended)), Step 5 ("Ticket 003"),
Decisions 4, 5, and 6; `usecases.md` SUC-004.

## Acceptance Criteria

- [ ] New `RobotConfig` fields `distArriveTol` (mm) and `stallConfirm`
      (ms) added via the standard four-file coordinated edit
      (`source/types/Config.h`, `source/robot/DefaultConfig.cpp`,
      `source/robot/ConfigRegistry.cpp`,
      `data/robots/robot_config.schema.json`), following 071's
      no-unit-suffix identifier convention (unit documented in a comment).
      Both are `SET`/`GET`-able.
- [ ] The D-mode decel hook floors `v_cap` at `minWheelSpeed` once
      `d_remaining` is at or below a final-approach threshold, instead of
      allowing `v_cap` to asymptote toward zero.
- [ ] The decel hook tracks whether `d_remaining` is inside `distArriveTol`
      and failing to shrink; once this condition persists for
      `stallConfirm`, the hook calls a new `MotionCommand::forceComplete(reason)`.
- [ ] `MotionCommand::forceComplete(reason)` is a new public method that
      performs the same SOFT-stop teardown (ramp to (0,0), emit the
      configured `_doneEvtLabel` with the given `reason=` token) a normal
      DISTANCE-fired stop already takes, without requiring a
      `StopCondition` to have fired.
- [ ] Against ticket 001's stiction plant configured to reproduce the field
      failure signature (lands 1-3 mm short at near-zero commanded speed),
      a `D 200 200 500` drive completes within `distArriveTol` of 500 mm,
      at rest, with no backward travel and no thrash.
- [ ] The stiction-plant drive completes well before the TIME net would
      fire.
- [ ] `EVT done D` is emitted on both a strict-crossing completion
      (`reason=dist`, unchanged) and a stalled-short completion (a new,
      additive `reason=` token, e.g. `arrive` or `stall`) — hosts that
      only check for `EVT done D` (ignoring `reason=`) see no behavior
      change.
- [ ] A `D` drive against the ORIGINAL zero-stiction plant (no `SIMSET`
      stiction knobs configured) behaves identically to before this
      sprint — proven, not assumed: a control test demonstrates
      `d_remaining` reaches exactly zero via the strict crossing before
      the stall-confirm window could elapse.
- [ ] Default values for `distArriveTol`/`stallConfirm` are chosen/tuned
      against ticket 001's repro test (architecture-update.md Open
      Question 3 intentionally leaves exact numbers to this ticket,
      informed by the field data: 1-3 mm observed overshoot/undershoot,
      sub-second stall before reversal onset).
- [ ] Full existing test suite remains green (no regression against the
      zero-stiction control case).

## Testing

- **Existing tests to run**: `tests/simulation/unit/test_velocity_controller.py`
  (confirms `VelocityController`'s deadband/freeze tests remain valid
  unchanged — this ticket does not touch `VelocityController`), Planner
  D-mode / decel-hook tests, ticket 001's new `PhysicsWorld`/stiction
  tests, full suite.
- **New tests to write**: a D-drive-against-stiction-plant test confirming
  clean terminal completion (within tolerance, at rest, no reversal); a
  control test confirming the zero-stiction plant's behavior is provably
  unchanged (stall-confirm window cannot elapse before the strict crossing
  fires); a `RobotConfig`/`SET`/`GET` round-trip test for the two new
  fields.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Add `distArriveTol`/`stallConfirm` to `RobotConfig` via the
same four-file coordinated-edit pattern 071 established. In
`Planner.cpp`'s D-mode decel hook, add the `v_cap` floor (a min-clamp once
`d_remaining` is inside a final-approach zone) and a stall-confirm
counter/timestamp tracking `d_remaining`'s progress inside `distArriveTol`.
When the counter/timer exceeds `stallConfirm`, call
`MotionCommand::forceComplete(reason)` — a new public entry point mirroring
the existing internal SOFT-stop teardown logic (do not duplicate that
logic; refactor the shared teardown steps into a helper both the
`StopCondition`-fired path and `forceComplete` call, if the existing code
structure makes that natural). Validate against ticket 001's stiction
repro test and add the zero-stiction control test.

**Files to create/modify**:
- `source/types/Config.h`, `source/robot/DefaultConfig.cpp`,
  `source/robot/ConfigRegistry.cpp`,
  `data/robots/robot_config.schema.json` — new `distArriveTol`/
  `stallConfirm` fields.
- `source/superstructure/Planner.cpp`/`.h` — `v_cap` floor; stall-confirm
  counter/timer; call to `MotionCommand::forceComplete()`.
- `source/commands/MotionCommand.h`/`.cpp` — new public `forceComplete(reason)`
  method.
- Test files: new Planner/decel-hook tests exercising the stiction-plant
  and zero-stiction-control scenarios; `RobotConfig` `SET`/`GET`
  round-trip test for the two new fields.

**Testing plan**: run new decel-hook tests in isolation against both the
stiction-configured and zero-stiction plant, then the full suite.

**Documentation updates**: `docs/wire-protocol.md` (or equivalent) for the
new `EVT done D` stalled-short `reason=` token and the two new
`SET`-able fields.
