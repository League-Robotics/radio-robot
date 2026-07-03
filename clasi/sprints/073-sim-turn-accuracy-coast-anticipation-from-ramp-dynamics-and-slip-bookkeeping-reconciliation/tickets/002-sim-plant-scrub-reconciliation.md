---
id: "002"
title: "Sim plant scrub reconciliation"
status: open
use-cases:
- SUC-002
- SUC-003
depends-on: []
github-issue: ""
issue: sim-turn-undershoot.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim plant scrub reconciliation

## Description

`RobotConfig.rotationalSlip` defaults to `0.92` (real, bench-calibrated
per-robot value, `data/robots/tovez.json`/`togov.json`) and is used by
`Planner::beginRotation()`'s arc inflation
(`arc = ... / effectiveSlip(cfg.rotationalSlip)`) on the assumption that
the PLANT actually scrubs rotation by that factor. `PhysicsWorld::update()`'s
sub-step B computes `slip = effectiveSlip(_rotationalSlip) *
clampScrub(_bodyRotationalScrub)`; `_bodyRotationalScrub` defaults to
`1.0f` at the class level (`PhysicsWorld.h:330`) and is set ONLY by
`setBodyRotationalScrub()`/`SIMSET bodyRotScrub` (sprint 069). A
freshly-constructed, zero-configuration `Sim()` therefore has the
firmware compensating for a scrub the plant never actually applies —
producing the measured "clean sim `RT 9000` over-rotates to ~95.2°"
defect (+8.7%).

This ticket has two parts:

1. **Seed the plant's body-rotational scrub at `SimHandle` construction.**
   `SimHandle`'s constructor (`tests/_infra/sim/sim_api.cpp:164-212`)
   already seeds the plant's trackwidth from the loaded `RobotConfig`
   (`hal.setTrackwidth(cfg.trackwidth)`, its own comment: "harmless
   (idempotent)... documents the dependency"). This ticket adds one more
   line, immediately after it, extending the SAME pattern: `hal.plant().
   setBodyRotationalScrub(effectiveSlip(cfg.rotationalSlip));` — so the
   plant genuinely scrubs by the factor the firmware's inflation already
   assumes, by default, with zero explicit configuration. This is
   HOST_BUILD/sim-only (`tests/_infra/sim/sim_api.cpp` is not
   ARM-firmware-linked); zero real-hardware footprint.
2. **Decouple `PhysicsWorld::setSlip()`'s legacy derivation.**
   `setSlip(float straight, float turnExtra)` currently derives
   `_rotationalSlip = straight + turnExtra`. Every current caller that
   wants a genuine body-truth effect via this channel
   (`test_sim_otos_lever_arm.py`, `test_physics_world_basic.py`,
   `test_physics_world_body_scrub.py` — all grepped and confirmed) passes
   `turnExtra=0.0`; the TestGUI's `slip_turn_extra` control (an
   encoder-report-only knob, the only caller of a nonzero `turnExtra`) can
   currently perturb body truth by accident, relying on
   `effectiveSlip()`'s `<=0` clamp to neutralize the result rather than
   the channel being structurally unreachable. This ticket changes the
   derivation to `_rotationalSlip = straight` — dropping `turnExtra`'s
   effect on body truth entirely, while leaving `_slipStraight`/
   `_slipTurnExtra` (sub-step A′, encoder-report) completely untouched.

See `architecture-update.md` Step 1 (mechanism, confirmed by direct code
read of `PhysicsWorld.cpp:152-181`, `sim_api.cpp:164-212`, and every
`setSlip`/`sim_set_motor_slip` caller), Step 3 (`SimHandle`/`PhysicsWorld`
module boundaries), Step 4b (before/after diagram), Step 5 "Ticket 002",
Design Rationale Decision 2 (why narrow the derivation, not fully
consolidate `_rotationalSlip`/`_bodyRotationalScrub`) and Decision 3 (why
seed at `SimHandle` construction, not change `PhysicsWorld`'s own
class-level default); `usecases.md` SUC-002, SUC-003.

## Acceptance Criteria

- [ ] `SimHandle`'s constructor (`tests/_infra/sim/sim_api.cpp`) gains
      `hal.plant().setBodyRotationalScrub(effectiveSlip(cfg.rotationalSlip));`
      immediately after the existing `hal.setTrackwidth(cfg.trackwidth);`
      line.
- [ ] `Sim()` constructed with zero explicit configuration, `RT 9000` →
      true heading (`sim.get_true_pose()`) lands close to 90° (combined
      with Ticket 001's coast fix — this ticket alone still shows Ticket
      001's ~3.3° coast gap; the combined ≤~1° bar is Ticket 004's).
- [ ] `SIMSET bodyRotScrub=1.0` (explicit override back to neutral) +
      `SET rotSlip=1.0` (identity) still reproduces the pre-existing
      "no correction needed" identity behavior — the construction-time
      seed is an overridable default, not a floor or a locked value.
- [ ] `PhysicsWorld`'s own class-level default (`_bodyRotationalScrub =
      1.0f`) is UNCHANGED — bare `PhysicsWorld` unit tests that construct
      the class directly (not via `SimHandle`), e.g.
      `test_physics_world_basic.py`, `test_physics_world_body_scrub.py`,
      pass unmodified.
- [ ] `PhysicsWorld::setSlip(float straight, float turnExtra)`'s body
      changes from `_rotationalSlip = straight + turnExtra;` to
      `_rotationalSlip = straight;`. `_slipStraight`/`_slipTurnExtra`
      (sub-step A′ inputs) are unchanged.
- [ ] `setSlip(0.0f, <any nonzero turnExtra>)` produces
      `_rotationalSlip() == 0.0f` — verified by a new, direct
      `PhysicsWorld` unit assertion (via the existing public
      `rotationalSlip()` accessor), independent of any end-to-end sim
      test.
- [ ] `test_sim_otos_lever_arm.py::test_turn_with_slip_otos_matches_truth_encoder_diverges`
      (066-001, uses `straight=0.7, turnExtra=0.0`), `test_physics_world_basic.py`,
      and `test_physics_world_body_scrub.py` (both `turnExtra=0.0`) pass
      UNMODIFIED — confirmed by running these three files BY NAME, not
      just trusting the full-suite pass count (arithmetic result is
      identical under the new derivation since all three pass
      `turnExtra=0.0`).
- [ ] No `RobotConfig` field, wire command, or `SIMSET`/`SIMGET` key is
      added, removed, or renamed. Zero ARM-firmware-linked file touched —
      confirm `sim_api.cpp` and `PhysicsWorld.{h,cpp}` are both
      HOST_BUILD/sim-only.
- [ ] `docs/architecture/architecture-update-069.md`'s Open Question 4
      (consolidating `_rotationalSlip`/`bodyRotScrub`) is narrowed by this
      ticket's Design Rationale reference, not closed — full consolidation
      remains a documented future option (Decision 2), not attempted here.
- [ ] Full suite (`uv run python -m pytest`) passes at 2655 + this
      ticket's net new test count, zero unexplained failures.

## Testing

- **Existing tests to run**: `test_sim_otos_lever_arm.py`,
  `test_physics_world_basic.py`, `test_physics_world_body_scrub.py` (all
  three, by name, to confirm arithmetic non-effect), full suite.
- **New tests to write**: a direct `PhysicsWorld` unit test asserting
  `setSlip(0.0, <nonzero>)` → `rotationalSlip() == 0.0`, pinning the
  decoupling independently of any end-to-end angle assertion. A narrow
  `SimHandle`-construction test confirming the seeded
  `_bodyRotationalScrub` matches `effectiveSlip(cfg.rotationalSlip)` for
  the default robot config (optional, useful for isolating a
  construction-wiring bug from a formula bug before Ticket 004's combined
  sweep runs).
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Two independent, file-disjoint edits, may be done in either
order.

For the `SimHandle` seed: open `tests/_infra/sim/sim_api.cpp`, find the
constructor's existing `hal.setTrackwidth(cfg.trackwidth);` line (around
line 164-212 per architecture-update.md's citation), add the new
`setBodyRotationalScrub` line immediately after it, matching its comment
style. Confirm `effectiveSlip` is visible in this translation unit
(transitively via `Odometry.h` per the architecture doc — add the
`#include` explicitly if the build fails to resolve it).

For the `setSlip()` decoupling: open `source/hal/sim/PhysicsWorld.h`,
find `setSlip(float straight, float turnExtra)` (around line 125-129),
change the body from `_rotationalSlip = straight + turnExtra;` to
`_rotationalSlip = straight;`. Update the doc comment above it to state
the narrower derivation and why (mirror Design Rationale Decision 2's
summary: `turnExtra` is encoder-report-only, no longer perturbs body
truth). Do not touch `_slipStraight`/`_slipTurnExtra`'s own assignment
(sub-step A′, unaffected) or sub-step B's own formula in
`PhysicsWorld.cpp` (`slip = effectiveSlip(_rotationalSlip) *
clampScrub(_bodyRotationalScrub)` — unchanged, just now fed a narrower
`_rotationalSlip` input).

Run the three named existing tests in isolation first to confirm zero
arithmetic effect, then add the new direct unit assertion for the
decoupling, then run the full suite. `--clean` sim rebuild required
before running anything (stale incremental builds on `/Volumes` are a
known project gotcha).

**Files to create/modify**:
- `tests/_infra/sim/sim_api.cpp` — `SimHandle` constructor gains the new
  seed line.
- `source/hal/sim/PhysicsWorld.h` — `setSlip()`'s derivation change and
  doc comment update.
- New or existing `tests/simulation/unit/` file (e.g. extend
  `test_physics_world_body_scrub.py` or add a new
  `test_073_002_*.py`) — the new direct `setSlip` decoupling assertion.

**Testing plan**: `test_sim_otos_lever_arm.py`,
`test_physics_world_basic.py`, `test_physics_world_body_scrub.py` in
isolation first (confirm unaffected), then the new decoupling assertion,
then full suite. A manual `RT 9000` sanity check against a fresh `Sim()`
is useful here even though the ≤~1° bar is Ticket 004's to assert formally
(Ticket 001 must also be landed for that number to be meaningful).

**Documentation updates**: `PhysicsWorld.h`'s doc comment above
`setSlip()`; no wire-protocol doc change (no `SIMSET`/`SIMGET` key
changes).
