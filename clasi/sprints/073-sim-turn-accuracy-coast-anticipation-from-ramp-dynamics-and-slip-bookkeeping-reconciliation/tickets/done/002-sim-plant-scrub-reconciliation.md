---
id: '002'
title: Sim plant scrub reconciliation
status: done
use-cases:
- SUC-002
- SUC-003
depends-on: []
github-issue: ''
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

- [x] `SimHandle`'s constructor (`tests/_infra/sim/sim_api.cpp`) gains
      `hal.plant().setBodyRotationalScrub(effectiveSlip(cfg.rotationalSlip));`
      immediately after the existing `hal.setTrackwidth(cfg.trackwidth);`
      line.
- [x] `Sim()` constructed with zero explicit configuration, `RT 9000` →
      true heading (`sim.get_true_pose()`) lands close to 90° (combined
      with Ticket 001's coast fix — this ticket alone still shows Ticket
      001's ~3.3° coast gap; the combined ≤~1° bar is Ticket 004's).
      Measured: before this ticket (Ticket 001 alone) RT 9000 → 98.921°
      (+8.921°, +9.91%); after this ticket → 91.007° (+1.007°, +1.12%) —
      the over-rotation gap collapses to within Ticket 004's own ≤~1° bar
      already.
- [x] `SIMSET bodyRotScrub=1.0` (explicit override back to neutral) +
      `SET rotSlip=1.0` (identity) still reproduces the pre-existing
      "no correction needed" identity behavior — the construction-time
      seed is an overridable default, not a floor or a locked value.
      Measured: RT 9000 → 90.756° (+0.756°), matching the small pre-
      existing Ticket-001 coast residual, not the scrubbed/inflated cases.
- [x] `PhysicsWorld`'s own class-level default (`_bodyRotationalScrub =
      1.0f`) is UNCHANGED — bare `PhysicsWorld` unit tests that construct
      the class directly (not via `SimHandle`), e.g.
      `test_physics_world_basic.py`, `test_physics_world_body_scrub.py`,
      pass unmodified.
- [x] `PhysicsWorld::setSlip(float straight, float turnExtra)`'s body
      changes from `_rotationalSlip = straight + turnExtra;` to
      `_rotationalSlip = straight;`. `_slipStraight`/`_slipTurnExtra`
      (sub-step A′ inputs) are unchanged.
- [x] `setSlip(0.0f, <any nonzero turnExtra>)` produces
      `_rotationalSlip() == 0.0f` — verified by a new, direct
      `PhysicsWorld` unit assertion (via the existing public
      `rotationalSlip()` accessor), independent of any end-to-end sim
      test. See `tests/simulation/unit/test_073_002_setslip_decouple.py`
      (positive AND negative nonzero `turnExtra`, plus a `straight`-only
      non-effect pin).
- [x] `test_sim_otos_lever_arm.py::test_turn_with_slip_otos_matches_truth_encoder_diverges`
      (066-001, uses `straight=0.7, turnExtra=0.0`), `test_physics_world_basic.py`,
      and `test_physics_world_body_scrub.py` (both `turnExtra=0.0`) pass
      UNMODIFIED — confirmed by running these three files BY NAME, not
      just trusting the full-suite pass count (arithmetic result is
      identical under the new derivation since all three pass
      `turnExtra=0.0`).
- [x] No `RobotConfig` field, wire command, or `SIMSET`/`SIMGET` key is
      added, removed, or renamed. Zero ARM-firmware-linked file touched —
      confirm `sim_api.cpp` and `PhysicsWorld.{h,cpp}` are both
      HOST_BUILD/sim-only. Confirmed: root `CMakeLists.txt` explicitly
      excludes `hal/sim/` from the ARM build
      (`list(FILTER SOURCE_FILES EXCLUDE REGEX ".*/hal/sim/.*")`), and
      `tests/_infra/sim/sim_api.cpp` is a HOST_BUILD-only test-infra file
      never referenced by the firmware build.
- [x] `docs/architecture/architecture-update-069.md`'s Open Question 4
      (consolidating `_rotationalSlip`/`bodyRotScrub`) is narrowed by this
      ticket's Design Rationale reference, not closed — full consolidation
      remains a documented future option (Decision 2), not attempted here.
- [x] Full suite (`uv run python -m pytest`) passes at 2655 + this
      ticket's net new test count, zero unexplained failures. Result:
      2647 passed, 14 failed (2661 collected = 2655 + 6 new tests). Of the
      14 failures: 13 trace to the pre-existing, environmental
      `data/robots/active_robot.json` → `tovez_nocal.json` drift (unrelated
      to this ticket, not committed — see Testing notes below); 1
      (`test_069_rt_90deg_body_scrub.py::test_rt_90deg_identity_no_scrub`)
      is the documented, Ticket-004-owned "default is no longer a no-op by
      design" shift called out in architecture-update.md Step 5. Two
      OTHER tests that hardcoded the old neutral-default assumption
      (`test_sim_commands_registry.py::test_simset_atomic_all_or_nothing_bad_key`,
      `test_068_004_zero_error_three_pose_agreement.py::test_zero_error_three_pose_and_truth_agreement`)
      were deliberately updated in this ticket (not flagged by name in
      architecture-update.md's Step 1 code reading, but the same class of
      casualty from the same root cause, Decision 3's construction-time
      seed) and now pass.

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

### Implementation notes (as executed)

- New test file: `tests/simulation/unit/test_073_002_setslip_decouple.py`
  — a standalone `PhysicsWorld` compile-and-run harness (same pattern as
  `test_physics_world_body_scrub.py`) pinning `setSlip(0.0, <nonzero>)` →
  `rotationalSlip() == 0.0` for both a positive and a negative `turnExtra`
  (mirroring the TestGUI's negated `slip_turn_extra` convention), plus a
  `straight`-only non-effect pin; and two `SIMGET bodyRotScrub`-based tests
  (via the `sim` fixture) confirming the construction-time seed value and
  that it is overridable, not a floor.
- Two EXISTING tests were deliberately updated (beyond the three the
  ticket named as "confirmed unaffected") because Decision 3's
  construction-time seed changes what "the default" means, and both tests
  hardcoded the OLD neutral-default assumption without resetting
  `bodyRotScrub` explicitly:
  - `tests/simulation/unit/test_sim_commands_registry.py::test_simset_atomic_all_or_nothing_bad_key`
    hardcoded `"bodyRotScrub=1.000"` as the baseline; the test's actual
    claim is atomicity (unchanged by a rejected `SIMSET`), not the
    default's numeric value — changed to read the baseline dynamically and
    compare before/after.
  - `tests/simulation/system/test_068_004_zero_error_three_pose_agreement.py`'s
    `_configure_zero_error()` zeroed `slip_turn_extra` and `SET rotSlip=0`
    but never reset the plant's own `bodyRotScrub` — added
    `SIMSET bodyRotScrub=1.0` so "zero error" also neutralizes the new
    construction-time-seeded plant channel (sub-step A/encoders are never
    scrubbed, so an un-reset seed made `encpose=` diverge from
    true/otos/pose on turns). Both changes are additive resets, not
    assertion-weakening.
- Environmental, NOT this ticket's: 13 of the 14 full-suite failures trace
  to a pre-existing `data/robots/active_robot.json` → `tovez_nocal.json`
  pointer drift in the shared working tree (unrelated stakeholder testing,
  confirmed by every failure message referencing `tovez_nocal`/a neutral
  fallback calibration) — `tests/simulation/unit/test_robot_config.py` (8),
  `tests/simulation/unit/test_push_calibration.py` (4),
  `tests/simulation/system/test_070_004_sim_errors_from_cal.py` (1). No
  `data/robots/` file was modified or committed by this ticket.
- Empirical RT measurement (clean, zero-configuration `Sim()`, `tick_for(8000)`,
  `sim.get_true_pose()`):

  | Command | Before ticket 002 (Ticket 001 only) | After ticket 002 |
  |---|---|---|
  | `RT 9000` (90°) | 98.921° (+8.921°, +9.91%) | 91.007° (+1.007°, +1.12%) |
  | `RT 4500` (45°) | 50.105° (+5.105°, +11.34%) | 46.097° (+1.097°, +2.44%) |

  Measured by stashing/restoring the two ticket-002 diffs and rebuilding
  the sim lib (`cmake --build tests/_infra/sim/build --target clean` +
  rebuild) between runs.
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
