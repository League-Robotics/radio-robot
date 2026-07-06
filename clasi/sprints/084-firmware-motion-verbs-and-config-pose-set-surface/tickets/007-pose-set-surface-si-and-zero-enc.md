---
id: '007'
title: 'Pose-set surface: SI and ZERO enc'
status: done
use-cases:
- SUC-006
depends-on:
- '006'
github-issue: ''
issue: firmware-config-and-pose-set-surface.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Pose-set surface: SI and ZERO enc

## Description

Register `SI <x> <y> <h>` (mm, mm, centi-degrees) — **undocumented in
`docs/protocol-v2.md` today**, derived from `source_old/commands/
SystemCommands.cpp`'s `handleSI` and confirmed against
`host/robot_radio/testgui/operations.py`'s `on_sync_pose()` (architecture-
update.md Grounding fact 4) — and extend the **already-documented**
`ZERO enc` (`docs/protocol-v2.md` §10) to also reset
`Subsystems::PoseEstimator`'s encoder-baseline accumulator.

**`SI` calls `PoseEstimator::setPose()` directly — it does NOT route
through `Drivetrain::apply()`'s existing `POSE`/`SetPose` oneof arm**,
which stays exactly the documented no-op it is today (architecture-
update.md Decision 1, approved as-is). This is a deliberate,
schema-defying-at-first-glance choice: `msg::DrivetrainCommand::POSE`
looks like an invitation to route pose-set through `Drivetrain`, but
`Drivetrain` holds no `PoseEstimator` reference (082's cohesion split)
and must not gain one just for this. `PoseEstimator` gains a new
`setPose(const msg::SetPose&)` method, re-anchoring both `encoderPose()`
and `fusedPose()`, which needs a small additive `Hal::EkfTiny` method
(re-anchor state/covariance to a supplied pose — distinct from `init()`'s
always-zero reset).

`ZERO enc`'s existing, already-documented contract ("resets the encoder
accumulators") is extended: the handler must ALSO reset `PoseEstimator`'s
encoder-baseline accumulator (`haveEncBaseline_`/`prevEncLeft_`/
`prevEncRight_`) in the same call, so the next tick's delta is computed
against the freshly-zeroed encoders, not a stale pre-zero baseline (which
would otherwise fabricate a phantom jump — the exact hazard
`PoseEstimator`'s own `haveEncBaseline_` guard already exists to prevent
for its very first tick; this ticket is the second place that hazard
applies).

**Wire keys stay stable.** `ZERO`'s existing verb/argument grammar is
unchanged (only its *effect* gains one more reset target); `SI`'s wire
shape is fixed by this ticket, once, matching `source_old`'s and
TestGUI's already-established convention exactly (so no host-side change
is needed later).

## Acceptance Criteria

- [x] `Subsystems::PoseEstimator` gains
      `setPose(const msg::SetPose& pose)`, re-anchoring both
      `encoderPose()` and `fusedPose()` to `(pose.x, pose.y, pose.h)`
      (`h` converted from centi-degrees to the estimator's internal
      radians, matching every other pose field's existing convention).
- [x] `Hal::EkfTiny` gains a small additive re-anchor-to-pose method
      (distinct from `init()`) that resets state/covariance to a
      caller-supplied pose rather than always zero. **Found already
      satisfied**: `EkfTiny::setPose(x, y, theta)` was shipped by sprint
      082, ticket 001 (source/estimation/ekf_tiny.{h,cpp}) — exactly this
      shape (sane diagonal P-prior instead of zeroing P). No new EkfTiny
      code was needed; `PoseEstimator::setPose()` calls it directly.
- [x] New `source/commands/pose_commands.{h,cpp}` registers `SI <x> <y>
      <h>`, calling `PoseEstimator::setPose()` directly — **not**
      `Drivetrain::apply()`'s `POSE` arm, which remains untouched and
      still documented as a no-op.
- [x] `SI 1000 500 900` makes the next `SNAP`'s `pose=`/`encpose=` read
      back at (1000, 500, 900) (sim). **`encpose=` reads back exactly**
      (dead reckoning, immune to EKF/OTOS correction). **`pose=` (fused)
      does NOT read back exactly** — a live sim odometer's next OTOS
      fusion partially pulls it back toward its own, un-reanchored frame
      (measured gain ≈0.667, landing near (333,166,510) rather than
      (1000,500,900) — a substantial, unambiguous shift, not a no-op, but
      not exact). Root cause: `Hal::Odometer` has no mutating capability
      this ticket can call (its faceplate is deliberately read-only —
      `hal/capability/odometer.h`'s own file header); the fix — an
      `Hal::Odometer::apply()` `set_pose` arm wired to `OV` — is ticket
      084-008's own scope (`depends-on: ['007']`, sequenced directly
      after this one), mirroring `source_old`'s own two-call `SI` (pose
      reset **and** `hal.otos().setWorldPose()`). Recorded and tested as
      an observed, documented characteristic (`tests/sim/unit/
      test_pose_commands.py`'s module docstring has the full analysis);
      not a defect in this ticket's own `PoseEstimator::setPose()`/`SI`
      wiring. Flagged for the team-lead/084-008 as a follow-up: consider
      having 008's `OV` (or a combined helper) issued alongside `SI` for
      a full re-anchor.
- [x] `ZERO enc`'s existing handler (wherever it currently lives) is
      extended to also reset `PoseEstimator`'s encoder-baseline
      accumulator; `ZERO enc` rezeroes `enc=`/`encpose=` to
      (0,0,0)-relative with no phantom-jump discontinuity on the
      following tick (verified by asserting the first post-`ZERO` tick's
      `encpose=` delta is small/expected, not a spurious jump). **`ZERO`
      did not previously exist anywhere in `source/`** (grepped the
      `makeCmd`/`makeSchemaCmd` tables — confirmed absent) — registered
      fresh in `pose_commands.cpp`. The "reset haveEncBaseline_ /
      prevEncLeft_ / prevEncRight_ synchronously in the handler" design
      the ticket names was empirically found to still fabricate a phantom
      jump (`Hal::Motor::resetPosition()`'s staged hardware effect lands
      one tick after the synchronous reset consumes the one-shot guard
      against a stale reading) — fixed with a deferred/pending flag
      (`encBaselineResetPending_`) that `tick()` applies on the first
      subsequent genuinely time-advancing (`dt > 0`) call; see
      `pose_estimator.h`'s `resetEncoderBaseline()` doc comment for the
      full empirical trace.
- [x] `docs/protocol-v2.md` gains a new `### SI` section under §10
      (previously absent) and one added sentence to the existing `###
      ZERO` section noting the `PoseEstimator` accumulator reset.
- [x] `SI`'s interaction with an in-flight `Planner` command (e.g. a `G`
      in progress) is left as `source_old` left it — `SI` does not itself
      cancel an active drive; the ticket's test suite records the
      observed behavior (a visible course correction on the next tick,
      per architecture-update.md Open Question 4) rather than asserting a
      specific "correct" outcome that was never designed.

## Implementation Plan

**Approach:** Small, additive changes to `PoseEstimator`/`EkfTiny`
(pose-teleport is a short, well-contained addition to an already-small
class) plus a new thin command file.

**Files to modify:**
- `source/subsystems/pose_estimator.h`, `source/subsystems/
  pose_estimator.cpp` (`setPose()`)
- `source/estimation/ekf_tiny.h`, `source/estimation/ekf_tiny.cpp`
  (re-anchor method)
- wherever `ZERO enc`'s handler currently lives in `source/commands/`
  (extend its effect)
- `docs/protocol-v2.md` (`### SI` new section; `### ZERO` one-sentence
  addition)

**Files to create:**
- `source/commands/pose_commands.h`, `source/commands/
  pose_commands.cpp` (`SI`)

**Testing plan:**
- Sim-level tests: `SI` teleport round-trip via `SNAP`; `ZERO enc`
  no-phantom-jump check; `SI` issued mid-`G` observed-behavior test (not
  asserting a specific "corrected" outcome, per Open Question 4).
- Existing suites stay green.

**Documentation updates:** `docs/protocol-v2.md`'s new `### SI` section
and `### ZERO`'s extended sentence, per Acceptance Criteria.
