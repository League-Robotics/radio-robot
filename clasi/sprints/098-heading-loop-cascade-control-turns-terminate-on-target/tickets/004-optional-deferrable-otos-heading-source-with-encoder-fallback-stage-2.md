---
id: '004'
title: '[OPTIONAL/DEFERRABLE] OTOS heading source with encoder fallback (Stage 2)'
status: in-progress
use-cases:
- SUC-004
depends-on:
- '003'
github-issue: ''
issue:
- heading-loop-cascade-control-turns-terminate-on-target.md
- real-robot-motion-calibration-undershoot.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# [OPTIONAL/DEFERRABLE] OTOS heading source with encoder fallback (Stage 2)

## ⚠️ OPTIONAL/DEFERRABLE — skip if the overnight run's risk budget is spent

The mandatory path (001→002→003→006) already satisfies the sprint's
acceptance criterion WITHOUT this ticket. Skip entirely if ticket 003
already consumed the available risk budget, or if ticket 003's own results
were marginal and further hardware iteration is better spent re-tuning
Stage 1's gains than adding a new sensor path. If skipped, ticket 006 notes
the deferral and closes the sprint on Stage 1 alone.

## Description

Revive OTOS ticking in the live `main.cpp` loop and let `Motion::
SegmentExecutor` consume OTOS heading when connected/fresh, falling back to
Stage 1's encoder-derived heading otherwise. Heading (unlike OTOS
*position*, which has the off-center lever-arm problem —
`[[otos-offset-register-unwritable]]`) is mount-offset-independent, so it
is the one OTOS quantity this sprint trusts. Explicitly narrower than
sprint 099 (`restore-pose-estimation-otos-encoders-delayed-camera-fixes.md`'s
full pose-fusion restoration) — this ticket needs OTOS *heading* only.

Reference: `architecture-update.md` M6, Decision 4 (why the existing
`PoseEstimate` seam is reused rather than a new type), Open Question 3
(the I2C timing cost is unverified until measured — THIS ticket measures
it).

Depends on 003 — Stage 1 must be bench-verified-good before layering OTOS
on top of it.

## Acceptance Criteria

- [x] `main.cpp` ticks the OTOS leaf once per pass:
      `hardware.odometer()->tick(now)`, placed AFTER `hardware.tick(now)`
      and BEFORE `drivetrain.tick(...)` so a fresh pose is available before
      the executor consumes it this same pass.
- [x] `main.cpp` commits `bb.otos = hardware.odometer()->pose()` and
      `bb.otosConnected = hardware.odometer()->connected()` for telemetry,
      using `connected() && pose().stamp.valid` (NOT `fusableThisPass()`)
      to derive freshness/validity — `fusableThisPass()`'s one-sanctioned-
      caller, read-and-clear reset-suppression semantics are an
      EKF-fusion-gate concern this loop does not have
      (`architecture-update.md` Decision 4's own note); do not introduce a
      second caller of that method.
- [x] `Subsystems::Drivetrain::tick()` reads `hardware_.odometer()->
      pose()`/`connected()` directly each tick (it already holds
      `Hardware&`) and passes a real `msg::PoseEstimate` — instead of
      today's hardcoded `msg::PoseEstimate{}` — into `Motion::
      SegmentExecutor::tick()`.
- [x] `Motion::SegmentExecutor`'s measured-heading step (ticket 002's own
      PD/completion logic) prefers OTOS heading (`pose.h`) when the
      caller-supplied `PoseEstimate` is valid/connected, relative to a NEW
      baseline field capturing OTOS heading at phase start (mirroring
      `encDiff0`'s existing "relative to phase start" convention) — falls
      back to the encoder-derived heading (ticket 002's unmodified path)
      otherwise, TICK-BY-TICK (not latched for the whole phase — if OTOS
      drops mid-phase, the very next tick falls back to encoders).
- [x] SIM ACCEPTANCE: a new scenario injects an invalid/absent
      `PoseEstimate` and confirms behavior is IDENTICAL to ticket 002's
      encoder-only scenarios (bit-for-bit twist output); a second scenario
      injects a valid `PoseEstimate` with a deliberately-different heading
      than the encoder-derived one and confirms the executor's
      measured-heading step actually uses the OTOS value (observably
      different PD correction than the encoder-only case).
- [x] Full `uv run python -m pytest` stays green, no regression from ticket
      002's own baseline.
- [ ] HARDWARE ACCEPTANCE (do not skip even though this ticket is optional
      — if executed at all, it must be verified, not merely compiled):
  - [ ] A bench/stand check confirms `bb.otosConnected` reads true with the
        OTOS chip present (previously always false/never-set — confirm the
        wire actually shows the change).
  - [ ] A representative `turn_sweep.py --relay --both` subset (at minimum
        the cells ticket 003 used for its scatter check) shows NO
        regression vs. ticket 003's own recorded baseline.
  - [ ] Loop-timing/radio responsiveness is unaffected — no symptom
        matching `[[radio-needs-loop-yield]]` (radio appears dead /
        commands stop being serviced) observed during the session.
  - [ ] If EITHER the accuracy-regression check or the timing check fails,
        this ticket is REVERTED (the `main.cpp`/`drivetrain.cpp` changes
        backed out, `SegmentExecutor::tick()` reverts to receiving
        `msg::PoseEstimate{}`) rather than landed partially — Stage 1 must
        never regress for Stage 2's sake.

## Testing

- **Existing tests to run**: full `uv run python -m pytest`; ticket 003's
  own recorded `turn_sweep.py` results as the regression baseline.
- **New tests to write**: the two sim scenarios itemized above
  (invalid-`PoseEstimate` parity, valid-`PoseEstimate` source-selection).
- **Verification command**: `uv run python -m pytest`;
  `uv run python tests/bench/turn_sweep.py --relay --both` (regression
  subset).

## Implementation Plan

**Approach**: Additive-only changes at three existing call sites
(`main.cpp`'s loop body, `Drivetrain::tick()`'s call into
`executor_.tick()`, `SegmentExecutor`'s internal measured-heading step) —
no new classes, matching `architecture-update.md` M6's boundary.

**Files to modify**: `source/main.cpp`, `source/subsystems/drivetrain.
{h,cpp}`, `source/motion/segment_executor.{h,cpp}`,
`source/motion/motion_baseline.h` (new OTOS-heading baseline field),
`tests/sim/unit/segment_executor_harness.cpp`.

**Files to create**: none.

**Testing plan**: as above.

**Documentation updates**: none required structurally; record the
timing/accuracy measurements in this ticket's completion notes.

## Completion Notes (software portion — programmer)

- **`Motion::SegmentExecutor::tick()`'s signature did not yet carry a
  `PoseEstimate` parameter.** Architecture-update.md Decision 4 assumed
  ticket 002 (Stage 1) would add a `pose` parameter to `tick()` once,
  passing `msg::PoseEstimate{}` unused, so Stage 2 could "just start filling
  it with real data." Checked ticket 002's actual landed code
  (`ed4fcba2`..`3d3de839`): it kept `tick()` at its original 094-001 arity
  (`now, encLeft, encRight`) and the class genuinely pose-free — no
  `PoseEstimate` parameter anywhere. This ticket adds it now: `tick(uint32_t
  now, const msg::MotorState& encLeft, const msg::MotorState& encRight,
  const msg::PoseEstimate& pose = msg::PoseEstimate{})` — a DEFAULTED
  trailing parameter, so every existing caller (`Drivetrain`'s other call
  site never existed since this was the only one; every pre-existing sim
  scenario in `segment_executor_harness.cpp`) compiles and behaves unchanged
  without any explicit update, which also structurally *guarantees* (not
  just achieves via careful gating) the parity acceptance criterion: an
  omitted 4th argument default-constructs `stamp.valid == false`, bit-
  identical to Stage 1's own hardcoded empty `msg::PoseEstimate{}`. This is
  a one-ticket-later landing of Decision 4's own plan, not a conflict with
  it — flagging per the dispatch instructions' "report precisely what you
  found" guidance rather than silently forcing a different shape.
- **New `MotionBaseline` field is `otosHeading0`, not a reuse of the
  existing (dead) `heading0`.** `heading0` is already declared
  (`motion_baseline.h`) but its doc comment specifically means a future
  FULL EKF-fused pose heading (sprint 099's scope) and it is genuinely
  unused everywhere in `source/` today. Reusing it for Stage 2's raw,
  single-sensor OTOS heading would create a real semantic collision for
  whoever lands 099's fusion later (they would find `heading0` already
  "meaning something," incorrectly). Added a distinct `otosHeading0` field
  instead, leaving `heading0`/`pose0X`/`pose0Y` exactly as dead as before
  this ticket.
- **Freshness/connected combination happens once, in
  `Subsystems::Drivetrain::tick()`**, not in the executor: `msg::
  PoseEstimate otosPose = hardware_.odometer()->pose(); otosPose.stamp.valid
  = otosPose.stamp.valid && hardware_.odometer()->connected();` — then
  passed to `executor_.tick()`. `Motion::SegmentExecutor::measuredHeading()`
  therefore has a single gate to check (`pose.stamp.valid`), never queries
  `connected()` itself, and stays free of any `Hal::Odometer` dependency —
  matches Decision 4's "pose-shaped-parameter-capable" framing.
  `measuredHeading()`'s OTOS branch computes `wrapAngle(pose.pose.h -
  baseline_.otosHeading0)` (a small local `wrapAngle()` added to
  `segment_executor.cpp`'s anonymous namespace, identical atan2f/sinf/cosf
  identity to `stop_condition.cpp`'s own file-local one) — exact for any
  single-phase rotation under ±180°, which matches this sprint's scope
  (PRE_PIVOT/TERMINAL_PIVOT are single in-place pivots, never multi-turn);
  `omegaMeasured` stays encoder-derived unconditionally, matching the
  ticket's explicit "OTOS heading only, never rate" scope. `main.cpp`'s
  `bb.otos`/`bb.otosConnected` commit is separate and untouched by this —
  telemetry-only, `bb.otosValid`/`fusableThisPass()` deliberately untouched.
- **Parity scenario (`scenarioOtosInvalidPoseParityWithEncoderOnlyPath`,
  `tests/sim/unit/segment_executor_harness.cpp`)**: shadows two executors —
  A never passed a pose argument (default path every other scenario already
  exercises); B passed an EXPLICIT invalid `PoseEstimate` (`stamp.valid =
  false`, `pose.h = 2.5f` to prove the value is ignored, not coincidentally
  zero) every tick — both with nonzero `heading_kp`/`heading_kd` (2.0/0.3)
  so the pose-gate itself, not `Kp=Kd=0` degeneracy, is under test. Result:
  **bit-identical** (`twistA.v_x == twistB.v_x && twistA.omega ==
  twistB.omega`, exact `==`, every tick) across a PRE_PIVOT+TRANSLATE+
  TERMINAL_PIVOT segment. Confirmed by running the compiled harness binary
  directly (`c++ -std=gnu++20 -fno-exceptions -fno-rtti -DHOST_BUILD=1
  -Wall -Wextra`) — all 13 scenarios print `ALL SCENARIOS PASSED`, exit 0,
  zero compiler warnings.
- **Source-selection scenario
  (`scenarioOtosSourceSelectionUsesOtosHeadingWhenValid`)**: shadows A
  (encoder-only) against B (fed a fabricated OTOS heading reporting 40% more
  rotation than has actually occurred) on a zero-lag/zero-slip plant (so A's
  own P-term stays ~0, nothing to correct against); asserts `|twistB.omega -
  twistA.omega| > 0.05` rad/s is observed — confirming the OTOS value is
  actually consumed by `measuredHeading()`, not silently ignored.
- **Test counts**: baseline stated in the ticket is 898 passed
  (`uv run python -m pytest tests/sim tests/unit -q`). Post-change: **898
  passed** — unchanged, matching ticket 002's own precedent
  (`segment_executor_harness.cpp` is one pytest test regardless of internal
  scenario count; the two new scenarios enrich what that one test proves,
  11 → 13 internal scenarios, without changing the pytest-level count).
- **Builds**: `just build-sim` succeeds (host `libfirmware_host.dylib`).
  `just build-clean` succeeds — firmware hex builds clean (FLASH 86.00%,
  RAM 98.33%, both within the project's normal "always near-full" RAM
  envelope, `[[codal-ram-always-near-full]]`); host sim lib rebuilds after.
  No new compiler warnings from any touched file in either build.
- **Not done (explicitly out of scope, left for the team-lead on
  hardware)**: the HARDWARE ACCEPTANCE block (`bb.otosConnected` bench
  check, `turn_sweep.py --relay --both` regression subset against ticket
  003's baseline, loop-timing/radio-responsiveness check, and the
  revert-if-regressed decision) — left entirely unchecked below. Frontmatter
  `status` left at `in-progress` for the team-lead to finalize after that
  pass.
