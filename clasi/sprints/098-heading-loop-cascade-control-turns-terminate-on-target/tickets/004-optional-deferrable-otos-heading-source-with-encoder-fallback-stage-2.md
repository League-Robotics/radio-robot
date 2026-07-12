---
id: '004'
title: '[OPTIONAL/DEFERRABLE] OTOS heading source with encoder fallback (Stage 2)'
status: open
use-cases: [SUC-004]
depends-on: ['003']
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

- [ ] `main.cpp` ticks the OTOS leaf once per pass:
      `hardware.odometer()->tick(now)`, placed AFTER `hardware.tick(now)`
      and BEFORE `drivetrain.tick(...)` so a fresh pose is available before
      the executor consumes it this same pass.
- [ ] `main.cpp` commits `bb.otos = hardware.odometer()->pose()` and
      `bb.otosConnected = hardware.odometer()->connected()` for telemetry,
      using `connected() && pose().stamp.valid` (NOT `fusableThisPass()`)
      to derive freshness/validity — `fusableThisPass()`'s one-sanctioned-
      caller, read-and-clear reset-suppression semantics are an
      EKF-fusion-gate concern this loop does not have
      (`architecture-update.md` Decision 4's own note); do not introduce a
      second caller of that method.
- [ ] `Subsystems::Drivetrain::tick()` reads `hardware_.odometer()->
      pose()`/`connected()` directly each tick (it already holds
      `Hardware&`) and passes a real `msg::PoseEstimate` — instead of
      today's hardcoded `msg::PoseEstimate{}` — into `Motion::
      SegmentExecutor::tick()`.
- [ ] `Motion::SegmentExecutor`'s measured-heading step (ticket 002's own
      PD/completion logic) prefers OTOS heading (`pose.h`) when the
      caller-supplied `PoseEstimate` is valid/connected, relative to a NEW
      baseline field capturing OTOS heading at phase start (mirroring
      `encDiff0`'s existing "relative to phase start" convention) — falls
      back to the encoder-derived heading (ticket 002's unmodified path)
      otherwise, TICK-BY-TICK (not latched for the whole phase — if OTOS
      drops mid-phase, the very next tick falls back to encoders).
- [ ] SIM ACCEPTANCE: a new scenario injects an invalid/absent
      `PoseEstimate` and confirms behavior is IDENTICAL to ticket 002's
      encoder-only scenarios (bit-for-bit twist output); a second scenario
      injects a valid `PoseEstimate` with a deliberately-different heading
      than the encoder-derived one and confirms the executor's
      measured-heading step actually uses the OTOS value (observably
      different PD correction than the encoder-only case).
- [ ] Full `uv run python -m pytest` stays green, no regression from ticket
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
