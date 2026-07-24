---
status: pending
filed: 2026-07-23
filed_by: programmer (119-002 boundary-test diagnosis)
related:
- specify-and-assert-the-leg-handoff-contract.md
- chain-advance-completion-margin-narrow-pocket.md
tickets:
- 119-002
sprint: '122'
---

# Chain-advance completing-axis reset defeats same-axis compatible-leg continuity (SUC-003 regression)

## Description

`App::MoveQueue::tick()` (`src/firm/app/move_queue.cpp`) hard-resets the
axis matching the ENDING `Move`'s own stop-condition kind
(`shaperVX_`/`shaperOmega_`) to `(commandedSpeed=0, commandedAccel=0)` at
EVERY completion boundary — chain-advance or drain, unconditionally (118
ticket 003's own kept decision, see that block's comment). This reset
fires regardless of whether the INCOMING chained `Move` also commands
that same axis. For two genuinely compatible, same-axis, same-kind
chained legs (e.g. two `Distance` legs at the same `v_max`), the
completing axis's shaper is zeroed and then the chained `Move`'s own
`activate()` reads that just-zeroed `commandedSpeed()` back as its own
carried starting point — so the robot decelerates to near-zero and
re-accelerates at the boundary instead of carrying straight through.
This defeats SUC-051's own "seamless hand-off" intent and SUC-003's
"no dip to zero at a compatible same-`v_max` boundary" property for
exactly the case those use cases exist to serve.

**Reproduction**: `test_two_compatible_distance_legs_carry_velocity_
through_the_boundary_at_tour_level`
(`src/tests/testgui/test_tour_closure_gate.py`) — two 300mm `Distance`
legs at `v_max=150mm/s`. Measured (119-002, current tree, `--runxfail`):
velocity dips to 24.0mm/s (16% of `v_max`, far under the test's own 90%
no-dip floor) at the leg boundary, then recovers to cruise over ~8
cycles (~320ms) via a clean, monotonic accel/jerk-limited ramp — the
`Motion::VelocityShaper` restarting from a hard `(0,0)` reset, NOT the
previously-diagnosed reorder/stale-encoder oscillation (that failure mode
showed erratic, non-monotonic values alternating between roughly half
and above-`v_max`; this one does not).

**Why this is newly visible, not newly introduced**: this test's own
`SimLoop` session ran with shaping SILENTLY OFF before 119 ticket 001
(the "silent-off config boundary" D1 defect,
`docs/code_review/2026-07-22-turn-execution-review.md`) —
`configure_from_robot()` did not push `a_max`/`a_decel`/`alpha_max`/
`alpha_decel`/`j_max`/`yaw_jerk_max`, so `linearShaping` was `false` and
`activate()` staged the raw cruise target directly
(`Motion::VelocityShaper` never entered the picture — no taper, no
reset, no dip possible). 119 ticket 001 made that push unconditional
(default-on), so this test now genuinely exercises the shaped/reset path
for the first time since 118 ticket 003 made the reset unconditional —
the regression was latent, not new.

**Why 118 ticket 003's own re-sweep never caught this**: that ticket's
resolution explicitly tested and rejected a conditional (skip-the-reset-
on-chain-advance) variant against the tour-closure gate's turn-accuracy
metric (best worst-case 2.932° vs the shipped 2.323° — reverted, kept
unconditional as "more conservative"). But that sweep exercised ONLY
TOUR_1/TOUR_2, whose legs always ALTERNATE `Distance`/`Angle` — a
same-axis, same-kind compatible chain (this test's own scenario) never
occurred in that sweep, so the unconditional reset's cost to THIS
property was never measured there.

## Why not fixed directly (119-002's own scope boundary)

119 ticket 002 (the chain-advance leg hand-off contract) explicitly
excludes changing `MoveQueue::tick()`'s completion-handling logic or
re-deriving `kStoppingMarginFactorChain`/`kDiscretizationCyclesChain` —
those constants sit in an already-narrow, ~90-build-swept accuracy
pocket (`chain-advance-completion-margin-narrow-pocket.md`), and any
change to the reset step's own conditions would need to be re-swept
against that same gate before shipping. A same-axis-aware conditional
reset is a real candidate fix but is genuinely new work, not a
specify-then-assert task.

## Candidate fix / unblocking condition

Make the completing axis's reset conditional on whether the INCOMING
chained `Move` (`pending_[0]`, when `pendingCount_ > 0`) shares the
ending `Move`'s own stop-kind axis:

- Same axis, same kind (e.g. `Distance` -> `Distance`, both use `v_x`):
  skip the reset — carry `commandedSpeed()`/`commandedAccel()` straight
  through, restoring SUC-051/SUC-003 for the compatible case.
- Different axis or kind (the TOUR_1/TOUR_2 case 118 ticket 003 actually
  tuned against): keep the reset — this is the case the reset was added
  to protect (a stale residual leaking into a LATER Move's `landAtZero()`
  `remaining` computation on the SAME axis, once that axis is reused by
  some future Move).

Unblocks when a ticket implements this (or an equivalent same-axis-aware
condition) AND RE-SWEEPS `kStoppingMarginFactorChain`/
`kDiscretizationCyclesChain` jointly with it against the tour-closure
gate (the unconditional reset was part of what that sweep tuned
against, so a conditional version needs its own pass, not an assumed
carry-over) — or, if that re-sweep finds the conditional variant
regresses chain-turn accuracy again (as the pendingCount()-gated
variant already did once), an explicit stakeholder decision to accept
the same-axis dip and replace this test's own no-dip assertion with a
bounded-recovery-time one instead (state the accepted recovery window
in cycles/ms).

## Acceptance (future ticket)

- Either: `test_two_compatible_distance_legs_carry_velocity_through_the_
  boundary_at_tour_level` passes with its existing 90%-of-`v_max`
  no-dip floor intact (same-axis-aware reset shipped, re-swept clean
  against the tour-closure gate), or the stakeholder has explicitly
  accepted the dip and the test's own assertion is replaced with a
  stated, bounded-recovery-time check.
- `chain-advance-completion-margin-narrow-pocket.md`'s own narrow-pocket
  finding is re-verified (not silently changed) by whatever the fix
  turns out to be.
