---
id: '005'
title: 'Fix 118-001 straight-leg crab: same-generation actuation staging + same-generation
  telemetry pairs'
status: done
use-cases: []
depends-on: []
github-issue: ''
issue: straight-leg-crab-118-001-actuation-and-telemetry-pairing-skew.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix 118-001 straight-leg crab: same-generation actuation staging + same-generation telemetry pairs

## Description

**Filed from a concurrent, stakeholder-directed session**
(`clasi/issues/straight-leg-crab-118-001-actuation-and-telemetry-pairing-skew.md`,
committed `b04c5a0c`, with a turn-execution review §9 addendum and a repro
script,
`docs/code_review/2026-07-22-turn-execution-review-scripts/straight_drift_repro.py`).
118-001's schedule restore (commit `3189086f`) introduced two defects
together, verified live (v0.20260723.1) and reproduced headlessly on this
checkout:

- Truth/OTOS/fused pose ends at **y ≈ +31-32mm** over x ≈ +708mm on a
  straight `move_twist(v_x=150, stop_distance=700)` leg; final heading
  0.0° (yaw appears during accel, holds through cruise, cancels during
  decel). Measured truth heading DURING CRUISE: **+2.685°**.
- The host-visible encoder view is perfectly straight the entire time:
  per-frame `dL − dR = +0.00` on every frame, `enc L +708 R +708`,
  encpose y=0 θ=0. Firmware's own `pose` row agrees with truth (y +31) —
  only the host-side ENCODER view lies.

A "straight" leg translates along a line ~2.7° off its own heading, and
the encoder trace is structurally blind to it. Every accel/decel phase of
every Move (straight or turn) injects the same transient; hardware
inherits it identically at `kCycle=40ms`.

## Cause (two defects, both introduced by 118-001's schedule restore)

**A — One-cycle L/R actuation skew.** `drive_.tick()` sits in the
R-settle block, BETWEEN `motorL_.tick()` and `motorR_.tick()`
(`robot_loop.cpp`, current lines ~552-594). L writes duty from the target
staged LAST cycle; R writes THIS cycle's fresh target (the block's own
comment already says so: "−1 cycle" for L). During any ramp, R
physically leads L by one cycle. Predicted yaw transient
`Δθ = v_cruise · kCycle / b = 150 · 0.040 / 128 = 2.69°`; measured
**2.685°** — an exact mechanism match. Decel restores it, so the net
signature is lateral displacement with ZERO final heading error:
`y ≈ 660 · sin(2.69°) ≈ +31mm`; measured **+32.5mm**.

**B — Telemetry pairs fresh L with stale R.** `updateTlm()` + `tlm_.emit()`
run in the kClear block, after collect L but BEFORE collect R (current
lines ~554-562), so every frame carries THIS cycle's L against LAST
cycle's R. This pairing skew numerically CANCELS the physical skew from
(A) — measured `dL − dR = +0.00` every frame — so `encpose`/
`frame.twist`/the encoder trace report a straight path while the body
crabs. Any host consumer of per-frame L/R pairs is skewed during ramps
regardless of (A) alone.

**Root note (verified, not speculation):** 118-001 retired the 112-005
`drive_.tick()` hoist believing it was entangled with the 2026-07-18
glued-encoder failure — but that bug was SELECT ORDERING (both selects
issued before either collect), a completely different mechanism from
actuation-generation symmetry. The hoist was the part keeping L/R
actuation symmetric; the 118-001 restore threw out the good half (staging
symmetry) along with the bad (select ordering it was never actually
responsible for). This ticket restores the symmetric staging point
WITHOUT reintroducing the select-ordering bug — both properties are
achievable simultaneously; they were never in tension.

## Fix (both required — per the issue, each alone is insufficient)

1. **Fix A — same-generation actuation staging.** Stage wheel targets
   once per cycle at a point where BOTH motor ticks write the SAME
   generation. The issue's suggested placement: `drive_.tick()` above
   `motorL_.requestSample()` (both wheels then apply THIS cycle's stage,
   symmetrically ONE cycle old — not asymmetric as today). This preserves
   the per-port select→settle→collect interleave (118-001's own, correct,
   fix for the actual glued-encoder bug) — moving `drive_.tick()`'s
   position does not touch select ordering at all, they are orthogonal.
2. **Fix B — same-generation telemetry pairs.** Emit telemetry after BOTH
   collects. The issue's suggested placement: start of the pace block.
   **Placement latitude** (explicitly granted by the coordinating
   dispatch): the implementer MAY place `updateTlm()`/`emit` later in the
   pace block (e.g. after `odom_.integrate()`/`moveQueue_.tick()`) IF that
   demonstrably still produces same-generation L/R pairs on every frame,
   AND the resulting ack-latency consequence (a command's ack riding a
   later-emitted frame) is documented in both the harness and
   `docs/protocol-v4.md`. Same-generation L/R pairs is the HARD
   requirement regardless of exact placement within the pace block; do
   not trade correctness for convenience of placement.

Fixing (1) without (2) leaves twist/encpose numerically skewed during
ramps (the crab becomes visible in TLM but the actuation defect remains).
Fixing (2) without (1) makes the existing crab visible without removing
it. Ship both together.

## Gate addition (acceptance, per the issue — "would have caught both")

- On every straight closure-gate leg, assert TRUTH heading DURING CRUISE
  stays within a few tenths of a degree of the commanded heading —
  endpoint-only checks are PROVABLY BLIND to this failure shape (final θ
  error was measured 0.00° while the leg crabbed 31mm; an endpoint-only
  assertion cannot distinguish "never crabbed" from "crabbed and
  cancelled exactly by symmetric decel").
- Add the repro script's own scenario
  (`straight_drift_repro.py`'s `move_twist(v_x=150, stop_distance=700)`
  case) as a permanent regression test: y displacement over a 700mm
  straight leg must stay within a few mm (not the current ~31mm).
- Re-run the FULL closure-gate + button-acceptance gate set after the
  fix — this defect affects every accel/decel phase of every Move
  (straight AND turn legs), not just the straight case the issue
  happened to isolate; confirm no other gate regresses and that turn
  legs, which already have their own accuracy bands, don't hide a
  related symptom the straight-leg check newly surfaces.

## Design overlay coordination

`src/firm/app/DESIGN.md`'s own narrative describing `cycle()`'s call
order (§2/§4, and the inline `robot_loop.cpp` comments this ticket also
touches) currently states the 118-001-restored order as correct and
final — it needs updating to describe the corrected staging point and
telemetry-emit placement this ticket ships, and to note explicitly that
the symmetric-staging property (the 112-005 hoist's one genuinely good
half) is now restored without reintroducing the select-ordering bug the
hoist's retirement was actually about. **This is a DIRECT edit** on
`src/firm/app/DESIGN.md`'s canonical path — this sprint's overlay slot
belongs to ticket 002 (`src/firm/motion/DESIGN.md`) and is untouched by
this ticket. Sequence this edit before ticket 004 (docs relocation) runs,
since ticket 004 also touches this same file and must not describe the
pre-fix schedule as current.

## Acceptance Criteria

- [x] Fix A shipped: wheel-target staging point relocated so both
      `motorL_.tick()` and `motorR_.tick()` write the SAME generation's
      target (symmetrically one cycle old, or better — not the current
      asymmetric −1/−0 split). `drive_.tick()` now runs at the very top of
      `cycle()`, before `motorL_.requestSample()`.
- [x] Fix B shipped: `updateTlm()`/`emit` relocated so every frame pairs
      same-generation L/R encoder samples. If placed later than "start of
      pace block," the ack-latency consequence is documented in the
      harness AND `docs/protocol-v4.md`. Placed at the START of the pace
      block (the issue's own suggested default) — MOVE-completion ack
      latency is UNCHANGED ("rides the next frame"); enqueue/command acks
      (CONFIG/MOVE-enqueue/STOP) now typically ride the SAME cycle instead
      of the next — documented in `docs/protocol-v4.md` §7.2 and
      `app_robot_loop_harness.cpp`'s updated scenario comments.
- [x] Per-port select→settle→collect interleave (118-001's own fix for
      the actual 2026-07-18 glued-encoder bug) is UNCHANGED — verified:
      `grep 'runAndWait\|sleepUntil' robot_loop.cpp` still shows the same
      4-block wait list (kSettle/kClear/kSettle/kPace) with L
      select→settle→collect strictly before R select→settle→collect; no
      select-ordering regression.
- [x] Straight closure-gate legs assert truth heading DURING CRUISE
      within a few tenths of a degree — new assertion, not just an
      endpoint check. Added `StraightLegCruiseCheck`/
      `_assert_tour_gate(cruise_heading_tolerance_deg=...)` to
      `test_tour_closure_gate.py`, sampling truth heading every cycle for
      each straight (distance) leg's own full duration.
- [x] `straight_drift_repro.py`'s scenario added as a permanent
      regression test; y displacement over 700mm straight ≤ a few mm. New
      file `src/tests/sim/system/test_straight_leg_crab_regression.py` —
      measured final y=+0.0mm (tolerance 3mm), cruise heading 0.0000deg
      (tolerance 0.3deg).
- [x] Full closure-gate + button-acceptance gate set re-run and green
      (turn legs included — this defect is not straight-leg-specific).
      Required an UNPLANNED re-sweep of BOTH `MoveQueue::landAtZero()`
      margin constants (not just the chain one anticipated in the plan) —
      see "Re-sweep record" below.
- [x] Full `uv run python -m pytest` suite green: 1387 passed, 2 skipped,
      9 xfailed, 2 xpassed, 0 failed.
- [x] `src/firm/app/DESIGN.md` updated (direct edit, not the overlay) to
      describe the corrected staging/emit placement and the
      restored-symmetric-staging-without-reintroducing-select-ordering-bug
      note, plus the margin re-sweeps.
- [x] Sequenced before ticket 004 (docs relocation) — this ticket's own
      `src/firm/app/DESIGN.md` edit lands in this same commit, before
      ticket 004 runs.
- [x] Bench verification is DEFERRED to the phase-B bench session per
      this sprint's stated mandate — not required to close this ticket
      (same posture as every other ticket in 118/119).

## Re-sweep record (unplanned — discovered re-running the full gate set)

The plan anticipated only `kStoppingMarginFactorChain` might need
re-deriving (the already-known-narrow chain-advance pocket). Re-running
the FULL gate set (this ticket's own acceptance criterion) surfaced a
SECOND, unanticipated regression: Fix A's symmetric actuation staging
also shifts the AVERAGE commanded-to-duty latency (both leaves now lag
their own freshly-staged target by 1 cycle; previously R lagged 0, L
lagged 1, averaging 0.5) — this shifts BOTH of `MoveQueue::landAtZero()`'s
margin factors, not just the chain one.

**`kStoppingMarginFactorChain`** (`pendingCount() > 0`, `move_queue.cpp`):
old 0.60 re-measured 3.457° worst-case (TOUR_2/ideal turn 10,
`test_tour_closure_gate.py`), over its 2.5° gate. Fresh 1-D sweep at this
schedule:

    0.20: 4.111  0.30: 2.852  0.38: 2.852  0.40: 2.357  0.42: 2.357
    0.45: 2.481  0.48: 2.218  0.50: 2.342  0.52: 2.521  0.55: 2.748
    0.60: 3.457  0.65: 6.660  0.70: 7.266  0.80: 10.294 0.90: 12.378
    1.00: 14.255 1.10: 15.066                              [deg, worst-case]

Genuinely broad plateau `[0.40, 0.50]` (unlike 118-003's own narrow-pocket
finding). **Shipped: 0.48** (worst=2.218°, 0.282° margin under 2.5°).

**`kStoppingMarginFactorFinal`** (`pendingCount() == 0`, `move_queue.cpp`)
— NOT in the original plan; 118-003 found this regime cadence-robust and
never touched it. Old 1.00 re-measured a genuine 3.267°/3.178° UNDERSHOOT
on isolated ±90° managed turns (settle-based —
`test_gui_button_acceptance.py`'s `test_managed_angle_preset[±90]`/
`test_managed_seg_0_cdeg_turn[±90]`, caught by the full-suite run, not
anticipated). Fresh sweep (fast standalone `SimLoop`-based measurement,
same settle-based convention):

    0.50-0.65: 4.248  0.70-0.85: 2.998  0.87: 2.909 (asymmetric)
    0.88-0.96: 0.316  0.97-1.00: 3.267                     [deg, worst |error|]

Genuinely broad plateau `[0.88, 0.96]`. **Shipped: 0.92** (worst=0.316°,
2.68° margin under button-acceptance's 3.0° gate).

Both re-sweeps documented in full in `move_queue.cpp`'s own
anonymous-namespace comments and `src/firm/app/DESIGN.md` §1. Verified
green after both changes: closure gate (worst 2.218°), full
`test_gui_button_acceptance.py` (45 passed, 1 skipped — the skip is
pre-existing, unrelated), straight-leg-crab regression test, full
`uv run python -m pytest` suite (1387 passed / 0 failed).

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full suite); sim
  tour-closure gate (all legs, straight AND turn); button-acceptance
  suite; `app_robot_loop_harness` (schedule-order assertions).
- **New tests to write**: a cruise-heading assertion on straight
  closure-gate legs; a permanent regression test from
  `straight_drift_repro.py`'s own scenario; if telemetry placement moves
  later than "start of pace block," a harness assertion documenting the
  ack-latency consequence.
- **Verification command**: `uv run python -m pytest`, plus a direct run
  of `straight_drift_repro.py` against the rebuilt
  `src/sim/build/libfirmware_host.*` to confirm the measured crab is
  gone (or within the few-mm acceptance band), plus the full sim
  tour-closure gate and button-acceptance suite runs.
