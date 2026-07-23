---
status: pending
filed: 2026-07-23
filed_by: team-lead (turn-execution review §9 addendum; stakeholder-directed)
related: []
tickets: []
---

# Straight legs crab ~31mm/700mm: 118-001 schedule introduces L/R actuation skew AND a telemetry pairing skew that hides it

## Description

Observed live (v0.20260723.1) and reproduced headlessly on this checkout
(deterministic sim, ideal chip, `move_twist(v_x=150, stop_distance=700)`):

- Truth / OTOS / fused end at **y ≈ +31-32mm** over x ≈ +708mm; final heading
  **0.0°** (the yaw appears during accel, holds through cruise, cancels during
  decel). Measured truth heading during cruise: **+2.685°**.
- Host-visible encoder view is perfectly straight the whole time: per-frame
  `dL − dR = +0.00` on **every** frame, `enc L +708 R +708`, encpose y 0 θ 0.
- Firmware's own `pose` row agrees with truth (y +31) — only the host-side
  encoder view lies.

So a "straight" leg translates along a line ~2.7° off its heading, and the
encoder trace cannot see it. Every accel/decel of every Move (straight or
turn) injects the same transient; hardware inherits it identically at
kCycle = 40ms.

## Cause

Two defects introduced together by 118-001's schedule restore
(`robot_loop.cpp`, commit 3189086f):

**A — One-cycle L/R actuation skew.** `drive_.tick()` sits in the R-settle
block, *between* `motorL_.tick()` and `motorR_.tick()`. L writes duty from the
target staged **last** cycle; R writes **this** cycle's fresh target (the
block's own comment says so: "−1 cycle" for L). During any ramp, R physically
leads L by one cycle. Predicted yaw transient
`Δθ = v_cruise · kCycle / b = 150 · 0.040 / 128 = 2.69°`; measured **2.685°**.
Decel restores it, so the net signature is lateral displacement with zero
final heading error: `y ≈ 660 · sin(2.69°) ≈ +31mm`; measured **+32.5mm**.

**B — Telemetry pairs fresh L with stale R.** `updateTlm()` + `emit` run in
the kClear block, after collect L but *before* collect R, so every frame
carries this cycle's L against last cycle's R. The pairing skew numerically
cancels the physical skew from (A) — measured `dL − dR = +0.00` every frame —
so encpose/`frame.twist`/the encoder trace report a straight path while the
body crabs. Any host consumer of per-frame L/R pairs is skewed during ramps
regardless of (A).

Note: 118-001 retired the 112-005 `drive_.tick()` hoist believing it was part
of the 2026-07-18 glued-encoder failure, but that bug was select *ordering*
(both selects before either collect). The hoist was the part keeping L/R
actuation symmetric — the restore threw out the good half with the bad.

## Fix

Both required; both preserve 118-001's per-port select→settle→collect
interleave:

1. Stage wheel targets once per cycle at a point where **both** motor ticks
   write the same generation — e.g. `drive_.tick()` above
   `motorL_.requestSample()` (both wheels then apply this cycle's stage,
   symmetrically one cycle old).
2. Emit telemetry after both collects (move `updateTlm()`/`emit` to the start
   of the pace block) so frames carry same-generation encoder pairs.

Fixing (1) without (2) leaves twist/encpose skewed during ramps; fixing (2)
without (1) makes the crab visible but still present.

**Gate addition (would have caught both):** on a straight closure-gate leg,
assert truth heading during cruise stays within a few tenths of a degree —
endpoint-only checks are blind to this failure shape (final θ error is 0.00°).

## Related

- `docs/code_review/2026-07-22-turn-execution-review.md` §9 — full analysis
  with the measured per-cycle table.
- Repro: `docs/code_review/2026-07-22-turn-execution-review-scripts/straight_drift_repro.py`
  (run against `src/sim/build/libfirmware_host.*`).
- Commit 3189086f (118-001 schedule restore); retired 112-005 hoist.
- 2026-07-18 glued-encoder observation (select-latch ordering), quoted in
  `robot_loop.cpp`'s cycle() comment.
