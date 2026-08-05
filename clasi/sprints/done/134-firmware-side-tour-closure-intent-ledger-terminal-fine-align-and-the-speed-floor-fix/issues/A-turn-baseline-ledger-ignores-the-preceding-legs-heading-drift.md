---
status: in-progress
priority: high
sprint: '134'
tickets:
- 134-001
- 134-006
---

# Turn undershoot: the cumulative-baseline ledger carries a completed leg's baseline unchanged, ignoring the drift that leg accumulated

## Description

**Supersedes the `decelLatched` hypothesis as the leading explanation for TOUR_1
and TOUR_2's turn undershoot.** Sprint 131 ticket 006 (`7176605a`) disconfirmed
that hypothesis by direct measurement and found this instead.

## What was disconfirmed, and how

Post-130 review F10 proposed that `active_.decelLatched` was a one-way trap: a
transient `plannedRemaining` under-estimate drives the command toward zero, the
latch forbids recovery, and the 0.5 s stall backstop completes the Move wherever
it parked. It was a good hypothesis — angle-independent and additive, matching
the residual's signature.

Ticket 006 instrumented **every** `Planner::planWheels()` call and captured
TOUR_1 and TOUR_2 under both ideal and realistic error profiles — **2,351 ticks**,
covering every Distance and Angle move either tour issues. Findings:

- **Zero** ticks where `decelLatched` was held while a fresh `raw` recomputation
  showed `Accel`/`Hold` — the precise condition the proposed fix releases on.
- Every turn's phase trace is a clean, monotonic Accel → Hold → Decel → Closing.
- Every completion fires through the ordinary profile-complete branch
  (`plannedRemaining` landing within float noise of zero), **never** the stall
  backstop the hypothesis requires.

The latch-release fix was implemented anyway (a genuine latent-safety fix,
verified with a negative control), and the per-turn numbers are **bit-identical
before and after** — which is itself the proof the latch was not the cause.

## The new mechanism

TOUR_1's own `straight_leg_cruise_headings` check shows **leg 3 carrying 6.45° of
max true-heading drift under the *ideal*, zero-noise profile alone** — the
straight leg rotates the robot before the turn begins.

130-010's cumulative-baseline ledger then carries a completed Distance leg's
baseline **unchanged** — it does not fold in the drift that leg accumulated. So
the following turn targets a stale absolute heading and physically rotates
`90 − drift` degrees.

This is the mirror image of the 119-002 axis-drop-coast defect, on the other side
of a straight → turn boundary.

## What does not yet add up

6.45° of drift does not fully account for turn 4's −13.08°. **A second
contributor probably exists.** Do not treat this as a complete explanation — it
is a strong lead with a measured mechanism, not a closed case. Extend ticket
006's instrumentation rather than assume.

## Measured baseline to work against

`src/tests/testgui/test_tour_closure_gate.py`, per-turn error, at `7176605a`:

| tour / profile | 2 | 4 | 6 | 8 | 10 | 12 | 14 |
|---|---|---|---|---|---|---|---|
| TOUR_1 / ideal | −2.567 | −13.080 | −12.419 | −11.959 | −13.482 | −24.699 | — |
| TOUR_1 / realistic | −0.119 | −18.558 | −4.605 | −14.602 | −16.075 | −13.450 | — |
| TOUR_2 / ideal | −2.567 | −13.287 | −14.327 | **+5.192** (146°) | −16.667 | −8.153 | +12.778 |
| TOUR_2 / realistic | −0.119 | −18.660 | −2.902 | **+14.074** (146°) | −20.893 | −13.586 | +14.378 |

Turn 4's −13.080° was measured at **every commit** of sprint 131 and never moved
(base `22a3c368` −13.19°, then −13.080° from ticket 001 onward). That stability
is unusually useful: any movement a future change produces is unambiguously its
own.

**Note the sign flip on the 146° turn**: it now *overshoots* (+5.19 ideal /
+14.07 realistic) where `A-tour2-...` recorded −10.12°. Some sprint-131 ticket
shifted this number's sign, consistent with a **direction-dependent** rather than
magnitude-dependent upstream cause — itself a clue worth chasing.

## What to do

1. Extend ticket 006's instrumentation to attribute the full residual: how much
   is stale-baseline carry, how much is the unexplained remainder.
2. Fold the completed leg's actual heading drift into the ledger's carried
   baseline, or re-anchor the turn's target on measured heading at activation.
3. Explain the 146° sign flip before assuming any fix is complete.
4. **Do not add a rotation fudge constant.** `tovez_nocal.json` already carried
   rotation constants fitted to a sim artifact (1.006 / +12.1°) that injected
   ~12.5° of under-rotation into every real turn. Another correction layer is how
   that happened the first time.
5. Measure on the **single-step** harness (`test_tour_closure_gate.py`). The
   background-tick-thread harnesses are not bit-reproducible and ~30 mm of
   apparent spread will drown the signal.

## Verification

- Per-turn error within the shaped band across TOUR_1 and TOUR_2, both profiles.
- `test_tour_closure_gate.py` passes — it fails today and has failed since before
  sprint 131.
- The attribution from step 1 is recorded, so the residual is *explained* rather
  than merely reduced.

## Related

- Sprint 131 ticket 006 (`7176605a`), archived at
  `clasi/sprints/done/131-observable-over-proxy-.../tickets/done/006-*.md` — the
  disconfirmation, the instrumentation method, and the latch fix that shipped
  alongside it.
- [[A-tour2-146-degree-turn-still-undershoots-after-130-010]] — the originating
  issue, deliberately NOT closed by ticket 006 (`completes_issue: false`).
- `docs/code_review/2026-08-02-post-130-wheels-solid-review.md` F10 — the
  hypothesis this disconfirms. The review's reasoning was sound; the measurement
  disagreed.
