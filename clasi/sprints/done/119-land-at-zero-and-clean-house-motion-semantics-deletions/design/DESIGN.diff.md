---
source_file: DESIGN.md
source_hash: 8b3475a600e9bda38fc3bd5d64019e5a63b222f487eddf49b13dcfe7b3d89e0a
---
# Diff: DESIGN.md

Comparison of the sprint overlay copy of `DESIGN.md` against its pristine (seed-commit) canonical version.

```diff
--- DESIGN.md (pristine)
+++ DESIGN.md (current)
@@ -273,6 +273,89 @@
 `shapeAndStage()` (`move_queue.cpp`) is the ONE caller — see that file's
 own doc comment for the per-`Move`-kind axis-selection policy.
 
+**Chain-advance leg hand-off contract (119, DRAFT — verify/refine against
+shipped `move_queue.cpp` at execution time, same convention 118 ticket
+004 used for this overlay).** Moved out of §6 Open Questions: what the
+carried shaper state SHOULD do at a `Move`-completion boundary is a
+specified contract, not a tuned-around limitation.
+
+- **The axis matching the ending `Move`'s own stop-condition kind is
+  ALWAYS hard-reset to `(commandedSpeed=0, commandedAccel=0)` at the
+  completion boundary — chain-advance or drain, unconditionally.**
+  `Kind::Angle` resets `shaperOmega_`; `Kind::Distance` resets
+  `shaperVX_` (`move_queue.cpp`, the unconditional reset ahead of the
+  chain-advance/drain branch). This is NOT the "shaped decay from
+  carry-over" this section's own Open Questions entry used to describe —
+  118 ticket 003's resolution explicitly tested a conditional variant
+  (skip the reset on chain-advance, let the next Move's own accel-ramp
+  decay the residual naturally) against the 40ms closure gate and found
+  no improvement (best worst-case 2.932°, itself just as fragile) —
+  reverted, kept unconditional. **Correction to this issue's own
+  proposed-fix text**, which speculated "current: shaped decay from
+  carry-over" — that was accurate pre-118; the shipped, tested, and kept
+  behavior is unconditional reset. Rationale: a `Move` can end with a
+  nonzero residual `commandedSpeed_` (both the threshold backstop and the
+  land-at-zero predicate tolerate an imperfect-zero taper); without the
+  reset, that residual leaks into whichever LATER `Move` next uses the
+  SAME axis and corrupts ITS land-at-zero `remaining` computation with a
+  value describing the wrong `Move`.
+- **The axis NOT matching the ending `Move`'s stop-condition kind is
+  UNTOUCHED at a chain-advance boundary — SUC-051's own continuity
+  property, unchanged.** If the next `Move` commands that axis, it ramps
+  from wherever `commandedSpeed()` already was — genuine continuity, not
+  a from-rest restart. Only a full drain (`pendingCount() == 0`) resets
+  ALL FOUR shapers (`shaperVX_`/`shaperOmega_`/`shaperVLeft_`/
+  `shaperVRight_`) to `(0, 0)`, since the robot has genuinely stopped and
+  the NEXT unrelated `Move` (whenever it activates) must not inherit a
+  stale nonzero pair from a taper that never finished (e.g. a
+  timeout-backstop ending mid-taper).
+- **Sign reversal does not survive a boundary, by construction — not a
+  separate case to specify.** Because the completing axis's shaper is
+  unconditionally reset to `(0, 0)` (above), there is no carried nonzero
+  speed for a reversal to "survive" in the first place; the shaper-level
+  question the original issue posed is subsumed by the unconditional-
+  reset rule. What remains a genuinely separate, unresolved question is
+  the HARDWARE-level asymmetry: `NezhaMotor`'s 100ms `reversal_dwell_ms`
+  arms on the reversing wheel only at a D→RT boundary (asymmetric by
+  construction, `nezha_motor.cpp`) — 118 did not touch this. **Open
+  decision for the ticket owner**: accept the asymmetric per-wheel dwell
+  (state its measured heading-cost budget) OR specify symmetric dwell
+  (both wheels wait) if that budget is rejected — this contract
+  paragraph must pick one explicitly, not leave it implicit.
+- **vExit design reference
+  (`simple-velocity-control-acceleration-limited-shaper.md`) — adopted,
+  in the sense the shipped mechanism already matches its "0 on reversal
+  or empty queue" half exactly** (the unconditional completing-axis reset
+  above IS vExit=0, applied unconditionally rather than only on reversal/
+  empty-queue, which is a strictly more conservative special case of the
+  same rule). Its "ramp from next move's cruise" half describes the
+  SURVIVING axis's SUC-051 continuity, not the completing axis. No
+  separate vExit mechanism needs implementing — the existing reset +
+  continuity split already realizes it.
+- **Axis-drop coast at chain boundaries — the mechanism
+  `chain-advance-completion-margin-narrow-pocket.md` (filed 2026-07-23
+  from 118 ticket 003's resolution) traces the chain-advance completion
+  margin's narrow accuracy pocket to.** Tours alternate Distance/Angle
+  legs, so a chain-advance turn's own axis (`omega`, say) is exactly the
+  axis the NEXT `Move` does not command — it is the completing-and-reset
+  axis above, not a surviving one. Completion is scored at the ack
+  instant (the cycle `landAtZero()`/the threshold fires), but the plant's
+  own physical coast on that now-reset-to-zero-command axis is only
+  PARTIALLY visible by that instant — the reset zeroes the COMMAND, not
+  the plant's own residual angular/linear velocity, which continues to
+  decay physically for a few more cycles the ack-instant score never
+  observes. This is the concrete "axis-drop coast" this contract names:
+  the gap between "commanded axis reset to zero" (this cycle) and
+  "plant physically at rest on that axis" (a few cycles later, unscored
+  by the chain-advance ack-instant metric). `kStoppingMarginFactorChain`/
+  `kDiscretizationCyclesChain` (`move_queue.cpp`) are the swept
+  compensations for exactly this gap — this paragraph specifies WHY they
+  differ from the final-move case (which scores against a
+  settle-consistent, not ack-instant, completion), not a new mechanism to
+  implement. Closing the narrow pocket itself (rather than just naming
+  its mechanism) is out of this ticket's own scope — see the pool issue's
+  own "not urgent... future sprint" disposition.
+
 ## 5. Interfaces
 
 ### Exposes
@@ -311,17 +394,17 @@
   and angular shape independently). This is a stakeholder-set boundary,
   not an oversight — see `docs/protocol-v4.md` §5.2's own "what it is
   not" paragraph.
-- **Tour-embedded turns don't reach the isolated-turn sweep's own
-  optimum.** A `Move` chained via SUC-051's seamless hand-off starts its
-  own ramp from whatever the PRECEDING `Move` left the shaper state at,
-  not a clean from-rest start (118 ticket 004: `MoveQueue::tick()` now
-  resets the just-completed Move's own shaped axis on every completion,
-  not just the empty-queue drain, specifically to bound this — see
-  `move_queue.cpp`'s own comment at that reset call site for why). The
-  land-at-zero completion predicate's own margin constant
-  (`kStoppingMarginFactor`, `move_queue.cpp`) was verified against the
-  TOUR-level metric directly (not just an isolated single turn) — see
-  `test_tour_closure_gate.py`'s own sweep.
+- ~~Tour-embedded turns don't reach the isolated-turn sweep's own
+  optimum~~ — **RESOLVED, moved to §4 Design (119, "Chain-advance leg
+  hand-off contract")**: what the carried/reset shaper state does at a
+  completion boundary is now a specified contract (unconditional
+  completing-axis reset, untouched surviving axis, vExit-equivalent
+  reversal handling, named axis-drop-coast mechanism for the
+  chain-advance margin's own narrow pocket), not an open, tuned-around
+  limitation. The one genuinely still-open piece — the D→RT
+  `reversal_dwell_ms` hardware asymmetry's accept-vs-symmetrize decision
+  — is called out explicitly in that same Design paragraph as this
+  ticket's own decision to make, not left implicit here.
 - **Hardware residual.** A 2026-07-22 hardware bench session (tovez on the
   stand) measured a turn residual in roughly the same `0-8deg` band the
   earlier accel-only stage measured — the real plant's own coast-down
```
