---
status: pending
priority: high
---

# TOUR_2's 146-degree turn still undershoots ~10 deg after the 130-010 completion fix

## Description

Residual from sprint 130 ticket 010, which fixed the general turn-shaping
undershoot but did not fully close this case.

After the fix, TOUR_1's worst per-turn error is **2.72 deg** (was up to −20.8) and
`square.tour` passes its 80 mm CAMFIX gate at **41.7 / 43.0 mm** (was ~361 mm).
But TOUR_2's **146 deg** turn still misses by **−10.12 deg**, and
`src/tests/testgui/test_gui_button_acceptance.py::test_tour_2_runs_to_completion`
fails nondeterministically as a result.

Every other turn in both tours is in band. The surviving failure is specific to
the large-angle case.

## Cause

Ticket 010's root cause was that `Motion::Planner::tick()`'s Angle
`profile-complete` event could report a Move done while the commanded ramp
(`profileVelocity_`) was still ~30 deg/s: it tested only
`measured.plannedRemaining`, a lookahead sized to `actuationDelay`, with no check
that the plant had neared rest. `TestSim::WheelPlant`'s first-order lag
(tau = 0.13 s) outlives that window. In isolation the move coasts the rest of the
way; in a **chained** tour the next Move overrides the still-turning wheels on the
same tick, so the shortfall never recovers.

The fix gated that branch on `profileVelocity_` also reaching the rest floor
`settleReached()` uses. That closed the 90 deg case. Why 146 deg still misses by
roughly the same ~10 deg is not established — a residual that does NOT scale with
angle suggests a second, additive mechanism rather than more of the same one.

### Leading hypothesis (2026-08-02 review, F10) — `decelLatched` is a one-way trap

The review proposes a specific, angle-independent mechanism that matches this
residual's signature. Once any tick's `profileStep()` returns Decel/Closing,
`active_.decelLatched` clamps every later λ to min and **never lets it rise**
for the Move's remainder (`planner.cpp:1232-1240`). But the closing step
triggers off `plannedRemaining` — a *prediction* over sample-age +
actuationDelay (`planner.cpp:756-824`). One transient under-estimate drives
the command to ~0, the latch forbids recovery (directly contradicting
`profile.cpp:102-104`'s own "let re-measurement recover" comment), and the
0.5 s stall backstop completes the Move wherever it parked, `settled=false`.

That is **additive and angle-independent** — which is exactly why the residual
does not scale with angle. Nothing currently exercises a transient
misprediction against the latch, and `planner_tests` has no Angle scenario
above 90 deg at all.

**Fingerprint to look for on the single-step harness:** a stall-event
completion, with no timeout, on that turn.

**Proposed fix if confirmed:** release the latch when measured remaining rises
materially above the closing envelope — i.e. make re-measurement recover, as
the comment already promises — and add the missing scenarios (large angles,
tour-shaped chains under `NoisyPlant` lag).

Also worth checking:
- whether the large angle crosses a shaper phase boundary the 90 deg case does not
  (jerk-limited segment never reaching constant-velocity cruise, or a decel ramp
  that starts before the accel ramp finishes);
- whether `decelPlanFraction` (`PlannerLimits::landing`) interacts badly at larger
  angles;
- whether the rotation-calibration correction applied in
  `App::RobotLoop::handleMove()` is angle-dependent in a way the completion fix
  now exposes.

The same ticket also identified why tour numbers looked nondeterministic: tests
using a background tick thread (`SimLoop.connect(start_tick_thread=True)` — the
systest runner and `test_gui_button_acceptance.py`) are not bit-reproducible,
while the single-step harness (`test_tour_closure_gate.py`) is. Two harnesses were
being compared as though they were one. **Measure this issue on the single-step
harness**, or the ~30 mm of apparent spread will drown a 10 deg signal.

## Proposed fix

Root-cause the large-angle case rather than widening a tolerance. Ticket 010
rejected three alternative fixes (recorded in
`sim-tour-turn-shaping-undershoots-90-degree-turns.md`) — read those before
proposing a fourth, and do not add a rotation fudge constant:
`tovez_nocal.json` already carried rotation constants fitted to a sim artifact
(1.006 / +12.1 deg) that were injecting ~12.5 deg of under-rotation into every
turn. Another correction layer is how that happened the first time.

## Verification

- TOUR_2's 146 deg turn lands within the same band as its 90 deg turns.
- `test_tour_2_runs_to_completion` passes repeatedly, not once.
- No regression: TOUR_1 worst per-turn error <= 2.72 deg, `square.tour` <= 80 mm
  (41.7/43.0 today), `circle.tour` 9.6 mm PASS.
- `src/tests/sim` stays at its baseline (458 passed, 2 known pre-existing
  failures: `test_clock_sync_activation.py`, `test_fake_transport.py`).

## Related

- Residual of sprint 130 ticket 010; the mechanism and the three rejected
  alternatives are in
  [[sim-tour-turn-shaping-undershoots-90-degree-turns]].
