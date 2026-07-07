---
status: done
sprint: 086
tickets:
- '001'
- '002'
- '003'
- '004'
---

# Turn/drive terminal overshoot: motor velocity-loop reverse-spin makes turns backtrack and tours wreck

## Symptom

Driving TestGUI's **Sim Tour 2** renders a chaotic tangle of paths across the
playfield (operator screenshot, 2026-07-06). Manually: a turn "completes the
turn and then backtracks a little bit, wanders around quite a lot after the turn
is finished." This is deterministic in the **sim** (no hardware needed).

## Root cause (instrumented, data-backed — sim / `libfirmware_host`)

**The motor velocity loop overshoots clean through zero into a sustained reverse
spin at the end of every turn (and drive).** Measured per-wheel `vel(L,R)` across
one `RT 9000` (+90°), sampled via `SNAP`, with sim ground-truth heading:

```
during turn:      vel(L,R) = (-43, +43)   true_h 18→88°   mode=T
completion tick:  vel(L,R) = (-43, +43)   true_h=97.4°    mode=I  <- EVT done RT fires, STILL full turn speed
post +0 ms:       vel(L,R) = (+52, -52)   <- wheels REVERSE (magnitude EXCEEDS the -43 it was arresting)
post +100..800:   vel(L,R) = (+50→+39...) true_h 96.5 → 87.2°  <- ~10° backtrack, then settles
```

So:
- The turn **stops accurately** (~90° at `EVT done`) and the **pose estimator is
  correct** (`fused == true` heading exactly). The damage is purely the
  **post-stop physical motor overshoot**, not the stop logic or the estimate.
- When the commanded yaw ramps to zero, the velocity loop drives the spinning
  wheels past zero into a **reverse spin (~+52 for ~800ms)** that back-rotates
  the robot ~4–10° per turn and translates it.
- Drives show the same thing (`D 200 200 500` → true ~535mm, ~7% over): the
  wheel velocity is arrested abruptly at the stop and overshoots.

**Why the existing SMOOTH ramp-down does not prevent it:** `RT`/`TURN`/`D`
default to `StopStyle::SMOOTH` (`planner.h:21`, value 0), and on a stop the
Planner does `ramp_.setTarget(0,0)` (`planner.cpp:~425`). But with
`yaw_acc_max = 20 rad/s²` (`main.cpp:108`) the commanded yaw decelerates from
~1.75 rad/s to 0 in **~87ms** — far too fast for the motor loop, which overshoots
regardless — and there is **no deceleration/coast anticipation**: the wheel is
still at full turn speed when the stop fires (084's own deferred "Open Question 1
/ terminal precision"). `kSoftDeadlineMs = 3000` is not the limiter.

**Why tours become a wreck (Bug "B"):** the ~4–10° per-turn backtrack + ~7%
per-drive overshoot + the reverse-spin translation **compound** over Tour 2's
7 turns + 8 drives → the tour ends **~175mm and ~148° heading off** and flails
across the field. (A settle delay between legs does NOT help — confirmed: the
error is per-leg, not premature `mode=I`.) An earlier probe that reported
"77°/no completion" was a measurement artifact (bad EVT detection + sampling
mid-backtrack); the clean instrumented behavior above is the correct picture.

**Code touchpoints:**
- `source/subsystems/planner.cpp` — SMOOTH stop path (`~385–435`), `stageGoal`,
  the terminal-decel `vCap` that ALREADY exists for the GOTO/POSITION path
  (`~319`) but is NOT applied to `RT`/`TURN`/`D`.
- `source/motion/velocity_ramp.cpp` / `.h` — `yaw_acc_max`, `a_decel`, jerk.
- `source/hal/velocity_pid.cpp` (`Hal::MotorVelocityPid`, 081) + the motor
  zero-crossing dwell / reset-guard armor (078/079) — the loop that overshoots.
- `source/dev_loop.cpp` — after completion the Planner drain is gated off
  (`~201`, `~212`), so nothing actively holds the wheels at zero through the
  overshoot.
- `source/main.cpp:104-108` — `a_max=800`, `a_decel=800`, `yaw_rate_max=6`,
  `yaw_acc_max=20`.

## Fix plan (ordered, per stakeholder decision 2026-07-06)

**Phase 1 — fix the motor velocity loop FIRST (the root).** Make decelerating a
spinning wheel to zero NOT overshoot into a reverse spin: anti-windup / retune
`Hal::MotorVelocityPid`, and/or adjust the 078/081 zero-crossing dwell /
reset-guard so it does not inject a reverse pulse at the crossing. **Re-verify
the wedge/reversal safety armor (078/079) still holds** — do not weaken the
protections; this is why it is its own phase.

**Phase 2 — THEN add terminal decel/coast anticipation to `RT`/`TURN`/`D`.**
Un-defer 084 Open-Question-1: start decelerating BEFORE the target so the wheel
speed is near-zero when the stop fires (little momentum to arrest, minimal
residual overshoot). Reuse the decel-cap pattern already present for GOTO
(`planner.cpp:~319`, `vCap = sqrt(2·a_decel·dRemaining)`), extended to the
distance/rotation/heading stops. Port concept: `source_old`'s deferred
`SAFETY_MARGIN`/`ARRIVE` and the sprint-073 sim turn-accuracy work.

## Verification bar (MANDATED — this is how it shipped broken)

- **Per-leg geometry vs sim ground truth:** assert each leg's ACTUAL heading /
  position change (from `sim_get_true_pose_*`) vs commanded, within a tight
  tolerance, AND that the wheels are settled (no reverse-spin residual) at
  completion. Endpoint-distance-only tour tests are **banned**.
- **Rendered-tour check:** capture a Tour 1/2 trace image a human can eyeball —
  it must look like the intended figure, not a tangle.
- Deferred (per stakeholder, for now): a real-playfield camera HITL gate —
  revisit once the sim geometry is trustworthy.

**Process lesson:** sprints 084/085 asserted only tour ENDPOINT DISTANCE
(~175mm, mislabeled "near origin"); never heading error, trajectory shape, or a
rendered tour. That masked a plainly-broken behavior. The bar above prevents
recurrence.

## Scope / dependencies

Firmware (motor loop + Planner motion). Blocks trustworthy autonomous motion in
the whole TestGUI-revival program (082–085) — driving/traces work, but tours,
GOTO, and any multi-leg motion are visibly wrong until this lands. **Supersedes**
the earlier thin placeholder `motion-terminal-precision-decel-anticipation.md`
(this issue is the authoritative, root-caused writeup). Related motor-armor
context: sprints 078/079/081.
