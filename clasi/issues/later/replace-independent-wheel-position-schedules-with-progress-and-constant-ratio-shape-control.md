---
status: pending
---

# Replace independent wheel-position schedules with progress and constant-ratio shape control

## Description

The wheel controllers improved substantially after replacing numerically integrated velocity error with position error. The next design question is what position relationship to control. Pybricks follows a time-indexed trajectory and pauses its reference clock when saturated. Radio Robot need not adopt that schedule: Planner Moves are constant-curvature, constant-wheel-ratio maneuvers, and finishing later is preferable to distorting the path.

Generate progress online from measured remaining distance, stopping limits, and available authority. Control accumulated wheel-ratio error to preserve geometry. There is then no wall-clock position trajectory to pause.

## Recommendation

Use one outer **progress-and-shape coordinator** above the existing two per-wheel actuator controllers:

```text
Move geometry + measured remaining progress
        |
        v
Progress-and-shape coordinator
  - online progress speed and stopping envelope
  - accumulated constant-ratio shape correction
  - progress slowdown when shape authority is exhausted
        |
        v
left/right wheel-speed targets
        |
        v
Two per-wheel actuator controllers
  - calibrated feedforward
  - proportional speed feedback/damping
  - no independent wall-clock position integrals
        |
        v
left/right duty, shaping, and motor writes
```

Keep two physical wheel controllers because each motor needs its own calibration, immediate speed correction, freshness handling, and limits. Do not layer the coordinator over two independent wheel-position schedules; those accumulated-error loops would fight.

## Shape coordinate

For each constant-ratio Move, define a dimensionless wheel-direction vector:

$$
\mathbf q =
\begin{bmatrix}
q_L \\
q_R
\end{bmatrix},
\qquad
\max(|q_L|, |q_R|) = 1
$$

Examples:

- straight: $\mathbf q = [1, 1]$
- pivot: $\mathbf q = [-1, 1]$
- half-speed inner wheel: $\mathbf q = [0.5, 1]$
- stationary inner wheel: $\mathbf q = [0, 1]$

Latch encoder baselines at Move activation:

$$
\Delta p_L = p_L - p_{L,0},
\qquad
\Delta p_R = p_R - p_{R,0}
$$

Control the unnormalized cross-product residual:

$$
e_{\mathrm{shape}} = q_L \Delta p_R - q_R \Delta p_L
$$

Its target is zero. It is zero exactly when measured wheel travel has the requested ratio. Do not divide measured wheel distances: the ratio is undefined and noisy near move start. Do not normalize by $\sqrt{q_L^2 + q_R^2}$ at runtime; that Move-constant scale can be absorbed into the gain. Max-norm normalization keeps gain meaning reasonably consistent without a square root.

This residual is already accumulated ratio-rate error:

$$
e_{\mathrm{shape}}
=
\int (q_L v_R - q_R v_L)\,dt
$$

No separately summed integral is needed. Generate a bounded correction:

$$
\Delta v_{\mathrm{shape}}
=
\operatorname{clamp}
\left(
K_{\mathrm{shape}} e_{\mathrm{shape}},
-v_{\mathrm{shape,max}},
v_{\mathrm{shape,max}}
\right)
$$

Map progress speed and shape correction symmetrically back to wheel targets with signs that drive $e_{\mathrm{shape}}$ toward zero. Do not permanently designate either physical wheel as master.

Optional damping may later use:

$$
e_{\mathrm{shape-rate}} = q_L \hat v_R - q_R \hat v_L
$$

Add it only if bench evidence shows position-only shape control needs damping. A Luenberger observer may eventually provide $\hat v$, but is not required for the first ratio controller.

## Progress coordinate

Generate progress online rather than from a wall-clock position schedule:

1. Determine requested cruise speed.
2. Measure remaining distance or angle using the Move's natural stop coordinate.
3. Compute the stopping-speed envelope.
4. Slew progress under acceleration/deceleration and optional jerk limits.
5. Let elapsed wall time expand if the plant is slow.

For translational progress:

$$
v_{\mathrm{stop}}
=
\sqrt{2 a_{\mathrm{decel}} d_{\mathrm{remaining}}}
$$

$$
v_{\mathrm{progress,target}}
=
\min(v_{\mathrm{cruise}}, v_{\mathrm{stop}})
$$

Retain the existing jerk-limited online shaper if useful. A precomputed trapezoidal trajectory is not required.

There is no need for a generic projected $s_{\mathrm{path}}$. Reuse:

- distance Move: center-distance progress;
- angle Move/pivot: angular progress;
- timed `WHEELS`: elapsed wall time, bypassing planner ratio coordination unless ratio preservation is explicitly part of its contract.

## Geometry-first saturation

Shape correction has priority over progress. If the correction remains clamped while $|e_{\mathrm{shape}}|$ grows, reduce progress until both wheels can maintain the ratio:

```text
if shape correction is saturated and shape error is worsening:
    reduce progress speed
else if shape error has recovered:
    restore progress speed gradually
```

This is the online counterpart to Pybricks reference-clock pausing: preserve geometry, sacrifice speed, and let the Move take longer.

Use hysteresis and a sustained-condition window to avoid quantization chatter. Base exhausted authority on actual actuator truth where possible: applied duty, clamp, slew/dwell state, and per-wheel deficit, not only an intermediate velocity clamp.

## Relationship to current wheel controllers

Retain per wheel:

- calibrated speed-to-duty feedforward;
- small-authority proportional speed feedback;
- acceleration feedforward if bench-proven;
- freshness/connected/frozen gates;
- exact-zero behavior;
- duty quantization, slew, deadband, reversal dwell, and applied-duty telemetry;
- safety and deficit reporting.

Remove or disable for planner Moves:

- independent left/right wall-clock position references;
- independent position-error terms forcing separate schedules;
- any second ratio mechanism competing with $e_{\mathrm{shape}}$.

The shape residual is the one accumulated-error mechanism preserving the curve. Progress deliberately has no integral forcing elapsed-time position: if the plant is slow, the Move should take longer.

Raw `WHEELS` commands may still request arbitrary wheel speeds and bypass planner coordination while using the retained inner controllers.

## Why these variables

- **Versus independent wheels:** separates progress error from curvature error and prevents saturation from silently changing the path.
- **Versus distance/heading:** works uniformly for straights, arcs, pivots, reverse motion, and a stationary inner wheel; center distance is zero throughout a pivot.
- **Versus measured ratios:** obtains the integration benefit without division by near-zero travel.
- **Versus fused-pose path error:** raw encoder coordination is independently testable now; fused-pose correction can be added later once heading/pose truth is reliable.

## Implementation constraints

- Keep `Types::RobotState::Wheel::cmdVelocity` and the single `App::Drive::tick()` actuation path.
- Put coordination in `Motion::Planner`, not the hardware base.
- Reset/latch baselines on activation, replacement, cancellation, and estop.
- On encoder freshness loss, freeze accumulated correction and fail safely.
- Use measured cycle duration in shaping and damping.
- Continue safety timeout on wall time while progress slows.
- Expose shape error, correction, progress command, slowdown reason, and applied duties in replayable diagnostics.
- Avoid runtime square roots and measured-distance division in shape control.
- Keep observer evaluation separately measurable.

## Verification

### Unit and deterministic simulation

- Perfect-ratio straight, pivot, arc, reverse, and stationary-inner-wheel travel produces zero shape error.
- A lagging wheel produces correctly signed correction.
- Zero/near-zero travel causes no division or instability.
- Correction is bounded and cannot unexpectedly reverse a valid command.
- Derating one wheel slows progress while preserving wheel ratio.
- Removing derating restores progress without chatter or a command step.
- Stopping lands from measured remaining progress without a wall-clock position schedule.
- Pivot progress/completion does not depend on center distance.
- Exact-zero, replacement, estop, stale-sample, and timeout remain safe.

### Hardware

- Compare the current independent position-error controller with this design on identical straight, pivot, and arc cases.
- Record wheel positions, velocities, targets, applied duties, shape error, correction clamp, progress slowdown, and completion each cycle.
- Apply repeatable asymmetric load/derating and confirm duration increases instead of curvature changing.
- Score endpoint distance/angle, ratio residual, transient heading, saturation, settling, and chatter.
- Verify across delivered-period variation and multiple battery conditions before enabling by default.

## Decision requested

Approve **online progress plus unnormalized constant-ratio shape error** as the planner's two outer control variables, feeding two retained per-wheel speed/actuator controllers. Treat absolute heading/path correction and the Luenberger observer as later, separately justified layers.
