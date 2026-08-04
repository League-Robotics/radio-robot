---
status: pending
---

# Build a position-driven online trajectory manager after wheel positions are available

## Description

Once reliable per-wheel positions are available, define how Planner Moves generate and manage trajectories before adding more estimator or controller complexity.

Do not begin with a precomputed wall-clock trajectory of the form

$$
(p_{\mathrm{ref}}, v_{\mathrm{ref}}, a_{\mathrm{ref}}) = f(t)
$$

that the plant must chase on schedule. Instead, separate each Move into immutable geometry, online measured-progress generation, accumulated shape correction, and measured completion. If the robot is slowed by load, the Move should naturally take longer rather than accumulate schedule error or distort curvature.

This issue operationalizes the design in [[replace-independent-wheel-position-schedules-with-progress-and-constant-ratio-shape-control]].

## Recommended architecture

A Move has four responsibilities:

1. **Geometry:** fixed wheel ratio and endpoint.
2. **Progress generation:** speed to command now, based on measured remaining travel and stopping limits.
3. **Shape correction:** accumulated wheel-ratio correction preserving curvature.
4. **Completion:** measured endpoint and rest, independent of schedule.

```text
Move request
    |
    v
Immutable Move geometry
  qLeft, qRight, endpoint, limits
    |
    v
Online scalar progress generator <--- measured remaining progress
  cruise / accel / decel / optional jerk
    |
    +----> bounded shape correction <--- measured wheel positions
    |
    v
left/right wheel-speed targets
    |
    v
existing per-wheel actuator controllers
```

## 1. Immutable Move geometry

At Move activation, calculate a dimensionless wheel-direction vector:

$$
\mathbf q = [q_L, q_R]
$$

normalized so:

$$
\max(|q_L|, |q_R|) = 1
$$

Calculate the required final wheel displacements:

$$
D_L = q_L D
$$

$$
D_R = q_R D
$$

where $D$ is the dominant-magnitude wheel's required travel. Because one component of $\mathbf q$ has magnitude 1, this representation avoids projection, square roots, and runtime division.

Examples:

- straight 500 mm: $\mathbf q=[1,1]$, $D=500$ mm;
- symmetric pivot: $\mathbf q=[-1,1]$, with $D$ equal to either wheel's required arc;
- asymmetric arc: $\mathbf q=[0.5,1]$, with endpoints $0.5D$ and $D$;
- stationary inner wheel: $\mathbf q=[0,1]$.

Latch at activation:

- raw left/right encoder baselines;
- $\mathbf q$ and $D_L,D_R$;
- the Move's natural progress coordinate and endpoint;
- cruise, acceleration, deceleration, jerk, correction, and timeout limits.

Geometry remains immutable for the Move. Wall time does not alter it.

## 2. Online scalar progress generation

Maintain only scalar progress state:

- current progress speed $\lambda$;
- current progress acceleration if jerk limiting is enabled.

Nominal wheel targets are:

$$
v_{L,\mathrm{base}} = q_L \lambda
$$

$$
v_{R,\mathrm{base}} = q_R \lambda
$$

Each cycle, calculate remaining progress from measured wheel positions using the Move's natural coordinate. For the dominant-magnitude wheel:

$$
d_{\mathrm{remaining}}
=
D - \operatorname{sign}(q_d)\Delta p_d
$$

Calculate the stopping envelope:

$$
\lambda_{\mathrm{stop}}
=
\sqrt{2 a_{\mathrm{decel}}\max(0,d_{\mathrm{remaining}})}
$$

Choose:

$$
\lambda_{\mathrm{target}}
=
\min(\lambda_{\mathrm{cruise}},\lambda_{\mathrm{stop}})
$$

and slew $\lambda$ toward it.

- Acceleration-limited slew generates an online trapezoidal/triangular profile.
- Slew-limiting acceleration generates an online jerk-limited profile.

Measured progress decides when braking begins. If load slows the robot, the remaining distance decreases more slowly, braking begins later in wall time, and the Move takes longer automatically. No reference clock needs pausing.

Use each Move's natural progress coordinate where already available:

- distance Move: center-distance progress;
- angle Move/pivot: angular progress or equivalent wheel arc;
- timed Move/WHEELS: elapsed wall time, treated separately because it intentionally has a time budget rather than a measured endpoint.

## 3. Position-based shape correction

Using Move-relative wheel travel:

$$
\Delta p_L = p_L-p_{L,0}
$$

$$
\Delta p_R = p_R-p_{R,0}
$$

calculate:

$$
e_{\mathrm{shape}} = q_L\Delta p_R-q_R\Delta p_L
$$

The target is zero. The nominal progress command $\lambda\mathbf q$ lies along the requested wheel ratio and cannot itself correct this perpendicular error.

Use the perpendicular direction:

$$
\mathbf q_{\perp}=[-q_R,q_L]
$$

and bounded correction:

$$
\mathbf v_{\mathrm{shape}}
=
-K_{\mathrm{shape}}e_{\mathrm{shape}}\mathbf q_{\perp}
$$

which produces:

$$
v_L
=
q_L\lambda + K_{\mathrm{shape}}e_{\mathrm{shape}}q_R
$$

$$
v_R
=
q_R\lambda - K_{\mathrm{shape}}e_{\mathrm{shape}}q_L
$$

Clamp the shape contribution to explicit authority. Confirm signs with unit tests for every shape quadrant and reverse motion.

This wheel-position residual is the one accumulated-error mechanism for planner geometry. Do not also run independent left/right wall-clock position integrals.

## 4. Geometry-first feasibility and slowdown

After adding shape correction, check the resulting targets against:

- per-wheel speed ceilings;
- per-cycle acceleration/deceleration limits;
- shape-correction authority;
- actual actuator deficit/saturation information when available.

If shape correction remains clamped and $|e_{\mathrm{shape}}|$ continues worsening, reduce $\lambda$. Restore it gradually after recovery.

Policy:

> Preserve geometry first; sacrifice completion time second.

Use hysteresis and a sustained-condition window. A single quantized encoder step must not switch slowdown state. Continue safety timeout on wall time while progress slows.

## 5. Measured completion

A Move does not complete merely because $\lambda$ reaches zero. Require all applicable conditions:

1. The natural Move coordinate reached its endpoint:
   - center distance for a distance Move;
   - angle for a pivot/angle Move.
2. Shape error is within tolerance.
3. Both wheels are within rest-speed tolerance.
4. Conditions remain true for a short dwell.
5. Wall-time timeout remains an independent safety backstop.

This prevents the dominant wheel from completing while the other remains behind. Completion tolerances must be derived from loaded actuation/measurement behavior rather than chosen below achievable resolution.

## 6. Initial chaining policy

For the first implementation:

- stop at every Move endpoint;
- reset wheel-position baselines at activation;
- prove straight, pivot, reverse, stationary-inner-wheel, and arc behavior;
- prove asymmetric load increases duration instead of changing curvature.

Add nonzero exit speeds and chaining only afterward.

For later chaining:

- preserve nonzero progress only when adjacent Moves have compatible $\mathbf q$;
- otherwise decelerate to zero before changing ratio;
- hand off from the current reference/progress state, never from a noisy differentiated measurement;
- retain exact endpoint accounting so compatible chains do not accumulate landing error.

Do not solve geometry transitions, flying handoff, and the first shape controller in the same implementation step.

## 7. Relationship to existing layers

### Retain

- `Types::RobotState::Wheel::cmdVelocity` as the motion-to-base boundary;
- `App::Drive::tick()` as the single actuation path;
- two per-wheel calibrated feedforward/proportional speed controllers;
- freshness, exact-zero, duty quantization, slew, deadband, reversal dwell, applied-duty, and safety behavior;
- existing online jerk-limited shaping where it fits the scalar $\lambda$ generator.

### Replace or avoid for Planner Moves

- independent left/right wall-clock position references;
- precomputed wall-time position schedules requiring reference-clock pausing;
- duplicate ratio-control mechanisms;
- completion based only on planned time or generated speed reaching zero.

### Defer

- Luenberger velocity observer, unless measured velocity noise blocks necessary damping or stop detection;
- fused-pose/OTOS path correction, until heading truth is reliable;
- compatible nonzero-speed chaining;
- variable-curvature geometry within one Move.

## Implementation sequence after wheel positions

1. **Immutable geometry:** $\mathbf q$, per-wheel endpoints, baselines, natural progress coordinate.
2. **Acceleration-limited online stopping-envelope generator.** Prove measured-progress braking before adding jerk complexity.
3. **Position-based shape controller:** cross-product residual and bounded perpendicular correction.
4. **Geometry-first slowdown:** reduce progress under sustained exhausted shape authority.
5. **Measured completion and settling:** endpoint, shape, rest, dwell, timeout.
6. **Jerk limiting:** retain/add only if acceleration steps cause measured problems.
7. **Compatible Move chaining and nonzero exit speeds.**
8. **Luenberger observer:** only if velocity damping/reporting remains noise-limited.

## Observability

Record per control cycle:

- Move ID and lifecycle phase;
- $q_L,q_R$ and wheel baselines/endpoints;
- measured left/right relative positions;
- natural progress and remaining progress;
- $\lambda$, target $\lambda$, acceleration, and stopping envelope;
- shape error and bounded correction;
- final left/right velocity targets;
- measured wheel velocities and freshness;
- requested and applied duties;
- slowdown/saturation reason;
- completion predicates and timeout age.

The controller must be replayable from logged inputs; no hidden clock reads or implicit static state.

## Verification

### Pure math and unit tests

- Geometry/endpoints are correct for straight, pivot, asymmetric arc, reverse, and stationary-inner-wheel Moves.
- Perfect ratio travel produces zero shape error at every progress value.
- Perpendicular correction signs reduce shape error in every direction/quadrant.
- No measured-distance division occurs and zero initial travel is well-defined.
- Stopping envelope begins braking at the correct measured remaining distance.
- Acceleration-limited output is trapezoidal/triangular as appropriate.
- Optional jerk-limited output respects jerk, acceleration, velocity, and endpoint constraints.
- Timeout uses wall time independently of slowed progress.

### Deterministic plant tests

- A nominal plant reaches each endpoint with shape error and rest state inside tolerance.
- One-wheel gain/lag derating increases duration while preserving ratio.
- Persistent derating activates bounded slowdown without chatter.
- Removing derating restores progress gradually.
- Quantization, deadband, slew, reversal dwell, stale samples, and one-cycle timing variation do not violate safety or exact-zero behavior.
- Pivot completion does not depend on center distance.
- Incompatible chained geometry stops before changing ratio; compatible chaining is deferred until separately enabled.

### Hardware acceptance

- Compare current behavior against the online trajectory manager using matched straight, pivot, and arc Moves.
- Record endpoint error, shape residual, transient heading, duration, saturation, settling, and chatter.
- Apply repeatable asymmetric load and confirm path shape is preserved while duration expands.
- Verify braking and completion at multiple speeds and more than one battery condition.
- Do not enable by default until diagnostics explain every slowdown and completion decision.

## Decision requested

Approve a **position-driven online trajectory manager**: immutable constant-ratio geometry, one measured-progress scalar generator, one accumulated shape-error controller, geometry-first slowdown, and measured completion. Do not make a precomputed wall-clock trajectory or a Luenberger observer prerequisites for the first implementation.
