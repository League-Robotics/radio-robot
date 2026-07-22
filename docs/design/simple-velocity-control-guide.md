# Simple Velocity Control

A practical guide for a Micro:bit robot with a 40 ms (25 Hz) control loop.

> **Recommendation:** Use an acceleration-limited velocity command plus modest wheel-speed feedback. Start with `1.0 m/s²` wheel acceleration, `1.2 m/s²` wheel braking, `1.0 m/s²` maximum lateral acceleration, and `4.0 rad/s²` maximum yaw acceleration. Add jerk limiting only if testing exposes a remaining problem.

## What the 40 ms loop changes

A 40 ms loop runs at 25 Hz. That is too slow for aggressive motor-servo behavior, but it is fast enough to shape a small robot's motion when acceleration and braking unfold over several updates. Motor PWM can continue at a much higher carrier frequency while the 25 Hz loop refreshes the command it holds.

Judge a profile by its sample count:

- A 400 ms ramp contains ten updates and is represented well.
- A 40-80 ms ramp contains only one or two updates, so extra sophistication adds little.

## Measured robot and traction data

The robot is a two-wheel differential drive with one caster at the front and one at the back. The measurements used for the initial limits are:

| Measurement | Result |
|---|---:|
| Total robot mass | 0.550 kg |
| Mass supported by driven wheels | 0.450 kg |
| Mass supported by casters | 0.100 kg |
| Longitudinal breakaway force | 3 N |
| Sideways breakaway force | 4 N |

The driven wheels support:

```text
drive-wheel load fraction = 0.450 / 0.550
                          = 0.818
                          = 81.8%
```

The corresponding normal forces are:

```text
total normal force       = 0.550 * 9.81 = 5.40 N
drive-wheel normal force = 0.450 * 9.81 = 4.41 N
caster normal force      = 0.100 * 9.81 = 0.98 N
```

The wheels were prevented from rotating during the pull tests. Treating the measurements as whole-robot breakaway tests gives approximate friction coefficients of:

```text
longitudinal coefficient = 3 / 5.40 = 0.56
lateral coefficient      = 4 / 5.40 = 0.74
```

The equivalent whole-robot sliding accelerations are:

```text
longitudinal slide limit = 3 / 0.550 = 5.45 m/s²
lateral slide limit      = 4 / 0.550 = 7.27 m/s²
```

The casters may contribute some resistance to the pull test, so those figures are not precise driven-wheel traction limits. A useful working estimate applies the measured longitudinal coefficient to the load supported by the driven wheels:

```text
estimated drive traction = 0.56 * 4.41
                         = 2.45 N

estimated acceleration limit = 2.45 / 0.550
                            = 4.46 m/s²
```

This is still far above the proposed operating limits. At the recommended values:

| Limit | Required force | Share of estimated drive traction |
|---|---:|---:|
| `1.0 m/s²` acceleration | 0.55 N | 22% |
| `1.2 m/s²` braking | 0.66 N | 27% |
| `1.0 m/s²` lateral acceleration | 0.55 N | 14% of measured lateral breakaway force |

The measurements therefore suggest that traction is unlikely to be the first limitation. Encoder tracking, motor torque, battery voltage, caster behavior, tipping stability, and the desired feel of the motion are more likely to determine the final settings.

## Final acceleration recommendations

Use these as the initial software limits:

```text
wheel acceleration        = 1.0 m/s²
wheel braking             = 1.2 m/s²
maximum lateral accel     = 1.0 m/s²
maximum yaw acceleration  = 4.0 rad/s²
maximum yaw rate          = 1.5 rad/s
```

At the 40 ms update period, the longitudinal command increments are:

```text
acceleration increment = 1.0 * 0.040 = 0.040 m/s per update
braking increment      = 1.2 * 0.040 = 0.048 m/s per update
```

For example, reaching `0.5 m/s` at `1.0 m/s²` takes:

```text
ramp time = 0.5 / 1.0 = 0.50 seconds
updates   = 0.50 / 0.040 = 12.5 updates
```

That is enough samples for a useful ramp at 25 Hz. If the motion feels too abrupt, reduce wheel acceleration to `0.8 m/s²`. If tracking remains good and a livelier response is wanted, test `1.2 m/s²`; do not increase the production setting solely because the theoretical traction limit is much higher.

## Recommended control stack

```text
target velocity -> acceleration limiter -> velocity PI -> PWM -> motor
```

1. Read or snapshot the encoder counts and calculate measured wheel velocity.
2. Shape the requested velocity with acceleration and braking limits.
3. Use feedforward plus a modest PI correction to make the wheel track the shaped command.
4. Clamp the PWM output and prevent the integral term from winding up while saturated.

## The core velocity shaper

Keep a commanded velocity as state. Each update, move it toward the target by no more than the acceleration limit permits:

```text
dt = measured elapsed time
dv = clamp(v_target - v_cmd, -a_brake * dt, a_accel * dt)
v_cmd = v_cmd + dv
```

This velocity slew-rate limiter:

- Handles forward and reverse commands.
- Allows different acceleration and braking limits.
- Cannot overshoot a fixed velocity target.
- Produces a trapezoidal position move when combined with goal-aware braking.

Use the measured elapsed time rather than assuming every cycle is exactly 40 ms.

## Position moves: brake before the goal

For a position target, do not command maximum speed until the final encoder tick and then brake. Calculate the fastest speed from which the robot can still stop within the remaining distance:

```text
remaining = abs(goal_position - position)
v_stop = sqrt(2 * a_brake * remaining)
v_goal = sign(goal_position - position) * min(v_max, v_stop)

v_cmd += clamp(v_goal - v_cmd, -a_brake * dt, a_accel * dt)
```

This stopping-speed calculation turns the slew limiter into a practical point-to-point motion profile. Leave a small margin for:

- One loop of control latency.
- Encoder quantization.
- Motor deadband.
- Changes in braking performance as battery voltage falls.

## Close the wheel-speed loop

The shaped velocity is a command, not a guarantee. Battery voltage, floor friction, load, and motor mismatch all change the speed produced by a particular PWM value.

A compact controller is:

```text
error = v_cmd - v_measured
pwm = feedforward(v_cmd) + Kp * error + Ki * integral_error
pwm = clamp(pwm, -pwm_max, pwm_max)
```

Practical guidance:

- Use the actual elapsed time in every integration step.
- Average or filter noisy velocity estimates, but avoid so much filtering that braking feedback arrives late.
- Tune feedforward first, then proportional gain, then add only enough integral gain to remove steady bias.
- Stop or constrain the integral term while the output is saturated.

## Optional jerk limiting

Jerk limiting is possible at 25 Hz by keeping acceleration as another state and limiting how much it changes each update:

```text
a_cmd += clamp(a_requested - a_cmd, -j_max * dt, j_max * dt)
v_cmd += a_cmd * dt

samples_in_jerk_ramp = (a_max / j_max) / dt
```

Jerk limiting is useful when the acceleration ramp spans roughly five to ten updates. Below about three updates, it adds little beyond ordinary acceleration limiting.

Landing exactly on a target velocity with zero final acceleration requires starting the acceleration ramp-down early. That is where a complete S-curve planner becomes more useful.

## A sensible tuning order

1. Measure a reliable maximum wheel speed and a feedforward PWM-to-speed mapping.
2. Choose conservative acceleration and braking limits that do not cause slipping or brownouts.
3. Tune the velocity PI loop using constant velocity commands.
4. Add stopping-distance logic and test short and long moves in both directions.
5. Log position, measured velocity, commanded velocity, and PWM.
6. Add jerk limiting only if the logs or the chassis show a remaining problem.

## When a full trajectory generator is justified

A library such as Ruckig earns its complexity when the robot needs:

- Synchronized axes.
- Exact arrival times.
- Specified final velocity or acceleration.
- Repeatedly time-optimal moves.

For an ordinary differential-drive robot, acceleration-limited wheel commands, goal-aware braking, and sound feedback are usually the best first implementation.

## Bottom line

The 40 ms loop does not rule out rigorous control. Start with `1.0 m/s²` wheel acceleration, `1.2 m/s²` braking, `1.0 m/s²` lateral acceleration, and `4.0 rad/s²` yaw acceleration. Use measured timing, goal-aware braking, and a well-behaved wheel-speed loop; add jerk limiting only if testing identifies a real need.
