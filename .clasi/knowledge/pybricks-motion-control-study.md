# Pybricks motion-control study (2026-08-01/02)

Source-level study of Pybricks (`pybricks-micropython`, `lib/pbio/`) — the
LEGO-hub firmware whose motion layer solves the same problems as ours:
coarse encoders, noisy velocity, two-wheel coordination, stall detection.
Everything below was verified against their C sources; file/function names
cited inline. Companion synthesis: how each mechanism maps onto this
project's stack.

## 1. Architecture in one paragraph

A maneuver is solved ONCE at command time into a trapezoidal (optionally
jerk-limited) trajectory (`trajectory.c`), then only EVALUATED each 5 ms
tick (200 Hz). Control is position-PID-on-the-reference: the error signal
is measured angle vs the time-evaluated reference angle — even for speed
commands (since the integral of speed error IS position error, PI-on-speed
falls out with zero accumulation arithmetic). The D-term uses a
model-based speed estimate from a Luenberger observer (`observer.c`), not
differentiated encoder counts. Feedforward torque (friction + back-EMF +
inertia, from a per-motor-type model) is added outside the PID and the sum
is converted to a battery-normalized voltage (PWM duty = commanded_voltage
/ EMA-filtered measured pack voltage). ~13 user-tunable scalars per axis;
everything else derived from the motor model + one `precision_profile`
number.

## 2. The Luenberger observer (`observer.c`)

**The problem it solves**: encoders give coarse position; differentiating
position amplifies quantization noise (short window) or adds lag (long
window). At 1-degree resolution and 5 ms ticks, differentiated speed is
unusable for a D-term.

**The idea**: run a SIMULATION of the motor in parallel with reality. The
model takes the actually-applied voltage each tick and predicts the next
angle/speed/current (third-order linear model, per-motor-type constants
from offline system identification — 17 scaled-integer coefficients).
Then compare the model's predicted angle with the encoder's measured
angle, and feed the error back as a CORRECTION VOLTAGE through a gain:

    feedback_voltage = L(angle_measured - angle_estimated)
    model_input      = applied_voltage + feedback_voltage

The corrected simulation's speed state is the estimate: smooth like a
model, honest like a measurement. "Observer" = a construction that
estimates states you cannot measure (speed, current) from ones you can
(angle). "Luenberger" = the classic fixed-gain version (a Kalman filter
is the same structure with L derived statistically from noise models;
Luenberger just picks L).

**Pybricks refinements**:
- Two-slope correction gain: below 20° of angle error use a LOW gain
  (trust the model — smooth), above it a 7x HIGHER gain (snap back to
  the encoder — bounded divergence). `feedback_gain_low` 30–90 mV/deg
  per motor type.
- Coulomb friction in the model, linearly tapered to zero below
  500 mdeg/s (anti sign-chatter at rest).
- They keep BOTH speeds: user-facing `speed()` is a 100 ms windowed
  differentiation; the control law's D-term uses the observer estimate.
- Observer-side stall detection: if the correction voltage opposes and
  exceeds 75% of the applied voltage (model says "should be moving",
  encoder says "isn't"), sustained past `stall_time` → stalled.

**Our analogy**: it is dead reckoning with continuous gentle landmark
correction — exactly the relationship between our odometry and CAMFIX,
but at the single-motor timescale. Our planner's ZOH predict-to-now is a
prediction WITHOUT the correction structure; our plant model (gain
~1370 mm/s per duty, tau ~0.23 s, from plant ID 2026-07-26) is precisely
the model an observer needs.

## 3. Trajectory + control specifics worth keeping

- Precomputed trapezoid; new commands re-anchor on the CURRENT REFERENCE
  (not measured state), joining tangentially when accelerations match —
  glitch-free chaining (`Stop.NONE`) by construction.
- Integrators PAUSE rather than leak; pausing the integrator also pauses
  the REFERENCE CLOCK, so a stuck motor does not watch its target run
  away. Windup pause condition: P-term torque beyond a margin above max
  actuation, not opposing the reference direction.
- Drivebase = sum/difference virtual axes (distance = (L+R)/2, heading =
  (L−R)/2), each a full trajectory+PID; per-wheel commands exist only at
  torque output. Heading gets 2x torque authority ("doubled to take
  priority") so saturation degrades speed, not heading. Trajectory
  "stretch" makes both axes finish together (arcs = simultaneous
  relative moves, no special arc controller).
- `use_gyro(True)` swaps only the heading MEASUREMENT; the D-term stays
  on motor-observer speed "to guarantee stability".
- Completion: position tolerance AND speed tolerance (both windows), or
  endpoint-crossed for chained moves. COAST_SMART: next relative move
  starts from the previous TARGET (if within 2x tolerance) so chains do
  not accumulate landing error — the same insight as our cumulative
  baseline carry.
- Stall (control-side): integrator saturated + |speed| below limit +
  sustained `stall_time` (defaults 20 deg/s, 200 ms).

## 4. Calibration/config philosophy

Per motor TYPE (offline system ID, zero per-unit calibration): 17 model
constants + 4 reduced numbers (`rated_max_speed`, `feedback_gain_low`,
`precision_profile`, `pid_kp_low_speed_threshold`). Everything else
derived: `kp = nominal_torque/precision_profile` (full authority at one
tolerance-width of error), `ki = kp/2`, `kd = kp/8`. Runtime user surface
is ~13 scalars per axis (limits, pid, target_tolerances,
stall_tolerances). Strong validation of our PlannerLimits reduction and
of deriving gains from the population sweep's model.

## 5. Battery normalization (and our constraint)

Every actuation: `duty = commanded_voltage * MAX_DUTY /
battery_voltage_avg`, with the average EMA-filtered (1/128 per 5 ms tick,
tau ≈ 0.6 s). Model, gains, and thresholds thereby stay supply-invariant.
**We cannot read pack voltage on the Nezha hardware** (stakeholder,
2026-08-02), so there sag is an unobservable slope disturbance — handled
per the wheel-controller issue's supply-sag note (measure magnitude at
two charge states; accept within authority; common-mode slope estimator
parked). NOTE: the HiWonder 4-channel encoder driver EXPOSES battery
voltage at register 0x00 — on that hardware the Pybricks trick becomes
available.

## 6. What we decided to adopt / what validated / what doesn't transfer

ADOPT-CANDIDATES (recorded for the wheels-solid work):
1. Model-based speed observer for the wheel controller's measurement
   stage (attacks velocity noise at the source; more valuable at our low
   sample rates than at their 200 Hz).
2. Position-on-reference control for the fast loop (primary error uses
   clean encoder counts; noisy velocity appears only in the D-term).
3. Sum/difference task-space axes with heading priority — a planner-level
   consideration for later.

VALIDATED (no action, keep as confirmations): cumulative-baseline carry
(= COAST_SMART), stall = integrator-saturated + slow + sustained (= our
deficit flag design), minimal derived config surface, feedforward-owned
friction/dead zone, reference re-anchoring for chaining.

DOES NOT TRANSFER: 200 Hz / 5 ms assumptions (we were ~20 Hz on Nezha;
the HiWonder/coprocessor path changes this); their integer-only decimal
scaling discipline (we use floats); zero per-unit calibration (LEGO
manufacturing tolerance vs our measured 0.91-slope right-wheel residual).

## Sources

pybricks-micropython @ master (fetched 2026-08-01): lib/pbio/src/
{trajectory,control,integrator,observer,servo,drivebase,imu}.c,
control_settings.c, motor/servo_settings.c, include/pbio/*.h,
lib/pbio/src/dcmotor.c + battery.c (duty normalization), and
docs.pybricks.com Motor/DriveBase/Control pages.
