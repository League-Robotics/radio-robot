# XRP (XRPLib MicroPython) motion-control study (2026-08-12)

Source-level study of [XRPLib](https://github.com/Open-STEM/XRP_MicroPython),
the Open-STEM Experiential Robotics Platform library — the education-robot
stack most often held up next to ours. Read at `main` @ `23786b5deb00`
(2026-08-03), all 13 modules of `XRPLib/`, ~2,000 lines.

Companion study, and the far more useful one:
[`pybricks-motion-control-study.md`](pybricks-motion-control-study.md).
**Read that one first.** XRP sits *below* Pybricks on every axis this
project cares about, and below our own `Control::DifferentialDrive`. The
value of this document is mostly in saying so precisely, so the question
does not get re-opened from scratch.

## 0. The one-line answer

**Nothing in XRPLib should be imported at any layer.** Three small design
ideas are worth stealing (§7). Everything below the drive controller
assumes a plant we do not have.

## 1. Language and layering — it is all Python

There is **no C++ in the project**. The only non-Python code is the
quadrature decoder, and it is not C either: it is **RP2040 PIO assembly**,
written inline as a MicroPython decorator (`XRPLib/encoder.py:99-154`) — a
16-entry jump table indexed by (previous 2 pin bits, current 2 pin bits),
with the count held in the state machine's X register. Four PIO state
machines means a hard limit of four encoders.

Everything else is interpreted Python on MicroPython. The C in the picture
is only MicroPython's own `machine.I2C` / `machine.PWM` / `machine.Timer`
primitives.

| XRPLib module | Responsibility | Our nearest layer |
|---|---|---|
| `motor.py` | `set_effort(-1..1)` → `duty_u16` on an H-bridge, 50 Hz PWM | `platform/` + `hardware/nezha/` |
| `encoder.py` | PIO quadrature decode; 4 SMs = 4 encoders max | `hardware/nezha/` |
| `encoded_motor.py` | velocity PID on a 50 Hz virtual `Timer` | `Control::DifferentialDrive::fastPid()` |
| `pid.py` / `controller.py` | generic PID + injectable `Controller` interface | — |
| `differential_drive.py` | `straight()` / `turn()` — blocking closed-loop moves | `Motion::Planner` / `Navigator` |
| `imu.py` | LSM6DSO register driver + gyro integration at 208 Hz | `hardware/generic` OTOS + `Motion::Odometry` |
| `board.py` | battery ADC, button, LEDs | — |

Two board variants are handled by `sys.implementation._machine` string
sniffing throughout (`"Beta"`, `"NanoXRP"`), including inline gain tables.

## 2. Motor control — two disjoint paths that can fight each other

**Path 1, velocity control.** `EncodedMotor.__init__` starts a *virtual*
timer at 50 Hz (`encoded_motor.py:93-95`; `Timer(-1)`, deliberately leaving
the hardware timers for the student). Each tick (`_update`, `:198-208`):

```python
current_position = self.get_position_counts()
self.speed = current_position - self.prev_position   # raw delta per 20 ms
error  = self.target_speed - self.speed
effort = self.speedController.update(error)
self._motor.set_effort(effort)
```

Speed is an **unfiltered count delta per 20 ms tick**. No feedforward at
all — the integrator finds the operating point from scratch every time.
`kd = 0` on both board variants (`:74-86`), because differentiating that
delta is pure noise. They dodged the velocity-noise problem by declining
to use a D term; Pybricks solves it with a Luenberger observer.

**Path 2, `straight()` / `turn()`.** These do **not** use the velocity
loop. Both funnel into one `_move()` (`differential_drive.py:217-317`) — a
blocking `while True` at ~100 Hz (`time.sleep(0.01)`) running two PIDs and
writing **raw effort**:

```python
translation = distance_controller.update(distance_error)
rotation    = heading_controller.update(heading_error)
left  = translation - rotation
right = translation + rotation
```

then normalize down if over `max_effort`, boost up if under `min_effort`
and still correcting, scale by `voltage_scale`, `set_effort()`, sleep.
Exit when both controllers report `is_done()`, or on timeout. Returns
`True` iff it finished before the timeout.

**The defect:** both paths write `_motor.set_effort()` and nothing
arbitrates. `_move()` never disarms the 50 Hz timer on entry — it only
calls `stop()` on exit (`:315`), and `stop()` is what sets `target_speed =
None`. A program that calls `set_speed()` and then `straight()` has the
velocity PID writing effort underneath the move loop for the whole move.
This is exactly the class of bug our `Core::RobotLoop` motion-ownership
arbitration exists to prevent (cf. the Navigator sawtooth,
`zeroUnownedMotion`).

## 3. Planning — there is none

No trajectory generation, no trapezoid, no S-curve, no jerk limit, no
queue, no lookahead, no `(x, y)` pose estimate. Deceleration is emergent
from the P term shrinking as error closes. Composition is "call
`straight()`, then call `turn()`" in the student's script, each blocking
until done.

Completion is `PID.is_done()` (`pid.py:117-122`): error within `tolerance`
for `tolerance_count` **consecutive** updates — 10 consecutive at 100 Hz
for the drive controllers.

The contrast with Pybricks is the whole story: Pybricks solves a
trapezoid ONCE at command time and only *evaluates* it each 5 ms tick,
with a model-based speed observer feeding the D term. XRP does none of
that, and does not need to — a blocking P+D-to-zero-error move is a fine
answer for a classroom robot driving a metre.

## 4. IMU — Python, gyro-only, dead-reckoned

`imu.py` is a hand-written LSM6DSO register driver over I2C at 400 kHz,
with register bit fields laid over `bytearray`s via `uctypes.struct`
(`:48-53`) — a genuinely tidy trick.

A virtual `Timer` at the gyro ODR (208 Hz default) burst-reads 12 bytes
and **rectangularly integrates the gyro rates** into `running_pitch` /
`running_roll` / `running_yaw`, with `disable_irq()` around the accumulate
(`:559-577`). That is the entire attitude estimate: no accelerometer
fusion, no magnetometer, no drift correction, no covariance. `get_yaw()`
returns the unbounded integral; `get_heading()` returns it mod 360.

Bias comes from `calibrate()` (`:506-551`): average ~1 s of samples at
rest, subtract, and also derive accelerometer offsets assuming one axis is
vertical and reads 1000 mg. `get_default_imu()` calls it once at first
construction.

The drivetrain uses the IMU three ways:

- `turn()` takes `imu.get_yaw()` as primary feedback, falling back to
  encoder-differential heading `((right - left)/2) * 360/(track*pi)` when
  `use_imu=False` (`differential_drive.py:283-286`).
- `straight()` always passes `use_imu=True`, so the heading PID holds the
  yaw captured at move start — a gyro-based straight-line hold.
- `arcade()` holds heading during teleop whenever the turn axis is zero,
  re-latching the reference on the turn→straight transition
  (`:167-192`).

**The trap we already know.** A ~1 s bias calibration at boot, with **no
stillness check and no way to detect that it was poisoned**. This is
precisely the OTOS failure documented in
[`.claude/rules/hardware-bench-testing.md`](../../.claude/rules/hardware-bench-testing.md)
("The robot must be STILL when it boots") — measured on `tovez`
2026-08-08 as `+1.44 deg/s` standstill drift after booting mid-battery-
swap, versus `-0.006 deg/s` after one still reboot. XRP has the identical
hole without the diagnosis. Do not treat their calibration routine as a
model.

## 5. Why the bottom layers cannot transfer

Not because the code is bad. Because of a structural assumption:

> **XRPLib assumes the drivetrain is honest** — a plain brushed DC motor
> on a PWM H-bridge, with a quadrature encoder the MCU itself owns and
> decodes.

It has no armor because it does not need any. Every hard problem in our
stack exists because **we do not own the motor**:

- the wedge/latch family and the encoder boundary-latch behaviour,
- the Nezha brick physically latching its last commanded speed, and not
  clearing it on an nRF52 reset (936 mm of continued travel from one lost
  zero write, measured 2026-08-03),
- the external I2C bus hang when pull-ups are absent,
- `wheelFrozen` / deficit / stall as three distinct conditions.

None of those are expressible in XRPLib's model, so none of its lower
layers carry information we can use.

## 6. Layer-by-layer verdict

| Their layer | Our layer | Verdict |
|---|---|---|
| `motor.py` | `platform/`, `hardware/nezha/` | **Useless.** We have no duty cycle at that layer; the Nezha takes a speed command over its F9-framed UART. `brake()` vs `coast()` does not exist for us. 50 Hz PWM is not a number to copy. |
| `encoder.py` | `hardware/nezha/` | **Useless.** RP2040-specific. We read position off the brick. (`get_position_counts()` calls `sm.get()` five times to flush the FIFO — a stale-value hack, `:79-84`.) |
| `encoded_motor.py` | `Control::DifferentialDrive` | **Strictly weaker than what we have.** Ours already carries `dutyPerSpeed` feedforward, PI-on-position-error, accel feedforward (`kaff`), Stage C bias adaptation, and deficit/stall latching. Theirs is P+I on an unfiltered count delta. |
| `pid.py` | — | Nothing we lack, except `max_derivative` (an output slew clamp) and `tolerance_count` (§7). |
| `imu.py` | OTOS driver + `Motion::Odometry` | **Below us.** The OTOS fuses in the sensor; theirs is bare gyro integration. |
| `differential_drive._move()` | `Motion::Planner` / `Navigator` | **The only layer with something to teach us** — and it teaches a pattern, not code (§7). |

## 7. What is worth taking

**7.1 — Gate the speed/effort floor on "still correcting."** Ours
(`src/firm/control/differential_drive.cpp:151-162`, `applySpeedFloor`) is
structurally identical to theirs: scale *both* wheels by
`vMin / dominantMagnitude`, preserving the translation/rotation ratio
rather than boosting each wheel independently. The difference is the
gate — ours is unconditional whenever we own the command; theirs releases
the moment both axes are inside tolerance:

```python
correcting = abs(distance_error) > distance_tolerance \
          or abs(heading_error)  > heading_tolerance
effort = max(abs(left), abs(right))
if effort > max_effort:
    left, right = left * max_effort / effort, right * max_effort / effort
elif correcting and 0 < effort < min_effort:
    left, right = left * min_effort / effort, right * min_effort / effort
```

(`differential_drive.py:302-309`.) A floor that stays on at the target is
a floor that hunts around it — which is the shape of the "lands short,
stalls, lunges" terminal instability already on record. **Candidate fix,
not yet filed as an issue.**

**7.2 — `tolerance_count`: N consecutive in-tolerance samples as the
settle test** (`pid.py:52-58`), not a single-sample threshold crossing.
Cheap, and a better completion criterion than what our stop conditions
currently use.

**7.3 — One `_move(distance_target, heading_target, ...)` for both
straight and turn.** `straight()` calls it with `heading_target = 0`;
`turn()` calls it with `distance = 0` **and the two controllers swapped**
(`differential_drive.py:380`), so a user-supplied `main_controller` always
lands on the axis they care about. Our `Planner`/`Navigator` keep these
separate. The unification is clean and would collapse code — but it is
~40 lines of design idea, not an import.

Marginal, noted and rejected: `Controller` as an injectable interface
(`controller.py`) so a caller can pass a different control law into a move.
Good pedagogy; for us it is the wrong mechanism — a live config push is
the project's answer, and it satisfies
[`configuration-discipline.md`](../../.claude/rules/configuration-discipline.md)
(read-back, recorded values, promotion into the robot JSON) in a way an
ad-hoc object handoff does not.

## 8. Battery/voltage compensation — already answered, do not re-open

`DifferentialDrive.update_voltage_compensation()`
(`differential_drive.py:89-104`) reads the pack 8 times at construction and
scales all `_move()` effort by `nominal_voltage / measured`, clamped to
`[0.7, 1.6]`. Gains are tuned at nominal voltage; the duty is corrected for
the pack actually installed. `board.py:134-156` is the ADC read.

This is the right instinct and the **wrong version of it** — Pybricks
normalizes every actuation against an EMA-filtered pack voltage
(tau ≈ 0.6 s), not a one-shot reading at construction. Either way it is
already ruled out for us: **we cannot read pack voltage on the Nezha**
(stakeholder, 2026-08-02). `Hal::MotorDriver::supplyMillivolts()`
(`src/firm/hal/motor_driver.h:102`) returns 0 there; only the HiWonder
4-channel board exposes it, at register 0x00
(`src/firm/hardware/hiwonder/hiwonder_driver.h:87`). If we ever move to
HiWonder this reopens — as the **Pybricks** mechanism, not XRP's.

## 9. Do-not-copy list

- **50 Hz PWM** (`motor.py:18`, `:69-70`) — audible, coarse, heavy torque
  ripple, brutal low-duty quantization.
- **Unfiltered count-delta velocity** as a control input.
- **Derivative on error, unfiltered, with `time.ticks_ms()` dt**
  (`pid.py:70-87`) — at a 10 ms loop with 1 ms clock resolution the dt
  carries ~10% jitter, and dividing by it amplifies exactly that.
- **Blocking move loops.** They preclude checking a geofence at ~10 Hz
  *inside* a move, which
  [`playfield-testing.md`](../../.claude/rules/playfield-testing.md)
  requires. Everything concurrent in XRPLib has to become a `Timer`
  callback.
- **Encoder sign derived from the motor's `flip_dir` flag** rather than
  the encoder's own (`encoded_motor.py:135-139` — `Encoder` has its own
  `flip_dir`, defaulted `False` and then bypassed). That coupling is how
  you get a mirrored-heading bug; we have one on record already.
- **Board variants selected by substring-matching
  `sys.implementation._machine`**, with gain tables inlined at the branch
  — the opposite of configuration-as-a-file.

## Sources

XRPLib @ `main` / `23786b5deb00` (fetched 2026-08-12):
`XRPLib/{motor,encoder,encoded_motor,motor_group,pid,controller,
differential_drive,imu,imu_defs,board,timeout,defaults}.py`.
Companion: [`pybricks-motion-control-study.md`](pybricks-motion-control-study.md).
