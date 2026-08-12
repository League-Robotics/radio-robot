---
root: ../../../docs/design/design.md
---

# Kinematics

**Owner:** Eric Busboom · **Last reviewed:** 2026-08-09 · **Status:** in-flux

---

## 1. Purpose

`kinematics/` owns the **twist ↔ wheel-speed map**, and with it the only
sanctioned home for **chassis geometry** (track width, wheelbase).
Everything above states intent as a body twist `(v_x, v_y, omega)`;
everything below deals in per-wheel linear speeds and knows nothing about
the chassis.

This directory was retired by sprint 122 (its `BodyKinematics` folded flat
into `src/motion`) and is live again as of the platform/hardware/hal
reorganization, for the reason that reorganization exists: a drivetrain map
hardcoded to a single scalar track width is exactly what leaves "a mecanum
or X-drive robot with no home."

## 2. Orientation

```
kinematics/
  kinematics.h                Kinematics::Twist, Kinematics::Model, saturate()
  kinematics.cpp              Model::saturate() -- the one concrete method
  differential.*               two wheels; the former BodyKinematics math
  mecanum.*                    four wheels, holonomic
```

- **`Kinematics::Twist`** is a plain float aggregate. It is deliberately
  *not* `msg::BodyTwist3`: that type's `v_x`/`v_y`/`omega` are `int32_t`
  RAW wire counts that only mean anything through its own
  `packVX()`/`unpackVX()` 0.1 scale. See §4 for what that mismatch had
  already produced.
- **`Kinematics::Model`** is the interface: `wheelCount()`,
  `inverse(twist, wheels[])`, `forward(wheels[], twist)`, plus a concrete
  `saturate()`.
- Both implementations expose their equations twice: as **statics** taking
  the geometry explicitly (the pre-reorganization free-function shape,
  which every existing call site still uses) and as the **virtual
  overrides**, which call those statics with the instance's own geometry.
  One copy of each equation, two entry points.

`saturate()` is concrete rather than virtual because "scale every wheel by
the same factor so the fastest sits at the ceiling" is a property of
preserving the wheel-speed ratio, which is drivetrain-independent. Only
`wheelCount()` varies, and it reads that.

## 3. Constraints and Invariants

- **Chassis geometry lives here and nowhere below.** A `Hal::Motor` knows
  nothing about track width; a `Hal::Wheel` (when it exists) would know a
  diameter, not a chassis.
- **Pure functions.** No I2C, no clock, no global state, no heap. Every
  entry point is a pure map from its inputs.
- **No wire types.** This layer deals in real units. It includes nothing
  from `messages/`, which also keeps it usable from `motion/` under that
  subsystem's own dependency rule.
- **A model that cannot realize a requested component ignores it** rather
  than failing — `Differential::inverse()` drops `v_y`. Refusing
  an impossible motion is the caller's limit checking, not this map's job.

## 4. Design notes and open items

**A latent scale bug removed.** The pre-reorganization `BodyKinematics`
carried array-form overloads taking `msg::BodyTwist3`, and they assigned
raw floats straight into that struct's `int32_t` fields — bypassing
`packVX()`. Any consumer reading them back through `unpackVX()` would have
seen a 10× scale error on top of truncation to whole raw counts. It never
fired because nothing ever called them: every real call site used the
scalar forms. They are not carried forward.

**`Motion::Planner` does NOT take a `Kinematics::Model&`, and that is the
single largest gap between this reorganization and the proposal that
shaped it.** The proposal states that `Motion::Planner` "currently calls
`BodyKinematics` functions directly for its per-wheel profiling" and would
"take a `Kinematics&` instead of assuming differential." The first half is
not accurate, and it changes the size of the second half:

- `Motion::Planner` calls `BodyKinematics` **nowhere**. It inlines
  differential-drive algebra against `limits_.plant.trackWidth` in roughly
  fifteen places in `planner.cpp` — `halfTrack` splits, `(right - left) /
  trackWidth` yaw-rate recoveries, `alphaDecel * 0.5f * trackWidth`
  ceilings, per-wheel profile shaping.
- `Core::DifferentialDrive` does the same: its constructor takes exactly two
  `Hal::Motor&` and a scalar `trackWidth`, and `(targetRight_ -
  targetLeft_) / trackWidth_` is how it reports commanded omega.

So making the planner drivetrain-agnostic is not a parameter swap; it is a
rewrite of its per-wheel profiling model in terms of N wheels, in the one
component that actually drives the robot. That is a behavior-changing
project with its own hardware verification, not part of a reorganization
whose contract is "same behavior, better addresses" — doing it inside this
reorganization would have meant shipping an unverifiable diff.

What this directory delivers instead is the seam that project needs: a real
interface, a second implementation to prove it is one, and every existing
`BodyKinematics` caller already moved onto it. `Mecanum` is
unconstructed for exactly this reason — there is no four-wheel drivetrain
to hand it to yet. Togov is a real mecanum chassis
(`data/robots/togov.json`), so that is the next drivetrain, not a
hypothetical one.
