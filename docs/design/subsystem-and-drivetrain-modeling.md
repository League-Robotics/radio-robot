# Subsystems and Drivetrain Modeling — A Design Note

*Status: design discussion, distilled. 2026-06-29.*

A first-principles analysis of two questions: **what a subsystem fundamentally
is**, and **how drivetrain geometry (differential / mecanum / swerve) should be
modeled** relative to it. Prompted by a comparison with the WPILib `kinematics`
package, but the conclusions are framework-independent — WPILib is treated as one
encoding of the ideas, not as the authority.

---

## 1. Two different things called "subsystem"

WPILib collides two unrelated concepts under nearby names, and the collision is
the source of most confusion:

- **The `kinematics` package** is *pure math and POD data*:
  `DifferentialDriveKinematics`, `MecanumDriveKinematics`,
  `SwerveDriveKinematics` are stateless converters; `ChassisSpeeds`,
  `*WheelSpeeds`, `*WheelPositions`, `SwerveModuleState` are data carriers.
  **None of these own hardware. None of them are subsystems.** They turn a
  chassis-frame twist into wheel-frame numbers and back.
- **A hardware-owning coordination unit** (WPILib spells this `Subsystem` in its
  command framework) is the bundle of motors + encoders + estimator + control
  logic. *This* is what "the drivetrain as a thing" means.

The geometry variation (differential vs mecanum vs swerve) lives in the **math**,
not in the **unit**. This distinction drives everything below.

### Mapping to this codebase

| WPILib `kinematics` (math/data) | This repo |
|---|---|
| `ChassisSpeeds` | `BodyTwist3` (vx, vy, omega) |
| `DifferentialDriveKinematics` | `BodyKinematics` namespace (`inverse`/`forward`/`saturate`) |
| `MecanumDriveKinematics` | `MecanumKinematics` namespace |
| `Kinematics<S,P>` interface | `IKinematics.h` — **compile-time alias**, not a runtime interface |
| `*WheelSpeeds` / `*WheelPositions` | the informal `float wheels[N]` array (no named type) |
| `DifferentialDriveOdometry` | `control/Odometry` — *richer*: also does EKF/OTOS fusion |
| command-framework `Subsystem` | `subsystems/drive/Drive` (currently a thin slice) |

Note: this repo's `Kinematics` is selected at **compile time**
(`namespace Kinematics = BodyKinematics; constexpr int kWheelCount = 2;`). On a
micro:bit that is the correct embedded analog of WPILib's `Kinematics<S,P>` type
parameter — WPILib uses runtime virtual dispatch and heap freely; this target
cannot and should not. The alias is the right shape, not a workaround.

Also note: the current `Drive` subsystem is *much thinner* than a full
drivetrain unit — it's just the per-wheel velocity control + encoder outlier
filter ("CONTROL COLLECT" block). It does **not** yet own the kinematics, the
odometry, or the OTOS. The pieces of the WPILib-style collection exist but are
not composed into one hardware-owning bundle.

---

## 2. What a subsystem fundamentally is

Capability is **not** what makes something a subsystem. A team adds an elevator
one year and removes it the next; a swerve chassis is swapped for a better
swerve chassis with new distance sensors — the capabilities change every season,
yet "the drivetrain subsystem" persists. So whatever defines a subsystem, it is
*not* the capability set.

The individuating question is sharper: **why is the elevator a *different*
subsystem from the drivetrain?** Not location (same frame). Not capability (both
have motors and sensors). The answer is **coupling**:

- The two drive motors are the **same** subsystem: `vL` and `vR` *jointly*
  determine `(v, omega)`, they share one state estimate (the pose), and one
  authority must decide both together.
- The elevator is a **different** subsystem: its DOF (height) is mechanically
  decoupled from planar motion. You can command height without knowing the pose.
- Two motors on one elevator carriage are the **same** subsystem: yoked to one
  DOF; command them out of lockstep and you rack the carriage.

### Definition

> **A subsystem is the unit of single authority over a coherent slice of the
> robot's state: the maximal cluster of devices and control logic that must be
> coordinated together, hidden behind one intent-level interface. Its boundary
> falls where coupling to the rest of the robot goes to (near) zero.**

Three load-bearing ideas:

- **Coherent slice of state** — the elevator owns *height*, the drivetrain owns
  *pose/twist*, a perception module owns *the world estimate*, a power monitor
  owns *the energy/health estimate*. Not all slices are mechanical DOF; a
  sensors-only subsystem owns a slice of *estimated* state.
- **Single authority** — exactly one decision-maker commands those devices at any
  instant. This isn't a framework rule; it's the consequence of actuators being
  shared mutable state with physical side effects. Two controllers on one motor
  is a race with real-world stakes. The subsystem is the serialization point.
- **Boundary at minimal coupling** — cut where you can drive the thing through
  its intent vocabulary without reaching inside any other subsystem. Maximize
  coordination *within*, minimize it *across*. (Parnas/Constantine
  cohesion-vs-coupling, applied to *physical* coupling.)

### Consequences

- **Identity = boundary, not contents.** Adding a second elevator motor or new
  chassis sensors changes the *contents*; the slice-of-state it owns — its
  identity — does not move. Capability is what lives inside the cut and changes
  every season. The cut is the architecture.
- **It exposes intent and hides mechanism.** Height / twist / world-pose go in;
  motor volts, kinematics, sensor fusion stay inside.
- **It owns its own estimate.** The drivetrain owns its pose, so `Odometry`
  belongs *inside* the drivetrain boundary, not in a global module.
- **Boundaries are sometimes judgment calls.** New chassis distance sensors
  belong to whichever control loop they close: the drivetrain's own loop
  (wall-following, collision limiting) → inside the drivetrain; robot-wide
  planning → a separate perception slice the drivetrain merely consumes.

---

## 3. Drivetrain geometry is *contents*, not *boundary*

The wheels are coupled (they jointly make a body twist), they share one estimator
(pose), and they need one arbiter — and **that coupling exists identically for
differential, mecanum, and swerve.** So all three are the *same coordination
boundary* ("single authority over planar motion + pose") with different
*internal* structure (the kinematics map and wheel-state shape).

Therefore: **model the drivetrain's stuff (motors, encoders, kinematics,
odometry) as one bounded subsystem, distinct from the elevator and gripper —
always.** Swapping geometry changes what is *inside* the subsystem, not which
subsystem it is.

### One drive class or two?

Once you are *not* forcing a single substitutable `drive()` across geometries
(see §4), "one `Drivetrain` class with a swappable kinematics interior" vs. "two
classes `DifferentialDrive` / `MecanumDrive`" is a **pure cohesion/duplication
question**, decided by how much of the *shell* is genuinely shared:

- **Differential ↔ mecanum**: same shell — own N motors, spin them at commanded
  wheel speeds, filter encoders, integrate odometry. Only the inverse map, wheel
  count, and a `vy` term differ, and those already factor out cleanly.
  → **One shell + kinematics strategy** wins (on duplication, not principle).
- **Differential ↔ swerve**: the *control structure itself* diverges — swerve has
  per-module steering-angle servo loops a differential lacks. The shell is no
  longer shared. → **Two distinct classes** (possibly sharing a lower-level
  "swerve module" sub-unit) is defensible, maybe better.

Either way the drivetrain stays *one bounded thing*; the two-class case is two
*implementations of the same boundary*, not a breach of it.

---

## 4. The interface question: capability classes, not "base class: yes/no"

Should there be a single `IDriveSubsystem` with `drive(vx, vy, omega)`, even
though `vy` is unimplemented on a differential?

First, separate two **orthogonal** axes that are easy to conflate:

- **A — encapsulation / boundary**: is the drivetrain modeled as one bounded
  thing? → **Yes, always** (§3).
- **B — substitutability**: is there one generic *drive* interface every geometry
  implements, callable polymorphically? → the real question here.

A generic `Subsystem` base (contract: `periodic(now)`, lifecycle, "I own a slice
of state and its estimate") that drivetrains *and* elevators derive from is
legitimate and useful — it's what lets a container do
`for (Subsystem& s : subsystems) s.periodic(now);`. That base is **not** the
issue, because it never promises `strafe()`. Deriving `DifferentialDrive` and
`MecanumDrive` from it is fine.

The real principle (which replaces the weaker "differential can't strafe, so no
base class" argument — that reasoning is **roster-contingent** and a bad basis
for a decision):

> **Draw the interface around a capability class. The base command vocabulary
> must be the *intersection* of what every implementor can honor; capabilities
> only some members have go on a refinement interface.**

This dissolves the false "lie vs lowest-common-denominator" dilemma:

- **Holonomic family** {mecanum, X/omni, swerve}: the intersection *is*
  `(vx, vy, omega)`. A single `IDriveSubsystem` with the full holonomic command
  loses nothing and lies to no one. One interface, honest for all. **Endorsed.**
- **Mixed family** {differential, mecanum, swerve}: the intersection is
  `(v, omega)` — not a contingent collapse but the genuine capability reality (a
  non-holonomic drive truly cannot strafe under any environment). So:
  - `IGroundDrive { drive(v, omega); pose(); twist(); }` — honest base, all honor it.
  - `IHolonomicDrive : IGroundDrive { drive(vx, vy, omega); }` — the holonomic
    subset implements it.

The refinement is **not** extra machinery in the uniform case: if the whole
family is holonomic, everyone implements `IHolonomicDrive` and it collapses back
to the single base. You only pay for the second level when members genuinely sit
at two capability levels.

### When the flat `(vx, vy, omega)`-with-no-op IS acceptable

A flat interface where differential silently no-ops `vy` is a defensible
engineering choice whose safety depends on **who originates `vy`**:

- **Dangerous** if a *generic consumer* authors `vy` and dispatches it to a
  drivetrain whose concrete type it does not statically know → silent wrong
  behavior, surfaced at runtime on the field. The capability refinement is what
  buys this back: the consumer that needs strafe takes an `IHolonomicDrive&`, and
  a differential cannot be passed.
- **Harmless** if consumers either only ever command `(v, omega)`, or always know
  the concrete holonomic type when they command `vy`.

The defect to avoid is never "having a base class" — it's **putting a method on a
type that can't honor it, where a caller can't see the limitation from the
type.** That only bites in mixed families with runtime-polymorphic dispatch.

---

## 5. Application to this robot

- The real family is **differential + mecanum** — *mixed* (non-holonomic +
  holonomic). The principled structure is base at `(v, omega)` with a holonomic
  refinement for the strafe.
- But drivetrain is selected at **compile time** (`IKinematics` alias), so there
  is no runtime polymorphism over drivetrains: the concrete type is fixed per
  image. When the type is statically known per build, a differential image can
  simply **not expose `vy`** — the impossible command becomes unrepresentable
  rather than silently dropped, and the whole flat-vs-refined interface question
  largely dissolves. The interface hierarchy earns its keep mainly in the
  runtime-polymorphic, one-image-many-drivetrains world (the library/FRC
  situation), not this one.
- **The only thing crossing the subsystem boundary is the chassis twist
  (`BodyTwist3`) and the pose (`Pose2D`).** Wheel-state (`SwerveModuleState`,
  `float wheels[N]`) is internal currency — born in `Kinematics::inverse`,
  consumed by motor control, dead before it leaves the shell. Keeping wheel-shape
  from leaking upward is what makes a future swerve a cheap additive change.

---

## 6. Summary

1. "Subsystem" (a hardware-owning coordination unit) and "kinematics" (pure math)
   are different concepts; geometry varies the math, not the unit.
2. A subsystem is a **cut in the robot's coupling graph** — single authority over
   a coherent slice of state. **Identity is the boundary; capability is the
   changing contents.**
3. The drivetrain is **one bounded subsystem**, distinct from other subsystems,
   regardless of geometry. Geometry is interior.
4. A common drive *interface* is good or bad depending on **capability class
   membership**, not on whether a base class exists: base = the honest
   intersection; richer capabilities = refinement interfaces. The roster-driven
   "differential can't strafe" objection is contingent and should not drive the
   design.
5. For this compile-time-selected, embedded target, much of the interface
   debate dissolves: keep one drivetrain subsystem, swap the kinematics interior
   at build time, and let only the chassis twist and pose cross the boundary.
