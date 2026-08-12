---
root: ../../../docs/design/design.md
---

# control/

**Owner:** Eric Busboom · **Last reviewed:** 2026-08-12 · **Status:** stable

---

## 1. Purpose

`control/` holds the wheel-speed **control law** — `Control::DifferentialDrive`
(`fastPid()` plus the Stage A/B/C correction, bias adaptation, and
stall/deficit-latch machinery) — and nothing else. It is a single-purpose
layer for exactly one reason: this logic is neither orchestration (`core/`,
which decides WHEN to call it and what to do with its outputs) nor geometry
(`kinematics/`, which is stateless twist↔wheel-speed math with no PID, no
adaptation, no stall detection of its own) nor an interface (`hal/`, which
declares contracts but holds no stateful control logic anywhere in it).
Relocated verbatim out of `core/differential_drive.{h,cpp}` (136-006,
"control law out of `core/` — new `control/` layer") — the rename from
`Core::DifferentialDrive` is the only thing that changed; every PID gain,
every stage, every stall-detection threshold is untouched.

## 2. Orientation

```
control/
  differential_drive.{h,cpp}   Control::DifferentialDrive
```

One class today. `Control::DifferentialDrive` owns:

- **Stage A** — the open-loop duty/speed calibration and per-wheel accel/
  decel correction curve.
- **Stage B** — `fastPid()`, the closed-loop PID against commanded vs.
  measured wheel speed, plus position-error clamping.
- **Stage C** — slow bias adaptation (one adapted trim parameter per
  wheel) and the stall/deficit latch machinery that `Core::RobotLoop`
  reads to halt the robot when a wheel is commanded to move and isn't.

`Core::RobotLoop` (`core/robot_loop.cpp`) owns one `Control::DifferentialDrive`
instance and drives it every cycle via `tick()`/`update()`/`command()`;
`Core::RobotGraph`/`composeRobot()` (`core/boot_wiring.cpp`) constructs it.
Neither of those files' own responsibilities moved here — they still decide
*when* to call into this class and what to do with `Types::RobotState::Wheel::
cmdVelocity` once it writes it; this class only computes the number.

## 3. Constraints and Invariants

- **Layer position: `hal/` → `control/` → `core/`.** `control/` may reach
  down into `hal/` (`Hal::Motor`, the actuator it drives) and `firm/types/`
  (`Types::RobotState`, the per-cycle blackboard it reads/writes) — the
  same two cross-cutting floors `core/`, `kinematics/`, and `motion/` all
  stand on. It may NOT reach into `core/` or `motion/`: reaching upward
  into orchestration or across into the motion-planning library would
  invert the layering this reorganization exists to hold.
- **No `test_layer_isolation.py` entry.** That test's three-layer table
  (`hal/`, `platform/`, `hardware/`) never covered `core/`, `kinematics/`,
  or `motion/` either — all four are cross-cutting-adjacent layers with a
  `config/`/`firm/types/` dependency the mechanical grep test was never
  built to allow for. The boundary above is documented here in prose
  instead, matching those three siblings' existing treatment.
- **Pure relocation, not a redesign.** This ticket (136-006) changed the
  namespace and the file's directory; it did not touch a single PID gain,
  correction-stage constant, or stall/deficit threshold. Any future change
  to the control law itself belongs to a ticket that says so explicitly.

## 4. Design notes and open items

**The deferred `Hal::Wheel` migration lands here eventually, not yet.**
`hal/DESIGN.md` §4 describes moving this control law down onto a
closed-loop `Hal::Wheel` actuator abstraction (the raw `Hal::Motor` is
duty-commanded only — `setDuty()`, open loop — so there is no
angular-velocity entry point to build `Wheel` on top of today). That is a
real, wanted, and explicitly deferred piece of future work: it changes
*behavior* (where the control law's ownership boundary sits, what a
`Hal::Wheel` interface looks like), not just *location*, and so belongs in
its own future sprint. `control/` is named and scoped so that migration has
an obvious destination when it happens — this ticket does not attempt it.

**Why not fold into `kinematics/`.** `kinematics/` is stateless: `Kinematics::
Model` and its `Differential`/`Mecanum` implementations are pure twist↔
wheel-speed conversions with no per-cycle state, no PID integrator, no
adaptation history. `Control::DifferentialDrive` is the opposite — it is
almost entirely per-cycle state (bias, latch timers, position references).
Putting a stateful PID controller inside the stateless geometry layer would
break `kinematics/`'s own cohesion for the convenience of not adding a new
top-level directory.

**Why not leave it in `core/`.** `core/` is orchestration — `Core::RobotLoop`'s
single cooperatively-timed cycle, `Core::Comms`, `Core::Telemetry`,
`Core::Configurator`, `Core::Preamble`, the composition root. None of those
compute a control law; they call into one. Leaving `DifferentialDrive` in
`core/` mixed "decides what happens this cycle" with "computes the wheel
speed a motor should be commanded at," which is exactly the layering
ambiguity the platform/hardware/hal reorganization exists to remove.
