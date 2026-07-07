---
status: pending
---

# Planner motion planning via vendored Ruckig (jerk-limited online trajectory generation; stop-at-zero by construction)

> **DESIGN ISSUE — FOR DISCUSSION, NOT READY TO IMPLEMENT.** First pass. The
> stakeholder chose Ruckig (<https://github.com/pantor/ruckig>) to generate the
> motion plans; the C++-standard feasibility question (below) is the make-or-break
> item to settle before any implementation. We will talk this through further.
>
> Supersedes the earlier hand-rolled-trapezoid framing (this file replaces
> `planner-explicit-trapezoidal-motion-plan.md`). We are **not** hand-rolling
> trapezoids — Ruckig produces the (jerk-limited) trajectories.

## Problem (confirmed on real hardware, 2026-07-07)

Motion verbs mis-terminate, both symptoms of one root — the Planner has no real
motion plan:

1. **Not a clean profile.** `D 200 200 1000` overshoots to **~292 mm/s**
   (commanded 200) then sags to ~219 — it never tracks a proper
   accelerate/cruise/decelerate profile.
2. **Runs backward at the end.** `D 200 200 1000` reverses **~16 mm**,
   `T 200 200 1000` reverses **~23 mm** after `EVT done` — a *timed* command
   driving backward, which is unambiguously wrong.

Mechanism: `Motion::VelocityRamp` is a per-tick `approach()` toward a velocity
target; on a stop the Planner sets target `(0,0)` and keeps emitting a velocity
twist. The velocity servo (`Hal::MotorVelocityPid`) then sees `err = 0 -
measured < 0` on a still-coasting wheel and commands a **negative (reverse) duty
to brake**, overshooting zero into a reverse spin (isolated:
`compute(target=0, measured=268) → −0.59`). This is the 086/087 terminal-
overshoot family (`086-002`/`086-003`/`087-009`, and
`rt-open-loop-overshoot-under-synchronous-update.md`) — reduced but never
eliminated. Point band-aids (open-loop coast; re-tune the velocity gains) were
**rejected**: the fix belongs in the motion plan.

## Approach: the Planner generates jerk-limited trajectories with Ruckig

Vendor Ruckig's **community version (MIT)** and have the Planner turn each
command into a Ruckig **trajectory** — a first-class, inspectable motion-plan
object — instead of a per-tick velocity chase.

How Ruckig maps onto the stakeholder's intent:

- **"The motion plan is an object with phases and times."** Ruckig's `Trajectory`
  is exactly that: computed once via `otg.calculate(input, trajectory)`, it holds
  the whole profile (its jerk-limited phase segments and their durations) and is
  sampled at any time via `trajectory.at_time(t) → (position, velocity,
  acceleration)`. Inspectable and testable as a plan, not emergent servo
  behavior. (Ruckig replaces the trapezoid with a **jerk-limited** profile — the
  same three-region shape, smoothed at the corners.)
- **"Command the drive frame by acceleration."** Ruckig outputs `new_position /
  new_velocity / new_acceleration` each cycle; the Planner→Drivetrain edge carries
  the trajectory's acceleration (and/or velocity) rather than only a velocity
  twist.
- **"Implicit stop-at-zero; a deceleration cannot cross zero into reverse."** Set
  the target state's `target_velocity = 0` (and `target_acceleration = 0`): Ruckig
  plans a jerk-limited deceleration that arrives at the goal **at rest** and, by
  construction, never crosses the zero-velocity boundary. No terminal servo brake,
  so no reverse — the invariant holds by construction, not by a servo catching an
  overshoot after the fact.

### Integration sketch (to refine in discussion)

- On receiving a command (`D`/`T`/`TURN`/`RT`/`G`), the Planner builds an
  `InputParameter`: `current_{position,velocity,acceleration}` from the pose/
  velocity estimate at command time, `target_{position,velocity=0,acceleration=0}`,
  and `max_{velocity,acceleration,jerk}` from config.
- **Compute once, sample per tick** (preferred): `calculate()` the `Trajectory`
  object at command time; each control tick, `at_time(elapsed)` for the commanded
  (vel/accel) and hand it to the Drivetrain. This is cheap per-tick (no re-solve),
  gives the inspectable object, and yields `EVT done` at `trajectory.get_duration()`.
- **Or online/reactive** (`update()` each tick) where replanning from live feedback
  matters — e.g. `G` re-steering from the fused pose. Ruckig supports both; pick
  per verb.
- Replaces `Motion::VelocityRamp` **and** `Planner::applyStopAnticipation()` (the
  087-009 closed-form stop-distance cap is subsumed by Ruckig's planning).

## Library facts (community version)

- **License: MIT** (community) — vendorable. Pro/Cloud adds local waypoint
  real-time, hard constraint enforcement, tracking, interrupts — **not needed** if
  we stay state-to-state (see cloud caveat below).
- **No mandatory dependencies**; template-based; Eigen optional.
- **Fixed-DoF template `Ruckig<N>` uses `std::array` — no heap after
  construction**; real-time ("control cycles as low as 250 µs"). Use this form,
  not `Ruckig<DynamicDOFs>` (which uses `std::vector`/heap).
- **State-to-state is fully local.** Community's *waypoint* (intermediate-position)
  planning uses a **cloud API** — so **avoid waypoints**; use single state-to-state
  trajectories only (which is all we need). Flag if a future feature wants
  waypoints (that would need Pro).
- Vendor location: **`libraries/ruckig/`**, following the existing vendored-lib
  precedent (`libraries/tinyekf`, `libraries/cmon-pid`).

## CRITICAL feasibility concerns (settle these first)

1. **C++ standard — the make-or-break item.** Firmware compiles at
   **`-std=c++11 -fno-exceptions -fno-rtti`** (`libraries/codal-microbit-v2/
   target.json`), and the host sim is pinned to C++11 to match. **Ruckig's
   published community version is C++20.** The ARM toolchain (`arm-none-eabi-g++
   15.2`) *does* support C++20, so the constraint is CODAL's flags + the
   no-exceptions/no-RTTI policy, not the compiler. Options to evaluate:
   - **(a) Obtain a C++11 Ruckig variant.** Ruckig's README: *"A C++17, C++11, and
     C++03 version of Ruckig is also available — please contact us if you're
     interested."* Cleanest **if** it is available under a compatible (MIT/free)
     license — **contact pantor; confirm licensing** before assuming.
   - **(b) Encapsulate Ruckig behind a C++11 facade, compile its TU at C++20.**
     Ruckig internals in a translation unit built with per-file `-std=c++20`
     (+ exceptions if needed), exposed to the C++11 firmware only through a thin
     C++11-safe wrapper (POD in/out). Feasible with GCC 15 + per-TU flags; needs
     build-system work and careful ABI/exception isolation.
   - **(c) Port Ruckig to C++11** ourselves (defeats "just vendor it"; last resort).
2. **`-fno-exceptions`.** Ruckig may `throw` on invalid input/allocation. The
   online/offline solve returns a `Result` enum (`Working`/`Finished`/error) —
   confirm the used path is throw-free, or isolate throws (option b's TU).
3. **`double` on Cortex-M4F.** Ruckig computes in `double`; the M4F FPU is
   **single-precision only**, so `double` math is soft-float (slow). Mitigated by
   **computing the trajectory once per command** (offline `calculate()`), not every
   tick — but measure the solve time, and consider a `float` adaptation.
4. **Flash/RAM footprint** on the nRF52833 (512 KB flash / 128 KB RAM shared with
   CODAL). Measure the vendored footprint; the solver is algorithmically nontrivial
   (CI runs 5e9 random trajectories).
5. **Prototype on the host build first.** Validate the algorithm + integration in
   the C++11 host sim by relaxing *just the Ruckig TU* to C++20 there, before
   tackling the ARM/CODAL constraint. Keeps sim↔hardware on one shared code path.

## Open questions (for discussion)

- **DoF mapping.** One 1-DoF Ruckig per channel (linear `v_x`, rotational `omega`),
  or a coupled 2-DoF? How do `G` (x,y + heading), `TURN` (absolute heading), and
  `RT` (relative rotation) map? What are the max vel/accel/**jerk** limits per
  channel, and where do they live in config?
- **Offline vs online per verb** (`calculate()` once vs `update()` reactive) —
  especially `G`'s re-steer from the fused pose.
- **How the acceleration command + trajectory sampling are represented on the
  Planner→Drivetrain edge** (extend `msg::DrivetrainCommand`/`BodyTwist` with an
  acceleration + the sampled velocity?).
- **Closed-loop verbs** (`TURN`/`RT`/`G`) carry delicate 086/087 calibration tied
  to the current terminal servo — how they re-home onto the Ruckig model without
  regressing their accuracy.
- **Scope/sequencing.** Likely phased: vendor + host-sim prototype → C++-standard
  decision → Planner integration for `D`/`T` → then `TURN`/`RT`/`G` → retire
  `VelocityRamp`/`applyStopAnticipation`.

## Acceptance (to refine once the design is settled)

- Ruckig vendored (`libraries/ruckig/`, MIT) and building in both the host sim and
  the ARM firmware (C++-standard question resolved).
- The Planner holds an inspectable Ruckig `Trajectory` per command (phases +
  durations, `at_time()` sampling).
- A commanded deceleration provably cannot cross zero velocity into reverse.
- On the stand: `D`/`T` complete with **no** reverse and a clean jerk-limited
  wheel-velocity profile (no 292-vs-200 overshoot); `TURN`/`RT`/`G` still settle.
