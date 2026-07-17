---
root: ../DESIGN.md
---

# Motion (`src/firm/motion`, namespace `Motion`)

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-17 · **Status:** in-flux

---

## 1. Purpose

`motion/` solves jerk-limited (or trapezoid, if unconfigured) motion
profiles for a single 1-DoF channel. `Motion::JerkTrajectory` wraps one
vendored `ruckig::Ruckig<1>`/`ruckig::Trajectory<1>` pair (`src/vendor/
ruckig/`) and answers exactly one question per instance: "given where this
channel is now (or was last told it was), what is the jerk-limited path to
a requested target (position, and optionally velocity), and what is that
path's state at time T?" It knows nothing about goal kinds (arc, pivot,
straight leg), wire verbs, CODAL, or queueing — a future `Motion::Cmd`/
`Motion::Executor` (sprint 109 ticket 003) is the first consumer, holding
two instances (linear and rotational channel) and driving them from the
loop's cycle. This ticket (109-001) restores the solver only — nothing in
the loop calls it yet.

## 2. Orientation

One class, `JerkTrajectory`, with three solve entry points and one sample
entry point:

- **`solveToRest(targetPosition, maxVelocity)`** — position-control,
  decelerate to rest exactly at `targetPosition`. Equivalent to
  `solveToState(targetPosition, 0, maxVelocity)`.
- **`solveToState(targetPosition, targetVelocity, maxVelocity)`**
  (109-001's new entry point) — position-control, arrive at
  `targetPosition` carrying `targetVelocity` (nonzero) rather than stopping
  — the boundary-velocity-carry primitive a future queue/executor needs so
  consecutive same-direction commands don't decelerate to zero at each
  hand-off.
- **`solveToVelocity(targetVelocity, maxVelocity)`** — velocity-control,
  open-ended (no target position); used for cruise ramp-up and,
  target-velocity 0, any stop-triggered terminal decel.
- **`retarget(newRemaining)`** / **`reanchor(position, velocity)`** —
  divergence-triggered re-solves for a future replan policy (normal case:
  re-baseline and re-solve, seeded from this channel's own remembered
  state; gross case: re-solve seeded from a caller-supplied state,
  accepting a velocity discontinuity).
- **`sample(elapsed)`** / **`peek(elapsed)`** — evaluate the held
  trajectory; `sample()` updates the remembered seed state used by the next
  solve, `peek()` is a pure read that does not.

Every method's public signature uses only `float`/`JerkTrajectory::State` —
no `ruckig::` type crosses this class's boundary. See `jerk_trajectory.h`'s
class comment for the full design rationale (solve modes, the jerk == 0
sentinel, the direction-mirrored acceleration bounds) — it is the primary,
load-bearing design reference for this subsystem; this document stays at
map/boundary altitude and does not repeat it.

## 3. Constraints and Invariants

- **HOST_BUILD-pure, no `MicroBit.h`.** `jerk_trajectory.{h,cpp}` compile
  under both the ARM target and `-DHOST_BUILD` unchanged — this is a leaf
  library exactly like `kinematics/`, not an ARM-only module.
- **No heap.** `ruckig::Ruckig<1>`/`ruckig::Trajectory<1>` are
  compile-time-DoF, `std::array`-backed value types held as ordinary stack/
  member state — no dynamic allocation anywhere in this subsystem.
- **Seeding contract: never seed a solve from a measured observation.**
  Every `solveToRest()`/`solveToState()`/`solveToVelocity()` call reads its
  current (position, velocity, acceleration) back from this channel's OWN
  remembered last sample — never from a live sensor reading (`leftObs`/
  `rightObs` in the pre-102 codebase's terms). This is the single most
  important invariant in this subsystem: an earlier version of this
  codebase fed a measured wheel speed into a similar formula and produced a
  traced limit-cycle oscillation (bug 087-009, `.clasi/knowledge/`).
  `retarget()`/`reanchor()` are the ONLY two narrow, deliberate exceptions
  (both still solver-internal re-solves, never a bus read) — see the header
  comment. A `test_never_reads_measured_observations`-style static text
  check (`src/tests/sim/unit/test_jerk_trajectory.py`) pins this by scanning
  the class's own code for `leftObs`/`rightObs`.
- **`calculated_` UB guard.** A default-constructed `ruckig::Trajectory<1>`
  has real-zero `duration` but an uninitialized `profiles` array; `sample()`/
  `peek()`/`duration()` must never touch `traj_` before the first successful
  `calculate()` — guarded by the `calculated_` flag, returning a safe zero
  `State{}`/`0.0f` instead.
- **This subsystem does not own the never-solves-backward guard, divergence
  thresholds, or replan rate limiting.** Those are a future caller's
  (`Motion::Executor`, ticket 003) responsibility — `retarget()`/
  `reanchor()` solve whatever they are told to solve, including a
  backward-pointing target, by design (see the header comment's
  `scenarioBackwardTargetIsDefinedButUnguarded`-style test).
- **`jerk == 0` sentinel, not a literal zero jerk.** `configure()`'s
  `j_max`/`yaw_jerk_max == 0.0f` maps to Ruckig's own `max_jerk = +infinity`
  (a trapezoid profile), matching the existing `msg::PlannerConfig` wire
  convention — do not special-case a literal `0.0` max_jerk anywhere else in
  this subsystem.

## 4. Design

`JerkTrajectory` has one private worker, `solvePositionControl()`, shared by
`solveToRest()`, `solveToState()`, `retarget()`, and `reanchor()` — all four
are the same Ruckig `Position`-control-interface solve, differing only in
what current/target state (including, since 109-001, a target velocity)
each passes in. `solveToVelocity()` is the one genuinely different mode
(Ruckig's `Velocity` control interface, open-ended, no target position).
Every solve writes into a temporary `ruckig::Trajectory<1>` first and only
commits it to the held `traj_` on success (`Result::Working`) — a failed
solve must never corrupt the trajectory a caller is still sampling.
Direction (for the direction-mirrored acceleration bounds and the
no-reversal velocity band) is computed purely from `targetPosition -
currentPosition`'s sign — a math fact derived from the solve's own inputs,
never a caller-supplied flag.

## 5. Interfaces

### Exposes

- **`Motion::JerkTrajectory`** (`jerk_trajectory.h`) — `configure()`,
  `reset()`/`seedCurrent()`, `solveToRest()`/`solveToState()`/
  `solveToVelocity()`, `retarget()`/`reanchor()`, `sample()`/`peek()`,
  `duration()`. See §2 above and the header's own class comment for the
  full contract.

### Consumes

- **`msg::PlannerConfig`** (`messages/planner.h`) — `configure()`'s only
  input: `a_max`/`a_decel`/`v_body_max`/`j_max` (linear channel) or
  `yaw_acc_max`/`yaw_rate_max`/`yaw_jerk_max` (rotational channel). This is
  the ONLY `msg::*` type this subsystem references.
- **`ruckig::Ruckig<1>`/`ruckig::Trajectory<1>`/`ruckig::InputParameter<1>`**
  (`vendor/ruckig/`) — the vendored solver this class wraps; a private
  implementation detail never exposed past this header/`.cpp` pair.

### Not yet consumed by anything

As of 109-001, nothing in `app/` calls `Motion::JerkTrajectory` — the root
`src/firm/DESIGN.md` §2 dependency diagram's `motion` node has no incoming
edge yet. Ticket 003 (`Motion::Cmd`/`Motion::Executor`) and a future
`App::Pilot` are the first consumers.

## 6. Open Questions / Known Limitations

- **`solveToState()`'s direction-band interaction is untested for a
  target velocity that opposes the solve's own direction of travel** (e.g.
  requesting a positive `targetVelocity` on a negative-direction solve).
  Ruckig's own `min_velocity`/`max_velocity` band (§3's no-reversal
  invariant) would reject such an input as infeasible, which is the
  correct outcome, but it is the CALLER's job (a future `Motion::Executor`)
  to never construct such a request — same caller-responsibility boundary
  as `retarget()`/`reanchor()`'s unguarded backward target.
- **Flash budget.** Restoring Ruckig is flash-neutral until something
  calls it (dead-code-eliminated by `-Wl,--gc-sections` — see
  `vendor/ruckig/README.vendored.md`); this ticket's own
  `arm-none-eabi-size` before/after baseline is recorded in ticket
  001's completion notes for later tickets (the first real call site) to
  track against.
- **`Motion::Cmd`/`Motion::Executor`, `App::Pilot`, `App::HeadingSource`**
  do not exist yet — this doc describes only the restored solver
  (`JerkTrajectory`) landed by this ticket; the sprint 109 `sprint.md`
  Architecture section is the forward-looking reference for the rest of
  the `motion`/`app` additions still to come.
