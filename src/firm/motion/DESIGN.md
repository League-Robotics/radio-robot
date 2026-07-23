---
root: ../../../docs/design/design.md
---

# Motion (`src/firm/motion`, namespace `Motion`)

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-21 · **Status:** stable

---

## 1. Purpose

`Motion::StopCondition` answers exactly one question, every cycle, for
exactly one active bounded `Move`: **has this motion ended?** It captures
the three baselines a `Move`'s stop condition is measured against
(activation time, path length, heading) once, at activation, and compares
the caller's current readings against them on every subsequent `tick()`
call. It is the safety-critical bound math sprint 116's MOVE protocol
depends on: every `Move` the host sends is required to be self-bounding
(a kind-specific stop condition, or the `timeout` backstop when the
condition can't be reached), and this is the one place that decision gets
made. It is split out of `App::MoveQueue` (which owns and drives it) for
the same reason `BodyKinematics` is split out of `App::Drive`/`App::
Odometry`: it is pure comparison logic with no bus, no timing side
effects, and no state beyond what's passed in — the one place in the
firmware that can be trusted to compile, link, and be exercised
identically on ARM and in a host unit test with zero scaffolding, and
independently testable without standing up `MoveQueue`'s own enqueue/
replace/`ERR_FULL` machinery.

## 2. Orientation

One class, two methods:

- **Constructor** — `StopCondition(kind, threshold, timeout, now,
  pathLength, theta)`. Captures every baseline the comparison will need
  at construction time; there is no separate `arm()`/`activate()` step. A
  `Move`'s activation *is* this object's construction — the owning
  `MoveQueue` constructs a fresh `StopCondition` each time a `Move`
  activates and discards it when that `Move` ends.
- **`tick(now, pathLength, theta)`** — the per-cycle comparison. Returns
  one of three distinguishable `Outcome`s (`Continue`, `StopConditionMet`,
  `TimedOut`) rather than a collapsed bool, because the caller needs to
  tell the two "ended" cases apart to set `kFlagFaultMoveTimeout`
  correctly on the wire.

Three kinds, matching the wire's own `Move.stop` oneof in spirit (not in
type — see §3): `Kind::Time`, `Kind::Distance`, `Kind::Angle`.

## 3. Constraints and Invariants

- **Stateless comparison, no I2C, no globals, no heap, no owned
  collaborators.** `tick()` is `const` — it mutates nothing. Every
  reading it compares against (`now`, `pathLength`, `theta`) is a
  parameter, never read from a held `Devices::Clock&`/`App::Odometry&`
  reference. This is the whole reason the module is split out: it must
  compile and behave identically under `HOST_BUILD` and on ARM with no
  fakes or seams, and it must be constructible and testable with
  hand-fed numbers alone.
- **Zero dependency on `App::MoveQueue`, `App::Drive`, or any `msg::*`
  wire type.** `Kind`/`Outcome` are this module's own enums, not aliases
  of `msg::Move::StopKind` or any other generated type — `#include
  "motion/stop_condition.h"` pulls in nothing from `messages/` or
  `app/`. What happens once `tick()` reports the motion has ended
  (advance the queue, ack `Move.id`, set the timeout fault flag) is
  entirely `MoveQueue`'s job (ticket 005) — outside this module's
  boundary.
- **`theta()` is unwrapped; no modulo here.** `App::Odometry::theta()` is
  verified unwrapped (`theta_ += headingDelta`, no modulo anywhere in
  `odometry.cpp`) — the `Angle` kind diffs the caller's `theta` reading
  against its own activation baseline directly (`std::fabs(theta -
  activationTheta_)`). Adding wrap handling here would be solving a
  problem `Odometry` doesn't have.
- **Kind-specific outcome always takes precedence over timeout on a tied
  cycle.** `tick()` checks the kind-specific comparison first; `TimedOut`
  is only ever returned on a cycle where the kind-specific comparison did
  NOT also fire. This is a deliberate, tested tie-break (§4), not an
  incidental consequence of check ordering.
- **Zero/negative threshold clamps to 0, uniformly across every kind and
  timeout (sprint.md Architecture Open Question 1 — pinned here, not left
  implicit).** See §4 for the mechanism and §6 for why a uniform rule was
  chosen over a `Time`-specific carve-out.

## 4. Design

**Baseline capture.** The constructor precomputes two `[us]` deadlines
rather than storing `now` and a separate `[ms]` threshold/timeout to
convert on every `tick()` call — mirrors the deleted `App::Deadman::
arm()`'s own shape exactly (`deadline_ = clock_.nowMicros() + delta`,
`clock_.nowMicros() >= deadline_`):

- `timeDeadlineUs_ = now + millisToMicros(clampPositive(threshold))` —
  meaningful only when `kind == Kind::Time`.
- `timeoutDeadlineUs_ = now + millisToMicros(clampPositive(timeout))` —
  always meaningful, independent of kind.

`Kind::Distance`/`Kind::Angle` instead store the clamped `threshold_` as
a plain `[mm]`/`[rad]` float alongside the activation `pathLength`/`theta`
readings (`activationPathLength_`/`activationTheta_`) — no unit
conversion needed, since the caller's current readings arrive in the same
units.

**`tick()`'s comparison, in order:**

1. Compute `stopConditionMet` via a `switch` on `kind_`: `now >=
   timeDeadlineUs_` (`Time`), `std::fabs(pathLength -
   activationPathLength_) >= threshold_` (`Distance`), or
   `std::fabs(theta - activationTheta_) >= threshold_` (`Angle`).
2. If `stopConditionMet`, return `Outcome::StopConditionMet` —
   unconditionally, without even evaluating the timeout comparison's
   result. This is the tie-break: kind-specific always wins.
3. Otherwise, if `now >= timeoutDeadlineUs_`, return `Outcome::TimedOut`.
4. Otherwise, return `Outcome::Continue`.

**Zero/negative threshold mechanism.** `clampPositive(value)` is `(value
> 0.0f) ? value : 0.0f` — the same `>0` rule for threshold and timeout
both, with no per-kind special case. `NaN` comparisons are always false in
IEEE 754, so `clampPositive(NaN)` also yields `0` (defense in depth,
matching `Deadman::arm()`'s own NaN-safety posture for its `duration`
parameter). Consequence, worked through the comparison above: a clamped-
to-0 `Distance`/`Angle` threshold makes `std::fabs(delta) >= 0.0f`
trivially true from the very first `tick()` call (a magnitude can never
be negative); a clamped-to-0 `Time` threshold makes `timeDeadlineUs_ ==`
the activation `now`, so `now >= timeDeadlineUs_` is already true at or
after activation. All three kinds therefore fire `StopConditionMet` on
the very first `tick()` call when given a non-positive threshold — the
"deliberate one-cycle no-op" idiom sprint.md's Open Question 1 names,
achieved uniformly rather than by treating `Time` differently from
`Distance`/`Angle`. A clamped-to-0 `timeout` behaves the same way for
`TimedOut`, subject to the same tie-break (§3) if the kind-specific
condition is ALSO clamped to 0 that same construction.

## 5. Interfaces

### Exposes

- **`StopCondition(kind, threshold, timeout, now, pathLength, theta)`** —
  constructor; captures every baseline. `kind`: `Motion::StopCondition::
  Kind` (`Time`/`Distance`/`Angle`). `threshold`: `[ms]`/`[mm]`/`[rad]`
  depending on `kind` — Move.stop's own wire units. `timeout`: `[ms]`,
  the required safety backstop, independent of `kind`. `now`: `[us]`,
  `Devices::Clock::nowMicros()`'s own unit. `pathLength`/`theta`:
  `[mm]`/`[rad]`, the caller's `App::Odometry::pathLength()`/`theta()`
  readings AT ACTIVATION.
- **`tick(now, pathLength, theta)`** — the per-cycle comparison, `const`.
  Same units as the constructor's baseline arguments; returns
  `Motion::StopCondition::Outcome` (`Continue`/`StopConditionMet`/
  `TimedOut`).

### Consumes

Nothing — `stop_condition.h` includes only `<cstdint>`. Every reading it
needs is a plain parameter the caller (eventually `App::MoveQueue`,
ticket 005) supplies by reading its own `const Devices::Clock&` and
`App::Odometry&` collaborators and passing the results in. `Motion::`
has no `#include` of `app/`, `devices/`, or `messages/` anywhere in this
directory.

## 6. Open Questions / Known Limitations

- **Uniform clamp-to-zero vs. a `Time`-specific carve-out.** sprint.md's
  Architecture Open Question 1 left room for `time` to need `>= 0` rather
  than `> 0` "to allow a deliberate one-cycle no-op," distinct from
  `distance`/`angle`'s recommended `> 0` rule. This module resolves that
  by applying the SAME `>0, else clamp to 0` rule to every kind and to
  `timeout`, rather than special-casing `Time` — the clamp-to-0 behavior
  already produces exactly the "fires on the first `tick()` call" one-
  cycle-no-op result the carve-out was trying to preserve, without a
  second code path to keep in sync. If a future kind is added whose
  "immediate" semantics don't fall out of a bare magnitude/deadline
  comparison this cleanly, revisit whether the uniform rule still holds.
- **No clock-monotonicity guard.** `tick()`/the constructor assume `now`
  never decreases between calls (the same assumption every other
  `Devices::Clock`-driven module in this tree makes — `Clock::
  nowMicros()`'s own doc comment). A `now` that goes backward (not
  possible with the real ARM clock or `TestSim::SimClock`, which never
  self-rewinds) is out of scope here, same as it is for the deleted
  `App::Deadman`.
- **`MoveQueue`'s own construction cadence is out of this module's
  boundary.** Whether `MoveQueue` constructs a `StopCondition` on the
  stack, by value, or via some other storage strategy each time a `Move`
  activates is ticket 005's decision entirely — this module only
  documents that ITS OWN lifecycle is "one instance per activated Move,"
  not how the owner stores that instance.

---

# `Motion::VelocityShaper` (decel-into-the-goal campaign)

## 1. Purpose

`Motion::VelocityShaper` answers one question, every tick, for whichever
axis (linear or angular) `App::MoveQueue` asks it about: **given where I
am relative to the goal, how fast I'm going, and how hard I'm already
accelerating, what speed should I command NEXT?** It is the direct
follow-on to `StopCondition` above — `StopCondition` decides *when* a
`Move` has ended; this decides how the commanded speed *approaches* that
ending, so the actuation/momentum tail `StopCondition` firing exactly at
threshold-crossing still incurs is smaller by the time it fires.
Stakeholder directive #1: "a target velocity passed into some function
that gives you the next maximum speed you can assign to the wheels,"
speed dropping as the target is approached. Stakeholder directive #2
(same day): "your velocity shaper is not jerk-limited" — the commanded
acceleration itself needs its own rate limit, not just the commanded
speed. Stakeholder directive #3 (same day, scope correction): "I
literally just wanted acceleration slew rate limiting and velocity slew
rate limiting" — not a Ruckig-style profile solver. This module is the
result of all three: velocity slew-rate-limited, THEN accel slew-rate-
limited on top, nothing more elaborate.

## 2. Orientation

One class, `VelocityShaper::next(cruiseSpeed, remaining, dt, aMax, aDecel,
jMax)`, carrying two state fields across calls (`commandedSpeed_`,
`commandedAccel_`). Two chained rate clamps and an integrator, in order:

1. **Velocity clamp** (unchanged from this module's own very first pass):
   approach `cruiseSpeed` by at most `aMax*dt`, then cap the result's
   magnitude to the decel-taper ceiling `sqrt(2*aDecel*remaining)` — the
   textbook "decelerate to land exactly at `remaining==0`" curve.
2. **Accel clamp** (new): the velocity clamp's own result implies an
   acceleration this tick; slew `commandedAccel_` toward that implied
   accel by at most `jMax*dt`.
3. **Integrate** `commandedSpeed_` from the just-slewed `commandedAccel_`.

Both clamp inputs use `commandedSpeed_`/`remaining` adjusted by ONE
algebraic margin term each (not a branch, not a phase — see
`velocity_shaper.cpp`'s own comment for the exact one-line formulas):
a "predicted speed" (`commandedSpeed_` plus the velocity-domain distance
`commandedAccel_` will still cover if it decays to 0 under the jerk limit
starting now) feeds the velocity-approach clamp instead of raw
`commandedSpeed_`, and an "effective remaining" (`remaining` less the
distance the jerk-limited decel-of-decel itself consumes) feeds the
decel-taper ceiling instead of raw `remaining`. Both margins are the SAME
`x^2/(2*rate)`-shaped one-line closed form, applied in a different domain
each time — no lookahead solve, no separate phase/state machine. Without
them, a naive two-clamp implementation measurably overshoots (a bare
`test`-only build without either margin term drove `commandedSpeed_` from
0 to 350 while chasing a `cruiseSpeed=300` target during this module's own
in-tree unit tests, and reversed sign near a goal's own zero-crossing) —
the margins are required for physical correctness, not optional polish.

## 3. Constraints and Invariants

- **Stateful, host-clean — the same "no I2C, no globals, no heap" shape
  `StopCondition` established, minus statelessness.** Zero dependency on
  `App::MoveQueue`, `Motion::StopCondition`, or any `msg::*` wire type.
  State lives in the INSTANCE, not a static/global — `App::MoveQueue` owns
  one `VelocityShaper` per axis (`shaperVX_`/`shaperOmega_`/
  `shaperVLeft_`/`shaperVRight_`) so a chained/replaced Move's own ramp
  continues smoothly (SUC-051 seamless hand-off).
- **Unit-agnostic by construction.** The same class shapes a linear axis
  (`mm/s`, `mm/s^2`, `mm/s^3`) and an angular axis (`rad/s`, `rad/s^2`,
  `rad/s^3`) — the caller supplies matching units across every argument.
- **`remaining = +infinity` disables the decel taper, not a special code
  path.** `App::MoveQueue` passes `+infinity` for a `Kind::Time` Move —
  the decel-taper ceiling never binds, so the velocity-approach clamp
  alone governs the ramp-up. One code path; "no taper" is a parameter
  outcome, not a branch.
- **`reset()`/`syncTo(speed)`** — explicit state-management entry points
  `App::MoveQueue` calls at the moments raw floats used to just get
  reassigned: `reset()` zeroes BOTH fields (a genuine stop); `syncTo(speed)`
  sets `commandedSpeed_` and zeroes `commandedAccel_` (shaping disabled on
  this axis).
- **Never the terminal authority.** `VelocityShaper` never decides a
  `Move` has ended — that stays `StopCondition`'s job exclusively.

## 4. Design

See `velocity_shaper.h`'s own doc comment for the full per-parameter
contract and `velocity_shaper.cpp`'s own comment for the two-clamp
derivation and the two one-line margin terms. `App::MoveQueue`'s own
`shapeAndStage()` (`move_queue.cpp`) is the ONE caller — see that file's
own doc comment for the per-`Move`-kind axis-selection policy.

## 5. Interfaces

### Exposes

- **`VelocityShaper::next(cruiseSpeed, remaining, dt, aMax, aDecel,
  jMax)`** — instance method, mutates `commandedSpeed_`/`commandedAccel_`
  and returns the new `commandedSpeed_`. All arguments and the return
  value are plain `float`s in the caller's own chosen unit pair.
- **`VelocityShaper::reset()`**, **`VelocityShaper::syncTo(speed)`** —
  explicit state transitions, see §3 above.
- **`VelocityShaper::commandedSpeed()`**, **`VelocityShaper::
  commandedAccel()`** — const accessors `App::MoveQueue::activate()` reads
  when staging a chained Move's continuation point.

### Consumes

Nothing — `velocity_shaper.h` includes no project header beyond what
correctness needs (none). `App::MoveQueue` is the sole caller, owning one
instance per axis and supplying `aMax`/`aDecel`/`jMax` (or their angular
siblings `alphaMax`/`alphaDecel`/`yawJerkMax`) from its own live-tunable
`ShaperLimits` (`move_queue.h`, sourced fail-closed from
`Config::ShaperBootConfig` at boot, `config/boot_config.h`) and
`remaining`/`dt` computed from the SAME this-cycle `pathLength`/`theta`
`MoveQueue`'s own stop-condition comparison already reads — never a
second, independent computation (118 ticket 004: the former predicted-
pose anticipation this comment used to describe is deleted; see
`move_queue.h`'s own tick() doc comment for the land-at-zero completion
predicate that replaced it).

## 6. Open Questions / Known Limitations

- **Not a full time-optimal trajectory planner, deliberately.** Two
  chained rate clamps and an integrator, not a Ruckig-style seven-segment
  profile planned ahead of time with a known arrival time — no lookahead
  across a multi-leg path, no simultaneous multi-axis co-limiting (linear
  and angular shape independently). This is a stakeholder-set boundary,
  not an oversight — see `docs/protocol-v4.md` §5.2's own "what it is
  not" paragraph.
- **Tour-embedded turns don't reach the isolated-turn sweep's own
  optimum.** A `Move` chained via SUC-051's seamless hand-off starts its
  own ramp from whatever the PRECEDING `Move` left the shaper state at,
  not a clean from-rest start (118 ticket 004: `MoveQueue::tick()` now
  resets the just-completed Move's own shaped axis on every completion,
  not just the empty-queue drain, specifically to bound this — see
  `move_queue.cpp`'s own comment at that reset call site for why). The
  land-at-zero completion predicate's own margin constant
  (`kStoppingMarginFactor`, `move_queue.cpp`) was verified against the
  TOUR-level metric directly (not just an isolated single turn) — see
  `test_tour_closure_gate.py`'s own sweep.
- **Hardware residual.** A 2026-07-22 hardware bench session (tovez on the
  stand) measured a turn residual in roughly the same `0-8deg` band the
  earlier accel-only stage measured — the real plant's own coast-down
  dynamics, motor response, and I2C bus timing are not fully captured by
  the sim's idealized model. See
  `clasi/issues/angle-stop-overshoot-61-73-percent-on-hardware.md`'s own
  "Follow-on fix" sections for the full numbers.
