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
am relative to the goal and how fast I'm going, what speed should I
command NEXT?** It is the direct follow-on to `StopCondition` above —
`StopCondition` decides *when* a `Move` has ended; this decides how the
commanded speed *approaches* that ending, so the actuation/momentum tail
`StopCondition` firing exactly at threshold-crossing still incurs is
smaller by the time it fires. Stakeholder directive: "a target velocity
passed into some function that gives you the next maximum speed you can
assign to the wheels," speed dropping as the target is approached —
follow-on to `clasi/issues/angle-stop-overshoot-61-73-percent-on-hardware.md`'s
own "Option 1... remains the path to closing that residual further."

## 2. Orientation

One class, one static method: `VelocityShaper::next(cruiseSpeed,
currentSpeed, remaining, dt, aMax, aDecel)` — a pure function, no
instance, no state of its own (unlike `StopCondition`, which captures a
baseline at construction; this module has no baseline to capture, every
call is self-contained). The formula:

```
next = min(cruiseSpeed-approached-by-aMax*dt, sqrt(2*aDecel*max(remaining,0)))
```

computed as an "approach cruise, then clamp the result's magnitude to the
decel-taper ceiling" two-step (see `velocity_shaper.cpp`'s own comment for
why this is a strict generalization of the stakeholder's own literal
three-way-`min()` formula, correct — not just convenient — when
`currentSpeed`/`cruiseSpeed` carry different signs or `dt`/`remaining` are
degenerate).

## 3. Constraints and Invariants

- **Stateless, pure, host-clean — the same "no I2C, no globals, no heap"
  shape `StopCondition` established.** Zero dependency on `App::MoveQueue`,
  `Motion::StopCondition`, or any `msg::*` wire type — `velocity_shaper.h`
  includes nothing beyond what its own signature needs.
- **Unit-agnostic by construction.** The same function shapes a linear
  axis (`mm/s`, `mm/s^2`) and an angular axis (`rad/s`, `rad/s^2`) — the
  caller supplies matching units on both sides of each argument pair; the
  module itself has no unit opinion.
- **`remaining = +infinity` disables the decel taper, not a special code
  path.** `App::MoveQueue` passes `+infinity` for a `Kind::Time` Move (no
  position-based "remaining" exists for an elapsed-wall-clock stop) —
  `sqrt(2*aDecel*remaining)` diverges and the clamp never binds, so the
  accel-ramp clamp alone governs. One formula, one code path; "no taper"
  is a parameter choice.
- **Never overshoots `cruiseSpeed`, and gracefully ramps DOWN if
  `currentSpeed` is already past `cruiseSpeed`** (e.g. a live config
  change lowered the ceiling mid-`Move`) — the accel-ramp step is a
  bidirectional "approach," not a one-directional `current + aMax*dt`
  add, which the stakeholder's own literal formula does not by itself
  resolve for that case (see `.cpp`'s own comment).
- **Never the terminal authority.** `VelocityShaper` never decides a
  `Move` has ended — that stays `StopCondition`'s job exclusively, unfazed
  by whatever `VelocityShaper` shaped `App::Drive`'s target to this tick.

## 4. Design

See `velocity_shaper.h`'s own doc comment for the full per-parameter
contract and `velocity_shaper.cpp`'s own comment for the two-step
approach-then-clamp derivation and its equivalence to the stakeholder's
literal formula in the regime it was written for. `App::MoveQueue`'s own
`shapeAndStage()` (`move_queue.cpp`) is the ONE caller — see that file's
own doc comment for the per-`Move`-kind axis-selection policy (which
component of a `Move`'s velocity gets shaped, and what `remaining` means
per `Motion::StopCondition::Kind`).

## 5. Interfaces

### Exposes

- **`VelocityShaper::next(cruiseSpeed, currentSpeed, remaining, dt, aMax,
  aDecel)`** — static, pure. All six arguments and the return value are
  plain `float`s in the caller's own chosen unit pair (linear or
  angular). See `velocity_shaper.h` for the exact clamp/sign contract of
  each.

### Consumes

Nothing — `velocity_shaper.h` includes no project header beyond what
correctness needs (none). `App::MoveQueue` is the sole caller, supplying
`aMax`/`aDecel` from its own live-tunable `ShaperLimits` (`move_queue.h`,
sourced fail-closed from `Config::ShaperBootConfig` at boot,
`config/boot_config.h`) and `remaining`/`dt` computed from the SAME
predicted pose `MoveQueue`'s own stop-condition anticipation already
reads (`move_queue.h`'s own tick() doc comment) — never a second,
independent prediction.

## 6. Open Questions / Known Limitations

- **Not a jerk-limited profile.** `aMax`/`aDecel` bound acceleration
  magnitude, not its rate of change — a real trapezoidal/S-curve motion
  profile planned ahead of time with a known arrival time is NOT what
  this module does; it is a per-cycle, closed-form REACTIVE law. See
  `docs/protocol-v4.md` §5.2's own "what it is not" paragraph.
- **Tour-embedded turns don't reach the isolated-turn sweep's own
  optimum.** A `Move` chained via SUC-051's seamless hand-off starts its
  own ramp from whatever the PRECEDING `Move` left the shaped-speed state
  at, not a clean from-rest start — measured (sim) at roughly 2.0-2.4deg
  worst-case at TOUR level vs. ~0.3deg for an isolated single turn from
  rest (`test_tour_closure_gate.py`'s own sweep,
  `src/tests/notebooks/turn_prediction.ipynb` Section 9). Not further
  decomposed this campaign.
- **Hardware residual larger than sim predicted.** A 2026-07-22 hardware
  bench session (tovez on the stand) measured a ~4-8deg turn residual
  with the taper active — in the same ballpark as the PRE-taper
  anticipation-lead-only result, not the ~50% further improvement sim
  measured. The real plant's own coast-down dynamics, motor response, and
  I2C bus timing are not fully captured by the sim's idealized model. See
  `clasi/issues/angle-stop-overshoot-61-73-percent-on-hardware.md`'s own
  "Follow-on fix" section for the full numbers.
