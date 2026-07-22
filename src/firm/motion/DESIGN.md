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
