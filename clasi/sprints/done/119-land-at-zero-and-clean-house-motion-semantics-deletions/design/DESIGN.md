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

**Chain-advance leg hand-off contract (119-002, VERIFIED against shipped
`move_queue.cpp` — the planning-time draft's every claim below was
re-checked line-by-line against the current tree, post-118/post-119-001,
and holds).** Moved out of §6 Open Questions: what the carried shaper
state SHOULD do at a `Move`-completion boundary is a specified contract,
not a tuned-around limitation.

- **The axis matching the ending `Move`'s own stop-condition kind is
  ALWAYS hard-reset to `(commandedSpeed=0, commandedAccel=0)` at the
  completion boundary — chain-advance or drain, unconditionally.**
  `Kind::Angle` resets `shaperOmega_`; `Kind::Distance` resets
  `shaperVX_` (`move_queue.cpp`, the unconditional reset ahead of the
  chain-advance/drain branch). This is NOT the "shaped decay from
  carry-over" this section's own Open Questions entry used to describe —
  118 ticket 003's resolution explicitly tested a conditional variant
  (skip the reset on chain-advance, let the next Move's own accel-ramp
  decay the residual naturally) against the 40ms closure gate and found
  no improvement (best worst-case 2.932°, itself just as fragile) —
  reverted, kept unconditional. **Correction to this issue's own
  proposed-fix text**, which speculated "current: shaped decay from
  carry-over" — that was accurate pre-118; the shipped, tested, and kept
  behavior is unconditional reset. Rationale: a `Move` can end with a
  nonzero residual `commandedSpeed_` (both the threshold backstop and the
  land-at-zero predicate tolerate an imperfect-zero taper); without the
  reset, that residual leaks into whichever LATER `Move` next uses the
  SAME axis and corrupts ITS land-at-zero `remaining` computation with a
  value describing the wrong `Move`. **Verification note (119-002): this
  unconditional reset has a real cost the original issue/draft did not
  name — it fires even when the INCOMING chained `Move` commands the
  SAME axis (e.g. two consecutive `Distance` legs at the same `v_max`),
  defeating SUC-051 continuity for exactly that compatible-boundary case
  instead of only the alternating-axis case this paragraph otherwise
  discusses.** Reproduced against
  `test_two_compatible_distance_legs_carry_velocity_through_the_boundary_at_tour_level`
  (`src/tests/testgui/test_tour_closure_gate.py`): a genuine ~149→24mm/s
  dip-and-reramp at a same-axis `Distance`→`Distance` boundary, latent
  since 118 ticket 003 (whose own re-sweep only exercised TOUR_1/TOUR_2,
  which always alternate axes) and only now visible because 119 ticket
  001 made the shaper-limits push default-on for this test's own
  `SimLoop` session (previously silently off, so no taper/reset ever
  ran). Fixing the reset condition is a real, separate change — out of
  this ticket's own scope (it would need its own re-sweep of
  `kStoppingMarginFactorChain`/`kDiscretizationCyclesChain` against the
  already-narrow chain-margin pocket) — filed as
  `chain-advance-reset-defeats-same-axis-compatible-leg-continuity.md`;
  the boundary test's own `xfail` is re-pointed at it (§ below).
- **The axis NOT matching the ending `Move`'s stop-condition kind is
  UNTOUCHED at a chain-advance boundary — SUC-051's own continuity
  property, unchanged.** If the next `Move` commands that axis, it ramps
  from wherever `commandedSpeed()` already was — genuine continuity, not
  a from-rest restart. Only a full drain (`pendingCount() == 0`) resets
  ALL FOUR shapers (`shaperVX_`/`shaperOmega_`/`shaperVLeft_`/
  `shaperVRight_`) to `(0, 0)`, since the robot has genuinely stopped and
  the NEXT unrelated `Move` (whenever it activates) must not inherit a
  stale nonzero pair from a taper that never finished (e.g. a
  timeout-backstop ending mid-taper).
- **Sign reversal does not survive a boundary, by construction — not a
  separate case to specify.** Because the completing axis's shaper is
  unconditionally reset to `(0, 0)` (above), there is no carried nonzero
  speed for a reversal to "survive" in the first place; the shaper-level
  question the original issue posed is subsumed by the unconditional-
  reset rule. The remaining, genuinely separate question was the
  HARDWARE-level asymmetry: `NezhaMotor`'s 100ms `reversal_dwell_ms`
  arms on the reversing wheel only at a D→RT boundary (asymmetric by
  construction, `nezha_motor.cpp`) — 118 did not touch this.
  **Decision (119-002): ACCEPT the asymmetric per-wheel dwell — do not
  symmetrize.** Measured on the current tree (`data/robots/
  tovez_nocal.json`, ideal chip — no injected sensor error, isolating
  the mechanism itself — 90° turns, `test_tour_closure_gate.py`'s own
  `_run_tour_capture()`/sim-ground-truth reading, deterministic
  stepping): a genuine `D(300mm@150mm/s)`→`RT(90°)` chain-advance turn
  (crossing the reversal dwell on one wheel, the exact boundary this
  bullet is about) measured **-1.18° error** — no worse than an isolated
  (from-rest) chain-advance 90° turn measured under the SAME margin
  regime (`+2.90°` first turn / `+1.60°` once warmed up, two consecutive
  turns with no preceding `Distance` leg) — the dwell asymmetry itself
  does not measurably cost extra heading accuracy in a controlled,
  same-regime comparison (this supersedes the pre-execution turn-
  execution review's own "0.3° vs 1.4-1.7°" isolated/tour figures,
  `docs/code_review/2026-07-22-turn-execution-review.md` D5, which mixed
  the drain/ack-instant-vs-settle read convention across its two
  numbers rather than holding the completion regime fixed — this
  ticket's own comparison holds pendingCount()>0 (chain margin) fixed on
  both sides instead). Both figures also stay comfortably inside the
  project's own already-shipped, stakeholder-approved acceptance bands
  that already cover exactly this isolated-vs-tour-embedded distinction:
  `MANAGED_ANGLE_90_ABS_MARGIN_DEG=3.0°` (isolated single-preset turn,
  `test_gui_button_acceptance.py`, real GUI-driven) vs
  `TOUR_TURN_ERROR_MAX_DEG=5.0°` (chain-advance/tour-embedded turn,
  worst of 13 real-GUI-driven runs measured 3.86°, that file's own
  2026-07-22 sweep) — that file's own comment attributes the width of
  THAT gap primarily to real background-tick-thread scheduling jitter at
  the ack-instant read, not to the dwell mechanism itself. Symmetric
  dwell (holding BOTH wheels at commanded-zero through the reversal
  window, not just the reversing one) would cost real per-cycle margin
  on the axis that does not need to reverse, for no demonstrated
  accuracy benefit — rejected.
- **vExit design reference
  (`simple-velocity-control-acceleration-limited-shaper.md`) — adopted,
  in the sense the shipped mechanism already matches its "0 on reversal
  or empty queue" half exactly** (the unconditional completing-axis reset
  above IS vExit=0, applied unconditionally rather than only on reversal/
  empty-queue, which is a strictly more conservative special case of the
  same rule). Its "ramp from next move's cruise" half describes the
  SURVIVING axis's SUC-051 continuity, not the completing axis. No
  separate vExit mechanism needs implementing — the existing reset +
  continuity split already realizes it.
- **Axis-drop coast at chain boundaries — the mechanism
  `chain-advance-completion-margin-narrow-pocket.md` (filed 2026-07-23
  from 118 ticket 003's resolution) traces the chain-advance completion
  margin's narrow accuracy pocket to.** Tours alternate Distance/Angle
  legs, so a chain-advance turn's own axis (`omega`, say) is exactly the
  axis the NEXT `Move` does not command — it is the completing-and-reset
  axis above, not a surviving one. Completion is scored at the ack
  instant (the cycle `landAtZero()`/the threshold fires), but the plant's
  own physical coast on that now-reset-to-zero-command axis is only
  PARTIALLY visible by that instant — the reset zeroes the COMMAND, not
  the plant's own residual angular/linear velocity, which continues to
  decay physically for a few more cycles the ack-instant score never
  observes. This is the concrete "axis-drop coast" this contract names:
  the gap between "commanded axis reset to zero" (this cycle) and
  "plant physically at rest on that axis" (a few cycles later, unscored
  by the chain-advance ack-instant metric). `kStoppingMarginFactorChain`/
  `kDiscretizationCyclesChain` (`move_queue.cpp`) are the swept
  compensations for exactly this gap — this paragraph specifies WHY they
  differ from the final-move case (which scores against a
  settle-consistent, not ack-instant, completion), not a new mechanism to
  implement. Closing the narrow pocket itself (rather than just naming
  its mechanism) is out of this ticket's own scope — see the pool issue's
  own "not urgent... future sprint" disposition.

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
- ~~Tour-embedded turns don't reach the isolated-turn sweep's own
  optimum~~ — **RESOLVED, moved to §4 Design (119-002, "Chain-advance leg
  hand-off contract")**: what the carried/reset shaper state does at a
  completion boundary is now a specified, verified contract (unconditional
  completing-axis reset, untouched surviving axis, vExit-equivalent
  reversal handling, named axis-drop-coast mechanism for the
  chain-advance margin's own narrow pocket). The D→RT `reversal_dwell_ms`
  hardware asymmetry's accept-vs-symmetrize decision is ALSO resolved
  there — accepted, measured (-1.18° for a genuine D→RT chain-advance
  turn, no worse than an isolated chain-advance turn under the same
  margin regime) and within the project's already-shipped tour-turn
  acceptance bands. Nothing left open in this section for the hand-off
  contract; 119-002's own verification pass DID surface one adjacent,
  genuinely open item the original draft did not name — the same
  unconditional reset also defeats carried-velocity continuity for a
  same-axis COMPATIBLE chain boundary (e.g. two consecutive `Distance`
  legs), not just the alternating-axis case this contract otherwise
  covers — filed as
  `chain-advance-reset-defeats-same-axis-compatible-leg-continuity.md`,
  out of this ticket's own scope (see §4's own note on it).
- **Hardware residual.** A 2026-07-22 hardware bench session (tovez on the
  stand) measured a turn residual in roughly the same `0-8deg` band the
  earlier accel-only stage measured — the real plant's own coast-down
  dynamics, motor response, and I2C bus timing are not fully captured by
  the sim's idealized model. See
  `clasi/issues/angle-stop-overshoot-61-73-percent-on-hardware.md`'s own
  "Follow-on fix" sections for the full numbers.
