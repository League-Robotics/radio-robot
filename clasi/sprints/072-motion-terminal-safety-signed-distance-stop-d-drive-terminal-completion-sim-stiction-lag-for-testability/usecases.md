---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 072 Use Cases

Two issues drive this sprint (`clasi/issues/distance-stop-fabsf-accepts-backward-completion.md`,
`clasi/issues/d-drive-terminal-instability-reversal-thrash.md`), plus a
testability prerequisite the second issue explicitly calls out (the sim plant
is zero-lag/zero-stiction and cannot reproduce "lands short of target," per
sprint 069 architecture-update.md's own Open Question 3, which deferred
exactly this). SUC-001 covers the testability prerequisite; SUC-002/003 cover
issue 1 (the safety defect); SUC-004 covers issue 2 (the reliability defect).
Ticket 004 (regression sweep) verifies SUC-002/003/004 hold under the full
existing test suite and mints no new use case of its own.

SUC-001 narrows **UC-020: Configure Simulator Plant/Error Model at Runtime**
— proposed but not yet consolidated into `docs/usecases.md` (sprint 069
proposed it and flagged the same open question 069 itself flagged: confirm
at consolidation time). This sprint adds a fourth `PhysicsWorld` dynamics
knob family (stiction/breakaway, optionally response lag) to the same
UC-020 surface 069 built (`SIMSET`/`SIMGET`), so narrowing the same proposed
UC rather than minting a second one keeps the sim-plant-configuration use
case singular pending that consolidation decision.

SUC-002/003/004 narrow **UC-003: Drive Robot a Specific Distance** — the
existing top-level use case for the `D` command. UC-003's current text in
`docs/usecases.md` still describes the retired `MotorController.startDriveClean`
ratio-PID path (superseded by `beginDistance`/`MotionCommand`/`StopCondition`
well before this sprint); this sprint's SUCs describe the *current*
architecture's D-command termination behavior and its defects, and the
UC-003 text itself is stale independent of this sprint — flagged in Open
Questions for the next consolidation pass, not fixed here (out of this
sprint's scope).

---

## SUC-001: Configure Simulated Motor Stiction/Breakaway (and Optional Response Lag) at Runtime
Parent: UC-020 (proposed, pending consolidation — see note above)

- **Actor**: Developer / test tooling (Python host, via the sim's `SIMSET`/`SIMGET`
  wire surface established in sprint 069)
- **Preconditions**: Sim is running (`Sim()` / `SimTransport`). `PhysicsWorld`'s
  chassis integration is the current zero-lag, zero-stiction algebraic model
  (`vel = pwm/100 * nominalMaxSpeed * offsetFactor`, applied identically
  regardless of how small `pwm` is or whether the wheel was previously at
  rest).
- **Main Flow**:
  1. Developer sends `SIMSET stictionPwmL=<v> stictionPwmR=<v>` (per-wheel
     breakaway threshold, PWM units 0-100) and, optionally, `motorLagMsL=<v>
     motorLagMsR=<v>` (per-wheel first-order response time constant, ms).
  2. `SimCommands` dispatches to new `PhysicsWorld` setters (following the
     069/071 `simsetters::` free-function pattern: one canonical function per
     knob, shared between `SimCommands.cpp`'s registry and
     `tests/_infra/sim/sim_api.cpp`'s ctypes forwards).
  3. On the next `update(dt)`, a commanded `|pwm| < stictionPwmSide` produces
     **zero** velocity for that wheel this tick (the wheel does not break
     away), regardless of the wheel's velocity on the previous tick; once
     `|pwm| >= stictionPwmSide`, velocity follows the existing algebraic
     model (subject to the optional lag filter).
  4. `SIMGET stictionPwmL` (etc.) reads back the configured value.
- **Postconditions**: A drive that commands a very small PWM near the end of
  a decel ramp (exactly the regime `d-drive-terminal-instability-reversal-thrash.md`
  describes) can now genuinely fail to reach its commanded distance target in
  sim — a physical failure mode that was previously unreproducible except by
  an artificial forced-encoder-cap test harness that bypasses the plant
  entirely.
- **Acceptance Criteria**:
  - [ ] `stictionPwmL`/`stictionPwmR` (and, if implemented, `motorLagMsL`/
        `motorLagMsR`) are `SIMSET`/`SIMGET`-able.
  - [ ] Default value (0) is a no-op: every existing test that never calls
        these setters observes byte-identical `PhysicsWorld::update()` output
        (golden-TLM canary unaffected).
  - [ ] With `stictionPwmL/R` configured above the terminal-decel PWM a `D`
        drive would otherwise command near its target, a scripted `D` drive
        in sim measurably lands short of the target distance (the repro
        vehicle ticket 002/003's fixes are validated against).
  - [ ] The existing forced-encoder-cap sim harness (used to diagnose the
        issue against real firmware code) is not removed or broken by this
        change — it remains available as an independent diagnostic tool.

---

## SUC-002: Distance and Rotation Drives Stop Only on Commanded-Direction Travel
Parent: UC-003 (narrows)

- **Actor**: Python host issuing a `D` (distance) or `RT` (relative-turn)
  motion command; indirectly, anyone operating the robot (playfield safety).
- **Preconditions**: A `D <leftSpeed> <rightSpeed> <distanceMm>` or `RT
  <centideg>` command is active. `StopCondition::Kind::DISTANCE` currently
  fires on `fabsf(traveled) >= target` regardless of the sign of `traveled`
  relative to the commanded direction; `Kind::ROTATION` has the same
  `fabsf(diff)` pattern.
- **Main Flow**:
  1. `beginDistance()`/`beginRotation()` capture the commanded direction sign
     (from the commanded body velocity `v` for DISTANCE; from the commanded
     yaw rate `omega` for ROTATION) into the `MotionCommand`'s
     `MotionBaseline` snapshot at `start()`, alongside the existing
     `enc0`/`encDiff0` baseline fields.
  2. Each tick, `StopCondition::evaluate()` computes the signed delta
     (`traveled` for DISTANCE, `diff` for ROTATION) and gates completion on
     `signedDelta >= target`, where `signedDelta = rawDelta * commandedSign`
     — not `fabsf(rawDelta) >= target`.
  3. A drive that travels in the commanded direction completes exactly as
     before (this is a no-op for the common case; `signedDelta == |rawDelta|`
     when the robot moves the way it was told to).
  4. A drive that runs away in the OPPOSITE direction (backward on a forward
     `D`, or the wrong spin direction on an `RT`) never satisfies
     `signedDelta >= target` from wrong-direction travel alone — the DISTANCE/
     ROTATION stop does not fire for it.
  5. A reverse-commanded drive (`D -200 -200 500`) still completes normally
     on backward travel: `commandedSign = -1`, `rawDelta` goes negative as the
     robot backs up, `signedDelta = rawDelta * -1` grows positive and crosses
     `target`.
- **Postconditions**: `EVT done D reason=dist` (or `EVT done <verb> reason=rot`)
  is emitted if and only if the robot traveled the commanded magnitude in the
  commanded direction. A runaway in the wrong direction no longer self-reports
  as a successful completion.
- **Acceptance Criteria**:
  - [ ] A forward `D` (`D 200 200 500`) whose encoders instead accumulate
        500 mm of BACKWARD travel does NOT fire the DISTANCE stop from that
        backward travel (does not emit `EVT done D reason=dist`).
  - [ ] A reverse `D` (`D -200 -200 500`) that travels 500 mm backward DOES
        fire the DISTANCE stop and completes normally (no regression on the
        legitimate reverse-drive case).
  - [ ] `RT <cdeg>` in each direction (positive and negative) still terminates
        on its own commanded-direction arc; a wrong-direction encoder
        differential does not satisfy the ROTATION stop.
  - [ ] The existing `test_distance_fires_for_reverse` test
        (`tests/simulation/unit/test_stop_condition.py`), which currently
        asserts that ANY 200 mm of encoder travel fires a DISTANCE(200) stop
        with no notion of commanded direction, is split into (a) a
        commanded-reverse case that still fires and (b) a new
        commanded-forward-but-travels-backward case that must NOT fire —
        documenting the before/after behavior change explicitly.

---

## SUC-003: Distance Drive Aborts via a Wire-Visible Safety Stop on Runaway Reversal
Parent: UC-003 (narrows)

- **Actor**: Python host / playfield safety monitor.
- **Preconditions**: A forward `D` command is active. SUC-002's signed
  DISTANCE stop is in place (a wrong-direction runaway no longer
  self-reports success), but by itself it only stops FALSE completions — it
  does not, on its own, cut power to a robot that is actively moving away
  from its commanded direction. Today the only backstop for this scenario is
  the generous TIME net (2x nominal travel time + 2 s), which can let a
  runaway continue for several seconds before it fires.
- **Main Flow**:
  1. `beginDistance()` additionally installs a margin-based safety stop
     alongside the existing DISTANCE and TIME stops (a new
     `StopCondition::Kind`, e.g. `SAFETY_MARGIN`, sharing the same signed-delta
     computation as SUC-002's DISTANCE fix).
  2. Each tick, if the signed traveled distance goes more than a configured
     margin NEGATIVE relative to the commanded direction (the robot has
     demonstrably moved backward by more than the margin during a forward
     `D`), the safety condition fires.
  3. `MotionCommand::tick()` recognizes this stop kind as a safety-class
     termination: it forces an immediate (HARD-style) teardown regardless of
     the command's configured SOFT stop style, and emits `EVT safety_stop`
     (reusing the existing wire-visible `EVT safety_stop` label the
     keepalive watchdog already uses) with an additive `reason=` token,
     instead of the command's configured `EVT done D`.
- **Postconditions**: A robot running away backward during a forward `D` is
  cut to zero power within one margin-crossing tick and the host receives an
  unambiguous, distinctly-named terminal event — it does not have to wait for
  the TIME net, and it cannot mistake the outcome for a successful `EVT done
  D`.
- **Acceptance Criteria**:
  - [ ] A forward `D` whose encoders accumulate backward travel past the
        configured safety margin aborts within one control tick of crossing
        the margin (not the multi-second TIME net) and emits `EVT
        safety_stop` (not `EVT done D`).
  - [ ] The abort is a HARD stop (zero PWM immediately), not a SOFT ramp that
        continues traveling backward for up to 3 s while decelerating.
  - [ ] `EVT safety_stop`'s wire shape remains compatible with existing hosts
        that already recognize it from the keepalive-watchdog path (additive
        `reason=` token only; no change to the base label).
  - [ ] The safety margin is configurable (a new `RobotConfig`/`SET`-able
        field, not a hardcoded constant), consistent with 067's live-SET
        propagation rule.

---

## SUC-004: Distance Drive Completes Reliably Despite Motor Stiction Near the Target
Parent: UC-003 (narrows)

- **Actor**: Python host issuing a `D` command; indirectly, anyone relying on
  a `D` drive's commanded distance being accurate (path-following, playfield
  maneuvers built from `D` primitives).
- **Preconditions**: SUC-001's stiction plant model exists (the test vehicle).
  The D-mode decel hook (`Planner::driveAdvance`, DISTANCE branch) caps
  commanded speed at `v_cap = sqrt(2 * aDecel * d_remaining)`, asymptotically
  reaching exactly zero AT the target — a profile shape that assumes the
  plant can track arbitrarily small commanded speeds. The down-only ratchet
  (`if (v_cap < targetV) setTarget(v_cap, ...)`) never raises the target back
  up once it has been lowered. `VelocityController::update()` freezes its
  integrator below `minWheelSpeed`. Combined, a real (or now sim-modeled
  stiction) motor's terminal PWM can fall inside the breakaway dead zone
  before the encoder crosses the DISTANCE target, stalling the robot 1-3 mm
  short with no guaranteed path back to completion except the multi-second
  TIME net — during which the observed failure mode is windup-driven reversal
  and thrash, not a graceful recovery.
- **Main Flow**:
  1. The D-mode decel hook gains a terminal `v_cap` floor: within a final
     approach zone (`d_remaining` at or below a new small threshold), `v_cap`
     is floored at `minWheelSpeed` rather than allowed to asymptote toward
     zero, so the commanded speed never itself falls into the controller's
     own deadband/near-stiction regime purely as an artifact of the profile
     shape.
  2. Independent of (1), the decel hook tracks whether `d_remaining` is
     making progress once it is within a new arrive-tolerance band. If
     `d_remaining` stays inside the tolerance band and stops shrinking for a
     stall-confirm window (the wheels are stalled short, not still
     decelerating normally), the D drive completes NOW via a new
     `MotionCommand` entry point that forces the same termination path a
     normal DISTANCE-stop completion takes (SOFT ramp-down from a near-zero
     speed, `EVT done D`), tagged with an additive `reason=` token
     distinguishing it from a strict-crossing completion.
  3. This "stalled-short-completes" path is deliberately chosen over letting
     the down-only ratchet re-approach after a retreat (rejected alternative
     — re-approaching risks reproducing the observed thrash rather than
     resolving it; see architecture-update.md Design Rationale).
- **Postconditions**: A `D` drive against a stiction-modeled plant lands
  within the configured arrive tolerance of its target, at rest, without
  reversing or thrashing, and without needing the TIME net to terminate it.
  A small, bounded, intentional under-travel (up to the tolerance) is an
  accepted trade-off, not a defect.
- **Acceptance Criteria**:
  - [ ] Against SUC-001's stiction plant configured to reproduce the field
        failure signature (lands 1-3 mm short at near-zero commanded speed),
        a `D 200 200 500` drive completes within the configured arrive
        tolerance of 500 mm, at rest, with no backward travel and no thrash.
  - [ ] The drive completes well before the TIME net would fire.
  - [ ] `EVT done D` is still emitted on both a strict-crossing completion
        (`reason=dist`, unchanged) and a stalled-short completion (a new,
        additive `reason=` token) — hosts that only check for `EVT done D`
        (ignoring `reason=`) see no behavior change.
  - [ ] A `D` drive against the ORIGINAL zero-stiction plant (no `SIMSET`
        stiction knobs configured) behaves identically to before this sprint
        — the new terminal-completion path is not a no-op that happens to
        never fire in the historically-tested zero-lag environment; it is
        provably inert there because `d_remaining` reaches exactly zero via
        the strict crossing before the stall-confirm window could elapse.

---

## Open Questions Carried to Architecture

1. UC-020's own top-level minting/narrowing question (raised by sprint 069,
   still pending) is not re-litigated here; SUC-001 narrows the same
   proposed UC-020 069 proposed, deferring the mint-vs-narrow decision to the
   same future consolidation pass.
2. UC-003's stale prose in `docs/usecases.md` (describing the retired
   `MotorController.startDriveClean` path) is a pre-existing drift issue,
   independent of this sprint's changes, and is out of scope to fix here —
   flagged for the next consolidation pass.
3. Sprint 071's architecture-update.md named "sprint 072" as the recommended
   destination for the host-Python half of the identifier-unit-rename split
   it deferred (Decision 1). This sprint number has since been assigned to
   this motion-safety work instead; that rename-split work has no home and
   should be re-slotted into a future sprint by the team-lead/roadmap.
