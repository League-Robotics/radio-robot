---
id: "002"
title: "Motor velocity-loop / armor fix (root fix, phase 1) + armor regression re-verification"
status: open
use-cases: [SUC-001, SUC-002, SUC-004]
depends-on: ["001"]
github-issue: ""
issue: motion-turn-drive-terminal-overshoot.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Motor velocity-loop / armor fix (root fix, phase 1) + armor regression re-verification

## Description

**This is the sprint's highest-risk, highest-priority ticket** — the root
fix for the turn/drive terminal reverse-spin overshoot, and the ticket that
must NOT weaken the reversal-latch/wedge-detection safety armor (`Hal::Motor`,
sprints 078/079). Fix order is stakeholder-mandated: this ticket (the motor
loop, the actual root cause per the architecture doc's Grounding fact 2)
lands BEFORE ticket 003 (Planner decel anticipation, a mitigant only).

**Root cause** (architecture-update.md Grounding fact 2): `Hal::
MotorVelocityPid::compute()`'s integrator-freeze deadband gates on
`|target| < minDuty`, not `|error|` — once the ramp's target nears zero the
integrator freezes at whatever value it held while sustaining the prior
motion, while the proportional term (`kp * err`, with `err` still large
because the physical wheel is still spinning) can swing the raw duty to the
opposite sign. `Hal::Motor::armoredWrite()`'s zero-dwell-reversal gate
(`reversalDwell_`, default 100 ms) then catches that sign flip — but
catching it here means holding the wheel at commanded-zero (free coast) for
the whole dwell window while the deadbanded PID keeps recomputing a fresh,
undamped correction from the ever-growing coast error, so the correction
that finally lands once the dwell elapses is LARGER, not smaller, than if no
dwell had intervened.

**Bound by two invariants (architecture-update.md Design Rationale 1) —
non-negotiable, regardless of implementation mechanism:**

- **Invariant A**: a genuine unrequested reversal (e.g. a stale/glitched
  command re-issuing the opposite sign with no intervening commanded
  decel-to-zero) is still caught and dwelled exactly as today — no
  weakening of `reversalDwell_`'s protection against that case.
- **Invariant B**: a legitimate braking correction during a commanded
  decel-to-zero is not held off long enough to let the wheel coast further
  into overshoot territory.

**The exact mechanism is this ticket's own implementation decision** —
candidates include (not prescribed, not exclusive): retuning the PID's
deadband condition to gate on something other than bare `|target|`; adding
a bounded braking-duty cap specifically near a commanded-zero target;
adjusting `armoredWrite()`'s reversal test to distinguish a "braking toward
zero" transition from a genuine direction reversal; or a combination. Any
mechanism is acceptable as long as it demonstrably satisfies BOTH invariants
via the tests below.

**If no mechanism you try satisfies both invariants simultaneously**, this
is an architecture-level conflict, not an implementation bug to push
through — call `throw_ticket_exception` (`thrown_by="programmer"`,
`surface="internal"`) describing what was attempted and which invariant
could not be held, rather than silently relaxing either one. Do not ship a
"mostly fixed" version that trades one invariant for the other.

## Acceptance Criteria

- [ ] Ticket 001's regression test(s) for `RT 9000` and `D 200 200 500` now
      pass (residual post-completion velocity within the tight bound;
      distance overshoot materially reduced).
- [ ] **Invariant A test**: a genuine unrequested reversal (e.g. two
      opposite-sign `DEV M`/velocity commands issued back to back with no
      intervening decel-to-zero) is still caught by `reversalDwell_` exactly
      as before this ticket — same dwell behavior, same timing.
- [ ] **Invariant B test**: a commanded decel-to-zero (turn/drive stop)
      does not produce a held-at-zero coast period long enough to worsen the
      overshoot — i.e., the fix measurably improves on ticket 001's captured
      "before" baseline, not just moves the problem.
- [ ] **100% of pre-existing `tests/sim/unit/test_motor_policy.py` cases
      pass unmodified** (or, if any fixture value must change, the change is
      justified explicitly in this ticket's completion notes, not silently
      loosened).
- [ ] `tests/sim/unit/test_velocity_pid.py` / `test_velocity_pid_response.py`
      / `test_stiction_and_motor_lag.py` all still pass.
- [ ] `wedged()`/`wedgeSuspect()` semantics (the raw stuck-encoder latch and
      its motion-qualified variant) are unchanged — this ticket touches
      `armoredWrite()`'s reversal-detection only, never
      `updateWedgeDetector()`/`processResetIfPending()`.
- [ ] No change to `Hal::MotorVelocityPid::compute()`'s or
      `armoredWrite()`'s public signatures.
- [ ] Sim-only acceptance for this ticket — the stand HITL pass validating
      the complete (002+003) fix is ticket 004's job, not this ticket's.

## Implementation Plan

**Approach**: Start from ticket 001's failing regression tests. Instrument
`velocity_pid.cpp`'s `compute()` and `capability/motor.h`'s `armoredWrite()`
together (they interact, per Grounding fact 2) rather than changing one in
isolation. Prototype against the two invariant tests FIRST (write them
before attempting a fix), then iterate the mechanism until both are green
alongside the full existing armor suite.

**Files to modify**:
- `source/hal/velocity_pid.{h,cpp}` — control-law/deadband change, if the
  chosen mechanism touches it.
- `source/hal/capability/motor.h` — `armoredWrite()`'s reversal-detection
  logic, if the chosen mechanism touches it. `processResetIfPending()`/
  `updateRestTracking()`/`updateWedgeDetector()` must NOT be touched.
- No changes to `source/hal/nezha/nezha_motor.{h,cpp}` expected (the base/
  leaf split means the fix lives in shared base/PID code); if the
  implementation genuinely needs a leaf-side change, document why in this
  ticket's completion notes.

**Testing plan**:
- Extend `tests/sim/unit/test_motor_policy.py` with the Invariant A and
  Invariant B cases described above.
- Re-run the full `tests/sim/unit/test_motor_policy.py`,
  `test_motor_policy_harness.cpp`-backed suite, `test_velocity_pid*.py`,
  `test_stiction_and_motor_lag.py` unmodified — all must stay green.
- Re-run ticket 001's regression tests — both must now pass.
- Dual-caller check: confirm the fix, since it lives in shared `Hal`/
  `Motion`-adjacent code with no per-caller wiring, requires no separate
  `source/main.cpp` vs. `tests/_infra/sim/sim_api.cpp` changes (both already
  go through the same `Hal::Motor` base).

**Documentation updates**: None required at the wire/protocol level (no
verb/schema change). If the chosen mechanism is non-obvious, add a doc
comment at the fix site explaining the invariant it preserves, mirroring
the existing style of `armoredWrite()`'s own state-diagram comment.
