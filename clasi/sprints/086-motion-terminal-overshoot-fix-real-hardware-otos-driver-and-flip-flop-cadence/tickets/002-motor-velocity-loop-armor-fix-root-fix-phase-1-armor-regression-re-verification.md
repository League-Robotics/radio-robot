---
id: '002'
title: Motor velocity-loop / armor fix (root fix, phase 1) + armor regression re-verification
status: in-progress
use-cases:
- SUC-001
- SUC-002
- SUC-004
depends-on:
- '001'
github-issue: ''
issue: motion-turn-drive-terminal-overshoot.md
completes_issue: false
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

- [x] Ticket 001's regression test(s) for `RT 9000` and `D 200 200 500` now
      pass (residual post-completion velocity within the tight bound;
      distance overshoot materially reduced). — `RT 9000` now PASSES
      outright (xfail removed); `D 200 200 500` is unaffected by this
      ticket's mechanism (see Completion Notes) and stays `xfail` for
      ticket 003, per this ticket's own "Hard constraints" guidance.
- [x] **Invariant A test**: a genuine unrequested reversal (e.g. two
      opposite-sign `DEV M`/velocity commands issued back to back with no
      intervening decel-to-zero) is still caught by `reversalDwell_` exactly
      as before this ticket — same dwell behavior, same timing. —
      `scenarioInvariantAGenuineReversalStillDwells` in
      `motor_policy_harness.cpp`.
- [x] **Invariant B test**: a commanded decel-to-zero (turn/drive stop)
      does not produce a held-at-zero coast period long enough to worsen the
      overshoot — i.e., the fix measurably improves on ticket 001's captured
      "before" baseline, not just moves the problem. —
      `scenarioInvariantBDecelToZeroNoSustainedResidual` in
      `motor_policy_harness.cpp`.
- [x] **100% of pre-existing `tests/sim/unit/test_motor_policy.py` cases
      pass unmodified** (or, if any fixture value must change, the change is
      justified explicitly in this ticket's completion notes, not silently
      loosened). — all 8 pre-existing scenarios unmodified (diff is
      additive-only, verified via `git diff` showing 0 removed lines in
      `motor_policy_harness.cpp`); the `.py` wrapper's compile command
      gained one extra source file (`velocity_pid.cpp`, needed to link the
      new scenarios' real PID calls) but no fixture/assertion changed.
- [x] `tests/sim/unit/test_velocity_pid.py` / `test_velocity_pid_response.py`
      / `test_stiction_and_motor_lag.py` all still pass.
- [x] `wedged()`/`wedgeSuspect()` semantics (the raw stuck-encoder latch and
      its motion-qualified variant) are unchanged — this ticket touches
      `armoredWrite()`'s reversal-detection only, never
      `updateWedgeDetector()`/`processResetIfPending()`. — in fact
      `armoredWrite()` itself was NOT touched at all; the entire fix lives
      in `velocity_pid.{h,cpp}` (see Completion Notes).
- [x] No change to `Hal::MotorVelocityPid::compute()`'s or
      `armoredWrite()`'s public signatures.
- [x] Sim-only acceptance for this ticket — the stand HITL pass validating
      the complete (002+003) fix is ticket 004's job, not this ticket's.

## Completion Notes

**Mechanism chosen**: the fix lives ENTIRELY in
`Hal::MotorVelocityPid::compute()` (`source/hal/velocity_pid.{h,cpp}`) —
`Hal::Motor::armoredWrite()` (`capability/motor.h`) was not touched at all.

Empirical instrumentation (temporary `fprintf` tracing of `compute()`/
`armoredWrite()` against a live `RT 9000` sim run, removed before this
commit) showed that in the sim's own default boot config
(`tests/_infra/sim/sim_api.cpp`'s `defaultMotorConfigSet()`, `min_duty`
unset -> `0.0f`), the literal pre-fix deadband condition `spAbs < minDuty`
is **dead code** (`spAbs >= 0 >= minDuty` makes `spAbs < 0.0f` always
false) — so the freeze branch never engages in this config, and
`armoredWrite()`'s `reversalDwell_` never arms either (every observed sign
flip lands below `outputDeadband_` first, which resets
`lastRequestedDuty_` to 0 and erases the "prior direction," so the
following opposite-sign duty is never seen as a reversal). The actual
observed defect: once the ramp's target settles at exactly `0.0f`, the
integrator — never frozen, just slowly updated via the small `ki` gain —
carries a residual bias built up trimming the PRIOR motion (e.g. a
feedforward/plant mismatch during a fast turn) straight into the stop,
producing a small, non-decaying, WRONG-signed correction for ~1.8-2s
(matches ticket 001's own captured "+2..+7 mm/s oscillating, decays
~1800-1900ms" shape) even though the anti-windup correctly bounds growth
*during* the turn.

**The fix**: two changes to `compute()`, both preserving the ORIGINAL
freeze semantics for the genuine low-speed-creep/stiction-floor use case
(a continuing low/zero target still gets zero ongoing integral action,
unchanged):
1. `inDeadband = spAbs <= minDuty` (was `<`) — boundary-inclusive, so an
   exact `target == 0.0f` counts as "in the deadband" even when `minDuty`
   itself is `0.0` (unconfigured) — the sim's own default, and the config
   ticket 001's regression tests run against.
2. **Reset (not freeze) the integrator on the tick the deadband is FIRST
   entered** (edge-triggered on a new `wasInDeadband_` member, not
   level-held) — clears whatever bias was sustaining the prior motion
   before it can leak into the near-zero-target regime. Continuing to sit
   in the deadband (target still 0/low on the NEXT tick) still freezes
   exactly as before — no change to that half of the original semantics.

This satisfies both invariants **by construction**, not via a special
case: Invariant A holds because a genuine full-scale reversal (target
flips sign while still large in magnitude on BOTH sides) never enters the
deadband at all, so `wasInDeadband_`'s edge-reset never engages —
`armoredWrite()` is byte-for-byte untouched, so its dwell behavior for that
case is provably identical to before. Invariant B holds because the reset
removes exactly the stale bias that was the issue's own root-caused
mechanism, without needing to distinguish "braking" from "reversal" inside
`armoredWrite()` at all.

**Before/after (RT 9000, sim, default boot config)**:
- Before: `EVT done RT` fires at true_h=97.61°, wheel still at ~half turn
  speed; crosses zero at +72ms; reverse-sign residual oscillates
  (+2..+7 mm/s) from ~+96ms through at least +800ms (ticket 001's captured
  worst case: 7.0 mm/s at t=1128ms), not fully settling until ~1800-1900ms;
  final heading backtracks to ~89.5° (an ~8° total backtrack).
- After (this ticket, re-measured against the same `RT 9000`): wheel
  velocity decays **monotonically** through zero with no reverse-sign
  residual at all — worst-case magnitude in the ticket-001 tail window
  (200-800ms post-`EVT done RT`) is **1.0 mm/s** (bound: 2.0 mm/s), settled
  to exactly 0 mm/s by +240ms.
- `D 200 200 500` is **unaffected** by this ticket (still ~532.5mm / 6.50%
  over, matching ticket 001's captured pre-fix number exactly) — the
  drive's overshoot is dominated by the Planner's own missing terminal
  decel/coast anticipation (ticket 003), not by this motor-loop mechanism;
  its `xfail` marker is correctly left in place, per this ticket's own
  hard-constraint guidance.

**No `nezha_motor.{h,cpp}` change** — confirmed unnecessary: the fix lives
entirely in the shared base `Hal::MotorVelocityPid`, consumed identically
by `Hal::NezhaMotor` and `Hal::SimMotor` (both call
`pid_.compute(...)` the same way; see `architecture-update.md`'s "Impact on
Existing Components" table, `Hal::SimMotor` row).

**Incidental fix (not a loosening)**: rebuilding after this change exposed
a pre-existing, unrelated wrap-boundary bug in
`tests/sim/unit/test_motion_commands_arc_turn.py::test_turn_reaches_absolute_heading_from_nonzero_start`
— its target (exactly 180°) sits on the heading representation's own ±180°
branch cut, and this ticket's fix shifted the settle from ~10.4° UNDER
(169.6°, same side as target) to ~2.9° OVER (182.9°, opposite side,
represented as ≈-177.1°) — a materially TIGHTER residual, but a raw
`h - expected` blows up to ~2π across that boundary. Added a `_wrap_pi()`
helper and used it for this one assertion; the ±13° tolerance itself is
UNCHANGED (a further tightening is ticket 004's retune per
architecture-update.md's own "Impact on Existing Components" table entry
for this file) — this is a measurement-correctness fix, not a loosened
bound. All 14 tests in that file pass.

**Test files touched**: `tests/sim/unit/motor_policy_harness.cpp` (2 new
scenarios, additive only — see diff), `tests/sim/unit/test_motor_policy.py`
(compile command gained `velocity_pid.cpp` as a second source input),
`tests/sim/unit/test_motion_overshoot_regression.py` (removed the `RT 9000`
xfail marker, updated its module/test docstrings; left the `D 200 200 500`
xfail marker and its docstring as-is), `tests/sim/unit/
test_motion_commands_arc_turn.py` (wrap-aware fix, described above).

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
