---
id: '003'
title: "NullOdometer \u2014 collapse the nullable Hardware::odometer() contract"
status: in-progress
use-cases:
- SUC-003
depends-on:
- '002'
github-issue: ''
issue: null-odometer-object.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# NullOdometer — collapse the nullable Hardware::odometer() contract

## Description

Introduce `Hal::NullOdometer` so `Hardware::odometer()` always returns a
valid reference (never null). Collapse the three `if (odometer !=
nullptr)` branches in `main_loop.cpp` — **plus, per architecture-update.md
Decision 3 (a scope widening beyond the source issue's own stated Scope,
approved by team-lead)**, the equivalent null-guards in `source/main.cpp`
(`bb.otosPresent = (hardware.odometer() != nullptr)`) and
`source/runtime/configurator.cpp` (`if (odometer != nullptr)
odometer->configure(...)`).

Depends on ticket 002: `NullOdometer::fusableThisPass()` overrides the base
to unconditionally return `false`, composing with ticket 002's fusability
contract rather than re-inventing a validity signal — this ticket must
land ON TOP of 002's contract, not before it.

**Codebase-alignment finding this ticket relies on** (verified by direct
source read during sprint planning, not assumed): both concrete `Hardware`
owners already override `odometer()` to non-null —
`NezhaHardware::odometer()` since ticket 086-006, `SimHardware::odometer()`
since ticket 081-003. The base class's `return nullptr;` default is
reachable only through the abstract interface, not through any owner
actually constructed today. `otos_commands_harness.cpp`'s own docstring
independently confirms the "device absent → ERR nodev" branch is already
dead code in every build this tree produces (it now asserts the OPPOSITE:
all seven OTOS verbs reach dispatch and reply OK against real
`NezhaHardware`). This means collapsing the null checks to their
unconditional form does not change any currently-reachable production
value — it removes dead defensive branches.

## Acceptance Criteria

- [ ] `Hal::NullOdometer` (new, `source/hal/capability/null_odometer.h` —
      headers-only, zero `HOST_BUILD`/`PhysicsWorld` dependency, NOT under
      `hal/sim/`) implements every `Hal::Odometer` primitive inertly:
      `tick()` no-ops, `pose()` returns an identity/zero
      `msg::PoseEstimate` (stamp not valid), `connected()` returns `false`,
      every setter (`init`/`resetTracking`/`setPose`/`setLinearScalar`/
      `setAngularScalar`) discards, `fusableThisPass()` unconditionally
      returns `false` (overriding ticket 002's base flag-based logic
      entirely).
- [ ] `Subsystems::Hardware::odometer()`'s base-class default changes from
      `return nullptr;` to returning a valid `NullOdometer` (e.g. a
      static/shared instance, no per-call allocation) — every existing
      owner (`NezhaHardware`, `SimHardware`) keeps its own override
      unchanged.
- [ ] `main_loop.cpp`'s three `if (odometer != nullptr)` branches (the
      reset-drain guard + its discard-else-branch, and the COMMIT block's
      `bb.otosValid` branch) collapse to their unconditional form;
      `bb.otosValid` derives from `odometer->fusableThisPass()` directly
      (folding in ticket 002's contract), not a `!= nullptr` test. **Watch
      the sequencing carefully**: `fusableThisPass()` is a read-and-clear,
      single-caller contract (ticket 002) — if `poseEstimator_.tick()`'s
      gate already consumed/cleared it earlier this same pass, the COMMIT
      block must NOT call it a second time to derive `bb.otosValid` (that
      would incorrectly report "fusable" on the second read regardless of
      the first). Resolve this by deriving both values from ONE call this
      pass (e.g. capture the result once and reuse it), not two.
- [ ] `source/main.cpp:174`'s `bb.otosPresent = (hardware.odometer() !=
      nullptr)` is updated to reflect the non-nullable contract (the
      computed value does not change in practice — both concrete owners
      already override to non-null — but the code must no longer read as
      if null were possible).
- [ ] `source/runtime/configurator.cpp`'s `if (odometer != nullptr)
      odometer->configure(odometerConfig_)` guard (~line 199-202)
      collapses to an unconditional call.
- [ ] `uv run python -m pytest tests/sim` is green, including
      `otos_commands_harness.cpp`/`test_otos_commands_nodev.py` (already
      assert "OK, not ERR nodev" against real `NezhaHardware` — must still
      pass unchanged).
- [ ] No wire-observable behavior change: `bb.otosPresent`/`bb.otosValid`'s
      computed values are identical to pre-ticket behavior for every
      existing test scenario (verified, not assumed).

## Implementation Plan

**Approach**:
1. Add `source/hal/capability/null_odometer.h` implementing
   `Hal::Odometer` inertly per the Acceptance Criteria above. No `.cpp`
   needed if kept headers-only, matching `capability/odometer.h`'s own
   convention.
2. Change `Subsystems::Hardware::odometer()`'s base default in
   `source/subsystems/hardware.h` to return a `NullOdometer&`.
3. In `main_loop.cpp`: simplify the reset-drain block — both arms' actions
   still happen, just unconditionally (the discard-mailboxes else-arm
   becomes redundant since a `NullOdometer`'s `apply()` already discards
   inertly; drop the branch, always call `applySetPose()`/`apply()` when
   the mailboxes are non-empty). Simplify the COMMIT block's
   `if (odometer != nullptr) {...; bb.otosValid = true;} else {bb.otosValid
   = false;}` to derive `bb.otosValid` from the SAME `fusableThisPass()`
   read already used to gate `poseEstimator_.tick()` earlier this pass —
   do not call it a second time (see Acceptance Criteria's sequencing
   note). Resolve the exact restructuring at implementation time and
   record the resolution in the ticket's own commit message, since this
   is the one place ticket 003's scope brushes against ticket 002's
   read-once contract.
4. In `main.cpp`: simplify `bb.otosPresent = (hardware.odometer() !=
   nullptr)` to reflect the new contract (the field itself stays — per
   `otos_commands.h`'s own dependency on it for `ERR nodev` gating,
   unchanged wire behavior — only the nullable-looking check goes).
5. In `configurator.cpp`: drop the `if (odometer != nullptr)` guard around
   `odometer->configure(odometerConfig_)`.

**Files to modify**: `source/hal/capability/null_odometer.h` (new),
`source/subsystems/hardware.h`, `source/runtime/main_loop.cpp`,
`source/main.cpp`, `source/runtime/configurator.cpp`.

**Documentation updates**: none beyond the new file's own header comment
(matching every other `hal/capability/*.h` file's convention).

## Testing

- **Existing tests to run**: full `tests/sim`, with explicit attention to
  `otos_commands_harness.cpp`, `test_otos_commands_nodev.py`,
  `test_otos_commands.py`, `test_command_smoke.py` (all reference the
  nodev/device-presence contract), plus the SI/OZ/OR/OV tests from
  ticket 002 (re-run to confirm ticket 003 does not regress ticket 002's
  fix).
- **New tests to write**: a small unit test constructing a bare
  `Subsystems::Hardware`-derived stub that does NOT override `odometer()`,
  confirming the base default returns a non-null, inert `NullOdometer`
  (mirrors `configurator_harness.cpp`'s existing
  `checkTrue(hardware.odometer() != nullptr, ...)` sanity-check pattern).
- **Verification command**: `uv run python -m pytest tests/sim`
