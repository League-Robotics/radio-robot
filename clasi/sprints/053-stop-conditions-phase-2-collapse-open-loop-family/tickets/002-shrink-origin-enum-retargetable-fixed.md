---
id: 053-002
title: "Shrink Origin enum to RETARGETABLE/FIXED"
status: open
use-cases:
- SUC-003
depends-on:
- 053-001
issue: stop-conditions-as-a-first-class-system-primitive.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 053-002: Shrink Origin enum to RETARGETABLE/FIXED

## Description

`MotionCommand::Origin` currently has 7 variants: `VW, TURN, G, T, D, R, RT`.
The only usage is the D6 keepalive guard in `handleVW` which checks
`activeCmd().origin() == MotionCommand::Origin::VW`. All other variants are
treated identically (reply busy=). Reduce the enum to two values:
`RETARGETABLE` (formerly `VW`) and `FIXED` (all others). Update all
`setOrigin()` callsites and the busy-reply name table in handleVW.

## Acceptance Criteria

- [ ] `MotionCommand::Origin` enum in `source/commands/MotionCommand.h` has
  exactly two values: `RETARGETABLE` and `FIXED`.
- [ ] `beginVelocity` in `MotionControllerBegin.cpp` calls
  `_activeCmd.setOrigin(MotionCommand::Origin::RETARGETABLE)`.
- [ ] All other `setOrigin()` calls (`beginArc`, `beginTimed`, `beginDistance`,
  `beginGoTo`, `beginTurn`, `beginRotation`, `_startPreRotate`) call
  `setOrigin(MotionCommand::Origin::FIXED)`.
- [ ] The default in `configure()` (which resets origin) uses `RETARGETABLE`
  (matching the old `VW` default; `MotionCommand.cpp` resets `_origin =
  Origin::VW` on configure — update to `RETARGETABLE`).
- [ ] The keepalive guard in `handleVW` (`MotionCommands.cpp`) checks
  `origin() == MotionCommand::Origin::RETARGETABLE`.
- [ ] The `kOriginNames` busy-reply table (currently 7 strings) is replaced
  with a simple `const char* originName = (origin == Origin::RETARGETABLE)
  ? "RETARGETABLE" : "FIXED";` or equivalent inline expression. Wire output
  changes from e.g. `busy=T` to `busy=FIXED` — this is an internal-only wire
  change (the busy= value is informational, not parsed by hosts).
- [ ] `uv run --with pytest python -m pytest tests/simulation -q` passes with
  exactly 2 known failures.
- [ ] `python build.py --clean` exits 0.

## Implementation Plan

### Approach

Mechanical rename. Three files touch. The only behavioral change is the
busy= reply string (from verb-name to "FIXED"), which is informational only.

### Files to Modify

- `source/commands/MotionCommand.h`
  - Change enum class `Origin : uint8_t { VW, TURN, G, T, D, R, RT };` to
    `{ RETARGETABLE, FIXED };`.
  - Update the doc comment: remove the list of verb names; describe the
    property ("RETARGETABLE: target may be updated by a bare VW keepalive;
    FIXED: command runs to completion, VW keepalive replies busy=").

- `source/commands/MotionCommand.cpp`
  - In `configure()`, update the default reset: `_origin = Origin::RETARGETABLE;`

- `source/control/MotionControllerBegin.cpp`
  - `beginVelocity`: `setOrigin(Origin::RETARGETABLE)`.
  - `beginArc`: `setOrigin(Origin::FIXED)`.
  - `beginTimed`: `setOrigin(Origin::FIXED)`.
  - `beginDistance`: `setOrigin(Origin::FIXED)`.
  - `beginGoTo`: `setOrigin(Origin::FIXED)` (two callsites: PRE_ROTATE and
    PURSUE branches).
  - `beginTurn`: `setOrigin(Origin::FIXED)`.
  - `beginRotation`: `setOrigin(Origin::FIXED)`.
  - `_startPreRotate`: `setOrigin(Origin::FIXED)`.

- `source/commands/MotionCommands.cpp`
  - In `handleVW`, the keepalive guard: replace
    `origin() == MotionCommand::Origin::VW` with
    `origin() == MotionCommand::Origin::RETARGETABLE`.
  - Replace the `kOriginNames` array and indexing with:
    ```cpp
    const char* originName =
        (ctx->mc->activeCmd().origin() == MotionCommand::Origin::RETARGETABLE)
        ? "RETARGETABLE" : "FIXED";
    ```

### Testing Plan

- Run `uv run --with pytest python -m pytest tests/simulation -q`.
  Expect exactly 2 known failures. Verify keepalive guard tests pass.
- Check `tests/simulation/unit/` for any test that asserts on the exact
  `busy=` string. Update expected strings from `busy=T`/`busy=G`/etc. to
  `busy=FIXED` where needed.
- `python build.py --clean` exits 0.

### Documentation Updates

None required this ticket. The `source/commands/MotionCommand.h` doc comment
update is included above.
