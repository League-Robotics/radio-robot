---
id: '015'
title: Firmware cleanup sweep (kEncodeScratchCap assert, trap 1 ordering fix, dead
  ShaperBootConfig/CONFIG_PLANNER/CONFIG_WATCHDOG)
status: open
use-cases:
- SUC-003
depends-on:
- '013'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware cleanup sweep (kEncodeScratchCap assert, trap 1 ordering fix, dead ShaperBootConfig/CONFIG_PLANNER/CONFIG_WATCHDOG)

## Description

Sweep of firmware-side cleanups the issue's audit surfaced, none touching
the wire's live behavior:

1. Add a `static_assert` guarding `kEncodeScratchCap = 220`
   (`wire.cpp:684`) against the largest nested message this schema now
   declares, so an oversized message fails to COMPILE instead of
   silently returning `0` from `encode()` at runtime.
2. Fix **trap 1** — `main.cpp:166` calls `loadPersistedTuning()` before
   `robotLoop().boot()` at `:171`, and every `RealOtos` setter no-ops
   until `begin()` sets `initialized_`. Reorder so persisted OTOS tuning
   is applied AFTER `begin()` runs (either by moving the
   `loadPersistedTuning()` call, or restructuring `boot()` so persisted
   tuning routes through `install(OTOS)` at the correct point in the
   sequence — implementer's call, but the reordering must be provable,
   not just asserted).
3. Delete dead `ShaperBootConfig` (`boot_config.h:125-137`) — confirm
   zero live consumer first with a fresh grep before deleting (the issue
   claims it is read by no code; verify, don't re-derive). Note: by this
   point in the sprint `ShaperBootConfig` may already be gone if ticket
   002/005 didn't carry it forward into `Config::Robot`'s generated
   shape — confirm its actual status before assuming it still needs
   deleting here.
4. Remove `CONFIG_PLANNER`/`CONFIG_WATCHDOG` from the `ConfigTarget`
   enum if confirmed dead — `CONFIG_WATCHDOG`'s documented routing
   target (`config_commands.cpp`/`BinaryChannel`) is confirmed ABSENT
   from the current tree by this round's grep. If the stream watchdog
   window has NO other live configuration path, flag this explicitly as
   a gap (Open Question 4 in sprint.md) rather than silently deleting a
   capability — do not delete `CONFIG_WATCHDOG` without first confirming
   there is truly no live consumer of the capability it was meant to
   serve, not just of the enum value's name.

## Acceptance Criteria

- [ ] A `static_assert` on `kEncodeScratchCap` exists, sized against the
      largest nested message this schema declares.
- [ ] The assert is demonstrated to fire on a deliberately oversized
      message in a throwaway branch, reverted before this ticket lands
      (completion notes record the demonstration).
- [ ] `loadPersistedTuning()`'s OTOS-tuning application happens AFTER
      `begin()`/`initialized_` is true, verified by a new test: persist
      an OTOS scale, reboot (sim), confirm the persisted value — not the
      baked default — is what the chip-level setter receives (trap 1's
      own regression test, matching sprint.md's Success Criteria).
- [ ] `ShaperBootConfig` is deleted ONLY after a fresh grep in this
      ticket's own implementation confirms zero live consumers; if a
      consumer is found (or it's already gone by this point), the
      ticket's completion notes say so.
- [ ] `CONFIG_PLANNER` is removed from `ConfigTarget` if confirmed dead
      (it already has no wire consumer per the 115-003 planner-stack
      deletion).
- [ ] `CONFIG_WATCHDOG` is either removed (with a documented replacement
      configuration path for the stream watchdog window, if one exists)
      or explicitly left in place with a completion note stating the gap
      is real and unresolved, per Open Question 4 — not silently deleted
      without checking.
- [ ] Compiles under `HOST_BUILD` and ARM.

## Testing

- **Existing tests to run**: not required to run the full sim suite per
  this ticket's own gate (heavy verification is tickets 018/019), but
  the specific tests this ticket touches (trap-1 regression,
  `kEncodeScratchCap` assert demonstration) must pass.
- **New tests to write**: as listed in Acceptance Criteria.
- **Verification command**: `uv run python -m pytest <affected test
  paths> -q`; a separate ARM build check (`just build` or equivalent)
  for the static_assert and any ARM-only code path.

## Implementation Plan

**Approach**: Four largely independent sub-fixes; tackle in the order
listed (assert first — cheapest, no behavior change; trap 1 next — a
real ordering bug; dead-code deletions last, since they require fresh
verification before removal).

**Files to modify**: `src/firm/messages/wire.cpp` (generated — confirm
whether the assert belongs in generator engine-text per
`gen_messages.py`, or is added directly if it's genuinely fixed
text), `src/firm/main.cpp`, `src/firm/app/boot_wiring.{h,cpp}` or
`configurator.{h,cpp}` (wherever the OTOS-tuning-after-`begin()`
reordering ends up living), `src/firm/config/boot_config.h`
(`ShaperBootConfig` deletion, if confirmed dead and still present),
`src/protos/config.proto` or `robot_config.proto` (`ConfigTarget` enum
cleanup).

**Testing plan**: as above.

**Documentation updates**: none beyond inline comments explaining each
fix.
