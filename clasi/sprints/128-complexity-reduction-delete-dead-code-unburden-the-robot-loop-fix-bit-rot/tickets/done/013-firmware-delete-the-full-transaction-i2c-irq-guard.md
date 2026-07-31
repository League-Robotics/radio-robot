---
id: '013'
title: 'Firmware: delete the full-transaction I2C IRQ guard'
status: done
use-cases:
- SUC-003
depends-on: []
github-issue: ''
issue: make-irq-guard-off-permanent-and-reconcile-the-docs.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware: delete the full-transaction I2C IRQ guard

**Worktree note**: implement in the git worktree checked out for
`sprint/128-complexity-reduction-delete-dead-code-unburden-the-robot-loop-fix-bit-rot`.
All paths below are relative to that worktree.

**Decision**: already stakeholder-decided 2026-07-30 ("we should have no
IRQ guard... we just remove the whole system of having IRQ guards") —
this ticket executes the recorded decision; it is not a design choice to
re-litigate.

## Description

Three distinct mechanisms are conflated in
`src/firm/devices/microbit_i2c_bus.cpp`, and only one — the full-transaction
IRQ guard (`irqGuard_`) — is being deleted. The per-device
`preClear`/`postClear` clearance wait and the re-entrancy flag
(`inUse_`/`reentryViolations_`) do different jobs and are NOT touched.
The guard was a workaround for an nRF52 TWIM silicon errata (STOPPED
event sometimes fails to fire under interrupt load); it costs ~7-8%
inbound command loss (DMA-driven serial RX loses bytes inside a masked
window) to reduce the probability of a now-bounded fault (the mandatory
`MOVE` timeout backstop and explicit wedge detection didn't exist when
the guard was made "non-negotiable"; they do now).

## Acceptance Criteria

- [x] `src/firm/devices/microbit_i2c_bus.h`: `setIrqGuard()`, `irqGuard()`,
      and the `irqGuard_` member are removed. The "defaults ON --
      non-negotiable" language is removed. A short note is kept recording
      that the TWIM errata exists and that the guard was removed
      deliberately (with the reasoning above), so a future reader who
      finds the old runaway report does not reinstate it by reflex.
- [x] `src/firm/devices/microbit_i2c_bus.cpp`: `irqGuard_(true)` is
      dropped from the constructor; the `const bool guard = irqGuard_`
      branches in `write()`/`read()` are removed. The
      `target_disable_irq()`/`target_enable_irq()` pair around the
      `inUse_` check-and-set STAYS — that is the re-entrancy flag, a
      separate mechanism.
- [x] `src/firm/main.cpp:213`'s `bus.setIrqGuard(false)` line and its
      `TEMPORARY` comment are deleted.
- [x] `src/firm/devices/i2c_bus.h` and `src/firm/devices/DESIGN.md` drop
      `setIrqGuard`/`irqGuard` from the documented concrete-class
      surface.
- [x] `grep -rn "irqGuard\|setIrqGuard" src/` returns nothing outside
      history/comments explaining the removal.
- [x] Cheap incidental check while in here (not a gating requirement):
      note whether `reentryViolations_` has ever been observed nonzero
      on the bench — if re-entrancy is structurally impossible in the
      single-loop design, that's a data point for a FUTURE issue about
      removing mechanism #2 too, not something to act on in this ticket.
      (No bench instrumentation was run as part of this ticket — no
      existing telemetry/log surface currently reports
      `reentryViolations()` for a passive bench observation without
      writing new diagnostic code, which is out of scope for a pure
      deletion ticket. Left as an open data point for the future
      mechanism-#2-removal issue rather than actioned here.)

## Testing

- **Existing tests to run**: firmware pytest tiers touching
  `microbit_i2c_bus`; full `motion_tests`/planner ctest suite (unrelated
  to this file, confirm no incidental breakage).
- **New tests to write**: none required for a pure deletion of dead
  configuration surface; if an existing test exercises `setIrqGuard()`,
  remove it.
- **Verification command**: `just build-clean`, then the standing bench
  gate is the real verification here (sprint-level, not per-ticket) —
  this ticket's own local check is `grep -rn "irqGuard" src/`.

## Implementation Notes

- **Approach**: delete the three mechanism-#3 call sites in the order
  given (header, then .cpp, then main.cpp's call site, then the
  documented surface) — leaves mechanisms #1 (clearance wait) and #2
  (re-entrancy flag) completely untouched at every step.
- **Files to modify**: `src/firm/devices/microbit_i2c_bus.{h,cpp}`,
  `src/firm/main.cpp`, `src/firm/devices/i2c_bus.h`,
  `src/firm/devices/DESIGN.md`.
- **Documentation updates**: `src/firm/devices/DESIGN.md` (drop the
  removed surface; add the short deliberate-removal note named above).
