---
status: done
sprint: '128'
tickets:
- 128-013
---

# Delete the full-transaction I2C IRQ guard

## Description

**Decision (stakeholder, 2026-07-30): remove the IRQ guard entirely.** Not
"leave it off" — delete the mechanism. Nobody re-enables or reinstates it without
a demonstrated need.

> I think this experiment succeeded, and we should have no IRQ guard. [...] Don't
> go reverting this until we find a reason to turn it back on. If we're not
> seeing any problems resulting from having it off, then we just remove it. We
> just remove the whole system of having IRQ guards.

The 2026-07-28 change at `src/firm/main.cpp:213` (`bus.setIrqGuard(false)`,
labelled TEMPORARY) succeeded and has shipped for two days across sprint 126's
full camera-verified OTOS calibration campaign — 18 straight runs, 24 turns on
the playfield — and all of sprint 127's bench work, with no runaway.

## Cause — what the guard was actually for

Three distinct mechanisms are conflated in
`src/firm/devices/microbit_i2c_bus.cpp`, and only one of them is the IRQ guard.
Getting this apart is the whole point, because only one should be deleted:

| # | Mechanism | Where | Cost | Job |
|---|---|---|---|---|
| 1 | `waitForClearance()` per-device `preClear`/`postClear` timer | `write()`:61-65, `read()`:106-110 | none (a wait) | **This is "nothing hits the bus during the encoder settling period."** Runs BEFORE any masking, deliberately — *"never mask interrupts for a multi-ms wait."* |
| 2 | Re-entrancy flag `inUse_` check-and-set | `write()`:73-83, `read()`:114-124 | a few instructions | Detects/counts two callers overlapping a transaction (`reentryViolations_`). |
| 3 | **Full-transaction IRQ guard `irqGuard_`** | `write()`:83-90, `read()`:124-131 | **~7-8% inbound command loss** | Keeps IRQs masked across `bus_.write()`/`bus_.read()`. |

**The IRQ guard (#3) was never about bus contention.** Nothing in this firmware
touches I2C from an interrupt handler; bus ordering and settling are handled
structurally by the single-threaded loop schedule plus #1 and #2. #3 exists
solely as a workaround for an **nRF52 TWIM silicon errata**: the STOPPED event
sometimes fails to fire "under higher levels of background interrupt load"
(`NRF52I2C::waitForStop`), leaving `waitForStop()` spinning forever. Masking
interrupts for the transaction reduces the interrupt load that provokes a
hardware bug.

The cost is severe and well characterised: the nRF52 serial RX is DMA-driven, so
bytes arriving inside a masked window are lost outright. Measured inbound command
loss with the guard on is ~7-8% on direct USB.

**Why deleting it is now the right trade.** The errata's consequence is an
encoder wedge. In the pre-rebuild `source/` firmware a wedge was unbounded — it
broke the `D` distance-stop reading that encoder, and the opposite wheel ran away.
That is the incident that made the guard "non-negotiable," and that chain no
longer exists:

- The `D` distance-stop was deleted in the sprint 115 gut-to-minimal rebuild.
- Protocol v4/v5 makes `timeout` **mandatory** on every `MOVE` (`timeout <= 0` →
  `ERR_BADARG`), so a wedged encoder cannot produce an unbounded run — the
  timeout fires regardless of what the encoders report.
- Wedge detection is explicit (`Devices::MotorArmor::wedged()`, telemetry flags
  bit 7), and motor armor moved into the firmware base.

So we are paying a large, certain, permanent cost — an unreliable command
channel — to reduce the probability of a fault that is now detected and bounded.

## Proposed fix

**Delete #3. Keep #1 and #2.** They do different jobs, they are nearly free, and
`reentryViolations_` is live diagnostic value.

1. `src/firm/devices/microbit_i2c_bus.h` — remove `setIrqGuard()`, `irqGuard()`,
   and the `irqGuard_` member. Remove the "defaults ON — non-negotiable" language
   at lines 6 and 136-142. Keep a short note recording that the TWIM errata
   exists and that the guard was removed deliberately, with the reasoning above,
   so the next reader who finds the old runaway report does not reinstate it by
   reflex.
2. `src/firm/devices/microbit_i2c_bus.cpp` — drop `irqGuard_(true)` from the
   constructor (line 26) and the `const bool guard = irqGuard_` branches in
   `write()`/`read()`. The `target_disable_irq()` / `target_enable_irq()` pair
   around the `inUse_` check-and-set **stays** — that is #2, and it is unrelated.
3. `src/firm/main.cpp:213` — delete the `bus.setIrqGuard(false)` line and its
   TEMPORARY comment. Once the mechanism is gone there is nothing to set.
4. `src/firm/devices/i2c_bus.h:8` and `src/firm/devices/DESIGN.md` — drop
   `setIrqGuard`/`irqGuard` from the documented concrete-class surface.

Removal is preferable to leaving the knob at `false`: a disabled knob invites
someone to flip it during a future debugging session and rediscover the 7-8% loss
the hard way.

## Verification

- Firmware builds and flashes; the bench gate passes on the stand.
- Inbound command loss stays at the guard-off level (~0%, versus ~7-8% with it
  on). This is the property being protected.
- Wedge-latch and bus-error counters over a normal driving workload are no worse
  than the current guard-off baseline.
- `grep -rn "irqGuard\|setIrqGuard" src/` returns nothing outside history.

## Open question — worth understanding, NOT a reason to keep the guard

Telemetry flags bit 7 (`kFlagFaultWedgeLatch`, `Devices::MotorArmor::wedged()`)
reads **set** on the live robot (`flags = 219`). It was already set before any
sprint-127 driving, and has not corresponded to any observed runaway or
misbehaviour across two days of calibration and bench runs.

Worth understanding on its own merits — a fault bit that is always on is a fault
bit nobody will ever read — but it does not gate the deletion:

- Is it a sticky latch that trips once and never clears, or does it re-assert
  live? Read `MotorArmor::wedged()`'s derivation rather than inferring from the
  name.
- Is it set immediately after a clean power cycle with no driving?

Direct precedent in the same flags word: **bit 6** (`kFlagFaultI2CSafetyNet`) is
documented at `src/firm/app/telemetry.h:76-122` as saturating on nearly every
frame of a healthy robot, because `Otos::readPositionVelocity()` does a
register-select write immediately followed by a read with no scheduled gap. A
previous claim that bit 6 was a boot-time one-shot latch was **falsified by
direct on-chip measurement**. Bit 7 deserves the same scrutiny before anyone
reads meaning into it.

Also worth one cheap check while in here: whether `reentryViolations_` has ever
been nonzero. If re-entrancy is structurally impossible in the single-loop
design, #2 could eventually go too — but that is a separate question and needs
its own evidence, not an assumption.

## Related

- `clasi/issues/i2c-safety-net-bit-conflates-otos-settle-wait-with-loop-schedule-health.md`
  — bit 6's saturation problem; the precedent for a fault bit in this word that
  does not mean what it appears to.
- `src/firm/app/telemetry.h:60-135` — flags-word class taxonomy and bit 6's
  falsified-claim history.
- `.claude/rules/hardware-bench-testing.md` — the standing bench gate.
- The original runaway that made the guard non-negotiable was measured against
  the `source/` tree's `D` distance-stop, which no longer exists. Any future
  argument for reinstating the guard must account for the mandatory `MOVE`
  timeout backstop that replaced it, rather than citing that incident directly.
