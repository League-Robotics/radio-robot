---
status: pending
---

# Latent RAM corruption: shrinking a static buffer hard-faults the firmware at boot

## Description

Commit `5065775a` lowered `App::kCmdRingDepth` from 12 to 6
([src/firm/app/comms.h](../../src/firm/app/comms.h)). With that one constant
at 6 the firmware **hard-faults during boot**, before the banner finishes
clocking out: the robot emits the first ~13 bytes of
`DEVICE:NEZHA2:robot:...` and dies. Restoring it to 12 makes the same tree
boot and stream telemetry normally.

Bisected on hardware 2026-07-27, each point built and flashed via the
`MICROBIT` mass-storage drive and captured on raw serial through the reboot:

| commit | boot serial |
|---|---|
| `bd7f75b8` | 97 bytes — full banner + `TLM:` frames |
| `3238bdaf` | 97 bytes — full banner + `TLM:` frames |
| `5065775a` | **13 bytes — `DEVICE:NEZHA2`, then dead** |
| `5065775a` with `kCmdRingDepth` restored to 12 | 97 bytes — full banner + `TLM:` frames |

Fault signature: `HardFault_Handler`, `lr = 0xffffffe9` (exception return),
with a **garbage stacked PC** (observed `0xb240b580`, not a valid flash or
RAM address). The byte count before death is nondeterministic (13 and 6 both
observed across builds), i.e. the fault lands at a varying moment relative
to the serial DMA — an interrupt-time fault, not straight-line code.

## Cause

**Not yet found.** What is established is that the trigger is a RAM *layout*
change, not a bounds error:

- Both rings are internally consistent. `cmdRing_[kCmdRingDepth]` is only
  ever indexed `% kCmdRingDepth` ([comms.cpp:173-185](../../src/firm/app/comms.cpp#L173-L185))
  and `ackRing_[kAckRingDepth]` only `% kAckRingDepth`
  ([telemetry.cpp:165-215](../../src/firm/app/telemetry.cpp#L165-L215)). Neither
  indexes the other. So the documented "keep the two depths EQUAL" rule
  (telemetry.h:226) is about ack *observability*, and violating it should
  drop acks, not fault.
- The change that breaks it **shrinks** static RAM by roughly a kilobyte.
  A buffer getting smaller cannot itself overflow anything.

The only explanation consistent with both facts is pre-existing memory
corruption — a stray write, a stale pointer, or a stack/heap collision —
whose victim depends on where objects land. At depth 12 the damage falls
somewhere harmless; at 6 it lands on something load-bearing.

One concrete lead worth checking first: the linker map shows `.heap` and
`.stack` **overlapping**. `.heap` starts at `0x20005558` with size
`0x1a2a8`, ending at `0x2001F800`; `.stack` is 8 KB growing down from the
top of RAM (`0x20020000`), i.e. from `0x2001E000`. That is ~6 KB of overlap.
Observed fault SPs (`0x2001fb70`, `0x2001f988`) sit inside that window.

## Proposed fix

`kCmdRingDepth` is back at 12 with a comment saying plainly that 12 is a
value known to *survive*, not a value known to be *correct*. That is a
tourniquet, not a fix — the defect is still in the tree and will resurface
the next time anything perturbs static RAM (a new member, a resized buffer,
a compiler change).

To actually close it:

1. Check the `.heap` / `.stack` overlap above against the CODAL linker
   script and `CODAL_STACK_SIZE`. If the heap really can allocate into the
   stack's range, that alone explains everything.
2. Reproduce deterministically by padding: add a `static volatile uint8_t
   pad[N]` and sweep `N`. If the fault comes and goes with `N`, it is
   confirmed layout-dependent and the sweep localizes the victim.
3. Put a watchpoint on whatever `N` implicates, or enable a stack canary /
   MPU guard region at the stack limit, and catch the writer in the act.

## Verification

Build and flash, capture raw serial through the reboot, and require the
FULL banner plus at least one `TLM:` frame:

```bash
just build
# capture, then: cp MICROBIT.hex /Volumes/MICROBIT/
```

A pass is 97 bytes beginning `DEVICE:NEZHA2:robot:tovez:` — a fail is any
truncation. Note this is a *boot* gate: it is checked through the reboot the
drag-flash causes, because the board does not reset on serial open
(`serial-monitor-never-shows-the-banner`).

Regression guard worth adding: the truncated banner was invisible to every
existing test. Nothing in `src/tests/` flashes hardware, and the sim builds
a host binary with a completely different RAM layout, so a fault that only
manifests on the ARM image cannot be caught there today.

## Related

- The bench gate this blocked: the velocity-trim hardware run
  (`rebuild-the-motion-planner-core-per-wheel-distance-profiling-velocity-domain-pid.md`).
- `silent-robot-dead-external-i2c-bus` — a *different* failure with a
  similar surface (banner then silence). Distinguish by halting and reading
  PC: in RAM = CODAL busy-wait (I2C wedge); `HardFault_Handler` = this bug.
