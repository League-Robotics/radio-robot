---
status: pending
---

# Stale incremental builds weld mismatched object files into the firmware — presented as "memory corruption" and cost a day

## Description

Changing `App::kCmdRingDepth` (a `constexpr` in
[src/firm/app/comms.h](../../src/firm/app/comms.h)) from 12 to 6 in commit
`5065775a` produced firmware that **hard-faulted at boot** — truncated
banner (`DEVICE:NEZHA2` and dead), garbage vtable dispatch (`blx r3` to
`0xb240b580`), totally silent robot. The failure was bisect-stable across
the whole day and looked exactly like latent RAM corruption. Two agents and
a stakeholder session were spent on it.

**The code was fine. The build was stale.** `comms.o` did not recompile
when `comms.h` changed, so the final image contained an ODR violation:

- `main.o` (rebuilt) and the **linker** sized `main::comms` for a 6-deep
  ring: object size `0x22c`.
- `comms.o` (stale, from a depth-12 compile) still held a constructor
  whose compiler-emitted zero-init memset swept **`0x420` bytes** —
  a 12-deep ring — through the 6-deep object and `0xF0+` bytes past its
  end, zeroing the neighboring statics.

Caught in the act with a DWT hardware watchpoint on
`main::motorL.inner_` (`0x200044e0`): sane at `main.cpp:129`, then the trap
fired **inside `memset`, called from `App::Comms::Comms`**
(`comms.cpp:141` — a constructor with *no* memset in its source, only
`Cmd cmdRing_[kCmdRingDepth]{}` member init), with `inner_` half-zeroed
(`0x20002610 → 0x20002600`, LSB-first byte sweep). The armor's `Motor&`
became garbage; the first virtual call through it (`motorL.position()` at
the odometry constructor) jumped into nowhere — which is the hardfault.

The two-instruction proof, from the linked ELF:

```
stale build :  App::Comms::Comms  ...  mov.w r2, #1056   ; 0x420 = 12 * sizeof(Cmd)
clean build :  App::Comms::Comms  ...  mov.w r2, #528    ; 0x210 =  6 * sizeof(Cmd)
map (both)  :  main::comms  size 0x22c                   ; sized for 6
```

**A clean build (`just build-clean`) of the same commit, same depth 6,
boots and streams telemetry normally.** Verified on hardware 2026-07-27
(full banner + `TLM:` frames on raw serial captured through the reboot).

Why the bisect lied so convincingly: `comms.o` never rebuilt across the
entire session, so every incremental build was really testing "does main.o
agree with the stale 12-deep comms.o" — depth 12 in the header ⇒ agreement
⇒ boots; depth 6 ⇒ mismatch ⇒ faults. 100% reproducible, and
indistinguishable from a genuine code bug until the watchpoint named the
writer.

## Cause

The ARM build's incremental dependency tracking does not reliably rebuild
dependents when a header changes, on this checkout's `/Volumes` filesystem.
This is a **known, previously-documented hazard** (memory / knowledge note
`stale-incremental-build-on-volumes`: "--clean before HITL flash") — what
is new is the failure *mode*: not merely "old behavior", but a **silently
ODR-corrupted image** whose symptoms imitate memory corruption and send
debugging in exactly the wrong direction.

Root cause of the staleness itself (mtime granularity on the mount, cmake
depfile handling, caching, …) has NOT been established — that is this
issue's remaining work.

## Proposed fix

1. **Find why the dep-tracking misses.** Touch `comms.h`, run `just
   build`, check whether `comms.cpp.o` recompiles; inspect the generated
   depfiles and the mtimes the build system compares on `/Volumes`.
2. Until that is fixed, **make the flashable-artifact path clean-build**:
   either `just build` does a clean ARM build (the micro:bit compile is
   the small part of the cost), or `mbdeploy deploy --build` warns/refuses
   on incremental images.
3. Optional cheap tripwire: a post-link check comparing a size-bearing
   constructor's memset against the map (the exact check that convicted
   this instance), or a CI step that builds clean and incremental and
   diffs the hex.

## Verification

- Touch `src/firm/app/comms.h` (whitespace), `just build`, confirm
  `comms.cpp.o` gets a fresh mtime (today it does not — re-verify as part
  of step 1).
- Behavior gate, on hardware: flash and capture raw serial **through the
  reboot** (the board does not reset on serial open); pass = full
  `DEVICE:NEZHA2:robot:tovez:...` banner plus `TLM:` frames.

## Resolution state of the original symptom

- `kCmdRingDepth` remains **12** on the branch (the sprint-124 sizing,
  equal to `kAckRingDepth`) with an honest comment at its definition; 6 is
  functionally safe if deliberately chosen *and clean-built*.
- The "latent RAM corruption" framing in earlier revisions of this issue
  and in commit messages `9eca7b0b`/`1daf1cbd`/`e137d1b6` is **withdrawn**:
  there is no evidence of any memory-corruption bug in the source. The
  intermediate leads those revisions carried (heap/stack overlap — wrong,
  misread map; 2 KB MSP region — standard CODAL layout, never a lead) are
  also withdrawn.
- Unrelated and still true: the bench robot currently wedges after banner
  + 2 frames in the documented dead-external-I2C-bus state
  (`system_timer_wait_cycles ← NRF52I2C::waitForStop ←
  NezhaMotor::hardReset ← Preamble::probeSlot`, confirmed by live
  backtrace). Per `.clasi/knowledge/silent-robot-dead-external-i2c-bus`,
  the prior occurrence of this exact signature was the motor brick's
  battery being dead; the physical check of the brick's power is the next
  step there. The bus demonstrably worked earlier the same day (the
  600-trial speed calibration ran over it).

## Related

- Memory notes: `stale-incremental-build-on-volumes` (the hazard),
  `codal-ram-always-near-full` (CODAL's always-full RAM layout is never
  the suspect), `silent-robot-dead-external-i2c-bus` (the look-alike bench
  failure and its one-command discriminator: halt and read PC — flash/
  HardFault vs RAM/busy-wait).
- The velocity-trim hardware gate
  (`rebuild-the-motion-planner-core-per-wheel-distance-profiling-velocity-domain-pid.md`)
  is unblocked by this resolution once the brick is powered.
