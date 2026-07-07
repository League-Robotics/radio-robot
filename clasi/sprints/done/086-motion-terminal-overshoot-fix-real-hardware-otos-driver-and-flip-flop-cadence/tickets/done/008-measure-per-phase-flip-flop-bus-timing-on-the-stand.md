---
id: 008
title: Measure per-phase flip-flop bus timing on the stand
status: done
use-cases:
- SUC-008
depends-on: []
github-issue: ''
issue: flip-flop-cadence-below-design-target.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Measure per-phase flip-flop bus timing on the stand

## Description

**Measure first — this issue is explicit that the cadence gap (measured
~44-52 Hz vs. a ~80-90 Hz design estimate) is not urgent and not a
correctness bug** (the embedded PID closes the loop cleanly at the measured
cadence). This ticket is measurement only — zero production code change by
design.

`I2CBus` already carries the instrumentation needed (`txnCount()`/
`errCount()`/`lastErr()`/`dumpRecent()`, a 24-entry timestamped transaction
log) — compiled into every build already, not gated by `HOST_BUILD`/
`ROBOT_DEV_BUILD`. No wire command exposes it today. Reviving the stale
pre-rebuild `DBG OTOS`/`DBG OTOS BENCH` wire family (`docs/protocol-v2.md`
§14 — confirmed not implemented anywhere in `source/`) is explicitly NOT
the path here.

**Default measurement path (architecture-update.md Design Rationale 5)**:
use `pyOCD`/`gdb` (already fully documented and set up for exactly this,
`.claude/rules/debugging.md`) to read the live static `I2CBus` instance's
counters/log directly, with ZERO firmware change. A new dev-only wire verb
is a fallback ONLY if a debugger-based read proves impractical for the
number of samples needed (e.g., halting the core to read memory perturbs
the very timing being measured) — do not build one up front on
speculation.

Specifically test the **double-counted-clearance hypothesis**: does a
single in-use port's `REQUEST_DUE`'s `preClear=4000` and the PRECEDING
`COLLECT_DUE`'s duty-write `postClear=4000` (`writeMotorRun()`,
`nezha_motor.cpp`) each pay a full, non-overlapping 4 ms wait for what is
effectively the same real-world gap, or is time being double-counted?

## Acceptance Criteria

- [x] A per-phase timing breakdown (request write, settle wait, collect
      read, scheduling/main-loop overhead) is captured and recorded for at
      least one 2-port closed-loop motion session reproducing the 079-006
      measured ~19-22 ms (~44-52 Hz) per-motor cadence.
- [x] The double-counted-clearance hypothesis is explicitly confirmed or
      refuted with data — not left as a guess. **(REFUTED with data.)**
- [x] No firmware/production code change in this ticket (measurement only).
      **(Zero. `_logOn` was toggled via a gdb RAM write on the live static
      instance; no source changed, no reflash.)**
- [x] The measurement approach used (pyOCD/gdb read vs. a fallback wire
      verb, if that path was needed) is documented in this ticket's
      completion notes along with the raw/summarized data, so ticket 009 has
      a concrete basis to decide from. **(Primary pyOCD/gdb path worked; no
      fallback verb needed.)**

## Completion Notes (team-lead measurement session, 2026-07-06, fw `0.20260706.17`)

**Method (primary pyOCD/gdb path — no fallback verb needed, zero firmware
change).** The 24-entry `I2CBus::_log` ring is opt-in, gated by `_logOn`
(default false — [i2c_bus.cpp:149](../../../../source/com/i2c_bus.cpp)). Enabled
it by a gdb RAM write to the live function-local static `main::i2cBus`
(`set var 'main'::i2cBus._logOn = 1`, address `0x20002f94` = `main::i2cBus+572`),
zeroed `_logHead`/`_logTotal`, resumed. Then drove a 2-port closed-loop
session over serial (`DEV DT PORTS 1 2`, `S`-commanded, watchdog fed), and
halted + dumped `_log`/`_logTotal`/`_logHead`/`_devices` — the ring holds
timestamps recorded during free-running operation, so reading it after the
halt does NOT perturb the measured cadence (the ticket's stated concern).
Two captures: (A) steady speed, (B) oscillated speed to force `writeMotorRun`
duty writes (`0x60`) into the window (it is throttled by `kMinWriteIntervalUs`
= 40 ms AND skips when duty is unchanged — [nezha_motor.cpp:300,309](../../../../source/hal/nezha/nezha_motor.cpp),
so a settled free-spinning stand emits none).

**Single I2C device.** All four ports share the one Nezha brick at `0x10`
(one readback register); `_devices[0].addr=16`, `_devices[1]` empty. The
"2-port" flip-flop interleaves two ports on the same address (confirmed: the
collect-read low byte alternates between two encoder values).

**Per-phase breakdown (capture B, 24-txn window, `t` [us]):**

| Transition | Measured gap | What it is |
|---|---|---|
| `REQUEST` (`0x46` W) → `COLLECT` (R) | ~4.55 ms | requestEncoder settle: its `postClear=4000` (with `preClear=4000` as a floor) + the ~0.55 ms read |
| `COLLECT` (R) → `MOVE` (`0x60` duty W) | ~1.0 ms | writeMotorRun fires immediately after collect (`preClear=0`); present only when duty changed & the 40 ms throttle allows |
| `MOVE` (`0x60` W) → next `REQUEST` (`0x46` W) | **~4.97 ms** (4967/4967/4966/4958/4980, n=5) | writeMotorRun `postClear=4000` + next requestEncoder `preClear=4000` |
| (no duty write) `COLLECT` → next `REQUEST` | ~4.97 ms | collectEncoder `postClear=4000` + next requestEncoder `preClear=4000` |
| **Per port-visit** | ~9.5 ms (no duty write) – ~10.5 ms (with) | two ~4.75 ms clearance windows + ~1 ms duty write when present |
| **Per-motor cadence (2 ports interleaved)** | **~19–21 ms (~48–52 Hz)** | two port-visits — reproduces the 079-006 ~19-22 ms / ~44-52 Hz figure |

**Double-counted-clearance hypothesis: REFUTED — with data on the exact pair
the ticket names.** The `MOVE`(`0x60`, `postClear=4000`) → next `REQUEST`
(`0x46`, `preClear=4000`) gap measured **~4.97 ms across five duty-write
cycles (4958–4980 µs), not ~8 ms.** `I2CBus::write`'s entry spin computes
`entryDeadline = max(readyAt, lastEnd + preClear)`
([i2c_bus.cpp:62-66](../../../../source/com/i2c_bus.cpp)). After the duty
write, `readyAt = lastEnd + 4000` (its `postClear`); the next request's
`lastEnd + preClear` is *also* `lastEnd + 4000`; `max()` collapses them to
the identical instant, so exactly ONE 4 ms window is paid, once. If the two
4 ms clearances stacked, the gap would be ~8 ms and the per-motor cadence
~half (~26 Hz) — neither is observed. The `requestEncoder` `preClear=4000`
is a redundant floor for this single-device case (the preceding txn's
`postClear` already establishes the same deadline), NOT additive; it exists
to cover a preceding txn that carried no/smaller `postClear`.

**Conclusion for ticket 009.** There is NO double-counted or otherwise
reclaimable clearance time. The ~52 Hz cadence is the sum of two genuinely
separate, non-overlapping ~4 ms clearance windows per port-visit
(request-settle + inter-transaction gap), each of which is the 079-006
TWIM-stall safety margin (see [nezha_motor.cpp:382-406](../../../../source/hal/nezha/nezha_motor.cpp)
— the `postClear`/`preClear` pair was added specifically to give a single
in-use port a real ≥4 ms gap and stop the multi-second CODAL
`waitForStop()` stalls). Removing either reopens 079-006 — ticket 009's hard
NON-GOAL. → **Ticket 009 outcome is doc-correction (outcome 2), not
optimization.**

Raw dumps: `scratchpad/bench_008_raw.txt` (capture A), captured table above
(capture B). Measurement scripts: `scratchpad/bench_008_flipflop.py`,
`scratchpad/bench_008b_dutywrite.py`.

## Implementation Plan

**Approach**: Attempt the `pyOCD`/`gdb` read-of-live-counters path first
(per `.claude/rules/debugging.md`'s existing preconditions/workflow:
`pyocd list` to confirm one board connected, background the gdbserver,
drive `gdb` non-interactively in batch mode). Read `I2CBus::txnCount()`/
`errCount()`/`dumpRecent()`'s backing fields (the per-device `DeviceSlot`
table and the `TxnLog` ring) directly from the running firmware's static
`I2CBus` instance. Correlate timestamps against the known flip-flop phase
transitions (`REQUEST_DUE`/`COLLECT_DUE`) to build the per-phase breakdown.

If this proves impractical (e.g., halting to read memory perturbs the
measurement, or the 24-entry log ring wraps too fast to catch a clean
window), fall back to a minimal new dev-only diagnostic verb (the `DEV`
family, NOT the stale `DBG` family) exposing `dumpRecent()`/counters over
the wire for repeated sampling — document why the primary approach didn't
work if this fallback is used.

**Files to create/modify**: None expected (measurement-only). If the
fallback wire verb is needed: a small addition to `source/commands/
dev_commands.{h,cpp}` — kept minimal, diagnostic-only, `ROBOT_DEV_BUILD`-
gated like the rest of the `DEV` family.

**Testing plan**: N/A in the usual sense — this ticket's "test" is the
measurement session itself. If the fallback wire verb is added, it gets a
minimal smoke test (verb acks, returns sane data) but no behavior-change
tests are needed since nothing behavioral changes.

**Documentation updates**: None yet — ticket 009 is where the design doc
gets corrected (if that's the chosen outcome) or the optimization gets
documented.
