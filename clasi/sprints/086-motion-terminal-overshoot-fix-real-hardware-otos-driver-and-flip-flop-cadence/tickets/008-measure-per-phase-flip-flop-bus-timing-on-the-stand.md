---
id: "008"
title: "Measure per-phase flip-flop bus timing on the stand"
status: open
use-cases: [SUC-008]
depends-on: []
github-issue: ""
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

- [ ] A per-phase timing breakdown (request write, settle wait, collect
      read, scheduling/main-loop overhead) is captured and recorded for at
      least one 2-port closed-loop motion session reproducing the 079-006
      measured ~19-22 ms (~44-52 Hz) per-motor cadence.
- [ ] The double-counted-clearance hypothesis is explicitly confirmed or
      refuted with data — not left as a guess.
- [ ] No firmware/production code change in this ticket (measurement only).
- [ ] The measurement approach used (pyOCD/gdb read vs. a fallback wire
      verb, if that path was needed) is documented in this ticket's
      completion notes along with the raw/summarized data, so ticket 009 has
      a concrete basis to decide from.

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
