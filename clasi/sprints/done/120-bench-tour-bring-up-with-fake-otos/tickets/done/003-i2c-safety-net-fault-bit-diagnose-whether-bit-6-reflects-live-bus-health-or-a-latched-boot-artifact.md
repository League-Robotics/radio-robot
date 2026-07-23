---
id: '003'
title: 'I2C safety-net fault bit: diagnose whether bit 6 reflects live bus health
  or a latched boot artifact'
status: done
use-cases:
- SUC-071
depends-on: []
github-issue: ''
issue: bench-i2c-safety-net-fault-asserts-every-cycle.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# I2C safety-net fault bit: diagnose whether bit 6 reflects live bus health or a latched boot artifact

## Description

`flags` bit 6 (`kFlagFaultI2CSafetyNet`) is set on every telemetry frame
on real hardware — idle AND driving (45/45, 23/23 measured) — directly
contradicting 118-001's own before/after acceptance claim that the
loop-schedule restore (`kSettle`/`kClear` back to their genuine 4ms
budget) would clear this bit while driving. It did not.

This ticket is diagnosis-first, not a guessed fix (per sprint.md's
Architecture Decision 4): reading the source shows
`MicroBitI2CBus::clearanceSafetyNetCount()` is a monotonically
non-decreasing counter, the bit is derived as `count() > 0`, and
`resetStats()` (which zeroes the counter) is declared but never called
anywhere in production firmware
(`grep -rn "resetStats\b" src/firm/` confirms zero call sites outside its
own declaration/definition). This is a strong, testable hypothesis — a
single early (boot/`Preamble`) trip could latch the bit permanently for
the rest of the session with no ongoing bus-health signal at all — but it
is NOT yet confirmed against real hardware timing, and 118-001 already
made one unconfirmed prediction about this exact bit that turned out
wrong. Do not repeat that pattern.

This ticket can run parallel-independent of tickets 001/002 (no shared
files, no shared state) but executes serially per this sprint's
`worktree: false` — no execution-order dependency either way.

## Acceptance Criteria

- [x] Raw `MicroBitI2CBus::clearanceSafetyNetCount()` value (not just the
      derived `flags` bit) traced via pyOCD/DBG
      (`.claude/rules/debugging.md`) across: (a) an idle window after
      boot, (b) a driving window, (c) boot/`Preamble` specifically vs.
      steady-state cycling. Trace data recorded in this ticket (see
      "Hardware Trace" section below).
- [x] The trace determines the actual behavior — **a third outcome,
      distinct from both outcomes this ticket anticipated** (see
      "Diagnosis" below for the full reasoning):
      - It is NOT "flat after one early boot increment" — the raw
        counter climbs continuously for as long as the robot runs, both
        idle and driving (measured: 97 at ~4.6s post-flash, 167 at ~8s
        post-reset, Δ243/~14s and Δ148/~8.6s across two independent idle
        brackets — never flat).
      - It is also NOT the loop-schedule/motor-path defect the OTHER
        anticipated outcome named — an exact 1:1 accounting (both idle
        brackets: the safety-net delta matches HALF of `Devices::Otos`'s
        own transaction-count delta, exactly, in each bracket) shows
        100% of trips are caused by `Devices::Otos`'s own self-contained
        register-read pattern (write-then-immediate-read, no scheduled
        loop gap), completely independent of drive state, with ZERO
        trips attributable to the motor path 118-001's restore actually
        protects. The interleave fix is confirmed fully effective for
        its real target.
      - No code fix ships (see "Diagnosis" for why every candidate fix
        requires a design decision beyond this ticket's authorized
        scope). Filed as
        `clasi/issues/i2c-safety-net-bit-conflates-otos-settle-wait-with-loop-schedule-health.md`
        for a future sprint/stakeholder decision.
- [x] Whichever outcome: `src/firm/app/DESIGN.md`'s `flags` bit-string
      §4 entry for bit 6 (already drafted DRAFT text in this sprint's
      design overlay) is replaced with the confirmed, non-DRAFT
      description — done in this sprint's design overlay
      (`clasi/sprints/120-bench-tour-bring-up-with-fake-otos/design/DESIGN.md`
      §4); applied to the canonical doc at sprint close per this
      sprint's overlay convention. `src/firm/app/telemetry.h`'s own bit
      6 doc comment (source code, not overlay-gated) corrected directly.
- [x] No fix ships (N/A — no fix to verify on hardware).
- [x] This ticket's own record states explicitly why: NOT the
      anticipated "confirmed latch theory" (that was refuted by hardware
      evidence) — see "Diagnosis" below for the actual, hardware-proven
      root cause and why it does not warrant a fix in this ticket's
      scope.

## Hardware Trace

Robot "tovez", `/dev/cu.usbmodem2121102`, real (non-`FAKE_OTOS`) build,
v0.20260723.2 base + this sprint's tickets 1/2. Method: `arm-none-eabi-gdb
-q --batch` against a backgrounded `pyocd gdbserver -t nrf52833 --persist`
(per `.claude/rules/debugging.md`), briefly halting to `print
'main()::bus'.clearanceSafetyNetCount_` (and the per-device `txnCount` for
the left motor, addr 0x10, and OTOS, addr 0x17, via `bus.devices_[0]`/
`devices_[1]`), then resuming (`monitor go`) and detaching. All reads
below are from clean, verified sessions (`Successfully halted device`
confirmed, no connection-error output) — a handful of samples taken after
the SWD link degraded (`Unexpected ACK '0'`, a known pyOCD/USB flakiness
per `.claude/rules/debugging.md`'s Gotchas) read back stale zeros and were
discarded per the systematic-debugging evidence-gathering discipline, not
used as data.

| When | `clearanceSafetyNetCount_` | motor(L) `txnCount` | OTOS `txnCount` |
|---|---|---|---|
| ~4.6s post-flash (near boot) | 97 | 327 | 162 |
| ~8s post an SWD-triggered reset | 167 | 609 | 302 |
| Idle bracket 1, start | 725 | 2841 | 1418 |
| Idle bracket 1, +~14s (idle, no drive commands) | 968 (Δ243) | 3810 (Δ969) | 1904 (Δ486 → 243 bursts) |
| Idle bracket 2, start (fresh pyOCD session) | 443 | 1711 | 854 |
| Idle bracket 2, +~8.6s (idle) | 591 (Δ148) | 2304 (Δ593) | 1150 (Δ296 → 148 bursts) |

Both idle brackets show an EXACT match: Δ`clearanceSafetyNetCount_` ==
Δ(OTOS `txnCount`)/2, i.e. every OTOS burst read (one write + one read =
2 `txnCount` increments) trips the safety net exactly once, with zero
residual trips left over to attribute to the motor. A separate, clean
(no concurrent gdb) 6/6-passing `twist_drive.py` run (`--v-x 150
--duration 5000/6000/8000`) confirmed the robot drives normally
(encoders 0→896/874 over one such run) while a serial-only (gdb-free)
telemetry peek immediately after showed `flags` bit 6 still set
(`0x28db`/`0x48db`, both idle and driving) — consistent with the bit
being driven by Otos's own load-independent read cadence, not by drive
state. (Note: driving-window gdb sampling was attempted directly but
abandoned after it was found to disrupt serial command ack timing — a
halted core cannot service the UART per debugging.md's own documented
caveat — causing spurious ack-timeout FAILs in the bench script; the
idle-window brackets plus the code-level proof below are sufficient and
do not carry this risk.)

## Diagnosis

**Root cause (code + hardware, fully confirmed):**
`Devices::Otos::readPositionVelocity()` (`src/firm/devices/otos.cpp`,
called from `Otos::tick()`, itself called unconditionally every cycle
from `RobotLoop::cycle()`'s trailing `kPace` block regardless of
`moveQueue_.active()`) issues a register-select `bus_.write()`
(`postClear=kBusClearance=4000us`) immediately followed by a
`bus_.read()` (`preClear=kBusClearance`) on the SAME device address, with
**no intervening loop-scheduled gap** — the two calls are back-to-back
in the same function. `MicroBitI2CBus::waitForClearance()`
(`src/firm/devices/microbit_i2c_bus.cpp`) therefore ALWAYS finds `now <
entryDeadline` on the read half and increments
`clearanceSafetyNetCount_`, unconditionally, on every single Otos burst
read — at Otos's own `kReadPeriod=20000us` cadence, independent of
`moveQueue_.active()`, independent of 118-001's `kSettle`/`kClear`
restore, and independent of the loop schedule entirely.

Contrast: `NezhaMotor::requestEncoder()`/`collectEncoder()`
(`src/firm/devices/nezha_motor.cpp`) is a genuine split-phase pattern —
`requestEncoder()`'s `postClear=4000` sets the device's `readyAt`, and
`collectEncoder()` (the read half) is called from a LATER cycle phase,
across a real scheduled gap (`kSettle`, `robot_loop.cpp`). This is
exactly the case `waitForClearance()`'s own doc comment describes ("the
loop was supposed to own this gap... count the trip") — and the
hardware trace above shows this path contributes ZERO trips in either
measured window. 118-001's restore is fully effective for what it was
actually built to protect.

**Why this ticket ships no code fix.** The two outcomes this ticket's
own acceptance criteria anticipated were (A) a genuine, ongoing
loop-schedule/motor-path defect — ship a fix that clears the bit during
driving, or (B) a one-shot boot latch — correct the record, no fix. The
hardware evidence fits neither cleanly: the counter is NOT flat (rules
out B literally), but the "defect" is not a loop-schedule regression
either (rules out the motor-side reading of A — 118-001's fix is
provably clean). The true cause is `Devices::Otos`'s own necessary,
correctly-functioning bus-settle wait, which the shared safety-net
counter was never designed to distinguish from a genuine caller-side
schedule violation. Making the bit literally "clear during driving"
requires one of:

1. Redesigning `Otos`'s own I2C register-read pattern to cross a real
   scheduled loop gap (mirroring the motor's split-phase shape) — a real
   hardware-timing change to a currently-working, bench-proven sensor
   path, well outside this ticket's authorized file scope
   (`microbit_i2c_bus.{h,cpp}`/`robot_loop.cpp`) and risky to guess
   without its own dedicated bench-verified ticket.
2. Introducing a policy/design decision into the fault-bit's derivation
   (e.g., per-device trip accounting excluding OTOS, or a caller-intent
   "self-contained wait" flag through `I2CBus::write()`/`read()`) — a
   genuine architecture/stakeholder call about what the fault bit should
   mean, not something to guess after 118-001 already guessed wrong once
   about this exact bit.

Per this ticket's own Description ("diagnosis-first, not a guessed
fix"), no fix ships. The finding is filed as
`clasi/issues/i2c-safety-net-bit-conflates-otos-settle-wait-with-loop-schedule-health.md`
with three concrete candidate fixes for a future sprint to pick up.
Corrected records: `src/firm/app/telemetry.h`'s bit 6 doc comment, this
sprint's design overlay (`design/DESIGN.md` §4), this sprint's own issue
file, and 118-001's own ticket + source issue
(`clasi/sprints/done/118-loop-schedule-truth-firmware-loop-reorder-sim-cadence-parity/`).

## Implementation Plan

### Approach

1. Build the current firmware (`just build-clean`) and flash it
   (`mbdeploy deploy <robot-UID> --hex MICROBIT.hex`; UID
   `9906360200052820a8fdb5e413abb276000000006e052820`; APPROTECT
   auto-mass-erase expected/normal).
2. Start a pyOCD GDB server (`just debug`, backgrounded, logged) and
   attach non-interactively (`arm-none-eabi-gdb -q --batch`, per
   `.claude/rules/debugging.md`'s Agent Guidance — never an interactive
   REPL).
3. Set a breakpoint (or use `pyocd commander`'s `read32`/peripheral
   inspection) to read `MicroBitI2CBus`'s `clearanceSafetyNetCount_`
   member directly (not just the derived telemetry bit) at multiple
   points: immediately post-boot (before `Preamble::done()`), during an
   idle window, and during an active drive commanded via `move_twist()`
   or the bench MOVE scripts.
4. Optionally, use ticket 002's `FAKE_OTOS` build (if it has landed) as a
   convenient way to compare "OTOS tick present" vs. "OTOS tick skipped"
   — this is a nice-to-have cross-check, not a dependency; the trace in
   step 3 can also be done by temporarily patching out the real OTOS
   tick call for a one-off comparison build if ticket 002 hasn't landed
   yet.
5. Based on the trace, either implement and verify a fix, or write the
   corrected characterization and file/update the acceptance-claim
   correction against 118-001's own record.
6. Always kill the backgrounded pyOCD server when done (a lingering
   server blocks the next session and `mbdeploy deploy`).

### Files to Create/Modify

- `src/firm/devices/microbit_i2c_bus.{h,cpp}` — IF a real fix is needed
  (e.g., wiring `resetStats()` in, or narrowing what increments the
  counter).
- `src/firm/app/robot_loop.cpp` — IF the fix belongs at the call site
  (e.g., deriving the bit differently) rather than inside `I2CBus`
  itself.
- `src/firm/app/DESIGN.md` (canonical) — apply this sprint's overlay
  edit (replace DRAFT bit-6 text with the confirmed conclusion) at
  sprint close.
- 118-001's own ticket/sprint record — corrected acceptance claim, if
  the diagnosis is "latched artifact, no fix."

### Testing Plan

- Hardware (required, this IS the diagnosis): pyOCD/DBG trace of the raw
  counter idle vs. driving vs. boot, recorded in this ticket. If a fix
  ships, a hardware re-run confirming the bit now clears during driving.
- No sim-level test can substitute here — this is real-bus timing
  behavior the sim's ideal I2C model does not reproduce.

### Documentation Updates

- `src/firm/app/DESIGN.md` — apply this sprint's overlay diff (bit 6
  description), replacing the DRAFT hedge with the confirmed conclusion.
- 118-001's acceptance record — corrected claim, if applicable.
