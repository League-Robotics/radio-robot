---
id: '003'
title: 'I2C safety-net fault bit: diagnose whether bit 6 reflects live bus health
  or a latched boot artifact'
status: open
use-cases: [SUC-071]
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

- [ ] Raw `MicroBitI2CBus::clearanceSafetyNetCount()` value (not just the
      derived `flags` bit) traced via pyOCD/DBG
      (`.claude/rules/debugging.md`) across: (a) an idle window after
      boot, (b) a driving window, (c) boot/`Preamble` specifically vs.
      steady-state cycling. Trace data recorded in this ticket.
- [ ] The trace determines ONE of:
      - **The count keeps climbing during driving** → a real, ongoing
        bus-timing defect remains despite 118-001's restore. Diagnose
        the specific trigger (candidates per the source issue: OTOS burst
        read in the pace block; the 40ms DutyPredictor/write-throttle
        margin at 35ms; the interleave fix genuinely incomplete for the
        real bus schedule) and ship a fix that makes the bit clear during
        driving on hardware.
      - **The count is flat after one early (boot/`Preamble`) increment**
        → the bit is a latched, cumulative artifact unrelated to ongoing
        bus health. No code change ships for the bit's derivation unless
        a stakeholder wants `resetStats()` wired in (out of this ticket's
        scope unless the diagnosis says otherwise); instead, 118-001's
        own acceptance record is corrected to state the bit's true
        behavior, citing this ticket's trace as evidence.
- [ ] Whichever outcome: `src/firm/app/DESIGN.md`'s `flags` bit-string
      §4 entry for bit 6 (already drafted DRAFT text in this sprint's
      design overlay) is replaced with the confirmed, non-DRAFT
      description and applied to the canonical doc at sprint close.
- [ ] If a fix ships, it is verified on real hardware (bit reads clear
      during driving) before this ticket is marked done — no guessed fix
      ships without on-chip confirmation.
- [ ] If no fix ships, this ticket's own record states explicitly why
      (the confirmed latch theory) so a future reader doesn't re-open
      this as an unexplained gap.

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
