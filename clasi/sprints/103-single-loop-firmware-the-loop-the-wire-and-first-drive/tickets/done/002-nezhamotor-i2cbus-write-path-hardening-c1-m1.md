---
id: '002'
title: NezhaMotor/I2CBus write-path hardening (C1, M1)
status: done
use-cases:
- SUC-002
depends-on: []
github-issue: ''
issue:
- nezha-motor-write-path-hardening.md
- single-loop-firmware-p3-p7-continuation.md
completes_issue:
  nezha-motor-write-path-hardening.md: true
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# NezhaMotor/I2CBus write-path hardening (C1, M1)

## Description

Fix two defects the 2026-07-13 code review found in code this sprint
KEEPS unchanged otherwise (`clasi/issues/nezha-motor-write-path-hardening.md`):

- **C1 (critical)**: `NezhaMotor::writeRawDuty()`
  (`source/devices/nezha_motor.cpp:325-372`) commits
  `lastWrittenPct_`/`lastWriteTimeUs_` unconditionally, ignoring
  `bus_.write()`'s status. A transient NAK on a **stop** write (pct==0) is
  latched as "already written," permanently defeating the watchdog's
  "re-assert Neutral every cycle" robustness — `armoredWrite(0)`
  early-returns forever once `pct == lastWrittenPct_ == 0`, even though the
  wheel is still physically spinning.
- **M1 (major)**: `I2CBus`'s inter-transaction clearance waits
  (`i2c_bus.cpp:67-68,112-113`) are hard `while(clockUs()<deadline){}`
  busy-spins with no yield — up to ~4ms of scheduler-blocking spin nearly
  every cycle, violating the project's "the loop must yield" rule
  (radio/serial starve otherwise).

This ticket is independent of every other ticket in this sprint — it is a
correctness fix to already-KEPT code, not new architecture — and should
land early so the hardened leaves are what every later `source/app/`
ticket builds on.

## Acceptance Criteria

- [x] `writeRawDuty()` commits `lastWrittenPct_`/`lastWriteTimeUs_` ONLY
      when `bus_.write()` returns `kOk`/`MICROBIT_OK`; on any other status
      the write-on-change gate does NOT suppress a retry of the same value
      next tick (stop stays throttle-exempt, matching its existing
      exemption).
- [x] No remaining `while(clockUs()<deadline){}`-style busy-spin with no
      yield anywhere in the write/clearance path. `I2CBus`'s per-device
      `readyAt` stamps remain as a non-blocking, sleep-not-spin safety net
      (`clear()`'s existing peek-only semantics are the model — do not add
      a new spin).
- [x] The safety net raises a telemetry fault bit (wired against ticket
      001's `Telemetry.fault_bits`, via whatever narrow signal `I2CBus`
      exposes for "a clearance safety net fired") the first time it fires
      after this ticket's own change — proving it is now detectable rather
      than silently swallowed. (If ticket 001 has not yet landed when this
      ticket executes, add the narrow signal to `I2CBus` now and wire the
      actual `fault_bits` write in ticket 005/008 — note this explicitly
      in completion notes rather than silently dropping the requirement.)
- [x] `devices_motor_harness.cpp` and every other `devices_*` unit test
      stay green.
- [x] `clasi/issues/nezha-motor-write-path-hardening.md` is referenced by
      this ticket's `issue:` frontmatter (already set) and is eligible to
      move to done once this ticket closes.

## Implementation Plan

**Approach**: Two narrowly-scoped, independent edits in already-understood
files. For C1: guard the `lastWrittenPct_`/`lastWriteTimeUs_` assignment in
`writeRawDuty()` behind the `bus_.write()` status check; do not change the
function's other behavior (slew limiting, direction/speed encoding). For
M1: the REAL fix for the general case is architectural (the single loop
owns clearance gaps via `runAndWait`, ticket 008) — this ticket's own scope
is narrower: confirm `I2CBus::clear()`'s existing non-spinning peek is what
remains as the safety net, and that nothing in `i2c_bus.cpp`'s `write()`/
`read()` entry-spin exceeds a bounded, yield-free window before falling
back to a peek-and-fault-bit posture rather than an unbounded spin. Read
`i2c_bus.cpp:67-68,112-113` closely before editing — the entry-spin at
those lines is the ACTUAL busy-wait (waiting for `readyAt`), distinct from
`clear()`'s already-safe peek; only the former needs to change.

**Files to create/modify**:
- `source/devices/nezha_motor.cpp` — `writeRawDuty()` (C1).
- `source/devices/i2c_bus.h`/`i2c_bus.cpp` — clearance entry-spin (M1) +
  a narrow fault-signal accessor if ticket 001 hasn't landed yet.

**Testing plan**:
- Existing tests to run: `devices_motor_harness.cpp` and sibling
  `devices_*` harnesses (host-buildable, `HOST_BUILD` scripted-fake I2C).
- New tests to write: a scripted-fake test proving a NAK'd `pct==0` write
  is retried next tick (not permanently suppressed) — use `I2CBus::
  scriptWrite()` to inject one failing status, confirm `writeRawDuty(0)`
  is attempted again on the following call. A second test proving the
  entry-spin's bound (if any residual bounded wait remains) never exceeds
  its documented ceiling using the scripted clock (`I2CBus::setClock()`/
  `advanceClock()`).
- Verification command: `uv run python -m pytest tests/sim/unit/ -k device_bus_or_motor`
  (adjust the `-k` filter to the actual test file names once located) plus
  the direct C++ harness build/run per the project's existing
  `devices_*` test invocation.

**Documentation updates**: none beyond this ticket's own completion notes
and the issue's closure.

## Completion Notes

**C1 fix (`source/devices/nezha_motor.cpp`).** `writeMotorRun()` now
returns the CODAL status from `bus_.write()` (was `void`) — the *only*
signature change in the file, applied at its single call site. In
`writeRawDuty()`, the `lastWriteTimeUs_`/`lastWrittenPct_` commit was moved
to *after* the bus write and gated on `status == kOk`. Slew clamping still
reads `lastWrittenPct_` as of before the attempt, so a NAK'd write
correctly leaves the slew base at the last value that *actually* landed —
no other behavior (write-on-change, throttle, slew cap, coast-at-zero
exemption, the -128 first-write sentinel) changed. `writeMotorRun` stayed
`private`; no header surface beyond the return-type change.

**M1 fix (`source/devices/i2c_bus.{h,cpp}`, `i2c_bus_host.cpp`).** Both
forks' `write()`/`read()` entry-side `while(clockUs()<deadline){}` spins
(4 sites total, i2c_bus.cpp:67-68/112-113 and the HOST_BUILD mirror) are
replaced by a shared-declaration, per-fork-implemented private helper,
`waitForClearance(entryDeadline)`:
- Real fork: if called early, bumps the new `clearanceSafetyNetCount_`
  counter and calls `fiber_sleep()` for the shortfall (rounded UP to whole
  ms) — the *same* cooperative vendor primitive `clock.h`'s `Sleeper`
  wraps. This was a deliberate choice over a pure fault-and-proceed (no
  wait at all): the issue file's own words settle it —
  `nezha-motor-write-path-hardening.md`'s M1 fix direction says "the
  I2CBus per-device readyAt stamps remain only as a **sleep**-not-spin
  safety net" — and skipping the wait entirely on real hardware, before
  ticket 008 lands, would reintroduce the exact wedge condition
  `docs/knowledge/2026-07-04-encoder-wedge.md` documents (motor2's request
  write currently *does* arrive ~4ms early almost every cycle, immediately
  after motor1's duty write stamps the SAME address's readyAt — both
  motors share I2C address 0x10). `fiber_sleep()` yields to the scheduler
  (serial/radio fibers run), unlike the old spin, while still honoring the
  real vendor clearance requirement.
- HOST_BUILD fork: no fiber scheduler or wall clock to sleep against, so
  it jumps the fake clock directly to `entryDeadline` in one step (was: a
  1us-per-iteration self-advancing loop). Same observable end-state every
  pre-existing scripted scenario already asserted (fake clock lands
  at/after the deadline) — only the *mechanism* changed from a loop to a
  single jump, plus the new counter bump.

**Fault-bit wiring: counter added now, `Telemetry.fault_bits` write
deferred to ticket 005 (per the ticket's own conditional).** Ticket 001
landed before this ticket executed and already reserved
`fault_bits` bit 0 for exactly this ("I2CBus readyAt clearance safety-net
trip (ticket 002/005)" — `telemetry.proto`/ticket 001's completion notes).
However `source/app/` (where `Telemetry` gets populated from device state)
does not exist yet — only `source/devices/` does — so this ticket cannot
literally set a bit in a `Telemetry` struct. It exposes the narrow signal
instead: `I2CBus::clearanceSafetyNetCount() const`, a monotonic counter
bumped once per early-call trip, reset by `resetStats()`. Ticket 005 reads
this counter (or a derived edge/level) to set `fault_bits` bit 0.

**Tests.** `devices_i2c_bus_harness.cpp` gained scenario 9
(`scenarioEarlyCallBumpsClearanceSafetyNetCounter`): a call arriving before
a stamped `readyAt` bumps the counter exactly once and the fake clock still
lands at/after the deadline (proving the "sleep the shortfall in one jump,
never a spin" HOST_BUILD mechanism); an already-clear call never bumps it.
`devices_motor_harness.cpp` gained scenario 10
(`scenarioNakedStopWriteIsRetriedNextTickNotLatched`): establishes a
nonzero `appliedDuty()` via a successful write, then scripts a NAK'd stop
(`pct==0`) write — `appliedDuty()` stays at the previous nonzero value (not
falsely reported as stopped), `errCount()` records exactly one error, and
the identical stop target is retried (not suppressed by write-on-change)
on the following tick, this time succeeding.

**Test results:** `c++ -std=c++20 -Wall -Wextra -DHOST_BUILD -I source`
compiles both new/modified harnesses and both `I2CBus` forks clean;
`devices_i2c_bus_harness` and `devices_motor_harness` binaries both report
`OK` (0 failures) run directly. `uv run python -m pytest tests/sim/unit/ -q`:
**334 passed** — identical count to ticket 001's baseline, confirming no
regression anywhere else in the domain. `just build` (full ARM firmware,
real non-HOST_BUILD fork of both touched files): succeeds clean —
`Built target MICROBIT`, flash 27.84%, RAM 98.33% (normal — RAM is always
near-full by design per project convention, not a regression signal).
`tests/unit/`'s pre-existing 121 host failures (ticket 001's documented,
Decision-4 `envelope_pb2` breakage) were left untouched, as instructed.

**Surprises / notes for downstream tickets:**
1. The literal ticket-text phrase "clear()'s existing peek-only semantics
   are the model — do not add a new spin" reads, in isolation, as if the
   fix should skip waiting entirely (peek + fault-bit + proceed, no wait).
   The linked issue file's own fix-direction text is more precise ("a
   **sleep**-not-spin safety net") and was treated as authoritative over
   the shorter ticket paraphrase, specifically to avoid reintroducing the
   documented encoder-wedge failure mode on real hardware between this
   ticket landing and ticket 008 (which will make the loop pre-emptively
   own every gap, at which point this safety net should rarely/never trip
   in practice). Flagging in case a reviewer reads the ticket text as
   requiring a zero-wait posture instead.
2. No hardware bench verification was performed for this ticket — it is a
   pure host-testable leaf-behavior fix (write-status gating, clearance
   timing mechanism) with no protocol/wire surface change, matching the
   ticket's own testing plan (host harnesses + `just build` compile
   check only, no bench-gate item in its acceptance criteria). The next
   ticket that exercises the motor/bus leaves on the stand (008/010) will
   be the first real-hardware proof of this change.
