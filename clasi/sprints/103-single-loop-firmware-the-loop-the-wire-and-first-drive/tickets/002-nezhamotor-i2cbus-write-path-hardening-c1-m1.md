---
id: '002'
title: NezhaMotor/I2CBus write-path hardening (C1, M1)
status: open
use-cases: [SUC-002]
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

- [ ] `writeRawDuty()` commits `lastWrittenPct_`/`lastWriteTimeUs_` ONLY
      when `bus_.write()` returns `kOk`/`MICROBIT_OK`; on any other status
      the write-on-change gate does NOT suppress a retry of the same value
      next tick (stop stays throttle-exempt, matching its existing
      exemption).
- [ ] No remaining `while(clockUs()<deadline){}`-style busy-spin with no
      yield anywhere in the write/clearance path. `I2CBus`'s per-device
      `readyAt` stamps remain as a non-blocking, sleep-not-spin safety net
      (`clear()`'s existing peek-only semantics are the model — do not add
      a new spin).
- [ ] The safety net raises a telemetry fault bit (wired against ticket
      001's `Telemetry.fault_bits`, via whatever narrow signal `I2CBus`
      exposes for "a clearance safety net fired") the first time it fires
      after this ticket's own change — proving it is now detectable rather
      than silently swallowed. (If ticket 001 has not yet landed when this
      ticket executes, add the narrow signal to `I2CBus` now and wire the
      actual `fault_bits` write in ticket 005/008 — note this explicitly
      in completion notes rather than silently dropping the requirement.)
- [ ] `devices_motor_harness.cpp` and every other `devices_*` unit test
      stay green.
- [ ] `clasi/issues/nezha-motor-write-path-hardening.md` is referenced by
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
