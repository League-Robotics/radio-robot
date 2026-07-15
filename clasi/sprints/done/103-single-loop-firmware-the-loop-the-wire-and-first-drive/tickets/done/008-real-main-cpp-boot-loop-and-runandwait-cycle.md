---
id: 008
title: "Real main.cpp \u2014 boot loop and runAndWait cycle"
status: done
use-cases:
- SUC-008
depends-on:
- '002'
- '004'
- '005'
- '006'
- '007'
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Real main.cpp — boot loop and runAndWait cycle

## Description

Replace `source/main.cpp` (the sprint-102 banner-only stub) with the real
single loop, wiring every module from tickets 002-007 together per the
archived plan's one-page main loop, implemented verbatim in shape:
`markTime()`/`sleepUntil()`/`runAndWait()` as the three timing primitives
the whole cycle is built from (`runAndWait(gap, body) == markTime(); body();
sleepUntil(mark, gap)`), a telemetry-emitting boot loop, then the main
cycle (motor1 request→settle(pump comms)→collect, telemetry emit during
motor1's clearance, motor2 request→settle(dispatch+deadman+drive.tick())
→collect, perception+odometry, pace sleep).

Depends on tickets 002 (hardened leaves), 004 (Comms/Deadman), 005
(Telemetry), 006 (Drive/Odometry), 007 (Preamble) — everything this ticket
wires together.

## Acceptance Criteria

- [x] `grep 'runAndWait\|sleepUntil' source/main.cpp` (or wherever the
      primitives/cycle body ultimately live, if split across files) shows
      the complete timing schedule: three `runAndWait` blocks + one
      `sleepUntil` pace call, matching the archived plan's schedule
      one-for-one (two encoder-settle windows, one post-duty-write
      clearance window, one cycle pace).
- [x] No device call sleeps or blocks on its own anywhere in the cycle —
      every required gap is a `runAndWait`/`sleepUntil` call (this is M1's
      fix, ticket 002, made real at the loop level).
- [x] Boot loop: `while (!preamble.done()) { preamble.step(); tlm.emit();
      uBit.sleep(kPreamblePace); }` — telemetry flows from power-on;
      commands are NOT consumed during boot (no `Comms::pump()` call in
      the boot loop).
- [x] Main loop dispatch: at most one decoded command is applied per
      cycle, via a `switch (cmd.take())` (or equivalent) covering
      `Twist`/`Config`/`Stop`/`None`; every path that applies a command
      acks via `tlm.ack(cmd.corrId)`.
- [x] `Deadman::expired()` is checked exactly once per cycle; on expiry,
      `drive.stop()` is called — no exceptions, no other path to stop
      being gated by the deadman.
- [x] `just build` produces a hex; the image flashes and boots (banner +
      identifies) — confirmed via `mbdeploy` (bench step, may run as part
      of this ticket or be deferred to ticket 010's own flash — this
      ticket's own acceptance is the BUILD succeeding; ticket 010 owns the
      full bench gate).
- [x] Construction order in `main()` matches `device_bus.h`'s own
      documented declaration-order rationale where applicable (bus before
      leaves, leaves before app/ modules that read them) — even though
      `DeviceBus` itself is gone, its comment's underlying reasoning
      (construction order matters) still applies to the flat construction
      list this ticket writes.

## Implementation Plan

**Approach**: This ticket is pure composition — it should introduce no new
logic of its own beyond the `markTime`/`sleepUntil`/`runAndWait`
primitives (small, mechanical, and used nowhere else, so implementing them
here is appropriate) and the command-dispatch switch (which decides WHAT a
decoded command does, genuinely new logic this ticket owns since no other
ticket's module makes that decision). Write the cycle body by
transcribing the archived plan's own pseudocode line-by-line, substituting
this sprint's actual class/method names (confirmed against tickets
002-007's real APIs, not assumed from the plan's illustrative naming).

**Files to create/modify**:
- `source/main.cpp` (replaced — the sprint-102 stub is fully superseded)
- Possibly a small `source/app/timing.h` for the
  `markTime`/`sleepUntil`/`runAndWait` primitives if they warrant their own
  header (this ticket's own call — inline in `main.cpp` is also acceptable
  if small enough).

**Testing plan**:
- Existing tests to run: the full `devices_*` + `wire_*` + `app/`-module
  suites from tickets 001-007 (this ticket's wiring should not regress any
  of them).
- New tests to write: none required at the unit level (this is a real
  `main()`, not host-buildable in the usual sense — its actual test IS the
  bench gate, ticket 010). If a host-buildable smoke test of the cycle
  body's call ORDER is feasible (e.g. a fake-device-backed dry run
  confirming the sequence, not real timing), it is a nice-to-have, not a
  required acceptance criterion.
- Verification command: `just build` (produces the hex); the `grep`
  command from the acceptance criteria above.

**Documentation updates**: the cycle's own inline comments should carry
forward the archived plan's "Properties to reason from" framing (the
brick's four timing constraints map one-to-one onto the three `runAndWait`
blocks plus the pace `sleepUntil`) so a future reader can verify the
schedule against real hardware constraints without re-deriving it.

## Completion Notes

`source/main.cpp` replaced with the real single loop. `markTime()`/
`sleepUntil()`/`runAndWait()` are file-local primitives built on
`system_timer_current_time()` (vendor `[ms]` call) + `uBit.sleep()` — the
archived plan's own "Properties to reason from" paragraph says the main
loop's settle waits themselves "yield via `uBit.sleep`", so the boot loop
and the main cycle share the same underlying primitive. `sleepUntil()`
always sleeps `>=1ms`, uniformly (never a zero-length "sleep"), not just on
the final pace call.

Timing constants: `kSettle = 4` / `kClear = 4` `[ms]`, ported from the
retired `DeviceBus::kEncoderSettleMs` (device_bus.h, git history
`88e04f1b^`) and corroborated by `NezhaMotor`/`Otos`'s own `4000us`
preClear/postClear windows (`nezha_motor.cpp`, `otos.h`'s `kBusClearance`);
`kCycle = 16` taken verbatim from the archived plan's own sketch comment
(`sleepUntil(cycleStart, kCycle); // pace to ~16ms`).

Cycle body transcribes the archived plan's pseudocode against the REAL
ticket 002-007 APIs (confirmed against the actual headers, not assumed):
`motorL.requestSample()`/`motorL.tick(nowUs)` (no separate
`collectAndControl` exists — `NezhaMotor::tick()` collects+controls+writes
in one 5-step call), `App::Cmd`'s `status`/`env.cmd_kind` read directly
(there is no `Cmd::take()` method — the switch reads `cmd.status`/
`cmd.env.cmd_kind` and relies on `cmd` being a fresh cycle-local variable
for the "at most once per cycle" bound, per the implementation plan's own
"substituting actual names" instruction).

New, ticket-owned logic: `toDeviceMotorConfig()` — converts
`Config::defaultMotorConfigs()`'s wire-plane `msg::MotorConfig` into the
`Devices::MotorConfig` `NezhaMotor`'s constructor needs. No existing
converter existed (`Devices::` and `msg::`/`Config::` are isolated from
each other by design), so this ticket added the one place both types are
reachable.

`ConfigDelta` dispatch (Step 7 Open Question 3): decodes, does not apply,
acks `ERR_UNIMPLEMENTED` — matches the architecture doc's own resolution
("schema now, runtime later" precedent), not a gap.

`Telemetry::Frame` is a single persistent local across cycles (not
per-cycle-local): encoder/vel/conn fields are staged in the `kClear`
window from the cycle's own fresh `motorL`/`motorR` samples; pose/otos
fields are staged at the END of the cycle (`applyOtosSample()` +
`odom.integrate()`, after `motorR.tick()`) for the NEXT cycle's emit —
matching `Telemetry`'s own "always carries the last staged snapshot"
contract and `applyOtosSample()`'s "reaches Telemetry before that cycle's
frame is built" doc comment.

Fault/event bits wired: `kFaultI2CSafetyNet` (from
`bus.clearanceSafetyNetCount()`), `kFaultWedgeLatch` (from
`motorL.wedged() || motorR.wedged()`), `kEventDeadmanExpired` (from
`deadman.expired()`), `kEventBootReady` (set once, after the boot loop
exits). `kFaultI2CNak` stays unwired (ticket 005's own doc: "no
per-transaction NAK aggregate exists at this scope" — still true here, not
a regression). No wire bit exists for "malformed comms frame count"
(`telemetry.proto`'s current fault_bits layout has no such bit — bits 0-2
are I2C-safety-net/wedge/I2C-NAK only) — `comms.malformedCount()` is
therefore NOT surfaced to telemetry this ticket; adding a bit is a
schema change (ticket 001's territory), out of this ticket's scope.

Drive-by style fix (separate commit, per team-lead instruction): renamed
ticket 007's `kPowerSettleUs`/`kMaxPreambleUs` (`source/app/preamble.h`,
`.cpp`) to `kPowerSettle`/`kMaxPreamble` with `// [us]` trailing tags.
Swept `source/app/` for other unit-suffixed identifiers — none found
beyond the pervasive `nowUs`/`startUs_`/`otosLastAttemptUs_`-style
parameter/field naming already established across `source/devices/`
(`NezhaMotor::tick(uint64_t nowUs)`, `Otos::tick(uint64_t nowUs)`,
`ColorSensorLeaf`/`LineSensorLeaf::beginStep(uint64_t nowUs)`, etc.) —
that convention predates sprint 103 (device-bus-tickets.md) and is used by
dozens of call sites across `devices/`, every `tests/sim/unit/*_harness.cpp`,
and this ticket's own `preamble.h`/`main.cpp` call sites into those same
APIs. Renaming it is a repo-wide sweep well outside this ticket's "drive-by
fix" scope (which named two specific ticket-007 constants) — left as a
pre-existing, out-of-scope naming inconsistency, not fixed here.

Verification: `just build-clean` → `MICROBIT.hex` produced (RAM 98.33%
used, FLASH 35.37% — RAM near-full is expected/by-design per project
knowledge, not a regression). `grep 'runAndWait\|sleepUntil'
source/main.cpp` shows exactly three `runAndWait` call sites (lines
229/234/259) plus the final `sleepUntil(cycleStart, kCycle)` pace call
(line 315), matching the canonical schedule. `uv run python -m pytest
tests/sim/unit -q` → 339 passed (devices_*/wire_*/app/-module suites all
green, no regression). `tests/unit/` has 130 pre-existing failures
(`test_protocol_binary_client.py`, `test_protocol_pose_fix.py`,
`test_serial_conn_binary_plane.py`, `test_bridge_pty_e2e.py`) — all
targeting wire arms ticket 001 already deleted (Decision 4, "tests/unit/
breakage expected"); none touch `preamble`/`main.cpp`, confirmed
unrelated to this ticket's changes.

NOT done this ticket (explicitly out of scope per team-lead instruction):
robot flash/bench verification — deferred to ticket 010's own bench gate.
No host-buildable smoke test of the cycle body's call order was added
(the ticket's own testing plan marks this a nice-to-have, not required).
