---
id: '008'
title: "Real main.cpp — boot loop and runAndWait cycle"
status: open
use-cases: [SUC-008]
depends-on: ['002', '004', '005', '006', '007']
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

- [ ] `grep 'runAndWait\|sleepUntil' source/main.cpp` (or wherever the
      primitives/cycle body ultimately live, if split across files) shows
      the complete timing schedule: three `runAndWait` blocks + one
      `sleepUntil` pace call, matching the archived plan's schedule
      one-for-one (two encoder-settle windows, one post-duty-write
      clearance window, one cycle pace).
- [ ] No device call sleeps or blocks on its own anywhere in the cycle —
      every required gap is a `runAndWait`/`sleepUntil` call (this is M1's
      fix, ticket 002, made real at the loop level).
- [ ] Boot loop: `while (!preamble.done()) { preamble.step(); tlm.emit();
      uBit.sleep(kPreamblePace); }` — telemetry flows from power-on;
      commands are NOT consumed during boot (no `Comms::pump()` call in
      the boot loop).
- [ ] Main loop dispatch: at most one decoded command is applied per
      cycle, via a `switch (cmd.take())` (or equivalent) covering
      `Twist`/`Config`/`Stop`/`None`; every path that applies a command
      acks via `tlm.ack(cmd.corrId)`.
- [ ] `Deadman::expired()` is checked exactly once per cycle; on expiry,
      `drive.stop()` is called — no exceptions, no other path to stop
      being gated by the deadman.
- [ ] `just build` produces a hex; the image flashes and boots (banner +
      identifies) — confirmed via `mbdeploy` (bench step, may run as part
      of this ticket or be deferred to ticket 010's own flash — this
      ticket's own acceptance is the BUILD succeeding; ticket 010 owns the
      full bench gate).
- [ ] Construction order in `main()` matches `device_bus.h`'s own
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
