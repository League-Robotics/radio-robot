---
id: '003'
title: DBG debug message channel (bench/Sim only), exception-proof host handler
status: open
use-cases: [SUC-003]
depends-on: []
github-issue: ''
issue: 05-dbg-debug-message-channel-for-bench-and-sim.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# DBG debug message channel (bench/Sim only), exception-proof host handler

## Description

Landed early because the duty-sweep and calibration work in tickets 006
and 007 wants it. Stakeholder, 2026-07-31: *"Can we add another response
field so that you can send debug messages?... Let's go make a DBG. It
only needs to get compiled in when you're on the bench or in SIM."*

1. `src/protos/commands.proto` — `DBG = 18 [(binary) = false];` (cleartext
   verb, the next free slot after `HELP` at 17), regenerated into
   `wire_commands.py`, `commands_pb2.py`, `src/firm/messages/commands.h`.
2. `src/firm/app/debug.h`/`debug.cpp` — `App::debugf()`, `DBG_EVERY(n, ...)`,
   `DBG_MILLI(x)`. No-op unless `ROBOT_DEBUG`; `HOST_BUILD` implies it, so
   Sim always has it and the shipped ARM build does not. `DBG_MILLI`
   exists because newlib-nano has no printf float support — all debug
   formatting goes through integer milli-units.
3. `App::setDebugSink()`, wired to `App::Comms` on the robot and the sim
   harness's comms.
4. Host: `serial_conn.py` routes DBG to its own `on_debug` callback,
   **never raising**; `SimLoop.drain_debug_lines()` on the Sim side.

## Acceptance Criteria

- [ ] ARM release build: `DBG` compiles out entirely — grep-verified, and
      confirm no flash-size regression (binary-size comparison
      before/after).
- [ ] Sim and bench builds: `debugf()` lines arrive host-side and appear
      in the TestGUI console.
- [ ] A malformed or oversized DBG line does **not** disturb telemetry
      frame delivery — regression test for the `_log` NameError that
      killed a reader thread mid-session in the abandoned prior attempt;
      the host handler must be exception-proof by construction (wrap the
      whole `on_debug` dispatch, never let it propagate into the reader
      loop).

## Testing

- **Existing tests to run**: firmware pytest tiers (ARM-build DBG-absence
  check), `uv run python -m pytest`.
- **New tests to write**: a host test that feeds `serial_conn.py`'s
  reader a malformed/oversized DBG line and asserts telemetry frame
  delivery continues uninterrupted; a Sim test that `debugf()` output
  reaches `SimLoop.drain_debug_lines()`.
- **Verification command**: `uv run pytest`.

## Implementation Plan

- **Approach**: reuse the abandoned session's own design (it already
  earned its keep once — exposed a stalled-Move-reports-success defect
  the first time it ran) rather than redesigning; the risk to guard
  against on re-land is specifically the reader-thread exception, so add
  the malformed-line test before anything else.
- **Files to create/modify**: `src/protos/commands.proto` (+ regenerated
  `wire_commands.py`, `commands_pb2.py`, `src/firm/messages/commands.h`),
  `src/firm/app/debug.h` (new), `src/firm/app/debug.cpp` (new),
  `src/firm/app/comms.{h,cpp}` (sink wiring),
  `src/host/robot_radio/robot/serial_conn.py`, the Sim harness's comms
  equivalent, TestGUI console panel.
- **Documentation updates**: note in `src/firm/app/DESIGN.md` (or
  equivalent) that `DBG` is compiled-conditional and why (flash cost,
  wire traffic) — this is exactly the kind of load-bearing comment issue
  01 wants kept.
