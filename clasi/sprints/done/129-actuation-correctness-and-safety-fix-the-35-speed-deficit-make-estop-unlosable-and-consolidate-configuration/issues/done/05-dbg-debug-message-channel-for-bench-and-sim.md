---
status: done
priority: medium
sprint: '129'
tickets:
- 129-003
---

# DBG: a firmware->host debug message channel, compiled in only for bench and Sim

Stakeholder, 2026-07-31: *"Can we add another response field so that you can
send debug messages? I think we've got budget for returning messages. Let's go
make a DBG. It only needs to get compiled in when you're on the bench or in
SIM."*

## What was built (and is being abandoned with the rest)

- `src/protos/commands.proto` — `DBG = 18 [(binary) = false];` (cleartext verb),
  regenerated into `wire_commands.py`, `commands_pb2.py` and
  `src/firm/messages/commands.h`.
- `src/firm/app/debug.h` / `debug.cpp` — `App::debugf()`, `DBG_EVERY(n, ...)`,
  `DBG_MILLI(x)`. No-op unless `ROBOT_DEBUG`; `HOST_BUILD` implies it, so Sim
  always has it and the shipped ARM build does not.
- `App::setDebugSink()`, wired to `App::Comms` on the robot and to the sim
  harness's comms.
- Host: `serial_conn.py` routes DBG to its own `on_debug` callback, never
  raising; `SimLoop.drain_debug_lines()` on the Sim side.

`DBG_MILLI` exists because newlib-nano has no printf float support — `%f` emits
nothing on ARM. Any debug formatting must go through integer milli-units.

## Why it earned its keep immediately

The first thing it printed was the bench completing a Move with `anch=687`,
which is what exposed the stalled-Move-reports-success defect below. That
defect had been invisible from the host for the whole session.

## Known defect it surfaced, still open

`bool done = timedOut || stalled;` — a Move terminated by the **stall detector**
completes reporting `err = 0`, indistinguishable from success. The managed +500
button therefore "succeeds" while the robot has stopped short. This needs its
own fault bit, the same treatment as
[[wheel-frozen-fault-flag-in-telemetry]].

## Care needed on re-land

A `_log` NameError inside the host DBG handler killed the reader thread during
this session — the debug channel must never be able to take down telemetry. Keep
the handler exception-proof and add a test that a malformed DBG line does not
disturb frame delivery.

## Acceptance

- ARM release build: `DBG` compiles out entirely, no flash cost, no wire traffic.
- Sim and bench builds: `debugf()` lines arrive host-side and appear in the GUI
  console.
- A malformed or oversized DBG line does not interrupt telemetry.
