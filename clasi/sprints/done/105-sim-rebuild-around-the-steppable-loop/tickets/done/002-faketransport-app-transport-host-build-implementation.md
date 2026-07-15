---
id: '002'
title: 'FakeTransport: App::Transport HOST_BUILD implementation'
status: done
use-cases:
- SUC-019
depends-on: []
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# FakeTransport: App::Transport HOST_BUILD implementation

## Description

`App::Transport` (`source/app/comms.h`) is already an abstract,
`HOST_BUILD`-safe interface (`readLine()`/`send()`/`sendReliable()`) with
two ARM-only concrete adapters (`SerialTransport`/`RadioTransport`); no
`HOST_BUILD` implementation exists yet. This ticket builds one: an
in-memory, FIFO-based fake a test can push armored `"*B..."` command lines
into (so `Comms::pump()` reads them exactly as if from a real serial/radio
line) and read captured outbound armored lines back out of (every
`Comms::sendReply()`/`Telemetry` emit call). This is a small, standalone,
reusable primitive — no dependency on ticket 001 or 003 — that later
tickets (004's `sim_api`, 006's pytest scenarios) build on to drive
commands into and read telemetry out of the sim loop.

## Acceptance Criteria

- [x] A `HOST_BUILD`-only `FakeTransport` class implementing `App::Transport`
      exists (e.g. `tests/sim/support/fake_transport.h`), with:
      - an inbound FIFO a test populates with complete armored lines
        (`enqueueInbound(const char* line)`), consumed one line per
        `readLine()` call per `Comms::pump()`'s own "at most one line per
        call" contract;
      - two outbound captures (matching `Transport::send()` vs.
        `sendReliable()`'s distinct drop-on-full vs. must-not-drop
        semantics) a test can drain/inspect after stepping the loop.
- [x] `readLine()` returns `false` (no line ready) when the inbound FIFO is
      empty — never blocks, matches `Transport::readLine()`'s documented
      non-blocking contract.
- [x] A unit test constructs an `App::Comms` over two `FakeTransport`
      instances (serial + radio stand-ins), enqueues a real armored
      `twist` `CommandEnvelope` line (built via the same `msg::wire::
      encode()`/armor helpers `Comms`/existing wire tests already use),
      calls `pump()`, and confirms the decoded `Cmd` matches.
- [x] The same test (or a paired one) constructs an `App::Telemetry` over
      the fake transports, calls `emit()`, captures the outbound line from
      `FakeTransport`, dearmors + decodes it, and confirms it round-trips
      to a real `msg::ReplyEnvelope`/`Telemetry` frame.
- [x] `FakeTransport` never allocates from the heap on the hot path
      (matches the project's no-heap-in-hot-path convention) — a bounded,
      fixed-capacity FIFO (ring buffer or `std::deque`, since this is
      test-only `HOST_BUILD` code where `std::deque` is already used
      elsewhere, e.g. `i2c_bus_host.cpp`'s scripted queues) is acceptable.

## Completion Notes

- `tests/sim/support/fake_transport.h` (new): `TestSupport::FakeTransport`,
  the ONE canonical `App::Transport` double. `enqueueInbound(const char*)`
  pushes onto a `std::deque<std::string>`; `readLine()` pops the oldest
  entry, NUL-terminates into the caller's buffer, and returns `false`
  immediately (buffer untouched) when the queue is empty. `send()`/
  `sendReliable()` each append to their own `std::deque<std::string>`
  (`sent()`/`sentReliable()`), matching the two transports' distinct
  drop-on-full vs. must-not-drop call sites.
- **Dedup**: `tests/sim/unit/app_comms_harness.cpp` and
  `tests/sim/unit/app_telemetry_harness.cpp` each carried their own ad hoc
  `FakeTransport` (the latter also had a second `QueueableFakeTransport`
  variant purely to add a queue its base fake lacked). Both files were
  migrated onto the shared header (`#include "support/fake_transport.h"`,
  `using TestSupport::FakeTransport;`); `QueueableFakeTransport` is gone
  entirely — the shared class's `readLine()` already returns `false` when
  nothing was ever enqueued, which is exactly what the old no-queue variant
  needed. Call sites renamed: `queueLine()` → `enqueueInbound()`,
  `queueSize()` → `inboundSize()`, `sendLog()` → `sent()`,
  `sendReliableLog()` → `sentReliable()`. No scenario assertions changed —
  same coverage, one primitive. `test_app_comms.py`/`test_app_telemetry.py`
  each gained a second `-I tests/sim` compiler flag so `"support/
  fake_transport.h"` resolves.
- **Ticket-time call on the "New tests to write" file-placement question**:
  folded the Comms/Telemetry *integration* round-trip proofs (SUC-019's own
  AC bullets 3/4 — armored twist decoded via `pump()`, a `Telemetry::emit()`
  frame round-tripped) into the existing, now-migrated
  `app_comms_harness.cpp`/`app_telemetry_harness.cpp` scenarios rather than
  re-proving them a third time in a new harness — the ticket text itself
  sanctions this ("fold the scenarios into a small addition to the existing
  ... harnesses if that reads more naturally"), and duplicating an
  already-covered round trip would cut against this ticket's own dedup
  mandate. `tests/sim/unit/fake_transport_harness.cpp` +
  `tests/sim/unit/test_fake_transport.py` were still written, but scoped to
  what nothing else covers: `FakeTransport` proven in ISOLATION (empty-queue
  `readLine()` returns `false` immediately; `enqueueInbound()` drains
  strict FIFO order one line per call; an armored `"*B..."` line survives
  the round trip byte-for-byte; `send()`/`sendReliable()` are genuinely
  separate captures) — this is the smallest possible compile unit (no
  `comms.cpp`/`wire.cpp`/`wire_runtime.cpp` linked, just the header + the
  abstract `App::Transport` base).
- Verification: `uv run python -m pytest tests/sim/unit/ -k "transport or
  comms or telemetry" -v` → 29 passed. Full suite: `uv run python -m
  pytest` → 563 passed. Manual `-Wall -Wextra -std=c++20` compiles of all
  three harnesses produced zero warnings. No production code touched
  (`Comms`/`Telemetry` untouched, per the ticket's own "Files to modify:
  none").

## Testing

- **Existing tests to run**: `tests/sim/unit/test_app_comms.py`,
  `tests/sim/unit/test_app_telemetry.py`, `tests/sim/unit/test_wire_codec.py`
  (confirm no regression to the wire/armor path this reuses).
- **New tests to write**: `tests/sim/unit/fake_transport_harness.cpp` +
  `tests/sim/unit/test_fake_transport.py` (or fold the scenarios into a
  small addition to the existing `app_comms_harness.cpp`/
  `app_telemetry_harness.cpp` if that reads more naturally — a ticket-time
  call).
- **Verification command**: `uv run python -m pytest tests/sim/unit/ -k "transport or comms or telemetry" -v`.

## Implementation Plan

**Approach**: `FakeTransport` holds two queues — one inbound (lines a test
pushes, `readLine()` pops), and captures for `send()`/`sendReliable()`
(append-only vectors of `std::string`/fixed buffers a test drains). No
production code changes: `Comms`/`Telemetry` already depend only on the
abstract `Transport&` interface, so `FakeTransport` slots in wherever
`SerialTransport`/`RadioTransport` do today, with no change to either
class. Reuse the SAME armor/dearmor and `msg::wire::encode()`/`decode()`
helpers the existing wire tests (`test_wire_codec.py`,
`wire_codec_harness.cpp`) already exercise, rather than hand-rolling a
parallel encode path in the test — this keeps the fake transport testing
real wire round-trips, not a shortcut.

**Files to create**:
- `tests/sim/support/fake_transport.h` (new shared support directory —
  see architecture-update.md Step 7 Open Question 1; this ticket may
  establish `tests/sim/support/` if ticket 001 hasn't already, or ticket
  004 may create it — coordinate at execution time, first ticket to need
  it creates the directory).
- `tests/sim/unit/fake_transport_harness.cpp` + matching pytest wrapper.

**Files to modify**: none (pure addition; `Comms`/`Telemetry` are
untouched, per architecture-update.md's own "Unchanged" section).

**Testing plan**: unit-level round-trip tests as described in Acceptance
Criteria — this ticket's own correctness is fully provable off-hardware,
no bench gate needed (it touches no production/ARM code).

**Documentation updates**: a short file-header comment on
`fake_transport.h` explaining its role (mirrors `comms.h`'s own
documentation density/style) — no external doc changes needed.
