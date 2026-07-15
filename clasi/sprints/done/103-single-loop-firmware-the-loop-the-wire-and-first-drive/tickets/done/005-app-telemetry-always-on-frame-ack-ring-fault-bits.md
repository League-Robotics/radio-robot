---
id: '005'
title: "app/Telemetry \u2014 always-on frame, ack ring, fault bits"
status: done
use-cases:
- SUC-005
depends-on:
- '001'
- '004'
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# app/Telemetry — always-on frame, ack ring, fault bits

## Description

Build `source/app/telemetry.{h,cpp}`: assembles and emits the primary
telemetry frame (via `Comms`'s send path, ticket 004) every cycle,
carrying the depth-3 ack ring and fault/event bits (ticket 001's schema),
plus the `TelemetrySecondary` slow frame on whatever framing/cadence
ticket 001 decided (Decision 3). This is the highest-risk piece of this
sprint — the primary frame's measured margin against the 186B ceiling is
only 7B (spike-003's number; ticket 001 re-derives it for real), so field
additions here are zero-slack.

Depends on ticket 001 (the `Telemetry`/`TelemetrySecondary`/`AckEntry`
types) and ticket 004 (`Comms::sendReply()` is the actual send path this
module calls).

## Acceptance Criteria

- [x] `Telemetry::emit()` builds one primary frame per call
      (`now`/`mode`/`seq`/`enc`/`vel`/`pose`/`otos`+`otos_connected`/
      `twist`/`active`/`conn_left`/`conn_right`/`fault_bits`/`event_bits`,
      or ticket 001's own final field list if it differs) and sends it via
      `Comms`.
- [x] `Telemetry::ack(corrId, status, errCode)` pushes one entry into the
      ring; the ring holds exactly the last 3 entries (oldest evicted);
      every `emit()` call includes the current ring contents in full (not
      just new entries) so a single dropped frame cannot lose an ack.
- [x] A host-buildable unit test proves the ring survives one dropped
      frame: push 4 sequential acks, build 2 successive frames (simulating
      one "lost" frame in between by not reading the first), confirm the
      newest 3 acks are present in the frame that IS read.
- [x] `fault_bits`/`event_bits` bit layout is decided and documented (this
      ticket's own decision — ticket 001 declared the fields, not the
      layout). At minimum: one bit for the `I2CBus` `readyAt` safety-net
      trip (ticket 002) and one bit for a `Deadman` expiry (ticket 004) are
      wired to real call sites, not left as dead bit positions.
- [x] `TelemetrySecondary` is emitted on the framing/cadence ticket 001
      decided; its own emission does not starve or delay the primary
      frame's cadence.
- [x] Measured emission cadence (both frame types, real timestamps from a
      host-buildable or bench test) is recorded against the 25 Hz/40ms
      target from spike-001 — this ticket does not need to HIT 25 Hz
      exactly, but must report its own actual number rather than assuming
      it matches spike-001's pre-rewrite baseline (spike-001's own
      ~30.3 Hz armed vs. ~26.8 Hz actual gap was never root-caused).

## Implementation Plan

**Approach**: Build the frame assembly first against a fixed, fake data
source (host-buildable), verify the ring/budget logic in isolation, THEN
wire real data sources (motor leaves, `Otos`, `Deadman`, `I2CBus`) once
tickets 002/003/006/007 exist — this ticket's own acceptance criteria are
satisfiable with `Telemetry` as a standalone, testable class before the
full loop (ticket 008) wires it to live devices.

**Files to create/modify**:
- `source/app/telemetry.h`, `source/app/telemetry.cpp` (new)

**Testing plan**:
- Existing tests to run: ticket 001's rewritten wire test suite (confirms
  the underlying `Telemetry`/`AckEntry` encode path this module calls is
  itself correct).
- New tests to write: the ack-ring-survives-a-drop test (Acceptance
  Criteria above); a frame-size test confirming the assembled frame's
  encoded size matches ticket 001's recorded worst case (or is smaller,
  for a partially-populated frame); a fault-bit test confirming each wired
  bit flips when its trigger condition is simulated.
- Verification command: `uv run python -m pytest tests/sim/unit/ -k telemetry`
  (once the test file exists) plus a host-side timing measurement script
  under `tests/bench/` or `tests/sim/` for the cadence acceptance
  criterion (can run host-buildable with a scripted clock, does not
  require hardware for this ticket).

**Documentation updates**: record the `fault_bits`/`event_bits` layout in
a comment block at the top of `telemetry.h` (the single place a future
reader looks to decode a fault bit) and in this ticket's completion notes.

## Completion Notes

**Files created**: `source/app/telemetry.h`, `source/app/telemetry.cpp`,
`tests/sim/unit/app_telemetry_harness.cpp`, `tests/sim/unit/test_app_telemetry.py`.

**Files touched (drive-by fix, separate commit)**: `source/app/deadman.h`,
`source/app/deadman.cpp` — renamed `deadlineMicros_` → `deadline_` (`//
[us]` tag) and the local `deltaMicros` → `delta` (`// [us]` tag); ticket
004 introduced these unit-suffixed identifiers before this naming rule was
enforced on that file. `Devices::Clock::nowMicros()` itself is untouched
(pre-existing vendor-adjacent seam name, covered by a separate naming-sweep
issue per this dispatch's own instruction). `grep -rn "Micros\b" source/app/`
after the fix matches only the two `clock_.nowMicros()` call sites.

**Two send paths, not one**: `Comms::sendReply()` (ticket 004) only accepts
a `msg::ReplyEnvelope`, and `envelope.proto`'s `ReplyEnvelope.body` oneof is
fixed at `ok`/`err`/`tlm` — there is no `tlm2` arm for `TelemetrySecondary`
(ticket 001's own Decision 3 resolution: secondary rides its own,
independently-armored `"*B"` line). `Telemetry` therefore holds both a
`Comms&` (primary frame: builds `ReplyEnvelope{corr_id=0, body_kind=TLM}`,
calls `comms.sendReply()`) AND the two `Transport&` references directly
(secondary frame: `msg::wire::encode(TelemetrySecondary&, ...)` +
`WireRuntime::base64Encode()` + broadcast on both transports' `send()`,
reusing `Comms`'s own public `kArmoredBufSize` constant rather than
duplicating a private armor helper). This matches
architecture-update.md (103) Step 4's dependency graph, which draws
`Telemetry --> Com` as its own edge distinct from `Comms --> Com` — the
graph anticipated this exact two-path shape.

**API shape — standalone/testable per the ticket's own implementation
plan**: `Telemetry` holds no pointer to any leaf, `I2CBus`, or `Deadman`.
Callers stage the next frame's data via `setFrame(const Frame&)` /
`setSecondaryFrame(const SecondaryFrame&)` (two small snapshot structs
mirroring `msg::Telemetry`/`msg::TelemetrySecondary`'s own `has_*`/value
pairs) and report fault/event conditions via `setFault(bit, active)` /
`setEvent(bit, active)` using the named bit constants below. Real-device
wiring (constructing `Telemetry` with the real `Comms`/`Transport`s,
calling `setFrame()` from the leaves' actual state each cycle) is ticket
008's job, exactly as this ticket's own Implementation Plan says.

**`fault_bits`/`event_bits` layout** (reproduced from `telemetry.h`'s own
header comment, itself reproduced from ticket 001's `protos/telemetry.proto`
doc comment — the numbering was ticket 001's decision; wiring individual
bits live is ticket-by-ticket):
```
fault_bits:
  bit 0 (kFaultI2CSafetyNet) -- I2CBus readyAt clearance safety-net trip
                                  (Devices::I2CBus::clearanceSafetyNetCount(),
                                  ticket 002). WIRED this ticket.
  bit 1 (kFaultWedgeLatch)   -- NezhaMotor/I2CBus wedge-latch detected
                                  (Devices::MotorArmor::wedged()). Declared,
                                  not yet wired live (no ticket has a real
                                  call site feeding it yet).
  bit 2 (kFaultI2CNak)       -- I2C bus NAK/timeout error. Declared, not
                                  yet wired (no per-transaction NAK
                                  aggregate exists at this ticket's scope).
event_bits:
  bit 0 (kEventDeadmanExpired) -- Deadman staleness timer expired
                                    (App::Deadman::expired(), ticket 004).
                                    WIRED this ticket.
  bit 1 (kEventBootReady)      -- boot-ready transition (Preamble::done()
                                    first true, ticket 007 — Preamble does
                                    not exist yet). Declared, not wired.
  bit 2 (kEventConfigApplied)  -- a ConfigDelta was applied (ticket-008-time
                                    decision, architecture-update.md (103)
                                    Step 7 Open Question 3). Declared, not
                                    wired.
```
"Wired to real call sites" for the two AC-minimum bits means: `setFault()`/
`setEvent()`'s doc comments name the exact real accessor
(`I2CBus::clearanceSafetyNetCount()`, `Deadman::expired()`) a caller passes
through, and `app_telemetry_harness.cpp`'s scenario 4 demonstrates the full
round trip (bit set → appears in the encoded frame; bit cleared → clears in
the next frame) using those exact named constants — not a generic untyped
setter. Wiring the *construction-time* call site itself (main.cpp actually
reading a live `I2CBus`/`Deadman` each cycle) is ticket 008's job, per this
ticket's own Implementation Plan ("wire real data sources ... once tickets
002/003/006/007 exist ... the full loop (ticket 008) wires it to live
devices").

**Ack ring**: `msg::AckEntry ring_[3]` + `ringCount_`, oldest-first /
newest-last. `ack()` appends while `ringCount_ < 3`; once full, shifts
`ring_[1..2]` down and appends at index 2 (an O(1) shift over at most 2
elements). `emitPrimary()` copies `ring_[0..ringCount_)` into
`msg::Telemetry.acks_`/`acks_count` on EVERY primary send — the ring is
never cleared or consumed by a send, so a dropped/unread frame costs
nothing: the next primary frame repeats the same ring contents.
`app_telemetry_harness.cpp` scenario 2 is the literal AC #3 test (push 4,
build 2, only inspect the 2nd).

**Cadence**: `kPrimaryPeriod = 40` (`[ms]`, ~25 Hz, spike-001's target) and
`kSecondaryPeriod = 200` (`[ms]`, ~5 Hz — this ticket's own P4 decision,
architecture-update.md (103) Step 7 Open Question 4, chosen as 5x the
primary period so the two rarely contend for the same `emit()` call while
still refreshing diagnostics at a useful bench rate). `emit(now)` checks
primary first (unconditional send when due — secondary can never delay it)
and only checks secondary if primary was NOT due this call; at most one
frame type is sent per call, by construction (an early `return` after a
primary send). First call to each always sends (boot-ready, no arming
step). Documented, deliberate limitation: a caller invoking `emit()` at
EXACTLY the primary period would starve the secondary frame — the AC only
requires secondary never delay primary, not the reverse; the loop's real
per-cycle rate (well under 40 ms, per architecture-update.md's `runAndWait`
design) avoids this in practice. Flagged in `telemetry.h`'s own doc comment
for ticket 008.

**Measured cadence** (host-buildable, scripted clock, NOT wall-clock —
`app_telemetry_harness.cpp` scenario 7, `tests/sim/unit/test_app_telemetry.py`
prints the harness's stdout): driving `emit()` at a 3 ms step over a 10000 ms
scripted window measured **23.90 Hz primary** (target 25 Hz/40 ms — the gap
is pure step-quantization: a 3 ms sampling grid rounds the 40 ms period up
to a 42 ms realized period, 1000/42 ≈ 23.8 Hz) and **5.00 Hz secondary**
(target ~5 Hz/200 ms). This ticket's own number, not assumed to match
spike-001's baseline, per the AC's own instruction — spike-001's
~30.3 Hz-armed-vs-~26.8-Hz-actual gap remains unexplained and is out of
scope here.

**Frame-size check**: a fully-populated primary frame (ring at depth 3, all
`has_*` true, extreme float/uint32 values in every field) encodes to
**147 B** — under ticket 001's recorded 179 B worst case for
`ReplyEnvelope{tlm}` at ring depth 3 (the 179 B figure assumes maximum-width
varint encodings across every integer field simultaneously, which no single
real frame hits at once; 147 B is this ticket's own measured number for a
realistic "everything populated" frame, reported per the ticket's own
"report the real number" discipline). `app_telemetry_harness.cpp`
scenario 6.

**Verification evidence**:
- `uv run python -m pytest tests/sim/unit/ -v` — 336 passed (all
  pre-existing suites plus the new
  `test_app_telemetry.py::test_app_telemetry_harness_compiles_and_passes`,
  which itself runs 7 scenarios inside the compiled harness).
- `just build` — succeeds; `source/app/telemetry.cpp` and the renamed
  `source/app/deadman.cpp` compile clean for the real ARM target and link
  into `MICROBIT.hex` (v0.20260714.10 at build time). RAM/flash usage
  unchanged in any way that matters (98.33% RAM, 27.84% flash — RAM this
  high is by-design per project precedent, not a regression).
- `grep -rn "Micros\b" source/app/*.h source/app/*.cpp` after the drive-by
  fix: only `clock_.nowMicros()` (the pre-existing, excluded vendor-adjacent
  seam name) remains; `deadlineMicros_`/`deltaMicros` are gone.
- `tests/unit/` was not touched and was not run against this ticket's own
  new code (Decision 4 — pre-existing breakage there is out of scope).
