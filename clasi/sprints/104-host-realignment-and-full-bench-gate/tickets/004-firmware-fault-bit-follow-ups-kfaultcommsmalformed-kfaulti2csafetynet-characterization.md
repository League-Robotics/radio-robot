---
id: '004'
title: "Firmware fault-bit follow-ups \u2014 kFaultCommsMalformed + kFaultI2CSafetyNet\
  \ characterization"
status: done
use-cases:
- SUC-014
depends-on: []
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware fault-bit follow-ups — kFaultCommsMalformed + kFaultI2CSafetyNet characterization

## Description

Two small, already-flagged firmware gaps from sprint 103:

1. `source/app/comms.cpp`'s `malformedCount_` has existed since ticket
   103-004 with an explicit forward-reference comment: "surfaced later as
   a Telemetry fault bit (ticket 005)". Ticket 103-005 did NOT implement
   this — confirmed by reading `telemetry.h`'s actual `fault_bits` layout
   (only bits 0-2 claimed: `kFaultI2CSafetyNet`, `kFaultWedgeLatch`,
   `kFaultI2CNak`). A malformed/undecodable inbound frame is currently
   invisible on the wire — only visible via an internal counter no host
   tool reads.
2. Ticket 103-010's bench session found `kFaultI2CSafetyNet` is a
   boot-time one-shot latch (fires once during `Preamble`, coincident with
   the `Preamble::done()` transition, then never re-fires — confirmed
   across every re-run capture in that session's "power ON" pass). The
   bit's own doc comment does not yet say this, risking a future bench
   reader chasing a healthy `fault=1` as a live problem — 103-010's own
   Results explicitly recommended documenting it.

This ticket has no dependency on tickets 001-003 (host-only) and can
proceed independently/in parallel per architecture-update.md's Migration
Concerns.

## Acceptance Criteria

- [x] `kFaultCommsMalformed` bit defined in `telemetry.h` at the next free
      bit position (bit 3, following the existing 0/1/2 pattern) — confirm
      first whether `fault_bits`'s actual generated/wire field type has
      room for it without a `.proto` change (architecture-update.md Step 7
      Open Question 2); if it does NOT, stop and re-scope this ticket as a
      real schema change with its own budget analysis rather than silently
      forcing the bit in.
- [x] `main.cpp` gains one `setFault(kFaultCommsMalformed, ...)` call site
      reading `Comms::malformedCount()`, following the exact pattern
      already used for `I2CBus::clearanceSafetyNetCount()` (same file,
      same idiom).
- [x] A `HOST_BUILD` firmware unit test sends a malformed/undecodable
      frame through `Comms` and confirms `kFaultCommsMalformed` sets in
      the resulting telemetry frame.
- [x] `telemetry.h`'s `fault_bits` doc comment block is updated to include
      bit 3 in the same format as bits 0-2.
- [x] `kFaultI2CSafetyNet`'s doc comment is updated to state the observed
      boot-time-one-shot characterization from 103-010 (fires once during
      `Preamble`/boot-ready transition, never during driving; a healthy
      robot can show `fault_bits` bit 0 set permanently after boot with no
      ongoing problem).
- [x] `just build-clean` (or equivalent HOST_BUILD) passes with the new
      bit; no RAM/flash regression beyond what one new constant + one call
      site implies (sanity check only, not a hard budget gate — this is
      not a wire-format change).

## Completion Notes

**Open Question 2, resolved**: `fault_bits` is already a plain `uint32
fault_bits = 21` proto field (not an enum or fixed-width sub-type) — bit 3
fits inside the existing field with **zero** `.proto` schema/wire-shape
change. Ran `python3 scripts/gen_messages.py` after editing
`protos/telemetry.proto`'s doc comment (added bit 3, corrected bit 0's
characterization) and confirmed byte-for-byte: `git status --porcelain
source/messages/ host/robot_radio/robot/pb2/` reported nothing changed —
`kCommandEnvelopeMaxEncodedSize=115` / `kReplyEnvelopeMaxEncodedSize=179`
unchanged (the generator already sizes the varint worst-case off
`0xFFFFFFFF`, so a new bit inside the same field never moves the number).
No re-scope needed; only the proto doc comment changed, not the schema.

**What changed**:
- `protos/telemetry.proto` — `fault_bits` doc comment: added bit 3
  (`kFaultCommsMalformed`) in the same format as bits 0-2; corrected bit
  0's comment with 103-010's boot-one-shot-latch finding.
- `source/app/telemetry.h` — `constexpr uint32_t kFaultCommsMalformed = 1u
  << 3;` added after `kFaultI2CNak`; the bit-layout doc comment block
  updated to match (bit 0 correction verbatim-sourced from 103-010's own
  ticket file §"`fault_bits` bit 0 (`kFaultI2CSafetyNet`) — boot-time
  latch, not continuous"; bit 3 added).
- `source/main.cpp` — one new line next to the existing two `setFault()`
  calls: `tlm.setFault(App::kFaultCommsMalformed, comms.malformedCount() >
  0);` (same idiom as `kFaultI2CSafetyNet`/`kFaultWedgeLatch`).
- `source/app/comms.cpp` / `source/app/comms.h` — corrected two stale
  comments that said "surfaced later as a Telemetry fault bit (ticket
  005)" / "Ticket 005 reads it" — 103-005 declared the bit but never wired
  this call site (this ticket's whole premise); now cites 104-004 and the
  real call site.
- `tests/sim/unit/app_telemetry_harness.cpp` — new scenario 8
  (`scenarioMalformedFrameSetsCommsMalformedFaultBit`): pumps a malformed
  armored line (`"*Xsomeunrecognizedarmor"`, same input
  `app_comms_harness.cpp`'s own malformed-prefix scenario uses) through a
  real `App::Comms` instance via `pump()`, confirms
  `malformedCount()==1`, mirrors main.cpp's exact `setFault()` call, and
  confirms the NEXT `emit()`'d frame's independently re-encoded bytes
  carry `fault_bits == kFaultCommsMalformed`; a second half confirms the
  bit clears on a later frame once the caller stops reporting it (matches
  scenario 4's existing level-set discipline). Added a
  `QueueableFakeTransport` (mirrors `app_comms_harness.cpp`'s own
  `FakeTransport`) since the harness's existing `FakeTransport::readLine()`
  always returns `false` ("Telemetry never reads").

**Host**: searched `host/` for any place that names individual `fault_bits`
bits (not just carries the raw mask) — found none.
`host/robot_radio/robot/protocol.py`'s `TLMFrame.fault_bits`/`from_pb2()`
already pass the whole `int(telemetry.fault_bits)` through unconditionally
(103-009) with no per-bit constants or decoding anywhere in `host/` or
`tests/`; the docstring already just says "see telemetry.proto's own
comment for the bit numbering." No host code change was needed or made —
this ticket's own acceptance criteria are firmware+test only, and nothing
in `host/` requires updating for a bit that already rides transparently.

**Verification**:
- `python3 scripts/gen_messages.py` — regenerated with zero diff (see
  above).
- Compiled+ran `app_telemetry_harness` and `app_comms_harness` directly
  (`c++ -std=c++20 -Wall -Wextra -DHOST_BUILD`) — all 8 telemetry
  scenarios and all 8 comms scenarios pass.
- `just build-clean` — firmware build succeeds, produces
  `build/MICROBIT.hex`. FLASH 131848 B / 364 KB (35.37%), RAM 120768 B /
  122816 B (98.33%, expected — CODAL RAM is always near-full by design,
  per project knowledge; only flash overflow is a real regression signal
  here, and there is none).
- `uv run python -m pytest -q` — 568 passed (same count as session start;
  the new scenario lives inside the existing
  `test_app_telemetry_harness_compiles_and_passes` HOST_BUILD test, not a
  new pytest file).
- Hardware smoke flash: NOT performed — this ticket's acceptance criteria
  do not require it (bench soak/flash verification is ticket 007's scope
  per the dispatch brief), and nothing in this diff changes wire framing
  or motor/sensor behavior that would need re-proving on the stand ahead
  of that ticket.

## Testing

- **Existing tests to run**: `source/tests/` (or wherever `HOST_BUILD`
  firmware unit tests live — confirm exact path against 103's own test
  tree) for `comms`/`telemetry` — must stay green.
- **New tests to write**: malformed-frame-sets-fault-bit test (HOST_BUILD).
- **Verification command**: whatever `just`/`ctest` invocation 103's own
  `HOST_BUILD` tests use (mirror 103-002's/103-005's own testing plan
  format).

## Implementation Plan

**Approach**: Read `telemetry.h`'s existing `kFaultI2CSafetyNet`/
`kFaultWedgeLatch`/`kFaultI2CNak` wiring in `main.cpp` first to copy the
exact idiom for the new bit — this is a one-line addition following an
established pattern, not new design. Confirm `fault_bits`' generated wire
type (Open Question 2) before touching anything.

**Files to create/modify**:
- `source/app/telemetry.h` — `kFaultCommsMalformed` constant, doc comment
  for bit 3, `kFaultI2CSafetyNet` doc comment correction.
- `source/main.cpp` — one new `setFault()` call site.
- A `HOST_BUILD` test file exercising `Comms` with malformed input
  (location TBD by ticket-time investigation of 103's own test layout for
  `Comms`).

**Testing plan**: covered above.

**Documentation updates**: the `telemetry.h` doc-comment edits ARE the
documentation update this ticket delivers; also record in this ticket's
completion notes the confirmed answer to Open Question 2 (wire-width),
resolving it for future readers.

## SUC-014: Firmware fault-bit follow-ups from 103 ticket 010

Parent: `single-loop-firmware-p3-p7-continuation.md` (P6).

- **Actor**: Firmware engineer; bench operator reading `fault_bits`.
- **Preconditions**: `malformedCount_` exists, unwired; safety-net bit
  behavior observed but undocumented.
- **Main Flow**: Wire the malformed-frame bit; document the safety-net
  bit's true behavior.
- **Postconditions**: malformed frames are wire-visible; the safety-net
  bit reads unambiguously.
- **Acceptance Criteria**: see above; corroborated under sustained load by
  ticket 007's soak run (SUC-017).
