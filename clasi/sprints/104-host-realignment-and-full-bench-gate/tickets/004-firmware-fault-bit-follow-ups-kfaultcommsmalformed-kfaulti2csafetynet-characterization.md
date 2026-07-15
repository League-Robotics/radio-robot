---
id: '004'
title: "Firmware fault-bit follow-ups — kFaultCommsMalformed + kFaultI2CSafetyNet\
  \ characterization"
status: open
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

- [ ] `kFaultCommsMalformed` bit defined in `telemetry.h` at the next free
      bit position (bit 3, following the existing 0/1/2 pattern) — confirm
      first whether `fault_bits`'s actual generated/wire field type has
      room for it without a `.proto` change (architecture-update.md Step 7
      Open Question 2); if it does NOT, stop and re-scope this ticket as a
      real schema change with its own budget analysis rather than silently
      forcing the bit in.
- [ ] `main.cpp` gains one `setFault(kFaultCommsMalformed, ...)` call site
      reading `Comms::malformedCount()`, following the exact pattern
      already used for `I2CBus::clearanceSafetyNetCount()` (same file,
      same idiom).
- [ ] A `HOST_BUILD` firmware unit test sends a malformed/undecodable
      frame through `Comms` and confirms `kFaultCommsMalformed` sets in
      the resulting telemetry frame.
- [ ] `telemetry.h`'s `fault_bits` doc comment block is updated to include
      bit 3 in the same format as bits 0-2.
- [ ] `kFaultI2CSafetyNet`'s doc comment is updated to state the observed
      boot-time-one-shot characterization from 103-010 (fires once during
      `Preamble`/boot-ready transition, never during driving; a healthy
      robot can show `fault_bits` bit 0 set permanently after boot with no
      ongoing problem).
- [ ] `just build-clean` (or equivalent HOST_BUILD) passes with the new
      bit; no RAM/flash regression beyond what one new constant + one call
      site implies (sanity check only, not a hard budget gate — this is
      not a wire-format change).

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
