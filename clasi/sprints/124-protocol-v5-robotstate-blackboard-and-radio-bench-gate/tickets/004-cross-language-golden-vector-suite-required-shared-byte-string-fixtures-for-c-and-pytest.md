---
id: '004'
title: 'Cross-language golden vector suite (REQUIRED): shared byte-string fixtures
  for C++ and pytest'
status: in-progress
use-cases:
- SUC-002
depends-on:
- '003'
github-issue: ''
issue: protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Cross-language golden vector suite (REQUIRED): shared byte-string fixtures for C++ and pytest

## Description

**REQUIRED, non-negotiable** — one of the stakeholder's two structural
fixes for the defect class that shipped in 123. The 0x0A-in-binary-frame
bug shipped because firmware's and host's demuxers were two independent
heuristics with no shared vector forcing them to agree. A hand-maintained
copy of "the same" test data in each language is not sufficient — it is
exactly the kind of duplication that let the two sides drift in the
first place.

Build ONE shared fixture (all-`0x0A` payload, all-`0x00` payload,
`0x00..0xFF` sweep, empty payload, plus the CRC-scope vector from ticket
003) as a single file of framed byte strings. A C++ test and a pytest
both read THIS SAME file and assert byte-for-byte identical results —
two readers of one source, not two hand-maintained copies.

## Acceptance Criteria

- [ ] The fixture lives in exactly one place (e.g. a JSON/binary fixture
      file under a shared test-data directory), consumed by both a C++
      test and a pytest — not duplicated by hand into each language's
      own test file.
- [ ] Either suite fails if the firmware and host codecs ever produce
      different bytes for the same input.
- [ ] A deliberately-broken one-sided change (make during ticket review:
      change one side's delimiter or CRC-scope handling without the
      other) fails the shared vector test — verify this manually before
      closing the ticket.
- [ ] Covers all vectors from ticket 003 (COBS adversarial inputs, CRC
      scope) plus the grammar edge cases (data containing `:`, data
      containing `0x00`, a stray trailing `:` on a no-data verb, an
      unknown command, a truncated line).

## Testing

- **Existing tests to run**: `test_wire_codec.py`, `test_host_wire_codec.py`,
  existing `wire_runtime` C++ unit tests (must all still pass).
- **New tests to write**: the shared-fixture C++ test and pytest
  described above — this ticket's entire deliverable.
- **Verification command**: `uv run pytest` plus the C++ sim-tests build.
