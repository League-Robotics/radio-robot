---
id: '004'
title: 'Cross-language golden vector suite (REQUIRED): shared byte-string fixtures
  for C++ and pytest'
status: done
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

- [x] The fixture lives in exactly one place (e.g. a JSON/binary fixture
      file under a shared test-data directory), consumed by both a C++
      test and a pytest — not duplicated by hand into each language's
      own test file. `src/tests/fixtures/wire_golden_vectors.txt` (plain
      pipe-delimited hex lines) is the single source; read directly by
      `src/tests/unit/test_wire_golden_vectors.py` (host) and by
      `src/tests/sim/unit/wire_golden_vector_harness.cpp` (firmware,
      compiled/run via `src/tests/sim/unit/test_wire_golden_vector_harness.py`).
      The two pre-existing hand-hardcoded copies of the issue's §2 table
      (`wire_runtime_harness.cpp`'s `scenarioCobsKeyedOn0x0AAdversarialVectors`,
      `test_host_wire_codec.py`'s `test_cobs_delimiter_0x0a_adversarial_vectors_from_issue`)
      were migrated to read the fixture, not left as a second copy.
- [x] Either suite fails if the firmware and host codecs ever produce
      different bytes for the same input. Verified by deliberate break
      (see below).
- [x] A deliberately-broken one-sided change (make during ticket review:
      change one side's delimiter or CRC-scope handling without the
      other) fails the shared vector test — verify this manually before
      closing the ticket. Done: temporarily changed
      `kCrc16CcittFalsePoly` in `src/firm/messages/wire_runtime.cpp` from
      `0x1021` to `0x8005` (a different, real CRC-16 polynomial) —
      `wire_golden_vector_harness` failed 14 assertions across all 8
      vectors while the untouched Python suite
      (`test_wire_golden_vectors.py`) kept passing 18/18, proving the
      fixture (not either side's own internal self-consistency) is what
      catches cross-language drift. Reverted; `git diff` on
      `wire_runtime.cpp` is empty and both suites pass again.
- [x] Covers all vectors from ticket 003 (COBS adversarial inputs, CRC
      scope) plus the grammar edge cases (data containing `:`, data
      containing `0x00`, a stray trailing `:` on a no-data verb, an
      unknown command, a truncated line). **Partial, by design —
      documented decision, not an oversight:** the fixture covers all
      COBS/CRC-scope vectors (all-0x0A, all-0x00, 0x00..0x0F sweep from
      the issue, plus a 0x00..0xFF sweep, an empty payload, and a
      CRC-scope pair proving two command names differ) and both
      byte-content sub-items (`payload_contains_colon_and_zero` covers
      data containing `:` and `0x00`). The three remaining sub-items
      (stray trailing `:` on a no-data verb, an unknown command, a
      truncated *line*) are properties of the COMMAND:data LINE-GRAMMAR
      parser — ticket 124-005's own scope ("framing grammar cutover"),
      not yet implemented (`comms.cpp`'s own comments say so explicitly:
      "124-005's grammar cutover owns that parse"). SUC-002 (sprint.md),
      which team-lead named as this ticket's authoritative spec, does not
      list grammar vectors at all. Recommend 124-005 extend this SAME
      fixture (add rows, no new file) once its line parser exists, rather
      than this ticket inventing tests against not-yet-written code.

## Testing

- **Existing tests to run**: `test_wire_codec.py`, `test_host_wire_codec.py`,
  existing `wire_runtime` C++ unit tests (must all still pass).
- **New tests to write**: the shared-fixture C++ test and pytest
  described above — this ticket's entire deliverable.
- **Verification command**: `uv run pytest` plus the C++ sim-tests build.
