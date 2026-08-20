---
id: '011'
title: "Golden-vector fixture rewrite (ASCII) + REPL fourth vector set — primary\
  \ conformance gate"
status: open
use-cases: [SUC-006]
depends-on: ['010']
github-issue: ''
issue:
- protocol-v6-spec.md
- protocol-and-implementation-simplification-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Golden-vector fixture rewrite (ASCII) + REPL fourth vector set — primary conformance gate

## Description

This is the primary conformance gate per `sprint.md`'s Success Criteria and
Test Strategy. Rewrite `src/tests/fixtures/wire_golden_vectors.txt` as ASCII
line vectors (command-in/expected-line-out pairs and the reverse, spec
§11.3), replacing the COBS+CRC+protobuf vector format. Update the C++ and
Python conformance harnesses to assert against the rewritten fixture
byte-for-byte. Add the fourth vector set: for each vector, assert
`p(line)`'s printed output is byte-identical to the corresponding wire-form
vector (spec §11.3's "keeps the REPL from quietly becoming a second
protocol" check) — **this must be a real, executable assertion that fails
if the `p()` form and the wire form diverge**, not a doc note.

## Acceptance Criteria

- [ ] `wire_golden_vectors.txt` is rewritten as ASCII line vectors covering
      every verb in spec §3 (30 verbs) with both directions represented.
- [ ] The C++ harness asserts its encoder/decoder against every vector,
      100% pass.
- [ ] The Python harness asserts its encoder/decoder against every vector,
      100% pass.
- [ ] A fourth harness asserts `p(line)`'s printed stdout is byte-identical
      to the corresponding wire-form vector, for every vector in the set —
      implemented as a real test that FAILS if `p()`'s output and the wire
      form diverge by even one byte (spec §11.3/§10.2) — not a
      documentation note or manual check.
- [ ] The fixture lives in one file read by all three harnesses — no
      hand-copied per-language vector list (the same anti-drift principle
      as Design Rationale Decision 2, applied to conformance vectors per
      SUC-006's own acceptance criteria).
- [ ] This ticket runs only after tickets 008/009/010 have landed (all
      three implementations exist to test against) — sequencing confirmed
      by `depends-on`.

## Testing

- **Existing tests to run**: prior (pre-rewrite) golden-vector harness
  runs, to confirm the rewrite is a genuine replacement, not an addition
  alongside a stale binary fixture.
- **New tests to write**: the rewritten fixture itself and its
  three-plus-one harnesses (C++, Python, REPL byte-identical check) are the
  new tests.
- **Verification command**: the C++ conformance test binary, `uv run pytest
  src/tests -k golden_vector`, and the REPL vector-assertion test, all
  green.
