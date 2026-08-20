---
id: '014'
title: Bench script + sim/test-harness migration to v6
status: open
use-cases: [SUC-003, SUC-007, SUC-008]
depends-on: ['013']
github-issue: ''
issue:
- protocol-v6-spec.md
- protocol-and-implementation-simplification-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench script + sim/test-harness migration to v6

## Description

Migrate bench tooling (`src/tests/bench/`: `tlm_log.py`, `twist_drive.py`,
`move_protocol_bench.py`, `wire_truth.py` — `radio_bench_gate.py` gets its
own ticket, 015 — and others found by grep) and `src/tests/sim/**`
harnesses off any remaining binary-plane assumption now that ticket 013 has
deleted it. Grep the whole bench/sim tree for wire-framing, ack-observation,
or telemetry-field-shape assumptions rather than relying on the list above
being exhaustive.

## Acceptance Criteria

- [ ] Every bench script under `src/tests/bench/` that references wire
      framing, ack observation, or telemetry field shape is updated to the
      v6 ASCII forms (ticket 005's `TLMFrame` parse path, ticket 007's ack
      line-reading).
- [ ] `src/tests/sim/**` harnesses (including any that still reference
      deleted binary-plane symbols after ticket 013) are updated and green.
- [ ] A repo-wide grep for the deleted binary-plane symbol names
      (COBS/CRC/protobuf message type names/ack-ring struct names) across
      `src/tests/bench/` and `src/tests/sim/` returns zero hits.
- [ ] `radio_bench_gate.py` is explicitly excluded from this ticket's
      scope — it has its own ticket (015) given its size/importance as the
      sprint's closing gate.

## Testing

- **Existing tests to run**: the full `src/tests/sim/**` suite and every
  bench script's own smoke path.
- **New tests to write**: none beyond the migration itself — this ticket is
  a mechanical port, verified by the existing suites passing against v6.
- **Verification command**: `uv run python -m pytest src/tests/sim` plus a
  manual/sim smoke run of each migrated bench script.
