---
id: '006'
title: Firmware ok/err/done reply lines (alongside existing ack ring)
status: open
use-cases: [SUC-001, SUC-003]
depends-on: ['004']
github-issue: ''
issue:
- protocol-v6-spec.md
- protocol-and-implementation-simplification-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware ok/err/done reply lines (alongside existing ack ring)

## Description

Add `ok:<id>`/`err:<id>:<code>`/`done:<id>:<reason>` reply-line emission to
`Core::Comms`, alongside — not replacing — the existing ack ring. This is
spec §13 migration stage 3. Each reply is sent three times on consecutive
cycles (spec §8.1) so an outcome survives the measured ~5% radio loss
without a ring, a depth, an eviction policy, or a scan; the host takes the
first copy and ignores repeats by id (host-side dedup is ticket 007).
`done`'s `reason` field (`stop` or `timeout`) replaces v5's flags-bit
encoding of the same distinction. Wire the new `ERR_DUPLICATE_ID` (code 11,
spec §8.3) for a reused id — refused out loud, unlike v5's silent drop (the
real, recorded footgun spec §8.2 closes).

## Acceptance Criteria

- [ ] `ok:<id>` is emitted on acceptance (enqueue for a motion verb, or
      immediate apply for `SET`), three times on consecutive cycles.
- [ ] `err:<id>:<code>` is emitted on rejection, three times, using the code
      table in spec §8.3 (`ERR_UNKNOWN`=1, `ERR_BADARG`=2, `ERR_RANGE`=3,
      `ERR_FULL`=4, `ERR_UNIMPLEMENTED`=6, `ERR_NOT_CONFIGURED`=8,
      `ERR_BUSY`=10, `ERR_DUPLICATE_ID`=11).
- [ ] `done:<id>:<reason>` is emitted when an enqueued thing finishes, three
      times, `reason` ∈ `stop`/`timeout` — a plain word, not a flags-bit
      lookup.
- [ ] A reused id within the session is rejected `err:<id>:11`
      (`ERR_DUPLICATE_ID`), never silently dropped.
- [ ] Id `0` means "no ack wanted" and is honored on any verb with an
      optional id — no `ok`/`err`/`done` is emitted for it.
- [ ] `ESTOP` never carries an id and is never acked, on any code path (it
      must not queue behind anything, including an ack) — this ticket does
      not implement `ESTOP` itself (that is ticket 008) but must not
      regress the existing v5 `ESTOP` behavior while acks are dual-plane.
- [ ] Existing ack ring is untouched and its tests still pass (additive-only
      ticket).

## Testing

- **Existing tests to run**: `src/tests/sim/unit` Comms ack-ring suites
  (must still pass).
- **New tests to write**: sim/unit tests for `ok`/`err`/`done` 3x-repeat
  emission, duplicate-id rejection, id-0 suppression, `done` reason
  correctness for both stop-condition and timeout completions.
- **Verification command**: `uv run pytest src/tests/sim/unit -k "comms or
  ack or outcome"`.
