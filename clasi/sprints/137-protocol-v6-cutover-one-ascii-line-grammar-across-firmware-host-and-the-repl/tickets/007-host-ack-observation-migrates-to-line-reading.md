---
id: '007'
title: Host ack observation migrates to line-reading
status: open
use-cases: [SUC-001, SUC-003]
depends-on: ['006']
github-issue: ''
issue:
- protocol-v6-spec.md
- protocol-and-implementation-simplification-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host ack observation migrates to line-reading

## Description

Migrate `NezhaProtocol`'s `wait_for_ack()`-style outcome tracking to read
the ticket 006 `ok`/`err`/`done` lines, per spec §13 stage 3 ("`ok`/`err`/
`done` lines land alongside the ack ring... `wait_for_ack()` switches to
line-reading; the ring goes away" — ring removal is ticket 013). Implement
first-copy-wins de-duplication by id across the 3x repeat.

## Acceptance Criteria

- [ ] Host outcome-tracking (`wait_for_ack`/equivalent) recognizes
      `ok:<id>`/`err:<id>:<code>`/`done:<id>:<reason>` lines and resolves
      the corresponding pending call.
- [ ] Repeated copies of the same `ok`/`err`/`done` (same id, same verb) are
      de-duplicated — first copy wins, repeats are silently ignored, not
      re-processed as new events.
- [ ] A `done` with `reason == 'timeout'` and a `done` with `reason ==
      'stop'` are distinguishable by the host without inspecting any flags
      bit.
- [ ] The binary ack ring's existing host-side read path is untouched and
      its tests still pass (additive-only ticket; ring removal is ticket
      013).

## Testing

- **Existing tests to run**: host ack/outcome-tracking suites.
- **New tests to write**: a test feeding a triplicate `ok:7`/`ok:7`/`ok:7`
  sequence and asserting exactly one resolution; a `done:7:stop` vs.
  `done:7:timeout` distinction test.
- **Verification command**: `uv run pytest src/tests -k "ack or outcome or
  protocol"`.
