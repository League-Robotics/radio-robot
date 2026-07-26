---
id: '011'
title: 'Move/config enqueue-ack observability: confirm the post-123-006 fix, or fix
  if not'
status: in-progress
use-cases:
- SUC-007
depends-on:
- '005'
- 008
github-issue: ''
issue: telemetry-physical-layer-corruption-and-move-ack-observability.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Move/config enqueue-ack observability: confirm the post-123-006 fix, or fix if not

## Description

The linked issue's "historical framing" section records `move_protocol_bench.py`
scoring 8/43 pre-123-006, with `move_wheels`/`move_twist`/`config`
enqueue acks returning `ack=None` (`acks[]` ring empty, `ack_fresh`
never set) while STOP acks and motion execution worked fine. This may
already be resolved by 123-006's other fixes, or may resurface under
this sprint's new packed-ack encoding (ticket 008) — confirm which, and
fix if the defect is still present.

Re-run the enqueue-ack observability check against the current (v5)
wire/packed-ack format. If the gap has genuinely closed, this ticket's
job is to prove it and add a regression test so it cannot silently
regress again under a future encoding change. If it hasn't closed, fix
it: localize whether `handleMove()`'s `tlm_.ack(result.corrId, ...)` is
receiving a nonzero `corrId` and whether the enqueue ack reaches the
(now-packed) ring.

## Acceptance Criteria

- [ ] A fresh run of the enqueue-ack observability check scores
      enqueue acks observed for `move_wheels`/`move_twist`/`config` —
      not just `STOP` — matching or exceeding STOP's reliability.
- [ ] If the defect is still present, root-caused and fixed (confirm
      `result.corrId` is nonzero for a normal enqueue and that
      `tlm_.ack()` pushes to the packed ring from ticket 008).
- [ ] A regression test is added (or an existing one strengthened) so
      this specific defect class cannot silently regress under the new
      packed-ack encoding.
- [ ] Motion is independently confirmed to actually execute in the
      passing case (the original issue noted this wasn't verified in
      the no-ack repro) — not just that an ack was observed.

## Testing

- **Existing tests to run**: `move_protocol_bench.py` (or its v5
  successor, coordinated with ticket 012's bench-script promotion).
- **New tests to write**: the regression test above; a bench run
  scoring enqueue-ack observability across `move_wheels`/`move_twist`/
  `config`.
- **Verification command**: bench run per
  `.claude/rules/hardware-bench-testing.md`.
