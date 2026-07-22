---
id: 009
title: Protocol document (protocol-v4)
status: in-progress
use-cases:
- SUC-050
depends-on:
- '006'
- '007'
github-issue: ''
issue:
- gut-to-minimal-firmware-motion-stack-excision-move-protocol-minimal-telemetry.md
- protocol-set-point-the-minimal-firmware-s-complete-command-surface.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Protocol document (protocol-v4)

## Description

Writes `docs/protocol-v4.md` — the converged command-surface contract the
minimal firmware speaks once this sprint closes: transport/framing,
command table, the `Move` message shape (both velocity variants, all
three stop kinds, `timeout`/`replace`/`id`), response semantics (single
ack slot, completion ack against `Move.id`, the move-timeout fault flag),
and the full `ErrCode` error taxonomy — matching the protocol-set-point
issue exactly as actually shipped by tickets 001-007 (not just what the
issue proposed; if implementation deviated anywhere — e.g. the
zero-threshold or flushed-Move-ack conventions ticket 002/005 had to pin
down per the Architecture's Open Questions — the doc reflects the real,
shipped behavior). `docs/protocol-v2.md` and `docs/protocol-v3.md` each
gain a short superseded banner at the top pointing at v4 — kept, not
deleted, as the historical record of what shipped when (same posture the
project has taken with every prior protocol revision).

This is a documentation deliverable rather than a runtime behavior, so it
doesn't map to a dedicated SUC of its own — traced to SUC-050 (the core
"command a bounded MOVE" use case) as the closest behavioral anchor for
what it documents, per sprint.md's own Success Criteria bullet ("the
protocol document lands in `docs/` and matches the shipped contract
exactly"). Depends on 006 (firmware) and 007 (host) both landing first —
otherwise there's no "as shipped" contract yet to document.

## Acceptance Criteria

- [ ] `docs/protocol-v4.md` written: transport/framing (unchanged from
      v2/v3 — line-based, `*B`-armored binary plane, two-verb text plane),
      full command table (`HELLO`/`PING`/`config`/`stop`/`move`), the
      `Move` message shape, response semantics (single ack slot,
      completion ack, fault flag), and the complete `ErrCode` taxonomy.
- [ ] Every documented behavior is verified against the actually-shipped
      code (tickets 001-007), not just restated from the protocol
      set-point issue's proposal — in particular, wherever an Open
      Question from sprint.md's Architecture section required a runtime
      decision (zero-threshold handling, flushed-pending-Move ack
      policy), the doc states the decision that was actually implemented.
- [ ] `docs/protocol-v2.md` and `docs/protocol-v3.md` each gain a
      superseded banner pointing at `protocol-v4.md`; neither file is
      deleted.

## Testing

- **Existing tests to run**: none (documentation-only ticket) — confirm
  `python build.py` still builds clean (no doc change should touch
  generated code).
- **New tests to write**: none; this ticket's own acceptance criteria are
  the verification (doc content cross-checked against shipped behavior,
  by inspection).
- **Verification command**: n/a (documentation ticket) — `git diff
  --stat` should show only `docs/` files touched.
