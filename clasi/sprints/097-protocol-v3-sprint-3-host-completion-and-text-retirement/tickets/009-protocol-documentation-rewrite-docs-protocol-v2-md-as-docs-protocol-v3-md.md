---
id: '009'
title: 'Protocol documentation: rewrite docs/protocol-v2.md as docs/protocol-v3.md'
status: open
use-cases: [SUC-010]
depends-on: ['006', '007', '008']
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Protocol documentation: rewrite docs/protocol-v2.md as docs/protocol-v3.md

## Description

Rewrite `docs/protocol-v2.md` (2252 lines) as `docs/protocol-v3.md`,
describing the actual, final wire surface once tickets 006/007/008 have
landed (deliberately sequenced last among the content tickets, so it
documents a stable target, not a moving one):

- Envelope framing: `*B<base64(envelope_bytes)>\n`, the armor/dearmor
  rule, the 186-byte per-arm payload cap.
- Every implemented `CommandEnvelope`/`ReplyEnvelope` oneof arm: drive,
  segment, replace, stop, ping, echo, id, config, get, stream — field
  shapes, validation bounds (cite `protos/*.proto`, not restate every
  number by hand where the source is the source of truth), and which
  Blackboard queue/Configurator path each reaches.
- The five-verb text rump (PING, ID, HELLO, HELP, STOP) — hand-typeable,
  with STOP's explicit safety-affordance framing (a human with a raw
  serial terminal and no host program can always halt the robot).
- An explicit "parked, not on the wire" section naming R/TURN/G
  (Planner-bound, no binary replacement, un-parking owned by no current
  sprint), OTOS/pose text handlers (preserved as sprint 098's
  transcription reference; 098 owns their binary `pose`/`otos` arms),
  `dev_commands.cpp` (never had or needs a binary counterpart), and
  `handleTlm`/`QLEN` (bench-diagnostic rump, no binary substitute) — so a
  reader understands why each is absent from the wire without having to
  read this sprint's `architecture-update.md`.
- `rogo send`'s translator behavior (ticket 004) as the recommended human
  entry point, alongside `rogo binary <arm>` for direct envelope
  construction.

Mark `docs/protocol-v2.md` as superseded (a clear pointer at the top of
the file to `docs/protocol-v3.md`), do not delete it — history stays
reachable.

## Acceptance Criteria

- [ ] `docs/protocol-v3.md` exists and documents every implemented binary
      arm (drive/segment/replace/stop/ping/echo/id/config/get/stream) plus
      the five-verb text rump.
- [ ] `docs/protocol-v3.md` explicitly names and explains every preserved-
      but-not-on-the-wire family (R/TURN/G, OTOS/pose text, `dev_commands`,
      `handleTlm`/`QLEN`), including which future sprint (if any) owns
      each's eventual fate.
- [ ] `docs/protocol-v2.md` carries a clear, prominent superseded-by
      pointer to `docs/protocol-v3.md` at the top of the file; the file is
      NOT deleted.
- [ ] No section of `docs/protocol-v3.md` describes a verb tickets 006/
      007/008 deleted as if it were still live on the text plane.
- [ ] `tests/sim` stays green (documentation-only ticket; sanity check).

## Implementation Plan

### Approach

1. Read the final state of `source/commands/binary_channel.cpp`,
   `protos/envelope.proto`, `protos/motion.proto`, `protos/telemetry.proto`,
   `protos/config.proto` after tickets 006/007/008 land, as the source of
   truth for the wire surface (not this sprint's architecture document,
   which is a planning artifact, not the spec).
2. Draft `docs/protocol-v3.md` following `docs/protocol-v2.md`'s existing
   section structure where it still applies (overview, grammar, response
   taxonomy, error codes, `#id` correlation) and replacing verb-by-verb
   sections with the binary arm's field shapes + the rump's unchanged
   text sections.
3. Add the "parked, not on the wire" section, citing this sprint's
   architecture Decisions 5/6/7 by name/number so a future reader can find
   the full reasoning if needed.
4. Add the superseded-by banner to the top of `docs/protocol-v2.md`.

### Files to modify

- `docs/protocol-v3.md` (new)
- `docs/protocol-v2.md` (superseded banner only)

### Testing plan

- No code changes; `tests/sim` run as a sanity check per the sprint's
  blanket requirement.
- Manual cross-check: every verb named in `docs/protocol-v3.md` is
  grep-confirmed against the actual post-006/007/008 source tree (no
  stale claims).

### Documentation updates

- This ticket IS the documentation update.
