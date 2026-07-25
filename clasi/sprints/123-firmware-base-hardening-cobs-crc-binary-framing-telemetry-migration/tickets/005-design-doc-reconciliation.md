---
id: '005'
title: Design-doc reconciliation
status: open
use-cases: [SUC-001, SUC-002, SUC-003, SUC-004, SUC-005]
depends-on: ['002', '003', '004']
github-issue: ''
issue: cobs-crc-binary-framing-replace-base64-armor.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Design-doc reconciliation

## Description

Update every design doc this sprint's changes touch, so
`close_sprint`'s design validation passes and a future reader finds the
docs accurate: `docs/design/design.md` (§5 "Wire boundary" and the "122"
note's own forward-reference to this sprint), `src/firm/com/DESIGN.md`
(transport binary-cleanliness), `src/firm/messages/DESIGN.md` (base64
invariant replaced by COBS+CRC invariant, budget recompute reflected),
`src/firm/app/DESIGN.md` (Comms framing, telemetry migration completed —
resolves the "122-003" interim-placement note), `src/host/DESIGN.md` and
`src/host/robot_radio/DESIGN.md` (host decoder framing).

## Acceptance Criteria

- [ ] All six docs above updated to describe COBS+CRC as the CURRENT
      framing (not "planned" or "interim") once tickets 002-004 land.
- [ ] `messages/DESIGN.md`'s §3 base64-alphabet invariant either removed
      or clearly marked historical/superseded.
- [ ] `close_sprint`'s design validation (`validate_design`) passes with
      no dangling or missing `DESIGN.md`.
- [ ] `docs/design/design.md`'s own "122 (motion-library extraction...)"
      note, which explicitly forward-references "a future COBS+CRC
      framing rework," is updated to point at this sprint as landed,
      not future.

## Implementation Plan

- **Approach:** One documentation-only ticket, sequenced last among the
  code tickets so it reconciles against the FINAL, settled
  implementation (mirroring sprint 122 ticket 004's own precedent for
  this exact kind of ticket).
- **Files:** the six docs named above.
- **Testing:** `clasi design validate` (or the `validate_design` MCP tool).
- **Documentation:** this ticket IS the documentation update.
