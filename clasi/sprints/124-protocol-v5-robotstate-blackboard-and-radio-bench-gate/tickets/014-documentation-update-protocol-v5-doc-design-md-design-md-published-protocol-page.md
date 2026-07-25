---
id: '014'
title: 'Documentation update: protocol-v5 doc, design.md/DESIGN.md, published protocol
  page'
status: open
use-cases: []
depends-on: ['013']
github-issue: ''
issue: protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Documentation update: protocol-v5 doc, design.md/DESIGN.md, published protocol page

## Description

Last ticket — reflects the FINAL landed shape, after the bench gate
(ticket 013) has passed. Update `docs/protocol-v4.md` → a v5-titled
revision (rename or in-place version bump — implementer's call, see
sprint 124 Migration Concerns) covering: the uniform grammar, the
generated command registry (ticket 001), `ID`/`VER`/`PONG` (ticket 005),
packed telemetry field layout including `positionEpoch` (ticket 008),
and the `RobotState`/`firm/types` boundary crossing (ticket 007).

Also update `docs/design/design.md`, `src/firm/app/DESIGN.md`,
`src/firm/messages/DESIGN.md` for the `firm/types` crossing and
`TelemetrySecondary`'s deletion, and the published protocol page at
https://robots.jointheleague.org/.

## Acceptance Criteria

- [ ] `docs/protocol-v5.md` (or the in-place revised equivalent)
      documents the final grammar, the command registry's verb table,
      `ID`/`VER`/`PONG`, and the packed telemetry field list including
      `positionEpoch` and its rebaseline semantics.
- [ ] `docs/design/design.md` §2/§5 updated: `firm/types` is now a
      second shared floor alongside `Motion::WheelSink` — reflect this
      explicitly, not just note the directory exists.
- [ ] `src/firm/app/DESIGN.md` and `src/firm/messages/DESIGN.md`
      updated for `TelemetrySecondary`'s deletion and the new packed
      encoding.
- [ ] The published protocol page (robots.jointheleague.org) is updated
      to match the landed doc, not left pointing at protocol v4.

## Testing

- **Existing tests to run**: N/A (documentation).
- **New tests to write**: N/A — manual verification that the doc's
  command-registry table matches the generated registry from ticket 001
  (diff by hand or a lightweight script).
- **Verification command**: N/A.
