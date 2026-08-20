---
id: '005'
title: 'Host thdr:/t: parsing + tlm_log.py simplification'
status: open
use-cases: [SUC-002]
depends-on: ['004']
github-issue: ''
issue:
- protocol-v6-spec.md
- protocol-and-implementation-simplification-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host thdr:/t: parsing + tlm_log.py simplification

## Description

Add `thdr:`/`t:` line parsing to the host (`io/wire_codec.py`, additively,
alongside the existing protobuf `TLMFrame` decode), zipping each `t:` line
against the most recently received `thdr:` to produce the same
`TLMFrame`-shaped object bench scripts/TestGUI already read (`.pose`,
`.otos`, flag accessors) — preserved wherever the same information still
exists in `POSE`/`FULL` (`sprint.md`'s Impact on Existing Components).
Simplify `tlm_log.py` to "write the header row from `thdr:`, write data rows
from `t:`" — a valid CSV by construction, no separate schema maintained
host-side.

## Acceptance Criteria

- [ ] Host parses `thdr:`/`t:` line pairs into the existing `TLMFrame`
      public shape (or a documented, minimal successor) with no schema
      shared out-of-band — the header row alone is sufficient.
- [ ] `tlm_log.py` writes a valid CSV using the `thdr:` line as its header
      row directly, with no separate column-name table maintained
      host-side.
- [ ] A host that reconnects mid-stream and missed the `thdr:` line sends
      `TLM:NOW` to force a fresh frame with a header, and this is exercised
      by a test or documented bench recipe.
- [ ] The existing binary `TLMFrame` decode path is untouched and its tests
      still pass (additive-only ticket).

## Testing

- **Existing tests to run**: `src/tests` host telemetry parsing suites,
  `tlm_log.py`-adjacent tests if any.
- **New tests to write**: a test parsing a `thdr:`/`t:` pair into
  `TLMFrame` and asserting field values match; a `tlm_log.py` CSV-output
  smoke test.
- **Verification command**: `uv run pytest src/tests -k "telemetry or
  tlm"`.
