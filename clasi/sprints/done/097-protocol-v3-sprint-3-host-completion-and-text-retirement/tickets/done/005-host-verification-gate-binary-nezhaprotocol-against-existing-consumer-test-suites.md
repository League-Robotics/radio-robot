---
id: '005'
title: 'Light end-to-end verification: binary command plane + rogo proxy in sim; testgui
  baseline'
status: done
use-cases:
- SUC-005
depends-on:
- '001'
- '002'
- '003'
- '004'
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Light end-to-end verification: binary command plane + rogo proxy in sim; testgui baseline

## Description

**REWRITTEN — see `architecture-update-r2.md` Decision 9.** This ticket
is no longer a GO/NO-GO gate blocking firmware deletion — under Decision
9, tickets 006/007/008 gut the firmware text plane unconditionally
(`depends-on: []`), not contingent on this ticket's verdict. Its purpose
narrows to a light, honest end-to-end confidence check of what 097's host
side actually built, plus recording the `tests/testgui` baseline for the
record (informational — Decision 9 already accepts that TestGUI and the
other named legacy consumers break until separately rewired to the
proxy, so this ticket does not gate anything on that number staying flat).

**What this ticket verifies**:

1. The binary command plane (095/096, plus tickets 001/002/003's host
   completion) still works end-to-end in sim: `NezhaProtocol`'s converted
   methods (`ping`/`echo`/`get_id`/`get_ver`/`stop`/`drive`/`timed`/
   `distance`/`get_config`/`set_config`/`stream`/`snap`) round-trip
   correctly against the sim harness.
2. The rogo translator proxy (ticket 004) works end-to-end in sim: a text
   client connected to the proxy socket gets correct behavior while the
   proxy itself speaks only binary underneath — this is largely
   ticket 004's own acceptance criteria re-run as a combined smoke pass,
   not new coverage invented here.
3. Record the CURRENT `tests/testgui` failure count as a baseline
   snapshot for the record (NOT a gate — Decision 9 already accepts this
   number may increase once tickets 006/007/008 land, since TestGUI is a
   named consumer that breaks under the new model until rewired to the
   proxy). State plainly in the completion notes that this baseline is
   informational, not a constraint on 006/007/008.

## Acceptance Criteria

- [x] `tests/sim` is green.
- [x] `tests/unit` is green, including ticket 004's own new proxy tests.
- [x] `tests/testgui` is run once; the failure count is recorded in this
      ticket's completion notes as a dated snapshot, explicitly labeled
      "informational baseline, not a gate — Decision 9 accepts this
      number may rise once 006/007/008 land."
- [x] A short manual/scripted smoke pass exercises at least one verb per
      converted family (drive/segment/replace/config/telemetry) through
      BOTH `NezhaProtocol` directly and through the `rogo` proxy, against
      the sim harness.
- [x] Completion notes state plainly that this ticket does NOT gate
      tickets 006/007/008 — they proceed regardless of this ticket's
      outcome, per Decision 9's unconditional-gut model.

## Implementation Plan

### Approach

1. Run `tests/sim` and `tests/unit` in full; capture pass/fail counts.
2. Run `tests/testgui`; record the failure count and names as a dated
   snapshot (comparison-only, not pass/fail).
3. Exercise `NezhaProtocol` and the `rogo` proxy against the sim harness
   for one representative verb per family.
4. Write a short summary (not a go/no-go verdict — there is nothing this
   ticket is gating).

### Files to modify

- None expected (verification ticket).

### Testing plan

- `uv run python -m pytest tests/unit` — must be green.
- The project's `tests/sim` CI-gate command — must be green.
- `tests/testgui` run for the informational baseline snapshot only.

### Documentation updates

- None — this ticket's output is its own completion notes.

## Completion Notes (2026-07-10, team-lead verification)

Light end-to-end verification run (NOT a gate — Decision 9 guts firmware text unconditionally):

- `tests/sim`: **597 passed** (85s) — binary command plane (095/096) + host completion (001/002/003) intact after the gut.
- `tests/unit`: **239 passed** (13s) — includes ticket 004's rogo PTY-proxy tests (legacy_verbs/legacy_render/bridge_routing/bridge_pty_e2e).
- `tests/testgui`: **16 failed / 348 passed** (190s) — INFORMATIONAL baseline snapshot. Same 16 pre-existing failures as before the gut (TestGUI is a named consumer that talks raw text and is broken by Decision 9's unconditional gut until rewired to the rogo PTY proxy; owned by `realign-host-tooling-to-gutted-four-verb-wire-surface.md`). NOT a gate; Decision 9 accepts this number.
- Proxy + NezhaProtocol per-family smoke is covered by ticket 004's own passing test client (bridge_pty_e2e: real openpty + pyserial client) and the binary-channel/differential sim suites — one+ verb per family (drive/segment/replace/config/telemetry) round-trips both directly and through the proxy.

This ticket does NOT gate 006/007/008 (already landed unconditionally per Decision 9).
