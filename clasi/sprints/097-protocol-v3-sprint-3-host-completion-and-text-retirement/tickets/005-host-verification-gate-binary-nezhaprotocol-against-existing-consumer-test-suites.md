---
id: '005'
title: 'Host verification gate: binary NezhaProtocol against existing consumer test
  suites'
status: open
use-cases: [SUC-005]
depends-on: ['001', '002', '003', '004']
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host verification gate: binary NezhaProtocol against existing consumer test suites

## Description

This is an explicit **GO/NO-GO gate**, not a feature ticket. Its job is to
prove the host is fully on the binary path (or the retained text rump)
**before any firmware text handler is deleted** (tickets 006/007/008).
Deleting a text handler before its host caller has verifiably moved off
it would strand every consumer of that verb with no recourse — this
ticket is what makes that sequencing safe rather than merely intended.

Run `tests/unit` and `tests/sim` (the CI gate — NOT the full pytest
collection) in full. Record the `tests/testgui` failure count as the
**pre-deletion baseline** — this tier has 16 known pre-existing failures
(tracked by the separate
`realign-host-tooling-to-gutted-four-verb-wire-surface.md` issue, out of
scope to fix here); this ticket's job is to confirm that number does not
increase because of tickets 001-004's changes, not to fix it.

Review (and, where feasible, exercise against the sim harness) the
bench scripts under `tests/bench/` that call the `NezhaProtocol` methods
converted in tickets 002/003, for continued correctness. Hardware bench
execution itself is the team-lead's post-sprint consolidated gate, not
this ticket's — this ticket's job is host-side confidence before firmware
deletion starts, not a hardware sign-off.

**If this ticket finds a host consumer still depending on a text family
tickets 006/007/008 intend to delete** (a call site tickets 001-004
missed, or a bench script/TestGUI code path not accounted for in the
architecture's Step 1 research), **the deletion ticket for that specific
family must not proceed until the gap is fixed** — either by a follow-up
fix within this ticket's own scope (if small) or by throwing an exception
per the Exception Protocol (if it reveals a genuine architecture gap, e.g.
a text family this document assumed was safe to delete but isn't). Do not
silently proceed to 006/007/008 with a known-unverified consumer.

## Acceptance Criteria

- [ ] `tests/sim` is green.
- [ ] `tests/unit` is green.
- [ ] `tests/testgui` failure count is recorded explicitly in this
      ticket's completion notes and is `<=` the pre-sprint baseline (16)
      — not fixed, not increased. If it increased, root-cause and fix
      before marking this ticket done (a regression here blocks 006/007/
      008, it is not acceptable to note-and-proceed).
- [ ] Every bench script under `tests/bench/` that calls a
      `NezhaProtocol` method converted in ticket 002/003 (`drive`,
      `timed`, `distance`, `stop`, `ping`, `echo`, `get_id`, `get_ver`,
      `get_config`, `set_config`, `stream`, `snap`) is reviewed; each is
      either confirmed correct against the sim harness, or a gap is
      logged and fixed (see Description's "no proceed with known-
      unverified consumer" rule).
  - [ ] `rogo send`/`rogo binary` are manually exercised against the sim
        harness (or bench, if available) for at least one representative
        verb per converted family (drive/segment/replace/config/
        telemetry), confirming SUC-004's own acceptance criteria.
- [ ] A written go/no-go verdict is recorded in this ticket's completion
      notes, explicitly naming: (a) which text families are confirmed
      safe to delete in 006/007/008, and (b) any family found NOT yet
      safe, with what blocks it.

## Implementation Plan

### Approach

1. Run `tests/sim` and `tests/unit` in full; capture pass/fail counts.
2. Run (or attempt to run) `tests/testgui`; record the exact failure
   count and diff it against the known 16-failure baseline (cite the
   specific failing test names if the count differs at all, so a
   regression is traceable).
3. For each bench script under `tests/bench/` that imports/calls
   `NezhaProtocol`, grep for which converted methods it uses; for each,
   either run it against the sim harness (where the script supports sim
   mode) or manually trace the call through tickets 002/003's diff to
   confirm correctness.
4. Manually exercise `rogo send`/`rogo binary` against the sim harness for
   at least one verb per family.
5. Write the go/no-go verdict.

### Files to modify

- None expected (this is a verification ticket). If a gap is found and
  the fix is small, it may touch files from tickets 001-004's own
  boundaries (`serial_conn.py`, `protocol.py`, `legacy_translate.py`,
  `cli.py`) — if so, document exactly what was found and fixed, and why
  it wasn't caught by the originating ticket's own acceptance criteria.

### Testing plan

- `uv run python -m pytest tests/unit` — must be green.
- The project's `tests/sim` CI-gate command (per `.claude/rules/hardware-
  bench-testing.md` and the project's own test-runner convention) — must
  be green.
- `tests/testgui` run for failure-count recording only (not a pass/fail
  gate at 16 or fewer — record the exact number, don't average it out).
- Bench-script review as described above.

### Documentation updates

- None — this ticket's output is its own completion notes (the go/no-go
  verdict), not a doc file.
