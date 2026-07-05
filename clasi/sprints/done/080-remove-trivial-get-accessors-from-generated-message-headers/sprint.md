---
id: 080
title: Remove trivial get_* accessors from generated message headers
status: done
branch: sprint/080-remove-trivial-get-accessors-from-generated-message-headers
use-cases:
- SUC-001
- SUC-002
issues:
- remove-generated-get-accessors.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 080: Remove trivial get_* accessors from generated message headers

## Goals

Stop `scripts/gen_messages.py` emitting protobuf-style `get_*` accessors on
generated `msg` structs; regenerate `source/messages/*.h`; confirm firmware
and test builds are clean.

## Problem

The generated snake_case `get_kind()`/`get_ax()` getters violate the naming
convention twice over (snake_case functions, `get_` prefix) and are dead API —
no call sites exist outside the generated headers. The "generated code exempt"
clause covers the output files, not the generator template.

## Solution

Remove the field-getter branches of `_emit_message` (around lines 643–688).
Non-trivial getters (oneof kind discriminators, `Opt<>` wrappers): drop if
unused, else rename to bare-name lowerCamelCase. Regenerate and rebuild.

## Success Criteria

No `get_*` methods in regenerated `source/messages/*.h`; firmware build and
full host test suite clean; generator emits conforming API for any getter
that survives.

## Scope

### In Scope

- `scripts/gen_messages.py` emitter template.
- Regenerated `source/messages/*.h`.
- A guard (test or generator assertion) that trivial getters stay gone.

### Out of Scope

- Hand-edits to generated headers (never allowed).
- Any change to message wire encoding or field layout.

## Test Strategy

Regenerate, diff to confirm only getter removal, build firmware
(`mbdeploy deploy --build` compile check or `just build`) and run the host
suite (`uv run python -m pytest`). Expected: zero call-site fallout (grep
verified in the issue). No hardware gate needed — no runtime behavior change;
a stand smoke after flash is a courtesy check only.

## Architecture Notes

- Stakeholder decision 2026-07-04: scheduled after sprint 077 closed —
  satisfied.
- Sequenced last: sprint 079 regenerates/touches messages usage broadly;
  landing this after avoids churn.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan (auto-approve mode, 2026-07-05)

## Tickets

| # | Title | Depends On | Issue Ref |
|---|-------|------------|-----------|
| 001 | Remove get_* accessors from generated message headers: generator, regen, full call-site sweep, and regression guard | — | remove-generated-get-accessors.md |

Tickets execute serially in the order listed. Ticket 001:
`tickets/001-remove-get-accessors-from-generated-message-headers-generator-regen-full-call-site-sweep-and-regression-guard.md`
