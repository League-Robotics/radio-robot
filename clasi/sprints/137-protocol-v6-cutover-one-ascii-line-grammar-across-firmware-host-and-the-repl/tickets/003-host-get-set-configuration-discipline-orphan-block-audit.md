---
id: '003'
title: Host GET/SET + configuration-discipline orphan-block audit
status: open
use-cases: [SUC-001]
depends-on: ['002']
github-issue: ''
issue:
- protocol-v6-spec.md
- protocol-and-implementation-simplification-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host GET/SET + configuration-discipline orphan-block audit

## Description

Add `get`/`set` methods to `NezhaProtocol` (`robot/protocol.py`) speaking the
ticket 002 ASCII verbs over the existing transport, following the same
NaN/range semantics. Separately, audit the five orphan robot-JSON blocks
named in `sprint.md`'s Scope section (`wheels`, `encoders`, `gripper`,
`peripherals`, `perception`): for each, determine whether ticket 001's
config field table (spec §7.3's 9 groups) has a consuming field for it.
Delete blocks with zero consumer; keep (and, if needed, wire into the field
table) blocks that have one. This is the
`.claude/rules/configuration-discipline.md` "delete it, don't wire it"
invariant in practice.

## Acceptance Criteria

- [ ] `NezhaProtocol.get(name)` / `.set(name, value, id=...)` round-trip
      against firmware ticket 002's `GET`/`SET`, returning a Python float
      (not a string) on `get`, raising on `err`.
- [ ] A read-back smoke test pushes a value with `set()`, then confirms via
      `get()` that it landed — never trusting the `ok` alone
      (`.claude/rules/configuration-discipline.md`'s read-back invariant:
      "config that acks OK and lands nowhere is a live failure mode in this
      codebase, not a hypothetical").
- [ ] Each of the five orphan blocks (`wheels`, `encoders`, `gripper`,
      `peripherals`, `perception`) is resolved: deleted from the robot JSON
      schema/loader if it has zero consumer in the ticket 001 field table,
      or documented as consumed if it does. The audit's outcome (which
      blocks were deleted vs. kept, and why) is recorded in this ticket's
      own notes.
- [ ] No robot JSON in `data/robots/*.json` is left referencing a deleted
      block.
- [ ] Python identifiers follow `.claude/rules/coding-standards.md`'s units
      rule (no unit suffixes; units in `# [unit]` comments); existing
      `robot_radio` snake_case Python naming style is preserved — the
      CamelCase override in `naming-and-style.md` applies to C++ only.

## Testing

- **Existing tests to run**: `src/tests` host-side `robot_radio`
  config/protocol suites.
- **New tests to write**: a `get`/`set` round-trip test against the sim
  harness; a test (or explicit audit note in this ticket) enumerating the
  five orphan blocks' disposition.
- **Verification command**: `uv run pytest src/tests -k "protocol or
  config"`.
