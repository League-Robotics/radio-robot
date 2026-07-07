---
status: done
sprint: 088
tickets:
- 088-004
---

# Remove the "statement" terminology: wire-inbound things are "commands", internal things are "messages"

## Context

The stakeholder introduced the term **"statement"** (sprint 079's "statements
rename") for the raw lines arriving over the radio/serial channels, to
distinguish an *unparsed wire line* from a *parsed* `msg::*Command`. It was
enshrined in the naming rules — [.claude/rules/naming-and-style.md](.claude/rules/naming-and-style.md)
rule 4 defines the edge-type payload vocabulary as `{Command, Statement}`, with
`CommunicatorToCommandProcessorStatement` as the canonical example ("payload =
Statement: one unparsed wire line").

**The stakeholder has decided this was a mistake and wants the notion of
"statement" removed entirely (2026-07-07).** The corrected vocabulary:

- **Things that arrive through the radio or serial channel are `commands`.**
- **Internal-to-the-system representations are `messages`** (`msg::*`).

There is no third "statement" category.

## Scope

`statement` / `Statement` currently appears in ~85 places in `source/`, 1 in
`host/`, the naming rule, and several docs:

- **Core edge type + carriers** (the substantive renames):
  - `Subsystems::CommunicatorToCommandProcessorStatement` (the raw inbound wire
    line) — [main.cpp:204-230](source/main.cpp#L204-L230),
    [blackboard.h:28,139](source/runtime/blackboard.h#L28),
    [command_router.cpp:56](source/runtime/command_router.cpp#L56),
    [command_router.h](source/runtime/command_router.h).
  - Blackboard cell `statementsIn` — [blackboard.h:4,27,140](source/runtime/blackboard.h#L27).
  - Communicator accessors `hasStatement()` / `takeStatement()` —
    [main.cpp:202-230](source/main.cpp#L202-L230), Communicator subsystem.
  - Comment-level uses across `command_processor.*`, `dev_commands.h`,
    `motion_commands.h`, `telemetry_commands.cpp`, `radio.h`, `nezha_motor.cpp`.
- **Host:** [host/robot_radio/io/preview.py:5](host/robot_radio/io/preview.py#L5).
- **The naming rule itself:** [.claude/rules/naming-and-style.md](.claude/rules/naming-and-style.md)
  rule 4 — must be rewritten to drop the `Statement` payload type and state the
  `command` (wire-inbound) vs `message` (internal) split. (Cross-check
  [.claude/rules/coding-standards.md](.claude/rules/coding-standards.md) and
  [docs/reference/google-cppguide-condensed.md](docs/reference/google-cppguide-condensed.md).)
- **Docs:** `docs/protocol-v2.md`, `docs/kinematics-model.md`, and the
  architecture-update docs (078/079/081/084/087, plus done/ ones) reference the
  term historically — update the live ones; leave archived/`done/` history as-is.

## Open question for planning

Rule 4 used "Statement" specifically to mark the *unparsed* wire line as distinct
from a *parsed* `msg::*Command`. Removing it collapses that distinction: under the
new vocabulary the raw inbound line is itself a "command." Decide the concrete
rename for the edge type `CommunicatorToCommandProcessorStatement` —
e.g. `CommunicatorToCommandProcessorCommand` — and how (if at all) the
raw-vs-parsed distinction is expressed now (the parsed form is a `msg::*` message;
the raw line is the inbound `command`). Restate rule 4 accordingly. This is a
stakeholder-mandated naming rule being changed by the stakeholder, so the rule
edit is in scope.

## Notes

- Reverses sprint 079's "statements rename"; it does not restore any pre-079
  name blindly — apply the new `command`/`message` split deliberately.
- Wire key strings and serialized tokens are **not** identifiers and are out of
  scope (per the coding-standards exclusions) — this is a source-identifier and
  documentation change only.

## Acceptance

- No `statement`/`Statement` identifiers remain in `source/` or `host/` (comments
  included where they describe the removed concept); the edge type and blackboard
  cell are renamed under the `command`/`message` vocabulary.
- [.claude/rules/naming-and-style.md](.claude/rules/naming-and-style.md) rule 4 no
  longer contains a `Statement` payload type and documents the
  command-(wire)/message-(internal) split.
- Firmware and host build and all tests pass; behavior is unchanged (pure rename).
