---
id: '004'
title: 'Remove statement terminology: rename to command/message vocabulary'
status: open
use-cases: [SUC-005]
depends-on: []
github-issue: ''
issue: remove-statement-terminology.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Remove statement terminology: rename to command/message vocabulary

## Description

The stakeholder has reversed sprint 079's "statement" naming decision
(2026-07-07): things arriving over radio/serial are "commands"; internal
typed representations are "messages" — there is no third "statement"
category. `statement`/`Statement` appears in 101 occurrences across 17
files in `source/` plus 1 in `host/` (confirmed by direct grep during
planning), concentrated in `source/subsystems/communicator.{h,cpp}`,
`source/runtime/{blackboard.h, command_router.{h,cpp}}`, and
`source/subsystems/statement.h`. This is a large but purely mechanical
rename — no behavior changes. Per `architecture-update.md` Decision 1
(approved by the stakeholder): the wire-inbound edge is renamed to end in
`...Command`; pre-existing `...Command`-suffixed edges that already carry
a parsed `msg::*` struct (e.g. `Hal::DrivetrainToHardwareCommand`) are
grandfathered and NOT touched — do not expand this rename beyond the
`statement`/`Statement` footprint.

## Implementation Plan

**Approach**: A mechanical grep-and-rename batch, done fast (per project
convention for mechanical sprints — batch the edits, don't trickle them):

1. `Subsystems::CommunicatorToCommandProcessorStatement` →
   `Subsystems::CommunicatorToCommandProcessorCommand`.
2. `source/subsystems/statement.h` → `source/subsystems/wire_command.h`
   (update every `#include`).
3. `Communicator::hasStatement()`/`takeStatement()` →
   `hasCommand()`/`takeCommand()`.
4. `Rt::Blackboard::statementsIn` → `commandsIn`.
5. Sweep every remaining `statement`/`Statement` occurrence (identifiers
   and comments) across `command_processor.{h,cpp}`, `dev_commands.h`,
   `motion_commands.h`, `telemetry_commands.cpp`, `radio.h`,
   `nezha_motor.cpp`, `main.cpp`, `main_loop.{h,cpp}`, `commands.h`,
   `command_router.{h,cpp}`, `communicator.{h,cpp}` — reword each comment
   to "command" (wire-inbound) or "message" (internal `msg::*`) per what
   it actually meant, not a blind find-replace.
6. Rename the one host site: `host/robot_radio/io/preview.py:5`'s comment.
7. Rewrite `.claude/rules/naming-and-style.md` rule 4: drop the
   `Statement` payload type, state the command (wire-inbound) / message
   (internal) split, and explicitly document the resulting `...Command`
   edge-payload overload with pre-existing internal edges as intentional
   and grandfathered (not a bug), noting full `...Command`→`...Message`
   consistency is a deferred future issue, out of scope here.
8. Update live docs referencing "statement" (`docs/protocol-v2.md`,
   `docs/kinematics-model.md` if applicable); leave archived/`done/`
   sprint history untouched.

**Files to create/modify**: the 17 `source/` files identified above (12
distinct files after grouping `.h`/`.cpp` pairs), `host/robot_radio/io/preview.py`,
`.claude/rules/naming-and-style.md`, and any live (non-archived) docs
referencing "statement".

**Testing plan**: no new test content — acceptance is a clean grep sweep
plus a fully green build/test suite (proving zero behavior change).

**Documentation updates**: `.claude/rules/naming-and-style.md` rule 4;
`docs/protocol-v2.md` (and `docs/kinematics-model.md` if it references the
removed term).

## Acceptance Criteria

- [ ] `grep -rn "[Ss]tatement" source/ host/` returns nothing, excluding
      `tests_old/`, `source_old/`, and archived sprint/architecture
      history (`clasi/sprints/*/done/`, `docs/architecture/done/`).
- [ ] `CommunicatorToCommandProcessorStatement` →
      `CommunicatorToCommandProcessorCommand`; `source/subsystems/statement.h`
      → `source/subsystems/wire_command.h`; `hasStatement()`/`takeStatement()`
      → `hasCommand()`/`takeCommand()`; `statementsIn` → `commandsIn` —
      renamed at every call site (not just the definition).
- [ ] `.claude/rules/naming-and-style.md` rule 4 no longer contains a
      `Statement` payload type; documents the command/message split;
      explicitly notes the `...Command` edge-payload overload with
      pre-existing internal edges is intentional/grandfathered and that
      further consistency renaming is a deferred future issue.
- [ ] Firmware builds clean; `uv run python -m pytest` is fully green with
      no test-assertion changes required (pure rename — only code that
      references the renamed identifiers needs edits to compile).
- [ ] `docs/protocol-v2.md` and other live docs referencing "statement"
      are updated to the new vocabulary.

## Testing

- **Existing tests to run**: full `uv run python -m pytest` — must stay
  green with the rename applied consistently.
- **New tests to write**: none (pure rename; no new behavior to test).
- **Verification command**: `grep -rn "[Ss]tatement" source/ host/`
  (expect empty, modulo excluded paths) then `uv run python -m pytest`.
