---
status: done
sprint: 079
tickets:
- 079-002
- 079-005
- 079-006
---

# Rename wire-protocol lines from "commands" to "statements"

Stakeholder decision 4 of the 2026-07-04 tick-model design session — see
[tick-model-command-flow-and-the-command-board-design-sketch.md](tick-model-command-flow-and-the-command-board-design-sketch.md).

## Problem

Too many things are called "command." Lines arriving from the serial port /
radio relay AND the internal `msg::` control messages both use the word, which
muddies the design language now that the tick-model design distinguishes them
sharply:

- A **statement** is one wire line: verb, args, kv pairs, correlation id.
- Parsing a statement yields a **command**: `msg::*Command` and the
  `<Producer>To<Consumer>Command` edges keep the name.
- The Communicator produces statements; the processor consumes statements and
  produces commands.

## Scope

1. **Naming rule amendment**: `.claude/rules/naming-and-style.md` rule 4 —
   edge types become `<Producer>To<Consumer><Payload>` with payload ∈
   {Command, Statement}.
2. **Edge type rename**: `CommunicatorToCommandProcessorCommand` →
   `CommunicatorToCommandProcessorStatement`
   (`source/subsystems/communicator.h`, implemented 2026-07-04 in commit
   2599df3), plus its doc comments ("command line" → "statement").
   NOTE: the tick-model design issue separately changes this edge from
   returned-from-tick to held+taken (`hasStatement()`/`takeStatement()`) —
   coordinate if both land in the same sprint.
3. **Doc-language sweep**: `docs/protocol-v2.md` and source comments where
   "command line" means the wire line rather than a parsed command. Wire
   strings themselves (verbs, reply text) are NOT renamed — this is
   source/doc vocabulary only, no wire-format change.
4. **CommandProcessor naming call**: initial recommendation is KEEP the class
   name (it consumes statements but produces command dispatches); record the
   decision either way. The DEV "command family" keeps its name outright
   (DEV M / DEV DT are commands once parsed).

## Notes

- Grep confirmed "statement" is collision-free in the active `source/` tree
  (one hit, a comment about a C++ switch statement).
- Low priority; mechanical; batch-dispatch friendly.
