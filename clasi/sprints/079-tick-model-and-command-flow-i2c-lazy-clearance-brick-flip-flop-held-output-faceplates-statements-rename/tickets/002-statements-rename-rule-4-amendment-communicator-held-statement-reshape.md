---
id: '002'
title: Statements rename, rule-4 amendment, Communicator held-statement reshape
status: open
use-cases: [SUC-007]
depends-on: []
github-issue: ''
issue:
- rename-wire-lines-to-statements.md
- tick-model-command-flow-and-the-command-board-design-sketch.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Statements rename, rule-4 amendment, Communicator held-statement reshape

## Description

Land the statements vocabulary (decision 4) and reshape `Subsystems::
Communicator` to the three-beat held-output pattern (decision 6's
Communicator half) in the same ticket, so `communicator.h` is touched
exactly once (per the design sketch's explicit intent).

**Vocabulary** (`rename-wire-lines-to-statements.md`):
- `.claude/rules/naming-and-style.md` rule 4 amended:
  `<Producer>To<Consumer>Command` → `<Producer>To<Consumer><Payload>`, payload
  ∈ {Command, Statement}. Update the rule text and its example.
- Edge type rename: `CommunicatorToCommandProcessorCommand` →
  `CommunicatorToCommandProcessorStatement` (comment doc "command line" →
  "statement line" throughout `communicator.h`).
- Doc sweep: `docs/protocol-v2.md` and source comments where "command line"
  means the wire line, not a parsed command — grep for "command line" /
  "command-line" repo-wide (a handful of hits in `communicator.h`,
  `command_processor.h`, `dev_commands.h`, `com/radio.h` per the
  architecture doc's own grep) and correct each. Do **not** touch any wire
  string/verb/reply text — this is vocabulary only.
- `CommandProcessor`'s class name is **kept** (recorded decision, per the
  issue's own initial recommendation and this sprint's architecture doc
  Design Rationale 7) — no rename needed for that class.

**Communicator reshape** (tick-model decision 6, Part 2 of the design
sketch): `tick(uint32_t now)` changes from returning
`CommunicatorToCommandProcessorStatement` to `void`; add
`bool hasStatement() const` and
`CommunicatorToCommandProcessorStatement takeStatement()` (clears the held
flag). An untaken statement (main.cpp didn't call `takeStatement()` last
pass — should not happen in the intended wiring, but the contract must hold
regardless) causes `tick()` to **decline to poll its transports** — do not
overwrite `line_[]` while a statement is still held. `takeStatement()`
copies the line into the returned struct's own buffer (or the caller is
documented to copy before the next `tick()` — match whichever the current
`line_[256]` single-shared-buffer design already implies; the risk this
guards against is `line_[]` being overwritten by a serial poll before the
consumer reads it).

## Acceptance Criteria

- [ ] `.claude/rules/naming-and-style.md` rule 4 reads
      `<Producer>To<Consumer><Payload>` with the {Command, Statement}
      payload set and an updated example.
- [ ] `CommunicatorToCommandProcessorStatement` is the edge type name in
      `source/subsystems/communicator.h`; no reference to the old
      `CommunicatorToCommandProcessorCommand` name remains anywhere in
      `source/`.
- [ ] `Communicator::tick(uint32_t now)` returns `void`;
      `hasStatement()`/`takeStatement()` exist and behave per the class
      comment above.
- [ ] An untaken statement (test this explicitly) causes the next `tick()`
      to skip polling `serial_`/`radio_` — no line is silently dropped or
      overwritten.
- [ ] `grep -rn "command line" source/ docs/` (excluding wire-string
      literals) returns zero hits meaning "wire line" — remaining hits, if
      any, are genuinely about something else (e.g. a C++ `switch`
      statement, already confirmed collision-free by the source issue).
- [ ] No wire-format change: `docs/protocol-v2.md`'s verbs/reply text/keys
      are byte-identical before and after (diff review, not just tests).
- [ ] `CommandProcessor` class name unchanged.
- [ ] Both `ROBOT_DEV_BUILD` forks build; `main.cpp`'s current call site
      (`Subsystems::CommunicatorToCommandProcessorCommand in = comm.tick(now);`)
      is updated to the new held/taken shape — this ticket **does** touch
      `main.cpp` minimally (just the Communicator call site), full loop
      reshape is ticket 005's job.

## Implementation Plan

**Approach**: do the vocabulary/rule-4 amendment first (pure rename, no
behavior change, easy to review in isolation), then the Communicator
held-output reshape (behavior change, needs a host or bench smoke check),
then the minimal `main.cpp` call-site update so the tree still builds.

**Files to modify**:
- `.claude/rules/naming-and-style.md` — rule 4 text + example.
- `source/subsystems/communicator.h` — edge type rename, `tick()` signature,
  `hasStatement()`/`takeStatement()`, held-statement field(s), doc comment
  sweep.
- `source/subsystems/communicator.cpp` — `tick()` body: stop returning the
  edge, stage it, skip polling while held.
- `docs/protocol-v2.md` — "command line" → "statement line" language sweep
  (no wire content changes).
- `source/commands/command_processor.h`, `source/commands/dev_commands.h`,
  `source/com/radio.h` — comment-only "command line" → "statement"
  corrections per the grep.
- `source/main.cpp` — update the one `comm.tick(now)` call site to the new
  `hasStatement()`/`takeStatement()` shape (minimal — do not restructure the
  rest of the loop here; that's ticket 005).

**Testing plan**:
- Existing tests: `uv run python -m pytest`; `just build` both forks.
- New tests: a Communicator-level test (host-side if a harness exists for
  this subsystem, otherwise a bench smoke check) asserting: (a) a complete
  line produces `hasStatement()==true` and the correct `line`/`returnPath`;
  (b) calling `tick()` again **without** taking the statement does not
  advance to a second line even if one is queued in the driver; (c) after
  `takeStatement()`, `hasStatement()` is false and the next `tick()` can
  poll again.
- Stand check (light, not the full ticket 006 gate): confirm a serial round
  trip still works end to end after this ticket (`PING` → `OK PING`) since
  `main.cpp`'s call site changed.

**Documentation updates**: `docs/protocol-v2.md` sweep (above);
`.claude/rules/naming-and-style.md` (above). No architecture doc change
needed (already written this sprint).
