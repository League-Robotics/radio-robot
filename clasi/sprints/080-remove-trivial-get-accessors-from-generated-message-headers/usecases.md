---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 080 Use Cases

## SUC-001: Firmware developer reads a generated message field directly, with no dead pass-through accessor to call instead
Parent: UC-049 (new — "Generated Code Emits Only Conforming, Reachable API")

- **Actor**: Firmware/tooling developer authoring or maintaining code in
  `source/` or `tests/sim/unit/` that consumes a `msg::*` struct emitted by
  `scripts/gen_messages.py`.
- **Preconditions**: `scripts/gen_messages.py` generates `source/messages/*.h`
  from `protos/*.proto`. Chainable setters still exist for `Command`/`Config`
  types; no `get_*` accessor method exists on any generated struct.
- **Main Flow**:
  1. Developer needs to read a field of a generated struct — a plain scalar
     (`config.trackwidth`), an `Opt<T>` field (`command.feedforward.has` /
     `.val`), a oneof discriminator (`command.control_kind`), or a message/
     enum/string field.
  2. Developer reads the public field directly. There is no `get_*` method to
     reach for instead — the naming-and-style / coding-standards rules (no
     snake_case function names, no `get_` prefix) are trivially satisfied
     because the generator emits no accessor method for these fields at all.
  3. The six converted call sites (`source/subsystems/drivetrain.cpp`,
     `source/subsystems/communicator.cpp`, `source/commands/dev_commands.cpp`,
     `source/hal/capability/motor.h`, `tests/sim/unit/drivetrain_harness.cpp`,
     `tests/sim/unit/dev_command_outbox_harness.cpp`) compile and behave
     identically to before — same field, same value, one fewer indirection.
- **Postconditions**: No `source/messages/*.h` struct defines a `get_*`
  method. Every former `.get_foo()` call site now reads `.foo` (or `.foo_kind`
  for a oneof discriminator) directly.
- **Acceptance Criteria**:
  - [ ] `grep -rn "get_[a-z_]*(" source/messages/` returns nothing.
  - [ ] `grep -rn "\.get_[a-z_]*(" source/ tests/ --include=*.cpp --include=*.h`
        (excluding `source/messages/`) returns nothing.
  - [ ] `just build` and `uv run python -m pytest` are green, including the
        compiled `tests/sim/unit/drivetrain_harness.cpp` and
        `tests/sim/unit/dev_command_outbox_harness.cpp` binaries.
  - [ ] Both `ROBOT_DEV_BUILD` forks build (`dev_commands.cpp`, the only
        fork-gated file among the six, compiles under the `ROBOT_DEV_BUILD=1`
        default and the sweep does not regress the `ROBOT_DEV_BUILD=0` fork).

## SUC-002: Codegen maintainer is warned if a trivial getter reappears
Parent: UC-049 (narrows)

- **Actor**: Codegen/tooling maintainer editing `scripts/gen_messages.py`
  (e.g. adding a new proto message or field, or refactoring `_emit_message`).
- **Preconditions**: the getter-emission regression guard (new test) exists
  and runs under `uv run python -m pytest`.
- **Main Flow**:
  1. Maintainer edits `_emit_message` — e.g. by mistake, or via a bad merge,
     reintroduces a `get_*`-prefixed accessor branch.
  2. The guard test invokes the generator (in-process or via its `--dry-run`
     output) and asserts no emitted method name matches `get_[a-z_]*\(`.
  3. The test fails, naming the offending struct/field, before the change
     reaches a build or a reviewer.
- **Postconditions**: the regression is caught by the existing test gate
  without requiring a human to grep the generator on every change.
- **Acceptance Criteria**:
  - [ ] A new test asserts zero `get_*`-prefixed methods across every message
        emitted for all proto files (excluding the hand-authored `bridges.h`,
        which is not proto-generated).
  - [ ] The test passes against the sprint's regenerated generator output.
  - [ ] The test is part of the standard `uv run python -m pytest` run (no
        separate invocation needed).
