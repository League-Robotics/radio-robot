---
id: '001'
title: Config field table generator (shared declaration -> firmware + host tables)
status: open
use-cases: [SUC-001, SUC-002, SUC-006]
depends-on: []
github-issue: ''
issue:
- protocol-v6-spec.md
- protocol-and-implementation-simplification-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Config field table generator (shared declaration -> firmware + host tables)

## Description

Reshape `src/scripts/gen_messages.py` (and `src/protos/`) to emit three flat
generated data tables from one retained declaration: the verb table (30
verbs, spec §3), the 80-row config field table (`name, offset, type, scale,
min, max`, spec §7.3), and the telemetry column tables (`POSE`/`FULL`, spec
§6.3/§6.4) — one generated matching pair (a C++ table and a Python table) per
declared table, so there is exactly one place each verb/field/column name is
written. This is Design Rationale Decision 2 in `sprint.md`: it is what makes
the v5-era `pid.kff -> kaff` drift class (review F4) structurally
impossible — no second hand-maintained copy to drift from.

This ticket lands the generator and its generated output only. It does not
wire `GET`/`SET`/`TLM`/dispatch against the tables — that is tickets
002/004/008. The bar here is a green build with the new generated tables
present and unit-tested against the exact name lists in spec §6.3/§6.4/§7.3.

Decide and record the declaration syntax (`sprint.md` Step 7's open
question — a trimmed `.proto`-style IDL vs. a lighter declarative format).
Either is architecturally fine: the property that matters is one
declaration producing two generated outputs, not the syntax chosen.

## Acceptance Criteria

- [ ] One shared declaration source produces both a generated C++ table
      (`src/firm/messages/` or `src/firm/config/`) and a generated Python
      table (`src/host/robot_radio/config/` or similar), for all three table
      kinds: verb table (30 rows, spec §3.1/§3.2), config field table (80
      rows, spec §7.3, names verbatim from v5's `robot_config.proto`), and
      telemetry column tables (`POSE` 9 cols, `FULL` 30 cols, spec §6.3/§6.4).
- [ ] Field/column/verb names match spec §7.3/§6.3/§6.4/§3 exactly — a unit
      test enumerates the generated table and diffs it against the spec's
      own name lists.
- [ ] `grep -rn "<an existing sibling .cpp>" src --include=*.txt
      --include=*.py` is run to enumerate every hand-maintained source list
      (CMake plus every `src/tests/sim/**/test_*.py` file that carries its
      own `_APP_SOURCES` list and compiles a throwaway binary via a bare
      `c++` call) BEFORE this ticket's new firmware `.cpp`/`.h` files are
      considered done, and every new file is added to every list found.
- [ ] `src/firm/types/version_generated.h` is generated (via `build.py`, or
      `src/scripts/gen_version.py` + `gen_boot_config.py`) before this
      ticket's "before" smoke-test baseline is captured — a pristine tree
      otherwise yields ~23 phantom failures.
- [ ] The old protobuf-generating path is left intact and still builds
      (additive ticket — the binary plane is not touched until ticket 013).
- [ ] No identifier in generated or hand-written code starts with an
      uppercase letter for a function/method, and no identifier encodes a
      unit (units in `// [unit]` / `# [unit]` trailing comments per
      `.claude/rules/coding-standards.md`) — this does not apply to the wire
      key strings/table row names themselves, which are wire-visible and
      exempt.

## Testing

- **Existing tests to run**: `src/tests/sim/unit` (generator/codegen-adjacent
  tests), a firmware build, `uv run python -m pytest src/tests` scoped to any
  existing codegen tests.
- **New tests to write**: a unit test asserting the generated config-field
  table has exactly 80 rows with names matching spec §7.3's group/field
  lists; a unit test asserting the generated telemetry column tables match
  §6.3 (9 columns) and §6.4 (30 columns) in the documented order.
- **Verification command**: `uv run pytest src/tests -k codegen`, plus a
  firmware build.
