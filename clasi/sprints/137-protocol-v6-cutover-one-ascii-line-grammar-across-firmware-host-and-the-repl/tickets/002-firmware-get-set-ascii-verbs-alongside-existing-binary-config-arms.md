---
id: '002'
title: Firmware GET/SET ASCII verbs (alongside existing binary config arms)
status: open
use-cases: [SUC-001]
depends-on: ['001']
github-issue: ''
issue:
- protocol-v6-spec.md
- protocol-and-implementation-simplification-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware GET/SET ASCII verbs (alongside existing binary config arms)

## Description

Add `GET`/`SET` ASCII verb handling to `Core::Comms`/`Core::Configurator`,
alongside — not replacing — the existing binary config arms
(`CONFIG`/`GET_CONFIG`/`SET_FIELD`/`CFG`). This is spec §13 migration stage
1. `SET:<name>:<value>[:<id>]` looks up ticket 001's config field table,
validates NaN-before-range (`ERR_BADARG` before `ERR_RANGE`, spec
§7.1/§7.4), applies now-or-next-boot per the field's own liveness, and
replies `ok:<id>`/`err:<id>:<code>`. `GET:[<name>]` prints
`get:<name>:<value>` for one field, or all 80 fields (one line each) for a
bare `GET`. Outbound floats use the hand-rolled `formatFixed()` helper (spec
§7.2), since `newlib-nano` has no `printf` float support; inbound uses
`strtof`, already used elsewhere (`Comms::stageSeed`).

## Acceptance Criteria

- [ ] `SET` on a live field takes effect immediately; `SET` on a boot-only
      field is stored and immediately visible via `GET` even before a
      reboot (spec §7.1's collapse of the boot-only/live split into "now"
      vs. "next boot").
- [ ] `SET` with a non-finite value is rejected `err:<id>:2` (`ERR_BADARG`)
      BEFORE the range check (`ERR_RANGE`), per spec §7.1's explicit
      ordering (NaN compares false against both `<` and `>`, so an
      unchecked NaN would pass any bound check).
- [ ] Bare `GET` returns all 80 declared fields, one `get:name:value` line
      each, values formatted via `formatFixed()` to six fractional digits,
      no exponent (spec §7.2, e.g. `0.020000`).
- [ ] `GET:<name>` for one field returns exactly `get:<name>:<value>`.
- [ ] Every field name matches its v5 `robot_config.proto` name verbatim
      (spec §7.3) — no robot JSON needs editing as a side effect of this
      ticket.
- [ ] Existing binary config arms (`CONFIG`/`GET_CONFIG`/`SET_FIELD`/`CFG`)
      are untouched and their existing tests still pass (additive-only
      ticket).
- [ ] New/changed C++ identifiers follow `.claude/rules/coding-standards.md`
      / `naming-and-style.md`: lowerCamelCase functions/variables, no unit
      suffixes on any identifier (units in `// [unit]` comments); the
      `SET`/`GET` wire verb strings themselves are wire keys and exempt from
      the units rule.

## Testing

- **Existing tests to run**: `src/tests/sim/unit` Comms/Configurator suites,
  current config-parity tests (must still pass — the parity checker is
  untouched until ticket 013).
- **New tests to write**: sim/unit tests covering `SET` live-apply, `SET`
  boot-only store + immediate-`GET`-visibility, NaN-before-range ordering,
  and bare `GET` dumping all 80 fields.
- **Verification command**: `uv run pytest src/tests/sim/unit -k "comms or
  configurator or get_set"`, plus a firmware build/flash smoke check on the
  sim ABI (hardware verification is ticket 016).
