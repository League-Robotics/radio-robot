---
id: '002'
title: 'Wire-protocol adapter: rename unit-suffixed parameters in NezhaProtocol/TLMFrame
  parsing'
status: open
use-cases:
- SUC-001
depends-on:
- '001'
github-issue: ''
issue: remove-units-from-identifier-names-host-python.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Wire-protocol adapter: rename unit-suffixed parameters in NezhaProtocol/TLMFrame parsing

## Description

`host/robot_radio/robot/protocol.py` (1082 lines) is the sole owner of
`NezhaProtocol`, `TLMFrame`, `ParsedResponse`, `Stop`, and the module-level
`parse_*` functions. It depends only on `io/serial_conn.py` (ticket 001,
already renamed) and holds the single largest concentration of
unit-suffixed parameter names in this sprint (125 occurrences) — including
the re-exposed `read_ms` parameter this ticket must rename to `read_timeout`
(ticket 001's decided name, per Decision 2).

Renames (per `architecture-update.md` Step 5):
- `NezhaProtocol`'s `read_ms` parameter, re-exposed on `send`, `ping`,
  `echo`, `get_id`, `get_ver`, `get_help`, `get_config`, `set_config`,
  `timed`, `distance`, `go_to`, `turn`, `grip`, and the OTOS/J-port helpers
  → `read_timeout`.
- `arc`/`vw`/`drive`/`timed`/`distance`/`go_to`/`turn`'s `speed_mms`/
  `radius_mm`/`v_mms`/`omega_mrads`/`left_mms`/`right_mms`/`x_mm`/`y_mm`/
  `heading_cdeg`/`eps_cdeg` → each renamed to its bare quantity name with a
  `# [unit]` comment (e.g. `speed_mms` → `speed  # [mm/s]`).
- `stream`/`stream_drive`'s `period_ms`/`watchdog_ms`/`duration_ms` →
  renamed the same way.
- `Stop`'s classmethod parameters (`ms`, `mm`, `cdeg`, `eps_cdeg`, `arc_mm`)
  → renamed with `# [unit]` tags (name the *kind* of quantity, per
  `docs/coding-standards.md`'s ambiguity-resolution rule, since these are
  already bare unit abbreviations rather than `name_unit` compounds).

**What must NOT change**: `TLMFrame`'s dataclass fields (`enc`, `pose`,
`vel`, `twist`, `otos`, `line`, `color`, `ekf_rej`, `wedge`, `encpose`,
`otos_health`, `t`, `mode`, `seq`) are already unit-free — confirmed by
direct read this planning pass — and are **not touched**. `parse_tlm`'s
`kv` dict-key lookup strings (the same literal tokens) are wire tokens and
stay byte-identical. Every wire command is built **positionally** via
f-strings (e.g. `f"R {speed_mms} {radius_mm}"`) — renaming the Python
variable never changes the byte sequence sent, only the source-level name
feeding the format string.

## Hard Contract (applies to this and every sprint 076 ticket)

- **Pure rename — no behavioral change.** Wire command formatting and
  telemetry parsing produce byte-identical output before and after.
- **Every renamed declaration carries a `# [unit]` comment.**
- **Wire keys/tokens are STABLE**: `TLMFrame` field names, `parse_tlm`/
  `parse_response`/`parse_cfg`'s `kv`-key lookup strings, and any
  `SET`/`GET` key string built in this file are byte-identical to pre-076
  (diff-verify).
- **Full suite green throughout**: `uv run python -m pytest -q` remains
  **2682 passed, 0 failed**.
- **`read_ms` → `read_timeout` convergence**: this file re-exposes ticket
  001's decided name — use exactly `read_timeout`, not a variant.
- **Ignore environmental `data/robots` drift.**

## Acceptance Criteria

- [ ] Every `NezhaProtocol` method listed above (`send`, `ping`, `echo`,
      `get_id`, `get_ver`, `get_help`, `get_config`, `set_config`, `timed`,
      `distance`, `go_to`, `turn`, `grip`, OTOS/J-port helpers) renames
      `read_ms` → `read_timeout`, each carrying `# [ms]`.
- [ ] `arc`/`vw`/`drive`/`timed`/`distance`/`go_to`/`turn`'s `speed_mms`/
      `radius_mm`/`v_mms`/`omega_mrads`/`left_mms`/`right_mms`/`x_mm`/
      `y_mm`/`heading_cdeg`/`eps_cdeg` are renamed to their bare quantity
      name with a `# [unit]` comment, reusing whatever unit vocabulary the
      surrounding file/docstring already uses (`mm`, `mm/s`, `deg`,
      `cdeg`/centidegree spelling as currently used in file, etc.).
- [ ] `stream`/`stream_drive`'s `period_ms`/`watchdog_ms`/`duration_ms` are
      renamed to bare + `# [ms]`.
- [ ] `Stop`'s classmethod parameters (`ms`, `mm`, `cdeg`, `eps_cdeg`,
      `arc_mm`) are renamed to descriptive names with `# [unit]` tags.
- [ ] `TLMFrame`'s dataclass field names are byte-identical to pre-076 (git
      diff shows zero changes to the dataclass definition).
- [ ] `parse_tlm`'s, `parse_response`'s, and `parse_cfg`'s `kv` dict-key
      lookup string literals (`"t"`, `"mode"`, `"seq"`, `"enc"`, `"pose"`,
      `"encpose"`, `"vel"`, `"twist"`, `"otos"`, `"line"`, `"color"`,
      `"ekf_rej"`, `"otos_health"`, `"wedge"`) are byte-identical to
      pre-076.
- [ ] Every wire-command f-string builder (e.g. `f"R {speed} {radius}"`
      after rename) is confirmed to still emit the identical byte sequence
      as before, for the same input values — spot-check by running a
      protocol round-trip test or comparing formatted strings pre/post.
- [ ] Every renamed-parameter keyword call site **inside
      `robot/protocol.py` itself** is updated to the new name in this same
      ticket.
- [ ] Hard Contract above holds.

## Testing

- **Existing tests to run**: any `tests/simulation/unit/` test exercising
  `NezhaProtocol`, `parse_tlm`, `parse_response`, `parse_cfg`, or `Stop`
  (grep for `NezhaProtocol`/`parse_tlm`/`TLMFrame` imports to enumerate the
  exact files — this planning pass did not enumerate them individually).
- **New tests to write**: none required — pure rename.
- **Verification command**: `uv run python -m pytest -q` (confirm 2682
  passed, 0 failed).

## Implementation Plan

**Approach**: Work through `robot/protocol.py` method-by-method, renaming
parameters and adding `# [unit]` comments, leaving `TLMFrame` and every
`kv`-key string literal untouched.

1. Rename `read_ms` → `read_timeout` on every `NezhaProtocol` method listed
   above; add `# [ms]`.
2. Rename `arc`/`vw`/`drive`/`timed`/`distance`/`go_to`/`turn`'s
   unit-suffixed parameters to bare quantity names with `# [unit]`
   comments, preserving parameter order (positional wire formatting must
   not shift).
3. Rename `stream`/`stream_drive`'s `period_ms`/`watchdog_ms`/`duration_ms`.
4. Rename `Stop`'s classmethod parameters, choosing descriptive names per
   the ambiguity-resolution rule in `docs/coding-standards.md`.
5. Read `TLMFrame`'s dataclass definition and every `kv`-key lookup in
   `parse_tlm`/`parse_response`/`parse_cfg`; confirm zero changes are made
   there (this is a verification step, not an edit step).
6. Grep this file for every renamed parameter's old name to confirm no
   internal call site was missed.
7. Run protocol-related unit tests, then the full suite.

**Files to create/modify**:
- `host/robot_radio/robot/protocol.py` — modified (parameter renames +
  `# [unit]` comments only; `TLMFrame` and `kv`-key parsing untouched).

**Testing plan**: Run protocol-related tests under `tests/simulation/unit/`
individually, then `uv run python -m pytest -q` and confirm the 2682
baseline holds.

**Documentation updates**: None in this ticket.
