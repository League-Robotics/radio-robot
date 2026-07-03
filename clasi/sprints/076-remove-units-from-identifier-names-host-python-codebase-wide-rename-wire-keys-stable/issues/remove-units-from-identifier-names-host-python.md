---
status: in-progress
sprint: '076'
tickets:
- 076-001
- 076-002
- 076-003
- 076-004
- 076-005
- 076-006
- 076-007
- 076-008
- 076-009
- 076-010
- 076-011
---

# Remove units from identifier names — host Python half (split from sprint 071)

## Description

The parent issue `remove-units-from-identifier-names.md` (removing unit suffixes
from identifier names, units moved to a standard leading `# [unit]` line comment)
was split at sprint 071: the **firmware/sim C++** half landed in sprint 071
(8 tickets, `source/` clean of unit-suffixed identifier names except documented
wire-key/vendor exclusions). This issue is the **host Python** half, which sprint
071's planner measured as the LARGER of the two (~280+ unique identifiers:
~101 `_mm`, ~78 `_deg`, ~77 `_ms`, ~24 `_mms`, plus `_dps`/`_pct`/`_hz`;
`read_ms` alone has ~269 call sites) across `host/robot_radio/`, its tests, and
tools.

## Scope

Rename unit-suffixed identifiers (`_mm`, `_mms`, `_deg`, `_dps`, `_ms`, `_us`,
`_pct`, `_hz` trailing snake_case components) in:
- `host/robot_radio/` (the importable package: io, robot, testgui, calibration,
  config, media, etc.)
- `tests/` Python that references them
- host-side tools/scripts

Units move to a leading `# [unit]` comment per `docs/coding-standards.md`
(established in sprint 071-001).

## Convention & exclusions (established in sprint 071)

- Convention doc: `docs/coding-standards.md` — leading bracketed unit tag
  (`# [ms] description`), dimensionless fields untagged, compound/derived units,
  ambiguity-resolution rule.
- **Wire/serialized names STAY STABLE** (the sprint-071 exclusion table): SET/GET
  and SIMSET/SIMGET key strings, TLM/SNAP field tokens (`enc=`, `encpose=`,
  `otos=`, `pose=`, `otos_health=`, …), and `data/robots/*.json` +
  `robot_config.py` pydantic attribute names (the attribute name IS the JSON key —
  no aliases). The C ABI (`extern "C"`) names are already unit-free and
  positional. Rename only the internal Python identifiers, not any wire/serialized
  string.
- `parse_tlm`/`TLMFrame` field ATTRIBUTE names are internal Python — renamable —
  but their PARSING of the wire tokens (`otos=`, `encpose=`, `otos_health=`, …)
  must keep matching the firmware-emitted token spelling.

## Acceptance criteria

- No identifier in `host/robot_radio/` (and its tests/tools) embeds a unit
  suffix (`_mm`/`_mms`/`_deg`/`_dps`/`_ms`/`_us`/`_pct`/`_hz` trailing), except
  documented wire/serialized/ABI exclusions.
- Every renamed declaration carries the `# [unit]` comment.
- Pure rename — no behavioral change; the full test suite passes and the TestGUI
  + rogo CLI + bench tools still work.
- Wire compatibility preserved (SET/SIMSET keys, TLM tokens, JSON config keys
  byte-identical; a live SET/GET and a TLM parse still round-trip).
