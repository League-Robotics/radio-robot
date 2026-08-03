---
id: '002'
title: Extend gen_messages.py with pydantic + JSON Schema emission
status: open
use-cases:
- SUC-001
- SUC-005
depends-on:
- '001'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Extend gen_messages.py with pydantic + JSON Schema emission

## Description

Extend `src/scripts/gen_messages.py` (currently ~3040 lines, already a
schema-generic field-descriptor walker emitting CRC/COBS-aware C++ wire
messages) with two new emission modes sharing the same field-descriptor
walk: (1) a pydantic `BaseModel` hierarchy covering all 10 groups
(host-only included), (2) a JSON Schema document covering all 10 groups.
The existing C++ wire-message emission is unchanged in mechanism —
critically, its output for `robot_config.proto`'s 7 robot-config groups
now **doubles as `Config::Robot`'s own group storage** (Design Rationale
Decision 1: no separate "boot-config struct" generator — a second
generator producing a competing C++ type would reintroduce the
two-definitions disease this sprint exists to kill). Honor the
`(host_only)` proto option from ticket 001 to skip
`Identity`/`Connection`/`Vision` when emitting the C++ target (they still
get pydantic + JSON Schema).

## Acceptance Criteria

- [ ] Running the extended `gen_messages.py` against `robot_config.proto`
      produces a C++ header with one struct per robot-config group
      (`Geometry`/`Motors`/`Drive`/`WheelControl`/`Planner`/`Otos`/
      `Estimator`), field names/types matching the schema.
- [ ] The same run produces a pydantic `BaseModel` class per group (all
      10, host-only included).
- [ ] The same run produces a JSON Schema document declaring all 10
      groups.
- [ ] Host-only groups (`Identity`/`Connection`/`Vision`) do **not**
      appear in the generated C++ output.
- [ ] Generated C++ output compiles under `HOST_BUILD`.
- [ ] This ticket does not yet wire the generated header into anything
      (`Config::Robot`'s assembly is tickets 005/006) — its own
      acceptance is that the three artifacts generate correctly in
      isolation.

## Testing

- **Existing tests to run**: `gen_messages.py`'s existing coverage for its
  current C++ wire-message emission (`motor.proto`/`drivetrain.proto`/
  `envelope.proto`/etc.) must still pass — confirms no regression to the
  path this ticket extends but does not change.
- **New tests to write**: a generation smoke test running the extended
  generator against `robot_config.proto` and asserting all three output
  artifacts exist and are individually valid (C++ compiles standalone
  under `HOST_BUILD`; the pydantic file imports cleanly; the JSON Schema
  file is valid JSON Schema).
- **Verification command**: `uv run python src/scripts/gen_messages.py
  <args>` followed by a `HOST_BUILD` compile of the new header and
  `python -c "import <generated_module>"`.

## Implementation Plan

**Approach**: Add two new emitter functions inside `gen_messages.py` (or a
small number of functions it calls), reusing the existing
field-descriptor walk/type-mapping helpers rather than re-parsing the
`.proto` a second time. Wire up new CLI flags/output targets analogous to
how the script already selects its C++ output path.

**Files to modify**: `src/scripts/gen_messages.py`.

**Files to create**: generated output files are build artifacts — confirm
against how `boot_config.h`/`robot_config.py` are currently handled
(committed vs. generated-at-build-time) before deciding whether the new
outputs are committed.

**Testing plan**: as above.

**Documentation updates**: `gen_messages.py`'s file header comment gets a
short addendum describing the two new emission modes and why they were
added here rather than in a separate script (Design Rationale Decision 1).
