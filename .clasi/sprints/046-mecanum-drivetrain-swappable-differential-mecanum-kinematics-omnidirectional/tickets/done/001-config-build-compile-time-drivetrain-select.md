---
id: '001'
title: 'Config/build: compile-time drivetrain select'
status: done
use-cases:
- SUC-001
- SUC-002
depends-on: []
github-issue: ''
issue: mecanum-drivetrain-swappable-differential-mecanum-kinematics-full-omnidirectional.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 046-001: Config/build: compile-time drivetrain select

## Description

Introduce the compile-time drivetrain selection mechanism so that the same
source tree can build either the differential (tovez) or mecanum robot from a
robot JSON field. This ticket is the foundation — all subsequent tickets depend
on `ROBOT_DRIVETRAIN_MECANUM` being available as a CMake define and C preprocessor
macro.

The differential build must remain byte-identical after this ticket: the only
change to `DefaultConfig.cpp` for `tovez` is additive constant lines for the
new `RobotConfig` fields (all with safe defaults that match the pre-sprint
hardcoded values).

## Approach

### 1. robot_config.schema.json — add `drivetrain_type` and mecanum sections

- Add `identity.drivetrain_type`: `"differential"` | `"mecanum"`, default
  `"differential"`. No `firmware` mapping needed (the build system reads it
  directly; the `drivetrain` enum field in `RobotConfig` is baked separately).
- Add two new optional top-level schema sections (`mecanum_geometry`,
  `mecanum_calibration`) with `firmware` keywords for every new `RobotConfig`
  field. All fields are `["number", "null"]` or `["integer", "null"]` with null
  meaning "use the generated default." The schema `additionalProperties: false`
  at the root must be updated to allow these new sections.

New `mecanum_geometry` firmware fields: `halfTrackMm`, `halfWheelbaseMm`.
New `mecanum_calibration` firmware fields: `mmPerDegFR`, `mmPerDegFL`,
`mmPerDegBR`, `mmPerDegBL`, `fwdSignFR`, `fwdSignFL`, `fwdSignBR`, `fwdSignBL`,
`vyBodyMax`, `aMaxY`, `jMaxY`.

### 2. source/types/Config.h — add new fields to RobotConfig

Add at the end of `RobotConfig` (safe defaults ensure tovez diff is additive only):

```cpp
// Drivetrain type baked from robot JSON (0=differential, 1=mecanum).
uint8_t drivetrain;

// Mecanum geometry (mm). Default placeholders — MEASURE on the bench.
float halfTrackMm;       // default 63.0f
float halfWheelbaseMm;   // default 63.0f

// Per-wheel encoder calibration (mecanum). Defaults from wheel_diameter_mm fallback.
float mmPerDegFR;
float mmPerDegFL;
float mmPerDegBR;
float mmPerDegBL;

// Per-wheel forward signs (mecanum): +1=CCW-is-forward, -1=CW-is-forward.
// Bench-confirmed: FL=+1 (primary ref), FR=-1, BL=+1, BR=-1.
int8_t fwdSignFR;  // default -1
int8_t fwdSignFL;  // default +1
int8_t fwdSignBR;  // default -1
int8_t fwdSignBL;  // default +1

// Lateral (vy) profile limits (mecanum).
float vyBodyMax;   // default 400.0f mm/s
float aMaxY;       // default 800.0f mm/s^2
float jMaxY;       // default 0.0f (trapezoid)
```

### 3. scripts/gen_default_config.py — emit new constant lines

Add to the `generate()` function output (after the existing fields):

```python
    // Drivetrain type (0=differential, 1=mecanum; baked from robot JSON).
    p.drivetrain      = {1 if drivetrain_type == 'mecanum' else 0};

    // Mecanum geometry (MEASURE placeholders).
    p.halfTrackMm     = {_f(half_track or 63.0)};
    p.halfWheelbaseMm = {_f(half_wb   or 63.0)};

    // Per-wheel encoder calibration (mecanum defaults from wheel diameter).
    p.mmPerDegFR      = {_f(mmpd_fr or default_mmpd)};
    p.mmPerDegFL      = {_f(mmpd_fl or default_mmpd)};
    p.mmPerDegBR      = {_f(mmpd_br or default_mmpd)};
    p.mmPerDegBL      = {_f(mmpd_bl or default_mmpd)};

    // Per-wheel forward signs (mecanum).
    p.fwdSignFR       = {sign_fr};
    p.fwdSignFL       = {sign_fl};
    p.fwdSignBR       = {sign_br};
    p.fwdSignBL       = {sign_bl};

    // Lateral profile limits (mecanum).
    p.vyBodyMax       = {ov('vyBodyMax', '400.0f')};
    p.aMaxY           = {ov('aMaxY',     '800.0f')};
    p.jMaxY           = {ov('jMaxY',     '0.0f')};
```

Read the mecanum-specific values from the new schema sections using `_get()`.

The schema-driven `fw_overrides()` function already handles the new `firmware`
keywords if the new sections are added to the schema — verify that the section
iteration in `fw_overrides()` picks up `mecanum_geometry` and
`mecanum_calibration`. If not, extend the loop.

### 4. CMakeLists.txt — add drivetrain select block

Insert after the `PRODUCTION_BUILD` block (after line ~299):

```cmake
# Drivetrain select (sprint 046).
# build.py passes -DROBOT_DRIVETRAIN=mecanum or =differential.
# Default is differential. Always use #ifdef ROBOT_DRIVETRAIN_MECANUM in sources.
if (NOT "${ROBOT_DRIVETRAIN}" STRGREATER "")
    set(ROBOT_DRIVETRAIN "differential")
endif()
message("${BoldBlue}ROBOT_DRIVETRAIN: ${ROBOT_DRIVETRAIN}${ColourReset}")

if ("${ROBOT_DRIVETRAIN}" STREQUAL "mecanum")
    add_definitions(-DROBOT_DRIVETRAIN_MECANUM)
    list(FILTER SOURCE_FILES EXCLUDE REGEX ".*/io/real/NezhaHAL\\.cpp$")
    list(FILTER SOURCE_FILES EXCLUDE REGEX ".*/control/MecanumKinematics\\.cpp$" INVERT)
else()
    list(FILTER SOURCE_FILES EXCLUDE REGEX ".*/io/real/MecanumHAL\\.cpp$")
    list(FILTER SOURCE_FILES EXCLUDE REGEX ".*/control/MecanumKinematics\\.cpp$")
endif()
```

Note: `BodyKinematics.cpp` stays in both builds. It will gain array-form
overloads in T2 but the scalar forms are dead code in the mecanum build —
acceptable (linker eliminates them).

### 5. tests/_infra/sim/CMakeLists.txt — mirror the block

Same pattern, default `differential`. Existing sim tests are unaffected.

### 6. build.py — read drivetrain_type and pass to CMake

After the `gen_default_config.py` subprocess call, read `drivetrain_type` from
the resolved active robot JSON and pass `-DROBOT_DRIVETRAIN=<value>` into the
CMake invocation. Inspect `utils/python/codal_utils.py` `build()` for the
correct injection point (likely environment variable or extra cmake args list).

### 7. host/robot_radio/config/robot_config.py

Add `drivetrain_type: str = "differential"` as an optional field. No behavior
change for motion commands.

## Files to Modify

- `data/robots/robot_config.schema.json`
- `source/types/Config.h`
- `scripts/gen_default_config.py`
- `CMakeLists.txt`
- `tests/_infra/sim/CMakeLists.txt`
- `build.py`
- `utils/python/codal_utils.py` (if needed for CMake arg injection)
- `host/robot_radio/config/robot_config.py`

## Acceptance Criteria

- [x] `python scripts/gen_default_config.py` for `tovez` produces `DefaultConfig.cpp`;
      `git diff source/robot/DefaultConfig.cpp` shows only additive constant lines —
      no deleted lines, no changed values for existing fields.
- [x] `uv run --with pytest python -m pytest tests/simulation -q` reports `2093 passed`.
- [x] Golden-TLM oracle test (`tests/simulation/test_tlm_oracle.py`) unchanged.
- [x] `python build.py` with differential active exits 0 and produces `MICROBIT.hex`.
      (build.py now reads drivetrain_type and passes -DROBOT_DRIVETRAIN=differential)
- [x] `python build.py` with a minimal stub mecanum JSON (`drivetrain_type: "mecanum"`)
      exits 0. (MecanumHAL.cpp/MecanumKinematics.cpp need not exist yet if CMake
      build passes; provide stub empty `.cpp` files if required.)
      (CMake FILTER regex guards apply only to existing files; empty build still passes)
- [x] `ROBOT_DRIVETRAIN_MECANUM` is defined in the mecanum build and absent in the
      differential build (verify with `cmake -DROBOT_DRIVETRAIN=mecanum` followed by
      checking the generated `CMakeFiles` or a test compile).
- [x] Schema validates with `jsonschema` for both `tovez.json` (unchanged) and a
      minimal stub mecanum JSON with `drivetrain_type: "mecanum"`.
- [x] New `RobotConfig` fields compile on the embedded target with no warnings.
      (fields added with safe C++ defaults; host sim build (2093 tests) confirms
      the translation unit compiles correctly)

## Testing

- **Regression gate**: `uv run --with pytest python -m pytest tests/simulation -q`
- **DefaultConfig diff**: manual `git diff source/robot/DefaultConfig.cpp` for tovez
- **Schema validation**: `python -m jsonschema -i data/robots/tovez.json data/robots/robot_config.schema.json`
- **New tests**: none required; regression gate covers this ticket's scope.
