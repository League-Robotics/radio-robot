# Utils (`src/utils`)

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-21 · **Status:** stable

---

## 1. Purpose

`src/utils/` is build- and flash-tooling support: CMake helper modules,
the DAPLink/UF2 flashing scripts the deploy tool wraps, and a couple of
debug-console snippets. It has no runtime role in the firmware or the
host package — nothing here compiles into `firmware_host` or ships to
the robot. It is kept as its own directory rather than scattered into
`src/firm/` or repo-root scripts because it is infrastructure the CMake
build and `mbdeploy` (see
[`.claude/rules/hardware-bench-testing.md`](../../.claude/rules/hardware-bench-testing.md))
depend on, not project source code with its own architecture worth
documenting at length.

## 2. Orientation

- **`cmake/`** — `util.cmake`, `JSONParser.cmake`, `colours.cmake`: CMake
  helper modules included by the top-level build (JSON parsing for
  `codal.json`-style config, colored build-log output, misc utility
  macros).
- **`python/codal_utils.py`** — small Python helpers the CODAL-derived
  build tooling calls into.
- **`uf2conv.py`, `esptool.py`, `merge_hex.py`, `generate_libraries.py`,
  `targets.json`** — flashing/packaging scripts (UF2 conversion, hex
  merging) and the CODAL target-list data `generate_libraries.py`
  consumes when fetching vendor libraries into `src/libraries/` (see
  [`../libraries/DESIGN.md`](../libraries/DESIGN.md)).
- **`debug/dmesg.js`, `debug/meminfo.js`** — small scripts for a
  DAPLink/CMSIS-DAP debug console session, unrelated to the Python/CMake
  tooling above.

## 3. Constraints and Invariants

- **Nothing here is project source.** This directory has no dependency
  on `src/firm/`, `src/sim/`, or `src/host/`, and nothing in those trees
  depends on it at runtime — only the build/deploy tooling (CMake
  configure step, `mbdeploy`) invokes these scripts. Do not add
  project-behavioral code here; it belongs in `src/firm/` or
  `src/host/robot_radio/`.
- **Vendored/third-party scripts (`uf2conv.py`, `esptool.py`) are
  imported wholesale, not house-styled.** They come from their upstream
  projects (Adafruit's UF2 tooling, Espressif's `esptool`) and are
  excluded from this project's own naming/style conventions — bringing
  them into house style on next edit would make future upstream syncs a
  manual diff exercise instead of a drop-in replace.

## 4. Design

No further structure — this is a flat toolbox, not a layered subsystem.
Each script is independently invoked by the build or deploy pipeline; none
call into each other beyond `generate_libraries.py` reading
`targets.json`.

## 5. Interfaces

### Exposes

- **CMake helper modules** (`cmake/*.cmake`) — included by the top-level
  `CMakeLists.txt` build.
- **`generate_libraries.py`** — fetches/updates the vendored CODAL
  libraries into `src/libraries/` (gitignored — see
  [`../libraries/DESIGN.md`](../libraries/DESIGN.md)).
- **`uf2conv.py`, `merge_hex.py`** — flashing-format conversion, called
  by the deploy pipeline (`mbdeploy`) and/or `just` recipes.

### Consumes

- **`src/libraries/`'s `targets.json`-described vendor set** — the data
  `generate_libraries.py` fetches against.

## 6. Open Questions / Known Limitations

None known — this directory is stable, low-churn build infrastructure.
