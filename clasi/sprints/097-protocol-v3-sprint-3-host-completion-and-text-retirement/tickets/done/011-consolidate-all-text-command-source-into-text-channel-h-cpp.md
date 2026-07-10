---
id: '011'
title: Consolidate all text-command source into text_channel.{h,cpp}
status: done
use-cases: []
depends-on: []
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Consolidate all text-command source into text_channel.{h,cpp}

## Description

Pure file consolidation — zero behavior change. Collect all remaining
text-command source into a single file pair, `source/commands/
text_channel.{h,cpp}`, mirroring `binary_channel.{h,cpp}`. The
`source/commands/` directory shrinks from 9 file-pairs down to 4:
`arg_parse`, `binary_channel`, `command_processor`, `text_channel`.
The registered command table stays byte-for-byte identical: STOP, PING,
HELLO (plus the untouched binary channel).

Current inventory of `source/commands/` and its disposition:
- `arg_parse.{h,cpp}` — text arg-parsing infra → **stays** as its own file.
- `binary_channel.{h,cpp}` — binary plane → **stays**, gains `tickTelemetry()`
  + `telemetryEmitBinary()` (see below).
- `command_processor.{h,cpp}` — dispatch/reply infra → **stays**.
- `motion_commands.{h,cpp}` — STOP only (live text safety rump) → **move**
  into `text_channel`.
- `system_commands.{h,cpp}` — PING, HELLO (live rump), plus
  `formatDeviceAnnouncement()`/`deviceIdentity()` (external-linkage helpers
  used outside this file) → **move** into `text_channel`.
- `telemetry_commands.{h,cpp}` — **split**: the empty `telemetryCommands()`
  registrar is dead (registers zero commands since 097-008) and is dropped
  outright, along with its call in `command_router.cpp`; but `tickTelemetry()`
  + `telemetryEmitBinary()` are LIVE binary-plane periodic-emission code
  (called every pass by `main.cpp` and `sim_api.cpp`) — these **move to
  `binary_channel.{h,cpp}`**, NOT `text_channel` (they are binary-plane
  infrastructure, not text-command source, and deleting them outright would
  break the build). After this split, `telemetry_commands.{h,cpp}` is
  deleted — nothing remains in it.
- `pose_commands.{h,cpp}` — SI/ZERO; unregistered dead source kept as
  sprint-098's transcription reference for the binary `pose` envelope arm →
  **move verbatim** into `text_channel` (preserve all doc comments — they are
  the 098 reference).
- `otos_commands.{h,cpp}` — OI/OZ/OR/OP/OV/OL/OA; unregistered dead source,
  098 reference for the binary `otos` arm → **move verbatim**.
- `dev_commands.{h,cpp}` — DEV M/DT/STATE/STOP/WD, `ROBOT_DEV_BUILD`-gated,
  unregistered → **move verbatim**.

## Acceptance Criteria

- [x] `source/commands/text_channel.h` declares: the live-rump table builder
      (lowerCamelCase, e.g. `textCommands(Rt::CommandRouter&)`, per
      `.claude/rules/naming-and-style.md`) returning the STOP+PING+HELLO
      descriptors; `formatDeviceAnnouncement()`/`deviceIdentity()` with
      external linkage (unchanged signatures — `binary_channel.cpp` and
      `communicator.cpp` call them); and the existing unregistered builders
      `poseCommands()`/`otosCommands()`/`devCommands()` kept with external
      linkage exactly as today — declared but never called — so no
      `-Wunused` fallout and the registered-vs-dead distinction is preserved
      unchanged.
- [x] `text_channel.h`'s file-level header comment explains the layout: live
      rump section (STOP/PING/HELLO + identity helpers) vs.
      dead-source-kept-as-098-transcription-reference sections (pose, otos)
      vs. the DEV bench-diagnostic section — each clearly labeled.
- [x] `source/commands/binary_channel.h`/`.cpp` gain `tickTelemetry()` +
      file-local `telemetryEmitBinary()`, relocated verbatim from
      `telemetry_commands.{h,cpp}`, at global namespace scope (not nested
      under `namespace BinaryChannel`) so `main.cpp`/`sim_api.cpp` call
      sites are unchanged syntactically.
- [x] `source/runtime/command_router.cpp`'s `buildTable()` calls the single
      live-rump builder (`textCommands(router)`) instead of
      `systemCommands()` + `motionCommands()` + `telemetryCommands()`.
      Resulting registered table is identical: STOP, PING, HELLO (plus the
      binary channel, untouched). The `#include "commands/telemetry_commands.h"`
      line is dropped (no replacement needed — `buildTable()` no longer
      touches telemetry).
- [x] `#include` fixups at every actual (non-comment) includer of a moved/
      deleted header:
      - `source/subsystems/communicator.cpp`:
        `"commands/system_commands.h"` → `"commands/text_channel.h"`
        (uses `formatDeviceAnnouncement()`).
      - `source/commands/binary_channel.cpp`:
        `"commands/system_commands.h"` → `"commands/text_channel.h"`
        (uses `deviceIdentity()`).
      - `source/main.cpp`: `"commands/telemetry_commands.h"` →
        `"commands/binary_channel.h"` (uses `tickTelemetry()`).
      - `tests/_infra/sim/sim_api.cpp`: same swap as `main.cpp`.
      - `source/runtime/command_router.cpp`: drop
        `#include "commands/system_commands.h"` and
        `#include "commands/motion_commands.h"` and
        `#include "commands/telemetry_commands.h"`; add
        `#include "commands/text_channel.h"`.
- [x] Delete `motion_commands.{h,cpp}`, `system_commands.{h,cpp}`,
      `telemetry_commands.{h,cpp}`, `pose_commands.{h,cpp}`,
      `otos_commands.{h,cpp}`, `dev_commands.{h,cpp}`.
- [x] `tests/_infra/sim/CMakeLists.txt` updated: remove the 6 explicit
      `.cpp` lines for the deleted files (~lines 150–155), add one
      `"${SOURCE_DIR}/commands/text_channel.cpp"` line;
      `binary_channel.cpp` stays listed as-is. Update the two header-comment
      blocks that enumerate these files by name (top-of-file
      `ROBOT_DEV_BUILD` rationale comment; the "Present" file-by-file
      dependency list) to match the new layout. The ARM firmware's root
      `CMakeLists.txt` needs no edit — it globs `source/**/*.cpp`
      (`RECURSIVE_FIND_FILE`).
- [x] Grep-clean: no `#include` or symbol reference to any deleted header
      remains anywhere in `source/`, `tests/`, or scripts.
- [x] `docs/protocol-v3.md` updated at every stale file-path reference: the
      rump table (STOP/PING/HELLO source column), the `segment` source-
      constant citation, the `systemCommands()`/`motionCommands()`/
      `telemetryCommands()` builder citations (file:line), the STOP handler
      citation, and the pose/otos/dev "dead source reference" section —
      re-point text-family citations to `text_channel.{h,cpp}` and the
      `tickTelemetry()`/binary-telemetry citation to `binary_channel.{h,cpp}`.
      Sweep for any other doc under `docs/` pointing at the old filenames.
- [x] Clean build of both targets via `just build-clean`.
- [x] Unit/sim tests pass (`uv run python -m pytest`, plus the CMake sim
      build/tests).
- [x] Registered text verb set provably unchanged: STOP/PING/HELLO only
      (e.g. via the existing `tests/sim/unit/test_bare_loop_commands.py`
      coverage, updated only for file-name mentions in its own comments, not
      behavior).
- [x] Hardware bench smoke on the stand per
      `.claude/rules/hardware-bench-testing.md`: PING/HELLO round-trip and
      STOP over the real serial link.
- [x] Grep-clean confirmed (see above): zero `#include`/symbol references
      to any deleted header remain in any live (built or pytest-collected)
      file -- proven both by grep and by `just build-clean` succeeding
      clean for both targets plus the full `tests/sim`/`tests/unit`
      suites passing. A literal `grep -rn` for the six bare filenames
      across `source/ tests/ scripts/ docs/` is NOT zero (487 hits): the
      remainder are (a) deliberate, clearly-labeled historical citations
      this ticket's own files add ("formerly X", "git history: former
      X", or verbatim-quoted donor-file header comments explicitly
      preserved per this ticket's own "do not summarize or trim"
      instruction) in `text_channel.{h,cpp}`, `binary_channel.cpp`,
      `sim_api.cpp`, `docs/protocol-v3.md`, `docs/protocol-v2.md`, and
      `test_bare_loop_commands.py`; (b) pre-existing citations in files
      OUTSIDE this ticket's own "Files to modify" list (`blackboard.h`,
      `commands.h`, `configurator.h`, `command_router.h`,
      `hal/capability/*`, `hal/otos/otos_odometer.h`, `types/clock.h`,
      `motion/segment_executor.h`, `subsystems/{hardware,pose_estimator}.h`,
      `telemetry/tlm_frame.{h,cpp}`) that this ticket's own Implementation
      Plan does not list as touched; and (c) historical
      `docs/architecture/architecture-update-0NN.md` sprint snapshots and
      the already-parked, non-collected `tests/sim/parked-093/094/` +
      unrelated `tests/testgui/`/`tests/bench/`/`tests/unit/test_legacy_*`
      files. (b) is flagged for a follow-up sweep (ticket 010's own
      re-run, per this ticket's Sequencing note) rather than expanded
      into here, since editing them is outside this ticket's declared
      file scope.

## Sequencing note

This ticket **must complete before ticket 010** (migration closure: grep-
clean verification, line-count and flash/RAM report) finalizes — 010 is
in-progress; its counts/report must be re-run after this consolidation
lands, since 010's grep-clean and line-count figures will otherwise be
stale the moment this ticket's file moves land.

## Implementation Plan

- **Approach**: Mechanical move, not a rewrite. Cut-paste each function
  body verbatim into `text_channel.cpp`/`.h` (or `binary_channel.cpp`/`.h`
  for the tickTelemetry pair), preserving all existing doc comments
  (especially the pose/otos 098-reference commentary — do not summarize or
  trim it). Concatenate header file-header comments into one comment
  explaining `text_channel.h`'s three-section layout. Do not touch handler
  logic, descriptor tables, or any wire behavior.
- **Files to create**: `source/commands/text_channel.h`,
  `source/commands/text_channel.cpp`.
- **Files to modify**: `source/commands/binary_channel.h`,
  `source/commands/binary_channel.cpp`, `source/runtime/command_router.cpp`,
  `source/subsystems/communicator.cpp`, `source/main.cpp`,
  `tests/_infra/sim/sim_api.cpp`, `tests/_infra/sim/CMakeLists.txt`,
  `docs/protocol-v3.md`.
- **Files to delete**: `source/commands/motion_commands.{h,cpp}`,
  `source/commands/system_commands.{h,cpp}`,
  `source/commands/telemetry_commands.{h,cpp}`,
  `source/commands/pose_commands.{h,cpp}`,
  `source/commands/otos_commands.{h,cpp}`,
  `source/commands/dev_commands.{h,cpp}`.
- **Testing plan**: `just build-clean` (both ARM + host/sim targets);
  `uv run python -m pytest` (full suite, esp.
  `tests/sim/unit/test_bare_loop_commands.py`); repo-wide grep for the six
  deleted basenames; hardware bench smoke (PING/HELLO/STOP over serial, per
  hardware-bench-testing.md).
- **Documentation updates**: `docs/protocol-v3.md` file-path citations (see
  above); ticket 010 must be told to re-run its grep/line-count/flash-RAM
  report after this ticket lands.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full suite,
  especially `tests/sim/unit/test_bare_loop_commands.py`); the sim CMake
  build (`tests/_infra/sim`).
- **New tests to write**: none — pure refactor, no new behavior; existing
  coverage of STOP/PING/HELLO and of `tickTelemetry()`'s periodic emission
  must continue to pass unchanged.
- **Verification command**: `just build-clean && uv run python -m pytest`

## Hardware bench result (team-lead, 2026-07-10, robot on stand)

Flashed the consolidated pure-binary firmware (v0.20260710.5, 472 KB) to the robot (tovez, /dev/cu.usbmodem2121102) and smoke-tested over direct serial:
- text rump: `PING`→`OK pong t=`, `HELLO`→`DEVICE:NEZHA2:robot:tovez:...`, `STOP`→`OK stop` (bare-terminal safety affordance confirmed working).
- gutted verbs correctly rejected: `VER`/`SET`/`STREAM`/`S` all → `ERR unknown`.
- binary PING→`ok{t}`, binary ID→full DeviceId (fw 0.20260710.5, proto 2).
- binary DRIVE (250,250) fed continuously → motors spin: binary Telemetry stream (30 frames, monotonic seq) shows enc_left/right climbing (257/221), vel_left/right ≈244/237 ≈ commanded 250 mm/s.
- binary STREAM (StreamControl binary period=50) + binary STOP work.
Full pure-binary command plane + rump validated on real hardware.
