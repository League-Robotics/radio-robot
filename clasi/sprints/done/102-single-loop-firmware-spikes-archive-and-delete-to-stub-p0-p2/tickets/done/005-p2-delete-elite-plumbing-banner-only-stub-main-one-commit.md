---
id: '005'
title: P2 delete Elite plumbing + banner-only stub main (one commit)
status: done
use-cases:
- SUC-005
depends-on:
- '001'
- '003'
- '004'
github-issue: ''
issue: single-loop-firmware-de-fiber-delete-the-elite-plumbing-telemetry-only-return-path.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# P2 delete Elite plumbing + banner-only stub main (one commit)

## Description

Delete the entire discarded Elite orchestration stack and replace
`source/main.cpp` with a ~50-line banner-only stub, landing as exactly one
commit. This is the irreversible step this sprint exists to de-risk and
back up for — it must not run before tickets 001, 003, and 004 are done
(spikes recorded, tag pushed, both hexes archived and reflash-proven),
because this ticket deletes `codal.devicebus.json`, after which the
devicebus-bringup image can only be recovered from ticket 004's archived
hex, not rebuilt. (Ticket 002, a serial baud-ceiling spike, was dropped by
stakeholder decision 2026-07-14 — see sprint.md — so it is not a
dependency.)

**Delete inventory** (corrected for what commit `3c4a8c0a` already removed
on this branch — do not attempt to re-delete `source/motion/`,
`subsystems/nezha_hardware`, `hal/nezha/*`, `hal/otos/*`, or the four
`hal/capability` faceplates; those are already gone):

- `source/main.cpp` (replaced by the stub, not deleted outright)
- `source/runtime/` (blackboard, tick ordering, Configurator)
- `source/subsystems/` (Drivetrain orchestration)
- `source/commands/` (CommandProcessor, command routing, `binary_channel.cpp`)
  — **transcribe the `*B` base64 armor codec and `msg::wire` encode/decode
  out of `binary_channel.cpp` FIRST**, before deleting the file. It is the
  only working framing implementation and sprint 103's `Comms` needs it.
  Save the transcription as a standalone note or code snippet referenced
  from this ticket or handed to sprint 103's planning — do not silently
  lose it.
- `source/drive/` (Ruckig segment/trajectory planner)
- `source/telemetry/` (old frame builder)
- `source/hal/` (remaining capability/sim/velocity_pid content not already
  removed by `3c4a8c0a`)
- `source/com/i2c_bus.{h,cpp}` + `source/com/i2c_bus_host.cpp` (the parked
  duplicate — **the live `source/devices/i2c_bus.{h,cpp}` and
  `devices/{i2c_bus_host,clock_host}.cpp` are KEPT**; see the naming-trap
  note below)
- `source/estimation/` (EkfTiny)
- `source/types/{arg_schema,command_types,clock*,value_set}` (note:
  `types/clock.cpp`/`clock.h`/`clock_host.cpp` under `source/types/` are
  deleted; this is distinct from `source/devices/clock_host.cpp`, which is
  KEPT — confirm the correct path before deleting)
- `source/kinematics/i_kinematics.h`
- `source/devices/bringup_main.cpp`, `source/devices/fiber_runner.h`, plus
  the fiber/staging handoff machinery inside `device_bus.{h,cpp}` and
  `handles.h` (narrow these files, do not delete them — `device_bus`'s bus
  arbitration and `readyAt` safety net survive)
- `codal.devicebus.json`
- `libraries/{ruckig,tinyekf,cmon-pid}` + their CMake include lines
  (`CMakeLists.txt:224-232,377` — line numbers approximate, confirm at
  execution time)
- Dead CMake filters/flags: `BENCH_OTOS_ENABLED`, `PRODUCTION_BUILD`,
  `USE_ORDERED_TICK`, stale exclusion regexes, the `application_entry`
  block (`CMakeLists.txt:408-414` — approximate, confirm at execution time)

**Naming trap to avoid**: `devices/{i2c_bus_host,clock_host}.cpp` are
load-bearing CMake duplicate-symbol exclusion guards for the SURVIVING
`devices/i2c_bus` and `devices/clock` — do not delete them. Only
`com/i2c_bus_host.cpp` (the host shim for the DELETED `com/i2c_bus`
duplicate) is removed. See architecture-update.md Decision 4.

**Test/build/host fallout** (bundled into this ticket, not split out,
because a partial deletion would leave the pytest gate red):
- `tests/_infra/{sim,drive}` deleted (no sim build in phase 1)
- `tests/sim/unit/i2c_bus_clearance_harness.cpp` (`test_i2c_bus_clearance`)
  deleted — it is the sole consumer of the deleted `com/i2c_bus`
- ~35 dead pytest files/harnesses from the old stack deleted
- `pyproject` `testpaths` pruned to match
- justfile `build-sim`/`build-drive` recipes removed
- `check_config_sync` map updated to drop references to deleted config
  surface

**Stub `main.cpp` requirements**: ~50 lines, banner-only. `uBit.init()`,
serial banner announcement, and an idle loop that yields (radio-safe) —
nothing else. No device access, no motor energization, no command
handling. Do NOT create `source/robot/` (build.py:85-90 traps that
directory name as a dead generator trigger) or `source/app/` (that is
sprint 103's job when the real loop lands — see architecture-update.md
Decision 5).

## Acceptance Criteria

- [x] `*B` armor + `msg::wire` encode/decode logic transcribed out of
      `source/commands/binary_channel.cpp` into a standalone note/snippet
      BEFORE that file is deleted, and referenced from this ticket (for
      sprint 103 to consume). — `clasi/sprints/102-.../notes/
      armor-wire-codec-transcription.md`.
- [x] Full delete inventory above removed, correctly excluding what commit
      `3c4a8c0a` already deleted.
- [x] `devices/{i2c_bus_host,clock_host}.cpp` are UNCHANGED (not deleted);
      `com/i2c_bus.{h,cpp}` and `com/i2c_bus_host.cpp` ARE deleted.
- [x] `source/main.cpp` replaced with a banner-only stub (~50 lines); grep
      confirms no motor/device energization call exists anywhere in the
      stub or anything it calls.
- [x] `source/robot/` and `source/app/` do NOT exist after this ticket.
- [x] All of the above lands as exactly one commit. — `72d8be7e`.
- [x] `just build-clean` succeeds and produces a hex.
- [x] The stub is flashed to the bench robot and confirmed to banner
      (boot message visible over serial) — per
      `.claude/rules/hardware-bench-testing.md`, connect-only verification
      since the stub never energizes motors (no wheel/sensor check
      applicable to an inert stub). Then the archived default hex
      (v0.20260714.3) was reflashed and HELLO confirmed, leaving the bench
      robot functional.
- [x] The surviving pytest subset passes (`uv run python -m pytest`). —
      923 passed.
- [x] A repo-wide grep for every deleted header/module name (e.g.
      `runtime/`, `subsystems/`, `drive/`, `telemetry/`, `estimation/`,
      `hal/capability`, `i_kinematics`, `binary_channel`, `bringup_main`,
      `fiber_runner`, `codal.devicebus.json`, `ruckig`, `tinyekf`,
      `cmon-pid`) returns nothing under `source/`, `tests/`, `host/` — swept
      as `#include` paths specifically (comment/docstring prose mentioning a
      deleted path by name for historical/provenance reasons is not a
      dangling reference); the two live doc comments that overstated a
      current dependency (`kinematics/body_kinematics.h`, `messages/
      wire_runtime.h`) were corrected. One pre-existing exception:
      `tests/sim/parked-094/` still `#include`s several deleted paths — it
      was already parked (excluded from pytest collection via
      `norecursedirs`, referencing an even older `source_parked/094/` tree)
      before this ticket and is out of this ticket's delete inventory.
- [x] `codal.json`'s `MICROBIT_RADIO_MAX_PACKET_SIZE=250` setting is
      unchanged (not part of this delete — confirm it wasn't
      inadvertently touched). — confirmed, no diff.

## Implementation Plan

**Approach**: Execute the delete inventory as a single working session
culminating in one commit. Order within the session: (1) transcribe the
armor/wire codec out of `binary_channel.cpp` first; (2) delete
`source/{runtime,subsystems,commands,drive,telemetry,hal}`,
`com/i2c_bus*`, `estimation/`, the dead `types/` files,
`kinematics/i_kinematics.h`, `devices/{bringup_main.cpp,fiber_runner.h}`,
narrow `device_bus.{h,cpp}`/`handles.h` to remove fiber/staging code,
`codal.devicebus.json`, the vendored libs and their CMake lines, and the
dead CMake flags/filters; (3) write the stub `main.cpp`; (4) prune the
test/build/host fallout; (5) run `just build-clean`; fix any dangling
references iteratively until it's clean; (6) run the surviving pytest
subset; (7) flash and confirm banner on the stand; (8) run the repo-wide
grep sweep; (9) commit everything as one commit.

**Files to create/modify**: see the full delete inventory and stub
requirements above — this touches most of `source/`, `CMakeLists.txt`,
`codal.json` (verify unchanged), `pyproject`, `justfile`, and the test
tree. No file-by-file plan beyond the inventory is needed; the acceptance
criteria's grep sweep is the correctness backstop.

**Testing plan**: Existing surviving tests (`devices_*`, `wire_*`, and
whatever remains after the ~35-file prune) must stay green throughout.
`test_i2c_bus_clearance` is intentionally removed alongside `com/i2c_bus`
(expected reduction in test count, not a regression). No new tests are
added by this ticket — it is a deletion ticket; sprint 103 introduces new
tests for the new loop.

**Documentation updates**: none required beyond what's already covered by
`archive/README.md` (ticket 004) and the transcribed armor/wire codec note
this ticket produces for sprint 103.

## Verification (hardware bench gate)

Per `.claude/rules/hardware-bench-testing.md`: robot bench-mounted, wheels
off the ground. This ticket's gate is connect-and-banner only — the stub
never energizes motors, so there is no wheel-drive or encoder check to run
(that returns in sprint 103 once the real loop exists). Confirm device
identity via `mbdeploy list`'s ROLE column before flashing.

## Testing

- **Existing tests to run**: `uv run python -m pytest` — must be green
  after the prune (reduced test count expected; failures are not).
- **New tests to write**: none (deletion ticket).
- **Verification command**: `uv run pytest`

## Completion Notes

- **Commit**: `72d8be7e` (the one deletion commit) on this branch, plus a
  separate ticket-status/frontmatter commit and a version-bump commit per
  project convention.
- **Lines**: `git diff --shortstat 7acafd07..72d8be7e` — 194 files changed,
  294 insertions(+), 42987 deletions(-) (the vendored `libraries/ruckig`
  tree alone accounts for the bulk of the deletion beyond the ~15,900-line
  Elite-stack estimate).
- **`device_bus.{h,cpp}`/`handles.h` narrowing**: `handles.h` needed no
  edit — it never referenced `fiber_runner.h` or the fiber lifecycle at all
  (its "staging" is per-handle passthrough onto leaf fields, unrelated to
  the fiber). `device_bus.{h,cpp}` lost `start()`/`stop()`/`running()`/
  `setFiberRunner()`, the `CodalFiberRunner`/`HostFiberRunner` friend
  declarations, and the fiber lifecycle state fields; `runPreamble()`/
  `neutralizeAllMotors()` moved from private+friend-only to public so a
  future single foreground loop (sprint 103) can call them directly.
  `runCycleOnce()` and the bus-arbitration/cycle-schedule internals
  (`drainStagedInputs`/`serviceMotor`/`perceptionSlotStep`/
  `publishSamples`) are untouched. `device_bus_lifecycle_harness.cpp` +
  `test_device_bus_lifecycle.py` (DB-008's own acceptance harness for the
  now-removed `FiberRunner` injection seam) were deleted rather than
  rewritten — this ticket is deletion-only, no new tests.
- **`check_config_sync.py` — deviation from the written plan**: the ticket
  description's Test/build/host-fallout list says its map should be
  "updated to drop references to deleted config surface." Investigation
  found no such references exist to drop: the script's `PATCH_TO_PYDANTIC`
  map diffs `protos/config.proto`'s generated `Patch` messages against
  `host/robot_radio/config/robot_config.py`'s pydantic model — both kept,
  neither ever pointed at a deleted C++ path (the one `source/commands/
  config_commands.cpp` mention in the script's docstring is historical
  prose about an already-earlier-deleted file, not a live reference). Its
  own test (`tests/unit/test_check_config_sync.py`, 10 tests) passed
  unchanged before and after this ticket. No edit made.
- **`source/config/boot_config.{h,cpp}` — not slimmed**: the linked issue's
  summary mentions "gen_boot_config drops planner emission" as part of the
  sprint's overall keep-list, but ticket 005 itself lists `config/
  boot_config` only under Keep, with no action item, and `defaultPlanner
  Config()` has no dependency on anything deleted (it returns a
  `msg::PlannerConfig`, a kept `messages/` type). Left untouched — planner
  emission slimming, if wanted, is not gated by this ticket's acceptance
  criteria.
- **Surprises**: (1) `tests/_infra/sim`'s gitignored local `build/`/
  `__pycache__/` directories survived `git rm` (only tracked files are
  removed), which made `build.py`'s own `os.path.isdir(_host_sim_dir())`
  self-heal check see a stale local directory and try (and fail) to run
  cmake against it — not a regression, a leftover local build artifact; a
  clean checkout never has this directory. (2) `mbdeploy deploy --hex`
  resolves the hex path relative to the invocation, and the CODAL build
  writes `MICROBIT.hex` to the repo root (`CODAL_APP_OUTPUT_DIR "."` in
  `CMakeLists.txt`), not `build/MICROBIT.hex` — the first flash attempt
  pointed at the wrong path.
