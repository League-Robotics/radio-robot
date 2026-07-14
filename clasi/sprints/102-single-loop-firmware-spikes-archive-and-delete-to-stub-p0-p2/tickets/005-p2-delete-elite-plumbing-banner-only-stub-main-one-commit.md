---
id: "005"
title: "P2 delete Elite plumbing + banner-only stub main (one commit)"
status: open
use-cases: ["SUC-005"]
depends-on: ["001", "003", "004"]
github-issue: ""
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

- [ ] `*B` armor + `msg::wire` encode/decode logic transcribed out of
      `source/commands/binary_channel.cpp` into a standalone note/snippet
      BEFORE that file is deleted, and referenced from this ticket (for
      sprint 103 to consume).
- [ ] Full delete inventory above removed, correctly excluding what commit
      `3c4a8c0a` already deleted.
- [ ] `devices/{i2c_bus_host,clock_host}.cpp` are UNCHANGED (not deleted);
      `com/i2c_bus.{h,cpp}` and `com/i2c_bus_host.cpp` ARE deleted.
- [ ] `source/main.cpp` replaced with a banner-only stub (~50 lines); grep
      confirms no motor/device energization call exists anywhere in the
      stub or anything it calls.
- [ ] `source/robot/` and `source/app/` do NOT exist after this ticket.
- [ ] All of the above lands as exactly one commit.
- [ ] `just build-clean` succeeds and produces a hex.
- [ ] The stub is flashed to the bench robot and confirmed to banner
      (boot message visible over serial) — per
      `.claude/rules/hardware-bench-testing.md`, connect-only verification
      since the stub never energizes motors (no wheel/sensor check
      applicable to an inert stub).
- [ ] The surviving pytest subset passes (`uv run python -m pytest`).
- [ ] A repo-wide grep for every deleted header/module name (e.g.
      `runtime/`, `subsystems/`, `drive/`, `telemetry/`, `estimation/`,
      `hal/capability`, `i_kinematics`, `binary_channel`, `bringup_main`,
      `fiber_runner`, `codal.devicebus.json`, `ruckig`, `tinyekf`,
      `cmon-pid`) returns nothing under `source/`, `tests/`, `host/`.
- [ ] `codal.json`'s `MICROBIT_RADIO_MAX_PACKET_SIZE=250` setting is
      unchanged (not part of this delete — confirm it wasn't
      inadvertently touched).

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
