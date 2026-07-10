---
status: pending
---

# Sprint 095 — Restore Line + Color Sensors as Ticked Blackboard Devices

## Context

Stakeholder directive (2026-07-09): "adding back in the color sensor and the
line sensor as devices that can report. You're going to tick them. They're
going to copy their state into the blackboard."

The new tree has the `Hal::LineSensor` / `Hal::ColorSensor` faceplates
([line_sensor.h](source/hal/capability/line_sensor.h),
[color_sensor.h](source/hal/capability/color_sensor.h)) and the
`msg::LineSensorState` / `msg::ColorSensorState` messages (host-safe PODs,
already generated) — but **declaration-only**: no concrete leaves, no
`Subsystems::Hardware` seam, no blackboard cells, nothing ticked, no wire
report. The stakeholder has already sketched the landing zone in uncommitted
`main.cpp` WIP (`hardware_main()` split + `// Ticks` and `// Commit state to
the blackboard` section markers) — the sprint builds on that WIP, never
clobbers it.

Issue captured: `clasi/issues/restore-line-and-color-sensors-as-ticked-blackboard-devices.md`
— that file and this one are ONE work item (this is its stakeholder-approved
plan); the sprint claiming this work must link BOTH files.
The sprint-planner has fully drafted the sprint (research verified against the
live tree); its complete artifact drafts live in
`/Users/eric/.claude/plans/sprightly-stirring-riddle-agent-ac14a070abef8c401.md`.

## Two corrections found during planning (verified against code, not docs)

1. The live `TLM` verb is `handleTlm()` in
   [motion_commands.cpp](source/commands/motion_commands.cpp) (094-006).
   `source/telemetry/tlm_frame.cpp` is parked/unregistered since 093 —
   editing it would be dead work. Wire reporting extends `handleTlm()`.
2. `Subsystems::Hardware::tick()` was never actually renamed to
   `serviceBus()` despite architecture-update-094's prose (pre-existing
   doc/code drift; not fixed here).

## Design (follows existing precedents, no new patterns)

- **Real leaves** `Hal::PlanetXLineSensor` (I2C 0x1A) /
  `Hal::PlanetXColorSensor` (0x43 primary, APDS9960 0x39 fallback) in new
  `source/hal/planetx/` — named for the device vendor, not the bus owner
  (the `Hal::OtosOdometer` precedent). Ported from
  `source_old/hal/real/{Line,Color}Sensor.cpp` — **except** the color
  sensor's blocking `readRGBC()` (`fiber_sleep` up to 250 ms — the exact
  class of bus stall sprints 078–093 eliminated); port the non-blocking
  `pollRGBC()` ready-bit variant instead.
- **Bus safety**: `tick()` is non-blocking and rate-limited, reusing
  `I2CBus`'s per-device lazy-clearance mechanism and `OtosOdometer`'s
  `kReadPeriod`(20 ms)/`kBusClearance`(4000 us) template; sensor ticks are
  sequenced strictly **after** each pass's `hardware.tick(now)` bus action —
  never interleaved with the Nezha brick's 0x46 REQUEST/COLLECT split-phase
  (the 093 bus-hang scar). Rate gate also satisfies Hardware's
  "same-`now` re-tick is a no-op" contract for free.
- **Sim leaves** `Hal::SimLineSensor` / `Hal::SimColorSensor`
  (schedule-cycling observation models, ported near-verbatim from
  `source_old/hal/sim/`), owned by `SimHardware` with `sim*()`
  concrete-twin accessors for test scripting.
- **Seams**: `Subsystems::Hardware` gains `lineSensor()` / `colorSensor()`
  virtual accessors defaulting to shared-static `Hal::NullLineSensor` /
  `NullColorSensor` (the 090-003 `NullOdometer` pattern — no null checks
  anywhere), returning **by reference** (deliberate, flagged deviation from
  `odometer()`'s historical pointer; matches `motor(i)`).
- **Blackboard**: two new state-plane cells `bb.line` / `bb.color`
  (msg types already meet blackboard.h's host-safe-POD bar).
- **Tick + commit in BOTH composition roots, identically**: `main.cpp`'s
  loop (into the stakeholder's WIP markers) and `Rt::MainLoop`
  (`tick()`/`commit()`, the sim-harness path) — lockstep is an explicit
  ticket acceptance line.
- **Wire report**: `handleTlm()` appends `line=r0,r1,r2,r3` and
  `color=r,g,b,c` (old tree's exact wire vocabulary), each gated on its
  sensor's `connected` flag, omitted when never probed; grow
  `body[240]`/`rbuf[272]` to fit. All uint32 — no `%f`/newlib-nano concern.

**Out of scope**: calibration UX (no live SET/GET surface), line-following
behavior, boot-config generator changes, reviving tlm_frame.cpp, and
protocol-v2.md TLM reconciliation (pre-existing 094 debt — file a small
docs follow-up issue instead).

## Open questions — resolved

1. **Host TLM parser tolerance**: VERIFIED safe — `parse_tlm()`
   ([protocol.py:230](host/robot_radio/robot/protocol.py#L230)) is kv-driven
   and ignores unknown keys. Host *consumption* of the new fields belongs to
   the pending `realign-host-tooling` pool issue, not this sprint.
2. **Color chip variant**: port both variants with `begin()` auto-detect
   (cheap, matches old driver); the bench gate reveals which chip answers.
3. **Rate limits**: default both sensors to the OTOS 20 ms / 4000 us
   constants; per-sensor tuning is ticket-level judgment.
4. **Reference-returning seams**: approved (Decision 2 above).
5. **protocol-v2.md**: file a documentation-only follow-up issue; don't
   touch the doc this sprint.

## Execution path (CLASI)

1. Sprint-planner runs its staged MCP sequence: `create_sprint` (→ 095),
   `link_sprint_issues`, `detail_sprint`, writes the three drafted
   artifacts (sprint.md, usecases.md SUC-001..004, architecture-update.md),
   records the architecture-review gate (self-review verdict: APPROVE).
2. Team-lead records `stakeholder_approval` (this plan's approval),
   advances phase, re-engages planner to cut tickets in dependency order
   (leaves → seams/ownership → blackboard+wiring → TLM → sim tests →
   HITL bench gate).
3. `acquire_execution_lock`, execute tickets serially via programmer
   agents on the sprint branch. First wiring ticket absorbs/commits the
   stakeholder's main.cpp WIP as its base.
4. **Bench gate (SUC-004)**: build (`just build-clean`), flash via
   `mbdeploy deploy <full-UID> --hex MICROBIT.hex` (robot on stand),
   confirm over the real link: 4 plausible changing `line=` channels
   (white vs black target), plausible changing `color=` RGBC, and no
   regression to drive/encoder behavior with sensors ticking
   (before/after `MOVE`/`S` comparison).
5. Sprint-review, then `close_sprint` — stash the unrelated dirty files
   first (notebook CSVs, devices.json registry entry) so the bump commit
   doesn't sweep them; audit the bump commit after.

## Verification

- Host/sim: `just build-sim` + `uv run python -m pytest` (tests/sim) —
  new harnesses for leaf tick/cache/no-op-on-same-`now`, and a bare-loop
  test proving `line=`/`color=` presence/omission end-to-end via
  `sim_command()`. Existing suites stay green.
- Firmware: `just build` compiles clean for ARM; flash + stand check per
  the bench gate above (that gate, not sim, is what "done" means for the
  sensors).
