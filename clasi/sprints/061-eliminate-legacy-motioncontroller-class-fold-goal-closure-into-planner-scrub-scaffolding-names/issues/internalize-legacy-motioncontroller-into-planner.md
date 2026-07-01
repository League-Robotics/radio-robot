---
status: in-progress
sprint: '061'
tickets:
- 061-001
- 061-002
- 061-003
- 061-004
- 061-005
- 061-006
- 061-007
---

# Eliminate the legacy `MotionController` class — fold goal-closure into `Planner`, scrub all remaining legacy/scaffolding names

## Context

Sprint 060 completed the ordered-tick cutover (legacy loop deleted, `USE_ORDERED_TICK`
gone, `Drive2/bvc2/MotionController2` renamed to `Drive/bvc/Planner`). But one piece
of legacy survived: the old imperative `MotionController` class
(`source/superstructure/MotionController.h/.cpp` + `source/control/MotionControllerBegin.cpp`)
is still a public `Robot` value member, wrapped by reference inside `Planner` (which
holds a `MotionController&`) and also by `Superstructure` (holds a `MotionController&`).

The stakeholder wants **no legacy code left at all**. This issue eliminates the
standalone `MotionController` class so the new architecture's `Planner` is the single
goal-closure engine, and scrubs the residual `2`-era scaffolding names from the test
infrastructure.

This is a LARGE, high-risk refactor (the S/T/D/G state machines + BVC + MotionCommand
ownership are the core motion logic). It is intentionally run as its own sprint, on a
sprint branch the stakeholder will bench-test on tovez before any merge to master.
Behavior must be preserved — this is a structural move, not a behavior change.

## Goal (end-state)

- **No `MotionController` class or files.** The goal-closure logic (driveAdvance,
  the begin*() entry points, S/T/D/G state machines, the internal `_bvc`,
  `_activeCmd`/MotionCommand ownership, safety one-shot, mode()) lives in `Planner`
  (or is owned privately by `Planner`). `source/superstructure/MotionController.h/.cpp`
  and `source/control/MotionControllerBegin.cpp` are deleted (their contents move into
  `Planner.*` / a Planner-owned translation unit).
- **`Robot` no longer has a `motionController` member.** All call sites reach the
  goal-closure engine through `Planner` (`robot.planner`).
- **`Superstructure` references `Planner`** (not `MotionController`) for its
  goal-start dispatch + `mc()` accessor.
- **`Planner` exposes the API the call sites need** (e.g. `mode()`, `beginDistance`,
  `disableSafetyOneShot`, `hasActiveCommand`, `emitToActiveChannel`, `activeCmd`,
  `getMotionCommands` context) — or those call sites are rerouted.
- **Test-infra scaffolding names scrubbed:** rename the C-ABI shim symbols and Python
  helpers/filenames that still carry `drive2`/`motioncontroller2`/`mc2`
  (`drive2_api_*`, `bus_drain_api_drive2_*`, `Drive2Ctx`, `test_drive2_subsystem.py`,
  `test_motioncontroller2_smoke.py`, etc.) to the canonical names. Update both the
  C++ `extern "C"` names AND the Python ctypes call sites together.
- `grep -rIn "MotionController\b\|MotionController2\|Drive2\|bvc2\|drive2\|mc2" source/ tests/`
  returns nothing meaningful (provenance comments may be reworded or removed).

## Known call sites to reroute (verify with grep before starting)

`source/robot/Robot.h` / `Robot.cpp` (member decl + construction + `setHardwareState`,
`setRobotCtx`, `_motionCtx.mc`, `otosCorrect` emit, `distanceDrive` → `beginDistance`),
`source/superstructure/Superstructure.h/.cpp` (`MotionController& _mc`, `mc()`),
`source/superstructure/Planner.h/.cpp` (currently holds `MotionController& _mc` — absorb it),
`source/robot/RobotTelemetry.cpp` (`motionController.mode()` ×2 → `planner.mode()`),
`source/commands/SystemCommands.cpp` (`disableSafetyOneShot`, `_motionCtx.mc`),
`source/commands/MotionCommands.cpp/.h` (`MotionCtx::mc`, getMotionCommands),
`source/control/MotionControllerBegin.cpp` (all the begin*() bodies),
`source/commands/MotionCommand.*`, `source/control/MotionEventSink.h`,
`source/robot/LoopScheduler.h`, `source/robot/LoopTickOnce.cpp` (comments),
`source/COMMANDS.md` (doc references).

## Acceptance

- No `MotionController`/`MotionController2` class or files remain; `Planner` is the
  sole goal-closure engine; `Superstructure` and all handlers reference `Planner`.
- Test-infra `drive2`/`mc2` scaffolding names renamed to canonical (C ABI + Python).
- Codebase compiles cleanly (host + firmware `build.py --clean`).
- Host suite green except the 2 known-baseline config-golden failures
  (`test_tovez_validates_against_schema`, `test_default_robot_config_unchanged`),
  with the suite run **twice** to confirm stability.
- Behavior preserved: `test_golden_tlm.py`, `test_059_ordered_tick_parity.py`,
  `test_planner_subsystem.py` all green; golden-TLM byte-identical (mode char,
  pose, twist unchanged).
- Firmware `MICROBIT.hex` builds clean; bench checklist updated/retained for tovez.

## Notes

Follow-on from sprint 060. Test command is `uv run python -m pytest` (NOT
`uv run pytest`). Relates to [[message-based-subsystem-architecture]]. The sprint is
left OPEN on its branch after implementation — the stakeholder bench-tests on the
branch before close.
