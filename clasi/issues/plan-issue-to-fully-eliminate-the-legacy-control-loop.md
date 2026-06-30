---
status: pending
---

# Plan: Issue to fully eliminate the legacy control loop

## Context

`source/robot/LoopTickOnce.cpp` carries **two complete control paths** behind
`USE_ORDERED_TICK` (default = LEGACY). The new message-based / ordered-tick
architecture was built additively across sprints 054–059 (`drive2`, `bvc2`,
`subsystems::Sensors`, `MotionController2 planner`) but never became the live
default because 059-005 left **3 documented parity gaps** unresolved. The legacy
loop and its now-shadowed members (`drive`, `MotionController` + its internal
`_bvc`, the legacy `lineSensor`/`colorSensor_` periodic paths, `Superstructure`'s
direct use) remain the production code path. The `2` suffixes are temporary
migration scaffolding.

The stakeholder wants **no legacy left**: close the gaps, flip the default,
delete the legacy loop and all dead members, and rename the `2` subsystems back
to their real names. This change is being made to end a half-finished migration
that currently doubles the control-path surface area and confuses readers (the
question that prompted this: "why do we have a legacy?").

**Deliverable of this task:** rewrite the existing issue
`clasi/issues/make-ordered-tick-the-default-close-parity-gaps.md` into a
comprehensive, staged cleanup issue. (This is *the only file* this task writes —
the actual code cleanup happens later, as its own sprint, after the issue is
assessed by the CLASI process.)

## Decisions (confirmed with stakeholder)

- **Rewrite/expand the existing issue** — no duplicate issue file.
- **Full scope**: close 3 gaps → make `USE_ORDERED_TICK` default → delete legacy
  loop + dead members → **rename** `drive2→drive`, `bvc2→bvc`,
  `MotionController2→planner` (drop the `2` scaffolding).

## The 3 parity gaps (verified in code)

1. **Drive2 owns a private `_hw`, not `robot.state.actual`.**
   `Drive2::tickUpdate()` ([source/subsystems/drive/Drive2.cpp:74](source/subsystems/drive/Drive2.cpp#L74))
   refreshes its own `HardwareState _hw`; but `buildTlmFrame()`
   ([source/robot/RobotTelemetry.cpp:19](source/robot/RobotTelemetry.cpp#L19))
   reads `robot.state.actual`. The ordered-tick path keeps `drive.periodic()`
   alive ([LoopTickOnce.cpp:190](source/robot/LoopTickOnce.cpp#L190)) *only* to
   keep `state.actual` fresh for TLM. **Fix:** make `buildTlmFrame` read from
   `drive2.state()` / `sensors.state()` (or project Drive2's result into
   `state.actual`), then delete the `drive.periodic()` crutch. Telemetry bytes
   change → **regenerate `tests/_infra/golden_tlm_capture.json` under human review.**

2. **`MotorController::setCommandsRef` authority conflict.** Drive2's ctor binds
   `_mc.setCommandsRef(&_outputs)`, but the `Robot` ctor later calls
   `motorController.setCommandsRef(&state.outputs)`
   ([source/robot/Robot.cpp:121](source/robot/Robot.cpp#L121)), overriding it.
   **Fix:** pick a single authoritative motor-output buffer for the ordered-tick
   world and remove the dual wiring.

3. **`Sensors.tick()` lag timers independent of `LoopTickState`.** The facade
   keeps private `_lastLineTick`/`_lastColorTick`
   ([source/subsystems/sensors/Sensors.h](source/subsystems/sensors/Sensors.h))
   separate from `LoopTickState.lastLine/lastColor`. **Fix:** reconcile to one
   schedule so line/color reads fire exactly once per due interval; drop the
   legacy `lineSensor.periodic`/`colorSensor_.periodic` sync calls.

## What gets deleted / renamed (the "no legacy" work)

After gaps close and `USE_ORDERED_TICK` is the only path:

- **Delete** the `#ifndef USE_ORDERED_TICK` legacy branch in
  [LoopTickOnce.cpp:57-159](source/robot/LoopTickOnce.cpp#L57) and the
  `#ifdef`/`#else`/`#endif` scaffolding (keep only the ordered-tick body).
- **Delete dead members** from [Robot.h](source/robot/Robot.h) /
  [Robot.cpp](source/robot/Robot.cpp) once unreferenced:
  `subsystems::Drive drive`, legacy `subsystems::LineSensor lineSensor` +
  `subsystems::ColorSensor colorSensor_` periodic usage (Sensors facade owns the
  refs — confirm whether the wrapper instances can be folded in or stay as the
  facade's targets), `MotionController`'s internal `_bvc`, and any
  `_tlmBoundFn`/`drive.periodic` crutches. `subsystems::Ports ports` **stays**
  (no Ports2 yet) — call this out as remaining scaffolding, not a silent gap.
- **Rename (drop `2`):** `bvc2→bvc`, `subsystems::Drive2→subsystems::Drive`
  (after legacy `Drive` is gone), `MotionController2→planner` type stays but the
  member/identifier story should land on clean names. Sequence carefully against
  the C++ declaration-order constraints documented at
  [Robot.h:150-154](source/robot/Robot.h#L150).
- Verify nothing else references `USE_ORDERED_TICK` (grep confirms it lives only
  in LoopTickOnce.cpp today).

## Suggested staging in the issue (assessor will turn into sprint tickets)

1. **Gap close #1 (TLM source)** — rewire `buildTlmFrame`, drop `drive.periodic`
   crutch, regenerate golden capture *under review*.
2. **Gap close #2 (motor-output authority)** — single `setCommandsRef` owner.
3. **Gap close #3 (sensor schedule)** — unify lag timers.
4. **Flip default** — `USE_ORDERED_TICK` on; full host suite green LIVE.
5. **Delete legacy** — remove legacy loop branch + dead members.
6. **Rename / de-scaffold** — drop the `2` suffixes; mechanical, compiler-checked.
7. **Bench parity on tovez** — VW + TURN + a goto/turn/distance vs the
   pre-cutover build (human-operated; per the stand/floor rules).

## Files to reference in the issue (not modified by this task)

- [source/robot/LoopTickOnce.cpp](source/robot/LoopTickOnce.cpp) — the two paths + gap docs (lines 38-54)
- [source/robot/RobotTelemetry.cpp](source/robot/RobotTelemetry.cpp) — `buildTlmFrame` state reads
- [source/subsystems/drive/Drive2.cpp](source/subsystems/drive/Drive2.cpp) / `.h` — private `_hw`/`_outputs`, `state()`
- [source/robot/Robot.h](source/robot/Robot.h) / [Robot.cpp](source/robot/Robot.cpp) — member decls + ctor wiring
- [source/subsystems/sensors/Sensors.h](source/subsystems/sensors/Sensors.h) — facade lag timers
- [tests/simulation/unit/test_golden_tlm.py](tests/simulation/unit/test_golden_tlm.py) + `tests/_infra/golden_tlm_capture.json` — the canary + regen recipe
- [tests/simulation/unit/test_059_ordered_tick_parity.py](tests/simulation/unit/test_059_ordered_tick_parity.py) — VW/TURN parity oracles

## Acceptance (carried into the issue)

- `USE_ORDERED_TICK` is the *only* path; legacy loop branch and dead members deleted; `2` suffixes gone.
- Full host suite green with the ordered tick LIVE — including full-robot golden-TLM + motion tests — **any golden-snapshot change reviewed and justified by a human** (not auto-rubber-stamped).
- Bench parity confirmed on tovez (VW + TURN + a goto/turn/distance) vs the legacy build, before legacy deletion is final.
- `grep -r USE_ORDERED_TICK source/ tests/` returns nothing.

## Verification of this task

The task itself only writes the issue file. Verify by reading
`clasi/issues/make-ordered-tick-the-default-close-parity-gaps.md` after the edit:
it should retain `status: pending` frontmatter, cover all 3 gaps, the delete +
rename scope, the staged sequence, and the human-review gate on golden-TLM. The
actual cleanup is executed later as a CLASI sprint planned off this issue.
