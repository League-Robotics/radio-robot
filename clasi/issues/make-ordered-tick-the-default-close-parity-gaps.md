---
status: pending
---

# Make the ordered-tick loop the default and delete ALL legacy control-path code

## Context

`source/robot/LoopTickOnce.cpp` carries **two complete control paths** behind
`USE_ORDERED_TICK` (default = LEGACY). The message-based / ordered-tick
architecture was built additively across sprints 054–059 (`drive2`, `bvc2`,
`subsystems::Sensors`, `MotionController2 planner`) but never became the live
default because ticket 059-005 left **3 documented parity gaps** unresolved
(LoopTickOnce.cpp lines 38–54). The legacy loop and its now-shadowed members
(`subsystems::Drive drive`, `MotionController` + its internal `_bvc`, the legacy
`lineSensor`/`colorSensor_` periodic paths) remain the production code path.

The `2` suffixes (`drive2`, `bvc2`, `MotionController2`) are temporary migration
scaffolding. This issue tracks finishing the migration so there is **no legacy
left**: close the gaps, flip the default, delete the legacy loop and all dead
members, and rename the `2` subsystems back to their real names.

**Requires human review** — closing gap #1 means regenerating the golden-TLM
snapshot, which must be a deliberate, reviewed acceptance of new telemetry, not
an autonomous rubber-stamp. Bench parity on real hardware must be confirmed
before legacy deletion is final.

## The 3 parity gaps (verified in code)

1. **Drive2 owns a private `_hw`, not `robot.state.actual`.**
   `Drive2::tickUpdate()` (`source/subsystems/drive/Drive2.cpp:74`) refreshes its
   own `HardwareState _hw`; but `buildTlmFrame()`
   (`source/robot/RobotTelemetry.cpp:19`) reads `robot.state.actual`. The
   ordered-tick path currently keeps `drive.periodic()` alive
   (`LoopTickOnce.cpp:190`) *only* to keep `state.actual` fresh for TLM —
   defeating the cutover.
   **Fix:** make `buildTlmFrame` read from `drive2.state()` / `sensors.state()`
   (or project Drive2's result into `state.actual`), then delete the
   `drive.periodic()` crutch. Option (b) changes telemetry bytes →
   **regenerate `tests/_infra/golden_tlm_capture.json` under review** and confirm
   the new values are correct (not a silent drift).

2. **`MotorController::setCommandsRef` authority conflict.** Drive2's ctor binds
   `_mc.setCommandsRef(&_outputs)`, but the `Robot` ctor body then calls
   `motorController.setCommandsRef(&state.outputs)` (`source/robot/Robot.cpp:121`),
   overriding it.
   **Fix:** decide a single authoritative motor-output buffer for the
   ordered-tick world and remove the dual wiring.

3. **`Sensors.tick()` lag timers independent of `LoopTickState`.** The `Sensors`
   facade keeps its own `_lastLineTick`/`_lastColorTick`
   (`source/subsystems/sensors/Sensors.h`), separate from
   `LoopTickState.lastLine/lastColor`.
   **Fix:** reconcile to a single schedule so line/color reads fire exactly once
   per due interval; drop the legacy `lineSensor.periodic` /
   `colorSensor_.periodic` sync calls.

## Legacy deletion and de-scaffolding (the "no legacy left" work)

Once the gaps close and `USE_ORDERED_TICK` is the only path:

- **Delete** the `#ifndef USE_ORDERED_TICK` legacy branch in
  `LoopTickOnce.cpp:57-159` and the `#ifdef`/`#else`/`#endif` scaffolding (keep
  only the ordered-tick body).
- **Delete dead members** from `source/robot/Robot.h` / `Robot.cpp` once
  unreferenced: `subsystems::Drive drive`, the legacy
  `lineSensor.periodic`/`colorSensor_.periodic` usage (the `Sensors` facade owns
  the underlying refs — confirm whether the wrapper instances fold in or remain
  as the facade's targets), `MotionController`'s internal `_bvc`, and the
  `_tlmBoundFn`/`drive.periodic` TLM crutch.
- **Keep `subsystems::Ports ports`** — there is no `Ports2` facade yet, so the
  legacy `ports.periodic(ts, now)` call stays. Call this out explicitly as
  remaining scaffolding (a candidate for a future `Ports2`), **not** a silent gap.
- **Rename (drop the `2`):** `bvc2→bvc`, `subsystems::Drive2→subsystems::Drive`
  (after legacy `Drive` is removed), and land `MotionController2`/`planner` on
  clean names. Sequence carefully against the C++ declaration-order constraints
  documented at `Robot.h:150-154` (bvc before drive2 before sensors before
  planner).
- Confirm nothing else references `USE_ORDERED_TICK` (today it lives only in
  LoopTickOnce.cpp).

## Suggested staging (for sprint planning)

1. **Gap close #1 (TLM source)** — rewire `buildTlmFrame` to `drive2.state()` /
   `sensors.state()`, drop the `drive.periodic()` crutch, regenerate the golden
   capture *under human review*.
2. **Gap close #2 (motor-output authority)** — single `setCommandsRef` owner.
3. **Gap close #3 (sensor schedule)** — unify lag timers to one schedule.
4. **Flip default** — make `USE_ORDERED_TICK` the default; full host suite green
   with the ordered tick LIVE.
5. **Delete legacy** — remove the legacy loop branch and all dead members.
6. **Rename / de-scaffold** — drop the `2` suffixes (mechanical, compiler-checked).
7. **Bench parity on tovez** — VW + TURN + a goto/turn/distance vs the
   pre-cutover build (human-operated; per the stand/floor rules).

## Key references

- `source/robot/LoopTickOnce.cpp` — the two paths + gap docs (lines 38-54)
- `source/robot/RobotTelemetry.cpp` — `buildTlmFrame` state reads
- `source/subsystems/drive/Drive2.{h,cpp}` — private `_hw`/`_outputs`, `state()`
- `source/robot/Robot.{h,cpp}` — member declarations + ctor wiring
- `source/subsystems/sensors/Sensors.h` — facade lag timers
- `tests/simulation/unit/test_golden_tlm.py` + `tests/_infra/golden_tlm_capture.json` — the canary + regen recipe
- `tests/simulation/unit/test_059_ordered_tick_parity.py` — VW/TURN parity oracles

## Acceptance

- `USE_ORDERED_TICK` is the *only* path; legacy loop branch and dead members
  deleted; the `2` suffixes are gone.
- Full host suite green with the ordered tick LIVE — including the full-robot
  golden-TLM and motion tests — with **any golden-snapshot change reviewed and
  justified by a human** (not auto-rubber-stamped).
- Bench parity confirmed on tovez (VW + TURN + a goto/turn/distance) vs the
  legacy build, before legacy deletion is final.
- `grep -r USE_ORDERED_TICK source/ tests/` returns nothing.

## Notes

Found during sprint 059 (Phase 3). Relates to [[message-based-subsystem-architecture]].
Until this lands, the robot runs the legacy loop; the message-driven architecture
is present and unit-tested but is not the live control path.
