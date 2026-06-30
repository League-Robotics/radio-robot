---
status: pending
---

# Make the ordered-tick loop the default (close the 3 Phase-3 parity gaps)

## Context

Phase 3 (sprint 059) wired the message-driven ordered tick into `loopTickOnce` but
left it **`#ifdef USE_ORDERED_TICK`, OFF by default** (ticket 059-005) because a
byte-clean live cutover was not achievable in-sprint. The legacy loop remains live;
the new path is built, routed (`BusDrain`), and unit-tested, but not the default.

This issue tracks closing the 3 documented parity gaps so `USE_ORDERED_TICK` can
become the default and the legacy loop deleted. **Requires human review** — closing
gap #1 means regenerating the golden-TLM snapshot, which must be a deliberate,
reviewed acceptance of new telemetry, not an autonomous rubber-stamp.

## The 3 parity gaps (from 059-005)

1. **Drive2 owns a private `_hw` instead of `robot.state.actual`.** `Drive2::tickUpdate()`
   refreshes its own `HardwareState _hw`, but `buildTlmFrame()` reads `robot.state.actual`.
   The ordered-tick path currently keeps `drive.periodic()` alive to keep `state.actual`
   fresh — defeating the cutover. **Fix options:** (a) Drive2 writes its fused result back
   into `robot.state.actual`, or (b) `buildTlmFrame` reads from `drive2.state()`.
   Option (b) changes telemetry bytes → **regenerate `golden_tlm_capture.json`** under
   review and confirm the new values are correct (not a silent drift).

2. **`MotorController::setCommandsRef` override.** Drive2's ctor sets
   `_mc.setCommandsRef(&_outputs)`, but the `Robot` ctor body then calls
   `motorController.setCommandsRef(&state.outputs)`, overriding it. Decide a single
   authority for the motor-output buffer in the ordered-tick world.

3. **`Sensors.tick()` lag timers independent of `LoopTickState`.** The `Sensors` facade
   keeps its own `_lastLineTick`/`_lastColorTick`, separate from
   `LoopTickState.lastLine/lastColor`. In the ordered tick, reconcile to a single
   schedule so line/color reads fire exactly once per due interval.

## Acceptance

- Build with `USE_ORDERED_TICK` as the default (legacy path removed or clearly deprecated).
- Full host suite green with the ordered tick LIVE — including the full-robot golden-TLM
  and motion tests — with any golden-snapshot change reviewed and justified.
- Bench parity confirmed on tovez (VW + TURN + a goto/turn/distance) vs the legacy build.

## Notes

Found during sprint 059 (Phase 3). Relates to [[message-based-subsystem-architecture]].
Until this lands, the robot runs the legacy loop; the message-driven architecture is
present and unit-tested but not the live control path.
</content>
