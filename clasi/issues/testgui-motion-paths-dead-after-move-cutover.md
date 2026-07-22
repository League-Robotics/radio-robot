---
status: pending
---

# TestGUI motion paths dead after the MOVE cutover (S managed + Turn on both transports; unmanaged dead on serial)

## Description

Post-115-117, TestGUI's motion buttons are broken: managed S-drive and Turn
do nothing on BOTH transports; unmanaged S works only in Sim (serial
unmanaged is a silent hasattr no-op). Stakeholder-reported 2026-07-22.

## Cause

- Serial managed path: `binary_bridge.translate_command()` is a permanent
  dead stub (legacy_verbs/render modules deleted long ago) — returns ERR,
  sends nothing (binary_bridge.py:86-102, 209-210).
- Sim managed path: `_run_motion_async` imports `planner.tour`, whose module
  body reads deleted `telemetry_pb2.ACK_STATUS_*` (tour.py:126) →
  AttributeError kills the worker silently; beyond that, `SimLoop.move()`
  still builds the deleted flat arc-Move fields (sim_loop.py:606-610).
- Serial unmanaged: `_HardwareTransport` has no `run_unmanaged`; the GUI's
  hasattr guard silently no-ops (__main__.py:767,772).

## Proposed fix (OOP, tiers A+B; tours stay dormant)

Port the single-leg button paths to the new protocol: `SimLoop.move()` →
new Move schema (distance leg = MoveTwist(v_x)+stop distance; turn leg =
MoveTwist(omega)+stop angle, host-picked yaw rate); `SimTransport`
D/RT/SEG dispatch → direct `self._loop.move(...)`, no planner.tour import;
`_HardwareTransport` gains `run_unmanaged` + a D/RT/SEG → move_twist
dispatch (or direct button wiring). Distinct move ids for completion acks.

## Related

- angle-stop-overshoot-61-73-percent-on-hardware.md — GUI turns will show
  the same overshoot; expected until the trajectory-controller arc.
