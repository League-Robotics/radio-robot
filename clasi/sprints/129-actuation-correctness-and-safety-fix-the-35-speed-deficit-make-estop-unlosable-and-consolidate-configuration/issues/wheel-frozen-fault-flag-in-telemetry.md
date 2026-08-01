---
status: in-progress
priority: medium
sprint: '129'
tickets:
- 129-002
---

# Wheel-frozen fault flag in telemetry, surfaced as a red GUI banner

Stakeholder, 2026-07-31: *"If you've commanded an encoder and it's been
commanded to move for the last cycle and it hasn't moved, then it's frozen,
right? ... That should be on the telemetry, and if it is, then the test program
should be throwing big red errors when it happens."*

The detection already existed and had **zero consumers**:
`Devices::NezhaMotor::wedgeSuspect()` was computed and never read by anything.

## What was built (and is being abandoned with the rest)

- `telemetry.h` — `kFlagFaultWheelFrozenLeft = 1u << 19`,
  `kFlagFaultWheelFrozenRight = 1u << 20`.
- `robot_state.h` — `wheelFrozenLeft` / `wheelFrozenRight` in `Health`.
- `robot_loop.cpp` — publishes `motorL_.wedgeSuspect()` / `motorR_.wedgeSuspect()`.
- Host: decode in `protocol.py`, red banner in the TestGUI, and the host drive
  loop aborts the leg on the flag rather than driving on.
- `src/tests/testgui/test_wheel_frozen_indicator.py`.

## Correction worth carrying forward

An earlier claim in this session that the published flag was the ungated
`wedgeLatched_` was wrong in a way that matters: `wedgeLatched_` fires on
healthy moves too, so publishing it directly would produce a banner that cries
wolf on every leg. The flag must be the **gated** suspicion
(`wedgeSuspect()`) — commanded nonzero duty for N consecutive cycles with no
encoder change — not the raw latch.

## Acceptance

- Physically stall one wheel on the stand; the correct per-wheel flag sets
  within ~0.5 s and the GUI shows a red banner naming which wheel.
- A full healthy 700 mm leg raises neither flag — verify explicitly, because a
  false positive here is worse than no flag at all.

Related: [[otos-frozen-at-a-constant-on-tovez]] (the same
reporting-but-not-changing gap, for a sensor rather than an actuator)
