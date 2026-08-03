---
status: pending
priority: medium
---

# The "loud, never silent" observability contract is inert as shipped

## Description

2026-08-02 post-130 review, **F6 (MAJOR)**. The firmware's stated posture is that
faults are loud. Verified today: most of the loud parts are wired to nothing.

## The deficit flag cannot fire

`src/firm/app/drive.cpp:338-350` latches the speed-deficit fault only when
**both** clamps are saturated:

```cpp
const bool pidSaturatedLeft = gains_.pidMax > 0.0f && std::fabs(pidLeft) >= gains_.pidMax;
updateDeficit(std::fabs(errLeft) > bounds_.deficitThreshold && biasSaturatedLeft &&
                 pidSaturatedLeft, ...);
```

Every robot ships `wheel_pid_max: 0.0` (`tovez.json:144`, `togov.json:133`,
`tovez_nocal.json:119`), so `pidSaturatedLeft` is **permanently false**. The one
loud observable for silently slow-running wheels cannot fire.

This is not theoretical: 130-006 ran four residual sweeps delivering **70–85% of
commanded** and the flag stayed clear — correct per the code, useless per the
contract.

**Fix:** decouple the deficit condition from `pidMax`. A bias-pinned wheel, or a
sustained error with no clamp saturated at all, should still speak.

## The other dead or lying bits

| Bit | State |
|---|---|
| 8 `kFlagFaultI2CNak` | No data path. `MicroBitI2CBus`'s `errCount()`/`reentryViolations()` and its transaction ring have **zero callers** — the counts exist, the abstract `Devices::I2CBus` interface just does not expose them (`i2c_bus.h` exposes only `clearanceSafetyNetCount()`). Widen the interface with the rollup; the per-device breakdown stays on the concrete class. Absorbed here from the closed standalone issue. |
| 6 | **Saturates within ~1 s of boot** — every OTOS read trips the clearance safety net (`otos.cpp:333-353`). Count only *unexpected* trips, or the bit means nothing. |
| 10 (deadman), 12 (config-applied) | Wired to nothing. Delete or wire. |

## The wire carries no setpoint

`msg::Telemetry` has **no commanded wheel velocity and no applied duty**, and no
effective gains. A capture shows what the wheels did but not what they were
asked to do, so no host-side analysis can separate a controller error from a
command error. This is also why the systest dataset has to reconstruct
`cmd_vel` from tx records.

**Fix:** per-wheel `cmd_velocity` and applied duty in the frame.

## And no config read-back at all

Split out as its own issue because the post-mortem ranks it #1 by leverage —
see [[A-no-firmware-to-host-config-readback]].

## Test gap that let this ship

`applySpeedFloor()` and `updateDeficit()` — both live policies — have **zero
scenarios** in `src/tests/sim/unit/app_drive_harness.cpp` (744 lines, covering
Stage A/B/C well). Verified: no floor or deficit scenario exists. That is
sprint-130 midpoint finding #4, still open.

## Verification

- The deficit flag fires on a real 70–85%-delivery run and does not fire on a
  healthy one.
- Injecting a NAK through the sim bus advances `errCount()` and raises bit 8 on
  the next frame exactly once; bench: unplug the OTOS mid-session (a known NAK
  generator) and see bit 8 in a `tlm_log.py` capture.
- Bit 6 is clear on a healthy boot.
- Telemetry carries per-wheel commanded velocity and applied duty; the systest
  recorder reads them instead of reconstructing.
- `telemetry.h`'s bit comments match reality — today `:126-128` claims bit 8 has
  "no per-transaction NAK aggregate", which is false; the aggregate exists and
  is merely unreachable.
- Harness scenarios exist for the floor and the deficit policy.

## Related

- `docs/code_review/2026-08-02-post-130-wheels-solid-review.md` F6.
- `docs/code_review/2026-08-02-sprint-130-midpoint.md` finding 4.
- Absorbs the closed `surface-i2c-error-counts-through-the-bus-interface.md`.
