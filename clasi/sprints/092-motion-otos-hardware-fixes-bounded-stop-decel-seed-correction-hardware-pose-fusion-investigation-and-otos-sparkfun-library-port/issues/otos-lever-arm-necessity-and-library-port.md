---
status: in-progress
sprint: 092
tickets:
- 092-003
- 092-004
---

# Re-test whether the OTOS host-side lever-arm is necessary, and faithfully port the SparkFun OTOS library

Stakeholder (Eric) design discussion 2026-07-07, prompted by review of
[`source/hal/lever_arm.h`](../../source/hal/lever_arm.h). Related:
[[poseestimator-fused-pose-frozen-on-hardware]] (shares the OTOS-on-hardware
fusion path), [[odometer-owns-reset-and-fusability]].

## Hypothesis

The host-side lever-arm compensation — [`source/hal/lever_arm.h`](../../source/hal/lever_arm.h)'s
`LeverArm::sensorToCentre()` / `centreToSensor()`, ~15 lines of pure math whose
**only** production consumer is [`source/hal/otos/otos_odometer.cpp`](../../source/hal/otos/otos_odometer.cpp)
(call sites `:148`, `:195`) — is probably **not necessary**. The OTOS chip has a
documented mounting-offset register (`REG_OFFSET`, 0x10–0x15) that compensates
the lever arm internally; you set it once in the device and read chassis-centre
pose directly. The likely real problem is that our OTOS driver is a partial,
hand-rolled implementation. The fix: **port the upstream SparkFun OTOS library
faithfully** (near line-by-line), after which `REG_OFFSET` works and the entire
host-side lever-arm can be deleted.

Upstream references to mirror:
- Arduino C++ (closest to our C++): <https://github.com/sparkfun/SparkFun_Qwiic_OTOS_Arduino_Library/>
- Python: <https://github.com/sparkfun/qwiic_otos_py/blob/2a26ada89ec03b15bb15b70223db47c4cb9e8ef6/build/lib/qwiic_otos.py>

## Why the old "REG_OFFSET is unwritable" claim is suspect

Both [`lever_arm.h`](../../source/hal/lever_arm.h)'s header and
[`source_old/hal/real/OtosSensor.cpp:38-43`](../../source_old/hal/real/OtosSensor.cpp#L38-L43)
assert this specific OTOS unit *"silently ignores writes to the offset register
block 0x10-0x15 (the I2C write ACKs, but the register reads back 0), while
position 0x20 and scalars 0x04/0x05 write and hold fine."* That claim is the
sole justification for doing the lever arm host-side (and for the `db11b7c`
~433 mm phantom-spin regression lore captured in `lever_arm.h`'s
same-instant-heading contract).

But the claim is internally suspicious:

- Upstream writes `REG_OFFSET` (`kRegOffXL = 0x10`) through the **exact same**
  `_writePoseRegs` helper, and the **exact same** int16 scaling
  (`kMeterToInt16 = 32768/10`, `kRadToInt16 = 32768/π`), that it uses to write
  the **position** registers (`kRegPosXL = 0x20`).
- Our own driver already writes 0x20 successfully — boot-zero via
  `writeXYH(kRegPositionXl, …)` ([`otos_odometer.cpp:67`](../../source/hal/otos/otos_odometer.cpp#L67))
  and `setPose()` ([`:217`](../../source/hal/otos/otos_odometer.cpp#L217)).
- Same bus, same code path, same scaling, allegedly different result. That
  smells like a **flawed readback** in the original verification — wrong scaling
  on read-back, reading before the chip latched, the wrong signal-process mode,
  or an old chip-firmware quirk — not a genuine dead register.

Our constants already equal upstream's: `kPosMmPerLsb = 0.305`
([`otos_odometer.h:215`](../../source/hal/otos/otos_odometer.h#L215)) ==
`kInt16ToMeter` (10 m / 32768 ≈ 0.3052 mm/LSB), and `kHdgRadPerLsb` ==
`kInt16ToRad` (π / 32768). So writing **and reading back** `REG_OFFSET` is
trivial with constants we already have — the re-test is cheap.

## Work this issue should cover

1. **Bench re-test on the stand** (robot is mounted, wheels off the ground — safe
   to spin; see `.claude/rules/hardware-bench-testing.md`). Write `REG_OFFSET`
   (0x10–0x15) with the real mounting offset, **read it back**; then drive a pure
   in-place spin and watch for the lever-arm phantom-translation arc (the
   `db11b7c` signature). If the chip compensates, the phantom translation
   disappears **and** the register reads back non-zero → the chip honors it.
2. **Faithful port of the SparkFun OTOS library** into a proper `Hal` OTOS driver
   (register map, scaling constants, `setOffset`/`getOffset`, `setPosition`,
   `setLinearScalar`/`setAngularScalar`, signal-process config, IMU calibration,
   product-ID check) mirroring upstream near line-by-line — conforming to project
   naming rules (CamelCase types / lowerCamelCase functions, **no units in
   identifiers**, units in `// [unit]` tags; wire/register names exempt).
3. **If the chip honors `REG_OFFSET`:** delete [`source/hal/lever_arm.h`](../../source/hal/lever_arm.h)
   and all host-side lever-arm compensation plus its tests
   (`tests/sim/unit/lever_arm_harness.cpp`, `tests/sim/unit/test_lever_arm.py`,
   and the `otos_odometer_harness.cpp` assertions that check against it). The
   offset becomes a one-time device write in `begin()`; `tick()`/`setPose()` drop
   their `LeverArm::` calls and the mounting-yaw round-trip simplifies.
4. **If the chip genuinely does not honor it** (old claim holds under a clean
   re-test): keep host-side compensation but **fold** the two `lever_arm.h`
   functions into `OtosOdometer` — they have exactly one consumer, and the
   standalone file's "future shared sim leaf" rationale
   ([`lever_arm.h:19-24`](../../source/hal/lever_arm.h#L19-L24)) is speculative /
   YAGNI. Record the confirmed hardware defect with fresh bench evidence
   (supersedes the `source_old` note).

## Acceptance

A clean bench verdict on whether `REG_OFFSET` compensates on this unit, and the
driver left in exactly one of the two end states above — with `lever_arm.h`
either deleted or folded into `OtosOdometer`, never left standalone.
