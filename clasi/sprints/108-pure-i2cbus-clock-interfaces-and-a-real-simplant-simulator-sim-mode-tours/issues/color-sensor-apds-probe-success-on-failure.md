---
status: in-progress
sprint: '108'
tickets:
- 108-008
---

# Color sensor APDS probe detects "present" on a NAK'd bus read

From the 2026-07-13 code review (docs/code_review/2026-07-13-devices-drive-review.md,
Part 1 finding **M4**). The `Devices::ColorSensorLeaf` is KEPT by the single-loop
rebuild, so this defect survives it; it also affects the firmware running today.

## Defect

The APDS fallback detection in `beginStep` (source/devices/color_sensor.cpp:57-70) is
success-on-failure: `readReg8` (color_sensor.cpp:195-199) ignores transaction status, so
a NAK'd readback leaves `out=0` — and `en == 0x00` is exactly the "detected" condition.
A robot with NO color sensor at all latches `present()==true`, runs `initApds()` against
nothing, and issues failing APDS transactions at every due perception slot forever
(perpetual bus errors, connected-flapping, the absent-device skip defeated). The Alt
probe path is safe only by accident (failure → 0 → "not found").

## Fix

Probe via a status-returning read (`readReg8Status()`) and require transaction OK before
concluding anything from the register value. Verify on the bench by booting an image
with the color sensor unplugged: `present()` must latch false and the perception slot
must skip it (no recurring bus errors in the I2C diagnostics).
