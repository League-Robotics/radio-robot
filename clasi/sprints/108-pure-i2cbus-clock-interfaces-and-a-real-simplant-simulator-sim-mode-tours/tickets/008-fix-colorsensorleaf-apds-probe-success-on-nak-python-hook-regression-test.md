---
id: "008"
title: "Fix ColorSensorLeaf APDS probe success-on-NAK + Python hook regression test"
status: open
use-cases: ["SUC-044"]
depends-on: ["005"]
github-issue: ""
issue: "color-sensor-apds-probe-success-on-failure.md"
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix ColorSensorLeaf APDS probe success-on-NAK + Python hook regression test

## Description

From the 2026-07-13 code review (finding M4). `Devices::ColorSensorLeaf::
beginStep` (`source/devices/color_sensor.cpp:57-70`) probes the APDS
fallback via a status-IGNORING `readReg8()` (`color_sensor.cpp:195-199`).
A NAK'd readback leaves `out=0`, and `en == 0x00` is exactly the
"detected" condition — so a robot with NO color sensor at all latches
`present()==true` and issues failing APDS transactions at every due
perception slot forever.

Fix: probe via the status-returning `readReg8Status()` (already present on
the class — used by other call sites, e.g. `readReg16Status`) and require
the transaction to report OK before concluding `en==0x00` means "the
device answered with enable-register value 0."

Concretely, in `beginStep()`'s `ApdsProbe` phase (color_sensor.cpp:57-70):
```cpp
// Before
writeReg8(kColorDeviceAddrApds, 0x80, 0x00);
uint8_t en = readReg8(kColorDeviceAddrApds, 0x80);
if (en == 0x00) { ... }

// After
writeReg8(kColorDeviceAddrApds, 0x80, 0x00);
uint8_t en = 0;
bool ok = readReg8Status(kColorDeviceAddrApds, 0x80, en);
if (ok && en == 0x00) { ... }
```
(Adjust exact call shape to the surrounding code; the point is the OK
check gates the conclusion, not just the byte value.)

This sprint's SimPlant (ticket 002) NAKs unrecognized/absent color-sensor
addresses by design — this fix's regression test is a natural Stage 4
Python hook test (ticket 005's ctypes hook ABI): register a write or read
hook that NAKs the APDS probe address and assert `present()` stays false.

## Acceptance Criteria

- [ ] `beginStep()`'s APDS probe path uses `readReg8Status()` and requires
      `ok == true` before concluding presence from the register value.
- [ ] New Python hook test: register a hook that returns a NAK status for
      the APDS probe address (`0x80`/`0x39<<1` per the class's own
      addressing), step the sim, assert `ColorSensorLeaf::present() ==
      false` and that no further probe attempts recur (the perception slot
      skips it, per the issue's own bench-verification wording).
- [ ] Existing color-sensor unit coverage
      (`tests/sim/unit/devices_sensors_harness.cpp`/
      `test_devices_sensors.py` or wherever this leaf's other tests live)
      still passes — the Alt-probe-path and the genuine-present path are
      unaffected by this fix (the issue notes the Alt path "is safe only
      by accident," not that it needs changing).
- [ ] Bench (final acceptance, can be deferred to the sprint's own final
      bench-verification pass if hardware access is not available mid-
      ticket): boot an image with the color sensor unplugged;
      `present()==false`, no recurring bus errors in the I2C diagnostics.

## Implementation Plan

**Approach**: Minimal, surgical fix — do not touch the Alt-probe path or
any other detection logic; only the APDS status-ignoring read changes.

**Files to modify**:
- `source/devices/color_sensor.cpp` (`beginStep()`'s `ApdsProbe` phase)

**Testing plan**:
- New Python hook test (see Acceptance Criteria) — this is the primary
  regression coverage and the reason this ticket depends on ticket 005's
  hook ABI rather than landing in Stage 1.
- Existing: run the color-sensor leaf's existing sim-unit coverage to
  confirm no regression on the genuine-present and Alt-probe paths.
- Verification command: `uv run python -m pytest tests/sim -k color`
  (adjust the `-k` filter to whatever the migrated test file is actually
  named once ticket 009 lands, or target the new hook test directly if it
  is added before ticket 009's broader migration).

**Documentation updates**: none beyond the code comment at the fixed call
site explaining why `readReg8Status()` is required here (a one-line note
pointing at this issue).
