---
status: pending
sprint: '114'
---

# devices/ naming sweep: move units out of identifiers into // [unit] tags

From the 2026-07-13 code review (docs/code_review/2026-07-13-devices-drive-review.md,
Part 1 STYLE section). `source/devices/` is KEPT by the single-loop rebuild, so these
violations of the project naming rule (.claude/rules/coding-standards.md: units never
appear in any identifier; units go in a leading bracketed `// [unit]` comment tag)
survive it. Mechanical rename sweep — batch-dispatch style, bulk regex, per-ticket
commits.

## Violations found (systematic across devices/)

- **Methods:** clock.h:43 `nowMicros()`, :47-48 `setMicros()/advanceMicros()`, :67
  `sleepMillis()`; i2c_bus.h:281 `clockUs()`; otos.h:380 `writePoseMm()`.
- **Constants:** device_bus.h `kPowerSettleMs`, `kPreambleRetryPacingMs`,
  `kOtosBeginRetryPacingMs`, `kEncoderSettleMs`, `kCyclePaceMs`, `kVelocityStaleUs`;
  nezha_motor.cpp `kMinWriteIntervalUs`, `kDelayUs`; otos.h:305-306
  `kPosMmPerLsb`/`kHdgRadPerLsb`.
- **Members:** nezha_motor.h `lastTickUs_/lastFreshUs_/lastWriteTimeUs_`; handles.h
  `velocityStagedUs_`; otos.h `lastReadUs_`; color_sensor.h/line_sensor.h
  `lastAttemptUs_/lastReadUs_`; i2c_bus_host.cpp:42 `g_fakeClockUs`.
- **Parameters:** pervasive `nowUs` (nezha_motor.h:132, otos.h:164/189, color_sensor.h,
  line_sensor.h, device_bus.h:323-326), `periodUs` (line_sensor.cpp:72,
  color_sensor.cpp:84).

Name the quantity (`now`, `settle`, `pace`, `stale`, `writeInterval`, `posPerLsb` …) with
the unit in the trailing tag. Everything else in devices/ already conforms (case rules,
trailing underscores, tag usage), so this is a contained sweep.

## Sequencing

Do this AFTER the single-loop rebuild's P2/P3 land (the rebuild deletes some of these
files' machinery and renames survivors' contexts) — sweeping first would churn code that
is about to be restructured. Fold into the rebuild's port work if convenient.
