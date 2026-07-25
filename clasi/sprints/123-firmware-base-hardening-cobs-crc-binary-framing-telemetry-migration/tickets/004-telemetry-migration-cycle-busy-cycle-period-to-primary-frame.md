---
id: '004'
title: Telemetry migration - cycle_busy/cycle_period to primary frame
status: open
use-cases: [SUC-005]
depends-on: ['002', '003']
github-issue: ''
issue: cobs-crc-binary-framing-replace-base64-armor.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Telemetry migration - cycle_busy/cycle_period to primary frame

## Description

Move `cycle_busy`/`cycle_period` (added to
`TelemetrySecondary` as an interim placement by sprint 122 ticket 003)
onto the primary per-cycle `Telemetry` frame, now that COBS+CRC has
restored primary-frame headroom. Resolve Open Question 4 (remove from
secondary, or keep on both) with the stakeholder before finishing this
ticket.

## Acceptance Criteria

- [ ] `cycle_busy`/`cycle_period` present on the primary `Telemetry`
      frame every cycle (proto field addition, `Frame` staging using the
      existing `previousCycleStartUs_`/`everCycled_` bookkeeping —
      unchanged mechanism, only destination frame moves).
- [ ] Sim test asserts exact per-cycle values under the virtual clock
      (mirroring sprint 122 ticket 003's own test, relocated).
- [ ] Host `TLMFrame` and the TestGUI telemetry panel read the fields
      from the primary frame.
- [ ] `TelemetrySecondary`'s copy is removed (or explicitly retained
      with a stated reason) per Open Question 4's resolution — not left
      ambiguous.

## Implementation Plan

- **Approach:** Straightforward field relocation now that ticket 002 has
  freed the headroom; reuses sprint 122's existing bookkeeping without
  change.
- **Files:** `src/firm/protos/telemetry.proto`, `src/firm/app/telemetry.{h,cpp}`,
  regenerated `messages/telemetry.h`/`wire.cpp`, host `TLMFrame` decode,
  TestGUI telemetry panel display line.
- **Testing:** Sim unit test (exact virtual-clock values); TestGUI manual
  check that the panel line still displays.
- **Documentation:** `app/DESIGN.md`'s "122-003" interim-placement note
  updated to reflect the completed migration, ride ticket 005.
