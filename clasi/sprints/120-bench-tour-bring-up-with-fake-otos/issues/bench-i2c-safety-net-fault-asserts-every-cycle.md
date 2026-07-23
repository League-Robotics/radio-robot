---
status: in-progress
filed: 2026-07-23
filed_by: team-lead (phase-B bench session, v0.20260723.2 on the stand)
related:
- restore-the-interleaved-request-settle-tick-loop-schedule.md
sprint: '120'
tickets:
- 120-003
---

# I2C safety-net fault (flags bit 6) asserts every cycle on hardware — interleave fix did not clear it

## Observed (real hardware, v0.20260723.2, robot on the stand)

`flag_fault_i2c_safety_net` (flags bit 6) is set on **every** telemetry frame,
**idle AND driving** (idle 45/45 frames, fwd-drive 23/23). Contrast:
`flag_fault_wedge_latch` mostly CLEARS during a drive (2/23) but is set at idle
(45/45) — so these are NOT both just latched-since-boot; the i2c_safety_net bit
is genuinely re-asserting each cycle.

This directly contradicts the interleave issue's own before/after acceptance
signal (`restore-the-interleaved-request-settle-tick-loop-schedule.md`,
Verification step 4): "confirm the I2C clearance safety-net fault bit is now
clear while driving — it was tripped every cycle by the blocking in-tick()
settle." The 118-001 restore (kSettle=4/kClear=4) was predicted to clear it. It
did not. (The issue text mislabeled it bit 0; it is bit 6, `kFlagFaultI2CSafetyNet`.)

## Candidate causes (INVESTIGATE — do not assume)

- **OTOS on the shared I2C bus.** Phase B confirmed `otos_present` AND
  `otos_connected` are True every frame — the real OTOS is live on the I2C
  bus (refuting the "servo port" premise elsewhere). The interleave issue was
  written assuming a servo-port OTOS, i.e. NO extra I2C traffic. The OTOS
  burst read in the pace block may be what trips the clearance safety net now.
- The 40ms DutyPredictor/write-throttle margin interaction (119-005 set the
  throttle to 35ms).
- The fix genuinely being incomplete for the real bus schedule.

Use pyOCD/DBG (see .claude/rules/debugging.md) to catch where
`MicroBitI2CBus::waitForClearance()` trips, idle vs driving, with the OTOS
tick present vs skipped.

## Priority

Medium-high. Not obviously a motion defect (all motion behaviors pass), but a
fault bit asserting every cycle is either a real bus-timing problem or a
false-positive that makes the fault channel useless. Resolve which. Whichever
it is, the interleave issue's acceptance claim must be corrected to match
reality.
