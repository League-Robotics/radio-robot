---
status: resolved
filed: 2026-07-23
filed_by: team-lead (phase-B bench session, v0.20260723.2 on the stand)
related:
- restore-the-interleaved-request-settle-tick-loop-schedule.md
- i2c-safety-net-bit-conflates-otos-settle-wait-with-loop-schedule-health.md
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

## Resolution (120-003, 2026-07-23)

Diagnosed on real hardware via pyOCD/DBG (halted-SWD reads of the raw
`MicroBitI2CBus::clearanceSafetyNetCount_` member, robot "tovez",
`/dev/cu.usbmodem2121102`). Findings:

- The counter is a CONTINUOUSLY LIVE, monotonically growing value — NOT
  a boot-time one-shot latch (falsifying the pre-existing doc claim in
  `telemetry.h`/`src/firm/app/DESIGN.md`, now corrected). Measured
  climbing at every sample point: 97 at ~4.6s post-flash, 167 at ~8s
  post an SWD-triggered reset, Δ243 over a ~14s idle bracket, Δ148 over
  a second, independent ~8.6s idle bracket.
- Root cause, confirmed by an EXACT 1:1 accounting in both idle
  brackets (Δ243 matches half of `Devices::Otos`'s own `txnCount` delta
  of 486; Δ148 matches half of 296): `Devices::Otos::readPositionVelocity()`
  (and its sibling register helpers) issue a register-select `write()`
  immediately followed by a `read()` on the SAME device with NO
  intervening loop-scheduled gap, so `waitForClearance()` trips on
  every single Otos burst read, unconditionally, at Otos's own ~20ms
  cadence — regardless of `moveQueue_.active()` (`Otos::tick()` runs
  every cycle either way, explaining why the bit is set idle AND
  driving identically). `NezhaMotor`'s split-phase
  `requestEncoder()`/`collectEncoder()` path (the thing 118-001's
  `kSettle`/`kClear` restore actually protects) contributes ZERO
  measured trips in either window — the interleave fix is fully
  effective for its real target; it was never going to affect this bit,
  which has nothing to do with the loop schedule.
- **No code fix ships.** Making the bit literally clear during driving
  would require either redesigning `Otos`'s own I2C register-read
  pattern (real hardware-timing risk to a currently-working sensor
  path, out of this ticket's authorized file scope) or a stakeholder
  policy decision about what the fault bit should exclude — filed as
  `clasi/issues/i2c-safety-net-bit-conflates-otos-settle-wait-with-loop-schedule-health.md`
  for a future sprint.
- Corrected records: `src/firm/app/telemetry.h`'s bit 6 doc comment,
  this sprint's design overlay (`design/DESIGN.md` §4), and 118-001's
  own ticket + source issue (both in
  `clasi/sprints/done/118-loop-schedule-truth-firmware-loop-reorder-sim-cadence-parity/`).
