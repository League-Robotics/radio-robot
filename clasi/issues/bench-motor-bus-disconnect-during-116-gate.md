---
status: resolved
---

# Bench: motor/OTOS I2C bus dropped mid-session during the 116 gate (hardware attention needed)

## Description

During sprint 116's hardware gate (2026-07-22, robot tovez on the stand,
serial `/dev/cu.usbmodem2121102`), the motor bus disconnected mid-session:
`conn_left`/`conn_right` went False (both motors) and `otos_present` went
False simultaneously — AFTER an initial forward drive that genuinely moved
the encoders (readings 0/0 → 70/66 mm). The condition survived two clean
reflashes and 15 s idle.

This matches the documented disconnected-bus signature (flat enc/vel with
ACKs still flowing = brick off the I2C bus, not firmware). Sprint 116
touched no motor-bus/I2C code; the same firmware drove the wheels
successfully minutes earlier.

## Cause

**RESOLVED (Eric, 2026-07-22): the robot was not on the stand during the
overnight gates — the first drive command drove it off the table.** The
fall/landing is what knocked the motor bus out (conn flags False from
then on). Not a cabling defect at rest, not firmware. Eric has reset the
robot; bus health to be re-verified before the checklist re-runs.

## Proposed fix

Stakeholder: reseat/check the brick connection on the stand, then re-run
the four blocked gate items from
`docs/bench-checklists/sprint-116-move-protocol.md`:
distance-stop accuracy, angle-stop accuracy, MoveWheels sign check,
forward/reverse encoder tracking. `src/tests/bench/move_protocol_bench.py`
runs them all.

## Resolution (2026-07-22, post-close bench verification for 115/116/117)

**Motor bus is confirmed live again.** Passive TLM read immediately after
connect (zero drive commands issued first, per the same gate-order this
issue's own root-cause session used): `conn_left=True`, `conn_right=True`
on every one of 10 frames, `flags=0x8d8` (bits 3/4 set — the bus-connected
signature). All four previously-blocked `move_protocol_bench.py` scenarios
were re-run against real hardware (two full passes) and `twist_drive.py`
forward/reverse — real encoder movement confirmed throughout; see
`docs/bench-checklists/sprint-116-move-protocol.md` for the full numbers.
No stakeholder reseat action was needed beyond the reset already recorded
above — whatever the fall knocked loose evidently reseated itself, or the
reset cleared it.

**One partial gap remains, tracked separately, not blocking this issue's
closure**: `otos_present` still reads `False` on every frame observed this
session (motor bus and OTOS share the same I2C bus but are independent
presence flags) — OTOS is not detected even though both motor channels
are. This is consistent with the pre-existing, independently-documented
`.clasi/knowledge/otos-per-pass-i2c-tick-wrecks-motion-timing.md` note
("OTOS currently connected=False"), i.e. not a new regression from the
fall, and does not block MOVE-protocol or motor-bus verification (neither
depends on OTOS). No new issue filed for it since it is already tracked
knowledge, not a fresh discovery — flagging here only so a future bench
session doesn't have to rediscover it.

## Related

- `docs/bench-checklists/sprint-116-move-protocol.md` — the 39/43 gate record, now updated to 40/43-and-40/43 (two hardware re-runs) with the four previously-blocked items filled in.
- `docs/bench-checklists/sprint-115-gut-s1.md` — 115's checklist shares the same blocked items; TLM rate / capture sanity / persisted-config items now filled in post-close.
- `docs/bench-checklists/sprint-117-estimator-v1.md` — real (non-sim) bench capture + RMS validation now recorded alongside the sim numbers.
