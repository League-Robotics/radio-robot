---
status: pending
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

## Related

- `docs/bench-checklists/sprint-116-move-protocol.md` — the 39/43 gate record.
- `docs/bench-checklists/sprint-115-gut-s1.md` — 115's checklist shares the same blocked items.
