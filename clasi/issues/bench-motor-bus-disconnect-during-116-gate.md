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

Physical/electrical, on the bench: brick power or I2C cabling to the
Nezha brick + OTOS (they share the bus). Needs stakeholder eyes/hands on
the stand. (Per project rule: this is a bus-connectivity observation, not
a power/battery attribution.)

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
