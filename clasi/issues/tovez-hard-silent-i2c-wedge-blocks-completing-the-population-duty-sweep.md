---
status: pending
priority: high
tickets:
- 130-001
---

# tovez went hard-silent (I2C-bus wedge) mid-sweep, blocking completion of the population duty-sweep protocol

## Description

During sprint 130 ticket 001's duty-sweep bench session (2026-08-01),
`tovez` went completely silent -- no `PING` reply, no telemetry -- after 45
good rungs of `src/tests/bench/duty_sweep.py`'s full-range population sweep,
starting at the right-wheel-forward duty=0.293 rung.

### Diagnosis (read-only, no register writes)

```
pyocd commander -u 9906360200052820a8fdb5e413abb276000000006e052820 -t nrf52833 \
  -c "halt" -c "reg pc lr sp"
```
showed the core RUNNING with PC in RAM (`0x20002058`) -- not crashed, not
halted, spinning. A backtrace via

```
arm-none-eabi-gdb -q --batch build/MICROBIT -ex "target remote :3333" \
  -ex interrupt -ex backtrace -ex detach
```

showed it spinning in `codal::system_timer_wait_cycles` / called from
`codal::system_timer_current_time` (`Timer.cpp`) -- an infinite busy-wait.
This matches the project's own documented "silent robot = dead external I2C
bus" signature (banner + a few good telemetry frames, then total silence;
every subsequent reboot re-wedges at the same point), previously root-caused
on 2026-07-25 to a dead motor-brick battery starving the external I2C bus's
pull-ups (`Devices::MicroBitI2CBus::write` -> `NRF52I2C::waitForStop`, IRQs
masked, so even serial DMA stops -- total silence rather than a loud fault
bit).

Reproduced across 3 fresh connection attempts, including one with
`reset=True` (a hard DTR reset through DAPLink). The wedge is NOT a one-off
software glitch -- it recurs on every boot, meaning it is a persistent
condition (bus unpowered / battery dead or critically low), not something a
reflash or soft reset can clear. USB enumeration stayed healthy throughout
(`mbdeploy list` kept showing `tovez` normally at its usual port) -- this is
not a cable, port, or CDC issue.

No register writes were attempted (per `.claude/rules/debugging.md` and the
"never write hardware registers" convention) -- only `halt`/`reg` reads and a
`gdb backtrace`, both non-destructive.

### Impact on ticket 130-001

The population duty-sweep protocol was cut short. Captured and committed:

- A COMPLETE left-wheel characterization (both directions, full 0.04-1.0
  duty range) -- genuinely linear to duty~0.55 (materially higher than
  129-006's ~0.30 estimate for the left wheel; the calibration note argues
  that estimate was itself session/battery-specific, not a fixed ceiling),
  saturating/declining slightly above that.

Missing (blocked by this fault):

- The right wheel's own full range (only 3 low-duty forward rungs captured,
  duty<=0.242; NO reverse data at all).
- The entire simultaneous-both-wheels grid (zero rungs).

The parallel-lines/population verdict and the derived `biasMax`/`vMin`/
breakaway-band constants recorded in `data/robots/tovez.json`'s
`control._drive_calibration_note` (2026-08-01 entry) rest on this incomplete
n=3 sample and are explicitly flagged LOW CONFIDENCE there -- they must not
be treated as final or used to settle ticket 004's Stage C design question
(intercept-only vs slope-dominated adaptation) either way.

## Needed follow-on (stakeholder-scheduled bench session -- physical
intervention required, an agent cannot do this)

1. Check/replace or recharge the motor brick's battery, or otherwise verify
   the external I2C bus is powered, before any further bench work on `tovez`.
2. Re-run `src/tests/bench/duty_sweep.py` once the robot responds normally
   (PING/telemetry) again, to complete the right wheel's full range and the
   simultaneous grid. The tool's `--from-csv` flag can re-run just the
   fit/population analysis against a combined dataset without re-driving
   the already-good left-wheel rungs, if that's convenient.
3. This is the SAME physical session that should also address 129-006's own
   still-open freshly-charged-battery re-run (separating a hard
   power-delivery ceiling from a battery-sag artifact) and the full
   multi-motor population campaign (physically swapping motors) -- both
   already flagged in ticket 130-001's own scope note as
   stakeholder-scheduled follow-ons, now joined by this hardware fault as a
   third, more urgent reason the same session is needed.

## Related

- Sprint 130 ticket 001 (`clasi/sprints/130-wheels-solid-one-wheel-speed-
  controller-in-drive-planner-honesty-pass-one-composition-root/tickets/
  001-population-duty-sweep-protocol-baked-defaults-adaptation-bounds.md`)
- `clasi/sprints/130-wheels-solid-one-wheel-speed-controller-in-drive-
  planner-honesty-pass-one-composition-root/issues/duty-sweep-single-wheel-
  vs-simultaneous-current-limit-129-006.md`
- `data/robots/tovez.json`'s `control._drive_calibration_note`, 2026-08-01
  entry -- full measurement writeup.
