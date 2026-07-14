---
status: obsolete
---

> **OBSOLETE (2026-07-14 stakeholder triage).** Superseded by the single-loop
> firmware rebuild (`clasi/issues/single-loop-firmware-de-fiber-delete-the-elite-plumbing-telemetry-only-return-path.md`;
> review: `docs/code_review/2026-07-13-devices-drive-review.md`). Actuation remains bus-scheduled by design in the single-loop rebuild (a clean decouple hung the bus, per knowledge); the surviving substance — model the ~120-140ms latency explicitly and tune against it — is carried by host-planner-design-lessons-from-drive-v2-review.md item 8.

# Motor actuation lags command by ~80–160 ms — actuation is coupled to encoder sensing on the shared I²C brick

## Status: needs a real fix — interim mitigation (transport delay built into the trajectory plan) shipped on `spike/093-presolved-decel-to-zero`

Measured in the `wheel_motion_trace` notebook (093): there is a **~80–160 ms
gap between a drive command and the wheel actually breaking loose**. That is
4–8 full 20 ms control loops of dead time before any torque reaches the wheel.
Eric's verdict: "this is stupid — we're going to fix it." This issue carries
the real fix forward. In the meantime the delay is compensated *in the plan*
(see "Interim mitigation" below), which corrects where/when the motion ends
but does NOT remove the dead time itself.

## Root cause — actuation cadence is the sensing cadence

The Nezha brick is one I²C device (`0x10`) shared by all four motor channels.
`NezhaHardware::tick()` (`source/subsystems/nezha_hardware.cpp`) is a
**flip-flop sequencer**: it does exactly ONE bus phase per 20 ms tick —
`REQUEST_DUE` (write the `0x46` encoder-request) on one tick, `COLLECT_DUE`
(read the 4-byte encoder response, then run the full `NezhaMotor::tick()`
5-step contract — including the duty write) on the next. With two polled
wheels that is **2 ports × 2 phases × 20 ms = ~80 ms** between successive duty
writes to a given wheel, and up to ~160 ms of worst-case phase alignment
between issuing `setVelocity()` and the corresponding `0x60` duty write
actually going out.

Because the duty write only happens inside the `COLLECT_DUE` phase, **motor
actuation runs at the encoder-sampling cadence** even though the PID + duty
write need no encoder transaction of their own and could run every 20 ms.

## Why the obvious fix (decouple actuation from sensing) is blocked

Attempted on this branch (`spike/093`): split `NezhaMotor::tick()` into
`sampleTick()` (sense: collect encoder → velocity) and `controlTick()`
(actuate: run mode → armored duty write), then run `controlTick()` for every
polled port **every 20 ms tick**, leaving only `sampleTick()` in the flip-flop
COLLECT phase.

**This hung the firmware** (0/6 PINGs; required revert + reflash). Root cause:
the encoder read is **split-phase on the same device** — `0x46` REQUEST on one
tick, the paired read on the next — and *nothing else may touch `0x10` between
those two*. Running `controlTick()` every tick injects a `0x60` duty write
*between* a pending encoder REQUEST and its COLLECT, corrupting the read and
wedging the I²C bus → the main loop blocks on the bus and stops servicing
serial/radio. The careful REQUEST→COLLECT pairing is the same sequencing that
protects against the encoder-wedge → motor-runaway failure
([[i2c-irqguard-vs-serial-rx]], [[encoder-wedge-boundary-latch]]), so it
cannot simply be interleaved.

## Viable directions (none yet built/verified)

1. **Pipeline both ports.** REQUEST port1 + REQUEST port2 on one tick, COLLECT
   port1 + COLLECT port2 (each: read → duty write) on the next → per-wheel
   control cadence ~40 ms instead of ~80 ms, and no write ever lands between a
   REQUEST and its own COLLECT. Open question to bench-test: does the brick
   hold two per-`motorId` encoder requests pending simultaneously?
2. **Write duty at both safe points** — issue the duty write at the top of the
   REQUEST phase *and* after the COLLECT read (the two points where no encoder
   read is in flight), roughly halving actuation latency with a minimal
   sequencing change.
3. **Non-split (atomic) encoder read** — if the `0x46` value can be read in one
   REQUEST+read transaction, each port collapses to one tick and the duty write
   decouples cleanly. Reverses the 079-004 split-phase design; needs the wedge
   protection re-proven.

Whichever is chosen must be bench-verified against reversal stress (do NOT
reintroduce the wedge/runaway) and must not disable `I2CBus::_irqGuard`.

## Interim mitigation (shipped on this branch)

The trajectory planner (`source/subsystems/planner.cpp`) now models this
transport dead time so the plan begins its terminal decel early enough that
the lagging plant reaches the target rather than overshooting — Eric's "if you
know you've got a 40 ms delay, build it into your plan." This is a compensation,
not a cure: the wheel still starts late; the plan just accounts for it.

## Related

- [[d-t-turn-terminal-reverse-stakeholder-decision]] — the terminal
  reverse-creep this latency interacts with; the reversal armor MUST stay.
- [[i2c-irqguard-vs-serial-rx]] — the same shared-bus IRQ-masking that couples
  I²C timing to serial RX drops; another symptom of everything sharing `0x10`.
