# Polling telemetry during a move stops the robot dead (radio relay)

**Status:** open, root cause unknown
**Measured:** 2026-08-13, tovez on battery, `getez` relay, camera truth
**Severity:** high for tooling — it makes a healthy robot look broken

## The measurement

Interleaved A/B, camera-supervised, fenced, aiming back to field centre before
every leg so position cannot confound it. Identical command both arms:
`move_twist(140, 0, 0, stop_distance=200)`.

| arm | legs (of 200 mm commanded) | mean |
|---|---|---|
| quiet — nothing sent on the robot link while moving | 197, 197, 198 | **197.5 mm** |
| `proto.tlmNow()` polled at 5 Hz while moving | 0, 0, 1 | **0.3 mm** |

3/3 each way, alternating. The move does not merely slow down — it never
activates: `kFlagActive` (bit 2) was never set in 75 frames of a polled move,
and the encoder delta is exactly `(0, 0)`.

Camera polling is irrelevant (it goes to the aprilcam daemon, not the robot
link) and is used for the fence in both arms.

## Why it matters

It presents as a dead robot. It cost most of a session on 2026-08-13: a
commanded 300 mm leg measured 11 mm of travel, `vel=(0,0)`, encoders frozen at
a byte-identical value across minutes — and that was read as a hardware fault
(latched `kFlagFaultWedgeLatch`, "the robot is wedged, power-cycle it"). It was
not. The standing gate `src/tests/bench/twist_drive.py`, which polls politely,
passed **6/6** on the same robot minutes later, and a quiet 300 mm leg measured
294.8 mm.

Any bench or playfield script that samples telemetry *during* a move over the
relay is measuring a robot that is not moving because it is being sampled.

## What is NOT the explanation

- Not the wedge latch. `kFlagFaultWedgeLatch` is reported only —
  `robot_loop.cpp:598` copies `motorL_.wedged() || motorR_.wedged()` into
  telemetry, and nothing in `control/` or `core/` gates motor output on it.
- Not `kFlagFaultI2CSafetyNet` (bit 6). Per `core/DESIGN.md:1118` that is a
  continuously live, monotonically growing counter — measured at 97 within
  4.6 s of flashing a *healthy* tovez. It is normal background, not a fault.
- Not motor-bus connectivity. `connL`/`connR` are backed by a real transaction
  (`nezha_motor.cpp:284`, `connected_ = (writeResult == kOk && readResult == kOk)`)
  and both read YES throughout.
- Not battery, not the field edge (the fenced re-run above controls for both).

## Hypotheses, untested

1. The inbound `TLM:NOW` traffic collides with the outbound MOVE frame in the
   relay's TX path, so the MOVE is dropped — consistent with the move never
   activating. A 0.5 s quiet window immediately after the command appeared to
   restore motion in one unfenced trial (202.8 mm) but that run was
   edge-contaminated and is not trustworthy; **re-measure this fenced** — it is
   the cheapest discriminator, and if true the fix is host-side pacing, not
   firmware.
2. Command processing preempts or clears the planner queue.
3. Relay half-duplex starvation at any inbound rate.

## Not established

Whether this reproduces over **direct USB**. A 500 mm stability probe polling
at 20 Hz over the *gauti* USB bridge on 2026-08-12 did show normal motion
(4.86 s, peak 156 mm/s), which suggests the effect is specific to the radio
path — but that was a different firmware build and a different transport, and
no controlled USB A/B has been run.

## Reproduce

`src/tests/bench/`-style script; the working copy used is
`poll_ab.py` (scratchpad, 2026-08-13). Recentre first — starting outside the
fence aborts every leg and looks like the same symptom for a different reason.
