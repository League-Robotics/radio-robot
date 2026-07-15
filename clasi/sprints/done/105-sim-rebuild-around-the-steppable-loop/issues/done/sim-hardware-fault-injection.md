---
status: done
sprint: '105'
tickets:
- 105-005
---

> **RETARGETED (2026-07-14 stakeholder triage).** SimMotor/SimHardware and
> the current host sim harness are deleted by the single-loop rebuild; the
> ASK survives unchanged as a design requirement for the rebuild's P7 sim
> (a thin steppable-loop sim over the devices layer's HOST_BUILD fakes,
> whose scripted I2CBus can natively fake NAKs, stale reads, and wedge
> latch-ups — a better fault-injection seam than SimMotor ever was).

> **DELIVERED (2026-07-15, ticket 105-005).** Disconnect, wedge, and dropout
> are built against `tests/sim/plant/wheel_plant.{h,cpp}`'s three new
> knobs — `WheelPlant::setDisconnected(bool)`, `WheelPlant::freezePosition
> (bool)`, `WheelPlant::setDropoutRate(float)` — each changing only HOW
> `WheelPlant::scriptEncoderResponse()` scripts its next `Devices::I2CBus`
> response (never `step()`'s own duty→velocity→position integration), the
> exact "thin steppable-loop sim over the devices layer's HOST_BUILD
> fakes" seam this issue's retargeting called for. `TestSim::SimApi`
> exposes them via `plantLeft()`/`plantRight()` accessors
> (`tests/sim/support/sim_api.h`). Proven end-to-end, against the FIRMWARE's
> own observable reaction in decoded telemetry (not just the plant/leaf in
> isolation), by three pytest scenarios in
> `tests/sim/system/faults/fault_knobs_harness.cpp` +
> `test_fault_knobs.py` (`uv run python -m pytest tests/sim/system/ -k
> fault -v`):
>   - **Motor disconnect** — `connLeft`/`connRight` flip false in decoded
>     telemetry while the knob is active on one motor only, and recover to
>     true once cleared (`Devices::NezhaMotor::connected()` is recomputed
>     fresh every `collectEncoder()` call, never latched, so no separate
>     "reconnect" step is needed).
>   - **Encoder wedge** — `kFaultWedgeLatch` sets in decoded telemetry
>     within `Devices::MotorArmor`'s own `kWedgeThreshold` (10 consecutive
>     unchanged reads) while driving, and — set AND clear semantics,
>     `robot_loop.cpp`'s own live `tlm_.setFault(kFaultWedgeLatch,
>     motorL_.wedged() || motorR_.wedged())` call, never a one-shot latch at
>     the wire level — clears again once the knob releases and the reported
>     position catches up to the plant's own live position (which kept
>     advancing underneath the whole time).
>   - **Encoder dropout** — at a 25% hold rate, decoded telemetry shows no
>     false `kFaultWedgeLatch` and `velLeft` never starves toward zero,
>     matching the freshness-gate contract
>     `devices_motor_harness.cpp` scenario 8 already proves in isolation for
>     the leaf alone, now driven through the full loop.
>
> **OTOS staleness/warn-bit injection remains deferred** — the firmware
> does not fuse OTOS at all yet (`App::Odometry`'s own file header: "no
> pose fusion happens here... the robot does not fuse"), so there is no
> firmware reaction to verify against. Revisit once host-side fusion (106+)
> exists.

# Sim hardware fault injection — disconnect, wedge, encoder dropout

## Problem

The host-side simulation harness (see
`clasi/issues/host-side-simulation-environment-for-the-new-tree-design-write-up.md`)
ships v1 with healthy hardware only: every `SimMotor` reports `connected() == true`
and `wedged() == false`, and the OTOS always returns a fresh pose. The firmware's
*fault-handling* paths — how it reacts to a motor dropping off the bus, an encoder
that sticks at a stale value (the wedge/latch family), a sensor that stops updating —
therefore have no deterministic, off-hardware way to be exercised. Today those paths
can only be provoked on the bench, unreliably (you cannot make real hardware wedge on
command), which is exactly why the wedge saga took as long as it did to root-cause.

## Why this is worth capturing

A simulator's highest-leverage capability is injecting faults that real hardware
won't produce on demand. The encoder wedge specifically
(`docs/knowledge/2026-07-04-encoder-wedge.md`, and the `later/`
`encoder-wedge-corrupts-tour-legs.md` issue) is a stale-readback failure the firmware
must detect and recover from; a deterministic in-sim wedge would turn "reproduce it on
the bench and hope it triggers" into a fast, repeatable regression test.

## Sketch (not a v1 commitment)

Follow-on ctypes-backdoor knobs on the sim devices (no wire surface — same rule as the
rest of the sim's error knobs):

- **Motor disconnect** — force `SimMotor::connected()` to false for a named port; verify
  the firmware's connected-gating and any DEV/telemetry reporting.
- **Encoder wedge / stuck value** — freeze a `SimMotor`'s reported encoder at its
  current value (or an injected one) while the plant keeps moving, reproducing the
  boundary-latch flavor; verify the wedge detector fires and recovery unfreezes it.
- **Encoder dropout** — drop a fraction of encoder samples (read returns "no new data")
  to exercise the read-failure / outlier-filter recovery paths (cf. sprint 064).
- **OTOS staleness / warn bits** — hold the OTOS pose stale or assert a warn flag to
  exercise the fusion gate and health reporting (cf. sprints 065/074), once firmware
  fusion consumes the OTOS at all.

## Preconditions / when to pick this up

- The v1 sim harness (motors + OTOS + plant + C ABI + Python fixtures) must exist first.
- Most valuable once there are firmware consumers whose fault-reactions are worth
  regression-testing — the wedge detector already exists; OTOS-health reactions arrive
  with fusion. Revisit when either becomes a priority.

Deferred by stakeholder decision 2026-07-04 (v1 sim scope excludes fault injection);
filed so the capability is not lost.
