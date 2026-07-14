---
status: obsolete
sprint: 099
---

> **OBSOLETE (2026-07-14 stakeholder triage).** Superseded by the single-loop
> firmware rebuild (`clasi/issues/single-loop-firmware-de-fiber-delete-the-elite-plumbing-telemetry-only-return-path.md`;
> review: `docs/code_review/2026-07-13-devices-drive-review.md`). Motion::SegmentExecutor is deleted (motion/ already removed; drive/ v2 follows in the rebuild's P2). OTOS is already slot-scheduled in DeviceBus, and heading fusion moves host-side (host plans, robot follows). The slip-immunity goal is carried by host-planner-design-lessons-from-drive-v2-review.md item 10.

# OTOS heading source for the SegmentExecutor heading loop (deferred from sprint 098)

Sprint 098 shipped the encoder-heading cascade that makes turns terminate on
target (100% within ±1°). Its Stage-2 ticket (098-004) tried to feed the
heading loop **OTOS (gyro-fused) heading** for slip immunity, with tick-by-tick
encoder fallback. The software was implemented and sim-verified (bit-identical
parity when the pose is invalid), but it **regressed catastrophically on
hardware and was reverted** per the ticket's own revert gate. This issue
captures the remaining work for sprint 099 (the pose-estimation sprint, the
natural home for OTOS on the I2C bus).

## Two blocking findings from the 098-004 hardware attempt (2026-07-12)

1. **The OTOS chip reads `connected=False` on tovez right now.** TLM `otos.h`
   is a constant 0.0 and `otos_connected=False` at rest — the current firmware
   does not detect/initialize the OTOS in the live loop (093/094 stripped OTOS
   ticking from `main.cpp`; the `Hal::OtosOdometer` leaf is constructed and
   `begin()`s but nothing ticked it before 098-004). **099 must restore OTOS
   detection/init first** — there is no OTOS heading to consume until then.

2. **A naive per-pass `odometer()->tick(now)` on the SHARED I2C bus wrecks the
   motion loop.** The OTOS (0x17) shares the bus with the motor-brick flip-flop
   sequencer (0x10). Ticking it every pass — especially a disconnected chip
   doing failing/retrying reads — disrupted the flip-flop cadence and encoder
   sensing so badly the heading loop over-rotated wildly (−90°→−192°, peak wheel
   637 mm/s vs a 384 ceiling). Reverting restored ±0.65° turns immediately.
   **099 must fold the OTOS read into the flip-flop I2C schedule (or make it
   non-blocking / connection-gated) so it cannot perturb the motion timing** —
   see `motor-actuation-latency-flipflop-coupling.md`,
   `.clasi/knowledge/` i2c-irqguard-vs-serial-rx.

## The (reverted) design to revive once the above are fixed

The 098-004 code (reverted in commit `00525ff1`, recoverable from `70d46177`)
was sound in shape — reuse it:
- `Subsystems::Drivetrain::tick()` threads a real `msg::PoseEstimate` into
  `Motion::SegmentExecutor::tick()`'s (defaulted) pose parameter instead of the
  hardcoded `msg::PoseEstimate{}`.
- `SegmentExecutor::measuredHeading()` prefers OTOS heading
  (`pose.pose.h − baseline_.otosHeading0`, a new phase-start baseline mirroring
  `encDiff0`) when `pose.stamp.valid`, else falls back tick-by-tick to the
  encoder-differential heading. `omegaMeasured` stays encoder-derived (OTOS
  heading only, never rate — Decision 4).
- Sim parity (invalid pose → bit-identical to encoder-only) + source-selection
  scenarios already exist in the reverted commit.

**Acceptance (099):** with OTOS detected and its read safely scheduled, a
`turn_sweep.py --relay --both` run shows NO regression vs 098's ±1° baseline
AND no radio/loop-timing symptom; then demonstrate slip immunity in a scenario
where encoder heading and OTOS heading actually diverge (e.g. a deliberately
slipped/scrubbed turn).

Related: `heading-loop-cascade-control-turns-terminate-on-target.md` (parent),
`restore-pose-estimation-otos-encoders-delayed-camera-fixes.md` (sprint 099).
