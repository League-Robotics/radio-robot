---
source_file: design.md
source_hash: db86ddcae0ed9e27d4f34c8bed8a41921709a9074402b2e911f0815887818897
---
# Diff: design.md

Comparison of the sprint overlay copy of `design.md` against its pristine (seed-commit) canonical version.

```diff
--- design.md (pristine)
+++ design.md (current)
@@ -17,11 +17,11 @@
 commands and streams telemetry, and a Python host package that talks to
 it over USB serial or a radio relay, with a parallel host-build
 simulator for development without hardware. The current architecture
-(post sprint 116, "MOVE protocol cutover", on `master`) is deliberately
-minimal — the firmware speaks exactly three inbound commands (**MOVE /
-CONFIG / STOP**): `Move` carries its own velocity (twist or wheels
-variant), a stop condition (time/distance/angle), and a required
-`timeout` backstop, queued 1-active + 4-pending — and emits one
+(post sprint 117, "predict-to-now estimator v1", on `master`) is
+deliberately minimal — the firmware speaks exactly three inbound
+commands (**MOVE / CONFIG / STOP**): `Move` carries its own velocity
+(twist or wheels variant), a stop condition (time/distance/angle), and a
+required `timeout` backstop, queued 1-active + 4-pending — and emits one
 telemetry frame (**frame v2**: per-wheel `EncoderReading`/`OtosReading`
 with their own sample times, a single `flags` bit-string, a single ack
 slot, packed line/color words) every 20 ms cycle. There is still **no**
@@ -32,6 +32,10 @@
 command rather than reviving it. Every motion is now structurally
 self-bounding (its own stop condition or timeout), which supersedes the
 deadman it replaces — there is no `App::Deadman` anywhere in this tree.
+Sprint 117 adds `App::StateEstimator`, a passive predict-to-now module
+that extrapolates wheel/body state from the same telemetered readings —
+it does not yet drive motion (the trajectory controller that will
+consume it is a later sprint, gated on this one being bench-proven).
 
 The host side (`src/host/robot_radio/`) still carries the code that was
 built against the pre-115 motion stack — tour/path/navigation planning —
@@ -57,7 +61,7 @@
 
 | Subsystem | Role |
 |---|---|
-| [`app/`](../../src/firm/app/DESIGN.md) | The single cooperatively-timed control loop (`App::RobotLoop`) and its passive modules: Comms, Telemetry, Drive, Odometry, MoveQueue, Preamble. |
+| [`app/`](../../src/firm/app/DESIGN.md) | The single cooperatively-timed control loop (`App::RobotLoop`) and its passive modules: Comms, Telemetry, Drive, Odometry, MoveQueue, StateEstimator, Preamble. |
 | [`com/`](../../src/firm/com/DESIGN.md) | ARM-only raw transports: USB CDC serial, the micro:bit radio, persisted radio-channel storage. |
 | [`config/`](../../src/firm/config/DESIGN.md) | Generated boot configuration — per-robot calibration baked at build time from `data/robots/active_robot.json`. |
 | [`devices/`](../../src/firm/devices/DESIGN.md) | I2C-attached device leaves (Nezha motors, OTOS, color/line sensors), the shared `MotorArmor` policy, the velocity PID, and the pure `I2CBus`/`Clock`/`Sleeper` hardware seams. |
@@ -265,6 +269,29 @@
 [`src/firm/motion/DESIGN.md`](../../src/firm/motion/DESIGN.md) for the
 full detail.
 
+**117 (predict-to-now estimator v1) — landed.** A new passive `app/`
+module, `App::StateEstimator`, ticks once per cycle (trailing `kPace`
+block, after OTOS sampling and odometry integration) reading the SAME
+`Frame` data `Telemetry` already stages — no new on-chip measurement
+storage, no bus access of its own. It holds per-wheel and body state as
+PEER estimates (each independently valid/stale), extrapolated
+zero-order-hold ("predict to now": `distance = basis.position +
+basis.velocity × age`, generalizing the deleted `HeadingSource::
+headingLead()` equation to the full body pose) plus a v1 complementary
+blend against OTOS heading/omega whose weights are fail-closed baked
+config, defaulting to 0.0 (encoder-only output this sprint, per
+stakeholder decision) and live-tunable via a new `ConfigDelta.estimator`
+(`EstimatorConfigPatch`) oneof arm, mirroring `OtosConfigPatch`'s
+existing merge-then-apply pattern — NOT persisted to flash (unlike motor
+gains/OTOS calibration; a reboot reverts to the baked default). The
+estimator's predictions are NOT exposed on the wire this sprint —
+validation (leave-one-out one-step-ahead RMS analysis) runs host-side
+directly against the raw `EncoderReading`/`OtosReading` fields sprint 115
+already telemetered, via a captured TLM-log CSV, not a live query
+against the on-chip estimator instance. See
+[`src/firm/app/DESIGN.md`](../../src/firm/app/DESIGN.md) for the full
+detail.
+
 Flow of one cycle, at orientation altitude:
 
 1. **Comms in** — `App::Comms` polls the two transports (serial, radio)
@@ -286,8 +313,10 @@
    bounded work (OTOS sampling, odometry integration, telemetry
    assembly).
 4. **State out** — `App::Odometry` integrates encoder deltas through
-   `BodyKinematics::forward()`; `App::Telemetry` emits the primary TLM
-   frame (or the slower secondary diagnostic frame) through Comms.
+   `BodyKinematics::forward()`; `App::StateEstimator` (117) ingests the
+   same cycle's staged `Frame` and refreshes its wheel/body ZOH
+   predict-to-now estimates; `App::Telemetry` emits the primary TLM frame
+   (or the slower secondary diagnostic frame) through Comms.
 5. **Pace** — a final `runAndWait` paces the cycle to `kCycle` = 20 ms
    (~50 Hz), matching `Telemetry::kPrimaryPeriod` so every cycle emits a
    primary frame.
@@ -380,7 +409,9 @@
 has landed — `Twist` (arm 19) and `ConfigDelta.watchdog` (field 4) are
 `reserved`, not reused; `App::Deadman` is deleted; see
 [`src/firm/motion/DESIGN.md`](../../src/firm/motion/DESIGN.md) for the
-new `Motion::StopCondition` module.
+new `Motion::StopCondition` module. Sprint 117's `App::StateEstimator`
+has landed — see [`src/firm/app/DESIGN.md`](../../src/firm/app/DESIGN.md)
+for its full boundary/interface detail.
 
 ## 6. Open Questions / Known Limitations (system-level)
 
@@ -393,6 +424,17 @@
   `move_twist()`/`move_wheels()` builders; reviving the higher-level
   tour/nav machinery onto the new wire surface is explicit future work,
   not part of 116.
+- **Sprint 117 (predict-to-now estimator v1) has landed.**
+  `App::StateEstimator` ticks every cycle with wheel/body peer ZOH
+  estimates; its OTOS-fusion weights are fail-closed baked config,
+  defaulting to 0.0 (encoder-only v1) and live-tunable via the new
+  `ConfigDelta.estimator` arm — NOT persisted to flash. Its predictions
+  are not exposed on the wire; validation runs host-side against the raw
+  telemetered readings (a captured TLM-log CSV), per the stakeholder's
+  leave-one-out one-step-ahead RMS methodology. Fake OTOS, external/
+  camera pose fusion, and the remaining-distance trajectory controller —
+  the source issue's further-out goals — remain future work, not part of
+  117.
 - **The design-doc-set's mechanical validator cannot express "this
   child is out of scope because it symlinks outside the repository."**
   `src/vendor` remains permanently undocumented for that reason (§4).
```
