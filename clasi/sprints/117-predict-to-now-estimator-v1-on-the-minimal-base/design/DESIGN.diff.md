---
source_file: DESIGN.md
source_hash: e980d65b2634496b4c5139abec9333a20ecfbeb1b7ababa20533a16c8c33679a
---
# Diff: DESIGN.md

Comparison of the sprint overlay copy of `DESIGN.md` against its pristine (seed-commit) canonical version.

```diff
--- DESIGN.md (pristine)
+++ DESIGN.md (current)
@@ -19,7 +19,10 @@
 * `Odometry` (dead reckoning, plus cumulative path length),
 * `MoveQueue` (the 1-active + 4-pending bounded-motion queue — every
   `Move` is self-bounding by construction, so there is no separate
-  staleness gate), and
+  staleness gate),
+* `StateEstimator` (117 — predict-to-now wheel/body peer estimates,
+  zero-order-hold extrapolation, v1 complementary blend against OTOS),
+  and
 * `Preamble` (boot-time device detection).
 
 This is the seam that owns the robot's *timing* — every I2C
@@ -59,6 +62,33 @@
 `Motion::Executor`) per active `Move`. The command surface is now
 MOVE+STOP+CONFIG{motor,otos} — no deadman.
 
+**117 (predict-to-now estimator v1) — landed.** A new passive module,
+`App::StateEstimator` (`app/state_estimator.{h,cpp}`), is added
+alongside `Odometry` — NOT a replacement for it: `Odometry`'s dead-
+reckoned `x_`/`y_`/`theta_` still feed `frame_.pose` exactly as before,
+and `StateEstimator` reads that same per-cycle `Frame` data as an
+independent, additive consumer, greenfield in the same sense
+`StateEstimator`'s own source issue used for the deleted `Pilot`/
+`HeadingSource` era: it does not yet drive motion (no consumer wires its
+output into `Drive`/`MoveQueue` this sprint — that is the later
+trajectory-controller sprint, gated on this one being bench-proven).
+`StateEstimator` holds per-wheel and body state as PEER estimates (each
+independently valid/stale), computes zero-order-hold "predict to now"
+extrapolation from the newest basis reading — generalizing the deleted
+`HeadingSource::headingLead()` equation (`heading = basis.heading +
+basis.omega × age`) to the full body pose (x, y, heading, v_x, v_y,
+omega) — and blends a v1 complementary weight against OTOS heading/omega
+whose weights are fail-closed baked config (`Config::
+defaultEstimatorConfig()`), defaulting to 0.0 (encoder-only output this
+sprint, per stakeholder decision) and live-tunable via a new
+`ConfigDelta.estimator` (`EstimatorConfigPatch`) arm dispatched by
+`RobotLoop::handleConfig()`, mirroring `OtosConfigPatch`'s existing
+merge-then-apply pattern — see §3/§4 below for the full detail. Pure
+computation: never touches the I2C bus, never sleeps, no `Devices::
+Clock&` collaborator of its own (every query takes an explicit `now`/`t`
+argument, mirroring `Motion::StopCondition`'s "hand-fed readings, no
+owned collaborator" shape — see that module's own file-header precedent).
+
 ## 2. Orientation
 
 `RobotLoop` has two phases. `boot()` steps `Preamble` until every device
@@ -68,9 +98,10 @@
 for the left motor, decode at most one inbound command (`Comms::pump`),
 apply it (`processMessage`), request/settle/collect/PID for the right motor,
 the unconditional `moveQueue_.tick(now, odom_)` call, then a trailing block that samples OTOS, integrates
-odometry (`Odometry::integrate`), polls line/color at a rate-limited,
-alternating cadence (`updateLineColor()` — see below), and paces the whole
-cycle. `Telemetry::emit()` is called once per cycle and decides for itself
+odometry (`Odometry::integrate`), refreshes `App::StateEstimator`'s
+predict-to-now estimates from that same cycle's staged `Frame` (117 —
+see below), polls line/color at a rate-limited, alternating cadence
+(`updateLineColor()` — see below), and paces the whole cycle. `Telemetry::emit()` is called once per cycle and decides for itself
 whether to send the primary frame, the secondary diagnostic frame, or (on a
 tie) alternate between them. `Drive`, `Odometry`, and `MoveQueue` are pure,
 bounded, non-bus-touching helpers that `RobotLoop` calls at specific points
@@ -94,6 +125,23 @@
 explicitly cleared the same cycle (it was not even touched), matching the
 wire spec's "line/color word fresh" (fresh THIS frame, not merely "known at
 some point") semantics.
+
+**Predict-to-now estimation (`RobotLoop`'s `StateEstimator::update()`
+call, 117).** Runs once per cycle from the trailing `kPace` block,
+immediately after `frame_.pose` is staged (i.e. after
+`applyOtosSample()` and `odom_.integrate()` — the same position this
+sprint's source issue specified as "after applyOtosSample()/
+odom_.integrate(), before pilot_.plan()"; `Pilot` no longer exists, so
+this is simply the end of that block). `update(frame, now)` reads
+`frame.encLeft`/`frame.encRight` (position, velocity, their own collect
+`time`) to refresh each wheel's peer `WheelEstimate` basis, and
+`frame.pose`/`frame.twist` (already fused by `Odometry`/
+`BodyKinematics::forward()` earlier the same cycle) plus `frame.otos`/
+`frame.otosPresent` (when fresh) to refresh the body peer's
+`BodyEstimate` basis via the v1 complementary blend. Pure computation
+over already-staged data — no I2C access, no sleep, bounded work, same
+posture `Odometry::integrate()` and `applyOtosSample()` already keep in
+this same block.
 
 ## 3. Constraints and Invariants
 
@@ -146,13 +194,14 @@
   alternating. There is still no full 3-way round-robin abstraction
   (otos|line|color) — each sensor is its own bounded step, not a unified
   scheduler class.
-- **Config patches cover `MotorConfigPatch` and `OtosConfigPatch`
-  (109-004) only.** `RobotLoop::handleConfig` replies `ERR_UNIMPLEMENTED`
-  for `DRIVETRAIN`/`WATCHDOG`/`NONE` (`DrivetrainConfigPatch` has no
-  on-robot fusion consumer). `PlannerConfigPatch` is GONE, not merely
-  out of scope — 115-005 (gut S1) deleted the type and `ConfigDelta`'s own
-  `PLANNER` oneof arm entirely, along with `Pilot`/`Motion::Executor`, the
-  only things that ever consumed it. `OtosConfigPatch` (issue
+- **Config patches cover `MotorConfigPatch`, `OtosConfigPatch`
+  (109-004), and `EstimatorConfigPatch` (117) only.** `RobotLoop::
+  handleConfig` replies `ERR_UNIMPLEMENTED` for `DRIVETRAIN`/`WATCHDOG`/
+  `NONE` (`DrivetrainConfigPatch` has no on-robot fusion consumer).
+  `PlannerConfigPatch` is GONE, not merely out of scope — 115-005 (gut
+  S1) deleted the type and `ConfigDelta`'s own `PLANNER` oneof arm
+  entirely, along with `Pilot`/`Motion::Executor`, the only things that
+  ever consumed it. `OtosConfigPatch` (issue
   `otos-calibration-config-message.md`) restores a RUNTIME path to
   `Devices::Otos::setLinearScalar()`/`setAngularScalar()`/`setOffset()`/
   `init()` — previously only ever called once at boot from baked
@@ -160,7 +209,12 @@
   `handleConfig()` (still "the loop's own cycle" per the single-loop bus
   ownership invariant above: a rare, command-triggered I2C/config
   transaction sandwiched into the existing schedule, not a new per-cycle
-  bus consumer).
+  bus consumer). `EstimatorConfigPatch` (117) merges present
+  `weight_heading_otos`/`weight_omega_otos`/`staleness_ms` fields onto
+  `StateEstimator`'s own live weight state — a pure in-memory update, NOT
+  an I2C transaction (unlike the OTOS branch above), and NOT persisted
+  into `persistedTuning_`/flash (Design Rationale Decision 4, overlay
+  `design.md`'s sibling — a reboot reverts to the baked JSON default).
 
 ## 4. Design
 
@@ -354,6 +408,26 @@
 - **`Preamble::step()`/`done()`/per-device status accessors:** `step()`
   never blocks; `done()` is true once every device has reached a terminal
   state (present-and-ready or confirmed-absent).
+- **`StateEstimator::update(frame, now)`/`wheelAt(wheel, t)`/`bodyAt(t)`/
+  `whereAmI(now)`/`wheelNow(wheel)`/`reset(x, y, heading)`/
+  `innovations()`/`setWeights(weights)`** (117): `update()` — call once
+  per cycle from the trailing `kPace` block, after `frame_.pose` is
+  staged; refreshes both wheel peers' and the body peer's basis. `wheelAt`/
+  `bodyAt` — pure ZOH extrapolation from the current basis to an
+  explicit query time `t`; no owned clock, hand-fed `t` always, mirroring
+  `Motion::StopCondition`'s own testability shape. `whereAmI(now)` is
+  exactly `bodyAt(now)`; `wheelNow(wheel)` returns the wheel's raw basis
+  with no extrapolation. `reset(x, y, heading)` re-anchors the body
+  peer's world pose only (wheel peers are untouched — they track
+  per-wheel distance, not world pose, the same reasoning `Odometry::
+  pathLength()` is untouched by `Odometry::reset()`). `innovations()`
+  returns the most recent OTOS-vs-predicted heading/omega residual —
+  computed for diagnostic/validation purposes even while its fusion
+  weight is 0, never fed back into the estimate itself at that weight.
+  `setWeights()` is `RobotLoop::handleConfig()`'s own entry point for a
+  live `EstimatorConfigPatch` (§3 above) — a plain in-memory update, not
+  a bus transaction. All of the above are pure computation: no I2C
+  access, no sleep, bounded per call.
 
 ### Consumes
 
@@ -376,6 +450,20 @@
   `Motion::fromMove()` (115-005, still gone) — the recreated `motion/`
   directory contains only this one small, pure-comparison module,
   mirroring `kinematics/`'s existing small-pure-computation pattern.
+- **`Telemetry::Frame`** (117): `StateEstimator::update()` reads the SAME
+  per-cycle `Frame` struct `Telemetry::setFrame()` stages — it does not
+  hold its own leaf/bus references and does not read `Devices::Motor`/
+  `Devices::Otos` directly. Wire-plane `msg::EstimatorConfigPatch` stops
+  at `RobotLoop::handleConfig()` exactly like `msg::MotorConfigPatch`/
+  `msg::OtosConfigPatch` already do (devices/app isolation invariant
+  above, extended by analogy) — `StateEstimator`'s own `setWeights()`
+  takes a plain, Devices-local-style weights struct, never a `msg::*`
+  type.
+- **`Config::defaultEstimatorConfig()`** (117, `config/boot_config.h`):
+  fail-closed baked fusion-weight defaults (`weight_heading_otos =
+  weight_omega_otos = 0.0` this sprint, `staleness_ms`), constructed once
+  at boot in `main.cpp` and passed to `StateEstimator`'s constructor —
+  see [config/DESIGN.md](../config/DESIGN.md).
 
 ## 6. Open Questions / Known Limitations
 
@@ -413,3 +501,20 @@
   wholesale by 115-005 — the git tag `pre-gut-motion-stack` is the
   authoritative historical record if that design work is ever revisited,
   not a summary re-derived from memory here.
+- **`StateEstimator`'s predictions are not exposed on the wire (117).**
+  Neither `msg::Telemetry` nor `msg::TelemetrySecondary` gained a field
+  for `whereAmI()`/`wheelNow()` output this sprint — validation runs
+  host-side against the raw `EncoderReading`/`OtosReading` fields already
+  telemetered (sprint 115), replaying the identical ZOH math in Python
+  over a captured TLM-log CSV. A future on-robot consumer (the
+  remaining-distance trajectory controller) will need `whereAmI()`
+  results live, in-process — that consumer calls the estimator directly
+  (same process, same cycle), not over the wire, so this gap may never
+  need closing; flagged as open only because it was an explicit sizing
+  choice, not an oversight.
+- **`EstimatorConfigPatch`-set fusion weights are volatile, not
+  persisted.** Unlike `MotorConfigPatch`/`OtosConfigPatch` (114-004),
+  a live-tuned weight does not survive a reboot — it reverts to the
+  baked JSON default. Revisit once fake-OTOS/external-pose fusion
+  (future sprints) give these weights real, nonzero, bench-validated
+  values worth persisting.
```
