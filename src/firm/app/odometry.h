// odometry.h -- App::Odometry: integrates wheel motion into a world pose
// estimate (encoder-only dead reckoning), plus the minimal OTOS-only
// perception step (applyOtosSample() below).
//
// Boundary: inside -- reading both motors' position deltas, calling
// BodyKinematics::forward(), accumulating x/y/theta; outside -- fusing
// with OTOS/camera (the host's job) and minimal OTOS sampling itself.
// applyOtosSample() below is therefore a FREE FUNCTION, not an Odometry
// method -- it lives in this same file pair as a bounded perception step,
// not a separate module.
//
// Minimal-OTOS-only-perception: a full 3-way Perception round-robin
// (otos|line|color) is deliberately NOT built -- Telemetry carries no
// line=/color= fields yet, so there is nothing for those two slots to
// feed. `Preamble` still detects line/color PRESENCE at boot; only their
// steady-state sampling is absent (see DESIGN.md §6). applyOtosSample()
// samples the Otos leaf and copies the result straight into a
// Telemetry::Frame -- no perception class, no round-robin scheduler.
// Otos::tick()'s OWN internal rate limiting (kReadPeriod, otos.h) is left
// completely unchanged; applyOtosSample() is safe to call every cycle
// because a too-soon call is already a documented no-bus-traffic no-op
// inside Otos::tick() itself. Bus discipline (never calling this from
// inside a motor request->collect window) is the loop's job; this
// function itself is a single bounded call with no internal sleeps.
#pragma once

#include <cstdint>

#include "app/telemetry.h"
#include "devices/nezha_motor.h"
#include "devices/otos.h"

namespace App {

class Odometry {
 public:
  // left/right -- the SAME two NezhaMotor leaves Drive drives, in
  // BodyKinematics' own L/R convention. trackWidth -- [mm],
  // BodyKinematics::forward()'s own `b` parameter. The constructor
  // snapshots each leaf's CURRENT position() as the delta baseline (both
  // leaves default to 0 before their first tick() -- nezha_motor.h) so the
  // very first integrate() call sees a zero delta, not a phantom jump from
  // whatever the leaf's boot-time absolute position happens to be.
  //
  // Rebaselining note (encoder-reset-on-reboot semantics): NezhaMotor's own
  // position() is relative to ITS OWN encoder zero, which is re-anchored
  // every firmware boot (nezha_motor.h's begin()/hardReset()). Odometry's
  // x_/y_/theta_ are therefore only ever continuous WITHIN one firmware
  // session -- a reboot resets both the leaves' own encoder baseline AND
  // this class's fresh-constructed x_/y_/theta_ to zero together, with no
  // attempt made here to reconcile across the discontinuity. A host
  // consuming this pose over the wire is responsible for detecting a
  // reboot (e.g. a telemetry sequence-number reset) and handling the
  // discontinuity itself -- this class does not add any reboot-detection
  // or cross-session pose-splicing logic.
  Odometry(Devices::NezhaMotor& left, Devices::NezhaMotor& right, float trackWidth);

  // Reads both leaves' position() (the leaf's OWN cached encoder position
  // -- no shadow copy kept here beyond the delta baseline below), computes
  // this cycle's per-wheel delta against the last integrate() call, and
  // maps the delta pair through BodyKinematics::forward() (NOT a
  // hand-rolled equivalent) to get a per-cycle (distance, headingDelta)
  // pair. This is valid without a separate dt because forward()'s
  // equations are linear/homogeneous in vL/vR -- feeding it position
  // DELTAS directly yields (distance, headingDelta) for exactly this
  // cycle, the same way feeding it velocities would yield (v, omega).
  // Accumulates x_/y_/theta_ via midpoint-arc integration (heading at
  // theta_ + headingDelta/2 -- the standard differential-drive
  // dead-reckoning update, needed because forward() itself only returns
  // BODY-frame deltas, not a world-frame pose). Call once per loop cycle,
  // after both leaves' own tick() has run that cycle.
  void integrate();

  float x() const { return x_; }          // [mm]
  float y() const { return y_; }          // [mm]
  float theta() const { return theta_; }  // [rad]

  // lastDistance/lastHeadingDelta -- the most recent integrate() call's own
  // PER-CYCLE body-frame forward-travel/heading-change (the same
  // BodyKinematics::forward() outputs integrate() accumulates into x_/y_/
  // theta_, exposed here BEFORE accumulation -- 109-005: Motion::Executor's
  // DISTANCE-mode completion criterion needs encoder-relative PROGRESS
  // since a command's own activation, which App::Pilot accumulates itself
  // call-by-call from this per-cycle delta; it is not something Odometry
  // itself needs to track cumulatively for its own purposes). Both 0.0f
  // before the first integrate() call.
  float lastDistance() const { return lastStepDistance_; }        // [mm]
  float lastHeadingDelta() const { return lastStepHeadingDelta_; }  // [rad]

  // Snap the dead-reckoned pose to (x, y, theta) and RE-ANCHOR the delta
  // baseline to each leaf's CURRENT position(), so the next integrate() sees
  // a zero delta rather than a phantom jump from the old baseline. This is
  // the in-session pose reset a future wire verb will drive (no binary arm
  // exists yet -- see DESIGN.md §6); it is exercised today by the host
  // simulator's teleport-to-origin (tests/_infra/sim/sim_harness.h
  // SimHarness::setTruePose()). Additive: no existing caller's behaviour
  // changes unless it calls reset().
  void reset(float x, float y, float theta);  // [mm] [mm] [rad]

 private:
  Devices::NezhaMotor& left_;
  Devices::NezhaMotor& right_;
  float trackWidth_;  // [mm]

  float lastLeft_ = 0.0f;   // [mm] delta baseline -- see constructor comment
  float lastRight_ = 0.0f;  // [mm]

  float x_ = 0.0f;      // [mm]
  float y_ = 0.0f;      // [mm]
  float theta_ = 0.0f;  // [rad]

  float lastStepDistance_ = 0.0f;      // [mm] see lastDistance()'s own comment
  float lastStepHeadingDelta_ = 0.0f;  // [rad] see lastHeadingDelta()'s own comment
};

// applyOtosSample() -- the minimal OTOS-only perception step (see file
// header). Samples `otos` (rate-limited internally by its own
// readDue()/kReadPeriod, unchanged) and copies the result into `frame`'s
// otos/otosConnected/hasOtos fields; call this BEFORE the caller's next
// Telemetry::setFrame(frame)/emit() -- it must reach Telemetry before that
// cycle's frame is built. `hasOtos` mirrors otos.present() (a chip was
// ever detected at boot) rather than a per-tick freshness bit, because
// Telemetry always carries the LAST staged snapshot -- a rate-limit-skipped
// cycle should still report the most recent real reading, not flip
// has_otos off. `otosConnected` mirrors the leaf's own live, per-tick
// connected(). No pose fusion happens here -- the robot does not fuse; the
// raw OTOS pose rides to the host verbatim for host-side fusion. A chip
// that was never detected (present() false) is a total no-op beyond
// setting the two bools false -- `frame.otos` is left exactly as the
// caller staged it.
void applyOtosSample(Devices::Otos& otos, uint64_t now, Telemetry::Frame& frame);  // [us]

}  // namespace App
