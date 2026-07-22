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
// Minimal-OTOS-only-perception: this file only ever samples OTOS -- line/
// color sampling (115-005, gut S1) is wired directly into
// App::RobotLoop::updateLineColor(), not through a shared perception class
// here or a round-robin scheduler; each sensor is its own bounded, rate-
// limited step, not a unified abstraction. applyOtosSample() samples the
// Otos leaf and copies the full reading (position, heading, AND the
// measured velocities, per telemetry.proto's OtosReading) straight into a
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
#include "devices/motor.h"
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
  Odometry(Devices::Motor& left, Devices::Motor& right, float trackWidth);

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
  // BODY-frame deltas, not a world-frame pose). Also accumulates
  // fabsf(distance) into pathLength_ unconditionally, every call -- see
  // pathLength()'s own doc comment. Call once per loop cycle, after both
  // leaves' own tick() has run that cycle.
  void integrate();

  float x() const { return x_; }          // [mm]
  float y() const { return y_; }          // [mm]
  float theta() const { return theta_; }  // [rad]

  // Cumulative |distance| accumulated across every integrate() call since
  // construction -- an odometer, not a net-displacement value: forward
  // travel and reverse travel over the same ground both add (116-003,
  // Motion::StopCondition's DISTANCE kind baselines against a snapshot of
  // this value at MOVE activation and diffs it against the current reading,
  // so it must monotonically grow with path length regardless of direction
  // reversals mid-move).
  //
  // reset()'s interaction: pathLength() is NOT zeroed by reset(). reset()
  // re-anchors x_/y_/theta_ (and the delta baseline) to a caller-supplied
  // pose snapshot -- a teleport, not a "trip odometer" request. Zeroing
  // pathLength() on every reset() would be a surprising, undocumented side
  // effect for a caller that never asked for it (e.g. the host simulator's
  // teleport-to-origin), and StopCondition baselines against a pathLength()
  // snapshot taken at MOVE activation regardless of when the odometer
  // itself last reset -- it only ever needs the DELTA since that snapshot,
  // never an absolute zero. If a future caller needs a zeroed trip odometer,
  // that is a distinct method, not an implicit reset() side effect.
  float pathLength() const { return pathLength_; }  // [mm]

  // Snap the dead-reckoned pose to (x, y, theta) and RE-ANCHOR the delta
  // baseline to each leaf's CURRENT position(), so the next integrate() sees
  // a zero delta rather than a phantom jump from the old baseline. This is
  // the in-session pose reset a future wire verb will drive (no binary arm
  // exists yet -- see DESIGN.md §6); it is exercised today by the host
  // simulator's teleport-to-origin (tests/_infra/sim/sim_harness.h
  // SimHarness::setTruePose()). Additive: no existing caller's behaviour
  // changes unless it calls reset(). Does NOT touch pathLength() -- see
  // that accessor's own doc comment above.
  void reset(float x, float y, float theta);  // [mm] [mm] [rad]

 private:
  Devices::Motor& left_;
  Devices::Motor& right_;
  float trackWidth_;  // [mm]

  float lastLeft_ = 0.0f;   // [mm] delta baseline -- see constructor comment
  float lastRight_ = 0.0f;  // [mm]

  float x_ = 0.0f;      // [mm]
  float y_ = 0.0f;      // [mm]
  float theta_ = 0.0f;  // [rad]

  float pathLength_ = 0.0f;  // [mm] cumulative |distance| -- see pathLength()
};

// applyOtosSample() -- the minimal OTOS-only perception step (see file
// header). Samples `otos` (rate-limited internally by its own
// readDue()/kReadPeriod, unchanged) and copies the full reading (x, y,
// heading, v_x, v_y, omega) plus the burst's own read time into `frame`'s
// `otos` field; call this BEFORE the caller's next
// Telemetry::setFrame(frame)/emit() -- it must reach Telemetry before that
// cycle's frame is built.
//
// `frame.otosPresent` -- 115-005: the new telemetry.proto flags bit 0
// (otos_present) is documented as "OtosReading fresh THIS frame", a
// tighter contract than the old (pre-115) hasOtos, which mirrored
// otos.present() (a chip was EVER detected at boot) so a rate-limit-skipped
// cycle wouldn't flip it off. Now that OtosReading carries its own `time`
// field, freshness itself is the signal a caller needs -- frame.otosPresent
// is therefore `otos.present() && otos.poseFresh()`: true only on a cycle
// this function's own otos.tick() call actually refreshed the cached pose.
// `frame.otosConnected` mirrors the leaf's own live, per-tick connected(),
// unchanged from before. No pose fusion happens here -- the robot does not
// fuse; the raw OTOS pose rides to the host verbatim for host-side fusion.
// `frame.otos` itself is only overwritten when otosPresent is true --
// otherwise it is left exactly as the caller last staged it (Telemetry's
// own "last staged snapshot" contract), even though the flags bit that
// gates its validity will correctly read false that frame.
void applyOtosSample(Devices::Otos& otos, uint64_t now, Telemetry::Frame& frame);  // [us]

}  // namespace App
