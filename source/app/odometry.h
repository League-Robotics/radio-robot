// odometry.h -- App::Odometry: integrates wheel motion into a world pose
// estimate (encoder-only dead reckoning), plus the sprint's minimal
// OTOS-only perception step (applyOtosSample() below).
//
// architecture-update.md (103) Step 3 "Odometry" boundary: inside --
// reading both motors' position deltas, calling BodyKinematics::forward(),
// accumulating x/y/theta; outside -- fusing with OTOS/camera (the HOST's
// job, unchanged by this sprint) and minimal OTOS sampling itself
// (explicitly listed as OUTSIDE Odometry's own boundary in the
// architecture doc). applyOtosSample() below is therefore a FREE FUNCTION,
// not an Odometry method -- it lives in this same file pair only because
// the ticket's own file layout lists exactly drive.{h,cpp}/odometry.{h,cpp}
// ("owned by this ticket, not a separate module" -- ticket 006's own
// description). Serves SUC-006.
//
// Ticket 103-006's minimal-OTOS-only-perception decision
// (architecture-update.md Step 7 Open Question 1): the archived plan's full
// 3-way Perception round-robin (otos|line|color) is deliberately NOT built
// this sprint -- telemetry.proto carries no line=/color= fields at all yet,
// so there is nothing for those two slots to feed. `Preamble` (ticket 007)
// still detects line/color PRESENCE at boot; only their steady-state
// sampling is absent this sprint. applyOtosSample() implements the AC's "a
// direct call" option (over "a small shared struct"): it samples the Otos
// leaf and copies the result straight into a Telemetry::Frame -- no new
// perception class, no round-robin scheduler. Otos::tick()'s OWN internal
// rate limiting (kReadPeriod, otos.h) is left completely unchanged;
// applyOtosSample() is safe to call every cycle (the AC's "at least once
// per cycle" contract) because a too-soon call is already a documented
// no-bus-traffic no-op inside Otos::tick() itself -- see
// app_odometry_harness.cpp's rate-limit scenario. Bus discipline (never
// calling this from inside a motor request->collect window) is the LOOP's
// job (ticket 008); this function itself is a single bounded call with no
// internal sleeps.
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
  // Rebaselining note (encoder-reset-on-reboot semantics, per this
  // ticket's own documentation requirement): NezhaMotor's own position()
  // is relative to ITS OWN encoder zero, which is re-anchored every
  // firmware boot (nezha_motor.h's begin()/hardReset() -- unchanged this
  // sprint). Odometry's x_/y_/theta_ are therefore only ever continuous
  // WITHIN one firmware session -- a reboot resets both the leaves' own
  // encoder baseline AND this class's fresh-constructed x_/y_/theta_ to
  // zero together, with no attempt made here to reconcile across the
  // discontinuity. A host consuming this pose over the wire is responsible
  // for detecting a reboot (e.g. a telemetry sequence-number reset) and
  // handling the discontinuity itself -- this ticket does not add any
  // reboot-detection or cross-session pose-splicing logic.
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

  // Snap the dead-reckoned pose to (x, y, theta) and RE-ANCHOR the delta
  // baseline to each leaf's CURRENT position(), so the next integrate() sees
  // a zero delta rather than a phantom jump from the old baseline. This is
  // the in-session pose reset the wire's SI/OZ/ZERO verbs were meant to drive
  // (deferred, no binary arm yet -- see robot_loop.cpp's handleConfig scope);
  // it is exercised today by the host simulator's teleport-to-origin
  // (tests/_infra/sim/sim_harness.h SimHarness::setTruePose()). Additive: no
  // existing caller's behaviour changes unless it calls reset().
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
};

// applyOtosSample() -- the ticket's minimal OTOS-only perception step (see
// file header). Samples `otos` (rate-limited internally by its own
// readDue()/kReadPeriod, unchanged) and copies the result into `frame`'s
// otos/otosConnected/hasOtos fields, to be called BEFORE the caller's next
// Telemetry::setFrame(frame)/emit() -- the AC's "reaches Telemetry before
// that cycle's frame is built" contract. `hasOtos` mirrors otos.present()
// (a chip was ever detected at boot) rather than a per-tick freshness bit,
// because Telemetry always carries the LAST staged snapshot (telemetry.h's
// own doc comment) -- a rate-limit-skipped cycle should still report the
// most recent real reading, not flip has_otos off. `otosConnected` mirrors
// the leaf's own live, per-tick connected(). No pose fusion happens here --
// the robot does not fuse; the raw OTOS pose rides to the host verbatim for
// host-side fusion. A chip that was never detected (present() false) is a
// total no-op beyond setting the two bools false -- `frame.otos` is left
// exactly as the caller staged it (mirrors Otos::tick()'s own "never begun
// -> zero bus traffic" contract; nothing here invents a zero/clobber
// convention Otos itself doesn't already have).
void applyOtosSample(Devices::Otos& otos, uint64_t now, Telemetry::Frame& frame);  // [us]

}  // namespace App
