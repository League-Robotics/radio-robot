// odometry.h -- Motion::Odometry: integrates wheel motion into a world pose
// estimate (encoder-only dead reckoning).
//
// Boundary: inside -- taking both wheels' position deltas (handed in by the
// caller every cycle, NOT read from a held Devices::Motor&), calling
// BodyKinematics::forward(), accumulating x/y/theta. Outside -- reading the
// leaves themselves (the base's job -- src/firm/app/robot_loop.cpp reads
// Devices::Motor::position() and passes the two floats in), fusing with
// OTOS/camera (the host's job), and OTOS sampling itself (App::
// applyOtosSample(), src/firm/app/otos_sample.{h,cpp} -- base-side, since it
// needs Devices::Otos& and Telemetry::Frame&, neither of which src/motion may
// depend on; sprint 122 ticket 002 split it out of this file, which used to
// hold both).
//
// 122-002 (motion-library extraction): moved from src/firm/app/odometry.*
// into src/motion/, behind the velocity-sink boundary (wheel_sink.h). This
// class no longer holds a Devices::Motor& -- integrate()/the constructor
// take the CURRENT wheel positions as plain float parameters instead, the
// same values Devices::Motor::position() already returns, read by the
// caller (App::RobotLoop) at the exact point in the cycle this class used to
// read them itself. Zero behavior change: the values flowing into
// integrate() are identical, just handed in rather than pulled.
#pragma once

namespace Motion {

class Odometry {
 public:
  // trackWidth -- [mm], BodyKinematics::forward()'s own `b` parameter.
  // initialLeftPosition/initialRightPosition -- the two leaves' CURRENT
  // position() readings at construction time (mirrors what the caller
  // already has on hand), used to seed the delta baseline so the very
  // first integrate() call sees a zero delta, not a phantom jump from
  // whatever the leaf's boot-time absolute position happens to be.
  Odometry(float trackWidth, float initialLeftPosition, float initialRightPosition);  // [mm] [mm] [mm]

  // Takes both leaves' CURRENT position() readings (the caller's job to
  // read them -- see this file's own header), computes this cycle's
  // per-wheel delta against the last integrate()/construction baseline, and
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
  void integrate(float leftPosition, float rightPosition);  // [mm] [mm]

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
  // baseline to each leaf's CURRENT position (leftPosition/rightPosition,
  // handed in by the caller -- same contract as integrate() above), so the
  // next integrate() sees a zero delta rather than a phantom jump from the
  // old baseline. This is the in-session pose reset a future wire verb will
  // drive (no binary arm exists yet -- see DESIGN.md §6); it is exercised
  // today by the host simulator's teleport-to-origin
  // (src/sim/sim_harness.h SimHarness::setTruePose()). Additive: no
  // existing caller's behaviour changes unless it calls reset(). Does NOT
  // touch pathLength() -- see that accessor's own doc comment above.
  void reset(float x, float y, float theta, float leftPosition, float rightPosition);  // [mm] [mm] [rad] [mm] [mm]

 private:
  float trackWidth_;  // [mm]

  float lastLeft_ = 0.0f;   // [mm] delta baseline -- see constructor comment
  float lastRight_ = 0.0f;  // [mm]

  float x_ = 0.0f;      // [mm]
  float y_ = 0.0f;      // [mm]
  // [rad] unwrapped, monotonically accumulated -- never modulo-wrapped to
  // [-pi, pi]. float32 has ~7 significant decimal digits; once theta_
  // accumulates to roughly 10,000 rad (achievable over a long-running
  // session of continuous turning), the per-cycle headingDelta increment
  // approaches float32's representable epsilon at that magnitude and
  // further small deltas start silently rounding away.
  float theta_ = 0.0f;

  float pathLength_ = 0.0f;  // [mm] cumulative |distance| -- see pathLength()
};

}  // namespace Motion
