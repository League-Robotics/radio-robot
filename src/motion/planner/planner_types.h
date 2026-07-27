// planner_types.h -- the plain, motion-owned value types the Planner's
// public surface speaks: Move (what to do), PlannerLimits (construction
// config), TickResult (per-cycle completion event). No msg::*, no wire
// types -- the base's dispatch converts wire msg::Move -> Motion::Move at
// the boundary (motion-planner sketch §2). All structs are trivially
// copyable and C-layout so the ctypes harness mirrors them directly.
#pragma once

#include <cstdint>

namespace Motion {

// One bounded motion. Direction comes from the velocity sign; thresholds
// are positive magnitudes (protocol-v4 Move semantics, minus wire baggage).
struct Move {
  // Kind -- what ENDS this Move. Time/Distance/Angle are stop CONDITIONS
  // measured against the clock or odometry. `Stop` is the odd one out and
  // deliberately so: it is the PLANNED STOP queue entry
  // (command-ingestion-ring-buffered-comms-subsystem-routing-two-stops.md
  // §5, the wire's `STOP` verb) -- "come to a stop when you reach THIS
  // point in the queued sequence." It gets its own enumerator rather than a
  // bool on the side because "a Move that is a stop" and "a Move that stops
  // when X" are different in kind, and a bool would leave every switch here
  // silently doing the wrong thing for it.
  enum class Kind : uint8_t { Time, Distance, Angle, Stop };
  enum class VelocityKind : uint8_t { Twist, Wheels };

  uint32_t id = 0;
  Kind kind = Kind::Time;
  float threshold = 0.0f;  // [ms] Time / [mm] Distance / [rad] Angle; positive
  float timeout = 0.0f;    // [ms] safety backstop; 0 = none
  VelocityKind velocityKind = VelocityKind::Twist;
  float v_x = 0.0f;        // [mm/s] signed cruise, Twist
  // Carried so a holonomic drivetrain needs no type change, and because a
  // twist HAS three components (naming-and-style rule 2) -- but accepted
  // and ignored on this differential build, exactly as the wire's own
  // MoveTwist.v_y is. A Distance Move with v_x == 0 and v_y != 0 is a
  // pure-sideways motion this drivetrain cannot make, and move() rejects
  // it rather than silently driving nothing.
  float v_y = 0.0f;        // [mm/s] signed cruise, Twist; ignored on differential
  float omega = 0.0f;      // [rad/s] signed cruise, Twist
  float vLeft = 0.0f;      // [mm/s] signed cruise, Wheels
  float vRight = 0.0f;     // [mm/s] signed cruise, Wheels
};

struct PlannerLimits {
  float vMax = 600.0f;          // [mm/s] linear velocity ceiling
  float aMax = 0.0f;            // [mm/s^2] linear accel-ramp ceiling
  float aDecel = 0.0f;          // [mm/s^2] linear decel-taper ceiling
  float omegaMax = 8.0f;        // [rad/s] angular velocity ceiling
  float alphaMax = 0.0f;        // [rad/s^2] angular accel-ramp ceiling
  float alphaDecel = 0.0f;      // [rad/s^2] angular decel-taper ceiling
  float trackWidth = 100.0f;    // [mm] wheel separation
  float controlPeriod = 50.0f;  // [ms] the loop interval one tick() plans for
  float actuationDelay = 0.0f;  // [ms] command-staged-to-wheels latency compensated in planning
  float velocityFilterWeight = 1.0f;  // EMA weight on fresh encoder velocity; 1 = unfiltered
  uint32_t otosStaleness = 200;       // [ms] max OTOS age still eligible to blend
  float headingOtosWeight = 0.0f;     // [0..1] complementary blend, fail-closed default

  // Settle-confirm completion: when on, a Distance/Angle Move that has
  // reached profile-complete additionally waits until it has physically
  // ARRIVED and come to rest before the completion is reported, up to
  // settleWindow. Off by default -- profile-complete is already exact in a
  // zero-error plant; settle-confirm buys robustness against a lagging or
  // overshooting real plant at the cost of a few idle cycles.
  bool requireSettle = false;
  float settleWindow = 0.0f;  // [ms] max extra wait past profile-complete

  // Heading hold on Distance Moves: P gain on the UNCOMMANDED angular
  // axis, driving heading back to the Move's activation baseline. The
  // correction is purely differential, so the profiled path length -- and
  // therefore the distance exactness -- is untouched. 0 = off.
  float headingHoldGain = 0.0f;  // [1/s] rad/s of correction per rad of error

  // M4 duty-plane output stage (issue §7.6): the per-wheel velocity->duty
  // PID relocated into the motion library. Fail-closed: all-zero gains
  // (the default) leave the duty outputs at 0 and every velocity-plane
  // consumer untouched. Gains mirror the robot-JSON vel_* vocabulary.
  float velKff = 0.0f;   // [duty/(mm/s)] feedforward slope
  float velKp = 0.0f;    // [duty/(mm/s)] proportional
  float velKi = 0.0f;    // [duty/(mm/s)/s] integral rate
  float velIMax = 0.0f;  // [duty] integrator clamp; 0 disables integration

  // Acceleration feedforward (velKaff ~= plantTau * velKff): a first-order
  // plant needs (v + tau*dv/dt)/gain of duty to TRACK a ramp, not v/gain.
  // Feeding the commanded accel through this term keeps ramp tracking
  // error near zero, so the integral never winds up during ramps -- the
  // measured cruise-entry overshoot spike was exactly that windup
  // releasing. 0 = off.
  float velKaff = 0.0f;  // [duty/(mm/s^2)]
  float velIAccelGate = 1.0e9f;  // [mm/s^2] integral ramp gate (wheel_pid.h)
  // Motor write-suppression deadband ([-1,1] duty): the leaf's armored
  // write forces any |duty| below this to zero before it reaches the bus,
  // so a NONZERO commanded velocity whose PID duty computes below it is a
  // command the hardware silently ignores -- measured on the robot as the
  // settle creep stalling dead for ~1 s (then walking back on one wheel
  // only) while ~0.02-duty commands evaporated. The duty stage floors the
  // staged duty here whenever the commanded velocity is nonzero. 0 = off.
  float dutyFloor = 0.0f;
  // Settle-confirm arrival tolerances -- "close enough to the target" for
  // THIS robot's fine-positioning resolution. On an ideal plant the
  // defaults are easily reachable; on a stiction plant whose minimum
  // creep step is one dutyFloor pulse per control period, an epsilon
  // finer than that step is unreachable and every settling Move burns
  // the whole settleWindow before completing at expiry.
  float settleEpsilonLinear = 1.0f;     // [mm]
  float settleEpsilonAngular = 0.005f;  // [rad]

  // Jerk ceilings (S-curve shaping): bound how fast the PROFILE's own
  // acceleration may change, and taper acceleration to zero exactly at
  // cruise (no instant accel corners -> no whiplash). 0 = disabled
  // (plain trapezoid, all pre-existing behavior unchanged).
  float jerkMax = 0.0f;     // [mm/s^3] linear axis
  float yawJerkMax = 0.0f;  // [rad/s^3] angular axis

  // Settle rest floors -- "at rest" thresholds on the FILTERED measured
  // velocity. A per-robot property: they must sit ABOVE the robot's own
  // filtered velocity-noise floor (else rest never confirms) and LOW
  // enough that the post-settle coast (~floor * plant tau) stays inside
  // the arrival epsilons. Defaults match the previous built-in constants.
  float settleRestVelocity = 5.0f;  // [mm/s]
  float settleRestOmega = 0.02f;    // [rad/s]
};

// The per-tick completion event, returned by tick() (never written into
// RobotState -- acks are protocol bookkeeping, not robot state). At most
// one Move can end per tick.
struct TickResult {
  bool completed = false;
  uint32_t moveId = 0;
  bool timedOut = false;
  // True when the Move was, at the moment it completed, both within the
  // arrival epsilon of its target and at rest. Always evaluated and
  // reported truthfully, whether or not `requireSettle` deferred the
  // completion to obtain it (a settleWindow expiry completes with
  // settled = false and timedOut = false -- the window ran out, the Move
  // did not).
  bool settled = false;
};

}  // namespace Motion
