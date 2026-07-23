// move_queue.h -- App::MoveQueue: owns the lifecycle of the robot's queued
// and active bounded motions. Full history/design rationale (StateEstimator&
// removal, land-at-zero derivation, margin-factor sweeps): src/firm/app/DESIGN.md.
//
// Boundary: inside -- the 5-slot array (1 active + 4 pending), replace/
// flush/enqueue/ERR_FULL bookkeeping, chain-advance on stop/timeout, one
// Motion::StopCondition per active Move, the land-at-zero predicate. Outside
// -- validating a Move's shape (RobotLoop::handleMove()), a velocity
// variant -> wheel duty (Drive), "traveled far enough" (StopCondition +
// App::Odometry). tick(now, odom) takes CURRENT readings, not the held
// Odometry&, so a same-cycle chain-advance can reuse them as the next
// Move's StopCondition baseline instead of a second, disagreeing read.
#pragma once

#include <cstdint>

#include "app/drive.h"
#include "app/odometry.h"
#include "devices/clock.h"
#include "messages/envelope.h"
#include "motion/stop_condition.h"
#include "motion/velocity_shaper.h"

namespace App {

// ShaperLimits -- Motion::VelocityShaper's own accel/decel/jerk ceilings
// (full derivation: velocity_shaper.h). Mirrors Config::ShaperBootConfig
// (config/ may not depend on app/); main.cpp converts between them.
// Namespace-scope, not nested in MoveQueue: a nested struct's default
// member initializers can't be referenced by a default constructor
// argument of the enclosing, still-being-defined class (C++ restriction).
//
// Disabled-axis sentinel: aMax<=0 OR aDecel<=0 OR jMax<=0 disables LINEAR
// shaping; alphaMax<=0 OR alphaDecel<=0 OR yawJerkMax<=0 disables ANGULAR
// shaping and land-at-zero on that axis (tick()'s doc comment). Default
// ShaperLimits{} (all-0) is the no-op config -- shaping is unconditionally
// ON in production (gen_boot_config.py requires all six robot-JSON keys).
struct ShaperLimits {
  float aMax = 0.0f;         // [mm/s^2] linear accel-ramp ceiling
  float aDecel = 0.0f;       // [mm/s^2] linear decel-taper ceiling
  float alphaMax = 0.0f;     // [rad/s^2] angular accel-ramp ceiling
  float alphaDecel = 0.0f;   // [rad/s^2] angular decel-taper ceiling
  float jMax = 0.0f;         // [mm/s^3] linear jerk ceiling
  float yawJerkMax = 0.0f;   // [rad/s^3] angular jerk ceiling
};

class MoveQueue {
 public:
  static constexpr int kMaxPending = 4;

  // corrId echoed back unchanged; err is ERR_NONE (enqueued/activated) or
  // ERR_FULL (rejected, queue provably unchanged).
  struct EnqueueResult {
    uint32_t corrId = 0;
    msg::ErrCode err = msg::ErrCode::ERR_NONE;
  };

  // Reported when a Move ends (StopConditionMet or TimedOut).
  struct Completion {
    uint32_t moveId = 0;
    bool timedOut = false;
  };

  // At most one completion per tick() call -- only one Move is ever active.
  struct TickResult {
    bool completed = false;
    Completion completion{};  // valid iff completed
  };

  // shaperLimits defaults to ShaperLimits{} (shaping OFF); live-retunable via setShaperLimits().
  MoveQueue(Drive& drive, Odometry& odom, const Devices::Clock& clock,
            ShaperLimits shaperLimits = ShaperLimits{});

  // replace==true: flushes pending (no ack), preempts active, `move`
  // activates this SAME call (fresh StopCondition baseline). replace==
  // false: activates if empty, else appends (or ERR_FULL no-op past 4).
  EnqueueResult enqueue(const msg::Move& move, uint32_t corrId);

  // Per-cycle tick: ticks the always-armed threshold/timeout backstop; on
  // StopConditionMet/TimedOut, activates the next pending Move THIS SAME
  // CALL (seamless hand-off, SUC-051) or Drive::stop() if now empty.
  //
  // Land-at-zero (src/firm/app/DESIGN.md "118 ticket 004"): on Continue, a
  // TWIST Move on its stop-condition axis, with that axis's ShaperLimits
  // enabled, ALSO completes once `remaining <= (commandedSpeed^2 /
  // (2*decelCeiling)) * marginFactor` (already inside our own braking
  // envelope) -- kStoppingMarginFactorChain (pendingCount() > 0, ships
  // 0.48) or kStoppingMarginFactorFinal (pendingCount() == 0, ships 0.92);
  // sweep data in move_queue.cpp's anonymous namespace. WHEELS/Kind::Time
  // never qualify; axis disabled (default) means this path never fires.
  //
  // Shaping: on Continue, also re-stages velocity via Drive::setTwist()/
  // setWheels() through Motion::VelocityShaper (non-primary axes UNSHAPED),
  // a no-op per axis whenever ShaperLimits disables it.
  TickResult tick(uint64_t now, const Odometry& odom);  // [us]

  // Live-tuning entry point: plain in-memory write, never persisted.
  void setShaperLimits(ShaperLimits limits) { shaperLimits_ = limits; }
  ShaperLimits shaperLimits() const { return shaperLimits_; }

  // Drains every pending slot and ends the active Move (if any), no ack for
  // any of them (STOP acks itself via corr_id). Always calls Drive::stop().
  void flush();

  bool active() const { return active_.occupied; }  // frame_.mode/driving_ derivation

  // True iff BOTH axes' ShaperLimits are disabled (shapeAndStage()'s early-
  // return gate) -- ANDed with active() to drive kFlagFaultShapingDisabled.
  bool shapingDisabled() const;

  // Test/observability seam -- not called by RobotLoop.
  int pendingCount() const { return pendingCount_; }
  const msg::Move& pendingAt(int index) const { return pending_[index]; }  // index < pendingCount()
  uint32_t activeMoveId() const { return active_.moveId; }  // valid iff active()

 private:
  // Stores the active Move's StopCondition-construction ARGUMENTS as plain
  // scalars (it has no default ctor -- stop_condition.h); tick() rebuilds
  // a fresh, byte-identical StopCondition every call.
  struct ActiveMove {
    bool occupied = false;
    uint32_t moveId = 0;
    Motion::StopCondition::Kind kind = Motion::StopCondition::Kind::Time;
    float threshold = 0.0f;             // [ms]/[mm]/[rad] depending on kind
    float timeout = 0.0f;               // [ms]
    uint64_t activationNow = 0;         // [us]
    float activationPathLength = 0.0f;  // [mm]
    float activationTheta = 0.0f;       // [rad]

    // Cruise-target velocity, captured at activation; shapeAndStage() shapes toward these.
    msg::Move::VelocityKind velocityKind = msg::Move::VelocityKind::NONE;
    float cruiseVX = 0.0f;      // [mm/s] TWIST only
    float cruiseVY = 0.0f;      // [mm/s] TWIST only (always 0 -- see Drive::setTwist()'s own doc comment)
    float cruiseOmega = 0.0f;   // [rad/s] TWIST only
    float cruiseVLeft = 0.0f;   // [mm/s] WHEELS only
    float cruiseVRight = 0.0f;  // [mm/s] WHEELS only
  };

  // Stages `move`'s velocity onto drive_ and populates active_ -- shared by
  // enqueue()'s two activation paths and tick()'s chain-advance path.
  void activate(const msg::Move& move, uint64_t now, float pathLength, float theta);

  // Shaped speed for whichever axis ShaperLimits enables, re-staged via
  // drive_. Called only on Continue; pathLength/theta reuse tick()'s own.
  void shapeAndStage(uint64_t now, float pathLength, float theta);

  // The land-at-zero predicate (tick()'s doc comment). Pure query.
  bool landAtZero(float pathLength, float theta, float dt) const;

  Drive& drive_;
  Odometry& odom_;
  const Devices::Clock& clock_;

  ActiveMove active_;
  msg::Move pending_[kMaxPending];
  int pendingCount_ = 0;

  // shaperLimits_ is the live-tunable config; the four shaper* members are
  // the RUNNING per-axis VelocityShaper instances, MoveQueue-level (not
  // ActiveMove-level) so a chained Move ramps smoothly. tick() calls
  // .reset() on every completion so a residual speed can't leak into the
  // NEXT Move's own completion decision on the same axis.
  ShaperLimits shaperLimits_;
  Motion::VelocityShaper shaperVX_;      // [mm/s] / [mm/s^2]
  Motion::VelocityShaper shaperOmega_;   // [rad/s] / [rad/s^2]
  Motion::VelocityShaper shaperVLeft_;   // [mm/s] / [mm/s^2]
  Motion::VelocityShaper shaperVRight_;  // [mm/s] / [mm/s^2]
  uint64_t lastShapeNow_ = 0;  // [us] dt baseline for shapeAndStage(), reset at each activate()
};

}  // namespace App
