// planner.h -- Motion::Planner: the standalone motion planner (design of
// record: docs/design/motion-planner-sketch.md, stakeholder-reviewed
// 2026-07-25). Owns everything between "a Move arrived" and "here are the
// wheel velocity targets for the next control interval": the move queue and
// lookahead, the discrete-exact trapezoid profiler (profile.h), and the
// sense-side estimation (estimation.h).
//
// The two-method contract, enforced by constness in both directions:
//   tick(const RobotState&)  -- DO THE WORK; provably cannot touch the
//                               blackboard. Returns the completion event.
//   update(RobotState&) const -- SAVE tick()'s results into the state;
//                               provably computes nothing new. The ONLY
//                               Planner method that mutates RobotState.
// Moves are handed in as they arrive via move(); activation happens on the
// next tick() (which is where current time/pose baselines live).
#pragma once

#include <cstdint>

#include "estimation.h"
#include "planner_types.h"
#include "types/robot_state.h"

namespace Motion {

class Planner {
 public:
  // 1 active + 4 pending, protocol v4's 5-deep queue.
  static constexpr int kQueueDepth = 5;

  explicit Planner(const PlannerLimits& limits);

  // Queue a Move. replace=true drops everything (active included) and the
  // new Move activates on the next tick(). Returns false when the queue is
  // full (caller acks ERR_FULL; queue provably unchanged).
  bool move(const Move& next, bool replace);

  // Flush everything and command zero.
  void stop();

  // See the file header. `state.time.cycleStart` is the clock.
  TickResult tick(const RobotState& state);
  void update(RobotState& state) const;

  // Observability (tests / telemetry derivation).
  bool active() const { return active_.occupied; }
  int pendingCount() const { return pendingCount_; }
  uint32_t activeMoveId() const { return active_.move.id; }  // valid iff active()
  float commandedLeft() const { return cmdLeft_; }    // [mm/s]
  float commandedRight() const { return cmdRight_; }  // [mm/s]

 private:
  struct ActiveMove {
    bool occupied = false;
    Move move{};
    uint32_t activationTime = 0;   // [ms]
    float baselinePath = 0.0f;     // [mm] pathLength() at activation
    float baselineHeading = 0.0f;  // [rad] heading() at activation
    bool closingIssued = false;    // last command was the exact terminal step
  };

  enum class Axis : uint8_t { None, Linear, Angular, Wheels };
  static Axis axisOf(const Move& m);

  // Pop pending_[0] into active_, capturing baselines from the current
  // pose -- or from the completed predecessor's cumulative boundary
  // (baseline + threshold) on a same-measure chain, so sub-tick boundary
  // residual is debited to the successor and a chain leaks zero total
  // error. Resets the profile carry velocity unless the new Move
  // continues the same axis (same-axis carry, sketch §3 lookahead).
  void activateNext(uint32_t now);

  // The boundary velocity to land the ACTIVE Move at: 0, or the entry
  // velocity the next pending Move can accept (same axis, same direction).
  float boundaryVelocity(float dt) const;

  // Compute the active Move's command for the next interval; stages
  // cmdLeft_/cmdRight_ and updates the profile carry state.
  void planActive(uint32_t now, float dt);

  // No active Move: ramp the staged command down to zero within limits.
  void drainToZero(float dt);

  PlannerLimits limits_;
  WheelChannel left_, right_;
  PoseTracker pose_;

  Move pending_[kQueueDepth]{};
  int pendingCount_ = 0;
  ActiveMove active_{};

  // Positive-frame previous command on the profiled axis (the carry the
  // next tick ramps from); per-wheel carries for Wheels Moves and drain.
  float profileVelocity_ = 0.0f;
  float cmdLeft_ = 0.0f;   // [mm/s]
  float cmdRight_ = 0.0f;  // [mm/s]
  Axis lastAxis_ = Axis::None;
  float activeBoundary_ = 0.0f;  // last planned boundary velocity, positive frame

  // Cumulative-baseline carry staged between a completion and the next
  // activation (see activateNext()).
  bool carryValid_ = false;
  Move::Kind carryKind_ = Move::Kind::Time;
  float carryPath_ = 0.0f;     // [mm]
  float carryHeading_ = 0.0f;  // [rad]

  uint32_t lastTickTime_ = 0;  // [ms] staged for update()'s estimate bases
  bool ticked_ = false;
};

}  // namespace Motion
