// planner.h -- Motion::Planner: the standalone motion planner (design of
// record: docs/design/motion-planner-sketch.md, stakeholder-reviewed
// 2026-07-25). Owns everything between "a Move arrived" and "here are the
// wheel velocity targets for the next control interval": the move queue and
// lookahead, the discrete-exact trapezoid profiler (profile.h), and the
// sense-side estimation (estimation.h).
//
// The two-method contract, enforced by constness in both directions:
//   tick(const Types::RobotState&)  -- DO THE WORK; provably cannot touch the
//                               blackboard. Returns the completion event.
//   update(Types::RobotState&) const -- SAVE tick()'s results into the state;
//                               provably computes nothing new. The ONLY
//                               Planner method that mutates Types::RobotState.
// Moves are handed in as they arrive via move(); activation happens on the
// next tick() (which is where current time/pose baselines live).
#pragma once

#include <cstdint>

#include "estimation.h"
#include "planner_types.h"
#include "types/robot_state.h"
#include "wheel_pid.h"

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
  TickResult tick(const Types::RobotState& state);
  void update(Types::RobotState& state) const;

  // Observability (tests / telemetry derivation).
  bool active() const { return active_.occupied; }
  int pendingCount() const { return pendingCount_; }
  uint32_t activeMoveId() const { return active_.move.id; }  // valid iff active()
  float commandedLeft() const { return cmdLeft_; }    // [mm/s]
  float commandedRight() const { return cmdRight_; }  // [mm/s]

  // M4 duty-plane outputs (planner_types.h velK* gains; 0 while the duty
  // stage is unconfigured). Computed every tick from this tick's staged
  // velocity targets vs the filtered measured wheel velocities.
  float commandedDutyLeft() const { return dutyLeft_; }    // [-1, 1]
  float commandedDutyRight() const { return dutyRight_; }  // [-1, 1]

  // Live-tuning entry points (the CONFIG wire arm / persisted tuning):
  // plain in-memory updates, never persisted here.
  void applyVelGains(float kff, float kp, float ki, float iMax);
  void applyShaperLimits(float aMax, float aDecel, float alphaMax,
                         float alphaDecel, float jerkMax, float yawJerkMax);
  const PlannerLimits& limits() const { return limits_; }

  // True once applyShaperLimits() has ever landed real profile ceilings
  // (boot config or a wire EstimatorConfigPatch). The loop publishes
  // kFlagFaultShapingDisabled -- the loud off-state for the silent-off
  // shaping config boundary (119-001) -- from this while a Move is active.
  bool shaperConfigured() const { return shaperConfigured_; }

 private:
  struct ActiveMove {
    bool occupied = false;
    Move move{};
    uint32_t activationTime = 0;   // [ms]
    float baselinePath = 0.0f;     // [mm] signed mean wheel position at activation
    float baselineHeading = 0.0f;  // [rad] heading() at activation
    bool closingIssued = false;    // last command was the exact terminal step
    // Settle-confirm (PlannerLimits::requireSettle): profile-complete has
    // fired and we are holding the completion back until the body has
    // arrived and stopped, or until settleWindow expires.
    bool settling = false;
    uint32_t settleStart = 0;  // [ms] when profile-complete fired
  };

  enum class Axis : uint8_t { None, Linear, Angular, Wheels };
  static Axis axisOf(const Move& m);

  // M4 duty output stage -- runs on every tick() exit path (planner.cpp).
  void stageDuty(float dt);  // [s]

  // The measured state the completion tests and the settle gate read,
  // computed once per tick from the freshly integrated estimate.
  struct Measurement {
    float bodyVelocity = 0.0f;  // [mm/s] filtered, signed
    float omega = 0.0f;         // [rad/s] filtered, signed
    // Distance-to-go on the active Move's own axis, in its own units
    // ([mm] Distance, [rad] Angle).
    //   plannedRemaining  -- extrapolated forward by the ZOH predict and
    //       the actuation delay: the residual at the instant this tick's
    //       command takes EFFECT, which is what the profiler must plan
    //       against and what profile-completion tests.
    //   anchoredRemaining -- the residual against the last MEASURED wheel
    //       positions, extrapolated by nothing. Noise-free by construction
    //       (it only moves when a real sample lands), which is what "have
    //       we actually ARRIVED?" has to ask.
    float plannedRemaining = 0.0f;
    float anchoredRemaining = 0.0f;
  };
  Measurement measure(uint32_t now) const;  // [ms]

  // Has the active Move physically arrived and come to rest? (See
  // PlannerLimits::requireSettle.) Evaluated on every completion so
  // TickResult::settled is truthful even with settle-confirm off.
  bool settleReached(const Measurement& measured, float dt) const;

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
  void planActive(uint32_t now, float dt, const Measurement& measured);

  // Add the heading-hold differential on top of an already-staged pair of
  // wheel commands (Distance Moves only; no-op at zero gain). Clamped so
  // the faster wheel never exceeds vMax, and differential by construction
  // so the profiled path length is unchanged.
  void applyHeadingHold();

  // No active Move: ramp the staged command down to zero within limits.
  void drainToZero(float dt);

  // Age the staged command by one tick (see cmdLeftPrevious_).
  void rollCommandHistory();

  PlannerLimits limits_;
  WheelChannel left_, right_;
  PoseTracker pose_;
  WheelPid pidLeft_, pidRight_;   // M4 duty stage (inert at zero gains)
  float dutyLeft_ = 0.0f;   // [-1, 1] this tick's duty output
  float dutyRight_ = 0.0f;  // [-1, 1]

  Move pending_[kQueueDepth]{};
  int pendingCount_ = 0;
  ActiveMove active_{};

  // Positive-frame previous command on the profiled axis (the carry the
  // next tick ramps from); per-wheel carries for Wheels Moves and drain.
  float profileVelocity_ = 0.0f;
  float profileAccel_ = 0.0f;  // [mm/s^2 or rad/s^2] last step's implied accel (jerk carry)
  float cmdLeft_ = 0.0f;   // [mm/s] staged last tick
  float cmdRight_ = 0.0f;  // [mm/s] staged last tick
  // Staged the tick before last -- the command actually driving the wheels
  // over the just-elapsed interval when there is one cycle of actuation
  // latency. measure()'s anticipation needs it; nothing else does.
  float cmdLeftPrevious_ = 0.0f;   // [mm/s]
  float cmdRightPrevious_ = 0.0f;  // [mm/s]
  // Last tick's filtered measurement -- the braking stage's actuation-lead
  // compensation derives the measured accel from it (see stageDuty()).
  float measuredLeftPrevious_ = 0.0f;   // [mm/s]
  float measuredRightPrevious_ = 0.0f;  // [mm/s]
  Axis lastAxis_ = Axis::None;
  float activeBoundary_ = 0.0f;  // last planned boundary velocity, positive frame

  // Cumulative-baseline carry staged between a completion and the next
  // activation (see activateNext()).
  bool carryValid_ = false;
  Move::Kind carryKind_ = Move::Kind::Time;
  float carryPath_ = 0.0f;     // [mm]
  float carryHeading_ = 0.0f;  // [rad]

  bool ticked_ = false;  // tick() has run at least once
  bool shaperConfigured_ = false;  // applyShaperLimits() ever landed
};

}  // namespace Motion
