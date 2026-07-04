// drivetrain.h -- Subsystems::Drivetrain: the two-wheel differential
// Drivetrain faceplate. Turns a body twist or per-wheel velocity targets
// into a ratio-governed pair of msg::MotorCommands for two Hal::Motors,
// returned (never pushed) as a DrivetrainToMotorCommand.
//
// Ported from the locked interface sketch in clasi/sprints/077-greenfield-
// faceplate-hal-drivetrain-and-dev-bench-system/issues/greenfield-rebuild-
// faceplate-hal-in-a-fresh-source-old-tree-parked.md, Step 4.
//
// Drivetrain holds NO Hal::Motor reference or pointer: tick() takes the two
// wheels' observations (msg::MotorState) as arguments and RETURNS the two
// commands it wants applied -- the wiring layer (main.cpp, ticket 5) is the
// only place that calls Motor::apply() with them. This keeps Drivetrain
// free of any dependency on Hal::Motor's concrete leaf (NezhaMotor); it only
// knows the faceplate's message types.
//
// No PID lives here -- that stays entirely inside NezhaMotor (ticket 3). The
// only control law this class runs is the ratio (sync) governor: see
// governRatio() below.
//
// Two deviations from the issue's Step 4 code sketch, both documented at
// their point of use below:
//   1. The ratio governor runs for BOTH the TWIST arm (kinematics-derived
//      targets) and the WHEELS arm (directly-set targets) -- not only after
//      kinematics. The WHEELS arm is exactly how ticket 7's coupled-rig
//      curve test (ratio_governor_curve.py: "command a curve (unequal wheel
//      targets)" on ports 3+4) exercises the governor, so it must be
//      governed too; only the NEUTRAL arm is a true ungoverned pass-through.
//   2. setMotorCapabilities() is a small addition beyond the sketch's public
//      surface, needed so capabilities().onboard_position can be computed
//      without Drivetrain holding a Hal::Motor reference (see its own
//      comment below). Ticket 3 set the precedent for a documented,
//      rationale-backed deviation from the issue's sketch (Motor::apply()
//      returning bool).
#pragma once

#include <stdint.h>

#include "messages/drivetrain.h"
#include "messages/motor.h"

namespace Subsystems {

// Command-out edge type, named by its endpoints (<Producer>To<Consumer>Command
// per .claude/rules/naming-and-style.md): what the Drivetrain sends to its
// two wheel Motors.
struct DrivetrainToMotorCommand {
  msg::MotorCommand left;
  msg::MotorCommand right;
};

class Drivetrain {
 public:
  // --- Primitive setters -- the real implementation of each command arm. ---

  // A twist is a directed body-frame velocity: v_x, v_y, omega (matches
  // msg::BodyTwist3; math subscripts keep their underscore). v_y is honored
  // only on holonomic drivetrains -- this Drivetrain is differential-only
  // (capabilities().holonomic is always false this sprint; Tovez first, per
  // the issue's locked control-architecture decision 5), so v_y is always
  // ignored -- see commandedWheelTargets()'s TWIST case for the ignore site
  // and the forward-pointer to the future mecanum ticket
  // (architecture-update.md Open Question 6).
  void setTwist(float v_x, float v_y, float omega);   // [mm/s] [mm/s] [rad/s]

  // Direct per-wheel velocity targets -- bypasses kinematics entirely (used
  // by, e.g., the coupled bench rig's curve tests, ticket 7, where
  // "left"/"right" are whichever two ports DEV DT PORTS bound, not
  // necessarily a real drivetrain's wheels). Still passes through the ratio
  // governor like the TWIST arm -- see tick().
  void setWheelTargets(float left, float right);      // [mm/s] signed wheel velocities

  void setNeutral(msg::Neutral mode);

  // --- Faceplate verbs. ---

  void configure(const msg::DrivetrainConfig& config);
  void apply(const msg::DrivetrainCommand& command);   // unpacks oneof -> setters above

  // now: [ms]. leftObs/rightObs: this tick's sampled MotorState for the two
  // bound wheels -- arguments only, never stored, never read from a clock or
  // a Motor reference. See the class comment.
  DrivetrainToMotorCommand tick(uint32_t now,
                                 const msg::MotorState& leftObs,
                                 const msg::MotorState& rightObs);

  msg::DrivetrainState state() const;          // assembled from getters
  msg::DrivetrainCapabilities capabilities() const;

  // Records the two bound wheel motors' capabilities, needed only so
  // capabilities().onboard_position can report accurately without
  // Drivetrain holding a Hal::Motor reference. This is a plain data copy
  // (msg::MotorCapabilities is a POD message, not a handle) -- it does not
  // reintroduce the "no motor handles inside Drivetrain" constraint the
  // rest of this class observes. The wiring layer (ticket 5) calls this
  // once after binding a motor pair (DEV DT PORTS), querying each bound
  // Motor's own capabilities().
  void setMotorCapabilities(const msg::MotorCapabilities& left,
                             const msg::MotorCapabilities& right);

 private:
  enum class Mode : uint8_t { NEUTRAL, TWIST, WHEELS };

  // Computes this Drivetrain's currently-commanded wheel velocity targets,
  // BEFORE the ratio governor -- kinematics for TWIST, a direct pass-through
  // for WHEELS, zero for NEUTRAL. Shared by tick() (which then governs the
  // result against live observations) and state() (which has no
  // observations to govern against, and reports the pre-governor commanded
  // targets as-is).
  void commandedWheelTargets(float* targetLeft, float* targetRight) const;

  // Ratio (sync) governor: if a wheel underachieves its target (bogged
  // down), lower the shared speed ceiling so left/right hold their
  // commanded ratio (curvature), instead of letting the healthy wheel run
  // away. Ported CONCEPT (not byte-for-byte -- architecture-update.md does
  // not require that here, unlike ticket 3's split-phase encoder
  // sequencing) from source_old/control/MotorController.cpp's syncGain
  // cross-wheel coupling, re-targeted at velocity TARGETS instead of the
  // duty-cycle PWM output it originally adjusted.
  // DrivetrainConfig.sync_gain is the tuning knob (0 = fully independent).
  void governRatio(float* targetLeft, float* targetRight,
                    const msg::MotorState& leftObs,
                    const msg::MotorState& rightObs) const;

  msg::DrivetrainConfig config_ = {};
  Mode mode_ = Mode::NEUTRAL;

  // TWIST-arm state.
  float v_x_ = 0.0f;      // [mm/s]
  float v_y_ = 0.0f;      // [mm/s] always ignored this sprint -- see setTwist()
  float omega_ = 0.0f;    // [rad/s]

  // WHEELS-arm state.
  float wheelTargetLeft_ = 0.0f;    // [mm/s]
  float wheelTargetRight_ = 0.0f;   // [mm/s]

  msg::Neutral neutralMode_ = msg::Neutral::BRAKE;

  msg::MotorCapabilities leftMotorCaps_ = {};
  msg::MotorCapabilities rightMotorCaps_ = {};
};

}  // namespace Subsystems
