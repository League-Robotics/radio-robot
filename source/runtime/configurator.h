// configurator.h -- Rt::Configurator: sprint 087's single config-application
// authority (architecture-update-r1.md Step 3/Decision 4). Constructed with
// references to Subsystems::Drivetrain/PoseEstimator/Hardware -- the ONE
// deliberate exception to "no subsystem pointers outside the loop" this
// design is organized around (every other runtime/subsystems component
// depends only on Rt::Blackboard's queue/state types, never a concrete
// subsystem reference).
//
// 094-002: Subsystems::Planner was relocated out of source/ entirely (see
// source_parked/094/subsystems/planner.h's own header note) -- this class no
// longer holds a Planner& or calls its configure(). The kPlanner
// ConfigDelta target still folds onto plannerConfig_ and still publishes
// bb.plannerConfig (msg::PlannerConfig, a wire message type, untouched by
// the class relocation) so a future revival's Configurator wiring has
// somewhere to resume from -- it just never reaches a live subsystem today.
//
// Owns ITS OWN persistent per-target desired-config copy (drivetrainConfig_/
// motorConfig_[]/plannerConfig_/odometerConfig_) -- never re-derives a fold
// baseline from bb's published *Config cells. This is what makes the
// field-masked fold (Rt::ConfigDelta, source/runtime/commands.h) actually
// safe against back-to-back same-target deltas: each applyOne() call folds
// directly onto this persistent copy, so two field-disjoint deltas always
// compose no matter what baseline either delta's CALLER used to build it
// (see commands.h's ConfigDelta comment for the clobber scenario this
// avoids).
//
// applyOne(bb) pops exactly one Rt::ConfigDelta from bb.configIn per call
// (a no-op if empty), folds it (field-masked) into the addressed target's
// desired-config copy, calls that target's own configure() ONLY when the
// fold actually changed something (bitwise-compared before/after), and
// publishes the resulting value into the matching bb.*Config cell
// unconditionally (even on a no-op fold -- cheap, and keeps bb.*Config
// always in sync with this class's own state; see configurator.cpp).
//
// publish(bb) seeds all four bb.*Config cells from this Configurator's
// current per-target values without requiring a delta to have been posted
// first (boot-time use, before the loop starts -- architecture-update-r1.md
// Reference code's `configurator.publish(bb)` call).
//
// pending(bb) is a thin `!bb.configIn.empty()` read, used by the loop's
// slack `else if` branch (ticket 007) to decide whether there is a delta
// worth calling applyOne() for THIS pass.
#pragma once

#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "messages/odometer.h"
#include "messages/planner.h"
#include "runtime/blackboard.h"
#include "subsystems/drivetrain.h"
#include "subsystems/hardware.h"
#include "subsystems/pose_estimator.h"

namespace Rt {

class Configurator {
 public:
  // bootDrivetrainConfig/bootPlannerConfig -- the two boot-default configs
  // architecture-update-r1.md's Reference code originally passed to this
  // constructor (`Configurator configurator(drivetrain, poseEstimator,
  // planner, hardware, Config::defaultDrivetrainConfig(),
  // defaultPlannerConfig());` -- the `planner` argument is gone as of
  // 094-002, see this file's own header note) -- no separate boot
  // MotorConfig/OdometerConfig argument, by design:
  //   - Per-port MotorConfig is seeded by READING BACK `hardware.config(p)`
  //     for each port (ticket 087-004's Hardware::config() getter already
  //     holds exactly the same array NezhaHardware/SimHardware's own
  //     constructor was given -- re-passing it here would be a redundant,
  //     driftable second copy).
  //   - OdometerConfig has no boot-config source anywhere in the tree today
  //     (source/commands/otos_commands.h's own file header: "linear_scalar
  //     = angular_scalar = 0.0f. No boot-config generator feeds it") --
  //     odometerConfig_ defaults to a zero-valued msg::OdometerConfig{},
  //     matching that established convention exactly.
  Configurator(Subsystems::Drivetrain& drivetrain, Subsystems::PoseEstimator& poseEstimator,
               Subsystems::Hardware& hardware,
               const msg::DrivetrainConfig& bootDrivetrainConfig,
               const msg::PlannerConfig& bootPlannerConfig);

  // Pops exactly one Rt::ConfigDelta from bb.configIn (never more) --
  // see the class comment above for the full apply/publish contract.
  void applyOne(Blackboard& bb);

  // Seeds all four bb.*Config cells from this Configurator's own current
  // per-target values -- boot-time use, before any delta has been posted.
  void publish(Blackboard& bb);

  // True iff bb.configIn has at least one undrained delta.
  bool pending(const Blackboard& bb) const;

 private:
  Subsystems::Drivetrain& drivetrain_;
  Subsystems::PoseEstimator& poseEstimator_;
  Subsystems::Hardware& hardware_;

  // This Configurator's OWN persistent per-target desired-config copies --
  // the single source of truth folded onto and published from. Never read
  // back from bb (see class comment).
  msg::DrivetrainConfig drivetrainConfig_ = {};
  msg::MotorConfig motorConfig_[Subsystems::Hardware::kMotorCount] = {};
  msg::PlannerConfig plannerConfig_ = {};
  msg::OdometerConfig odometerConfig_ = {};
};

}  // namespace Rt
