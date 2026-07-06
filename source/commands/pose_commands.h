#pragma once

// ---------------------------------------------------------------------------
// pose_commands.h -- the pose-set command family (084-007, SUC-006): `SI`
// (re-anchor the believed world pose) and `ZERO` (rezero the bound pair's
// encoders -- this ticket implements the already-documented `enc` sub-verb
// only; docs/protocol-v2.md section 10's "### ZERO"). Both are production
// protocol-v2 verbs (architecture-update.md (084) Grounding facts 4/5),
// gated under ROBOT_DEV_BUILD for the same reason motion_commands.*/
// config_commands.* are: this sprint's new source/ tree IS the dev firmware
// (no separate production loop exists yet), so every command family that
// touches Hardware/Drivetrain/PoseEstimator compiles in only under that
// define -- see dev_commands.h's file header for the full rationale.
//
// **Decision 1 (architecture-update.md (084)):** `SI` calls
// `Subsystems::PoseEstimator::setPose()` DIRECTLY -- it does NOT route
// through `Subsystems::Drivetrain::apply()`'s existing `POSE`/`SetPose`
// oneof arm, which stays exactly the documented no-op it is today
// (subsystems/drivetrain.cpp's `POSE` case). `Drivetrain` holds no
// `PoseEstimator` reference by design (082's cohesion split) and must not
// gain one just for this. See that decision's own Context/Alternatives/Why/
// Consequences for the full rationale -- do not re-route `SI` through
// `Drivetrain` without revisiting that decision.
//
// `ZERO enc` reuses the SAME bound-pair-encoder-reset primitive `DEV M <n>
// RESET` already exercises (`Hal::Motor::resetPosition()` -- a concrete,
// synchronous-TO-CALL method: it only stages `resetPending_ = true`: see
// hal/capability/motor.h's own doc comment, "zero encoder (staged, not
// immediate)" -- the actual hardware effect lands at the top of the leaf's
// next tick(), not through any outbox this file has to drain), applied to
// BOTH of `Drivetrain::ports()`'s currently-bound motors, PLUS a new call
// into `PoseEstimator::resetEncoderBaseline()` so the pose estimator's own
// encoder-delta baseline is resynced in the SAME wire dispatch -- see that
// method's doc comment (pose_estimator.h) for the phantom-jump hazard this
// prevents.
//
// SI/ZERO are synchronous, config-plane-shaped verbs, like SET/GET (084-006)
// -- no outbox, no per-pass drain step (architecture-update.md (084)'s
// "Consequences" section: "SET/GET/SI/ZERO/OTOS are all synchronous,
// config-plane, or one-shot, with nothing to hold across a tick boundary").
// PoseCommandState is therefore NOT added to DevLoop (source/dev_loop.h) --
// mirrors ConfigCommandState's own placement (config_commands.h).
// ---------------------------------------------------------------------------

#if ROBOT_DEV_BUILD

#include <vector>

#include "command_types.h"
#include "subsystems/drivetrain.h"
#include "subsystems/hardware.h"
#include "subsystems/pose_estimator.h"

// ---------------------------------------------------------------------------
// PoseCommandState -- see this file's header comment. Every pointer must be
// set before poseCommands()'s handlers are called -- mirrors
// configCommands()'s/motionCommands()'s own contract.
// ---------------------------------------------------------------------------
struct PoseCommandState {
  Subsystems::Hardware* hardware = nullptr;
  Subsystems::Drivetrain* drivetrain = nullptr;
  Subsystems::PoseEstimator* poseEstimator = nullptr;
};

// Returns the pose-set command table (SI, ZERO), bound to the given shared
// state.
std::vector<CommandDescriptor> poseCommands(PoseCommandState& state);

#endif  // ROBOT_DEV_BUILD
