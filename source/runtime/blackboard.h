// blackboard.h -- Rt::Blackboard: sprint 087's two-plane transport. Owns, as
// plain members, the committed state-plane snapshot x[k] (current-value
// cells: motor/drivetrain/pose/planner observations, current config) and
// every command-plane queue that connects each subsystem (statementsIn,
// driveIn, motorIn[], configIn, poseResetIn, motorResetIn[],
// otosSetPoseIn). Pure data -- no method computes anything; holds NO
// subsystem pointer of any kind (SUC-006). See
// clasi/sprints/087-two-plane-blackboard-synchronous-update-loop-
// configurator-and-command-queue-transport-greenfield/
// architecture-update-r1.md ("The blackboard" Reference code section) for
// the full design; this header is a direct port of that Reference code.
//
// Host-safe by construction (Decision 10). Every member type below is a
// host-safe POD:
//   - the eight state-cell msg::* types are auto-generated
//     (scripts/gen_messages.py) into source/messages/*.h, zero CODAL deps;
//   - Rt::PoseResetCommand / Rt::ConfigDelta are defined in the lightweight,
//     CODAL-free source/runtime/commands.h (an enum, a uint32_t, and one
//     msg::SetPose member -- also generated) -- NOT inline in this header,
//     since ticket 087-004 moved them out so
//     Subsystems::PoseEstimator::tick()'s poseResetIn parameter can name
//     Rt::PoseResetCommand without pose_estimator.h including this file
//     (the "subsystems never include blackboard.h" boundary rule);
//   - Subsystems::Hardware::kPortCount is reachable via subsystems/
//     hardware.h alone, which includes only <stdint.h>, runtime/queue.h,
//     messages/motor.h, and the CODAL-free hal/capability/*.h interfaces;
//   - statementsIn's payload, Subsystems::
//     CommunicatorToCommandProcessorStatement, lives in the CODAL-free
//     source/subsystems/statement.h -- NOT subsystems/communicator.h, which
//     pulls in MicroBit.h/com/radio.h/com/serial_port.h.
// This is what makes Rt::Blackboard instantiable in a host test harness
// (tests/sim/unit/runtime_blackboard_harness.cpp) with the plain system
// C++ compiler -- no ARM toolchain, no MicroBit.h transitively included.
// Any FUTURE addition to Blackboard must be checked against this same
// host-safe-POD bar before being added (architecture-update-r1.md's
// Migration Concerns, Decision 10).
#pragma once

#include <cstdint>

#include "messages/common.h"
#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "messages/odometer.h"
#include "messages/planner.h"
#include "runtime/commands.h"
#include "runtime/queue.h"
#include "subsystems/hardware.h"
#include "subsystems/statement.h"

namespace Rt {

constexpr uint32_t kPortCount = Subsystems::Hardware::kPortCount;  // 4

// Owned by the loop. Holds NO subsystem pointers -- only the committed
// snapshot x[k] (state plane) and the command queues (command plane).
struct Blackboard {
  // === State plane: committed snapshot x[k]. Written ONLY by the loop's
  //     commit step (from each subsystem's state()); read-only during a
  //     pass. ===
  msg::MotorState motor[kPortCount];  // from Hardware
  msg::DrivetrainState drivetrain;    // from Drivetrain
  msg::PoseEstimate encoderPose;      // from PoseEstimator
  msg::PoseEstimate fusedPose;        // from PoseEstimator
  msg::PlannerState planner;          // from Planner
  bool otosValid = false;             // odometer sample present?
  msg::PoseEstimate otos;             // from Hardware, when valid

  // Current config -- published by the Configurator on apply; read by
  // GET/telemetry. Replaces every shadow.
  msg::DrivetrainConfig drivetrainConfig;
  msg::MotorConfig motorConfig[kPortCount];
  msg::PlannerConfig plannerConfig;
  msg::OdometerConfig odometerConfig;

  // === Command plane: queues. Each drained by exactly ONE consumer
  //     (driveIn has two producers -- Decision 1's authority-gated
  //     arbitration). ===
  WorkQueue<Subsystems::CommunicatorToCommandProcessorStatement, 16>
      statementsIn;                          // Communicator -> router
  Mailbox<msg::DrivetrainCommand> driveIn;    // router(DEV DT)/Planner -> Drivetrain
  Mailbox<msg::MotorCommand> motorIn[kPortCount];  // router/routeOutputs -> Hardware
  WorkQueue<ConfigDelta, 16> configIn;        // router -> Configurator
  WorkQueue<PoseResetCommand, 4> poseResetIn;  // router -> PoseEstimator
  bool motorResetIn[kPortCount] = {};         // ZERO enc -> Hardware
  Mailbox<msg::SetPose> otosSetPoseIn;        // SI re-anchor -> odometer
};

}  // namespace Rt
