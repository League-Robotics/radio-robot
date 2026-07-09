// main_loop.cpp -- Rt::MainLoop: see main_loop.h for the class-level
// contract. Post-093 gut: no watchdog check, no pose/planner ticks, no
// loop-originated wire output -- tick() is Hardware.tick() -> Drivetrain.
// tick() -> commit -> routeOutputs, in that order, every pass.
#include "runtime/main_loop.h"


#include "hal/capability/hal_command.h"

namespace Rt {

MainLoop::MainLoop(Subsystems::Hardware& hardware, Subsystems::Drivetrain& drivetrain)
    : hardware_(hardware), drivetrain_(drivetrain) {}

void MainLoop::routeOutputs(Blackboard& bb) {
  // Drivetrain's ONE addressed output, unpacked into per-port bb.motorIn[]
  // -- gated on drivetrain_.active() (queried AFTER drivetrain_.tick() ran
  // this pass, so it reflects whatever THIS pass's driveIn pop/governance
  // just decided): a bare authority-steal/standby output must never reach
  // hardware.
  if (drivetrain_.hasCommand()) {
    Hal::DrivetrainToHardwareCommand cmd = drivetrain_.takeCommand();
    if (drivetrain_.active()) {
      bb.motorIn[cmd.wheel[0].port - 1].post(cmd.wheel[0].command);
      bb.motorIn[cmd.wheel[1].port - 1].post(cmd.wheel[1].command);
    }
  }
}

void MainLoop::commit(Blackboard& bb, uint32_t now) {
  // === COMMIT (clock edge): copy each subsystem cell into bb -> x[k+1]. ===
  for (uint32_t port = 1; port <= kPortCount; ++port) {
    bb.motors[port - 1] = hardware_.state(port);
  }
  bb.drivetrain = drivetrain_.state();
}

void MainLoop::tick(Blackboard& bb, uint32_t now) {
  // === MANDATORY: control. Reads bb (x[k]); consumes commands routed
  //     during the previous slack; each subsystem writes its OWN cell. ===
  hardware_.tick(now, bb.motorIn, bb.motorResetIn);
  drivetrain_.tick(now, bb.motors, kPortCount, bb.driveIn);

  // === COMMIT (clock edge): x[k] -> x[k+1]. ===
  commit(bb, now);

  routeOutputs(bb);
}

}  // namespace Rt

