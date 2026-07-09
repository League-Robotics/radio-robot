// main_loop.cpp -- Rt::MainLoop: see main_loop.h for the class-level
// contract. Post-093 gut: no watchdog check, no pose/planner ticks, no
// loop-originated wire output -- tick() is Hardware.tick() -> Drivetrain.
// tick() -> commit, in that order, every pass. (093/094 teardown) There is
// no routeOutputs() step any more -- Drivetrain's held output currently
// goes nowhere; see main_loop.h's file header.
#include "runtime/main_loop.h"

namespace Rt {

MainLoop::MainLoop(Subsystems::Hardware& hardware, Subsystems::Drivetrain& drivetrain)
    : hardware_(hardware), drivetrain_(drivetrain) {}

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
  hardware_.tick(now);
  drivetrain_.tick(now, bb.motors, kPortCount, bb.driveIn);

  // === COMMIT (clock edge): x[k] -> x[k+1]. ===
  commit(bb, now);
}

}  // namespace Rt

