// main_loop.cpp -- Rt::MainLoop: see main_loop.h for the class-level
// contract. Sprint 094 ticket 094-005 reorders tick() to `hardware_.
// tick(now)` -> `drivetrain_.tick(now, bb.segmentIn, bb.driveIn)` -> commit,
// and deletes routeOutputs() -- Subsystems::Drivetrain (094-004) now stages
// its own wheel writes directly through hardware_'s motor refs, so there is
// nothing left to route.
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
  // === MANDATORY: control. ===
  //
  // hardware_.tick() stays FIRST (its pre-094 position): it flushes
  // whatever Drivetrain STAGED onto the motor refs last pass (via
  // hardware_.motor(port).apply(), inside Drivetrain::tick() below) and
  // collects fresh encoders -- so a setpoint staged THIS pass is flushed
  // the FOLLOWING pass, identical one-pass latency to the pre-094
  // `routeOutputs() -> bb.motorIn[] -> next-pass drain` chain (the
  // load-bearing sequencing decision -- architecture-update.md Section 5,
  // "Loop order"). drivetrain_.tick() then reads FRESH encoders via
  // hardware_.state(), runs its own SegmentExecutor/escape-hatch dispatch,
  // and stages THIS pass's setpoints (flushed next pass by the step
  // above).
  hardware_.tick(now);
  drivetrain_.tick(now, bb.segmentIn, bb.driveIn);

  // === COMMIT (clock edge): x[k] -> x[k+1]. Nothing left to route --
  // Drivetrain already staged its own wheel writes above. ===
  commit(bb, now);
}

}  // namespace Rt
