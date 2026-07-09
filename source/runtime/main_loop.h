// main_loop.h -- Rt::MainLoop: sprint 093's gutted cyclic executive
// (clasi/sprints/093-simplify-the-main-loop-bare-wheel-driving-executive/
// architecture-update.md Step 5), replacing the four-subsystem/watchdog/
// reply-sink design ticket 087-007 built and later tickets (088-092) grew.
// Reordered/shrunk again by sprint 094 ticket 094-005 (see below). This
// class is used by tests/_infra/sim/sim_api.cpp's SimHandle only -- the
// real ARM entry point (source/main.cpp) inlines the identical
// `hardware.tick(now); drivetrain.tick(now, bb.segmentIn, bb.driveIn);
// bb.drivetrain = drivetrain.state();` sequence directly in its own bare
// `for(;;)` loop (the stakeholder's harmonization decision: the Drivetrain
// connects into the bare-loop main() directly, as one line calling tick on
// the drivetrain with the queues from the blackboard) -- keeping this class
// alive lets the sim harness share one mandatory-tick implementation
// instead of hand-mirroring it a second time, while main.cpp itself stays
// the minimal, explicit loop 093 established.
//
// This class owns exactly two subsystem references -- `Hardware&` and
// `Drivetrain&` -- and nothing else: no watchdogs, no reply sinks, no
// pose/planner references, no per-verb bookkeeping. Safety supervision (the
// two loop-owned safety-watchdog classes plus the `estop()` bypass), pose
// estimation (the encoder/OTOS fusion classes), and loop-originated wire
// output (`EVT`/periodic `TLM`) are gone from the tick entirely -- not
// stubbed, deleted (their classes remain parked, un-wired, on disk; see the
// architecture update's "Parked" list). Motion-goal closure is back, in a
// different shape: `Drivetrain` itself now owns a `Motion::SegmentExecutor`
// (094-004) and stages its own output directly through `hardware_` -- there
// is no longer a separate motion-executor subsystem for this loop to tick,
// nor an addressed output for it to route.
//
// tick(bb, now) is now a three-step, one-pass sequence with no branching
// bookkeeping between steps (094-005 deletes the former fourth step,
// routeOutputs() -- nothing is left to route): tick `Hardware` (flushes
// whatever Drivetrain staged onto the motor refs last pass, collects fresh
// encoders), tick `Drivetrain` (drains `bb.segmentIn`/`bb.driveIn`, runs the
// executor/escape-hatch dispatch, stages this pass's setpoints directly
// through `hardware_`'s motor refs), commit their fresh state into `bb`.
// See main_loop.cpp for the exact sequencing and why `Hardware::tick()`
// stays first.
//
// Command ingestion (`CommandRouter::route()`) is, as before, NOT wrapped
// here -- `CommandRouter` is its own top-level object (declared beside
// `Rt::Blackboard`, not nested inside the loop); main.cpp's/sim_api.cpp's
// own slack phase calls it directly. There is no runtime config-application
// authority left to wire in either (093: boot config is applied once,
// directly, at construction) and no watchdog-feed hook (093 removes the
// watchdog itself).
//
// SAFETY NOTE (stakeholder-directed removal, 2026-07-08): this loop no
// longer runs a serial-silence safety watchdog or an `estop()` bypass. That
// is acceptable ONLY because the robot runs mounted on a stand with wheels
// off the ground for the whole of this loop's current operating envelope
// (`.claude/rules/hardware-bench-testing.md`) -- see architecture-update.md
// Decision 2. This is a standing risk the moment this firmware is ever run
// off the stand.
#pragma once

#include "runtime/blackboard.h"
#include "subsystems/drivetrain.h"
#include "subsystems/hardware.h"


namespace Rt {

class MainLoop {
 public:
  MainLoop(Subsystems::Hardware& hardware, Subsystems::Drivetrain& drivetrain);

  // The mandatory control tick + commit -- see main_loop.cpp for the exact
  // per-subsystem sequencing. Called once per outer pass by BOTH main.cpp
  // (real hardware) and tests/_infra/sim/sim_api.cpp (SimHardware) -- the
  // 1:1-mirror invariant (both wiring sites share this ONE function).
  void tick(Blackboard& bb, uint32_t now);

 private:
  // commit -- copies each subsystem's freshly-ticked state into bb ->
  // x[k+1] (bb.motors[]/bb.drivetrain only, post-093 -- no pose/planner/
  // otos cells remain to copy). Called from tick() as its last step
  // (094-005: routeOutputs(), formerly called after this, is deleted --
  // Drivetrain::tick() already staged its own wheel writes directly through
  // hardware_'s motor refs, ahead of commit()).
  void commit(Blackboard& bb, uint32_t now);

  Subsystems::Hardware& hardware_;
  Subsystems::Drivetrain& drivetrain_;
};

}  // namespace Rt

