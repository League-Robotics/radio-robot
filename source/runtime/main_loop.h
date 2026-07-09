// main_loop.h -- Rt::MainLoop: sprint 093's gutted cyclic executive
// (clasi/sprints/093-simplify-the-main-loop-bare-wheel-driving-executive/
// architecture-update.md Step 5), replacing the four-subsystem/watchdog/
// reply-sink design ticket 087-007 built and later tickets (088-092) grew.
//
// This class now owns exactly two subsystem references -- `Hardware&` and
// `Drivetrain&` -- and nothing else: no watchdogs, no reply sinks, no
// pose/planner references, no per-verb bookkeeping. Safety supervision (the
// two loop-owned safety-watchdog classes plus the `estop()` bypass), pose
// estimation (the encoder/OTOS fusion classes), motion-goal closure (the
// motion-executor class), and loop-originated wire output (`EVT`/periodic
// `TLM`) are gone from the tick entirely -- not stubbed, deleted (their
// classes remain parked, un-wired, on disk; see the architecture update's
// "Parked" list).
//
// tick(bb, now) is now a four-step, one-pass sequence with no branching
// bookkeeping between steps: tick `Hardware`, tick `Drivetrain`, commit
// their fresh state into `bb`, then route `Drivetrain`'s own output back
// into `bb.motorIn[]`. See main_loop.cpp for the exact sequencing.
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

#if ROBOT_DEV_BUILD

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
  // otos cells remain to copy). Called from tick() immediately before
  // routeOutputs().
  void commit(Blackboard& bb, uint32_t now);

  // routeOutputs -- drains Drivetrain's OWN output edge (hasCommand()/
  // takeCommand()) into bb.motorIn[], gated on drivetrain_.active() (queried
  // AFTER drivetrain_.tick() ran this pass): a bare authority-steal/standby
  // output must never reach hardware. Planner's half of this method (091-
  // era) is deleted along with Planner itself being unwired -- Drivetrain's
  // output is the only edge left to route.
  void routeOutputs(Blackboard& bb);

  Subsystems::Hardware& hardware_;
  Subsystems::Drivetrain& drivetrain_;
};

}  // namespace Rt

#endif  // ROBOT_DEV_BUILD
