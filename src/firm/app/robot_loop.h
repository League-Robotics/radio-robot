// robot_loop.h -- App::RobotLoop: the boot loop and main per-cycle schedule.
// Compiles under -DHOST_BUILD (no MicroBit.h) via the Devices::Clock&/
// Devices::Sleeper& time seam instead of raw vendor timer/sleep calls.
//
// Two entry points: run() is what main.cpp calls -- boot() once, then
// cycle() forever (never returns). A host test instead calls boot() and
// cycle() directly so it can step a bounded number of cycles and inspect
// state in between.
//
// Timing primitives: runAndWait(gap, body) == markTime(); body();
// sleepUntil(mark, gap) -- the block visibly scopes exactly the work that
// borrows the wait; the body itself never touches the bus and never
// sleeps. `grep 'runAndWait\|sleepUntil'` on this file is the firmware's
// complete timing schedule. Built on Devices::Clock::nowMicros() (converted
// [us] -> [ms]) and Devices::Sleeper::sleepMillis() -- see devices/clock.h's
// own file header for the real vs. HOST_BUILD impls.
//
// Design/rationale: DESIGN.md.
#pragma once

#include <cstdint>

#include "app/comms.h"
#include "app/deadman.h"
#include "app/drive.h"
#include "app/odometry.h"
#include "app/pilot.h"
#include "app/preamble.h"
#include "app/telemetry.h"
#include "config/persisted_tuning.h"
#include "devices/clock.h"
#include "devices/i2c_bus.h"
#include "devices/motor.h"
#include "devices/otos.h"

namespace App {

class RobotLoop {
 public:
  // Every reference below is an already-constructed leaf/app module the
  // cycle body touches by name (main.cpp on ARM, or a host harness, owns
  // construction and wiring order). bus is needed directly for the cycle
  // body's own bus.clearanceSafetyNetCount() fault read; color/line leaves
  // are NOT referenced here -- Preamble already holds them and is called by
  // name, never reached into directly.
  // tuningStore (114-004, SUC-003) -- the persisted-live-tuning seam;
  // trailing and defaulted to nullptr so every EXISTING call site (main.cpp,
  // and every one of TestSim::SimHarness's construction sites) keeps
  // compiling unchanged. Real firmware's main.cpp passes a real
  // Config::MicroBitTuningStore*; a null store means "persistence
  // disabled," which is every sim/test composition root's own case
  // (sprint.md: "the sim has no flash ... vacuous by construction") --
  // handleConfig()'s own write-policy check (persistTuningIfChanged())
  // no-ops entirely when this is null, doing zero extra work per CONFIG
  // dispatch on a composition root that never configured one.
  RobotLoop(Devices::I2CBus& bus, Devices::Motor& motorL,
            Devices::Motor& motorR, Devices::Otos& otos, Comms& comms,
            Telemetry& tlm, Drive& drive, Odometry& odom, Deadman& deadman,
            Preamble& preamble, Pilot& pilot, const Devices::Clock& clock,
            Devices::Sleeper& sleeper,
            Config::TuningStore* tuningStore = nullptr);

  // Runs boot() once, then cycle() forever. Never returns -- this is what
  // main.cpp's int main() calls after constructing real hardware.
  [[noreturn]] void run();

  // Boot loop: `preamble.step()` until `preamble.done()`, staging/emitting
  // a boot telemetry frame each pass and pacing via
  // sleeper_.sleepMillis(kPreamblePace). Sets kEventBootReady on the
  // done() first-true transition, then returns.
  void boot();

  // One pass of the main cycle body (the runAndWait/markTime/sleepUntil
  // schedule and the command-dispatch switch). Call boot() first --
  // cycle() assumes every device is already resolved; no readiness checks
  // happen below this line.
  void cycle();

  // Configuration-completeness gate (114-001, SUC-001). markConfigured()
  // is called EXACTLY ONCE by whichever atomic boot path configured the
  // whole graph -- main.cpp's own Config::default*() sequence (real
  // firmware; always immediate, since the boot bake completes before
  // run() starts), or TestSim::SimHarness's configurePlanner()+
  // configureMotor() pair (sim/test composition roots). Idempotent: a
  // second call is a harmless no-op, so a caller that fans out over
  // multiple config calls (SimHarness) may call it from whichever call
  // completes the set. handleTwist()/handleMove() refuse (ERR_NOT_CONFIGURED)
  // until this has fired; handleStop()/handleConfig() stay unconditional.
  void markConfigured() { configured_ = true; }
  bool isConfigured() const { return configured_; }

  // reapplyPersistedTuning (114-004, SUC-003) -- applies a TuningSnapshot
  // loaded from a Config::TuningStore, through the SAME per-kind appliers
  // handleConfig() itself uses for a live wire CFG patch (no duplicated
  // merge-then-write logic). Also seeds persistedTuning_/lastPersistedBlob_
  // so a SUBSEQUENT live patch's own write-policy change-detection compares
  // against what was just reapplied, not an empty baseline (which would
  // force an immediate, redundant re-save of exactly what was just
  // loaded). Called by main.cpp's boot sequence ONLY when
  // Config::shouldWipe() was false for the loaded version -- a caller that
  // wipes instead never calls this at all (SUC-003: "wipe, proceed on
  // boot-bake alone").
  void reapplyPersistedTuning(const Config::TuningSnapshot& snapshot);

 private:
  uint32_t markTime() const;                    // [ms]
  void sleepUntil(uint32_t mark, uint32_t gap);  // [ms] [ms]

  template <typename Body>
  void runAndWait(uint32_t gap, Body body);  // [ms]



  // Update tlm_ from bus_/motorL_/motorR_/ comms_.
  void updateTlm();

  // Dispatches the <=1 decoded command in cmd to its own handler by
  // cmd_kind (NONE is a no-op). Each handler applies its command and acks
  // via the telemetry ack ring.
  void processMessage(const Cmd& cmd);
  void handleTwist(const msg::CommandEnvelope& env);
  void handleConfig(const msg::CommandEnvelope& env);
  void handleStop(const msg::CommandEnvelope& env);
  void handleMove(const msg::CommandEnvelope& env);

  // --- CONFIG appliers (114-004) -- the merge-then-apply logic
  // handleConfig()'s own MOTOR/OTOS branches use, factored out so
  // reapplyPersistedTuning() (boot-triggered) and handleConfig()
  // (wire-triggered) share exactly one applier per patch kind. PLANNER's
  // own applier already existed (pilot_.applyPlannerPatch()) -- reused
  // directly, not duplicated here. ---

  // applyMotorConfigPatch -- UNCHANGED extraction of handleConfig()'s own
  // prior MOTOR-branch logic: kp/ki/kff/i_max/kaw mirror onto BOTH
  // motorL_/motorR_ when present; travel_calib applies to the side
  // `patch.side` addresses only.
  void applyMotorConfigPatch(const msg::MotorConfigPatch& patch);

  // applyOtosPatch -- UNCHANGED extraction of handleConfig()'s own prior
  // OTOS-branch logic (scale setters, merge-then-write offset triple,
  // trigger init()).
  void applyOtosPatch(const msg::OtosConfigPatch& patch);

  // persistTuningIfChanged -- 114-004 write policy (sprint.md Open
  // Question 3). See robot_loop.cpp's own doc comment for the
  // change-detection rationale.
  void persistTuningIfChanged();

  // Drains every pending Motion::Executor completion event (bounded --
  // Motion::kEventRingDepth) into Telemetry's ack ring, keyed by each
  // event's own command id (Move.id), not the enqueueing envelope's
  // corr_id -- see pilot.h/executor.h's own doc comments.
  void drainPilotEvents();

  Devices::I2CBus& bus_;
  Devices::Motor& motorL_;
  Devices::Motor& motorR_;
  Devices::Otos& otos_;
  Comms& comms_;
  Telemetry& tlm_;
  Drive& drive_;
  Odometry& odom_;
  Deadman& deadman_;
  Preamble& preamble_;
  Pilot& pilot_;
  const Devices::Clock& clock_;
  Devices::Sleeper& sleeper_;

  // Persists across cycle() calls. Each field is written by the part of
  // the cycle that owns it (encoder/vel/conn after motorL_/motorR_'s own
  // tick(); pose after odom_.integrate(); otos via applyOtosSample()) and
  // read back whole by the NEXT cycle's tlm_.setFrame()/emit() call --
  // Telemetry always carries the last staged snapshot, so a field updated
  // late in one cycle is simply one cycle "stale" when it reaches the
  // wire, never lost.
  bool driving_ = false;  // true once a Twist is applied, cleared on Stop/deadman
  Telemetry::Frame frame_;

  // Configuration-completeness gate (114-001) -- see markConfigured()/
  // isConfigured() above for the contract. false until markConfigured()
  // fires; never reset back to false afterward (a composition root is
  // configured for its whole lifetime once it is configured at all).
  bool configured_ = false;

  // --- Persisted live-tuning (114-004, SUC-003) ---

  // Null on every sim/test composition root (see the constructor's own
  // doc comment); real firmware's main.cpp always passes a real
  // Config::MicroBitTuningStore*.
  Config::TuningStore* tuningStore_ = nullptr;

  // The running, cumulative merge of every live-tunable field touched by
  // a CFG patch (or a boot-time reapply) since this composition root
  // started -- NOT a copy of the last incoming patch alone. Each field
  // starts Opt<T>{has=false}; handleConfig() merges only the PRESENT
  // fields of each new patch into this, the same merge-then-write shape
  // gains/travel_calib/offset already use. serializeSnapshot(persistedTuning_)
  // is what actually reaches flash.
  Config::TuningSnapshot persistedTuning_ = {};

  // The last blob actually written via tuningStore_->save() -- the write
  // policy's own change-detection baseline (persistTuningIfChanged()).
  // Starts all-zero, matching a fresh persistedTuning_'s own serialized
  // form exactly (nothing tuned yet == nothing to persist yet).
  Config::Blob lastPersistedBlob_ = {};
};

}  // namespace App
