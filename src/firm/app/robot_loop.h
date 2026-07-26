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
#include "app/drive.h"
#include "app/preamble.h"
#include "app/telemetry.h"
#include "config/persisted_tuning.h"
#include "devices/clock.h"
#include "devices/color_sensor.h"
#include "devices/i2c_bus.h"
#include "devices/line_sensor.h"
#include "devices/motor.h"
#include "devices/otos.h"
#include "firm/types/robot_state.h"
#include "motion/move_queue.h"
#include "motion/odometry.h"
#include "motion/state_estimator.h"

namespace App {

class RobotLoop {
 public:
  // kCycle -- the whole-schedule pace target for one cycle() call (~25 Hz;
  // 106-001, restored by 118 after commit 5f5a2ba7's regression -- see
  // robot_loop.cpp's own file-scope comment for the timing-budget
  // derivation of kSettle/kClear/kWindows/kPace from this value). Public
  // (unlike those other schedule constants, which stay robot_loop.cpp
  // implementation details) so any composition root that must run at
  // firmware's OWN control period -- most notably
  // TestSim::SimHarness::kCycleDtUs (sim_harness.h), the sim's per-step
  // virtual-time advance -- derives its own constant from this ONE
  // declaration instead of an independently-hardcoded matching literal that
  // can drift apart again silently (118 ticket 003,
  // sim-cycle-must-match-firmware-period.md).
  static constexpr uint32_t kCycle = 40;  // [ms] whole-schedule pace target (~25 Hz)

  // Every reference below is an already-constructed leaf/app module the
  // cycle body touches by name (main.cpp on ARM, or a host harness, owns
  // construction and wiring order). bus is needed directly for the cycle
  // body's own bus.clearanceSafetyNetCount() fault read. color/line
  // (115-005, gut S1) ARE referenced here directly -- Preamble still owns
  // detecting their PRESENCE at boot (called by name, never reached into),
  // but this class's own kPace block calls each leaf's own
  // readDue()/tick()/reading() directly for rate-limited, alternating
  // steady-state sampling -- see cycle()'s own comment at that block.
  // stateEstimator (117) -- App::StateEstimator&, threaded through exactly
  // like MoveQueue&/Preamble& above (an already-constructed passive module
  // the cycle body touches by name; the composition root -- main.cpp on
  // ARM, src/sim/sim_harness.h, or a host harness -- owns construction and
  // wiring order). handleConfig()'s own ESTIMATOR branch (ticket 003)
  // merges a live EstimatorConfigPatch's present fields onto
  // stateEstimator_.weights() and calls setWeights(); RobotLoop::cycle()'s
  // trailing kPace block calls stateEstimator_.update(state_, nowMs) once
  // per cycle, immediately after state_.pose is published (124-009:
  // state_ IS Motion::StateEstimator::Input now -- no separate hand-copy)
  // -- see DESIGN.md's "Predict-to-now estimation" note.
  //
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
            Devices::Motor& motorR, Devices::Otos& otos,
            Devices::ColorSensorLeaf& color, Devices::LineSensorLeaf& line,
            Comms& comms, Telemetry& tlm, Drive& drive, Motion::Odometry& odom,
            Motion::MoveQueue& moveQueue, Preamble& preamble,
            Motion::StateEstimator& stateEstimator,
            const Devices::Clock& clock, Devices::Sleeper& sleeper,
            Config::TuningStore* tuningStore = nullptr);

  // Runs boot() once, then cycle() forever. Never returns -- this is what
  // main.cpp's int main() calls after constructing real hardware.
  [[noreturn]] void run();

  // Boot loop: `preamble.step()` until `preamble.done()`, staging/emitting
  // a boot telemetry frame each pass and pacing via
  // sleeper_.sleepMillis(kPreamblePace). Sets kFlagEventBootReady on the
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
  // run() starts), or TestSim::SimHarness's own composition-time config
  // calls (sim/test composition roots). Idempotent: a second call is a
  // harmless no-op, so a caller that fans out over multiple config calls
  // (SimHarness) may call it from whichever call completes the set.
  // handleMove() refuses (ERR_NOT_CONFIGURED) until this has fired;
  // handleStop()/handleConfig() stay unconditional.
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

  // clampToPositionWireBound -- pure helper for the position-rebaseline
  // policy's defensive fallback (124-008, sprint 124 architecture
  // Decision 6): given a wheel's current position (mm), returns it
  // unchanged if within the wire's declared EncoderReading.position
  // (abs_max) bound, or clamped to +-that bound with *clamped set true
  // otherwise. Exposed as a static, side-effect-free method (rather than
  // kept file-local in robot_loop.cpp) purely so it is unit-testable in
  // isolation: under the CURRENT margin/bound relationship
  // (kPositionRebaselineMargin, robot_loop.cpp, is strictly less than the
  // bound, and Devices::Motor::rebaseline() unconditionally zeroes
  // position() on return) the wheel-section publish point in cycle() can
  // never actually reach the clamped branch through a live Motor -- this is
  // insurance against that invariant changing later, not the expected
  // path today. A direct test calls this method with an out-of-bound
  // value, bypassing rebaseline() entirely, to prove the clamp itself is
  // correct independent of whether production code can currently reach
  // it.
  static float clampToPositionWireBound(float pos, bool* clamped);

 private:
  uint32_t markTime() const;                    // [ms]
  void sleepUntil(uint32_t mark, uint32_t gap);  // [ms] [ms]

  template <typename Body>
  void runAndWait(uint32_t gap, Body body);  // [ms]

  // Dispatches the <=1 decoded command in cmd to its own handler by
  // cmd_kind (NONE is a no-op). Each handler applies its command and acks
  // via tlm_.ack().
  void processMessage(const Cmd& cmd);
  void handleMove(const msg::CommandEnvelope& env);
  void handleConfig(const msg::CommandEnvelope& env);
  void handleStop(const msg::CommandEnvelope& env);

  // --- CONFIG appliers (114-004) -- the merge-then-apply logic
  // handleConfig()'s own MOTOR/OTOS branches use, factored out so
  // reapplyPersistedTuning() (boot-triggered) and handleConfig()
  // (wire-triggered) share exactly one applier per patch kind. ---

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

  Devices::I2CBus& bus_;
  Devices::Motor& motorL_;
  Devices::Motor& motorR_;
  Devices::Otos& otos_;
  Devices::ColorSensorLeaf& color_;
  Devices::LineSensorLeaf& line_;
  Comms& comms_;
  Telemetry& tlm_;
  Drive& drive_;
  Motion::Odometry& odom_;
  Motion::MoveQueue& moveQueue_;
  Preamble& preamble_;
  Motion::StateEstimator& stateEstimator_;
  const Devices::Clock& clock_;
  Devices::Sleeper& sleeper_;

  // The blackboard (124-009, robot-state-blackboard-one-struct-...md):
  // persists across cycle() calls, exactly like frame_ used to. Each
  // section is written by the part of the cycle that owns it, published
  // at the earliest point it is internally coherent (wheels immediately
  // after both L/R collects; otos/perception/pose/command/health in the
  // trailing kPace block, in dependency order) -- see cycle()'s own
  // comments at each publish point. tlm_.update(state_) is the ONE place
  // this whole struct is read (SUC-004's single-assembly-point contract);
  // no other consumer exists yet this sprint (Decision 1 defers the
  // Drive/Sensors ownership reshuffle that would add one, to 125).
  //
  // No `driving_` hand-toggled bool (116, protocol-set-point issue):
  // state_.command.moveActive derives from moveQueue_.active() directly --
  // MoveQueue is the single source of truth for whether motion is in
  // progress, matching the deleted Deadman-era bool's own set/clear call
  // sites one-for-one (activate/flush/timeout-drain).
  Types::RobotState state_;

  // Position-rebaseline policy (124-008, sprint 124 architecture Decision
  // 6): per-wheel software-rebaseline generation counters, owned and
  // incremented HERE (never by Devices::Motor/NezhaMotor/MotorArmor, which
  // this ticket leaves unmodified) -- an 8-bit wrap-around counter
  // suffices (Decision 6's own sizing). See cycle()'s own comment at the
  // wheel-section publish point (the trigger call site).
  uint8_t positionEpochLeft_ = 0;
  uint8_t positionEpochRight_ = 0;

  // Line/color alternation cursor (115-005) -- true means the NEXT pass
  // ticks line_, false means it ticks color_. Owned by cycle()'s own
  // kPace-block body (the tick itself is a plain inline block, not a
  // separate private method); this cursor is the only piece of that
  // cadence that must persist across cycle() calls.
  bool lineTurnNext_ = true;

  // --- Loop-timing telemetry (122-003, cycle_busy/cycle_period -- staged
  // onto the PRIMARY frame since 123-004 -- see telemetry.h's Frame doc
  // comment for the full migration history). previousCycleStartUs_/
  // everCycled_ track cycle()'s OWN call history in [us] (independent of
  // markTime()'s [ms]-truncated cycleStart, which has too little
  // resolution for a diagnostic meant to surface sub-millisecond I2C
  // stalls) -- NOT touched by boot(), which never calls cycle(), so the
  // first-ever cycle() call always reports cyclePeriod == 0 (no previous
  // cycle() call exists yet). Unchanged mechanism from 122-003 -- only the
  // destination (state_.time.cycleBusy/cyclePeriod, 124-009) moved.
  uint64_t previousCycleStartUs_ = 0;  // [us]
  bool everCycled_ = false;

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
