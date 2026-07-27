// robot_loop.h -- App::RobotLoop: the boot loop and main per-cycle
// schedule. run() = boot() once then cycle() forever; host tests call
// boot()/cycle() directly. Timing goes through the Devices::Clock/Sleeper
// seam so this compiles under -DHOST_BUILD. Design: DESIGN.md.
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
#include "motion/planner/planner.h"
#include "motion/odometry.h"
#include "motion/state_estimator.h"

namespace App {

class RobotLoop {
 public:
  // Whole-schedule pace target for one cycle(). Public so composition
  // roots (TestSim::SimHarness::kCycleDtUs) derive from this one
  // declaration instead of a drifting duplicate literal.
  static constexpr uint32_t kCycle = 40;  // [ms] (~25 Hz)

  // All references are already-constructed modules; the composition root
  // (main.cpp or a harness) owns construction and wiring order.
  // tuningStore may be null (sim/test roots): persistence disabled.
  RobotLoop(Devices::I2CBus& bus, Devices::Motor& motorL,
            Devices::Motor& motorR, Devices::Otos& otos,
            Devices::ColorSensorLeaf& color, Devices::LineSensorLeaf& line,
            Comms& comms, Telemetry& tlm, Drive& drive, Motion::Odometry& odom,
            Motion::Planner& planner, Preamble& preamble,
            Motion::StateEstimator& stateEstimator,
            const Devices::Clock& clock, Devices::Sleeper& sleeper,
            Config::TuningStore* tuningStore = nullptr);

  [[noreturn]] void run();

  // Boot loop: probe devices until Preamble::done(), emitting boot frames.
  void boot();

  // One pass of the main cycle. Call boot() first -- no readiness checks.
  void cycle();

  // Configuration-completeness gate: handleMove() refuses until this
  // fires (once, idempotent).
  void markConfigured() { configured_ = true; }
  bool isConfigured() const { return configured_; }

  // Applies a loaded TuningSnapshot through the same per-kind appliers a
  // live CFG patch uses, and seeds the write-policy baseline.
  void reapplyPersistedTuning(const Config::TuningSnapshot& snapshot);

  // Clamp a wheel position to the wire bound; static for isolated testing.
  static float clampToPositionWireBound(float pos, bool* clamped);

  // Read-only view of the blackboard (sim/test observability).
  const Types::RobotState& state() const { return state_; }

 private:
  uint32_t markTime() const;                    // [ms]
  void sleepUntil(uint32_t mark, uint32_t gap);  // [ms] [ms]

  template <typename Body>
  void runAndWait(uint32_t gap, Body body);  // [ms]

  // Dispatch the <=1 decoded command per cycle; every path acks.
  void processMessage(const Cmd& cmd);
  void handleMove(const msg::CommandEnvelope& env);
  void handleConfig(const msg::CommandEnvelope& env);
  void handleStop(const msg::CommandEnvelope& env);

  // Boot-window commands are NACKed (ERR_NOT_CONFIGURED), never dropped.
  void rejectDuringBoot(const Cmd& cmd);

  // CONFIG appliers, shared by handleConfig() and reapplyPersistedTuning().
  void applyMotorConfigPatch(const msg::MotorConfigPatch& patch);
  void applyOtosPatch(const msg::OtosConfigPatch& patch);

  // Flash write policy: save only when the serialized snapshot changed.
  void persistTuningIfChanged();

  // --- cycle() steps, in schedule order ---

  // Publish one wheel's state section (rebaseline/clamp/read), after its
  // collect. `clamped` reports the defensive wire-bound clamp.
  void publishWheel(Devices::Motor& motor, Types::RobotState::Wheel& wheel,
                    uint8_t& positionEpoch, bool& clamped);
  void publishWheels();               // both wheels + wedge/clamp health
  void publishOtos();
  void publishLineColor(bool tickedLine);
  void publishPose();
  void publishHealth();
  void ackDriveCompletion();
  void stopAll();
  void publishTiming(uint64_t cycleStartUs);  // [us] cycleBusy/cyclePeriod
  // Move-fault flags + completion ack (rides next frame).
  void publishMoveResult(const Motion::TickResult& moveResult);

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
  Motion::Planner& planner_;
  Preamble& preamble_;
  Motion::StateEstimator& stateEstimator_;
  const Devices::Clock& clock_;
  Devices::Sleeper& sleeper_;

  // The blackboard: each section written by the cycle step that owns it,
  // at its point of coherence; tlm_.update(state_) is the one assembly
  // point.
  Types::RobotState state_;

  // Per-wheel software-rebaseline generation counters.
  uint8_t positionEpochLeft_ = 0;
  uint8_t positionEpochRight_ = 0;

  // Incremented at the top of every cycle(); parity picks line vs color.
  uint32_t cycleCount_ = 0;

  // cycle() call history for cycleBusy/cyclePeriod telemetry.
  uint64_t previousCycleStartUs_ = 0;  // [us]
  bool everCycled_ = false;

  bool configured_ = false;

  // Persisted live-tuning: cumulative merge of every tuned field, plus the
  // last blob actually written (change-detection baseline).
  Config::TuningStore* tuningStore_ = nullptr;
  Config::TuningSnapshot persistedTuning_ = {};
  Config::Blob lastPersistedBlob_ = {};
};

}  // namespace App
