// robot_loop.h -- App::RobotLoop: the boot loop and main per-cycle
// schedule. run() = boot() once then cycle() forever; host tests call
// boot()/cycle() directly. Timing goes through the Devices::Clock/Sleeper
// seam so this compiles under -DHOST_BUILD. Design: DESIGN.md.
//
// Command ingestion (command-ingestion-ring-buffered-comms-subsystem-
// routing-two-stops.md): the loop PUMPS App::Comms several times per cycle
// -- once inside each existing runAndWait body -- so neither transport's
// own small buffer is the backpressure, then DRAINS the resulting command
// ring once per cycle through routeCommand(). RobotLoop decides only WHERE
// each command goes; what it means is the destination subsystem's job:
//
//     WHEELS -> App::Drive       (dumb, time-bounded teleop)
//     MOVE   -> Motion::Planner  (the motion queue)
//     STOP   -> Motion::Planner  (a PLANNED stop, queued in sequence)
//     ESTOP  -> Drive + Planner  (halt now, both)
//     CONFIG -> App::Configurator
//
// Exactly one subsystem owns motion at a time, and that is enforced HERE,
// at routing: a WHEELS command clears the planner, a MOVE clears Drive's
// armed command.
#pragma once

#include <cstdint>

#include "app/comms.h"
#include "app/configurator.h"
#include "app/drive.h"
#include "app/preamble.h"
#include "app/telemetry.h"
#include "devices/clock.h"
#include "devices/color_sensor.h"
#include "devices/i2c_bus.h"
#include "devices/line_sensor.h"
#include "devices/motor.h"
#include "devices/otos.h"
#include "firm/types/robot_state.h"
#include "motion/odometry.h"
#include "motion/planner/planner.h"

namespace App {

class RobotLoop {
 public:
  // Whole-schedule pace target for one cycle(). Public so composition
  // roots (TestSim::SimHarness::kCycleDtUs) derive from this one
  // declaration instead of a drifting duplicate literal.
  static constexpr uint32_t kCycle = 40;  // [ms] (~25 Hz)

  // All references are already-constructed modules; the composition root
  // (main.cpp or a harness) owns construction and wiring order. The
  // persisted-tuning store is NOT a parameter here any more -- it belongs
  // to App::Configurator, which owns the whole CONFIG lifecycle.
  RobotLoop(Devices::I2CBus& bus, Devices::Motor& motorL,
            Devices::Motor& motorR, Devices::Otos& otos,
            Devices::ColorSensorLeaf& color, Devices::LineSensorLeaf& line,
            Comms& comms, Telemetry& tlm, Drive& drive,
            Configurator& configurator, Motion::Odometry& odom,
            Motion::Planner& planner, Preamble& preamble,
            const Devices::Clock& clock, Devices::Sleeper& sleeper);

  [[noreturn]] void run();

  // Boot loop: probe devices until Preamble::done(), emitting boot frames.
  void boot();

  // One pass of the main cycle. Call boot() first -- no readiness checks.
  void cycle();

  // Configuration-completeness gate: handleMove()/handleWheels() refuse
  // until this fires (once, idempotent).
  void markConfigured() { configured_ = true; }
  bool isConfigured() const { return configured_; }

  // setRotationCalibration -- the measured affine turn response,
  // `actual = gain * commanded + offset`, per direction of rotation.
  // handleMove() inverts it so an ANGLE-stopped move LANDS on the angle the
  // host asked for. Seeded from the robot JSON by main.cpp, mirroring how
  // Drive gets its wheel calibration; the identity (gain 1, offset 0) is
  // deliberately the default, so an uncalibrated robot behaves exactly as
  // before rather than being silently skewed.
  //
  // offsets are RADIANS here (the robot JSON carries degrees; main.cpp
  // converts) so this matches Move::threshold's own units.
  void setRotationCalibration(float gainPos, float offsetPos,
                              float gainNeg, float offsetNeg) {
    rotGainPos_ = gainPos;
    rotOffsetPos_ = offsetPos;
    rotGainNeg_ = gainNeg;
    rotOffsetNeg_ = offsetNeg;
  }

  // Clamp a wheel position to the wire bound; static for isolated testing.
  static float clampToPositionWireBound(float pos, bool* clamped);

  // Read-only view of the blackboard (sim/test observability).
  const Types::RobotState& state() const { return state_; }

 private:
  uint32_t markTime() const;                    // [ms]
  void sleepUntil(uint32_t mark, uint32_t gap);  // [ms] [ms]

  template <typename Body>
  void runAndWait(uint32_t gap, Body body);  // [ms]

  // Drain the command ring and dispatch each entry to its owning
  // subsystem. Every path acks -- either the outcome of the routing itself
  // (enqueue/ERR_FULL/ERR_BADARG) or, for the queued kinds, a later
  // completion ack from the destination.
  void routeCommand(const Cmd& cmd);
  void handleMove(const msg::CommandEnvelope& env);
  void handleWheels(const msg::CommandEnvelope& env);
  void handleStop(const msg::CommandEnvelope& env);
  void handleEstop(const msg::CommandEnvelope& env);

  // MOVE enqueue is idempotent on Move.id: a host that retries an enqueue
  // whose ack was lost re-sends the SAME id under a fresh corr_id, and
  // without this the move would execute twice. Move.id 0 means "unset" (the
  // host-side default) and is never matched or recorded -- deduplicating it
  // would drop every default-id move after the first.
  bool alreadyAccepted(uint32_t id) const;
  void recordAccepted(uint32_t id);

  // Boot-window commands are NACKed (ERR_NOT_CONFIGURED), never dropped.
  void rejectDuringBoot(const Cmd& cmd);

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
  Configurator& configurator_;
  Motion::Odometry& odom_;
  Motion::Planner& planner_;
  Preamble& preamble_;
  const Devices::Clock& clock_;
  Devices::Sleeper& sleeper_;

  // The blackboard: each section written by the cycle step that owns it,
  // at its point of coherence; tlm_.update(state_) is the one assembly
  // point.
  Types::RobotState state_;

  // Per-wheel software-rebaseline generation counters.
  uint8_t positionEpochLeft_ = 0;
  uint8_t positionEpochRight_ = 0;

  // Parity picks line vs color in the pace block.
  //
  // KNOWN DEFECT, deliberately left alone by the command-ingestion rework:
  // nothing increments this. It has been stuck at 0 since the counter was
  // introduced, so `(cycleCount_ % 2) == 1` is permanently false and the
  // LINE sensor is never ticked -- only the color sensor is. The one-line
  // fix is real but it adds an I2C transaction to every other pace block,
  // which shifts the loop period the motion tuning is calibrated against;
  // that belongs to its own change with its own bench measurement, not to
  // a command-plane rework whose gate is a square tour.
  uint32_t cycleCount_ = 0;

  // cycle() call history for cycleBusy/cyclePeriod telemetry.
  uint64_t previousCycleStartUs_ = 0;  // [us]
  bool everCycled_ = false;

  // Move.ids accepted so far, newest overwriting oldest. The window must
  // outlive completion: the retry being defended against can land after the
  // original move has already run, so ids are not evicted when a move leaves
  // the planner queue. Sized well above the 5-deep queue; a long session
  // eventually wraps and could re-accept a very old id, which is harmless
  // because the retry window is seconds and hosts assign ids monotonically.
  static constexpr int kAcceptedMoveIdCount = 16;
  uint32_t acceptedMoveIds_[kAcceptedMoveIdCount] = {};  // 0 = empty slot
  int acceptedMoveIdNext_ = 0;

  bool configured_ = false;

  // Turn calibration; identity until main.cpp seeds it from the robot JSON.
  // See setRotationCalibration(). Offsets are [rad], matching
  // Motion::Move::threshold.
  float rotGainPos_ = 1.0f;
  float rotOffsetPos_ = 0.0f;
  float rotGainNeg_ = 1.0f;
  float rotOffsetNeg_ = 0.0f;
};

}  // namespace App
