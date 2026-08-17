// robot_loop.h -- Core::RobotLoop: the boot loop and main per-cycle
// schedule. run() = boot() once then cycle() forever; host tests call
// boot()/cycle() directly. Timing goes through the Hal::Clock/Sleeper
// seam so this compiles under -DHOST_BUILD. Design: DESIGN.md.
//
// EXPLORATORY-KERNEL REWRITE (2026-08-15,
// clasi/issues/differentialdrive-one-class-one-fiber-exploratory-worktree.md):
// motor control moved out of this loop entirely, into
// Control::DifferentialDrive's OWN fiber -- RobotLoop no longer holds a
// Hal::Motor&, never calls requestSample()/tick() on a motor, and never
// schedules the encoder split-phase settle windows (the kernel's fiber
// does all of that on its own cadence). What used to be Motion::Planner/
// Motion::Navigator/Motion::Odometry -- the whole src/firm/motion/ tree --
// is DELETED; there is no motion queue and no encoder-derived pose any
// more.
//
// Command ingestion: the loop still PUMPS Core::Comms and DRAINS the
// resulting command ring once per cycle through routeCommand(). RobotLoop
// decides only WHERE each command goes:
//
//     WHEELS -> Control::DifferentialDrive  (converts mm/s -> counts/s, calls drive())
//     STOP   -> Control::DifferentialDrive  (neutral() -- now IMMEDIATE; the
//                                            queue that made STOP a PLANNED
//                                            stop is gone, a semantic
//                                            narrowing of this exploratory
//                                            tree)
//     ESTOP  -> Control::DifferentialDrive  (estop())
//     CONFIG -> Core::Configurator
//     MOVE / GO_TO -> ERR_UNIMPLEMENTED (deregistered; Motion::Planner/
//                     Motion::Navigator, their only consumers, are gone --
//                     the wire registry/codec itself is UNCHANGED, a
//                     client that sends either verb gets a clean error
//                     ack instead of silent loss)
#pragma once

#include <cstdint>
#include "core/debug.h"

#include "core/comms.h"
#include "core/configurator.h"
#include "control/differential_drive.h"
#include "core/preamble.h"
#include "core/telemetry.h"
#include "hal/clock.h"
#include "hal/color_sensor.h"
#include "hal/i2c_bus.h"
#include "hal/line_sensor.h"
#include "hardware/generic/real_otos.h"
#include "firm/types/robot_state.h"

namespace Core {

class RobotLoop {
 public:
  // Whole-schedule pace target for one cycle(). This no longer has
  // anything to do with motor write timing (the kernel's own
  // kKernelCyclePeriod, boot_calibration.h, governs that now) -- it is
  // purely how often comms/telemetry/OTOS/line/color are serviced.
  // Unchanged from the pre-kernel value; the absolute-deadline pacing
  // discipline (131-005: sleep to cycleStart + kCycle, never a gap
  // relative to "now") still applies and still matters -- see cycle()'s
  // own call site.
  static constexpr uint32_t kCycle = 32;  // [ms] (~31 Hz)

  // All references are already-constructed modules; the composition root
  // (main.cpp or a harness) owns construction and wiring order.
  RobotLoop(Hal::I2CBus& bus, Hal::Otos& otos,
            Hal::ColorSensor& color, Hal::LineSensor& line,
            Comms& comms, Telemetry& tlm, Control::DifferentialDrive& drive,
            Configurator& configurator,
            Preamble& preamble, const Hal::Clock& clock,
            Hal::Sleeper& sleeper);

  [[noreturn]] void run();

  // Boot loop: probe devices until Preamble::done(), emitting boot frames.
  void boot();

  // One pass of the main cycle. Call boot() first -- no readiness checks.
  void cycle();

  // Configuration-completeness gate: handleWheels() refuses until this
  // fires (once, idempotent).
  void markConfigured() { configured_ = true; }
  bool isConfigured() const { return configured_; }

  // setRotationCalibration -- kept for API/wire compatibility (still
  // consumed by nothing internal now that MOVE's angle-stop correction is
  // deregistered, but the geometry.rotation_* fields still read back
  // through GET_CONFIG, and 132-007's test coverage exercises this
  // setter/getter pair directly). Offsets are RADIANS.
  void setRotationCalibration(float gainPos, float offsetPos,
                              float gainNeg, float offsetNeg) {
    rotGainPos_ = gainPos;
    rotOffsetPos_ = offsetPos;
    rotGainNeg_ = gainNeg;
    rotOffsetNeg_ = offsetNeg;
  }

  float rotationGainPos() const { return rotGainPos_; }
  float rotationOffsetPos() const { return rotOffsetPos_; }  // [rad]
  float rotationGainNeg() const { return rotGainNeg_; }
  float rotationOffsetNeg() const { return rotOffsetNeg_; }  // [rad]

  // configure -- the-configuration-object.md's "subsystems take the
  // whole object" entry point, for RobotLoop's own geometry/rotation
  // calibration slice.
  void configure(const Config::Robot& config) {
    setRotationCalibration(config.geometry.rotation_gain_pos, config.rotationOffsetPos(),
                            config.geometry.rotation_gain_neg, config.rotationOffsetNeg());
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

  template <typename Body>
  void runAndWaitUntil(uint32_t deadlineMark, uint32_t deadlineGap, Body body);  // [ms] [ms]

  // Drain the command ring and dispatch each entry to its owning
  // subsystem. Every path acks -- either the outcome of the routing itself
  // or a later completion ack.
  void routeCommand(const Cmd& cmd);
  void handleWheels(const msg::CommandEnvelope& env);
  void handleStop(const msg::CommandEnvelope& env);
  void handleEstop(const msg::CommandEnvelope& env);
  // handleMoveOrGoto() -- MOVE/GO_TO are deregistered (see this file's own
  // header): both verbs share this one ERR_UNIMPLEMENTED reply path.
  void handleMoveOrGoto(const msg::CommandEnvelope& env);

  // handleCalibrate() -- CALIBRATE arm: re-run the OTOS gyro bias
  // calibration on demand. Acks ERR_NONE and triggers
  // Hal::Otos::calibrateImu() ONLY when the robot is verifiably parked --
  // MEASURED wheel velocity still, read from the kernel's own output()
  // (there is no separate "commanded velocity" field on this side of the
  // interface any more -- a semantic narrowing from the pre-kernel
  // both-measured-and-commanded check, documented at the call site).
  // Refusals: ERR_BUSY (moving), ERR_NOT_CONFIGURED (no OTOS present).
  void handleCalibrate(const msg::CommandEnvelope& env);

  // handleGetConfig() -- the CONFIG binary arm's read-back half. Replies
  // SYNCHRONOUSLY (Comms::sendReply()) rather than through the ack ring.
  void handleGetConfig(const msg::CommandEnvelope& env);

  // checkWheelsCompletion() -- the kernel has no completion EVENTS (no
  // queue, no takeCompletion()); RobotLoop tracks the lease deadline it
  // itself handed to drive() and fires the completion ack when it passes.
  // Superseded by a newer WHEELS command (wheelsId_/wheelsDeadline_
  // simply get overwritten) or cancelled by STOP/ESTOP
  // (wheelsPending_ cleared) exactly like the pre-kernel single-slot
  // Drive::commandActive_/completionPending_ pair behaved.
  void checkWheelsCompletion(uint32_t nowMs);  // [ms]

  // Boot-window commands are NACKed (ERR_NOT_CONFIGURED), never dropped.
  void rejectDuringBoot(const Cmd& cmd);

  // --- cycle() steps, in schedule order ---

  // Publish one wheel's state section (rebaseline/clamp/read), after its
  // collect. `clamped` reports the defensive wire-bound clamp.
  void publishWheel(float rawPosition, float rawVelocity, uint32_t rawSampleTimeUs,
                    bool connected, Types::RobotState::Wheel& wheel, bool& clamped);  // [counts] [counts/s] [us]
  void publishWheels();               // both wheels + wedge/health, from drive_.output()
  void publishOtos();

  // Ticks exactly ONE perception leaf and returns which: true = line was
  // read this cycle, false = colour was.
  bool tickLineColor(uint64_t nowUs);  // [us]

  void publishLineColor(bool tickedLine);
  void applySeed();
  // publishPose() -- OTOS-only now: pose mirrors state_.otos when present,
  // else holds at Pose{}'s own zero default. Encoder-derived pose (the
  // old Motion::Odometry half) is gone with motion/.
  void publishPose();
  void publishHealth();
  void publishTiming(uint64_t cycleStartUs);  // [us] cycleBusy/cyclePeriod

#ifdef ROBOT_DEBUG
  // DBG fault injection (system test) + live-tuning arms. Compiled out of
  // shipped images with the whole inbound-DBG surface.
  void applyDbgAction(uint32_t now);  // [ms]

  // captureTuningBaseline -- snapshot the kernel's WHOLE Config the first
  // time a tuning verb (kVmin/kGain/kASteady/kPos) arrives, so DBG:clear
  // can restore it verbatim. Much simpler than the pre-kernel per-field
  // baseline capture: the kernel's config() already returns one flat,
  // copyable value type.
  void captureTuningBaseline();
#endif

#ifdef ROBOT_DEBUG
  uint32_t dbgWedgeUntilL_ = 0;  // [ms]
  uint32_t dbgWedgeUntilR_ = 0;  // [ms]

  bool dbgTuningBaselined_ = false;
  Control::DifferentialDrive::Config dbgConfigBaseline_{};
#endif

  Hal::I2CBus& bus_;
  Hal::Otos& otos_;
  Hal::ColorSensor& color_;
  Hal::LineSensor& line_;
  Comms& comms_;
  Telemetry& tlm_;
  Control::DifferentialDrive& drive_;
  Configurator& configurator_;
  Preamble& preamble_;
  const Hal::Clock& clock_;
  Hal::Sleeper& sleeper_;

  // The blackboard: each section written by the cycle step that owns it,
  // at its point of coherence; tlm_.update(state_) is the one assembly
  // point.
  Types::RobotState state_;

  // Single shared position-rebaseline generation counter. Control::
  // DifferentialDrive::rebasePosition() rebases BOTH encoders atomically
  // (there is no per-wheel rebase in the kernel), so both wheels'
  // positionEpoch always move together -- unlike the pre-kernel
  // per-wheel positionEpochLeft_/Right_ pair, one counter is now correct,
  // not a simplification that loses information.
  uint8_t positionEpoch_ = 0;

  uint32_t cycleCount_ = 0;

  uint64_t previousCycleStartUs_ = 0;  // [us]
  bool everCycled_ = false;

  // WHEELS completion-ack tracking -- see checkWheelsCompletion()'s own
  // doc comment.
  bool wheelsPending_ = false;
  uint32_t wheelsDeadline_ = 0;  // [ms]
  uint32_t wheelsId_ = 0;

  bool configured_ = false;

  // Turn calibration; identity until main.cpp seeds it from the robot
  // JSON. Read-back only now (MOVE's angle-stop correction that used to
  // consume this is deregistered along with MOVE itself).
  float rotGainPos_ = 1.0f;
  float rotOffsetPos_ = 0.0f;
  float rotGainNeg_ = 1.0f;
  float rotOffsetNeg_ = 0.0f;
};

}  // namespace Core
