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
#include "app/debug.h"

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
#include "motion/navigator/navigator.h"
#include "motion/odometry.h"
#include "motion/planner/planner.h"

namespace App {

class RobotLoop {
 public:
  // Whole-schedule pace target for one cycle(). Public so composition
  // roots (TestSim::SimHarness::kCycleDtUs) derive from this one
  // declaration instead of a drifting duplicate literal.
  // 130-007: 40 -> 50 (one 50 ms control period everywhere -- sim and
  // hardware identical). At 40 the loop's measured busy time (~21 ms) plus
  // vendor-bus-clearance overrun meant the pacer never actually slept
  // (delivered period drifted ~46-48 ms, the "47 ms" `boot_config.cpp`
  // baked as a separate, dishonest number).
  //
  // 130-011 bench re-measurement (2026-08-02) found that raising kCycle to
  // 50 did NOT make delivered == nominal as the paragraph above used to
  // claim: cycle_period read a rock-stable 54.000 ms +/- 0.006 ms idle,
  // not 50 -- with cycleBusy only ~21-23 ms (comfortably under budget, so
  // NOT the old kCycle=40 busy-time-overrun pattern). Same +4 ms at both
  // kCycle=40 and kCycle=50: structural, not a budget problem -- the four
  // `runAndWait()` pacing blocks each computed their own wait as a GAP
  // relative to that block's own entry mark, so any rounding/overrun any
  // one block picked up (fiber_sleep quantizing to a whole tick, real work
  // between marks) was simply re-added on top of the next block's own
  // fixed budget instead of being absorbed anywhere.
  //
  // 131-005 fixes this: cycle()'s trailing pacing block now targets an
  // ABSOLUTE end-of-cycle deadline (state_.time.cycleStart + kCycle)
  // rather than a gap relative to its own entry mark, so whatever the
  // three earlier blocks' own jitter/rounding already spent is absorbed
  // into this one final wait instead of compounding across all four (see
  // robot_loop.cpp's own runAndWaitUntil()/cycle() call site). Proven
  // self-correcting under injected jitter in a host-build unit test
  // (app_robot_loop_pacing_harness.cpp).
  //
  // 131-005's open hardware follow-up is now CLOSED (2026-08-07, tovez,
  // direct serial): delivered cycle_period reads 50.00 ms median, min
  // 49.97, max 50.06 -- the 54 ms of the 130-011 generation is gone and
  // delivered really does equal nominal. The pacer works.
  //
  // 50 -> 32 (2026-08-07, out of process): the loop was paced far slower
  // than its own work needs. Measured on tovez by setting kCycle BELOW
  // the floor, so the trailing sleepUntil() yields and the delivered
  // period becomes the loop's true work time:
  //
  //   as shipped (kClear=4)   floor 22.2 ms median, ~28 ms worst
  //   with kClear removed     floor 17.7 ms median, ~28 ms worst
  //
  // The mandatory part is 8 ms/cycle: two 4 ms encoder select->read
  // settles, one per motor, which the Nezha brick requires. Those are the
  // kSettle windows and they cost NOTHING in wall clock -- zeroing them
  // measured identical to keeping them, because the same wait simply
  // reappears inside Devices::I2CBus::waitForClearance()'s fiber_sleep.
  // The windows are where that mandatory wait gets spent USEFULLY (the
  // comms pump runs inside them); waitForClearance just sleeps.
  //
  // What was NOT mandatory was the third window, kClear (post-duty-write,
  // 4 ms): the next 0x10 transaction after a duty write is the NEXT
  // cycle's encoder select, tens of ms later and long past any clearance
  // deadline, so it guarded nothing. Removed here, worth 4.7 ms/cycle.
  //
  // Why 32 and not the 17.7 ms floor: the TAIL binds, not the median.
  // Worst-case cycles run ~28 ms in every configuration measured, so 32
  // keeps delivered == nominal by construction (the invariant the whole
  // 130-011/131-005 history above exists to protect, and which
  // control_period == kCycle depends on) instead of buying a faster
  // median at the price of an occasionally-stretched cycle. Verified at
  // exactly this period on tovez: 32.00 ms median, max 32.04.
  //
  // Everything that must move WITH this constant: Telemetry::
  // kPrimaryPeriod (telemetry.h), Devices::NezhaMotor's
  // kMinWriteIntervalUs (nezha_motor.cpp -- a hand-synced literal, since
  // devices/ may not include app/ headers), and every robot JSON's
  // planner.control_period AND planner.actuation_delay, which are both
  // "= kCycle" by construction (actuation_delay because a command staged
  // this cycle lands on the wheels the next one).
  static constexpr uint32_t kCycle = 32;  // [ms] (~31 Hz)

  // All references are already-constructed modules; the composition root
  // (main.cpp or a harness) owns construction and wiring order. The
  // persisted-tuning store is NOT a parameter here any more -- it belongs
  // to App::Configurator, which owns the whole CONFIG lifecycle.
  RobotLoop(Devices::I2CBus& bus, Devices::Motor& motorL,
            Devices::Motor& motorR, Devices::Otos& otos,
            Devices::ColorSensorLeaf& color, Devices::LineSensorLeaf& line,
            Comms& comms, Telemetry& tlm, Drive& drive,
            Configurator& configurator, Motion::Odometry& odom,
            Motion::Planner& planner, Motion::Navigator& navigator,
            Preamble& preamble, const Devices::Clock& clock,
            Devices::Sleeper& sleeper);

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

  // Read-only observability for setRotationCalibration()'s installed
  // values (test/bench observability -- mirrors Drive's own biasLeft()/
  // pidLeft()-style getters, drive.h). Added for 132-007's own configure()
  // unit test: nothing else reads rotation calibration back out today.
  float rotationGainPos() const { return rotGainPos_; }
  float rotationOffsetPos() const { return rotOffsetPos_; }  // [rad]
  float rotationGainNeg() const { return rotGainNeg_; }
  float rotationOffsetNeg() const { return rotOffsetNeg_; }  // [rad]

  // configure -- the-configuration-object.md's "subsystems take the
  // whole object" entry point (132-007), for RobotLoop's own
  // geometry/rotation calibration slice. A thin pull-and-forward onto
  // setRotationCalibration() above, reading Config::Robot's geometry
  // group and its two derived rotationOffsetPos()/rotationOffsetNeg()
  // methods (config/robot.h) instead of App::installRotationCalibration()'s
  // now-superseded msg::DrivetrainConfig-based conversion
  // (boot_calibration.cpp:75-81) -- this IS the "solve cleanly" 132-006
  // deferred here (configurator.h's own doc comment): boot_wiring.cpp
  // calls this directly, right after configurator_.loadBaked(), so
  // RobotLoop never needs a Configurator& to reach its own configuration
  // (which would be circular -- RobotLoop already holds a Configurator&
  // for CONFIG command routing).
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

  // 131-005: like runAndWait(), but the wait targets an ABSOLUTE deadline
  // (deadlineMark + deadlineGap) instead of a gap relative to a mark taken
  // at this call's own entry. Used ONLY by cycle()'s trailing block, whose
  // deadline is state_.time.cycleStart + kCycle -- so jitter/rounding
  // already spent by the three earlier runAndWait() blocks this cycle is
  // absorbed into this one final wait instead of compounding on top of it
  // (see cycle()'s own call site comment).
  template <typename Body>
  void runAndWaitUntil(uint32_t deadlineMark, uint32_t deadlineGap, Body body);  // [ms] [ms]

  // Drain the command ring and dispatch each entry to its owning
  // subsystem. Every path acks -- either the outcome of the routing itself
  // (enqueue/ERR_FULL/ERR_BADARG) or, for the queued kinds, a later
  // completion ack from the destination.
  void routeCommand(const Cmd& cmd);
  void handleMove(const msg::CommandEnvelope& env);
  void handleWheels(const msg::CommandEnvelope& env);
  void handleStop(const msg::CommandEnvelope& env);
  void handleEstop(const msg::CommandEnvelope& env);
  // handleGoto() -- 135-004: arms Motion::Navigator with a world- or
  // robot-frame target. Rejects with ERR_NOT_CONFIGURED (see this
  // method's own .cpp doc comment for why that ErrCode, not a new one)
  // while `!state_.otos.connected` -- SUC-005's own explicit gate, a goto
  // with no OTOS fix to navigate on is refused outright, never
  // accepted-then-immediately-aborted. Cancels active Drive teleop via
  // drive_.takeover(), same as handleMove().
  void handleGoto(const msg::CommandEnvelope& env);

  // handleGetConfig() -- 132-011: the CONFIG binary arm's read-back half.
  // Unlike every other case in routeCommand()'s switch, this one replies
  // SYNCHRONOUSLY (Comms::sendReply(), a ReplyEnvelope{cfg: ConfigSnapshot}
  // or {err: Error}) rather than through the ack ring -- a ring entry has
  // no room for a group's worth of values. See configurator.h's
  // encodeSnapshot() for the read-back itself.
  void handleGetConfig(const msg::CommandEnvelope& env);

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

  // SAFETY ARBITRATION (133-001), first in the schedule -- runs immediately
  // ahead of drive_.tick(), the single actuation path.
  //
  // MONOTONE CONTRACT: this method may write ONLY 0.0f to cmdVelocity, and
  // never a nonzero value. That restriction is what makes a loop-level
  // write to a decider-owned field legitimate: a zero-only writer cannot
  // originate motion, cannot fight a decider for control, and cannot
  // produce a speed no decider asked for. Do not relax it -- the moment
  // this method can emit a nonzero, App::RobotLoop becomes a third
  // decider, and the cmdVelocity ownership invariant (stated in full at
  // RobotLoop::publishWheels(), robot_loop.cpp) is void.
  //
  // Idleness is DERIVED (!planner_.active() && !drive_.owns()), never
  // announced or acquired. There is deliberately no idle-owner subsystem:
  // an owner must be told to take over, and the failure this guards is a
  // decider that has silently STOPPED publishing -- which is exactly the
  // case that would never tell anyone anything. See the .cpp's own comment
  // at the call site for the measured defect (vevov, 2026-08-03).
  void zeroUnownedMotion();

  // Publish one wheel's state section (rebaseline/clamp/read), after its
  // collect. `clamped` reports the defensive wire-bound clamp.
  void publishWheel(Devices::Motor& motor, Types::RobotState::Wheel& wheel,
                    uint8_t& positionEpoch, bool& clamped);
  void publishWheels();               // both wheels + wedge/clamp health
  void publishOtos();
  void publishLineColor(bool tickedLine);
  void applySeed();
  void publishPose();
  void publishHealth();
  void ackDriveCompletion();
  void publishTiming(uint64_t cycleStartUs);  // [us] cycleBusy/cyclePeriod
  // Move-fault flags + completion ack (rides next frame).
  void publishMoveResult(const Motion::TickResult& moveResult);
  // publishGotoResult() -- 135-004: NavResult's own counterpart, called
  // instead of publishMoveResult() on a cycle a goto (not an ordinary
  // Move) owns Drive -- cycle()'s own if/navigator_.active() branch, this
  // file's Landmine-1 fix (see robot_loop.cpp's own doc comment on both).
  void publishGotoResult(const Motion::NavResult& navResult);

#ifdef ROBOT_DEBUG
  // DBG fault injection (system test) + 133-003's live tuning arms: apply
  // one staged Comms::DbgAction and expire any duration-bounded injected
  // wedge. Compiled out of shipped images with the whole inbound-DBG
  // surface.
  void applyDbgAction(uint32_t now);  // [ms]

  // captureTuningBaseline -- snapshot what BOOT installed, once, the first
  // time a tuning verb (kVmin/kGain/kASteady/kPos) arrives. Two jobs, both
  // load-bearing:
  //
  //   1. `DBG:gain` is a MULTIPLIER on the boot-installed dutyPerSpeed
  //      pair, and setDutyPerSpeed() overwrites rather than accumulates.
  //      Without a fixed baseline, `gain 1.02 0.98` sent twice would
  //      compound to 1.0404/0.9604 -- an operator re-asserting the same
  //      value would silently get a different robot each time, which is
  //      the exact failure the echo contract exists to prevent one layer
  //      up. With it, the verb is idempotent and `gain 1 1` is a true
  //      identity.
  //   2. `DBG:clear` can then restore every tuning knob to boot, so a
  //      sweep is undoable without a reflash or a power cycle.
  //
  // The baseline is the value Configurator::install() landed, NOT
  // Drive::kDutyPerSpeed. Since 132-009 that constant is reference-only
  // and boot reads config.drive.duty_per_speed_left/right instead
  // (drive.h's own kDutyPerSpeed doc comment); multiplying the constant
  // would hand every robot tovez's gearboxes again -- the precise
  // anti-pattern that comment warns about.
  void captureTuningBaseline();
#endif

#ifdef ROBOT_DEBUG
  // Injected-wedge auto-clear deadlines, per wheel; 0 = no injection
  // armed, UINT32_MAX = latched until DBG:clear.
  uint32_t dbgWedgeUntilL_ = 0;  // [ms]
  uint32_t dbgWedgeUntilR_ = 0;  // [ms]

  // Boot-installed tuning snapshot -- see captureTuningBaseline() above.
  // `dbgTuningBaselined_` false means no tuning verb has landed yet, so
  // the other three fields are meaningless and DBG:clear has nothing to
  // restore.
  bool dbgTuningBaselined_ = false;
  Drive::AdaptationBounds dbgBoundsBaseline_{};
  float dbgDutyPerSpeedBaseLeft_ = 0.0f;   // [duty/(mm/s)]
  float dbgDutyPerSpeedBaseRight_ = 0.0f;  // [duty/(mm/s)]
#endif

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
  Motion::Navigator& navigator_;
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

  // Parity picks line vs color in the pace block: odd cycles tick LINE,
  // even cycles tick COLOR, so each sensor lands at kCycle/2 -- neither
  // sensor is worth an I2C transaction every cycle.
  //
  // Incremented once per cycle() at the top. It was stuck at 0 from its
  // introduction until 2026-08-07, which made `(cycleCount_ % 2) == 1`
  // permanently false and meant the LINE sensor was never ticked at all.
  // The concern that held the fix back -- that the extra transaction on
  // alternate cycles would shift the loop period the motion tuning is
  // calibrated against -- was measured on tovez when the fix landed and
  // is not real at kCycle=50: the alternation replaces the color read
  // rather than adding to it, and `LineSensorLeaf::tick()` returns before
  // touching the bus when no chip was detected, so an unfitted sensor
  // costs nothing.
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
