#include "core/robot_loop.h"
#include "core/debug.h"

#include <cmath>
#include <cstdio>

#include "messages/envelope.h"

namespace Core {

namespace {

constexpr uint32_t kPreamblePace = 10;  // [ms] boot-loop probe pacing

// Position wire-bound / rebaseline margin -- MILLIMETRE DOMAIN, restored.
//
// These were briefly rebaked into the counts domain (2,000,000 / 1,800,000
// counts) on the reasoning that the kernel is counts-native. That reasoning
// is right about the KERNEL and wrong about the WIRE. msg::EncoderReading
// is a wire field whose documented unit is [mm] (position) and [mm/s]
// (velocity) -- src/host/robot_radio/robot/protocol.py says so, and
// src/tests/bench/{plant_id,speed_sweep,stall_gate,crawl_sweep,
// velocity_profile_gate}.py all compute physical distances from it. Nothing
// host-side was migrated. Publishing counts under an unchanged frame SHAPE
// therefore changes what the field MEANS by ~14.19x (1 count == 0.1 deg ==
// 0.0705 mm at tovez's travel_calib) with no rename to warn anyone -- a
// protocol change by stealth, and protocol changes are explicitly out of
// scope for this exploration.
//
// So the counts->mm conversion happens HERE, at the outbound adapter, per
// wheel, from the same two travel calibrations the inbound WHEELS adapter
// uses (the issue's own "Outbound" contract). The kernel keeps its counts;
// the wire keeps its millimetres; the class still never sees mm.
//
// The float32-precision concern the counts rebake raised is real but is a
// property of the KERNEL's accumulator, not of this bound: 30 m of travel
// is ~425,500 counts, comfortably inside float32's exact-integer range
// (2^24), so the rebaseline fires long before precision degrades.
constexpr float kPositionWireBound = 32000.0f;         // [mm]
constexpr float kPositionRebaselineMargin = 30000.0f;  // [mm]

// [counts/s] a wheel reading at or below this counts as "still" for
// CALIBRATE's parked precondition. COUNTS REBAKE (2026-08-15): was 5.0
// mm/s; matches the leaf's own kReconfigureRestVelocity rebake (nezha_
// motor.h) at the same "generic at-rest guard" order of magnitude.
constexpr float kCalibrateStillSpeed = 60.0f;

uint32_t packLine(const Hal::LineReading& reading) {
  return (reading.raw[0] & 0xFFu) | ((reading.raw[1] & 0xFFu) << 8) |
         ((reading.raw[2] & 0xFFu) << 16) | ((reading.raw[3] & 0xFFu) << 24);
}

// Squeeze one 16-bit channel into the wire's 8 bits by scaling against the
// ADC's ACTUAL full scale, not against 65535.
uint8_t scaleColorChannel(uint32_t value, uint32_t fullScale) {  // [counts]
  if (fullScale == 0) return 0;
  const uint32_t scaled = (value * 255u) / fullScale;
  return static_cast<uint8_t>(scaled > 255u ? 255u : scaled);
}

uint32_t packColor(const Hal::ColorReading& reading, uint32_t fullScale) {  // [counts]
  return static_cast<uint32_t>(scaleColorChannel(reading.r, fullScale)) |
         (static_cast<uint32_t>(scaleColorChannel(reading.g, fullScale)) << 8) |
         (static_cast<uint32_t>(scaleColorChannel(reading.b, fullScale)) << 16) |
         (static_cast<uint32_t>(scaleColorChannel(reading.c, fullScale)) << 24);
}

}  // namespace

RobotLoop::RobotLoop(Hal::I2CBus& bus, Hal::Otos& otos,
                      Hal::ColorSensor& color, Hal::LineSensor& line,
                      Comms& comms, Telemetry& tlm, Control::DifferentialDrive& drive,
                      Configurator& configurator,
                      Preamble& preamble, const Hal::Clock& clock,
                      Hal::Sleeper& sleeper)
    : bus_(bus),
      otos_(otos),
      color_(color),
      line_(line),
      comms_(comms),
      tlm_(tlm),
      drive_(drive),
      configurator_(configurator),
      preamble_(preamble),
      clock_(clock),
      sleeper_(sleeper) {}

uint32_t RobotLoop::markTime() const {
  return static_cast<uint32_t>(clock_.nowMicros() / 1000);  // [us] -> [ms]
}

void RobotLoop::sleepUntil(uint32_t mark, uint32_t gap) {  // [ms] [ms]
  uint32_t elapsed = markTime() - mark;
  uint32_t remaining = (elapsed < gap) ? (gap - elapsed) : 0;
  if (remaining > 0) {
    sleeper_.sleepMillis(remaining);
  } else {
    sleeper_.yield();
  }
}

template <typename Body>
void RobotLoop::runAndWait(uint32_t gap, Body body) {  // [ms]
  uint32_t mark = markTime();
  body();
  sleepUntil(mark, gap);
}

template <typename Body>
void RobotLoop::runAndWaitUntil(uint32_t deadlineMark, uint32_t deadlineGap, Body body) {  // [ms] [ms]
  body();
  sleepUntil(deadlineMark, deadlineGap);
}

float RobotLoop::clampToPositionWireBound(float pos, bool* clamped) {
  *clamped = std::fabs(pos) > kPositionWireBound;
  return *clamped ? std::copysign(kPositionWireBound, pos) : pos;
}

void RobotLoop::routeCommand(const Cmd& cmd) {
  if (cmd.status != CmdStatus::kDecoded) return;

  switch (cmd.env.cmd_kind) {
    case msg::CommandEnvelope::CmdKind::MOVE:
    case msg::CommandEnvelope::CmdKind::GO_TO:
      handleMoveOrGoto(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::WHEELS:
      handleWheels(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::STOP:
      handleStop(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::ESTOP:
      handleEstop(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::CONFIG:
      tlm_.ack(cmd.env.corr_id,
               static_cast<uint32_t>(configurator_.applyGroup(
                   cmd.env.cmd.config.target, cmd.env.cmd.config.body_,
                   cmd.env.cmd.config.body_count)));
      break;
    case msg::CommandEnvelope::CmdKind::GET_CONFIG:
      handleGetConfig(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::CALIBRATE:
      handleCalibrate(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::SET_FIELD:
      tlm_.ack(cmd.env.corr_id,
               static_cast<uint32_t>(configurator_.applyField(
                   cmd.env.cmd.set_field.target,
                   static_cast<uint16_t>(cmd.env.cmd.set_field.field),
                   cmd.env.cmd.set_field.value)));
      break;
    case msg::CommandEnvelope::CmdKind::NONE:
    default:
      break;
  }
}

// handleMoveOrGoto() -- MOVE/GO_TO deregistered: Motion::Planner/Motion::
// Navigator, their only consumers, are deleted with the whole
// src/firm/motion/ tree (the exploratory-kernel rewrite). The wire
// registry/codec is UNCHANGED -- a client that still sends either verb
// gets a clean ERR_UNIMPLEMENTED ack instead of silent loss or a decode
// error, the same code ESTIMATOR's own permanently-unwired CONFIG target
// already uses for "no consumer any more."
void RobotLoop::handleMoveOrGoto(const msg::CommandEnvelope& env) {
  tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_UNIMPLEMENTED));
}

void RobotLoop::handleCalibrate(const msg::CommandEnvelope& env) {
  // Parked precondition, enforced HERE. The gyro averages ~612ms of
  // samples into its bias estimate; motion during that window is exactly
  // the boot fault this command exists to repair.
  //
  // MEASURED-ONLY now (a semantic narrowing from the pre-kernel check,
  // which also refused on a nonzero COMMANDED velocity): the kernel's
  // Command mailbox is private, so RobotLoop has no "what was just
  // commanded" signal any more, only drive_.output()'s measured velocity.
  // A reversal whose measured speed happens to pass exactly through zero
  // this cycle could in principle slip through; accepted for this
  // exploratory tree.
  if (!state_.otos.present) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_NOT_CONFIGURED));
    return;
  }
  const Control::DifferentialDrive::Output out = drive_.output();
  const bool still = std::fabs(out.velocityLeft) <= kCalibrateStillSpeed &&
                      std::fabs(out.velocityRight) <= kCalibrateStillSpeed;
  if (!still) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_BUSY));
    return;
  }
  otos_.calibrateImu(static_cast<uint8_t>(env.cmd.calibrate.samples));
  tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_NONE));
}

void RobotLoop::handleGetConfig(const msg::CommandEnvelope& env) {
  const msg::ConfigGroupTarget target = env.cmd.get_config.target;

  msg::ConfigSnapshot snapshot;
  const msg::ErrCode result = configurator_.encodeSnapshot(target, snapshot);

  msg::ReplyEnvelope reply;
  reply.corr_id = env.corr_id;
  if (result == msg::ErrCode::ERR_NONE) {
    reply.body_kind = msg::ReplyEnvelope::BodyKind::CFG;
    reply.body.cfg = snapshot;
  } else {
    reply.body_kind = msg::ReplyEnvelope::BodyKind::ERR;
    reply.body.err.code = result;
    reply.body.err.field = static_cast<uint32_t>(target);
  }
  comms_.sendReply(reply);
}

// handleWheels() -- the WHEELS->drive() adapter. msg::Wheels carries
// vLeft/vRight [mm/s] + duration [ms]; the kernel takes body (velocity,
// twist) in counts/s. Converts per-wheel via travel_calib (mm/deg) from
// the live Config::Robot, then folds the two wheel speeds into one
// (velocity, twist) pair -- left = velocity - twist, right = velocity +
// twist round-trips exactly for velocity=(l+r)/2, twist=(r-l)/2.
void RobotLoop::handleWheels(const msg::CommandEnvelope& env) {
  if (!configured_) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_NOT_CONFIGURED));
    return;
  }

  const msg::Wheels& wheels = env.cmd.wheels;
  if (wheels.duration <= 0.0f) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_BADARG));
    return;
  }

  const Config::Robot& cfg = configurator_.config();
  const float mmsToCountsL =
      (cfg.motors.travel_calib_left > 0.0f) ? 10.0f / cfg.motors.travel_calib_left : 0.0f;
  const float mmsToCountsR =
      (cfg.motors.travel_calib_right > 0.0f) ? 10.0f / cfg.motors.travel_calib_right : 0.0f;
  const float countsLeft = wheels.v_left * mmsToCountsL;
  const float countsRight = wheels.v_right * mmsToCountsR;
  const float velocity = 0.5f * (countsLeft + countsRight);
  const float twist = 0.5f * (countsRight - countsLeft);

  // v5-adapter auto-clear: a fresh WHEELS command supersedes any latent
  // estop latch. The kernel's estop() is a proper LATCH (cleared only by
  // estopClear() + a fresh command, per its own header doc) meant for a
  // future stateful client; the current v5 wire protocol has no separate
  // "clear estop" verb, so this adapter clears it here, automatically,
  // on every new WHEELS -- documented narrowing for this exploratory
  // tree's v5 compatibility shim.
  drive_.estopClear();
  drive_.clearStallLatch();
  drive_.drive(velocity, twist, static_cast<uint32_t>(wheels.duration));

  wheelsPending_ = true;
  wheelsDeadline_ = state_.time.cycleStart + static_cast<uint32_t>(wheels.duration);
  wheelsId_ = wheels.id;

  tlm_.ack(env.corr_id, 0);
}

// handleStop() -- STOP is now an IMMEDIATE neutral(), never a PLANNED
// stop: the planner queue that used to make STOP wait behind an
// in-flight Move is gone with Motion::Planner. This is a documented
// semantic narrowing of this exploratory tree -- see robot_loop.h's own
// header. `env.cmd.stop.id` is accepted but unused (there is no queue
// entry for it to key).
void RobotLoop::handleStop(const msg::CommandEnvelope& env) {
  drive_.neutral();
  wheelsPending_ = false;  // no completion ack for a superseded command
  tlm_.ack(env.corr_id, 0);
}

void RobotLoop::handleEstop(const msg::CommandEnvelope& env) {
  drive_.estop();
  wheelsPending_ = false;
  tlm_.ack(env.corr_id, 0);
}

void RobotLoop::checkWheelsCompletion(uint32_t nowMs) {
  if (!wheelsPending_) return;
  if (static_cast<int32_t>(nowMs - wheelsDeadline_) < 0) return;
  wheelsPending_ = false;
  tlm_.ack(wheelsId_, 0);
}

void RobotLoop::rejectDuringBoot(const Cmd& cmd) {
  if (cmd.status != CmdStatus::kDecoded) return;
  tlm_.ack(cmd.env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_NOT_CONFIGURED));
}

[[noreturn]] void RobotLoop::run() {
  boot();
  for (;;) {
    cycle();
  }
}

void RobotLoop::boot() {
  while (!preamble_.done()) {
    comms_.pump(markTime());
    Cmd bootCmd;
    while (comms_.takeCommand(bootCmd)) rejectDuringBoot(bootCmd);

    preamble_.step();  // one bounded probe action per pass

    Types::RobotState bootState;
    bootState.time.cycleStart = markTime();
    bootState.otos.connected = preamble_.otosConnected();
    tlm_.update(bootState, drive_);
    tlm_.emit(bootState.time.cycleStart);

    sleeper_.sleepMillis(kPreamblePace);  // paces probes AND yields (radio RX)
  }

  comms_.sendBanner();
  comms_.sendReady();
  state_.health.ready = true;
}

void RobotLoop::publishWheel(float rawPosition, float rawVelocity, uint32_t rawSampleTimeUs,
                             bool connected, uint32_t positionEpoch,
                             Types::RobotState::Wheel& wheel, bool& clamped) {
  wheel.position = clampToPositionWireBound(rawPosition, &clamped);
  wheel.velocity = rawVelocity;
  wheel.sampleTime = rawSampleTimeUs / 1000u;  // [us] -> [ms]
  wheel.connected = connected;
  wheel.positionEpoch = static_cast<uint8_t>(positionEpoch);
}

// publishWheels() -- reads Control::DifferentialDrive::output() ONCE per
// cycle (the kernel's own fiber is the only writer of the motors; this is
// a pure snapshot read, never a command). Triggers a software rebaseline
// (drive_.rebasePosition()) when either wheel's accumulated position
// crosses the counts-domain margin -- see this file's own kPositionWireBound/
// kPositionRebaselineMargin comment.
void RobotLoop::publishWheels() {
  const Control::DifferentialDrive::Output out = drive_.output();

  // counts -> mm, PER WHEEL, from the same two travel calibrations
  // handleWheels() uses on the way in. travel_calib is [mm/deg] and one
  // count is 0.1 deg, hence the 0.1 factor. A zero/absent calibration
  // yields 0 rather than a garbage scale -- the same fail-closed posture
  // the inbound adapter takes.
  const Config::Robot& cfg = configurator_.config();
  const float countsToMmLeft =
      (cfg.motors.travel_calib_left > 0.0f) ? cfg.motors.travel_calib_left * 0.1f : 0.0f;
  const float countsToMmRight =
      (cfg.motors.travel_calib_right > 0.0f) ? cfg.motors.travel_calib_right * 0.1f : 0.0f;

  const float positionLeft = out.positionLeft * countsToMmLeft;    // [mm]
  const float positionRight = out.positionRight * countsToMmRight;  // [mm]

  if (std::fabs(positionLeft) >= kPositionRebaselineMargin ||
      std::fabs(positionRight) >= kPositionRebaselineMargin) {
    drive_.rebasePosition();
  }

  bool clampedL = false;
  bool clampedR = false;
  // The epoch is the KERNEL's own per-wheel counter now, not a RobotLoop
  // tally. RobotLoop only REQUESTS a rebaseline; the kernel decides which
  // cycle it actually lands on (rebasePosition() is a one-shot counter
  // handshake consumed at the next cycle start). Counting here incremented
  // on the request instead, so the published epoch could advance a cycle
  // before the position it describes actually moved.
  publishWheel(positionLeft, out.velocityLeft * countsToMmLeft, out.sampleTimeLeft,
              out.connectedLeft, out.positionEpochLeft, state_.wheelLeft, clampedL);
  publishWheel(positionRight, out.velocityRight * countsToMmRight, out.sampleTimeRight,
              out.connectedRight, out.positionEpochRight, state_.wheelRight, clampedR);

  state_.health.wedgeLatch = out.wedgeLeft || out.wedgeRight;
  state_.health.wheelFrozenLeft = out.wedgeSuspectLeft;
  state_.health.wheelFrozenRight = out.wedgeSuspectRight;
  // Stall is now latched INSIDE the kernel (stallLatched_/stallHalted_,
  // cleared only by drive_.clearStallLatch()) -- RobotLoop just mirrors it
  // for telemetry/Comms::Status, it does not compute or re-latch it.
  state_.health.stallLeft = out.stallLeft;
  state_.health.stallRight = out.stallRight;
  state_.health.positionClamped = clampedL || clampedR;
}

void RobotLoop::publishOtos() {
  const bool otosPresent = otos_.present() && otos_.poseFresh();
  state_.otos.present = otosPresent;
  state_.otos.connected = otos_.connected();
  if (!otosPresent) return;

  const Hal::PoseReading reading = otos_.pose();
  state_.otos.x = reading.x;
  state_.otos.y = reading.y;
  state_.otos.heading = reading.heading;
  state_.otos.v_x = reading.v_x;
  state_.otos.v_y = reading.v_y;
  state_.otos.omega = reading.omega;
  state_.otos.sampleTime = static_cast<uint32_t>(otos_.sampleTime() / 1000);  // [us] -> [ms]
}

bool RobotLoop::tickLineColor(uint64_t nowUs) {
  const bool tickedLine = (cycleCount_ % 2) == 1;
  if (tickedLine) {
    line_.tick(nowUs);
  } else {
    color_.tick(nowUs);
  }
  return tickedLine;
}

void RobotLoop::publishLineColor(bool tickedLine) {
  const bool lineFresh = tickedLine && line_.readingFresh();
  const bool colorFresh = !tickedLine && color_.readingFresh();

  state_.perception.lineFresh = lineFresh;
  state_.perception.colorFresh = colorFresh;
  if (lineFresh) state_.perception.line = packLine(line_.reading());
  if (colorFresh) state_.perception.color = packColor(color_.reading(), color_.fullScale());

  state_.perception.lineValid = (state_.perception.lineValid || lineFresh) && line_.connected();
  state_.perception.colorValid = (state_.perception.colorValid || colorFresh) && color_.connected();
}

// applySeed() -- OTOS-only now: the encoder-odometry half (Motion::
// Odometry::reset()) is gone with motion/. The wire SEED reply/ack shape
// is unchanged.
void RobotLoop::applySeed() {
  const Comms::SeedRequest seed = comms_.takeSeed();
  if (!seed.pending) return;

  otos_.setPose(seed.x, seed.y, seed.heading);

  if (seed.reply != nullptr) {
    char line[64];
    std::snprintf(line, sizeof(line), "SEED:%d:%d:%d",
                  static_cast<int>(seed.x), static_cast<int>(seed.y),
                  static_cast<int>(seed.heading * 1000.0f));
    seed.reply->sendReliable(line);
  }
}

// publishPose() -- OTOS-only: mirrors state_.otos when present, else
// holds at Pose{}'s own all-zero default. state_.otos.present (already
// published on the wire as kFlagOtosPresent) IS the validity flag a
// client checks -- there is no separate per-Pose validity field.
void RobotLoop::publishPose() {
  if (state_.otos.present) {
    state_.pose.x = state_.otos.x;
    state_.pose.y = state_.otos.y;
    state_.pose.heading = state_.otos.heading;
    state_.pose.v_x = state_.otos.v_x;
    state_.pose.v_y = state_.otos.v_y;
    state_.pose.omega = state_.otos.omega;
  } else {
    state_.pose = Types::RobotState::Pose{};
  }
}

void RobotLoop::publishHealth() {
  const Control::DifferentialDrive::Output out = drive_.output();
  const bool moving = !out.estopped && !out.leaseExpired && !out.stallHalted &&
                      (out.velocity != 0.0f || out.twist != 0.0f || wheelsPending_);
  state_.command.mode = moving ? Types::Mode::Velocity : Types::Mode::Idle;
  state_.command.moveActive = moving;
  state_.health.i2cSafetyNetCount = bus_.clearanceSafetyNetCount();
  state_.health.commsMalformedCount = comms_.malformedCount();
  state_.health.commandsDroppedCount = comms_.commandsDroppedCount();
}

// checkKernelHeartbeat() -- RobotLoop's ONE remaining safety job.
//
// The wheel kernel runs on its own fiber and is the only thing that
// writes the motors. If that fiber dies, exits, or logic-stalls, nothing
// else notices: the Nezha brick physically latches its last commanded
// speed and does not reset on an nRF52 reset, so a robot whose kernel
// stopped keeps driving at whatever it was last told, indefinitely. That
// is the measured 936 mm-and-still-going runaway class.
//
// So: watch Output::cycleCount advance. If it stalls for more than
// kSentinelPeriods kernel cycles WHILE motion was commanded, raise a
// sticky telemetry fault and force both motors to zero through
// Hal::Motor::emergencyStop() -- which writes immediately rather than
// staging, precisely because the thing that executes stages (the kernel's
// tick()) is what just died.
//
// SCOPE, honestly: this covers failures where the MAIN loop still runs --
// kernel fiber crashed, exited, or logic-stalled while the bus is alive.
// It CANNOT cover the dead-bus busy-wait (lesson 17): CODAL I2C
// transactions never yield, so waitForStop() freezes every fiber
// including this one. In that failure the bus that would have carried the
// zero write is itself the thing that died. Only the deferred hardware
// WDT closes that gap; this narrows it, it does not shut it.
void RobotLoop::checkKernelHeartbeat() {
  const uint32_t beat = drive_.output().cycleCount;

  // "Motion was commanded" is inferred from applied duty rather than from
  // the mailbox (which is private to the kernel by design). A stalled
  // kernel keeps publishing its LAST Output, so a nonzero appliedDuty in
  // that frozen snapshot is exactly the dangerous case: duty on the wire,
  // nobody left to take it off.
  const Control::DifferentialDrive::Output out = drive_.output();
  const bool motionCommanded =
      out.appliedDutyLeft != 0.0f || out.appliedDutyRight != 0.0f;

  if (beat != lastKernelBeat_) {
    lastKernelBeat_ = beat;
    kernelStallCycles_ = 0;
    return;
  }
  if (!motionCommanded) {
    // A parked robot whose kernel is idle is not a fault. Do not
    // accumulate against it, or a long stop would eventually trip.
    kernelStallCycles_ = 0;
    return;
  }
  if (++kernelStallCycles_ < kSentinelPeriods) return;

  // Fire. Sticky: a kernel that died once is not to be trusted again
  // without a reboot, and a self-clearing bit would let the event scroll
  // past unseen in a telemetry log. Re-asserted every cycle while the
  // condition holds -- one lost zero write is permanent on this brick
  // (stopNotTaken, lesson 3), so repetition is the point.
  state_.health.kernelStalled = true;
  // Latches estop AND writes both motors immediately, from THIS fiber --
  // the kernel's own fiber is the thing that is not running.
  drive_.emergencyStopMotors();
}

void RobotLoop::publishTiming(uint64_t cycleStartUs) {
  state_.time.cycleBusy = static_cast<uint32_t>(clock_.nowMicros() - cycleStartUs);  // [us]
  state_.time.cyclePeriod =
      everCycled_ ? static_cast<uint32_t>(cycleStartUs - previousCycleStartUs_) : 0u;  // [us]
  previousCycleStartUs_ = cycleStartUs;
  everCycled_ = true;
}

// cycle() -- the kernel's own fiber owns the motors entirely now; this
// loop never touches them. One pump-and-publish block, paced to kCycle
// via the same absolute-deadline discipline (131-005) the pre-kernel
// design used across its (now-gone) multiple motor-settle windows -- with
// no mandatory encoder-settle wait left to spend the pump inside, a
// single block covering the whole cycle is the natural shape.
void RobotLoop::cycle() {
  state_.time.cycleStart = markTime();  // [ms] pace anchor + wire `now`
  const uint64_t cycleStartUs = clock_.nowMicros();  // [us]
  ++cycleCount_;

  runAndWaitUntil(state_.time.cycleStart, kCycle, [&] {
    comms_.pump(state_.time.cycleStart);
    Cmd cmd;
    while (comms_.takeCommand(cmd)) routeCommand(cmd);

    uint64_t nowUs = clock_.nowMicros();

    publishWheels();  // reads drive_.output() -- the kernel's own snapshot
    checkKernelHeartbeat();
    otos_.tick(nowUs);
    publishOtos();
    publishLineColor(tickLineColor(nowUs));
    publishPose();
    publishHealth();
    publishTiming(cycleStartUs);
    checkWheelsCompletion(state_.time.cycleStart);

    tlm_.update(state_, drive_);

#ifdef ROBOT_DEBUG
    applyDbgAction(state_.time.cycleStart);
#endif

    applySeed();

    // Pump again so a line that arrived while the block above ran (comms,
    // sensors, telemetry assembly all take real wall-clock time) is
    // drained before the frame goes out, matching the pre-kernel design's
    // multi-pump-per-cycle discipline (robot_loop.h's own header).
    comms_.pump(state_.time.cycleStart);

    const Comms::TlmAction tlmAction = comms_.takeTlmAction();
    const bool forceFrame = tlm_.applyAction(tlmAction);

    tlm_.emit(state_.time.cycleStart, forceFrame);

    comms_.updateStatus(state_, tlm_);

    comms_.sendTlmReply(tlmAction);
  });
}

#ifdef ROBOT_DEBUG
void RobotLoop::captureTuningBaseline() {
  if (dbgTuningBaselined_) return;
  dbgConfigBaseline_ = drive_.config();
  dbgTuningBaselined_ = true;
}

// applyDbgAction() -- kVmin/kASteady/kPos now operate DIRECTLY in the
// kernel's own counts domain (counts/s, counts/s^2, counts respectively)
// rather than the pre-kernel mm/s convention -- a documented unit-domain
// narrowing for these bench-only verbs; configuration-discipline.md's
// "development / bench tuning: ad-hoc single-value pushes are expected
// and allowed" covers exactly this. kGain used to scale two INDEPENDENT
// per-side dutyPerSpeed values; the kernel has one fullDutyVelocity slot
// (not two), so both action.value/action.value2 are averaged into one
// scale factor -- the per-side split this verb used to offer is gone
// with the old class, not a choice made carelessly here.
void RobotLoop::applyDbgAction(uint32_t now) {
  if (dbgWedgeUntilL_ != 0 && dbgWedgeUntilL_ != UINT32_MAX &&
      static_cast<int32_t>(now - dbgWedgeUntilL_) >= 0) {
    dbgWedgeUntilL_ = 0;
  }
  if (dbgWedgeUntilR_ != 0 && dbgWedgeUntilR_ != UINT32_MAX &&
      static_cast<int32_t>(now - dbgWedgeUntilR_) >= 0) {
    dbgWedgeUntilR_ = 0;
  }

  for (Comms::DbgAction action = comms_.takeDbgAction();
       action.kind != Comms::DbgActionKind::kNone;
       action = comms_.takeDbgAction()) {
  switch (action.kind) {
    case Comms::DbgActionKind::kNone:
      break;
    case Comms::DbgActionKind::kMark:
      Core::debugf("%s", action.text);
      break;
    case Comms::DbgActionKind::kPing:
      Core::debugf("pong");
      break;
    case Comms::DbgActionKind::kWedge: {
      // setForcedWedge() lived on Hal::Motor -- RobotLoop holds no motor
      // reference any more, so DBG-injected wedge fault injection has no
      // target left to reach in this exploratory tree. The deadline
      // bookkeeping above stays (harmless), the injection call is gone.
      const uint32_t until =
          action.duration == 0 ? UINT32_MAX : now + action.duration;
      if (action.port & 1) dbgWedgeUntilL_ = until;
      if (action.port & 2) dbgWedgeUntilR_ = until;
      Core::debugf("wedge armed port=%u dur=%lu (no-op: motor leaf unreachable from RobotLoop)",
                  action.port, static_cast<unsigned long>(action.duration));
      break;
    }
    case Comms::DbgActionKind::kVmin: {
      captureTuningBaseline();
      Control::DifferentialDrive::Config cfg = drive_.config();
      cfg.vMin = action.value;  // [counts/s]
      drive_.setConfig(cfg);
      const long milli = DBG_MILLI(action.value);
      Core::debugf("vmin %ld.%03ld applied", milli / 1000, milli % 1000);
      break;
    }
    case Comms::DbgActionKind::kASteady: {
      captureTuningBaseline();
      Control::DifferentialDrive::Config cfg = drive_.config();
      cfg.aSteady = action.value;  // [counts/s^2]
      drive_.setConfig(cfg);
      const long milli = DBG_MILLI(action.value);
      Core::debugf("asteady %ld.%03ld applied", milli / 1000, milli % 1000);
      break;
    }
    case Comms::DbgActionKind::kPos: {
      captureTuningBaseline();
      Control::DifferentialDrive::Config cfg = drive_.config();
      cfg.posErrMax = action.value;  // [counts]
      drive_.setConfig(cfg);
      const long milli = DBG_MILLI(action.value);
      Core::debugf("pos %ld.%03ld applied", milli / 1000, milli % 1000);
      break;
    }
    case Comms::DbgActionKind::kGain: {
      captureTuningBaseline();
      const float scale = 0.5f * (action.value + action.value2);
      Control::DifferentialDrive::Config cfg = drive_.config();
      cfg.fullDutyVelocity =
          (scale > 0.0f) ? dbgConfigBaseline_.fullDutyVelocity / scale : cfg.fullDutyVelocity;
      drive_.setConfig(cfg);
      const long milliLeft = DBG_MILLI(action.value);
      const long milliRight = DBG_MILLI(action.value2);
      Core::debugf("gain L=%ld.%03ld R=%ld.%03ld applied (averaged -- one kernel-wide fullDutyVelocity)",
                  milliLeft / 1000, milliLeft % 1000,
                  milliRight / 1000, milliRight % 1000);
      break;
    }
    case Comms::DbgActionKind::kClear:
      dbgWedgeUntilL_ = 0;
      dbgWedgeUntilR_ = 0;
      if (dbgTuningBaselined_) {
        drive_.setConfig(dbgConfigBaseline_);
        dbgTuningBaselined_ = false;
        Core::debugf("clear (tuning restored to boot)");
      } else {
        Core::debugf("clear");
      }
      break;
    case Comms::DbgActionKind::kOtos: {
      Core::debugf("otos before: present=%d connected=%d probeId=0x%02X",
                  otos_.present() ? 1 : 0, otos_.connected() ? 1 : 0,
                  static_cast<unsigned>(otos_.lastProbeId()));
      otos_.begin();
      Core::debugf("otos after: present=%d connected=%d probeId=0x%02X",
                  otos_.present() ? 1 : 0, otos_.connected() ? 1 : 0,
                  static_cast<unsigned>(otos_.lastProbeId()));
      break;
    }
    case Comms::DbgActionKind::kUnrecognized:
      Core::debugf("unrecognized dbg: %s", action.text);
      break;
  }
  }
}
#endif  // ROBOT_DEBUG

}  // namespace Core
