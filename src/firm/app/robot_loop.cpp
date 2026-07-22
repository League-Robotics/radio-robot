// robot_loop.cpp -- App::RobotLoop implementation. See robot_loop.h's file
// header for the module's boundary and entry points; DESIGN.md for the
// timing-schedule rationale.
#include "app/robot_loop.h"

#include "kinematics/body_kinematics.h"
#include "messages/envelope.h"

namespace App {

namespace {

// Loop timing constants. kSettle is the vendor settle window between a
// motor's own request and collect, shared by both motors' settle windows;
// kClear is the same clearance value NezhaMotor/Otos use for every
// bus_.write()/bus_.read() postClear/preClear pair, applied here as the
// post-duty-write clearance window.
//
// kCycle is the STATED TOTAL for the whole schedule (all four pacing
// blocks, not just the trailing one) -- ~50 Hz/20ms (115-005, gut S1:
// primary telemetry emission now happens every cycle, matching
// App::Telemetry's own kPrimaryPeriod=20ms (telemetry.h) so the
// primary-frame throttle and the loop's own pace agree by construction --
// closes kcycle-kprimaryperiod-mismatch.md).
constexpr uint32_t kSettle = 0;  // [ms] encoder-settle window, both motors
constexpr uint32_t kClear = 0;   // [ms] post-duty-write clearance window
constexpr uint32_t kCycle = 20;  // [ms] whole-schedule pace target (~50 Hz)

// kWindows is what the three settle/clearance blocks above already consume
// before the final (perception+odometry+pace) block runs; kPace is that
// final block's own gap, DERIVED so it absorbs kWindows into kCycle's total
// rather than stacking a fresh kCycle on top of it -- anchoring the final
// block to kCycle directly (instead of kPace) would double-count kWindows
// under a zero-real-time-cost virtual clock, where every block's own
// elapsed-since-mark is provably 0 (each of the four blocks would then
// request its full nominal gap on top of the others instead of the whole
// schedule summing to kCycle). Passing kPace to the final block's own
// runAndWait keeps the schedule's four blocks summing to exactly kCycle
// under that worst case, the same invariant the other three blocks already
// have individually.
constexpr uint32_t kWindows = 2 * kSettle + kClear;  // [ms] time the 3 settle/clear
                                                      // blocks consume before the pace block
static_assert(kWindows <= kCycle,
              "kSettle+kClear+kSettle must fit inside the kCycle budget");
constexpr uint32_t kPace = kCycle - kWindows;  // [ms] final block's own gap, absorbing kWindows

constexpr uint32_t kPreamblePace = 10;  // [ms] boot-loop probe pacing

// --- 114-004 (SUC-003) persisted-tuning merge helpers -- pure struct
// merges, no RobotLoop state needed, so these stay free functions rather
// than private methods. ---

// mergeMotorGainsPatch -- folds `incoming`'s PRESENT gain fields onto
// `slot` (a running per-side TuningSnapshot merge target). Gains mirror
// onto BOTH bound motors regardless of `incoming.side` (matching
// applyMotorConfigPatch()'s own existing mirror below), so handleConfig()
// calls this once per side with the SAME incoming patch. travel_calib is
// intentionally excluded here -- it is side-selected, merged separately by
// handleConfig() itself, only into the ADDRESSED side's own slot.
void mergeMotorGainsPatch(msg::MotorConfigPatch& slot, const msg::MotorConfigPatch& incoming) {
  if (incoming.kp.has) slot.kp = incoming.kp;
  if (incoming.ki.has) slot.ki = incoming.ki;
  if (incoming.kff.has) slot.kff = incoming.kff;
  if (incoming.i_max.has) slot.i_max = incoming.i_max;
  if (incoming.kaw.has) slot.kaw = incoming.kaw;
}

// mergeOtosPatch -- `init` is deliberately excluded: a one-shot trigger,
// not a persisted value (persisted_tuning.h's own TuningSnapshot doc
// comment explains why).
void mergeOtosPatch(msg::OtosConfigPatch& slot, const msg::OtosConfigPatch& incoming) {
  if (incoming.linear_scale.has) slot.linear_scale = incoming.linear_scale;
  if (incoming.angular_scale.has) slot.angular_scale = incoming.angular_scale;
  if (incoming.offset_x.has) slot.offset_x = incoming.offset_x;
  if (incoming.offset_y.has) slot.offset_y = incoming.offset_y;
  if (incoming.offset_yaw.has) slot.offset_yaw = incoming.offset_yaw;
}

// packLine -- 4 raw grayscale channels (each already a single-byte I2C
// read, line_sensor.cpp's own readRaw()) into one uint32, ch1 in the low
// byte -- telemetry.proto's own `line` field layout.
uint32_t packLine(const Devices::LineReading& reading) {
  return (reading.raw[0] & 0xFFu) | ((reading.raw[1] & 0xFFu) << 8) |
         ((reading.raw[2] & 0xFFu) << 16) | ((reading.raw[3] & 0xFFu) << 24);
}

// packColor -- RGBC, each scaled from the chip's native 16-bit register
// down to 8 bits (top byte) into one uint32, R in the low byte --
// telemetry.proto's own `color` field layout.
uint32_t packColor(const Devices::ColorReading& reading) {
  return ((reading.r >> 8) & 0xFFu) | (((reading.g >> 8) & 0xFFu) << 8) |
         (((reading.b >> 8) & 0xFFu) << 16) | (((reading.c >> 8) & 0xFFu) << 24);
}

}  // namespace

RobotLoop::RobotLoop(Devices::I2CBus& bus, Devices::Motor& motorL,
                      Devices::Motor& motorR, Devices::Otos& otos,
                      Devices::ColorSensorLeaf& color, Devices::LineSensorLeaf& line,
                      Comms& comms, Telemetry& tlm, Drive& drive,
                      Odometry& odom, MoveQueue& moveQueue, Preamble& preamble,
                      const Devices::Clock& clock, Devices::Sleeper& sleeper,
                      Config::TuningStore* tuningStore)
    : bus_(bus),
      motorL_(motorL),
      motorR_(motorR),
      otos_(otos),
      color_(color),
      line_(line),
      comms_(comms),
      tlm_(tlm),
      drive_(drive),
      odom_(odom),
      moveQueue_(moveQueue),
      preamble_(preamble),
      clock_(clock),
      sleeper_(sleeper),
      tuningStore_(tuningStore) {}

// --- Timing primitives -- see robot_loop.h's header. markTime() reads
// clock_.nowMicros() ([us]) and converts to [ms], the unit every other
// timing constant/field in this file uses. sleepUntil() always sleeps
// >=1ms, never a zero-length "sleep" (that would be a spin in disguise),
// so it is always a real yield back to the radio/serial fibers on the real
// Sleeper impl -- no runAndWait block can ever degrade into a busy-wait. ---

uint32_t RobotLoop::markTime() const {
  return static_cast<uint32_t>(clock_.nowMicros() / 1000);  // [us] -> [ms]
}

void RobotLoop::sleepUntil(uint32_t mark, uint32_t gap) {  // [ms] [ms]
  uint32_t elapsed = markTime() - mark;
  uint32_t remaining = (elapsed < gap) ? (gap - elapsed) : 0;
  sleeper_.sleepMillis(remaining > 0 ? remaining : 1);
}

template <typename Body>
void RobotLoop::runAndWait(uint32_t gap, Body body) {  // [ms]
  uint32_t mark = markTime();
  body();
  sleepUntil(mark, gap);
}

void RobotLoop::updateTlm(uint32_t now) {  // [ms]
  frame_.mode = moveQueue_.active() ? msg::DriveMode::VELOCITY : msg::DriveMode::IDLE;

  frame_.encLeft.position = motorL_.position();
  frame_.encLeft.velocity = motorL_.velocity();
  frame_.encLeft.time = now;
  frame_.encRight.position = motorR_.position();
  frame_.encRight.velocity = motorR_.velocity();
  frame_.encRight.time = now;

  // Fused body-frame velocity (109-009 fix, carried forward): the two
  // leaves' current velocities through BodyKinematics::forward() yield the
  // fused body (v, omega) for THIS instant, the same equations Odometry
  // uses for per-cycle distance/headingDelta.
  BodyKinematics::forward(motorL_.velocity(), motorR_.velocity(), drive_.trackWidth(),
                           frame_.twist.v_x, frame_.twist.omega);

  tlm_.setFlag(kFlagActive, moveQueue_.active());
  tlm_.setFlag(kFlagConnLeft, motorL_.connected());
  tlm_.setFlag(kFlagConnRight, motorR_.connected());

  tlm_.setFlag(kFlagFaultI2CSafetyNet, bus_.clearanceSafetyNetCount() > 0);
  tlm_.setFlag(kFlagFaultWedgeLatch, motorL_.wedged() || motorR_.wedged());
  tlm_.setFlag(kFlagFaultCommsMalformed, comms_.malformedCount() > 0);

  tlm_.setFrame(frame_);
}

void RobotLoop::updateLineColor(uint64_t nowUs) {  // [us]
  bool lineFresh = false;
  bool colorFresh = false;

  if (lineTurnNext_) {
    line_.tick(nowUs);
    lineFresh = line_.readingFresh();
    if (lineFresh) frame_.line = packLine(line_.reading());
  } else {
    color_.tick(nowUs);
    colorFresh = color_.readingFresh();
    if (colorFresh) frame_.color = packColor(color_.reading());
  }
  lineTurnNext_ = !lineTurnNext_;

  tlm_.setFlag(kFlagLinePresent, lineFresh);
  tlm_.setFlag(kFlagColorPresent, colorFresh);
}

// handleMove -- replaces the deleted handleTwist() (116, protocol-set-point
// issue). Configuration-completeness gate FIRST (unchanged position/
// semantics from handleTwist()), then shape validation (a well-formed Move
// per the wire contract: a velocity variant present, a stop variant
// present, timeout > 0), then delegates to moveQueue_.enqueue() --
// move_queue.h's own boundary comment: "every Move this class's enqueue()
// ever sees is already permitted" is exactly this validation.
void RobotLoop::handleMove(const msg::CommandEnvelope& env) {
  // Configuration-completeness gate (114-001, SUC-001) -- FIRST statement,
  // before touching drive_/moveQueue_ at all. Real firmware satisfies this
  // immediately at boot (Decision 2, sprint.md) -- this branch is only
  // ever live for a composition root (SimHarness) that has not yet been
  // configured.
  if (!configured_) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_NOT_CONFIGURED));
    return;
  }

  const msg::Move& move = env.cmd.move;
  if (move.velocity_kind == msg::Move::VelocityKind::NONE ||
      move.stop_kind == msg::Move::StopKind::NONE || move.timeout <= 0.0f) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_BADARG));
    return;
  }

  MoveQueue::EnqueueResult result = moveQueue_.enqueue(move, env.corr_id);
  tlm_.ack(result.corrId, static_cast<uint32_t>(result.err));
}

// ConfigDelta runtime application: MotorConfigPatch and OtosConfigPatch
// (109-004) are live-applied below; every other patch kind (DRIVETRAIN/
// WATCHDOG/NONE) stays ERR_UNIMPLEMENTED, deliberately out of scope -- see
// DESIGN.md §3. PlannerConfigPatch (109-008's un-stub) is GONE -- 115-005
// (gut S1) deleted msg::PlannerConfigPatch and ConfigDelta's own PLANNER
// arm along with the rest of the motion stack; there is no third live
// branch here any more.
//
// 114-004 (SUC-003): each live branch below now ALSO merges the incoming
// patch's PRESENT fields into persistedTuning_ (the running cumulative
// live-tuning snapshot) and calls persistTuningIfChanged() -- the actual
// apply-to-RAM behavior on motorL_/motorR_/otos_ is UNCHANGED from before
// this ticket (applyMotorConfigPatch()/applyOtosPatch() below are verbatim
// extractions of what used to be inline here).
void RobotLoop::handleConfig(const msg::CommandEnvelope& env) {
  // OTOS (109-004, issue otos-calibration-config-message.md): restores a
  // runtime path to Otos::setLinearScalar()/setAngularScalar()/setOffset()/
  // init() -- previously only ever called once at boot from baked
  // boot_config. Direct, immediate calls (no staging): otos.h's own doc
  // comment for these four primitives already documents them as issuing
  // their I2C write immediately, "matching the OI/OR/OL/OA wire-command
  // shape" -- exactly this call site. This is still "the loop's own cycle"
  // doing the bus traffic (DESIGN.md §3's single-loop bus ownership
  // invariant): handleConfig() runs synchronously inside RobotLoop::cycle()
  // (via processMessage()), never from Otos's own tick()/staging methods or
  // an ISR -- it is a rare, command-triggered transaction sandwiched into
  // the loop's existing schedule, not a new per-cycle bus consumer.
  if (env.cmd.config.patch_kind == msg::ConfigDelta::PatchKind::OTOS) {
    const msg::OtosConfigPatch& patch = env.cmd.config.patch.otos;

    applyOtosPatch(patch);
    mergeOtosPatch(persistedTuning_.otos, patch);
    persistTuningIfChanged();

    tlm_.ack(env.corr_id, 0);
    return;
  }

  if (env.cmd.config.patch_kind != msg::ConfigDelta::PatchKind::MOTOR) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_UNIMPLEMENTED));
    return;
  }

  const msg::MotorConfigPatch& patch = env.cmd.config.patch.motor;

  // Merge into BOTH sides' persisted slots (gains mirror onto both bound
  // motors, matching applyMotorConfigPatch()'s own mirror below); merge
  // travel_calib into ONLY the addressed side's own slot (side-selected,
  // like the apply itself). `side` is re-stamped every call so a slot that
  // has never seen its own side-matching patch yet still deserializes with
  // the correct side (harmless if already correct).
  mergeMotorGainsPatch(persistedTuning_.motorL, patch);
  mergeMotorGainsPatch(persistedTuning_.motorR, patch);
  if (patch.travel_calib.has) {
    msg::MotorConfigPatch& target = (patch.side == msg::BoundMotorSide::LEFT)
                                         ? persistedTuning_.motorL
                                         : persistedTuning_.motorR;
    target.travel_calib = patch.travel_calib;
  }
  persistedTuning_.motorL.side = msg::BoundMotorSide::LEFT;
  persistedTuning_.motorR.side = msg::BoundMotorSide::RIGHT;

  applyMotorConfigPatch(persistedTuning_.motorL);
  applyMotorConfigPatch(persistedTuning_.motorR);
  persistTuningIfChanged();

  tlm_.ack(env.corr_id, 0);
}

// applyMotorConfigPatch -- UNCHANGED extraction of what used to be
// handleConfig()'s own inline MOTOR-branch logic (114-004's own Approach
// step 4: reapplyPersistedTuning(), below, shares this exact applier
// instead of duplicating it). Merges each motor's OWN current gains
// against whatever wire fields are PRESENT (config.proto's Opt<T>-presence
// convention) -- NOT a blanket mirror of one motor's gains onto the other,
// since the two leaves' calibration can legitimately differ. travel_calib
// is side-selected (config.proto's own MotorConfigPatch.side comment) --
// applies to exactly one leaf.
void RobotLoop::applyMotorConfigPatch(const msg::MotorConfigPatch& patch) {
  Devices::Gains gainsL = motorL_.gains();
  Devices::Gains gainsR = motorR_.gains();
  if (patch.kp.has) { gainsL.kp = patch.kp.val; gainsR.kp = patch.kp.val; }
  if (patch.ki.has) { gainsL.ki = patch.ki.val; gainsR.ki = patch.ki.val; }
  if (patch.kff.has) { gainsL.kff = patch.kff.val; gainsR.kff = patch.kff.val; }
  if (patch.i_max.has) { gainsL.iMax = patch.i_max.val; gainsR.iMax = patch.i_max.val; }
  if (patch.kaw.has) { gainsL.kaw = patch.kaw.val; gainsR.kaw = patch.kaw.val; }

  Devices::Opt<float> travelCalibL;
  Devices::Opt<float> travelCalibR;
  if (patch.travel_calib.has) {
    if (patch.side == msg::BoundMotorSide::LEFT) {
      travelCalibL.has = true;
      travelCalibL.val = patch.travel_calib.val;
    } else {
      travelCalibR.has = true;
      travelCalibR.val = patch.travel_calib.val;
    }
  }

  motorL_.applyGains(gainsL, travelCalibL);
  motorR_.applyGains(gainsR, travelCalibR);
}

// applyOtosPatch -- UNCHANGED extraction of what used to be
// handleConfig()'s own inline OTOS-branch logic. Offset triple is
// merge-then-write: setOffset() always writes x/y/heading together, so any
// field NOT present in this patch must carry the chip's own current value,
// read via getOffset() first, rather than clobbering it with 0. init is a
// plain trigger (not Opt<T>-wrapped) -- fire whenever true.
void RobotLoop::applyOtosPatch(const msg::OtosConfigPatch& patch) {
  if (patch.linear_scale.has) otos_.setLinearScalar(patch.linear_scale.val);
  if (patch.angular_scale.has) otos_.setAngularScalar(patch.angular_scale.val);

  if (patch.offset_x.has || patch.offset_y.has || patch.offset_yaw.has) {
    float x = 0.0f, y = 0.0f, heading = 0.0f;
    otos_.getOffset(x, y, heading);
    if (patch.offset_x.has) x = patch.offset_x.val;
    if (patch.offset_y.has) y = patch.offset_y.val;
    if (patch.offset_yaw.has) heading = patch.offset_yaw.val;
    otos_.setOffset(x, y, heading);
  }

  if (patch.init) otos_.init();
}

// persistTuningIfChanged -- 114-004 write policy (sprint.md Open Question
// 3: flash-write frequency/wear risk). CHANGE-DETECTION debounce: only
// calls tuningStore_->save() when this call's freshly-serialized
// persistedTuning_ blob differs from the last one actually written. A
// bench-tuning session streaming CFG patches rapidly (e.g. a TestGUI
// slider) would otherwise write flash on every single patch -- both a
// per-write latency risk inside a live control session and, over many
// sessions, page wear on a finite-endurance flash region shared with
// com/radio_channel.h's own persisted key (a real, not hypothetical,
// constraint -- see persisted_tuning.cpp's own kNumChunks budget). A
// patch that sets a field to the value it already holds, or that touches
// no persisted field at all, costs zero flash writes under this policy.
// Skipped entirely (no flash access, no serialize call) when tuningStore_
// is null -- every sim/test composition root's own case.
void RobotLoop::persistTuningIfChanged() {
  if (tuningStore_ == nullptr) return;

  Config::Blob blob = Config::serializeSnapshot(persistedTuning_);
  if (blob == lastPersistedBlob_) return;

  tuningStore_->save(Config::kConfigSchemaVersion, blob);
  lastPersistedBlob_ = blob;
}

void RobotLoop::reapplyPersistedTuning(const Config::TuningSnapshot& snapshot) {
  applyMotorConfigPatch(snapshot.motorL);
  applyMotorConfigPatch(snapshot.motorR);
  applyOtosPatch(snapshot.otos);

  persistedTuning_ = snapshot;
  lastPersistedBlob_ = Config::serializeSnapshot(persistedTuning_);
}

void RobotLoop::handleStop(const msg::CommandEnvelope& env) {
  drive_.stop();
  moveQueue_.flush();
  tlm_.ack(env.corr_id, 0);
}

// Dispatches the <=1 decoded command in cmd to its own handler by
// cmd_kind. `cmd` is a fresh, cycle-local variable (populated by at most
// one comms_.pump() call this cycle), so reading it here bounds dispatch
// to at most once per cycle by construction -- no separate "take" flag
// needed.
void RobotLoop::processMessage(const Cmd& cmd) {
  msg::CommandEnvelope::CmdKind kind = (cmd.status == CmdStatus::kDecoded)
      ? cmd.env.cmd_kind
      : msg::CommandEnvelope::CmdKind::NONE;
  switch (kind) {
    case msg::CommandEnvelope::CmdKind::MOVE:
      handleMove(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::CONFIG:
      handleConfig(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::STOP:
      handleStop(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::NONE:
    default:
      break;
  }
}

[[noreturn]] void RobotLoop::run() {
  boot();
  for (;;) {
    cycle();
  }
}

// ---- Boot: resolve every device before entering the control loop.
// Telemetry flows from power-on (frames report per-device status), so the
// host can tell booting from dead; commands are not consumed until the
// main loop starts (no Comms::pump() call here). ----
void RobotLoop::boot() {
  while (!preamble_.done()) {
    preamble_.step();  // one bounded probe action per pass

    Telemetry::Frame bootFrame;
    tlm_.setFrame(bootFrame);
    tlm_.setFlag(kFlagConnLeft, preamble_.leftConnected());
    tlm_.setFlag(kFlagConnRight, preamble_.rightConnected());
    tlm_.setFlag(kFlagOtosConnected, preamble_.otosConnected());
    tlm_.emit(markTime());  // boot frames: device detection status, faults

    sleeper_.sleepMillis(kPreamblePace);  // paces probes AND yields (radio RX)
  }
  tlm_.setFlag(kFlagEventBootReady, true);  // Preamble::done() first-true transition
}

// ---- Main cycle: devices resolved, no readiness checks below this line.
// TIMING: device calls are pure bus transactions and never sleep. Every
// required gap is a runAndWait block: it marks time on entry (immediately
// after the bus event that starts the clock), runs its body, then sleeps
// until at least the gap has elapsed since the mark. The block visibly
// scopes exactly the work that borrows the dead time. I2CBus keeps
// per-device readyAt stamps as a sleep-not-spin safety net (+ telemetry
// fault bit), so a mis-ordered loop degrades loudly, never silently. The
// three settle/clearance blocks' own bodies never touch the bus and never
// sleep; the schedule's 4th block (the trailing perception+odometry+pace
// block, kPace) is the one exception -- see its own comment below. ----
void RobotLoop::cycle() {
  uint32_t cycleStart = markTime();  // [ms] pace anchor

  Cmd cmd;

  // Request/collect MUST interleave per port: the 0x46 encoder-select is a
  // single latched state on the brick (one pending read; SimPlant models
  // the same via selectedPort_) -- issuing both selects before either
  // collect makes BOTH motors read the LAST-selected port's encoder
  // (observed 2026-07-18: an unmanaged pivot showed actual L == actual R
  // glued to the right wheel while cmd L/R were correctly mirrored).

  // 112-005 cycle-order fix (cut the trim/PD-loop dead time that caused the
  // terminal jitter): stage this cycle's wheel targets BEFORE the motor
  // ticks, so the target staged last cycle is WRITTEN this cycle instead of
  // next (-1 cycle).
  drive_.tick();  // twist -> wheel targets, written by THIS cycle's motor ticks

  // Request/collect MUST interleave per port: the 0x46 encoder-select is a
  // single latched state on the brick (one pending read) -- issuing both
  // selects before either collect makes BOTH motors read the last-selected
  // port's encoder.
  motorL_.requestSample();  // 0x46 write (brick holds ONE pending read)
  motorL_.tick(clock_.nowMicros());   // write L duty (fresh target) + collect L
  motorR_.requestSample();
  motorR_.tick(clock_.nowMicros());   // write R duty + collect R

  runAndWait(kSettle, [&] {           // >=4ms: L encoder settling, meanwhile --
    comms_.pump(cmd, cycleStart);     //   drain RX, decode <=1 frame into cmd
  });

  runAndWait(kClear, [&] {  // >=4ms: brick clears L's duty write, meanwhile --
    // Stage this cycle's encoder/velocity/connection fields onto the
    // persistent `frame_` (pose/otos/line/color were last updated at the
    // END of the PREVIOUS cycle, below -- still the frame's own "last
    // staged snapshot" contract) and emit.

    updateTlm(cycleStart);
    tlm_.emit(cycleStart);
  });

  runAndWait(kSettle, [&] {  // >=4ms: R encoder settling, meanwhile --
    // Apply <=1 decoded command; every path that applies one acks via
    // tlm_.ack(). `cmd` is a fresh, cycle-local variable (declared above,
    // populated by at most one comms_.pump() call this cycle), so reading
    // it here bounds dispatch to at most once per cycle by construction --
    // no separate "take" flag is needed.
    processMessage(cmd);

    // MoveQueue's per-cycle tick (116, protocol-set-point issue) --
    // replaces the deleted deadman_.expired() branch at this EXACT
    // schedule position. This is the load-bearing safety property
    // (SUC-053): it runs unconditionally, every cycle, regardless of
    // whether a command arrived this cycle -- the same way
    // deadman_.expired() did. Ends the active Move on StopConditionMet or
    // TimedOut, either chain-advancing the next pending Move THIS SAME
    // cycle (seamless hand-off, SUC-051) or calling Drive::stop() with an
    // empty queue (MoveQueue::tick()'s own contract) -- so host silence
    // always ends in motors stopped, with zero further host traffic
    // needed (no deadman lease to re-arm).
    MoveQueue::TickResult moveResult = moveQueue_.tick(clock_.nowMicros(), odom_);
    bool moveTimedOut = moveResult.completed && moveResult.completion.timedOut;
    // Level-set every cycle (telemetry.h's own setFlag() contract) -- true
    // only on the exact cycle a timed-out completion is reported this
    // call, false every other cycle (SUC-054).
    tlm_.setFlag(kFlagFaultMoveTimeout, moveTimedOut);
    if (moveResult.completed) {
      // MOVE completion ack (protocol-set-point issue, Responses section):
      // a SECOND ack on the cycle the command ends -- ack_corr ==
      // Move.id, ack_err == 0 regardless of outcome; a timeout ending is
      // distinguished by the flags bit set just above, not by ack_err.
      tlm_.ack(moveResult.completion.moveId, 0);
    }
  });

  // Final (perception + odometry + pace) block -- the schedule's 4th
  // runAndWait, matching the same "own mark, own gap" shape as the three
  // settle/clearance blocks above (see kPace's own comment for why the gap
  // must be derived, not a bare kCycle anchored to the cycle start). Body:
  // OTOS + odometry + rate-limited alternating line/color, outside any
  // motor request/collect window (this class's own bus-discipline
  // responsibility) -- all stage into `frame_` for the NEXT cycle's
  // tlm_.setFrame()/emit() call. Unlike the other three blocks, this one
  // DOES touch the bus (OTOS, and at most one of line/color) -- it is the
  // schedule's pace block, not a settle/clearance window, so that
  // constraint doesn't apply to it.
  runAndWait(kPace, [&] {
    uint64_t nowUs = clock_.nowMicros();

    applyOtosSample(otos_, nowUs, frame_);
    tlm_.setFlag(kFlagOtosPresent, frame_.otosPresent);
    tlm_.setFlag(kFlagOtosConnected, frame_.otosConnected);

    odom_.integrate();  // odometry from both fresh wheel samples
    frame_.pose = {odom_.x(), odom_.y(), odom_.theta()};

    updateLineColor(nowUs);
  });
}

}  // namespace App
