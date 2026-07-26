// robot_loop.cpp -- App::RobotLoop implementation. See robot_loop.h's file
// header for the module's boundary and entry points; DESIGN.md for the
// timing-schedule rationale.
#include "app/robot_loop.h"

#include <cmath>

#include "motion/body_kinematics.h"
#include "messages/envelope.h"

namespace App {

namespace {

// Loop timing constants. kSettle is the vendor settle window between a
// motor's own request and collect, shared by both motors' settle windows;
// kClear is the same clearance value NezhaMotor/Otos use for every
// bus_.write()/bus_.read() postClear/preClear pair, applied here as the
// post-duty-write clearance window.
//
// kCycle itself now lives on RobotLoop::kCycle (robot_loop.h, public) --
// 118 ticket 003 promoted it out of this anonymous namespace so external
// composition roots (TestSim::SimHarness's own kCycleDtUs) can derive from
// the SAME declaration instead of an independently-hardcoded matching
// literal. It is the STATED TOTAL for the whole schedule (all four pacing
// blocks, not just the trailing one) -- ~25 Hz/40ms (106-001; restored by
// 118 after commit 5f5a2ba7 zeroed kSettle/kClear and halved kCycle to 20,
// which only made the vendor's still-mandatory 4ms settle happen as a
// blocking sleep hidden inside motorL_.tick()/motorR_.tick() instead --
// see clasi/issues/restore-the-interleaved-request-settle-tick-loop-schedule.md),
// matching App::Telemetry's own kPrimaryPeriod=40ms (telemetry.h) so the
// primary-frame throttle and the loop's own pace agree by construction.
constexpr uint32_t kSettle = 4;  // [ms] encoder-settle window, both motors
constexpr uint32_t kClear = 4;   // [ms] post-duty-write clearance window

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
static_assert(kWindows <= RobotLoop::kCycle,
              "kSettle+kClear+kSettle must fit inside the kCycle budget");
constexpr uint32_t kPace = RobotLoop::kCycle - kWindows;  // [ms] final block's own gap, absorbing kWindows

constexpr uint32_t kPreamblePace = 10;  // [ms] boot-loop probe pacing

// Position-rebaseline policy (124-008, sprint 124 architecture Decision 6,
// revised twice -- see sprint.md). kPositionWireBound MUST match
// telemetry.proto's EncoderReading.position (abs_max) exactly. The 2000mm
// margin dwarfs one cycle's worst-case travel (wheel speeds in the low
// hundreds of mm/s at ~25Hz => tens of mm per cycle at most), so the
// trigger is expected to fire long before the bound itself is ever
// approached in normal operation -- see assembleFrame()'s own comment at
// the call site for the full mechanism.
constexpr float kPositionWireBound = 32000.0f;       // [mm] EncoderReading.position (abs_max)
constexpr float kPositionRebaselineMargin = 30000.0f;  // [mm] trigger threshold -- 2000mm below the bound

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

// mergeEstimatorPatch (117, ticket 003) -- folds `patch`'s PRESENT fields
// onto `weights` (a snapshot of stateEstimator_.weights(), taken by
// handleConfig()'s own ESTIMATOR branch before calling setWeights()).
// UNLIKE mergeMotorGainsPatch()/mergeOtosPatch() above, this merges
// directly onto the live App::FusionWeights value itself, not a
// persistedTuning_ slot -- EstimatorConfigPatch is never persisted (Design
// Rationale Decision 4, this sprint's overlay design/design.md): a reboot
// always reverts to the baked Config::defaultEstimatorConfig() default.
void mergeEstimatorPatch(Motion::FusionWeights& weights, const msg::EstimatorConfigPatch& patch) {
  if (patch.weight_heading_otos.has) weights.headingOtos = patch.weight_heading_otos.val;
  if (patch.weight_omega_otos.has) weights.omegaOtos = patch.weight_omega_otos.val;
  if (patch.staleness_ms.has) weights.staleness = static_cast<uint32_t>(patch.staleness_ms.val);
}

// mergeShaperPatch (decel-into-the-goal campaign) -- folds `patch`'s
// PRESENT a_max/a_decel/alpha_max/alpha_decel/j_max/yaw_jerk_max fields
// onto `limits` (a snapshot of moveQueue_.shaperLimits(), taken by
// handleConfig()'s own ESTIMATOR branch before calling setShaperLimits())
// -- the SAME present-field-merge shape mergeEstimatorPatch() immediately
// above uses, applied to App::ShaperLimits instead of App::FusionWeights.
// j_max/yaw_jerk_max (jerk-limited S-curve stage) ride the same merge.
// Also never persisted (config.proto's own EstimatorConfigPatch doc
// comment) -- a reboot always reverts to the baked
// Config::defaultShaperConfig() default.
void mergeShaperPatch(Motion::ShaperLimits& limits, const msg::EstimatorConfigPatch& patch) {
  if (patch.a_max.has) limits.aMax = patch.a_max.val;
  if (patch.a_decel.has) limits.aDecel = patch.a_decel.val;
  if (patch.alpha_max.has) limits.alphaMax = patch.alpha_max.val;
  if (patch.alpha_decel.has) limits.alphaDecel = patch.alpha_decel.val;
  if (patch.j_max.has) limits.jMax = patch.j_max.val;
  if (patch.yaw_jerk_max.has) limits.yawJerkMax = patch.yaw_jerk_max.val;
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
                      Motion::Odometry& odom, Motion::MoveQueue& moveQueue, Preamble& preamble,
                      Motion::StateEstimator& stateEstimator, const Devices::Clock& clock,
                      Devices::Sleeper& sleeper, Config::TuningStore* tuningStore)
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
      stateEstimator_(stateEstimator),
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

// clampToPositionWireBound -- see robot_loop.h's own declaration comment
// for why this is a public static (unit-testable in isolation from a live
// Motor's rebaseline() behavior).
float RobotLoop::clampToPositionWireBound(float pos, bool* clamped) {
  *clamped = std::fabs(pos) > kPositionWireBound;
  return *clamped ? std::copysign(kPositionWireBound, pos) : pos;
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

  // now -- read HERE, at the exact point in the cycle App::MoveQueue's own
  // pre-122-002 held Devices::Clock& used to read it internally (see
  // move_queue.h's own file header) -- Motion::MoveQueue may not depend on
  // devices/, so the caller (this class) now threads it through explicitly.
  Motion::MoveQueue::EnqueueResult result =
      moveQueue_.enqueue(move, env.corr_id, clock_.nowMicros());
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

  // ESTIMATOR (117, ticket 003): App::StateEstimator's own live fusion-
  // weight tuning arm. A pure in-memory update -- no I2C bus access, unlike
  // the OTOS branch above -- so it needs neither a bus-ownership comment nor
  // a persistTuningIfChanged() call: EstimatorConfigPatch is DELIBERATELY
  // never persisted into persistedTuning_/flash (Design Rationale Decision
  // 4, this sprint's overlay design/design.md) -- a reboot always reverts
  // to the baked Config::defaultEstimatorConfig() default. Present-field
  // merge onto a snapshot of the CURRENT live weights, mirroring
  // applyMotorConfigPatch()/applyOtosPatch()'s own merge-then-apply shape.
  //
  // a_max/a_decel/alpha_max/alpha_decel/j_max/yaw_jerk_max (decel-into-the-
  // goal campaign) ride this SAME arm, targeting moveQueue_'s ShaperLimits
  // directly -- the "smallest coherent path" reasoning config.proto's own
  // EstimatorConfigPatch doc comment gives (CONFIG_ESTIMATOR is already the
  // live-tune arm for MoveQueue-owned, non-FusionWeights state, so a fresh
  // ConfigTarget/Patch-message pair for one more MoveQueue-owned field
  // group would duplicate plumbing for no behavioral gain). Present-field
  // merge onto a snapshot of the CURRENT live ShaperLimits, mirroring the
  // FusionWeights merge above; applied independently of every other field
  // on this patch.
  //
  // The turn-prediction campaign's own time-lead anticipation constant
  // (formerly EstimatorConfigPatch field 4) -- DELETED (118 ticket 004,
  // land-at-zero-completion-delete-stop-lead.md): App::MoveQueue no longer
  // has a field to apply it to (the anticipation-lead completion mechanism
  // it drove is deleted -- see move_queue.h's own tick() doc comment for
  // the land-at-zero replacement). The wire field itself is `reserved` in
  // config.proto, not reused or removed from the wire -- a legacy client
  // still sending it is silently ignored (parses fine, has no effect),
  // never a decode error.
  if (env.cmd.config.patch_kind == msg::ConfigDelta::PatchKind::ESTIMATOR) {
    const msg::EstimatorConfigPatch& patch = env.cmd.config.patch.estimator;

    Motion::FusionWeights weights = stateEstimator_.weights();
    mergeEstimatorPatch(weights, patch);
    stateEstimator_.setWeights(weights);

    Motion::ShaperLimits shaperLimits = moveQueue_.shaperLimits();
    mergeShaperPatch(shaperLimits, patch);
    moveQueue_.setShaperLimits(shaperLimits);

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

// rejectDuringBoot -- see robot_loop.h's own doc comment. Mirrors
// handleMove()'s configured_ gate (ERR_NOT_CONFIGURED, same error code) but
// applies uniformly to any decoded command kind (MOVE/CONFIG/STOP) -- boot()
// never dispatches into handleMove()/handleConfig()/handleStop() at all
// (those touch moveQueue_/motorL_/motorR_/otos_, none of which are safe to
// act on before Preamble::done()), so this is boot()'s own single ack path,
// not a call into the cycle()-time handlers.
void RobotLoop::rejectDuringBoot(const Cmd& cmd) {
  if (cmd.status != CmdStatus::kDecoded) return;
  tlm_.ack(cmd.env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_NOT_CONFIGURED));
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
// Telemetry flows from power-on (frames report per-device status), and the
// text rump stays live so HELLO/PING can classify a rebooting robot while
// the preamble is still probing devices. A binary command decoded during
// this window is never executed AND never silently dropped -- rejectDuringBoot()
// (125-001) explicitly NACKs it (ERR_NOT_CONFIGURED) instead; the main loop
// is what actually dispatches commands, starting once boot() returns. ----
void RobotLoop::boot() {
  while (!preamble_.done()) {
    Cmd bootCmd;
    comms_.pump(bootCmd, markTime());
    rejectDuringBoot(bootCmd);  // 125-001: explicit NACK, never a silent drop

    preamble_.step();  // one bounded probe action per pass

    // A throwaway boot-time RobotState -- boot() has no real cycle to
    // publish sections from yet, only the per-device connectivity
    // Preamble has probed so far. Routing it through tlm_.update() (rather
    // than a direct flag poke) keeps every Telemetry flags_ mutation going
    // through the SAME two entry points (update()/setLiveFlag()) the main
    // loop uses -- see telemetry.h's own doc comments.
    Types::RobotState bootState;
    bootState.time.cycleStart = markTime();
    bootState.wheelLeft.connected = preamble_.leftConnected();
    bootState.wheelRight.connected = preamble_.rightConnected();
    bootState.otos.connected = preamble_.otosConnected();
    tlm_.update(bootState);
    tlm_.emit(bootState.time.cycleStart);  // boot frames: device detection status, faults

    sleeper_.sleepMillis(kPreamblePace);  // paces probes AND yields (radio RX)
  }
  // kFlagEventBootReady (Preamble::done() first-true transition) -- not one
  // of update()'s derived bits, so the loop above never touches it;
  // setLiveFlag() is the same live-mutation escape hatch cycle()'s own
  // post-tick fault bits use (telemetry.h's own doc comment covers both
  // uses).
  tlm_.setLiveFlag(kFlagEventBootReady, true);

  comms_.sendBanner();  // preamble done, main loop next -- see Comms::sendBanner()
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
  // Time section -- published FIRST (robot_state.h's own Time doc
  // comment): state_.time.cycleStart is the wire `now` field's own value
  // (the same instant this cycle's own tlm_.emit(cycleStart) call passes)
  // and the base cycleBusy/cyclePeriod are measured against, below.
  // Telemetry::update()'s own age computation uses a LATER derived instant
  // (cycleStart + cycleBusy) -- see that method's own doc comment
  // (telemetry.h) for why cycleStart alone is too early for that purpose.
  state_.time.cycleStart = cycleStart;

  // 122-003: a SEPARATE, un-truncated [us] read of the SAME instant
  // markTime() just anchored, kept only for the loop-timing telemetry
  // fields below -- markTime()'s own [ms] truncation would hide the
  // sub-millisecond I2C-stall/comms-burst detail those fields exist to
  // surface. Read here, at cycle()'s own top, so cycleBusy below measures
  // from the same starting instant cycleStart already anchors.
  const uint64_t cycleStartUs = clock_.nowMicros();  // [us]

  Cmd cmd;

  // 119 ticket 005 (fixes 118-001's straight-leg crab -- see
  // clasi/issues/straight-leg-crab-118-001-actuation-and-telemetry-pairing-skew.md
  // and docs/code_review/2026-07-22-turn-execution-review.md §9): drive_.tick()
  // runs HERE, before EITHER motor's own select, restoring the ONE genuinely
  // good half of the retired 112-005 hoist (same-generation actuation
  // staging) WITHOUT reintroducing the select-ordering bug 118 actually
  // fixed (see the interleave paragraph just below -- moving this call does
  // not touch select ordering at all, the two are orthogonal). Pure
  // computation (no bus, no sleep), so it is legal here. Both
  // motorL_.tick() and motorR_.tick() below therefore apply the SAME
  // staged target this cycle -- symmetric, one cycle old relative to
  // processMessage()'s own command-apply call (R-settle block, below,
  // unchanged position) -- instead of 118's asymmetric split (R fresh this
  // cycle, L stale by a further cycle), which produced a real per-cycle
  // L/R actuation skew during every ramp (measured +2.685deg cruise yaw on
  // a straight leg, exact match to the predicted v_cruise*kCycle/trackWidth
  // transient).
  drive_.tick();  // twist -> wheel targets; L and R apply THIS cycle's stage symmetrically below

  // Request/collect MUST interleave per port (118 -- restores the
  // interleaved schedule this file's own DESIGN.md §2/§3 already claims:
  // select L -> settle(borrow) -> collect L -> clear(borrow) -> select R
  // -> settle(borrow) -> collect R -> pace): the 0x46 encoder-select is a
  // single latched state on the brick (one pending read; SimPlant models
  // the same via selectedPort_) -- issuing both selects before either
  // collect makes BOTH motors read the LAST-selected port's encoder
  // (observed 2026-07-18: an unmanaged pivot showed actual L == actual R
  // glued to the right wheel while cmd L/R were correctly mirrored).

  motorL_.requestSample();  // 0x46 write (brick holds ONE pending read)

  runAndWait(kSettle, [&] {           // >=4ms: L encoder settling, meanwhile --
    comms_.pump(cmd, cycleStart);     //   drain RX, decode <=1 frame into cmd
  });

  motorL_.tick(clock_.nowMicros());   // collect L -> velocity PID -> duty write

  runAndWait(kClear, [&] {  // >=4ms: brick clears L's duty write, meanwhile --
    // 119 ticket 005: intentionally empty now. updateTlm()/tlm_.emit()
    // moved to the start of the trailing pace block (below), AFTER
    // motorR_.tick()'s own collect, so every emitted frame pairs
    // SAME-GENERATION L/R encoder samples -- the co-fix for the actuation
    // skew above. Emitting HERE (between L's collect and R's collect, as
    // 118 shipped it) paired fresh-L-this-cycle against stale-R-last-cycle;
    // that pairing skew numerically CANCELED the physical skew Fix A
    // removes, which is exactly why the host-visible encoder trace
    // (dL-dR, encpose, frame.twist) reported a perfectly straight leg while
    // the robot's true path crabbed ~31mm over 700mm -- see the
    // straight-leg-crab issue's full derivation. This window still borrows
    // its mandatory kClear settle time (the vendor's post-duty-write
    // clearance); there is just no bus-free work left to do inside it.
  });

  motorR_.requestSample();  // 0x46 write (brick holds ONE pending read)

  runAndWait(kSettle, [&] {  // >=4ms: R encoder settling, meanwhile --
    // Apply <=1 decoded command; every path that applies one acks via
    // tlm_.ack(). `cmd` is a fresh, cycle-local variable (declared above,
    // populated by at most one comms_.pump() call this cycle), so reading
    // it here bounds dispatch to at most once per cycle by construction --
    // no separate "take" flag is needed.
    //
    // 118 ticket 002: moveQueue_.tick() -- the MOVE stop decision -- does
    // not run from this block. It moved to the trailing pace block, AFTER
    // odom_.integrate()/stateEstimator_.update(), so the decision reads
    // THIS cycle's odometry/estimator state instead of the previous
    // cycle's (see that block's own comment below and
    // clasi/issues/stop-decision-must-see-this-cycles-odometry.md). 119
    // ticket 005: drive_.tick() no longer runs from this block either --
    // it moved to the very top of cycle(), above, so both motor ticks
    // apply a same-generation target (see that call's own comment). This
    // R-settle block now holds only command dispatch, pure compute.
    processMessage(cmd);
  });

  motorR_.tick(clock_.nowMicros());   // collect R -> velocity PID -> duty write

  // Wheel-section publish (124-009, blackboard issue's own publish rule:
  // "Wheels publish immediately after BOTH collects ... never after just
  // L. Coherence for the wheel section means same-generation L/R samples,"
  // the 119-005 straight-leg-crab pairing-skew lesson) -- MOVED here from
  // the old assembleFrame() (which ran later, in the pace block, but read
  // the SAME already-fresh motorL_/motorR_ state; publishing right at the
  // point of coherence rather than deferring is the only actual change).
  //
  // Position-rebaseline policy (124-008, sprint 124 architecture Decision
  // 6, revised twice -- see sprint.md): EncoderReading.position accumulates
  // monotonically for the entire session, but the wire's own sint32 field
  // is bounded to (abs_max)=kPositionWireBound. Each cycle, after reading a
  // wheel's raw position, rebaseline it in SOFTWARE
  // (Devices::Motor::rebaseline() -- existing, unmodified, zero-bus-traffic
  // -- NEVER Motor::resetPosition()/MotorArmor's staged dispatch, which can
  // still choose a real bus-touching hard reset at standstill and is
  // forbidden outright by the stakeholder ruling for this policy) once it
  // is within kPositionRebaselineMargin of that bound, and increment this
  // wheel's own positionEpoch counter -- owned and incremented HERE, never
  // by the device layer. Defensive fallback only (the margin dwarfs one
  // cycle's worst-case travel at any realistic wheel speed, so this should
  // not be the expected path): if position is somehow still beyond the
  // bound despite the margin, clamp it rather than let the wire pack it
  // wrapped, and set an observable fault flag (state.health.positionClamped,
  // derived into kFlagFaultPositionClamped by Telemetry::update()).
  float posL = motorL_.position();
  if (std::fabs(posL) >= kPositionRebaselineMargin) {
    motorL_.rebaseline();
    posL = motorL_.position();
    positionEpochLeft_ = static_cast<uint8_t>(positionEpochLeft_ + 1);
  }
  bool clampedL = false;
  posL = clampToPositionWireBound(posL, &clampedL);

  float posR = motorR_.position();
  if (std::fabs(posR) >= kPositionRebaselineMargin) {
    motorR_.rebaseline();
    posR = motorR_.position();
    positionEpochRight_ = static_cast<uint8_t>(positionEpochRight_ + 1);
  }
  bool clampedR = false;
  posR = clampToPositionWireBound(posR, &clampedR);

  // sampleTime (124-009, issue §B2/SUC-006): Devices::Motor::sampleTime()
  // [us] -- the nowUs of the tick() call that produced the CURRENTLY
  // cached position()/velocity() reading (ticket 002's enabling accessor)
  // -- converted to the [ms] cycle-domain state.time.cycleStart shares.
  // Left and right genuinely differ (the brick holds one pending read, so
  // motorL_.tick() and motorR_.tick() above are collected roughly
  // kSettle+kClear apart): this is the real per-sample skew
  // Telemetry::update()'s age = now - sampleTime projection carries, in
  // place of ticket 008's honest-zero placeholder (both stamped with the
  // same cycleStart).
  state_.wheelLeft.position = posL;
  state_.wheelLeft.velocity = motorL_.velocity();
  state_.wheelLeft.sampleTime = static_cast<uint32_t>(motorL_.sampleTime() / 1000);  // [us] -> [ms]
  state_.wheelLeft.connected = motorL_.connected();
  state_.wheelLeft.positionEpoch = positionEpochLeft_;
  state_.wheelLeft.cmdVelocity = drive_.vLeft();

  state_.wheelRight.position = posR;
  state_.wheelRight.velocity = motorR_.velocity();
  state_.wheelRight.sampleTime = static_cast<uint32_t>(motorR_.sampleTime() / 1000);  // [us] -> [ms]
  state_.wheelRight.connected = motorR_.connected();
  state_.wheelRight.positionEpoch = positionEpochRight_;
  state_.wheelRight.cmdVelocity = drive_.vRight();

  state_.health.wedgeLatch = motorL_.wedged() || motorR_.wedged();
  state_.health.positionClamped = clampedL || clampedR;

  // Final (perception + odometry + StateEstimator + telemetry-update +
  // MoveQueue stop-decision + pace) block -- the schedule's 4th runAndWait,
  // matching the same "own mark, own gap" shape as the three
  // settle/clearance blocks above (see kPace's own comment for why the gap
  // must be derived, not a bare kCycle anchored to the cycle start).
  //
  // 124-009 (issue §B1, "RobotState is not the wire frame" -- Eric,
  // 2026-07-25: "what should be happening in the main loop is that we
  // create or update the state object, and then at the very end we send
  // telemetry by construct -- one line that updates the telemetry with the
  // state object"): this block reads every remaining primary source ONCE
  // -- odom_.integrate(), otos_.tick(), the alternating line_/color_ tick,
  // and BodyKinematics::forward() for the fused twist -- publishing each
  // into state_ at its own point of coherence (sensors -> odom -> estimate,
  // the blackboard issue's own dependency order), THEN calls
  // stateEstimator_.update(state_, now) directly (state_ IS
  // Motion::StateEstimator::Input, ticket 007 -- no more hand-copied
  // estimatorInput), THEN tlm_.update(state_)/tlm_.emit(now) -- the one
  // assembly point, replacing the old ten-argument assembleFrame() and its
  // ten scattered flag-mutation calls (SUC-004).
  //
  // MoveQueue's own stop-decision tick() still runs AFTER
  // tlm_.update()/emit() (118 ticket 002 -- SUC-063: the decision must see
  // THIS cycle's odom_.integrate(), which it does, since integrate() runs
  // earlier in this SAME block -- and protocol-v4 §7.2: a completion ack
  // staged by tick() must not be visible before the NEXT cycle's own
  // emit() call). Outside any motor request/collect window (this class's
  // own bus-discipline responsibility) -- this block DOES touch the bus
  // (OTOS, and at most one of line/color), unlike the two settle blocks
  // and kClear.
  runAndWait(kPace, [&] {
    uint64_t nowUs = clock_.nowMicros();

    // odom_.integrate() runs FIRST (120-002): a FakeOtos (main.cpp's
    // FAKE_OTOS build variant) reports THIS cycle's just-integrated
    // Odometry pose, so it must be integrated before the OTOS tick below.
    // Side-effect-free for the real leaf too. 122-002: Motion::Odometry no
    // longer holds a Devices::Motor& (src/motion may not depend on
    // devices/) -- this class reads both leaves' CURRENT position() here
    // and hands the two floats in.
    odom_.integrate(motorL_.position(), motorR_.position());  // odometry from both fresh wheel samples

    // Perception step -- sample the OTOS. UNIFORM across both builds
    // (otos-fake-seam issue): otos_ is a Devices::Otos&, backed by either
    // the real SparkFun leaf (a rate-limited I2C burst read) or
    // App::FakeOtos (synthetic pose from odom_ + wheel twist) -- chosen
    // once at construction (main.cpp), never branched on here. Published
    // into state_.otos directly, once -- StateEstimator::update() and
    // Telemetry::update() both read it from there, never re-derived.
    otos_.tick(nowUs);
    const bool otosPresent = otos_.present() && otos_.poseFresh();
    state_.otos.present = otosPresent;
    state_.otos.connected = otos_.connected();
    if (otosPresent) {
      const Devices::PoseReading otosReading = otos_.pose();
      state_.otos.x = otosReading.x;
      state_.otos.y = otosReading.y;
      state_.otos.heading = otosReading.heading;
      state_.otos.v_x = otosReading.v_x;
      state_.otos.v_y = otosReading.v_y;
      state_.otos.omega = otosReading.omega;
      // sampleTime (issue §B2/SUC-006): Devices::Otos::sampleTime() [us],
      // converted to [ms] -- same rationale as the wheel section above.
      state_.otos.sampleTime = static_cast<uint32_t>(otos_.sampleTime() / 1000);  // [us] -> [ms]
    }

    // Line/color -- rate-limited, ALTERNATING steady-state sampling
    // (115-005, gut S1's own wiring). Ticks EXACTLY ONE of {line_, color_}
    // this call (never both -- the 098-004 per-pass-read regression
    // precedent: never let a per-cycle sensor read disrupt the motor
    // request/collect cadence) and alternates which one on the NEXT call.
    // Each leaf's own tick()/readDue() rate-limits the actual bus
    // transaction further (the same Otos::readDue() pattern). The OTHER
    // leaf's own fresh flag is simply never set true this cycle (it was
    // not even touched) -- matching the wire spec's "line/color word
    // fresh" (i.e. fresh THIS frame, not merely "known at some point")
    // semantics. packLine()/packColor() (this file's own anonymous
    // namespace) stay HERE, not in Telemetry::update() -- they need
    // Devices::LineReading/ColorReading, and Types::RobotState may only
    // ever hold cstdint-level data (robot_state.h's own dependency-free
    // contract, which src/motion also stands on) -- RobotLoop is still
    // the only class with access to line_/color_ this sprint (Decision 1),
    // so it is also the only class that can call these.
    bool lineFresh = false;
    bool colorFresh = false;
    if (lineTurnNext_) {
      line_.tick(nowUs);
      lineFresh = line_.readingFresh();
    } else {
      color_.tick(nowUs);
      colorFresh = color_.readingFresh();
    }
    lineTurnNext_ = !lineTurnNext_;

    state_.perception.lineFresh = lineFresh;
    state_.perception.colorFresh = colorFresh;
    if (lineFresh) state_.perception.line = packLine(line_.reading());
    if (colorFresh) state_.perception.color = packColor(color_.reading());

    // Fused body-frame velocity (109-009 fix, carried forward): the two
    // leaves' current velocities through BodyKinematics::forward() yield
    // the fused body (v, omega) for THIS instant, the same equations
    // Odometry uses for per-cycle distance/headingDelta.
    float twistVx = 0.0f;
    float twistOmega = 0.0f;
    BodyKinematics::forward(motorL_.velocity(), motorR_.velocity(), drive_.trackWidth(),
                             twistVx, twistOmega);

    // Pose section (encoder-only dead-reckoned pose + same-cycle fused
    // body-frame twist -- robot_state.h's own Pose doc comment). Published
    // once here; StateEstimator::update() and Telemetry::update() both
    // read it from state_, never re-derived.
    state_.pose.x = odom_.x();
    state_.pose.y = odom_.y();
    state_.pose.heading = odom_.theta();
    state_.pose.v_x = twistVx;
    state_.pose.v_y = 0.0f;
    state_.pose.omega = twistOmega;

    // Command section -- mode/moveActive, read fresh here (matching where
    // the pre-124-009 assembleFrame() read moveQueue_.active(), before
    // this cycle's own moveQueue_.tick() below runs). v_x/omega stay
    // unpopulated -- see robot_state.h's own Command doc comment for why.
    state_.command.mode = moveQueue_.active() ? Types::Mode::Velocity : Types::Mode::Idle;
    state_.command.moveActive = moveQueue_.active();

    // Health section -- the two live counters/latches knowable at this
    // point in the cycle (moveTimeout/shapingDisabled are refreshed AFTER
    // moveQueue_.tick(), below -- their data does not exist yet here).
    state_.health.i2cSafetyNetCount = bus_.clearanceSafetyNetCount();
    state_.health.commsMalformedCount = comms_.malformedCount();

    // Predict-to-now estimation (117 ticket 004): refreshes StateEstimator's
    // wheel/body peer bases immediately after odom_.integrate()/the OTOS
    // tick above, matching this sprint's overlay DESIGN.md §2 exactly. Pure
    // computation -- no I2C access, no sleep, bounded work, the same
    // posture odom_.integrate()/the OTOS tick already keep in this block.
    // 124-009: state_ IS Motion::StateEstimator::Input (ticket 007's
    // alias) -- passed DIRECTLY, no more hand-copied estimatorInput local.
    stateEstimator_.update(state_, static_cast<uint32_t>(nowUs / 1000));  // [us] -> [ms]

    // Time section -- cycleBusy/cyclePeriod bookkeeping (123-004, MOVED
    // here from the old assembleFrame()): cycleBusy is measured as late as
    // possible in the cycle (this instant), capturing the OTOS/line-color/
    // estimator compute time above; cyclePeriod is this cycle's own
    // cycleStart minus the previous cycle() call's cycleStart, tracked via
    // previousCycleStartUs_/everCycled_ (unchanged mechanism from 122-003).
    state_.time.cycleBusy = static_cast<uint32_t>(clock_.nowMicros() - cycleStartUs);  // [us]
    state_.time.cyclePeriod =
        everCycled_ ? static_cast<uint32_t>(cycleStartUs - previousCycleStartUs_) : 0u;  // [us]
    previousCycleStartUs_ = cycleStartUs;
    everCycled_ = true;

    // The one-line projection (124-009, issue §B1) -- state_ is complete
    // for this cycle (every section above published); update() derives
    // the WHOLE next frame and every flag it can know from state_ alone,
    // then emit() sends it if due. Exactly one tlm_.update() call in
    // cycle(), no direct flag-mutation calls anywhere in this file
    // (SUC-004, grep-enforceable).
    tlm_.update(state_);
    tlm_.emit(cycleStart);

    // MoveQueue's per-cycle tick (116, protocol-set-point issue; 118
    // ticket 002 relocates it HERE -- after odom_.integrate()/
    // stateEstimator_.update() above -- so the stop decision reads
    // odometry/estimator state staged THIS cycle, not the previous one).
    // It reuses `nowUs`, this block's own already-captured "current
    // reading" (mirrors move_queue.h's own "never re-read a current value
    // mid-tick" convention), rather than issuing a second
    // clock_.nowMicros() call. Replaces the deleted deadman_.expired()
    // branch's schedule role: this is the load-bearing safety property
    // (SUC-053) -- it runs unconditionally, every cycle, regardless of
    // whether a command arrived this cycle -- the same way
    // deadman_.expired() did. Ends the active Move on StopConditionMet or
    // TimedOut, either chain-advancing the next pending Move THIS SAME
    // cycle (seamless hand-off, SUC-051) or calling Drive::stop() with an
    // empty queue (MoveQueue::tick()'s own contract) -- so host silence
    // always ends in motors stopped, with zero further host traffic
    // needed (no deadman lease to re-arm). The staged stop/twist reaches
    // motor duty at the NEXT cycle's own drive_.tick() call (119 ticket
    // 005: the very top of cycle(), above) -- one cycle of
    // decision-to-duty latency, unchanged in shape from before this
    // ticket.
    //
    // This call MUST stay positioned AFTER tlm_.update()/tlm_.emit() every
    // cycle (unchanged from before 124-009): a completion ack staged here
    // is still not visible until the NEXT cycle's own emit() call -- "ack
    // rides the next frame," protocol-v4 §7.2, unaffected by this ticket.
    Motion::MoveQueue::TickResult moveResult = moveQueue_.tick(nowUs, odom_);

    // kFlagFaultMoveTimeout/kFlagFaultShapingDisabled -- refreshed HERE,
    // directly, immediately after tick() (see Telemetry::setLiveFlag()'s
    // own doc comment for why these two can't join tlm_.update() above):
    // both are queried by app_robot_loop_harness.cpp's SUC-054/119-001
    // scenarios as LIVE tlm_.flags() state, not decoded wire content, and
    // both require THIS cycle's own tick() outcome to already be visible
    // by the time cycle() returns -- exactly the pre-124-009 behavior,
    // unchanged in position or logic. Published into state_.health too
    // (blackboard completeness -- ticket 009's own "Health... populating
    // those sections is your job"), a level-set every cycle: true only on
    // the exact cycle a timed-out completion/all-axes-disabled MOVE is
    // reported this call, false every other cycle (SUC-054).
    state_.health.moveTimeout = moveResult.completed && moveResult.completion.timedOut;
    // Loud off-state (119 ticket 001,
    // kill-the-silent-off-shaping-config-boundary.md): set whenever a MOVE
    // is active with BOTH angular and linear ShaperLimits disabled --
    // reads moveQueue_'s OWN current state (post-tick(), so a Move that
    // just ended/started THIS cycle is already reflected), not moveResult.
    state_.health.shapingDisabled = moveQueue_.active() && moveQueue_.shapingDisabled();

    tlm_.setLiveFlag(kFlagFaultMoveTimeout, state_.health.moveTimeout);
    tlm_.setLiveFlag(kFlagFaultShapingDisabled, state_.health.shapingDisabled);

    if (moveResult.completed) {
      // MOVE completion ack (protocol-set-point issue, Responses section):
      // a SECOND ack on the cycle the command ends -- ack_corr ==
      // Move.id, ack_err == 0 regardless of outcome; a timeout ending is
      // distinguished by kFlagFaultMoveTimeout just above, not by ack_err.
      tlm_.ack(moveResult.completion.moveId, 0);
    }
  });
}

}  // namespace App
