// sim_api.cpp -- TestSim::SimApi implementation. See sim_api.h's file
// header for the class's boundary, file-placement rationale, and plant/PID
// tuning rationale.
#include "sim_api.h"

#include <cmath>

#include "kinematics/body_kinematics.h"

namespace TestSim {

namespace {

constexpr uint16_t kMotorWireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);
constexpr uint16_t kOtosWireAddr = static_cast<uint16_t>(Devices::kOtosDeviceAddr << 1);

constexpr float kTrackWidth = 130.0f;  // [mm] -- matches plant_harness.cpp's own kTrackWidth

// See sim_api.h's file header "Plant/PID tuning" section for the full
// derivation of every field set here.
Devices::MotorConfig makeMotorConfig(uint32_t port) {
  Devices::MotorConfig cfg;
  cfg.port = port;
  cfg.fwdSign = 1;
  cfg.wheelTravelCalib = 1.0f;
  cfg.velFiltAlpha = 1.0f;   // no smoothing -- velocity() reflects the plant's own raw sample exactly
  cfg.slewRate = 100.0f;     // wide enough that a saturated PID output reaches +-100% in ONE write
  cfg.velGains.kp = 0.01f;   // large enough that any twist this harness injects saturates immediately
  return cfg;                // (ki/kff/iMax/kaw/velDeadband all stay 0 -- pure-P is all this harness needs)
}

// begin()'s full successful-detect transaction counts -- see
// app_robot_loop_harness.cpp / plant_harness.cpp's own identically-named
// helpers for the byte-for-byte derivation this duplicates (this
// codebase's established per-file fixture-duplication convention).
void scriptMotorBeginSuccess(Devices::I2CBus& bus) {
  for (int i = 0; i < 4; ++i) {
    bus.scriptWrite(kMotorWireAddr, /*status=*/0);
    uint8_t data[4] = {0, 0, 0, 0};
    bus.scriptRead(kMotorWireAddr, data, 4, /*status=*/0);
  }
}

void scriptOtosBeginSuccess(Devices::I2CBus& bus) {
  for (int i = 0; i < 7; ++i) bus.scriptWrite(kOtosWireAddr, /*status=*/0);
  uint8_t id[1] = {0x5F};  // Devices::Otos::kExpectedProductId
  bus.scriptRead(kOtosWireAddr, id, 1, /*status=*/0);
}

}  // namespace

SimApi::SimApi()
    : motorL_(bus_, makeMotorConfig(1)),
      motorR_(bus_, makeMotorConfig(2)),
      otos_(bus_, Devices::OtosConfig{}),
      color_(bus_, Devices::ColorConfig{}),
      line_(bus_, Devices::LineConfig{}),
      comms_(serialLink_, radioLink_, "DEVICE:NEZHA2:sim:sim_api:1"),
      tlm_(comms_, serialLink_, radioLink_),
      deadman_(clock_),
      drive_(motorL_, motorR_, kTrackWidth),
      odom_(motorL_, motorR_, kTrackWidth),
      preamble_(motorL_, motorR_, otos_, color_, line_, clock_),
      robotLoop_(bus_, motorL_, motorR_, otos_, comms_, tlm_, drive_, odom_, deadman_, preamble_, clock_, sleeper_),
      plantLeft_(TestSim::kDefaultDutyVelMax, TestSim::kDefaultTau),
      plantRight_(TestSim::kDefaultDutyVelMax, TestSim::kDefaultTau),
      otosPlant_(kTrackWidth) {
  Devices::I2CBus::setClock(1000000);
  // "Pre-Preamble state" (sim_api.h's own step() doc comment): everything
  // above is constructed and wired, but App::Preamble::step() has not yet
  // been called even once -- driveBootToDone()/robotLoop_.boot() are step()'s
  // job (the first call), not the constructor's.
}

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

// Drives App::Preamble to done() via preamble_.step() calls we issue
// OURSELVES, advancing the fake Clock between each one -- NOT via
// robotLoop_.boot(), whose own while(!preamble_.done()) loop is one
// synchronous C++ call with no opportunity for a caller to advance the
// Clock in between. That distinction matters here specifically because
// this harness scripts NO bus response at all for color_/line_ (105-004's
// own "scripted present()==false" requirement -- there is nothing for a
// plant response to feed, since telemetry carries no line=/color= fields
// yet) -- Devices::ColorSensorLeaf::beginStep()/Devices::LineSensorLeaf::
// beginStep() only re-attempt their own probe once
// `nowUs - lastAttemptUs_ >= kAltRetryPeriod/kRetryPeriod` (50ms) has
// REALLY elapsed on the Clock they're handed; at a frozen Clock (the only
// kind a single robotLoop_.boot() call could ever offer them) every call
// after their own very first attempt is an immediate, permanent no-op, so
// App::Preamble::done() would never become true and boot() would spin
// forever. Left/Right/Otos, by contrast, each resolve on their OWN very
// first attempt (scripted success below) regardless of clock pacing.
//
// Left(1 write+1 read)*4, Right same, Otos(7 writes+1 read) are scripted
// BEFORE this loop starts and are fully consumed by their own first-ever
// probe (each one a 1-shot resolution, matching app_robot_loop_harness.cpp's
// own boot derivation) -- by the time Color's/Line's own round-robin turns
// come up the scripted I2CBus queues are already empty, so their own
// beginStep() bus calls harmlessly return kScriptMismatch (i2c_bus_host.cpp)
// every time, never popping an entry meant for another device.
void SimApi::driveBootToDone() {
  clock_.setMicros(0);
  preamble_.step();  // arms Preamble's own startUs_ at 0 -- power-settle no-op (see
                      // app_robot_loop_harness.cpp's identical two-phase priming)

  clock_.setMicros(50000);  // >= Preamble::kPowerSettle -- probing starts on the NEXT step()
  scriptMotorBeginSuccess(bus_);  // Left
  scriptMotorBeginSuccess(bus_);  // Right
  scriptOtosBeginSuccess(bus_);

  // Left/Right/Otos each resolve within the first few passes; Color/Line
  // each need up to kMaxAltAttempts(20)/kMaxAttempts(20) failed attempts,
  // 50ms apart, before their own retry budget exhausts and they latch
  // present()==false -- advancing 50ms every pass (>= both leaves' own
  // retry period) guarantees every attempt that comes due is due. 200
  // passes is a generous bound over the ~44 actually needed (1 + 1 + 1 +
  // 21 + 20, interleaved) -- if this is ever exceeded, done() staying
  // false is a real bug, not a slow-but-fine boot, so the while loop
  // below is intentionally left able to actually hang a test run rather
  // than silently forcing a false "success."
  for (int i = 0; i < 200 && !preamble_.done(); ++i) {
    preamble_.step();
    clock_.advanceMicros(50000);
  }
}

void SimApi::step(int cycles) {
  if (!booted_) {
    driveBootToDone();
    robotLoop_.boot();  // preamble_.done() is already true -- the while loop body
                         // never executes; this still exercises the REAL boot()
                         // method, including its setEvent(kEventBootReady, true) tail.
    booted_ = true;
    return;  // boot is atomic -- consumes this whole step() call regardless of `cycles`
  }

  for (int i = 0; i < cycles; ++i) {
    scriptCycleBusResponses();
    clock_.advanceMicros(kCycleDtUs);  // BEFORE cycle() -- so THIS cycle's own
                                        // clock_.nowMicros() reads (motor tick() dt,
                                        // Otos::readDue(), Deadman::expired()) see the
                                        // advanced time; RobotLoop::cycle()'s own
                                        // internal mark/elapsed pairs are unaffected --
                                        // see sim_api.h's kCycleDtUs comment.
    robotLoop_.cycle();
    ++cycleCount_;
  }
}

// ---------------------------------------------------------------------------
// Per-cycle plant + bus scripting
// ---------------------------------------------------------------------------

// DutyPredictor::tickPredict() -- see sim_api.h's own class comment for the
// full derivation/scope. Mirrors nezha_motor.cpp's tick() steps 2-4 (the
// pieces that decide whether writeRawDuty() actually reaches the bus) under
// this harness's own fixed gains.
bool SimApi::DutyPredictor::tickPredict(float position, int cycle) {
  // Step 2 (nezha_motor.cpp tick()) analogue: freshness-gated velocity
  // update. Runs UNCONDITIONALLY every tick() call -- steps 1-3 of the real
  // tick() execute regardless of mode_ (only step 4's SWITCH dispatch is
  // mode-gated); motorL_.tick()/motorR_.tick() are themselves called every
  // single robotLoop_.cycle() from cycle 0 onward for BOTH leaves,
  // independent of either leaf's own activationCycle_ below. Gating this
  // block on `active_` (as an earlier version of this predictor did) is a
  // bug: it would skip L's real encoder anchor at cycle 0 (mode_ is still
  // None then, but collectEncoder() still ran and still anchored
  // lastFreshRawEnc_/lastFreshUs_), pushing the predictor's own anchor one
  // cycle late and desyncing every freshElapsed computed after L's first
  // real velocity change -- caught empirically via a throwaway stderr trace
  // comparing this predictor's per-cycle state against decoded telemetry
  // during this ticket's own verification.
  //
  // raw uses the SAME tenths-of-mm quantization
  // WheelPlant::scriptEncoderResponse() encodes (wheelTravelCalib=1.0,
  // fwdSign=+1, this harness's own fixed convention -- see sim_api.h's
  // "Plant/PID tuning" section).
  int32_t raw = static_cast<int32_t>(std::lround(position * 10.0f));
  bool freshSample = !hasFreshSample_ || (raw != lastFreshRaw_);
  if (freshSample) {
    if (hasFreshSample_) {
      // elapsed-since-last-fresh-sample, derived from cycle COUNT rather
      // than a us clock read -- kCycleDtUs is a fixed per-cycle constant
      // (see sim_api.h's own comment), so (cycle - lastFreshCycle_) *
      // kCycleDtUs is exactly what NezhaMotor's own
      // (nowUs - lastFreshUs_) would read, with no clock-seam coupling.
      float freshElapsed = static_cast<float>(cycle - lastFreshCycle_) * (static_cast<float>(kCycleDtUs) / 1e6f);
      if (freshElapsed > 0.0f) {
        float lastPos = static_cast<float>(lastFreshRaw_) / 10.0f;
        float pos = static_cast<float>(raw) / 10.0f;
        filteredVelocity_ = (pos - lastPos) / freshElapsed;  // velFiltAlpha == 1.0 -- direct assignment, no EMA blend
      }
      // else: non-positive elapsed (defensive only, mirrors nezha_motor.cpp) -- hold filteredVelocity_.
    }
    // else: first-ever fresh sample -- anchor only, no velocity computed (matches tick()'s boot-anchor branch).
    hasFreshSample_ = true;
    lastFreshRaw_ = raw;
    lastFreshCycle_ = cycle;
  }
  // else: repeated raw (encoder hasn't advanced enough to change the tenths-of-mm quantum this cycle) --
  // hold filteredVelocity_ unchanged, matching tick()'s own "brick hasn't refreshed" branch.

  // Step 4 dispatch analogue -- Mode::None (no PID, no write, no-op) until
  // this leaf's own structural activation cycle; matches tick()'s own
  // switch(mode_) None/default case.
  if (!active_) {
    if (cycle < activationCycle_) return false;
    active_ = true;  // mode transitions to Active THIS cycle
  }

  // Pure-P PID (this harness's own fixed gains -- see sim_api.h's
  // DutyPredictor comment for why ki/kff/iMax/kaw's residual effect is
  // skipped), clamped to the PID's own output domain.
  constexpr float kPidKp = 0.01f;  // mirrors makeMotorConfig()'s own kp below
  float requestedDuty = kPidKp * (target_ - filteredVelocity_);
  requestedDuty = (requestedDuty < -1.0f) ? -1.0f : (requestedDuty > 1.0f ? 1.0f : requestedDuty);
  // averageDuty() (nezha_motor.cpp, sprint 101 boxcar output smoothing) is a
  // deliberately omitted no-op here: dutyAvgWindow_ defaults to 1 (off,
  // unchanged behavior, nezha_motor.h), and nothing in this harness ever
  // calls setDutyAvg() -- so requestedDuty above IS averageDuty(requestedDuty).

  // MotorArmor::armoredWrite() gate (motor_armor.h) -- reversal-dwell +
  // output-deadband. Sits BETWEEN the PID's raw output and writeRawDuty()'s
  // own write-on-change/slew gate below; MISSING this layer was this
  // predictor's first (wrong) hypothesis during verification -- a duty
  // sign flip (e.g. the STOP-after-forward-ramp transition) does NOT reach
  // writeRawDuty() as-is, it gets FORCED TO ZERO for a dwell window first.
  // This harness's own MotorConfig (makeMotorConfig()) leaves both armor
  // fields (reversalDwell/outputDeadband) unset, so configureArmor()
  // substitutes MotorArmor's own ship defaults exactly (motor_armor.h):
  // kDefaultOutputDeadband = 0.03, kDefaultReversalDwell = 100ms. nowMs is
  // derived from cycle count the same way freshElapsed is above (kCycleDtUs
  // is a fixed 50ms/cycle constant, so cycle*50 is exactly what
  // NezhaMotor's own nowMs would read at this tick, up to a constant
  // boot-time offset that never matters -- only RELATIVE now-vs-deadline
  // comparisons drive this gate).
  constexpr float kOutputDeadband = 0.03f;
  constexpr float kReversalDwellMs = 100.0f;
  uint32_t nowMs = static_cast<uint32_t>(cycle) * (kCycleDtUs / 1000);

  float dutyToWrite;
  if (requestedDuty == 0.0f || std::fabs(requestedDuty) < kOutputDeadband) {
    // Stop always wins: immediate, unclamped, cancels any dwell in flight.
    dwelling_ = false;
    lastRequestedDuty_ = 0.0f;
    dutyToWrite = 0.0f;
  } else {
    bool forwardAsIs = true;
    if (dwelling_) {
      if (nowMs < dwellDeadline_) {
        // Still holding at commanded-zero through the dwell window.
        lastRequestedDuty_ = 0.0f;
        dutyToWrite = 0.0f;
        forwardAsIs = false;
      } else {
        dwelling_ = false;  // dwell elapsed -- proceed in the new direction below
      }
    } else if (kReversalDwellMs > 0.0f && lastRequestedDuty_ != 0.0f &&
               ((requestedDuty > 0.0f) != (lastRequestedDuty_ > 0.0f))) {
      // Commanded sign change relative to the last duty actually forwarded --
      // write 0 now and arm the dwell; the new direction is withheld until
      // the dwell deadline.
      dwelling_ = true;
      dwellDeadline_ = nowMs + static_cast<uint32_t>(kReversalDwellMs);
      lastRequestedDuty_ = 0.0f;
      dutyToWrite = 0.0f;
      forwardAsIs = false;
    }
    if (forwardAsIs) {
      // Same-sign duty (or no prior direction to reverse from, or the dwell
      // just elapsed): forward as-is.
      lastRequestedDuty_ = requestedDuty;
      dutyToWrite = requestedDuty;
    }
  }

  // writeRawDuty() analogue (nezha_motor.cpp) -- quantize -> write-on-change -> slew -> stop exemption.
  int8_t pct = static_cast<int8_t>(std::lround(dutyToWrite * 100.0f));
  if (pct > 100) pct = 100;
  if (pct < -100) pct = -100;

  if (pct == lastWrittenPct_) return false;  // write-on-change gate

  bool stopping = (pct == 0);
  bool firstWrite = (lastWrittenPct_ == -128);
  int8_t written = pct;
  if (!stopping && !firstWrite) {
    constexpr uint8_t kSlewRate = 100;  // mirrors makeMotorConfig()'s own slewRate below
    int16_t delta = static_cast<int16_t>(pct) - static_cast<int16_t>(lastWrittenPct_);
    if (delta > static_cast<int16_t>(kSlewRate)) {
      written = static_cast<int8_t>(lastWrittenPct_ + kSlewRate);
    } else if (delta < -static_cast<int16_t>(kSlewRate)) {
      written = static_cast<int8_t>(lastWrittenPct_ - kSlewRate);
    }
  }
  lastWrittenPct_ = written;
  return true;
}

// Pushes exactly the I2CBus writes/reads THIS upcoming robotLoop_.cycle()
// call will issue, in the SAME chronological order (source/app/robot_loop.cpp
// cycle()'s own call sequence: L request -> L collect(+maybe duty) -> R
// request -> R collect(+maybe duty) -> OTOS burst(always due at this
// harness's kCycleDtUs >= Otos::kReadPeriod)) -- writes and reads are TWO
// SEPARATE shared FIFOs (i2c_bus_host.cpp), so what matters is each
// device's own writes staying in call order relative to every OTHER
// device's writes (ditto reads), which calling
// plantLeft->plantRight->otosPlant in this fixed order, once per cycle,
// already guarantees (TestSim::WheelPlant::scriptEncoderResponse()'s own
// "writes first, then the read" push order is exactly this call's own
// request-write/collect-read/[duty-write] shape once writeCount is right).
//
// Whether a SECOND ("extra") write lands on this cycle -- vs. the
// steady-state 1 (the request-write only) -- is now predicted dynamically,
// per leaf, per cycle, by DutyPredictor (106-003, SUC-026), superseding the
// single hand-derived pendingEventCycle_ index ticket 105-004 originally
// used. Two staging inputs feed the predictors:
//   1. Each leaf's OWN structural mode-activation cycle (R: cycle 0, L:
//      cycle 1) -- App::Drive::tick() runs BETWEEN motorL_.tick() and
//      motorR_.tick() within ONE cycle() call (robot_loop.cpp's own
//      cycle() body), so R's mode_ is already Active (Drive::tick() ran
//      moments earlier, same cycle 0) by the time motorR_.tick() runs on
//      cycle 0; L's motorL_.tick() runs BEFORE drive_.tick() has EVER
//      executed (cycle 0), so L's own first write is deferred to cycle 1.
//      DutyPredictor's own activationCycle_ constructor argument encodes
//      this directly (byte-for-byte the same derivation
//      app_robot_loop_harness.cpp's own scriptMotorCycle() comment
//      documents for the identical RobotLoop+Drive composition).
//   2. pendingCycle_/pendingVL_/pendingVR_ (staged by
//      stageActuationChange(), called from injectTwist()/injectStop()/
//      notePendingActuationChange()): a fresh command is DISPATCHED (the
//      switch in cycle()'s third runAndWait block) BEFORE that same
//      block's own drive_.tick() call, so R's very next tick() (later in
//      that SAME cycle) sees the new target -- R's predictor target
//      updates on pendingCycle_ itself; L's own tick() for that cycle
//      already ran EARLIER (before dispatch), so L's predictor target
//      does not update until pendingCycle_ + 1.
// From there, DutyPredictor::tickPredict() runs the ACTUAL write-on-change
// decision every cycle, for as many cycles/transitions as a scenario steps
// through -- not just the first one.
void SimApi::scriptCycleBusResponses() {
  if (pendingCycle_ >= 0) {
    if (cycleCount_ == pendingCycle_) predictorRight_.setTarget(pendingVR_);
    if (cycleCount_ == pendingCycle_ + 1) predictorLeft_.setTarget(pendingVL_);
  }

  plantLeft_.step(motorL_.appliedDuty(), static_cast<float>(kCycleDtUs) / 1e6f);
  plantRight_.step(motorR_.appliedDuty(), static_cast<float>(kCycleDtUs) / 1e6f);
  otosPlant_.step(plantLeft_.position(), plantRight_.position());

  bool extraL = predictorLeft_.tickPredict(plantLeft_.position(), cycleCount_);
  bool extraR = predictorRight_.tickPredict(plantRight_.position(), cycleCount_);

  plantLeft_.scriptEncoderResponse(bus_, kMotorWireAddr, extraL ? 2 : 1);
  plantRight_.scriptEncoderResponse(bus_, kMotorWireAddr, extraR ? 2 : 1);
  otosPlant_.scriptPoseResponse(bus_, kOtosWireAddr);  // always due -- kCycleDtUs (50ms) >= Otos::kReadPeriod (20ms)
}

// ---------------------------------------------------------------------------
// Command injection
// ---------------------------------------------------------------------------

void SimApi::injectCommand(const char* armoredLine) { serialLink_.enqueueInbound(armoredLine); }

void SimApi::stageActuationChange(int atCycle, float vL, float vR) {
  pendingCycle_ = atCycle;
  pendingVL_ = vL;
  pendingVR_ = vR;
}

void SimApi::notePendingActuationChange(int atCycle) {
  stageActuationChange(atCycle, /*vL=*/0.0f, /*vR=*/0.0f);  // every caller today is an autonomous
                                                              // App::Drive::stop() (deadman expiry) --
                                                              // see sim_api.h's own comment
}

void SimApi::injectTwist(float v_x, float omega, float duration, uint32_t corrId) {
  injectCommand(TestSupport::armorTwistCommand(v_x, omega, duration, corrId).c_str());
  // Same wheel-target math App::Drive::tick() runs (BodyKinematics::inverse()) -- the predictors need the
  // REAL staged target, not just "something changed", to predict decay-window writes (106-003).
  float vL = 0.0f, vR = 0.0f;
  BodyKinematics::inverse(v_x, omega, kTrackWidth, vL, vR);
  stageActuationChange(cycleCount_, vL, vR);  // consumed on the NEXT step()'s first cycle
}

void SimApi::injectStop(uint32_t corrId) {
  injectCommand(TestSupport::armorStopCommand(corrId).c_str());
  stageActuationChange(cycleCount_, /*vL=*/0.0f, /*vR=*/0.0f);  // App::Drive::stop() always stages (0, 0)
}

// ---------------------------------------------------------------------------
// Telemetry drain
// ---------------------------------------------------------------------------

std::vector<TestSupport::DecodedLine> SimApi::drainTelemetry() {
  std::vector<TestSupport::DecodedLine> result;
  const auto& sent = serialLink_.sent();
  for (; telemetryDrainIndex_ < sent.size(); ++telemetryDrainIndex_) {
    result.push_back(TestSupport::decodeOutboundLine(sent[telemetryDrainIndex_]));
  }
  return result;
}

// ---------------------------------------------------------------------------
// Timing diagnostic
// ---------------------------------------------------------------------------

// Devices::Sleeper (clock.h) never advances the paired Devices::Clock on a
// sleepMillis() call ("there is no implicit link between a requested sleep
// duration and how far the fake Clock moves; the harness decides" --
// clock_host.cpp's own comment) -- and this harness's own step() never
// advances the Clock DURING a cycle() call (only ever before one, at the
// top of the loop in step()). So EVERY runAndWait/sleepUntil call inside
// THIS single cycle() call sees elapsed-since-mark == 0, meaning each of
// the four sleeps robot_loop.cpp's cycle() body issues (L-settle,
// clearance, R-settle, final perception+odometry+pace) requests exactly its
// OWN gap parameter, no more, no less -- an invariant provable from
// robot_loop.cpp's own runAndWait()/sleepUntil() bodies, not merely
// observed here. That invariant is what lets a HOST_BUILD harness report a
// deterministic virtual total at all (a real ARM cycle's sleeps interact
// with genuine elapsed wall time and do NOT sum this way -- see the
// comparison this method's own caller records in ticket 106-001's
// completion notes).
//
// virtualCycleMillis is therefore the SUM of the four sleeps robot_loop.cpp's
// own published constants declare (kSettle=4, kClear=4, kSettle=4,
// kPace=28 -- robot_loop.cpp's own anonymous-namespace constants, not
// exported, duplicated here by citation per this codebase's established
// per-file fixture-duplication convention) -- 4+4+4+28 = 40ms == kCycle.
// This equality (not an inequality, not a coincidence) is 106-001's own
// fix: robot_loop.cpp's kPace is DERIVED as kCycle minus the three
// settle/clear windows specifically so this sum lands on kCycle exactly,
// closing the gap 105-004 found (the pre-106-001 code passed kCycle, not
// kPace, to the final block, so this same sum was 4+4+4+16=28ms against a
// 16ms kCycle target -- 12ms unabsorbed). sleepCount and lastSleepMillis
// below are the OBSERVED corroboration (not merely hardcoded trust):
// sleepCount must be exactly 4 (three runAndWait blocks plus the final
// perception+odometry+pace block), and lastSleepMillis must equal the
// final block's own kPace=28ms -- both checked live, not assumed.
CycleTimingReport SimApi::measureOneCycle() {
  CycleTimingReport report;
  int sleepsBefore = sleeper_.sleepCount();
  int yieldsBefore = sleeper_.yieldCount();

  step(1);

  report.sleepCount = sleeper_.sleepCount() - sleepsBefore;
  report.lastSleepMillis = sleeper_.lastSleepMillis();
  report.yieldCount = sleeper_.yieldCount() - yieldsBefore;

  // 3 non-final blocks (L-settle, clearance, R-settle) x kSettle/kClear (4ms
  // each, robot_loop.cpp) + the final, OBSERVED cycle-pace block.
  constexpr uint32_t kNonFinalBlockMillis = 4;
  report.virtualCycleMillis = 3 * kNonFinalBlockMillis + report.lastSleepMillis;

  return report;
}

}  // namespace TestSim
