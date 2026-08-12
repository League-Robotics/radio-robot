// app_robot_loop_pacing_harness.cpp -- host-build unit test for ticket
// 131-005 (SUC-131-005): proves Core::RobotLoop::cycle()'s trailing pacing
// block, now targeting an ABSOLUTE end-of-cycle deadline
// (state_.time.cycleStart + kCycle) rather than a gap relative to its own
// entry mark, converges the MEAN measured inter-cycle-start period
// (state_.time.cyclePeriod -- the same observable robot_loop.cpp's own
// publishTiming() derives and telemetry reports every frame) to kCycle
// even under injected per-block overrun and whole-millisecond sleep
// rounding, rather than drifting to kCycle plus a fixed structural offset.
//
// This is the exact defect 130-011 measured on real hardware: a
// rock-stable 54.000ms +/- 0.006ms delivered period against a 50ms
// nominal kCycle (cycleBusy only ~21-23ms, so not a budget overrun) --
// traced to the OLD scheme, where each of the four runAndWait() pacing
// blocks computed its own wait as a gap relative to THAT block's own
// entry mark, so any rounding/overrun one block picked up was simply
// re-added on top of the next block's own fixed budget instead of being
// absorbed anywhere. 131-005 fixes this by making the trailing block's
// wait an absolute deadline anchored to the cycle's own start mark (see
// robot_loop.h's kCycle doc comment and robot_loop.cpp's runAndWaitUntil()/
// cycle() call site).
//
// JitterySleeper (below) is a small, purpose-built Hal::Sleeper that
// wraps a TestSim::SimClock and advances it by MORE than the requested
// duration on every sleepMillis() call -- modeling both components
// 130-011's own on-robot investigation named as the likely cause
// ("markTime()'s ms-truncation compounding across the 4 runAndWait()
// pacing blocks"): a per-call rounding-up (a fiber_sleep() request
// quantizing to the next whole tick) plus a small variable additional
// overrun (real busy work taking a little longer than budgeted). This is
// the SAME "a Sleeper that genuinely advances its paired clock" pattern
// already established for exactly this kind of sub-cycle-skew test --
// TickingSleeper, app_robot_loop_harness.cpp's own SUC-006 scenario (that
// harness file is currently xfailed for unrelated reasons -- an
// independent, pre-existing RobotLoop constructor-shape break, see its
// own file header -- but the Sleeper-that-advances-its-clock pattern it
// established is reused here verbatim, in a fresh file rather than
// resurrecting the broken one).
//
// Deliberately built directly on Core::composeRobot() (app/boot_wiring.h,
// 130-002's shared composition root), NOT TestSim::SimHarness
// (src/firm/platform/host/sim_harness.h): SimHarness's own step() always advances its
// internal SimClock by exactly kCycleDtUs BEFORE calling cycle() and owns
// a fixed, non-substitutable SimSleeper -- there is no seam to inject a
// jittery Sleeper through it (see sim_api_harness.cpp's own
// scenarioVirtualCycleTimingDiagnostic() 131-005 REWRITE note for the
// consequence: that scenario's fake clock cannot represent intra-cycle
// elapsed time at all, so it cannot prove this convergence claim --  this
// harness exists to prove it instead). composeRobot() takes a plain
// Hal::Sleeper&, the same interface seam main.cpp/SimHarness already
// use, so this harness supplies its own JitterySleeper there.
//
// Compiled and run by test_app_robot_loop_pacing.py (subprocess, exit
// code 0 == pass), mirroring every other src/tests/sim/unit harness's own
// shape.
#include <cstdint>
#include <cstdio>

#include "core/boot_wiring.h"
#include "core/robot_loop.h"
#include "hal/clock.h"
#include "fake_transport.h"
#include "sim_clock.h"
#include "sim_plant.h"

namespace {

int g_failures = 0;

void checkTrue(bool condition, const char* what) {
  if (!condition) {
    std::printf("  FAIL: %s\n", what);
    ++g_failures;
  } else {
    std::printf("  PASS: %s\n", what);
  }
}

// JitterySleeper -- see this file's own header. Advances the paired
// TestSim::SimClock by (requested + injected overrun) on every
// sleepMillis() call, and by a small fixed amount on yield() (a real
// fiber yield still costs some scheduler time on hardware -- kept
// nonzero so a degenerate all-yield cycle cannot look free).
class JitterySleeper : public Hal::Sleeper {
 public:
  explicit JitterySleeper(TestSim::SimClock& clock) : clock_(clock) {}

  void sleepMillis(uint32_t duration) override {  // [ms]
    // A deterministic, repeating overrun pattern -- varies call to call
    // (never the SAME extra tax every time) so this test cannot pass by
    // accident on one lucky constant. Never negative: real fiber_sleep()
    // rounds UP to the next tick, never early. Length 7, DELIBERATELY not
    // a divisor of 4 (the fixed number of sleepMillis() calls per cycle):
    // a period-4 (or period-1/2) pattern would re-align to the SAME
    // pacing block every cycle, degenerating into a single repeated
    // per-cycle total rather than genuinely varying which block absorbs
    // how much overrun from cycle to cycle -- exactly the "mean over many
    // simulated cycles" this test is supposed to exercise, not a
    // one-cycle constant repeated N times.
    static const uint32_t kOverrunPattern[] = {1, 0, 2, 1, 3, 0, 1};  // [ms]
    constexpr int kPatternLen = 7;
    const uint32_t overrun = kOverrunPattern[callIndex_ % kPatternLen];  // [ms]
    ++callIndex_;
    const uint32_t actual = duration + overrun;  // [ms]
    clock_.advanceMicros(static_cast<uint64_t>(actual) * 1000);  // [ms] -> [us]
    ++sleepCount_;
  }

  void yield() override {
    clock_.advanceMicros(50);  // [us] token scheduler cost, never free
    ++yieldCount_;
  }

  int sleepCount() const { return sleepCount_; }
  int yieldCount() const { return yieldCount_; }

 private:
  TestSim::SimClock& clock_;
  int callIndex_ = 0;
  int sleepCount_ = 0;
  int yieldCount_ = 0;
};

}  // namespace

int main() {
  TestSim::SimPlant plant;
  TestSim::SimClock clock;
  JitterySleeper sleeper(clock);
  TestSupport::FakeTransport serialFake;
  TestSupport::FakeTransport radioFake;

  Core::RobotGraph graph = Core::composeRobot(
      plant, clock, sleeper, serialFake, radioFake, /*tuningStore=*/nullptr,
      "DEVICE:NEZHA2:pacing_test:test:1", "ID:unknown", Core::BootOverrides{});

  // Boot: unlike TestSim::SimHarness::driveBootToDone() (src/firm/platform/host/
  // sim_harness.h), no manual clock pre-seeding/stepping is needed here --
  // RobotLoop::boot() itself loops `preamble_.step(); sleeper_.
  // sleepMillis(kPreamblePace);` until Preamble::done(), and THIS
  // JitterySleeper (unlike SimHarness's own plain SimSleeper) genuinely
  // advances the paired clock on every sleepMillis() call, so boot()'s
  // own loop makes real progress on its own.
  graph.robotLoop().boot();
  checkTrue(graph.preamble().done(), "Preamble reaches done() during boot() -- JitterySleeper's "
                                      "own clock advance drives real progress");

  // Measure: run many cycles, accumulating each cycle's OWN measured
  // state().time.cyclePeriod [us] -- the SAME field robot_loop.cpp's own
  // publishTiming() derives and telemetry reports (the observable this
  // whole sprint insists on asserting, never a proxy for it). The FIRST
  // cycle after boot() never reports a period (everCycled_ starts false,
  // publishTiming()'s own doc comment) -- skipped by the `period != 0`
  // guard below.
  constexpr int kCycles = 200;
  uint64_t sumPeriodUs = 0;  // [us]
  int periodSamples = 0;
  uint32_t maxPeriodUs = 0;   // [us]
  uint32_t minPeriodUs = UINT32_MAX;  // [us]
  for (int i = 0; i < kCycles; ++i) {
    graph.robotLoop().cycle();
    const uint32_t period = graph.robotLoop().state().time.cyclePeriod;  // [us]
    if (period != 0) {
      sumPeriodUs += period;
      ++periodSamples;
      if (period > maxPeriodUs) maxPeriodUs = period;
      if (period < minPeriodUs) minPeriodUs = period;
    }
  }

  checkTrue(periodSamples > kCycles / 2, "most cycles report a nonzero measured cyclePeriod");

  const double meanPeriodMs =
      periodSamples > 0 ? (static_cast<double>(sumPeriodUs) / periodSamples) / 1000.0 : 0.0;
  const double maxPeriodMs = static_cast<double>(maxPeriodUs) / 1000.0;
  const double minPeriodMs =
      (minPeriodUs == UINT32_MAX) ? 0.0 : static_cast<double>(minPeriodUs) / 1000.0;

  std::printf(
      "  MEASURED: mean delivered inter-cycle period = %.3fms, range [%.3f, %.3f]ms over %d "
      "samples (kCycle = %ums, injected per-call overrun pattern 1/0/2/1/3/0/1ms, period 7 -- "
      "genuinely varies which pacing block absorbs how much from cycle to cycle)\n",
      meanPeriodMs, minPeriodMs, maxPeriodMs, periodSamples,
      static_cast<unsigned>(Core::RobotLoop::kCycle));

  // The whole point: the mean converges to kCycle, not kCycle plus a
  // fixed structural offset (130-011 measured +4ms on real hardware, at
  // BOTH kCycle=40 and kCycle=50 -- a defect this same size regardless of
  // nominal period is exactly the "structural offset" signature this test
  // must NOT reproduce). A +/-2ms band comfortably separates "converges to
  // nominal" from "off by a structural ~4ms" while still tolerating this
  // harness's own deliberately-injected rounding (mean 1ms/call): three of
  // the four blocks' own rounding is absorbed into the trailing block's
  // shrunk wait by construction; only the trailing block's OWN rounding
  // (this cycle, ~1ms on average) cannot itself be absorbed by anything
  // later in the SAME cycle, but it does not compound into the NEXT
  // cycle either -- cycle()'s own cycleStart is a fresh markTime() read
  // every cycle, not carried forward from the previous one.
  const double kCycleF = static_cast<double>(Core::RobotLoop::kCycle);
  checkTrue(meanPeriodMs > kCycleF - 2.0 && meanPeriodMs < kCycleF + 2.0,
            "mean delivered inter-cycle period converges to kCycle (within +/-2ms), not "
            "kCycle plus a fixed structural offset (130-011's own +4ms-at-any-nominal defect)");
  checkTrue(maxPeriodMs > minPeriodMs,
            "delivered period genuinely VARIES cycle to cycle (the period-7 jitter pattern vs. "
            "4 calls/cycle keeps re-phasing which block absorbs how much) -- this is a real "
            "mean over varying samples, not one cycle's constant repeated many times");

  if (g_failures > 0) {
    std::printf("%d scenario(s) FAILED\n", g_failures);
    return 1;
  }
  std::printf("All scenarios PASSED\n");
  return 0;
}
