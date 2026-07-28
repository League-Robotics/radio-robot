// nezha_staging_only_harness.cpp — off-hardware acceptance harness for
// ticket 094-004's second hard-constraint verification: confirms
// `Hal::NezhaMotor::setVelocity()`/`setDutyCycle()` remain STAGING-ONLY --
// no I2C bus write happens until an explicit `tick()` call (the flip-flop's
// own `COLLECT_DUE` dispatch, `NezhaHardware::serviceBus()`) -- the
// load-bearing assumption behind "Drivetrain stages now, `serviceBus`
// flushes next pass = identical one-pass latency to the pre-094
// `routeOutputs()` chain" (see drivetrain.h's class comment and
// architecture-update.md Section 5).
//
// Constructs a bare `Hal::NezhaMotor` directly (not through
// `Subsystems::NezhaHardware` -- this is a Motor-level property, not a
// flip-flop-sequencing one; `hardware_seam_harness.cpp`/
// `nezha_flipflop_harness.cpp` already cover the sequencer itself) against
// the HOST_BUILD scripted `I2CBus` fake, and asserts `I2CBus::txnCount()`
// is UNCHANGED by `setVelocity()` alone, then increases only once `tick()`
// actually dispatches (`armoredWrite()` -> `writeRawDuty()` -> the bus
// write). Mirrors hardware_seam_harness.cpp's shape: hand-rolled
// assertions, PASS/FAIL per scenario, nonzero exit on any failure. Run by
// test_nezha_staging_only.py.
#include <cstdint>
#include <cstdio>
#include <string>

#include "com/i2c_bus.h"
#include "hal/nezha/nezha_motor.h"
#include "messages/motor.h"

namespace {

int g_failureCount = 0;
std::string g_scenarioName;

void beginScenario(const std::string& name) {
  g_scenarioName = name;
  std::printf("--- %s\n", name.c_str());
}

void fail(const std::string& what) {
  ++g_failureCount;
  std::printf("  FAIL [%s]: %s\n", g_scenarioName.c_str(), what.c_str());
}

void checkTrue(bool condition, const std::string& what) {
  if (!condition) fail(what + " -- expected true, got false");
}

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %u, got %u", what.c_str(),
                  static_cast<unsigned>(expected), static_cast<unsigned>(actual));
    fail(buf);
  }
}

constexpr uint16_t kAddr7 = Hal::kNezhaDeviceAddr;                  // 0x10 (clear()/txnCount()'s convention)
constexpr uint16_t kWireAddr = static_cast<uint16_t>(kAddr7 << 1);  // 0x20 (write()/read()'s convention)

// Pre-loads a generous, uniform pool of canned (write, read) responses --
// mirrors hardware_seam_harness.cpp's own scriptGenerousPool() precedent
// (an over-sized pool is sufficient and self-detecting: errCount() would
// catch any under-run, though this harness's short scenarios need only a
// handful).
void scriptGenerousPool(I2CBus& bus, int count) {
  static uint8_t canned[4] = {0, 0, 0, 0};
  for (int i = 0; i < count; ++i) {
    bus.scriptWrite(kWireAddr, /*status=*/0);
    bus.scriptRead(kWireAddr, canned, 4, /*status=*/0);
  }
}

// 1. setVelocity() alone -- a plain C++ member write (mode_/velocityTarget_)
// -- performs ZERO I2C transactions.
void scenarioSetVelocityIsStagingOnly() {
  beginScenario("NezhaMotor::setVelocity() alone performs zero I2C transactions (staging-only)");
  I2CBus::setClock(1000000);
  I2CBus bus;
  scriptGenerousPool(bus, 10);

  msg::MotorConfig cfg;
  cfg.setPort(1).setFwdSign(1).setTravelCalib(1.0f);
  Hal::NezhaMotor motor(bus, cfg);
  motor.configure(cfg);

  uint32_t before = bus.txnCount(kAddr7);
  motor.setVelocity(200.0f);
  checkUintEq(bus.txnCount(kAddr7), before,
              "setVelocity() performed zero I2C transactions");

  // tick() -- the ONLY path that dispatches to the bus (collectEncoder()'s
  // read, then, for VELOCITY mode, armoredWrite() -> writeRawDuty()'s
  // write) -- see NezhaMotor::tick()'s own 5-step call-order contract.
  motor.tick(1000);
  checkTrue(bus.txnCount(kAddr7) > before,
            "tick() (serviceBus()'s COLLECT_DUE dispatch) is what actually reaches the bus");
}

// 2. setDutyCycle() alone -- same staging-only property, the OTHER live
// escape-hatch mode Drivetrain's WHEELS/TWIST arms could in principle
// reach (Motor::apply()'s DUTY_CYCLE arm) -- though Drivetrain itself only
// ever calls setVelocity() (drivetrain.cpp's tick()); this scenario proves
// the property holds at the Hal::NezhaMotor level generally, not merely
// for the one mode this ticket's Drivetrain happens to use.
void scenarioSetDutyCycleIsStagingOnly() {
  beginScenario("NezhaMotor::setDutyCycle() alone performs zero I2C transactions (staging-only)");
  I2CBus::setClock(2000000);
  I2CBus bus;
  scriptGenerousPool(bus, 10);

  msg::MotorConfig cfg;
  cfg.setPort(1).setFwdSign(1).setTravelCalib(1.0f);
  Hal::NezhaMotor motor(bus, cfg);
  motor.configure(cfg);

  uint32_t before = bus.txnCount(kAddr7);
  motor.setDutyCycle(0.5f);
  checkUintEq(bus.txnCount(kAddr7), before,
              "setDutyCycle() performed zero I2C transactions");

  motor.tick(1000);
  checkTrue(bus.txnCount(kAddr7) > before,
            "tick() is what actually reaches the bus");
}

}  // namespace

int main() {
  scenarioSetVelocityIsStagingOnly();
  scenarioSetDutyCycleIsStagingOnly();

  if (g_failureCount == 0) {
    std::printf("OK: NezhaMotor staging-only verification passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the staging-only scenarios\n", g_failureCount);
  return 1;
}
