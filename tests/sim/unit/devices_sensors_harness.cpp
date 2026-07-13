// devices_sensors_harness.cpp — off-hardware acceptance harness for ticket
// DB-006 (device-bus-tickets.md): exercises the REAL Devices::ColorSensor
// and Devices::LineSensor leaves (source/devices/color_sensor.cpp,
// source/devices/line_sensor.cpp) against DB-003's HOST_BUILD scripted
// Devices::I2CBus fake -- no MicroBitI2C, no CODAL, no real hardware.
//
// Modeled on tests/sim/unit/devices_otos_harness.cpp (that file's own header
// comment is this harness's explicit test precedent) -- compiles the ACTUAL
// source/devices/{color_sensor,line_sensor}.cpp against the SAME
// source/devices/{color_sensor,line_sensor}.h every ARM build compiles, with
// -DHOST_BUILD selecting source/devices/i2c_bus_host.cpp's scripted fake in
// place of the real MicroBitI2C-backed i2c_bus.cpp. Hand-rolled assertions,
// PASS/FAIL per scenario, nonzero exit on any failure. Run by
// test_devices_sensors.py, which compiles and runs this binary via
// subprocess. Includes ONLY devices/ headers plus plain C/C++ stdlib
// (isolation invariant) -- no messages/*.h, no hal/*, no source_old/*.
//
// --- Scripting model recap (i2c_bus_host.cpp) ---
// scriptWrite()/scriptRead() are TWO SEPARATE FIFOs (writes vs. reads), each
// matched strictly in the order the leaf under test calls write()/read() --
// content is not checked for writes (only address+order); reads deliver the
// scripted payload bytes IF the address matches, else the caller's buffer is
// left as pre-initialized (typically 0). An UNSCRIPTED write/read (empty
// queue) returns a distinct "mismatch" status without crashing -- this
// harness deliberately leans on that for the "absent device" scenario (an
// entirely empty I2CBus decodes to all-zero reads, exactly modeling a chip
// that never answers).
//
// The scriptXxx() helpers below each push EXACTLY the writes+reads one call
// to the leaf's corresponding private helper issues, in the SAME order --
// see each helper's comment for the transaction-count derivation from
// color_sensor.cpp/line_sensor.cpp.

#include <cstdint>
#include <cstdio>
#include <string>

#include "devices/color_sensor.h"
#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/i2c_bus.h"
#include "devices/line_sensor.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors devices_otos_harness.cpp) ---

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

void checkFalse(bool condition, const std::string& what) {
  if (condition) fail(what + " -- expected false, got true");
}

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %u, got %u", what.c_str(),
                  static_cast<unsigned>(expected), static_cast<unsigned>(actual));
    fail(buf);
  }
}

// --- Fixture helpers ---------------------------------------------------

constexpr uint16_t kAltWireAddr = static_cast<uint16_t>(Devices::kColorDeviceAddrAlt << 1);
constexpr uint16_t kApdsWireAddr = static_cast<uint16_t>(Devices::kColorDeviceAddrApds << 1);
constexpr uint16_t kLineWireAddr = static_cast<uint16_t>(Devices::kLineDeviceAddr << 1);

// kAltRetryPeriod/kLineRetryPeriod duplicated from color_sensor.h/
// line_sensor.h's private constants -- this file's own established
// convention (see devices_otos_harness.cpp's kReadPeriodUs) for restating a
// private leaf constant a test needs to pace its own clock advances by.
constexpr uint64_t kAltRetryPeriodUs = 50000;
constexpr uint64_t kLineRetryPeriodUs = 50000;
constexpr int kMaxAltAttempts = 20;
constexpr int kMaxLineAttempts = 20;

// Scripts one readReg16Alt()-shaped call: two write(reg-addr)/read(1-byte)
// pairs (lo then hi), matching readReg16AltStatus()'s exact call order.
void scriptAltReg16(Devices::I2CBus& bus, uint16_t value) {
  bus.scriptWrite(kAltWireAddr, 0);
  bus.scriptWrite(kAltWireAddr, 0);
  uint8_t lo[1] = {static_cast<uint8_t>(value & 0xFF)};
  uint8_t hi[1] = {static_cast<uint8_t>((value >> 8) & 0xFF)};
  bus.scriptRead(kAltWireAddr, lo, 1, 0);
  bus.scriptRead(kAltWireAddr, hi, 1, 0);
}

// Scripts one beginStep() AltProbe attempt: writeReg8(0x81) + writeReg8(0x80)
// + readReg16Alt(0xA4) -- 4 writes + 2 reads total (color_sensor.cpp
// beginStep()'s AltProbe branch).
void scriptAltDetectAttempt(Devices::I2CBus& bus, uint16_t probeValue) {
  bus.scriptWrite(kAltWireAddr, 0);  // writeReg8(0x81, 0xCA)
  bus.scriptWrite(kAltWireAddr, 0);  // writeReg8(0x80, 0x17)
  scriptAltReg16(bus, probeValue);   // readReg16Alt(0xA4) -- adds 2W + 2R
}

// Scripts beginStep()'s ApdsProbe attempt (2 writes + 1 read): writeReg8
// (0x80,0x00) then readReg8(0x80).
void scriptApdsDetectProbe(Devices::I2CBus& bus, uint8_t enReadback) {
  bus.scriptWrite(kApdsWireAddr, 0);
  bus.scriptWrite(kApdsWireAddr, 0);
  uint8_t data[1] = {enReadback};
  bus.scriptRead(kApdsWireAddr, data, 1, 0);
}

// Scripts initApds()'s full register program (8 writes + 1 read) -- called
// by beginStep() only when scriptApdsDetectProbe()'s enReadback == 0x00.
void scriptInitApds(Devices::I2CBus& bus, uint8_t enBeforeOr) {
  for (int i = 0; i < 6; ++i) bus.scriptWrite(kApdsWireAddr, 0);  // ATIME/CONTROL/EN off/AB/E7/EN on
  bus.scriptWrite(kApdsWireAddr, 0);                              // readReg8(0x80)'s internal write
  uint8_t data[1] = {enBeforeOr};
  bus.scriptRead(kApdsWireAddr, data, 1, 0);
  bus.scriptWrite(kApdsWireAddr, 0);  // final writeReg8(0x80, en|0x02)
}

// Scripts one readReg16Status()-shaped call (single 2-byte burst read) --
// the APDS steady-read register shape (tick()'s c/r/g/b reads).
void scriptApdsReg16(Devices::I2CBus& bus, uint16_t value) {
  bus.scriptWrite(kApdsWireAddr, 0);
  uint8_t raw[2] = {static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>((value >> 8) & 0xFF)};
  bus.scriptRead(kApdsWireAddr, raw, 2, 0);
}

// Scripts one readReg8Status()-shaped call to the APDS STATUS register
// (0x93) -- tick()'s AVALID poll.
void scriptApdsStatus(Devices::I2CBus& bus, uint8_t statusByte) {
  bus.scriptWrite(kApdsWireAddr, 0);
  uint8_t data[1] = {statusByte};
  bus.scriptRead(kApdsWireAddr, data, 1, 0);
}

// Scripts one LineSensor readRaw() call: 4 channel write(index)/read(byte)
// pairs.
void scriptLineRead(Devices::I2CBus& bus, const uint16_t raw[4]) {
  for (int ch = 0; ch < 4; ++ch) {
    bus.scriptWrite(kLineWireAddr, 0);
    uint8_t data[1] = {static_cast<uint8_t>(raw[ch])};
    bus.scriptRead(kLineWireAddr, data, 1, 0);
  }
}

// Local oracle for LineSensor::tick()'s normalize step (line_sensor.cpp) --
// this file's own established convention (mirrors devices_otos_harness.cpp's
// testSensorToCentre()) for a test oracle that can't reach a production
// symbol directly. alpha == 0 in every scenario below, so emaState is unused
// (kept for signature parity/documentation of the full formula).
int32_t expectedNormalize(uint16_t raw, uint32_t mn, uint32_t mx) {
  uint32_t span = (mx > mn) ? (mx - mn) : 255u;
  int32_t norm;
  if (raw <= mn) {
    norm = 0;
  } else if (raw >= mx) {
    norm = 1000;
  } else {
    norm = (static_cast<int32_t>(raw - mn) * 1000) / static_cast<int32_t>(span);
  }
  if (norm < 0) norm = 0;
  if (norm > 1000) norm = 1000;
  return norm;
}

// --- Scenarios ------------------------------------------------------------

// 1. ColorSensor: ALT chip detection succeeds only after several re-wake
//    retries (the exact docs/knowledge/encoders-read-zero-i2c-bus-hang.md
//    "re-assert wake registers each retry" sequence), never blocking (each
//    beginStep() call returns immediately; the harness paces nowUs itself).
//    Then a scripted color frame decodes to the expected r/g/b/c, and an
//    immediately-repeated tick() (readDue() still false) issues zero further
//    bus traffic.
void scenarioColorAltDetectionSucceedsViaRewakeRetry() {
  beginScenario("ColorSensor: ALT detection succeeds via re-wake retry, then decodes a scripted frame");

  Devices::I2CBus bus;
  Devices::ColorConfig cfg;  // zero-defaulted -> leaf applies its ship defaults
  Devices::ColorSensor sensor(bus, cfg);

  // Attempts 1 and 2: chip still powering up -- probe reads back zero.
  scriptAltDetectAttempt(bus, 0x0000);
  scriptAltDetectAttempt(bus, 0x0000);
  // Attempt 3: chip has woken up -- non-zero probe.
  scriptAltDetectAttempt(bus, 0x1234);

  int attemptsUsed = 0;
  for (int i = 0; i < kMaxAltAttempts + 2; ++i) {
    checkFalse(sensor.detectDone(), "detection not yet done before the successful attempt");
    sensor.beginStep(static_cast<uint64_t>(i) * kAltRetryPeriodUs);
    attemptsUsed = i + 1;
    if (sensor.detectDone()) break;
  }

  checkUintEq(static_cast<uint32_t>(attemptsUsed), 3, "detection succeeded on exactly the 3rd re-wake attempt");
  checkTrue(sensor.detectDone(), "detectDone() true after the successful attempt");
  checkTrue(sensor.present(), "present() true -- ALT chip detected");
  checkTrue(sensor.connected(), "connected() true -- ALT chip detected");
  checkUintEq(bus.errCount(Devices::kColorDeviceAddrAlt), 0, "no script under-run across the retry sequence");

  // A further beginStep() call is a total no-op (already Done).
  uint32_t txnBeforeExtra = bus.txnCount(Devices::kColorDeviceAddrAlt);
  sensor.beginStep(static_cast<uint64_t>(attemptsUsed) * kAltRetryPeriodUs);
  checkUintEq(bus.txnCount(Devices::kColorDeviceAddrAlt), txnBeforeExtra,
              "beginStep() after detectDone() issues zero further bus traffic");

  // A scripted ALT frame decodes to the expected r/g/b/c.
  constexpr uint16_t kC = 0x00AB, kR = 0x1234, kG = 0x5678, kB = 0x0FED;
  scriptAltReg16(bus, kC);  // 0xA6 probe (non-zero -> data ready)
  scriptAltReg16(bus, kR);  // 0xA0
  scriptAltReg16(bus, kG);  // 0xA2
  scriptAltReg16(bus, kB);  // 0xA4

  uint64_t tickNow = 10000000;
  sensor.tick(tickNow);

  checkTrue(sensor.readingFresh(), "tick(): readingFresh() true after a ready scripted frame");
  checkTrue(sensor.connected(), "tick(): connected() true after a clean frame read");
  Devices::ColorReading reading = sensor.reading();
  checkUintEq(reading.r, kR, "reading().r matches the scripted frame");
  checkUintEq(reading.g, kG, "reading().g matches the scripted frame");
  checkUintEq(reading.b, kB, "reading().b matches the scripted frame");
  checkUintEq(reading.c, kC, "reading().c matches the scripted frame");

  // readDue() rate limit: an immediately-repeated tick() (same nowUs) issues
  // zero further bus traffic.
  uint32_t txnAfterFrame = bus.txnCount(Devices::kColorDeviceAddrAlt);
  sensor.tick(tickNow);
  checkUintEq(bus.txnCount(Devices::kColorDeviceAddrAlt), txnAfterFrame,
              "tick() called again before readDue(): zero further bus traffic");

  checkUintEq(bus.errCount(Devices::kColorDeviceAddrAlt), 0, "no script under-run across the frame decode");
}

// 2. ColorSensor: ALT never responds across all kMaxAltAttempts retries --
//    detection falls back to probing the APDS9960 at 0x39, which answers;
//    present()/connected() become true via the fallback path. A scripted
//    APDS frame then decodes to the expected r/g/b/c too.
void scenarioColorFallsBackToApds() {
  beginScenario("ColorSensor: ALT never responds -- detection falls back to APDS9960");

  Devices::I2CBus bus;
  Devices::ColorConfig cfg;
  Devices::ColorSensor sensor(bus, cfg);

  for (int i = 0; i < kMaxAltAttempts; ++i) {
    scriptAltDetectAttempt(bus, 0x0000);  // every ALT attempt reads back zero
  }
  scriptApdsDetectProbe(bus, 0x00);  // ENABLE readback 0x00 -- APDS answered
  scriptInitApds(bus, 0x01);         // init()'s own EN readback (arbitrary nonzero)

  int callsUsed = 0;
  for (int i = 0; i < kMaxAltAttempts + 2; ++i) {
    sensor.beginStep(static_cast<uint64_t>(i) * kAltRetryPeriodUs);
    callsUsed = i + 1;
    if (sensor.detectDone()) break;
  }

  checkUintEq(static_cast<uint32_t>(callsUsed), static_cast<uint32_t>(kMaxAltAttempts) + 1,
              "detection took exactly kMaxAltAttempts ALT attempts plus one APDS attempt");
  checkTrue(sensor.detectDone(), "detectDone() true after the APDS fallback succeeds");
  checkTrue(sensor.present(), "present() true -- APDS9960 detected via fallback");
  checkTrue(sensor.connected(), "connected() true -- APDS9960 detected via fallback");
  checkUintEq(bus.errCount(Devices::kColorDeviceAddrAlt), 0, "no script under-run on the ALT side");
  checkUintEq(bus.errCount(Devices::kColorDeviceAddrApds), 0, "no script under-run on the APDS side");

  // A scripted APDS frame decodes to the expected r/g/b/c.
  constexpr uint16_t kC = 0x0111, kR = 0x0222, kG = 0x0333, kB = 0x0444;
  scriptApdsStatus(bus, 0x01);  // AVALID set -- data ready
  scriptApdsReg16(bus, kC);     // 0x94
  scriptApdsReg16(bus, kR);     // 0x96
  scriptApdsReg16(bus, kG);     // 0x98
  scriptApdsReg16(bus, kB);     // 0x9A

  sensor.tick(2000000);
  checkTrue(sensor.readingFresh(), "tick(): readingFresh() true after a ready scripted APDS frame");
  Devices::ColorReading reading = sensor.reading();
  checkUintEq(reading.r, kR, "reading().r matches the scripted APDS frame");
  checkUintEq(reading.g, kG, "reading().g matches the scripted APDS frame");
  checkUintEq(reading.b, kB, "reading().b matches the scripted APDS frame");
  checkUintEq(reading.c, kC, "reading().c matches the scripted APDS frame");

  checkUintEq(bus.errCount(Devices::kColorDeviceAddrApds), 0, "no script under-run across the APDS frame decode");
}

// 3. LineSensor: detection succeeds (a single successful readRaw() probe),
//    then a scripted raw frame normalizes to the expected 4-channel values
//    under two different calibration windows (ship-default and custom).
void scenarioLineRawToNormalized() {
  beginScenario("LineSensor: raw -> normalized produces the expected 4-channel values");

  Devices::I2CBus bus;
  Devices::LineConfig cfg;  // zero-defaulted -> calMin=0, calMax defaults to 255/channel
  Devices::LineSensor sensor(bus, cfg);

  uint16_t probeRaw[4] = {5, 5, 5, 5};
  scriptLineRead(bus, probeRaw);  // beginStep()'s detection probe

  sensor.beginStep(0);
  checkTrue(sensor.detectDone(), "detectDone() true after the first successful probe");
  checkTrue(sensor.present(), "present() true -- line sensor answered");
  checkTrue(sensor.connected(), "connected() true -- line sensor answered");
  checkUintEq(bus.errCount(Devices::kLineDeviceAddr), 0, "no script under-run on detection");

  // Ship-default calibration (calMin=0, calMax=255/channel): raw values
  // spanning white/mid/black.
  uint16_t rawA[4] = {0, 128, 255, 64};
  scriptLineRead(bus, rawA);
  sensor.tick(1000000);

  checkTrue(sensor.readingFresh(), "tick(): readingFresh() true after a clean raw read");
  Devices::LineReading readingA = sensor.reading();
  for (int ch = 0; ch < 4; ++ch) {
    checkUintEq(readingA.raw[ch], rawA[ch], "reading().raw[ch] passes the raw byte through unmodified");
    int32_t expected = expectedNormalize(rawA[ch], 0, 255);
    checkUintEq(readingA.normalized[ch], static_cast<uint32_t>(expected),
                "reading().normalized[ch] matches the ship-default-calibration oracle");
  }

  // Custom calibration window (calMin=50, calMax=200/channel) -- a fresh
  // LineSensor so beginStep()'s config_ starts from these explicit bounds.
  Devices::I2CBus bus2;
  Devices::LineConfig cfg2;
  for (int ch = 0; ch < 4; ++ch) {
    cfg2.calMin[ch] = 50;
    cfg2.calMax[ch] = 200;
  }
  Devices::LineSensor sensor2(bus2, cfg2);
  uint16_t probeRaw2[4] = {100, 100, 100, 100};
  scriptLineRead(bus2, probeRaw2);
  sensor2.beginStep(0);
  checkTrue(sensor2.present(), "present() true (custom calibration fixture)");

  uint16_t rawB[4] = {10, 50, 125, 220};  // below-min, near-min, mid, above-max
  scriptLineRead(bus2, rawB);
  sensor2.tick(1000000);

  checkTrue(sensor2.readingFresh(), "tick(): readingFresh() true (custom calibration)");
  Devices::LineReading readingB = sensor2.reading();
  for (int ch = 0; ch < 4; ++ch) {
    checkUintEq(readingB.raw[ch], rawB[ch], "reading().raw[ch] passes through unmodified (custom calibration)");
    int32_t expected = expectedNormalize(rawB[ch], 50, 200);
    checkUintEq(readingB.normalized[ch], static_cast<uint32_t>(expected),
                "reading().normalized[ch] matches the custom-calibration oracle");
  }

  checkUintEq(bus.errCount(Devices::kLineDeviceAddr), 0, "no script under-run (default-calibration fixture)");
  checkUintEq(bus2.errCount(Devices::kLineDeviceAddr), 0, "no script under-run (custom-calibration fixture)");
}

// 4. Absent-device detection: a completely unscripted I2CBus (every
//    write()/read() call returns the fake's "mismatch" status and decodes to
//    zero bytes -- exactly how a chip that never answers behaves) marks
//    BOTH ColorSensor and LineSensor not-connected within a small, BOUNDED
//    number of beginStep() calls -- proving termination (never hangs)
//    without any real sleep or an unbounded loop.
void scenarioAbsentDeviceNeverHangs() {
  beginScenario("Absent-device detection: bounded termination, present()/connected() stay false");

  // ColorSensor: no chip at either 0x43 or 0x39. beginStep()'s AltProbe/
  // ApdsProbe branches ignore transaction STATUS (an exact port of the
  // pre-port driver, which never checked it either -- see color_sensor.h's
  // file header) -- they only look at the decoded DATA value, so "absent"
  // must be modeled with explicit register VALUES that fail each branch's
  // own success test, not just an empty/unscripted bus: an unscripted read
  // defaults its buffer to 0, which happens to BE the ApdsProbe *success*
  // value (readback == 0x00) -- an artifact of this harness's zero-init,
  // not a real absent-device signature. 0x00/0x00 correctly fails the ALT
  // test (probe != 0); 0xFF correctly fails the APDS test (readback ==
  // 0x00) the way a floating/un-acked real bus would.
  {
    Devices::I2CBus bus;
    for (int i = 0; i < kMaxAltAttempts; ++i) {
      scriptAltDetectAttempt(bus, 0x0000);
    }
    scriptApdsDetectProbe(bus, 0xFF);  // readback != 0x00 -- APDS never answered either
    Devices::ColorConfig cfg;
    Devices::ColorSensor sensor(bus, cfg);

    int callsUsed = 0;
    const int kBound = kMaxAltAttempts + 2;  // 20 ALT attempts + 1 APDS attempt + margin
    for (int i = 0; i < kBound; ++i) {
      sensor.beginStep(static_cast<uint64_t>(i) * kAltRetryPeriodUs);
      callsUsed = i + 1;
      if (sensor.detectDone()) break;
    }

    checkTrue(sensor.detectDone(), "ColorSensor: detectDone() reached within a bounded call count");
    checkTrue(callsUsed <= kBound, "ColorSensor: termination bounded (never an unbounded/hanging loop)");
    checkFalse(sensor.present(), "ColorSensor: present() false -- neither chip ever answered");
    checkFalse(sensor.connected(), "ColorSensor: connected() false -- neither chip ever answered");

    // tick() on a never-detected leaf is a safe, total no-op.
    sensor.tick(999999999);
    checkFalse(sensor.readingFresh(), "ColorSensor: tick() on a never-detected leaf stays not-fresh");
  }

  // LineSensor: no chip at 0x1A.
  {
    Devices::I2CBus bus;  // no scripts queued at all
    Devices::LineConfig cfg;
    Devices::LineSensor sensor(bus, cfg);

    int callsUsed = 0;
    const int kBound = kMaxLineAttempts + 2;
    for (int i = 0; i < kBound; ++i) {
      sensor.beginStep(static_cast<uint64_t>(i) * kLineRetryPeriodUs);
      callsUsed = i + 1;
      if (sensor.detectDone()) break;
    }

    checkTrue(sensor.detectDone(), "LineSensor: detectDone() reached within a bounded call count");
    checkTrue(callsUsed <= kBound, "LineSensor: termination bounded (never an unbounded/hanging loop)");
    checkFalse(sensor.present(), "LineSensor: present() false -- chip never answered");
    checkFalse(sensor.connected(), "LineSensor: connected() false -- chip never answered");

    sensor.tick(999999999);
    checkFalse(sensor.readingFresh(), "LineSensor: tick() on a never-detected leaf stays not-fresh");
  }
}

}  // namespace

int main() {
  scenarioColorAltDetectionSucceedsViaRewakeRetry();
  scenarioColorFallsBackToApds();
  scenarioLineRawToNormalized();
  scenarioAbsentDeviceNeverHangs();

  if (g_failureCount == 0) {
    std::printf("OK: all Devices::ColorSensor/LineSensor scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Devices sensor scenarios\n", g_failureCount);
  return 1;
}
