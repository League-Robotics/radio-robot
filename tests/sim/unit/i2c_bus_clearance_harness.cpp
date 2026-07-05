// i2c_bus_clearance_harness.cpp — off-hardware acceptance harness for ticket
// 079-001 (SUC-008/SUC-009): exercises I2CBus's lazy per-device clearance
// timers (DeviceSlot.lastEnd/readyAt, write()/read()'s preClear/postClear,
// the non-spinning clear() peek) through the HOST_BUILD scripted fake
// (source/com/i2c_bus_host.cpp) — no MicroBitI2C, no CODAL, no wall clock.
//
// Compiled with -DHOST_BUILD (see test_i2c_bus_clearance.py's compile
// command), together with i2c_bus_host.cpp, against the same
// source/com/i2c_bus.h every ARM build compiles — so this proves the
// HOST_BUILD fork of that shared header/class actually builds and behaves,
// per architecture-update.md's "HOST_BUILD stub" section.
//
// Plain C++ program, hand-rolled assertions (a handful of scenarios do not
// warrant a test-framework dependency) — mirrors tests/sim/unit/
// motor_policy_harness.cpp's shape exactly: prints a PASS/FAIL line per
// scenario and exits nonzero if any assertion failed, run by the pytest
// wrapper in test_i2c_bus_clearance.py.

#include <cstdint>
#include <cstdio>
#include <string>

#include "com/i2c_bus.h"

namespace {

// --- Hand-rolled assertion plumbing (see motor_policy_harness.cpp) --------

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
  if (!condition) fail(what + " — expected true, got false");
}

void checkFalse(bool condition, const std::string& what) {
  if (condition) fail(what + " — expected false, got true");
}

void checkIntEq(int actual, int expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s — expected %d, got %d", what.c_str(),
                  expected, actual);
    fail(buf);
  }
}

void checkU64Eq(uint64_t actual, uint64_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s — expected %llu, got %llu",
                  what.c_str(), static_cast<unsigned long long>(expected),
                  static_cast<unsigned long long>(actual));
    fail(buf);
  }
}

// Nezha-shaped 8-byte frame; the fake never inspects payload content beyond
// the (off-by-default) transaction log, so its bytes are arbitrary.
uint8_t dummyFrame[8] = {0xFF, 0xF9, 1, 0x00, 0x46, 0x00, 0xF5, 0x00};

// 7-bit device address vs. its 8-bit wire (shifted) counterpart — the exact
// pair the source issue calls out as an easy off-by-one-bit trap.
constexpr uint16_t kAddr7 = 0x10;
constexpr uint16_t kWireAddr = static_cast<uint16_t>(kAddr7 << 1);   // 0x20

// --- Scenarios --------------------------------------------------------

// 1. Defaults (preClear=postClear=0) behave exactly like the pre-clearance-
//    timer bus: no spin (the fake clock, read via I2CBus::clock(), is
//    unchanged across the call — a live spin would auto-advance it) and the
//    scripted status is returned immediately.
void scenarioDefaultsAreFree() {
  beginScenario("defaults (preClear=postClear=0): no spin, immediate status");
  I2CBus::setClock(1000);
  I2CBus bus;
  bus.scriptWrite(kWireAddr, /*status=*/0);

  uint64_t before = I2CBus::clock();
  int status = bus.write(kWireAddr, dummyFrame, 8, false);
  uint64_t after = I2CBus::clock();

  checkIntEq(status, 0, "write(): scripted status returned unchanged");
  checkU64Eq(after, before, "write(): fake clock did not advance — no spin occurred");

  uint8_t canned[4] = {0xAA, 0xBB, 0xCC, 0xDD};
  uint8_t resp[4] = {0, 0, 0, 0};
  bus.scriptRead(kWireAddr, canned, 4, /*status=*/0);

  uint64_t beforeRead = I2CBus::clock();
  int readStatus = bus.read(kWireAddr, resp, 4, false);
  uint64_t afterRead = I2CBus::clock();

  checkIntEq(readStatus, 0, "read(): scripted status returned unchanged");
  checkU64Eq(afterRead, beforeRead, "read(): fake clock did not advance — no spin occurred");
}

// 2. A write's postClear holds the DEVICE (not just the call site) until
//    the clock clears the deadline — clear()'s peek reflects it, and
//    reflects it correctly once the test advances the clock past the
//    deadline (per architecture-update.md's exact contract:
//    lastEnd = now, readyAt = lastEnd + postClear).
void scenarioPostClearHoldsUntilClockAdvances() {
  beginScenario("postClear holds the device until the clock clears the deadline");
  I2CBus::setClock(5000);
  I2CBus bus;
  bus.scriptWrite(kWireAddr, /*status=*/0);

  int status = bus.write(kWireAddr, dummyFrame, 8, false,
                          /*preClear=*/0, /*postClear=*/4000);
  checkIntEq(status, 0, "write status");

  checkFalse(bus.clear(kAddr7), "clear(0x10) false immediately after the write");

  I2CBus::advanceClock(3999);
  checkFalse(bus.clear(kAddr7), "clear(0x10) still false 1us short of the deadline");

  I2CBus::advanceClock(1);
  checkTrue(bus.clear(kAddr7), "clear(0x10) true once the clock reaches readyAt");
}

// 3. The 7-bit-vs-8-bit convention: clear() takes the bare 7-bit address
//    (0x10), NOT the shifted 8-bit wire address (0x20) write()/read() take.
//    Passing the wire address by mistake queries a device that was never
//    transacted with, which clear() always reports clear — a DIFFERENT
//    (and silently wrong) answer from the real device's held-until-cleared
//    state. This is the off-by-one-bit trap the source issue calls out.
void scenario7BitVs8BitConvention() {
  beginScenario("clear() takes the 7-bit address, not the 8-bit wire address");
  I2CBus::setClock(9000);
  I2CBus bus;
  bus.scriptWrite(kWireAddr, /*status=*/0);
  bus.write(kWireAddr, dummyFrame, 8, false, /*preClear=*/0, /*postClear=*/4000);

  bool clearBareAddr = bus.clear(kAddr7);       // 0x10 — correct convention
  bool clearWireAddr = bus.clear(kWireAddr);    // 0x20 — the wrong-bit mistake

  checkFalse(clearBareAddr, "clear(0x10): the real device, still held");
  checkTrue(clearWireAddr, "clear(0x20): an untouched slot, always clear");
  checkTrue(clearBareAddr != clearWireAddr,
            "clear(0x10) and clear(0x20) must disagree — guards the bit convention");
}

// 4. A live entry spin (preClear whose deadline is still in the future)
//    self-advances the fake clock until the deadline is met, rather than
//    hanging — the HOST_BUILD safety net documented in i2c_bus_host.cpp —
//    and the resulting transaction still returns its own scripted status.
void scenarioPreClearSpinAutoAdvances() {
  beginScenario("a live preClear spin self-advances the fake clock and completes");
  I2CBus::setClock(0);
  I2CBus bus;
  bus.scriptWrite(kWireAddr, /*status=*/0);    // first write: arms postClear
  bus.scriptWrite(kWireAddr, /*status=*/0);    // second write: gated by preClear

  bus.write(kWireAddr, dummyFrame, 8, false, /*preClear=*/0, /*postClear=*/4000);
  uint64_t readyAt = I2CBus::clock() + 4000;   // lastEnd (== clock() here) + postClear

  // No advanceClock() call here — the second write's preClear=4000 entry
  // spin must clear the deadline on its own.
  int status2 = bus.write(kWireAddr, dummyFrame, 8, false,
                           /*preClear=*/4000, /*postClear=*/0);

  checkIntEq(status2, 0, "second (spin-gated) write still returns its scripted status");
  checkTrue(I2CBus::clock() >= readyAt,
            "fake clock landed at/after the deadline once the spin self-advanced it");
}

// 5. read() honors the same clearance contract and copies scripted response
//    bytes into the caller's buffer — the collectEncoder()-style path.
void scenarioReadRespectsClearanceAndData() {
  beginScenario("read() honors preClear/postClear and copies scripted bytes");
  I2CBus::setClock(20000);
  I2CBus bus;
  uint8_t canned[4] = {0x11, 0x22, 0x33, 0x44};
  bus.scriptRead(kWireAddr, canned, 4, /*status=*/0);

  uint8_t resp[4] = {0, 0, 0, 0};
  int status = bus.read(kWireAddr, resp, 4, false, /*preClear=*/0, /*postClear=*/1500);
  checkIntEq(status, 0, "read status");
  checkTrue(resp[0] == 0x11 && resp[1] == 0x22 && resp[2] == 0x33 && resp[3] == 0x44,
            "scripted response bytes copied into the caller's buffer");

  checkFalse(bus.clear(kAddr7), "postClear from a read() also holds the device");
  I2CBus::advanceClock(1500);
  checkTrue(bus.clear(kAddr7), "clear(0x10) true once the read's deadline elapses");
}

}  // namespace

int main() {
  scenarioDefaultsAreFree();
  scenarioPostClearHoldsUntilClockAdvances();
  scenario7BitVs8BitConvention();
  scenarioPreClearSpinAutoAdvances();
  scenarioReadRespectsClearanceAndData();

  if (g_failureCount == 0) {
    std::printf("OK: all I2CBus clearance-timer scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the I2CBus clearance-timer scenarios\n",
              g_failureCount);
  return 1;
}
