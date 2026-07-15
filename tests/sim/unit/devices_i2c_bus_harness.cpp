// devices_i2c_bus_harness.cpp — off-hardware acceptance harness for ticket
// DB-003 (device-bus-tickets.md): exercises Devices::I2CBus's lazy
// per-device clearance timers (DeviceSlot.lastEnd/readyAt, write()/read()'s
// preClear/postClear, the non-spinning clear() peek), scripted write/read
// FIFO ordering, per-device txn/err counters, and the IRQ-guard default-on
// policy — through the HOST_BUILD scripted fake (source/devices/
// i2c_bus_host.cpp) — no MicroBitI2C, no CODAL, no wall clock.
//
// Adapted from tests/sim/unit/i2c_bus_clearance_harness.cpp (079-001's
// SUC-008/SUC-009 acceptance harness for the pre-port source/com/i2c_bus.*)
// against the ported source/devices/i2c_bus.h/i2c_bus_host.cpp — same
// scenarios, plus new ones DB-003's acceptance criteria call out that the
// original harness didn't cover on its own: multi-entry FIFO ordering,
// per-device counters distinguishing two different addresses, and the
// IRQ-guard default-on assertion.
//
// Plain C++ program, hand-rolled assertions — mirrors every other
// tests/sim/unit harness's shape: prints a PASS/FAIL line per scenario and
// exits nonzero if any assertion failed, run by the pytest wrapper in
// test_devices_i2c_bus.py.

#include <cstdint>
#include <cstdio>
#include <string>

#include "devices/i2c_bus.h"

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

void checkU32Eq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s — expected %u, got %u", what.c_str(),
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
constexpr uint16_t kAddr7 = 0x10;                                     // Nezha motor
constexpr uint16_t kWireAddr = static_cast<uint16_t>(kAddr7 << 1);    // 0x20
constexpr uint16_t kAddr7B = 0x17;                                    // OTOS
constexpr uint16_t kWireAddrB = static_cast<uint16_t>(kAddr7B << 1);  // 0x2E

// --- Scenarios --------------------------------------------------------

// 1. Defaults (preClear=postClear=0) behave exactly like the pre-clearance-
//    timer bus: no spin (the fake clock, read via I2CBus::clock(), is
//    unchanged across the call — a live spin would auto-advance it) and the
//    scripted status is returned immediately.
void scenarioDefaultsAreFree() {
  beginScenario("defaults (preClear=postClear=0): no spin, immediate status");
  Devices::I2CBus::setClock(1000);
  Devices::I2CBus bus;
  bus.scriptWrite(kWireAddr, /*status=*/0);

  uint64_t before = Devices::I2CBus::clock();
  int status = bus.write(kWireAddr, dummyFrame, 8, false);
  uint64_t after = Devices::I2CBus::clock();

  checkIntEq(status, 0, "write(): scripted status returned unchanged");
  checkU64Eq(after, before, "write(): fake clock did not advance — no spin occurred");

  uint8_t canned[4] = {0xAA, 0xBB, 0xCC, 0xDD};
  uint8_t resp[4] = {0, 0, 0, 0};
  bus.scriptRead(kWireAddr, canned, 4, /*status=*/0);

  uint64_t beforeRead = Devices::I2CBus::clock();
  int readStatus = bus.read(kWireAddr, resp, 4, false);
  uint64_t afterRead = Devices::I2CBus::clock();

  checkIntEq(readStatus, 0, "read(): scripted status returned unchanged");
  checkU64Eq(afterRead, beforeRead, "read(): fake clock did not advance — no spin occurred");
}

// 2. A write's postClear holds the DEVICE (not just the call site) until
//    the clock clears the deadline — clear()'s peek reflects it, and
//    reflects it correctly once the test advances the clock past the
//    deadline (lastEnd = now, readyAt = lastEnd + postClear).
void scenarioPostClearHoldsUntilClockAdvances() {
  beginScenario("postClear holds the device until the clock clears the deadline");
  Devices::I2CBus::setClock(5000);
  Devices::I2CBus bus;
  bus.scriptWrite(kWireAddr, /*status=*/0);

  int status = bus.write(kWireAddr, dummyFrame, 8, false,
                          /*preClear=*/0, /*postClear=*/4000);
  checkIntEq(status, 0, "write status");

  checkFalse(bus.clear(kAddr7), "clear(0x10) false immediately after the write");

  Devices::I2CBus::advanceClock(3999);
  checkFalse(bus.clear(kAddr7), "clear(0x10) still false 1us short of the deadline");

  Devices::I2CBus::advanceClock(1);
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
  Devices::I2CBus::setClock(9000);
  Devices::I2CBus bus;
  bus.scriptWrite(kWireAddr, /*status=*/0);
  bus.write(kWireAddr, dummyFrame, 8, false, /*preClear=*/0, /*postClear=*/4000);

  bool clearBareAddr = bus.clear(kAddr7);     // 0x10 — correct convention
  bool clearWireAddr = bus.clear(kWireAddr);  // 0x20 — the wrong-bit mistake

  checkFalse(clearBareAddr, "clear(0x10): the real device, still held");
  checkTrue(clearWireAddr, "clear(0x20): an untouched slot, always clear");
  checkTrue(clearBareAddr != clearWireAddr,
            "clear(0x10) and clear(0x20) must disagree — guards the bit convention");
}

// 4. A live entry spin (preClear whose deadline is still in the future)
//    self-advances the fake clock until the deadline is met, rather than
//    hanging, and the resulting transaction still returns its own scripted
//    status.
void scenarioPreClearSpinAutoAdvances() {
  beginScenario("a live preClear spin self-advances the fake clock and completes");
  Devices::I2CBus::setClock(0);
  Devices::I2CBus bus;
  bus.scriptWrite(kWireAddr, /*status=*/0);  // first write: arms postClear
  bus.scriptWrite(kWireAddr, /*status=*/0);  // second write: gated by preClear

  bus.write(kWireAddr, dummyFrame, 8, false, /*preClear=*/0, /*postClear=*/4000);
  uint64_t readyAt = Devices::I2CBus::clock() + 4000;  // lastEnd (== clock() here) + postClear

  // No advanceClock() call here — the second write's preClear=4000 entry
  // spin must clear the deadline on its own.
  int status2 = bus.write(kWireAddr, dummyFrame, 8, false,
                           /*preClear=*/4000, /*postClear=*/0);

  checkIntEq(status2, 0, "second (spin-gated) write still returns its scripted status");
  checkTrue(Devices::I2CBus::clock() >= readyAt,
            "fake clock landed at/after the deadline once the spin self-advanced it");
}

// 5. read() honors the same clearance contract and copies scripted response
//    bytes into the caller's buffer — the collectEncoder()-style path.
void scenarioReadRespectsClearanceAndData() {
  beginScenario("read() honors preClear/postClear and copies scripted bytes");
  Devices::I2CBus::setClock(20000);
  Devices::I2CBus bus;
  uint8_t canned[4] = {0x11, 0x22, 0x33, 0x44};
  bus.scriptRead(kWireAddr, canned, 4, /*status=*/0);

  uint8_t resp[4] = {0, 0, 0, 0};
  int status = bus.read(kWireAddr, resp, 4, false, /*preClear=*/0, /*postClear=*/1500);
  checkIntEq(status, 0, "read status");
  checkTrue(resp[0] == 0x11 && resp[1] == 0x22 && resp[2] == 0x33 && resp[3] == 0x44,
            "scripted response bytes copied into the caller's buffer");

  checkFalse(bus.clear(kAddr7), "postClear from a read() also holds the device");
  Devices::I2CBus::advanceClock(1500);
  checkTrue(bus.clear(kAddr7), "clear(0x10) true once the read's deadline elapses");
}

// 6. Scripted write/read FIFO ordering: multiple scripted entries for the
//    SAME address are consumed strictly in the order they were queued, and
//    a call whose address doesn't match the head-of-queue entry reports the
//    mismatch status rather than silently matching a later entry.
void scenarioFifoOrdering() {
  beginScenario("scriptWrite()/scriptRead() are consumed strictly FIFO");
  Devices::I2CBus::setClock(30000);
  Devices::I2CBus bus;

  bus.scriptWrite(kWireAddr, /*status=*/0);   // 1st: OK
  bus.scriptWrite(kWireAddr, /*status=*/-5);  // 2nd: a distinct error status
  bus.scriptWrite(kWireAddr, /*status=*/0);   // 3rd: OK again

  int s1 = bus.write(kWireAddr, dummyFrame, 8, false);
  int s2 = bus.write(kWireAddr, dummyFrame, 8, false);
  int s3 = bus.write(kWireAddr, dummyFrame, 8, false);

  checkIntEq(s1, 0, "1st scripted write returns its own status");
  checkIntEq(s2, -5, "2nd scripted write returns its own (different) status");
  checkIntEq(s3, 0, "3rd scripted write returns its own status — order preserved");

  // A read scripted for a DIFFERENT address than what's actually called
  // reports the mismatch status rather than matching regardless of address.
  uint8_t canned[2] = {0x01, 0x02};
  bus.scriptRead(kWireAddrB, canned, 2, /*status=*/0);  // scripted for device B
  uint8_t resp[2] = {0, 0};
  int mismatchStatus = bus.read(kWireAddr, resp, 2, false);  // called for device A
  checkTrue(mismatchStatus != 0,
            "read() against a wrong-address scripted entry reports a mismatch, not OK");
}

// 7. Per-device txn/err counters: two distinct devices accumulate
//    independent txnCount()/errCount()/lastErr() — a failure on one device
//    must not perturb the other's counters.
void scenarioPerDeviceCounters() {
  beginScenario("txnCount()/errCount()/lastErr() are tracked independently per device");
  Devices::I2CBus::setClock(40000);
  Devices::I2CBus bus;

  bus.scriptWrite(kWireAddr, /*status=*/0);    // device A: 1 OK
  bus.scriptWrite(kWireAddr, /*status=*/0);    // device A: 2 OK
  bus.scriptWrite(kWireAddrB, /*status=*/-7);  // device B: 1 error

  bus.write(kWireAddr, dummyFrame, 8, false);
  bus.write(kWireAddr, dummyFrame, 8, false);
  bus.write(kWireAddrB, dummyFrame, 8, false);

  checkU32Eq(bus.txnCount(kAddr7), 2, "device A txnCount == 2");
  checkU32Eq(bus.errCount(kAddr7), 0, "device A errCount == 0 (no failures)");
  checkIntEq(bus.lastErr(kAddr7), 0, "device A lastErr == 0 (no failures)");

  checkU32Eq(bus.txnCount(kAddr7B), 1, "device B txnCount == 1");
  checkU32Eq(bus.errCount(kAddr7B), 1, "device B errCount == 1");
  checkIntEq(bus.lastErr(kAddr7B), -7, "device B lastErr == the scripted failure status");
}

// 8. IRQ-guard default-on: a freshly constructed I2CBus reports irqGuard()
//    true WITHOUT any caller opting in — the nRF52 TWIM errata mitigation
//    (issue "Armor stays intact") is non-negotiable and must never require
//    an explicit enable. setIrqGuard() still toggles it live for bench A/B.
void scenarioIrqGuardDefaultOn() {
  beginScenario("IRQ guard defaults ON; setIrqGuard() toggles it live");
  Devices::I2CBus bus;

  checkTrue(bus.irqGuard(), "irqGuard() is true immediately after construction");

  bus.setIrqGuard(false);
  checkFalse(bus.irqGuard(), "setIrqGuard(false) turns the guard off");

  bus.setIrqGuard(true);
  checkTrue(bus.irqGuard(), "setIrqGuard(true) turns the guard back on");
}

// 9. 103-002 (M1 fix, 2026-07-13 code review): a call that arrives BEFORE
//    a device's clearance deadline bumps clearanceSafetyNetCount() exactly
//    once per early call, and the fake clock still lands at/after the
//    deadline (the HOST_BUILD fork's own "sleep the shortfall in one jump,
//    never a spin loop" mechanism -- see i2c_bus_host.cpp's own
//    waitForClearance()). A call that is ALREADY clear never bumps the
//    counter -- the safety net only trips when it's actually needed.
void scenarioEarlyCallBumpsClearanceSafetyNetCounter() {
  beginScenario("an early call bumps clearanceSafetyNetCount() and lands at/after the deadline, never spinning");
  Devices::I2CBus::setClock(50000);
  Devices::I2CBus bus;
  checkU32Eq(bus.clearanceSafetyNetCount(), 0, "counter starts at 0");

  bus.scriptWrite(kWireAddr, /*status=*/0);   // motor1-shaped duty write: arms postClear
  bus.scriptWrite(kWireAddr, /*status=*/0);   // motor2-shaped request write: arrives early

  bus.write(kWireAddr, dummyFrame, 8, false, /*preClear=*/0, /*postClear=*/4000);
  checkU32Eq(bus.clearanceSafetyNetCount(), 0,
             "the FIRST write (nothing stamped yet) never trips the safety net");
  uint64_t readyAt = Devices::I2CBus::clock() + 4000;   // lastEnd (== clock() here) + postClear

  // No advanceClock() call -- the second write arrives immediately, well
  // before readyAt (the exact "motor1 write -> motor2 request" hot site the
  // 2026-07-13 review's M1 finding flagged).
  int status2 = bus.write(kWireAddr, dummyFrame, 8, false,
                           /*preClear=*/4000, /*postClear=*/0);

  checkIntEq(status2, 0, "the early (safety-net-tripped) write still returns its scripted status");
  checkU32Eq(bus.clearanceSafetyNetCount(), 1,
             "exactly one safety-net trip recorded for the one early call");
  checkTrue(Devices::I2CBus::clock() >= readyAt,
            "fake clock landed at/after the deadline once the shortfall was 'slept'");

  // A THIRD call, already clear (clock is now >= its own trivial deadline),
  // must NOT bump the counter again.
  bus.scriptWrite(kWireAddr, /*status=*/0);
  bus.write(kWireAddr, dummyFrame, 8, false, /*preClear=*/0, /*postClear=*/0);
  checkU32Eq(bus.clearanceSafetyNetCount(), 1,
             "a call that is already clear does not trip the safety net");
}

}  // namespace

int main() {
  scenarioDefaultsAreFree();
  scenarioPostClearHoldsUntilClockAdvances();
  scenario7BitVs8BitConvention();
  scenarioPreClearSpinAutoAdvances();
  scenarioReadRespectsClearanceAndData();
  scenarioFifoOrdering();
  scenarioPerDeviceCounters();
  scenarioIrqGuardDefaultOn();
  scenarioEarlyCallBumpsClearanceSafetyNetCounter();

  if (g_failureCount == 0) {
    std::printf("OK: all Devices::I2CBus scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Devices::I2CBus scenarios\n",
              g_failureCount);
  return 1;
}
