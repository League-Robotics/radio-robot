// nezha_flipflop_harness.cpp — off-hardware acceptance harness for ticket
// 079-004 (SUC-001/SUC-002/SUC-003/SUC-008/SUC-009): exercises the REAL
// Subsystems::NezhaHardware brick flip-flop sequencer + distribution role, and the
// REAL Hal::NezhaMotor split-phase encoder wiring, against ticket 001's
// HOST_BUILD scripted I2CBus fake — no MicroBitI2C, no CODAL, no wall
// clock, no real 4ms sleeps.
//
// Per the design sketch's "subsystem is the unit of test" principle (this
// is the confined, sanctioned hardware-fake exception — see
// architecture-update.md and motor_policy_harness.cpp's own header for the
// precedent), this compiles and links the ACTUAL source/hal/nezha/
// nezha_motor.cpp and subsystems/nezha_hardware.cpp against the SAME headers
// every ARM build compiles, with -DHOST_BUILD selecting i2c_bus_host.cpp's
// fake in place of the real MicroBitI2C-backed i2c_bus.cpp (see
// nezha_motor.cpp's own #ifndef HOST_BUILD guard for how it sheds its
// MicroBit.h dependency under this build). Mirrors motor_policy_harness.cpp/
// i2c_bus_clearance_harness.cpp's shape exactly: hand-rolled assertions,
// PASS/FAIL per scenario, nonzero exit on any failure.
//
// --- Why these scenarios can't inspect "which port" a bus transaction was
// for ---
// All four Nezha ports share ONE I2C device address (0x10) — the vendor
// frame's motorId byte (not the address) selects the channel. The
// HOST_BUILD scripted fake (i2c_bus_host.cpp) keys its per-device
// bookkeeping (txnCount/errCount/clear()) purely by ADDRESS, and does not
// record a write() call's payload bytes at all (see its write()'s
// `(void)data; (void)len;`). So a scripted scenario cannot distinguish "the
// HAL requested port 1's encoder" from "the HAL requested port 3's
// encoder" by inspecting the bus. Instead, these scenarios prove per-port
// scheduling behavior through each NezhaMotor OBJECT's own observable state
// (connected()/appliedDuty(), part of the public Hal::Motor faceplate) —
// only the port(s) the HAL's flip-flop actually calls requestSample()/
// tick() on ever transition those fields away from their construction-time
// defaults. An untouched port's connected()/appliedDuty() staying at its
// default is the proof "no bus transaction was ever scheduled for a port
// nobody addressed" (acceptance criteria) resolves to at this address-
// sharing tier.
//
// --- Scripting model note ---
// Every scenario below pre-loads a GENEROUS, uniform pool of scripted
// writes/reads (same address, status=OK, identical dummy payload) before
// driving any tick()/apply() calls. Since every entry is identical, WHICH
// entry a given call consumes is irrelevant — only the COUNT matters, and
// an under-scripted pool is self-detecting (an empty-queue "mismatch"
// still increments errCount() — see i2c_bus_host.cpp), so bus.errCount()
// == 0 at a scenario's end is the blanket proof nothing ran out. The
// PRECISE, scenario-specific proof is always a targeted assertion
// (txnCount() delta at an unambiguous point, connected(), or
// appliedDuty()) — see each scenario's comments.
//
// --- Two independent clocks ---
// NezhaHardware::tick(uint32_t now) takes a MILLISECOND "now" that only matters
// once collectEncoder() lands and NezhaMotor::tick() dispatches
// armoredWrite(duty, now) (078's reversal-dwell timing runs on this axis
// — requestSample() takes no "now" at all, so REQUEST_DUE ticks may pass
// any ms value). Separately, I2CBus's fake MICROSECOND clock
// (I2CBus::setClock()/advanceClock()) gates bus_.clear(kNezhaDeviceAddr)
// AND (via nezha_motor.cpp's HOST_BUILD system_timer_current_time_us()
// shim) NezhaMotor's own 40ms write-rate throttle. The dwell scenario
// advances the microsecond clock generously every cycle specifically so
// the throttle (a distinct, separately-tested mechanism) never
// incidentally gates the armor's own dwell-release write.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "com/i2c_bus.h"
#include "hal/nezha/nezha_motor.h"
#include "messages/motor.h"
#include "subsystems/nezha_hardware.h"

namespace {

// --- Hand-rolled assertion plumbing (see motor_policy_harness.cpp /
// i2c_bus_clearance_harness.cpp) ---

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

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s — expected %u, got %u", what.c_str(),
                  static_cast<unsigned>(expected),
                  static_cast<unsigned>(actual));
    fail(buf);
  }
}

void checkFloatEq(float actual, float expected, const std::string& what) {
  if (std::fabs(static_cast<double>(actual - expected)) > 1e-6) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s — expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

// --- Fixture helpers --------------------------------------------------

constexpr uint16_t kAddr7 = 0x10;                                   // bare 7-bit (clear()'s convention)
constexpr uint16_t kWireAddr = static_cast<uint16_t>(kAddr7 << 1);  // 0x20 (write()/read()'s convention)

msg::MotorConfig defaultConfigs[Subsystems::NezhaHardware::kMotorCount];

// resetDefaultConfigs -- 091-002: `polled` is now the CONFIGURED poll-set
// fed to NezhaHardware's constructor (polled_[], replacing the old
// command-derived portInUse_ flag) -- each scenario must therefore declare,
// up front, which port(s) it wants scheduled, rather than relying on a
// command to bring a port in (apply() no longer marks anything). `polledMask`
// bit i (0-based) means port i+1 starts polled=true; every other port
// defaults false -- mirrors gen_boot_config.py's own boot-config shape.
void resetDefaultConfigs(uint8_t polledMask = 0) {
  for (uint32_t i = 0; i < Subsystems::NezhaHardware::kMotorCount; ++i) {
    defaultConfigs[i] = msg::MotorConfig{};
    defaultConfigs[i].setPort(i + 1).setFwdSign(1).setTravelCalib(1.0f);
    defaultConfigs[i].setPolled((polledMask & (1u << i)) != 0);
  }
}

// Pre-loads `count` identical (address, status=OK) writes and (address,
// dummy-4-byte, status=OK) reads — see the file header's "Scripting model
// note". `count` should comfortably exceed the scenario's actual call
// count; harmless if it does (unconsumed entries just sit unused).
void scriptGenerousPool(I2CBus& bus, int count) {
  static uint8_t canned[4] = {0, 0, 0, 0};
  for (int i = 0; i < count; ++i) {
    bus.scriptWrite(kWireAddr, /*status=*/0);
    bus.scriptRead(kWireAddr, canned, 4, /*status=*/0);
  }
}

// Addresses a single port with `command` (non-broadcast) — the `DEV M <n>`
// shape (CommandProcessorToHardwareCommand, count=1).
Hal::CommandProcessorToHardwareCommand addressedOne(uint32_t port,
                                                const msg::MotorCommand& command) {
  Hal::CommandProcessorToHardwareCommand cmd;
  cmd.allPorts = false;
  cmd.count = 1;
  cmd.addressed[0].port = port;
  cmd.addressed[0].command = command;
  return cmd;
}

// Addresses two ports in ONE call (non-broadcast) — the `DEV DT STOP`
// shape (CommandProcessorToHardwareCommand, count=2, the bound pair).
Hal::CommandProcessorToHardwareCommand addressedTwo(uint32_t portA,
                                                const msg::MotorCommand& cmdA,
                                                uint32_t portB,
                                                const msg::MotorCommand& cmdB) {
  Hal::CommandProcessorToHardwareCommand cmd;
  cmd.allPorts = false;
  cmd.count = 2;
  cmd.addressed[0].port = portA;
  cmd.addressed[0].command = cmdA;
  cmd.addressed[1].port = portB;
  cmd.addressed[1].command = cmdB;
  return cmd;
}

msg::MotorCommand neutralCommand() {
  return msg::MotorCommand{}.setNeutral(msg::Neutral::COAST);
}

// One REQUEST_DUE + COLLECT_DUE pair for whichever port is currently the
// HAL's activePort_ (opaque to the caller — driven purely through tick()).
// Advances the I2C fake clock by exactly `postClearUs` between the two
// ticks so the collect's bus_.clear(kNezhaDeviceAddr) gate is satisfied
// (but not spun through, proving the request really armed a real
// deadline). `nowRequestMs`/`nowCollectMs` are the ms values passed to
// each tick() — irrelevant to REQUEST_DUE, load-bearing for COLLECT_DUE's
// armoredWrite()/dwell timing.
void runOneCycle(Subsystems::NezhaHardware& hal, uint32_t nowRequestMs,
                  uint32_t nowCollectMs, uint64_t postClearUs = 4000) {
  // (093/094 teardown) tick(now) no longer takes a motorIn[]/motorResetIn[]
  // pair -- see hardware.h's tick() doc comment. This harness exercises
  // the flip-flop scheduler purely via apply(), unaffected by this change.
  hal.tick(nowRequestMs);
  I2CBus::advanceClock(postClearUs);
  hal.tick(nowCollectMs);
}

// --- Scenarios ----------------------------------------------------------

// 1. Idle schedule (no port ever polled): tick() performs ZERO bus
//    actions, no matter how many times it's called (decision 1). 091-002:
//    constructed with polled=false for every port (the default) -- there is
//    no command that could bring a port into the schedule any more.
void scenarioIdleScheduleNoBusActions() {
  beginScenario("idle schedule (no port polled): tick() never touches the bus");
  resetDefaultConfigs(/*polledMask=*/0);
  I2CBus::setClock(1000000);
  I2CBus bus;
  Subsystems::NezhaHardware hal(bus, defaultConfigs);   // no port polled_ -- nothing ever scheduled


  for (uint32_t i = 0; i < 20; ++i) {
    hal.tick(100 + i);
  }

  checkUintEq(bus.txnCount(kAddr7), 0,
              "20 idle tick() calls performed zero I2C transactions");
}

// 2. Single in-use port: REQUEST_DUE -> (pass while the settle window is
//    still open) -> COLLECT_DUE, write-at-collect-only, and write-on-change
//    suppression on an unchanged repeat command. The "pass while unclear"
//    step is ALSO this ticket's required regression guard for
//    bus_.clear(kNezhaDeviceAddr) using the bare 7-bit address: if NezhaHardware
//    mistakenly called bus_.clear(kNezhaDeviceAddr << 1) (0x20) instead,
//    that queries a DeviceSlot NO real write()/read() ever populates (every
//    NezhaMotor transaction's 8-bit wire address collapses back to 7-bit
//    0x10 inside I2CBus — see i2c_bus_host.cpp's write()/read()), so
//    clear(0x20) would ALWAYS report true and the HAL would collect on the
//    very next tick regardless of the clock — an extra, unscripted-for
//    transaction this scenario's txnCount assertion below would catch.
void scenarioFlipFlopSequencingAndClearConvention() {
  beginScenario("flip-flop: request -> pass-while-unclear -> collect (7-bit clear() guard)");
  resetDefaultConfigs(/*polledMask=*/0b0001);   // 091-002: port 1 pre-polled -- no command brings it in any more
  I2CBus::setClock(1000000);
  I2CBus bus;
  Subsystems::NezhaHardware hal(bus, defaultConfigs);
  scriptGenerousPool(bus, 20);

  hal.apply(addressedOne(0, neutralCommand()));   // stages NEUTRAL on port 1's own setter (already polled)

  checkUintEq(bus.txnCount(kAddr7), 0, "apply() alone issues no bus traffic");


  hal.tick(1000);   // REQUEST_DUE: fires the 0x46 request
  checkUintEq(bus.txnCount(kAddr7), 1, "REQUEST_DUE issued exactly one transaction");

  hal.tick(1010);   // COLLECT_DUE attempt, clock NOT advanced -- must pass (no-op)
  checkUintEq(bus.txnCount(kAddr7), 1,
              "COLLECT_DUE before the settle window elapses performs zero additional "
              "transactions (also proves bus_.clear() uses the bare 7-bit address -- "
              "see this scenario's header comment)");

  I2CBus::advanceClock(4000);   // exactly the request's postClear
  hal.tick(1020);   // COLLECT_DUE, now clear: collects + dispatches (first NEUTRAL write)
  checkUintEq(bus.txnCount(kAddr7), 3,
              "collect landed: +1 read (collectEncoder) +1 write (first NEUTRAL "
              "dispatch, write-on-change never having seen this value before)");
  checkTrue(hal.motor(0).connected(), "port 1 reports connected() after a clean collect");
  checkFloatEq(hal.motor(0).appliedDuty(), 0.0f, "NEUTRAL dispatch wrote duty 0");

  hal.tick(1030);   // REQUEST_DUE again
  checkUintEq(bus.txnCount(kAddr7), 4, "second REQUEST_DUE issued one more transaction");

  I2CBus::advanceClock(4000);
  hal.tick(1040);   // COLLECT_DUE: collects, but NEUTRAL is unchanged -> no duty write
  checkUintEq(bus.txnCount(kAddr7), 5,
              "second collect: +1 read only -- the repeat NEUTRAL command is "
              "write-on-change-suppressed (write-at-collect-only, not write-every-collect)");

  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run across the whole sequence");
}

// 3. Two polled ports (091-002: pre-configured at construction, no longer
//    brought in by the addressed apply() call below), addressed via ONE
//    count=2 CommandProcessorToHardwareCommand (the `DEV DT STOP`-shaped
//    call): the flip-flop alternates strictly between them in
//    ascending-then-wrapping order, and the two UNPOLLED ports are never
//    touched at all.
void scenarioInUseTrackingAndRotation() {
  beginScenario("two pre-polled ports: strict rotation among polled ports only");
  resetDefaultConfigs(/*polledMask=*/0b0101);   // 091-002: ports 1 and 3 pre-polled -- the ports under test
  I2CBus::setClock(1000000);
  I2CBus bus;
  Subsystems::NezhaHardware hal(bus, defaultConfigs);
  scriptGenerousPool(bus, 40);

  hal.apply(addressedTwo(0, neutralCommand(), 2, neutralCommand()));   // stages NEUTRAL; ports already polled

  runOneCycle(hal, /*req=*/1, /*collect=*/2);
  checkTrue(hal.motor(0).connected(), "cycle A: port 1 (activePort_ defaults to 1) collects first");
  checkFalse(hal.motor(2).connected(), "cycle A: port 3 has not been reached yet");
  checkFalse(hal.motor(1).connected(), "port 2 is not polled -- untouched");
  checkFalse(hal.motor(3).connected(), "port 4 is not polled -- untouched");

  runOneCycle(hal, /*req=*/3, /*collect=*/4);
  checkTrue(hal.motor(2).connected(), "cycle B: rotation reached port 3 next (wrapping past unpolled port 2)");
  checkFalse(hal.motor(1).connected(), "port 2 still untouched");
  checkFalse(hal.motor(3).connected(), "port 4 still untouched");

  uint32_t txnBeforeC = bus.txnCount(kAddr7);
  runOneCycle(hal, /*req=*/5, /*collect=*/6);
  checkUintEq(bus.txnCount(kAddr7) - txnBeforeC, 2,
              "cycle C: rotation wrapped back to port 1 -- request+collect (no duty "
              "write, NEUTRAL unchanged) -- proves the cycle is 1,3,1,... not 1,2,3,4,...");

  checkFalse(hal.motor(1).connected(), "port 2 STILL never scheduled after 3 full cycles");
  checkFalse(hal.motor(3).connected(), "port 4 STILL never scheduled after 3 full cycles");
  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

// 4. Broadcast (allPorts=true, the `DEV STOP`/watchdog shape) leaves
//    polled_[] completely unaffected (091-002: apply() no longer touches
//    poll state in ANY branch, so there is nothing left to exempt broadcast
//    from) -- proven by zero bus activity across many hal.tick() calls
//    (every port still defaults unpolled), even though the command WAS
//    forwarded to every motor's setter (proven separately by directly
//    ticking one motor, bypassing the HAL's own scheduler entirely --
//    motor(port).tick() is part of the public Motor faceplate and is not
//    gated by poll-schedule membership, which is purely the HAL's own
//    internal scheduling concern).
void scenarioBroadcastNeverMarksInUse() {
  beginScenario("apply() broadcast: forwards to every setter, leaves polled_[] unaffected");
  resetDefaultConfigs(/*polledMask=*/0);
  I2CBus::setClock(1000000);
  I2CBus bus;
  Subsystems::NezhaHardware hal(bus, defaultConfigs);

  Hal::CommandProcessorToHardwareCommand broadcast;
  broadcast.allPorts = true;
  broadcast.addressed[0].command = neutralCommand();
  hal.apply(broadcast);


  for (uint32_t i = 0; i < 20; ++i) {
    hal.tick(100 + i);
  }
  checkUintEq(bus.txnCount(kAddr7), 0,
              "broadcast leaves polled_[] at its constructed (all-false) state -- "
              "20 tick() calls, zero bus activity");

  // Prove the broadcast's command really reached motor 1's setter: tick it
  // DIRECTLY (bypassing the HAL's scheduler entirely -- poll-schedule
  // membership gates only the HAL's OWN flip-flop, never the public Motor
  // faceplate).
  scriptGenerousPool(bus, 4);
  hal.motor(0).tick(200);
  checkFloatEq(hal.motor(0).appliedDuty(), 0.0f,
               "direct tick() proves the broadcast staged NEUTRAL on motor 1's setter");
}

// 5. DrivetrainToHardwareCommand (wheel[0]=left, wheel[1]=right): both wheels
//    are ALWAYS addressed (never a broadcast) and forwarded to their OWN
//    Hal::Motor setter. 091-002: this no longer has anything to do with
//    poll-schedule membership (apply() touches no poll state at all any
//    more) -- proven with BOTH ports left unpolled (constructed with
//    polledMask=0) and a DIRECT tick() on each motor (bypassing the HAL's
//    own scheduler entirely, exactly like scenarioBroadcastNeverMarksInUse's
//    own direct-tick() proof, above), which keeps this scenario's assertion
//    to forwarding alone -- whether/when the flip-flop schedules a port is
//    scenarioInUseTrackingAndRotation's concern, not this one's.
void scenarioDrivetrainToHardwareCommandForwarding() {
  beginScenario("apply(DrivetrainToHardwareCommand): both wheels' commands forwarded to their own setter");
  resetDefaultConfigs(/*polledMask=*/0);   // 091-002: forwarding is independent of polled_[]
  I2CBus::setClock(1000000);
  I2CBus bus;
  Subsystems::NezhaHardware hal(bus, defaultConfigs);
  scriptGenerousPool(bus, 8);

  Hal::DrivetrainToHardwareCommand dtCmd;
  dtCmd.wheel[0].port = 0;
  dtCmd.wheel[0].command = msg::MotorCommand{}.setDutyCycle(0.4f);
  dtCmd.wheel[1].port = 1;
  dtCmd.wheel[1].command = msg::MotorCommand{}.setDutyCycle(-0.4f);
  hal.apply(dtCmd);

  hal.motor(0).tick(200);
  hal.motor(1).tick(200);
  checkFloatEq(hal.motor(0).appliedDuty(), 0.4f, "left wheel (port 1) received its own forwarded command");
  checkFloatEq(hal.motor(1).appliedDuty(), -0.4f, "right wheel (port 2) received its own forwarded command");

  // Now prove apply() itself never touched polled_[]: the HAL's OWN
  // scheduler (hal.tick(), never called above -- only the direct
  // motor(port).tick() calls were) still performs zero bus actions for
  // EITHER wheel, even though both were just apply()'d.
  uint32_t txnBefore = bus.txnCount(kAddr7);
  for (uint32_t i = 0; i < 20; ++i) {
    hal.tick(300 + i);
  }
  checkUintEq(bus.txnCount(kAddr7) - txnBefore, 0,
              "apply(DrivetrainToHardwareCommand) left polled_[] unaffected -- the HAL's own "
              "flip-flop still performs zero bus actions for either wheel");
  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

// 6. The 40ms write-rate throttle interaction: a duty change delivered
//    within 40ms (fake-clock microseconds) of the PREVIOUS actual write is
//    suppressed at collect time; the same still-pending target succeeds
//    once >=40ms has elapsed. Uses appliedDuty() before/after each collect
//    as the ground truth for "did writeRawDuty() actually reach the bus
//    this cycle", independent of the exact slew-clamped value.
//
// 079-006 update: cycle 1 now targets 0.5 (not 0.9) and asserts the EXACT
// post-cycle-1 value. Before this ticket's sentinel-slew fix (scenario 8
// below), a fresh port's first write always landed exactly at the
// requested target regardless of magnitude, which is still true post-fix --
// what changed is that a first write to 0.9 now converges in ONE write
// (see scenario 8), leaving nothing left to throttle in cycles 2/3. Using
// 0.5 then RETARGETING to 0.9 before cycle 2 keeps this scenario's actual
// subject (the throttle, on a genuinely still-converging SECOND write)
// intact and independent of scenario 8's fix.
void scenarioWriteThrottleInteraction() {
  beginScenario("40ms write-rate throttle gates collect-time duty writes, not requests");
  resetDefaultConfigs(/*polledMask=*/0b0001);   // 091-002: port 1 pre-polled -- needed for the flip-flop to schedule it
  uint64_t t0 = 1000000;
  I2CBus::setClock(t0);
  I2CBus bus;
  Subsystems::NezhaHardware hal(bus, defaultConfigs);
  scriptGenerousPool(bus, 40);

  hal.apply(addressedOne(0, msg::MotorCommand{}.setDutyCycle(0.5f)));

  // Cycle 1: lastWriteTimeUs_ defaults to 0, so "now - 0" is astronomically
  // over 40ms regardless of the fake clock's absolute value -- the first
  // write always goes through, landing exactly at the requested 0.5 (the
  // -128 sentinel is exempted from the slew clamp -- scenario 8).
  runOneCycle(hal, 0, 1);
  float afterCycle1 = hal.motor(0).appliedDuty();
  checkFloatEq(afterCycle1, 0.5f, "cycle 1: first-ever write reaches the full requested duty");

  // Retarget to 0.9 -- no longer the first write, so THIS one is genuinely
  // slew-clamped (|0.9-0.5| step of 40 > the 25 maxDelta), giving cycles 2/3
  // a real not-yet-converged value to test the throttle against.
  hal.apply(addressedOne(0, msg::MotorCommand{}.setDutyCycle(0.9f)));

  // Cycle 2: only the request/collect's own postClear (4000us) elapses
  // since cycle 1's write -- well under 40000us -- so the throttle
  // suppresses this collect's write even though the target hasn't
  // converged yet (write-on-change would otherwise allow it).
  runOneCycle(hal, 2, 3, /*postClearUs=*/4000);
  float afterCycle2 = hal.motor(0).appliedDuty();
  checkFloatEq(afterCycle2, afterCycle1,
               "cycle 2 (only ~4ms since the last write): throttled -- appliedDuty() unchanged");

  // Cycle 3: advance the fake clock well past the 40ms mark since cycle 1's
  // write -- the still-unconverged 0.9 target now gets through (slew-capped
  // toward it, not landing exactly at 0.9 yet).
  runOneCycle(hal, 4, 5, /*postClearUs=*/50000);
  float afterCycle3 = hal.motor(0).appliedDuty();
  checkTrue(afterCycle3 != afterCycle2,
            "cycle 3 (>=40ms since the last write): throttle cleared -- a new write landed");

  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

// 7. 078's reversal-dwell armor still holds correctly when driven through
//    the flip-flop's COLLECT_DUE dispatch, at the exact ms boundary values
//    motor_policy_harness.cpp's own hot-sign-flip scenario uses (100ms
//    default reversalDwell_): immediate zero on the flip, held through the
//    dwell (including the "1ms short" boundary), released exactly at the
//    deadline. The fake MICROSECOND clock is advanced generously every
//    cycle so the (separately-tested) write-rate throttle never
//    incidentally gates the dwell's own release write.
void scenarioReversalDwellHoldsAtNewCadence() {
  beginScenario("078's reversal dwell holds through the flip-flop's new collect cadence");
  resetDefaultConfigs(/*polledMask=*/0b0001);   // 091-002: port 1 pre-polled
  I2CBus::setClock(10000000);
  I2CBus bus;
  Subsystems::NezhaHardware hal(bus, defaultConfigs);
  scriptGenerousPool(bus, 40);
  const uint64_t kUsGap = 50000;   // >> 40ms throttle, >> 4ms postClear -- isolates the dwell

  hal.apply(addressedOne(0, msg::MotorCommand{}.setDutyCycle(0.5f)));
  runOneCycle(hal, 500, 1000, kUsGap);
  checkTrue(hal.motor(0).appliedDuty() != 0.0f, "cycle 1 (ms=1000): initial direction forwarded");

  hal.apply(addressedOne(0, msg::MotorCommand{}.setDutyCycle(-0.5f)));   // sign flip
  runOneCycle(hal, 1005, 1010, kUsGap);
  checkFloatEq(hal.motor(0).appliedDuty(), 0.0f, "cycle 2 (ms=1010): reversal writes 0 immediately, arms the dwell");

  runOneCycle(hal, 1040, 1050, kUsGap);
  checkFloatEq(hal.motor(0).appliedDuty(), 0.0f, "cycle 3 (ms=1050): still mid-dwell, held at 0");

  runOneCycle(hal, 1100, 1109, kUsGap);
  checkFloatEq(hal.motor(0).appliedDuty(), 0.0f, "cycle 4 (ms=1109, one ms short of the 100ms deadline): still held at 0");

  runOneCycle(hal, 1109, 1110, kUsGap);
  checkTrue(hal.motor(0).appliedDuty() < 0.0f,
            "cycle 5 (ms=1110, dwell elapsed): new (negative) direction finally forwarded");

  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

// 8. 079-006 stand-campaign root-cause regression #1: the very first duty
//    write for a freshly-constructed motor must NOT slew-clamp from
//    writeRawDuty()'s -128 "no write yet" sentinel. MotorSlew::clampStep()
//    has no concept of that sentinel: fed -128 unconditionally (as every
//    prior sprint's own comment documented as intentional, ported-unchanged
//    behavior), clampStep(-128, 30, 25) returns -103 -- a WRONG-SIGN,
//    out-of-range (the Nezha 0x60 register's speed byte is documented 0-100)
//    intermediate write, i.e. an unrequested full-swing reversal as the
//    very first command ever sent to a fresh port. Confirmed on hardware
//    (079-006 stand campaign) as a real trigger for
//    docs/knowledge/2026-07-04-encoder-wedge.md's reversal-write-train
//    latch. Existing scenario 6 above only asserted "!= 0.0f" after the
//    first write, which the pre-fix -1.03 value also satisfies -- this is
//    why the bug went uncaught through 077/078/079-004/005; this scenario
//    asserts the actual value.
void scenarioFirstWriteExemptFromSentinelSlew() {
  beginScenario("first-ever duty write skips the -128 sentinel's slew clamp (079-006 root cause)");
  resetDefaultConfigs(/*polledMask=*/0b0001);   // 091-002: port 1 pre-polled
  I2CBus::setClock(1000000);
  I2CBus bus;
  Subsystems::NezhaHardware hal(bus, defaultConfigs);
  scriptGenerousPool(bus, 20);

  hal.apply(addressedOne(0, msg::MotorCommand{}.setDutyCycle(0.30f)));
  runOneCycle(hal, 1000, 1010, /*postClearUs=*/50000);

  checkFloatEq(hal.motor(0).appliedDuty(), 0.30f,
               "first-ever write reaches the FULL requested duty directly -- before the "
               "fix this computed clampStep(-128, 30, 25) = -103 (appliedDuty() = -1.03): "
               "wrong sign, magnitude > 1.0");

  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

// 9. 079-006 stand-campaign root-cause regression #2: the encoder request
//    (requestEncoder()'s 0x46 write, preClear=4000) and the duty write
//    (writeMotorRun()'s 0x60 write, postClear=4000) together guarantee a
//    real >=4ms gap around every 0x10 transaction, mirroring the
//    always->=4ms-apart transactions the OLD fused/blocking readEncoderSettle()
//    implicitly had. Before this fix, requestEncoder() carried no preClear
//    and writeMotorRun() carried no postClear at all, so a single in-use
//    port's own request-collect-write-request cycle could re-issue the next
//    0x46 request with ~0us real gap since the immediately-preceding duty
//    write -- confirmed on hardware as the trigger for a severe (multi-
//    second) NRF52I2C::waitForStop() TWIM stall (vendor CODAL driver,
//    libraries/codal-nrf52/source/NRF52I2C.cpp) once a fresh port was
//    actually driven with DUTY, not just addressed with a no-op command.
void scenarioRequestHonorsClearanceAfterDutyWrite() {
  beginScenario("079-006 root-cause fix: request/duty writes keep >=4ms real clearance around 0x10");
  resetDefaultConfigs(/*polledMask=*/0b0001);   // 091-002: port 1 pre-polled
  I2CBus::setClock(1000000);
  I2CBus bus;
  Subsystems::NezhaHardware hal(bus, defaultConfigs);
  scriptGenerousPool(bus, 40);

  hal.apply(addressedOne(0, msg::MotorCommand{}.setDutyCycle(0.30f)));


  hal.tick(1000);                // REQUEST_DUE
  I2CBus::advanceClock(4000);    // satisfy the request's own postClear
  hal.tick(1010);                // COLLECT_DUE: collects + dispatches the first duty write
  checkTrue(hal.motor(0).appliedDuty() != 0.0f, "duty write landed at collect");

  uint64_t clockBefore = I2CBus::clock();
  hal.tick(1020);                // next REQUEST_DUE -- deliberately NO manual clock advance
  uint64_t clockAfter = I2CBus::clock();

  checkTrue(clockAfter - clockBefore >= 4000,
            "the next 0x46 request's entry spin held for >=4000us of real (fake-clock) "
            "time since the preceding duty write -- before this fix that gap was ~0us, "
            "the observed hardware TWIM-stall trigger");

  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

// 10. 088-008: MotorConfig::fwd_sign genuinely negates NezhaMotor's own
//     reported encoder-position sign -- the sim-side, REAL-HAL proof of the
//     mirror-mounted wheel-direction fix (088-002) that
//     test_gen_boot_config_fwd_sign.py's own module docstring explicitly
//     disclaims trying to attempt ("the sim plant does not model physical
//     wheel mounting"): that file only proves the generator emits the
//     correct fwd_sign VALUE into the generated boot config, never that any
//     HAL object actually consumes it. It cannot, either -- confirmed by
//     inspection of source/hal/sim/sim_motor.cpp, Hal::SimMotor never reads
//     config_.fwd_sign anywhere (tick()/writeRawDuty()/encoderPosition() all
//     omit it); only the REAL Hal::NezhaMotor leaf exercised by THIS harness
//     consumes it, in both writeMotorRun()'s direction-byte selection (not
//     independently observable here -- the HOST_BUILD I2CBus fake discards
//     write() payload bytes entirely, see this file's own header note) and
//     position()'s decode (nezha_motor.cpp's tick(): "pos = (raw/10) *
//     travel_calib * fwd_sign" -- fully observable via position()).
//
//     Constructs two STANDALONE NezhaMotor objects (bypassing
//     NezhaHardware's flip-flop scheduler entirely -- unneeded for a single
//     collectEncoder()+tick() call), each on its OWN scripted I2CBus,
//     differing ONLY in fwd_sign (+1 / -1); scripts the IDENTICAL raw
//     encoder register bytes (1000 tenths-of-degree) for both. One tick()
//     each is enough for tick()'s own step-2 collectEncoder()+position()
//     conversion to run. The two motors must report EXACTLY opposite
//     position() signs from the identical underlying hardware reading --
//     proof fwd_sign is not merely a stored, inert config value on the leaf
//     that actually drives the physical wheels.
void scenarioFwdSignNegatesEncoderPositionSign() {
  beginScenario("088-008: fwd_sign negates NezhaMotor's reported encoder-position sign");
  I2CBus::setClock(1000000);

  I2CBus busPos;
  I2CBus busNeg;

  msg::MotorConfig cfgPos = msg::MotorConfig{}.setPort(1).setFwdSign(1).setTravelCalib(1.0f);
  msg::MotorConfig cfgNeg = msg::MotorConfig{}.setPort(1).setFwdSign(-1).setTravelCalib(1.0f);

  Hal::NezhaMotor motorPos(busPos, cfgPos);
  Hal::NezhaMotor motorNeg(busNeg, cfgNeg);

  // raw = 1000 (tenths of a degree), little-endian int32 -- resp[0] is the
  // LSB (see nezha_motor.cpp's collectEncoder()). Identical bytes scripted
  // on BOTH motors' own independent bus.
  uint8_t rawEnc[4] = {0xE8, 0x03, 0x00, 0x00};   // 1000 == 0x000003E8
  busPos.scriptRead(kWireAddr, rawEnc, 4, /*status=*/0);
  busNeg.scriptRead(kWireAddr, rawEnc, 4, /*status=*/0);

  motorPos.tick(1000);
  motorNeg.tick(1000);

  // mm = (raw/10) * travel_calib * fwd_sign = (1000/10) * 1.0 * fwd_sign = 100 * fwd_sign.
  checkFloatEq(motorPos.position(), 100.0f,
               "fwd_sign=+1: raw=1000 tenths-deg -> position=+100mm (same-sign passthrough)");
  checkFloatEq(motorNeg.position(), -100.0f,
               "fwd_sign=-1: the IDENTICAL raw encoder reading -> position=-100mm (negated) -- "
               "proves fwd_sign flips the reported encoder-position sign, the sim-side/real-HAL "
               "proof of the mirror-mounted wheel-direction fix (088-002) beyond the generator/"
               "config-value-only check test_gen_boot_config_fwd_sign.py adds");

  checkUintEq(busPos.errCount(kAddr7), 0, "no script under-run on the +1 motor's bus");
  checkUintEq(busNeg.errCount(kAddr7), 0, "no script under-run on the -1 motor's bus");
}

}  // namespace

int main() {
  scenarioIdleScheduleNoBusActions();
  scenarioFlipFlopSequencingAndClearConvention();
  scenarioInUseTrackingAndRotation();
  scenarioBroadcastNeverMarksInUse();
  scenarioDrivetrainToHardwareCommandForwarding();
  scenarioWriteThrottleInteraction();
  scenarioReversalDwellHoldsAtNewCadence();
  scenarioFirstWriteExemptFromSentinelSlew();
  scenarioRequestHonorsClearanceAfterDutyWrite();
  scenarioFwdSignNegatesEncoderPositionSign();

  if (g_failureCount == 0) {
    std::printf("OK: all NezhaHardware flip-flop scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the NezhaHardware flip-flop scenarios\n",
              g_failureCount);
  return 1;
}
