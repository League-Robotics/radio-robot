// sim_ctypes.cpp -- extern "C" C ABI over TestSim::SimHarness/TestSim::SimPlant.
//
// Sprint 108 ticket 005 (clasi/issues/plan-pure-i2cbus-clock-interfaces-a-
// real-simplant-simulator.md, Stage 3 part a; supersedes and FULLY closes
// clasi/issues/sim-api-ctypes-abi-for-sim-mode-tours.md -- that issue
// originally scoped the ABI over the older, now-deleted
// tests/sim/support/sim_api.h `SimApi`; this file targets the NEW
// SimHarness/SimPlant composition (ticket 108-002/108-003), which is the
// thing that now exists).
//
// Every export below is a THIN CALL-THROUGH -- no decision logic, no
// protocol/physics reasoning of its own. That logic lives entirely in
// SimHarness (composition + stepping/injection/drain) and SimPlant (wire
// protocol + fault knobs + hook dispatch). This file exists ONLY to give a
// ctypes-callable, C-linkage shape to those two classes' public C++ API, so
// ticket 006's Python `CFUNCTYPE`/`CDLL` wrapper (`sim_loop.py`) can drive a
// simulated robot without a Python<->C++ binding generator.
//
// ---- Handle lifecycle ----
// A `SimHandle` is an opaque `void*` -- actually a `TestSim::SimHarness*`,
// heap-allocated by sim_create() and freed by sim_destroy(). Never pass a
// handle to a call after destroying it (use-after-free, same as any other
// C API); never leak one (call sim_destroy() when done).
//
//   SimHandle sim_create(float trackWidth);
//     Constructs a SimHarness (trackWidth <= 0 uses SimHarness's own
//     default, TestSim::kDefaultTrackWidth) and immediately calls boot() --
//     callers never see a pre-boot handle, there is no separate C-side
//     boot export because there is nothing useful a caller could do with
//     an unbooted harness before stepping it anyway.
//   void sim_destroy(SimHandle h);
//   int sim_booted(SimHandle h);       // 1/0
//   int sim_cycle_count(SimHandle h);  // total robotLoop_.cycle() calls so far
//
// ---- Stepping ----
//   void sim_step(SimHandle h, int cycles);
//     cycles < 1 is a no-op (SimHarness::step()'s own loop guard).
//
// ---- Command injection ----
//   void sim_inject_twist(SimHandle h, float v_x, float omega, float duration, uint32_t corr);
//   void sim_inject_stop(SimHandle h, uint32_t corr);
//   void sim_inject_command(SimHandle h, const char* armoredLine);
//     Raw, non-actuation escape hatch -- pushes ANY already-armored ("*B...")
//     line straight onto the inbound FakeTransport, for tests that need a
//     wire shape sim_inject_twist()/sim_inject_stop() don't cover.
//
// ---- Telemetry drain ----
//   int sim_drain_tlm(SimHandle h, char* buf, int buflen);
//     Drains every raw (still-armored "*B...") outbound line captured since
//     the LAST sim_drain_tlm() call on this handle, newline-joins them, and
//     copies up to buflen-1 bytes plus a NUL terminator into buf (buf may be
//     NULL / buflen may be 0 to just drain-and-discard). Returns the number
//     of bytes the FULL joined string would occupy, NOT counting the NUL --
//     exactly like snprintf()'s own return-value convention, so a caller can
//     detect truncation (return value >= buflen) and knows how big a buffer
//     to retry with. NOTE: the drain always advances regardless of whether
//     buf was big enough -- a caller that truncates has still consumed
//     those lines; pass a buffer sized generously (a handful of KB comfortably
//     covers a burst of frames from one step() call) to avoid this in
//     practice. The lines returned are RAW wire text -- this file does not
//     dearmor or decode them; the Python side does that with the exact same
//     pb2 codec a real robot's replies go through (host/robot_radio/robot/pb2).
//
// ---- True pose ----
//   float sim_true_x(SimHandle h);  // [mm]
//   float sim_true_y(SimHandle h);  // [mm]
//   float sim_true_h(SimHandle h);  // [rad]
//     SimPlant's owned OtosPlant ground truth (SimHarness::trueX/Y/Heading())
//     -- bypasses OTOS drift/noise fault knobs entirely; see sim_harness.h's
//     own header for why these three are "the" true pose.
//
// ---- Fault-condition setters ----
// Thin call-throughs to SimPlant's own knobs (sim_plant.h). port: 1 = left
// (Nezha motorId 1), 2 = right (motorId 2) -- same numbering the real wire
// frame's byte [2] carries.
//   void sim_set_wheel_disconnected(SimHandle h, int port, int disconnected);  // 1/0
//   void sim_set_wheel_freeze(SimHandle h, int port, int freeze);              // 1/0
//   void sim_set_wheel_dropout_rate(SimHandle h, int port, float fraction);    // [0,1]
//   void sim_set_otos_drift(SimHandle h, float xDrift, float yDrift, float headingDrift);  // [mm][mm][rad]
//
// ---- Hook surface -- THE point of this sprint's scripting model ----
// (master plan's Target architecture, verbatim; see sim_plant.h's own
// "Intended ctypes bridge" comment, which this file implements exactly as
// documented there.)
//
//   typedef int (*SimHookFn)(void* ctx, uint16_t addr, uint8_t* data, int len);
//
//   void sim_set_read_hook(SimHandle h, SimHookFn cb, void* ctx);
//   void sim_set_write_hook(SimHandle h, SimHookFn cb, void* ctx);
//     Registers cb (a Python ctypes.CFUNCTYPE-wrapped callback) + an opaque
//     ctx pointer as SimPlant's read/write hook. cb == NULL CLEARS the hook
//     (SimPlant::clearReadHook()/clearWriteHook()) -- back to always calling
//     the default protocol handler.
//
//     Callback contract: cb(ctx, addr, data, len) is invoked in place of
//     SimPlant's own default handler for EVERY read()/write() on the bus
//     while registered (addr is the already-left-shifted 8-bit wire
//     address SimPlant's own defaultRead/defaultWrite dispatch on, e.g.
//     0x2E for OTOS, 0x20 for the Nezha motor channel -- see sim_plant.cpp's
//     own kMotorWireAddr/kOtosWireAddr). data/len are the SAME buffer/length
//     SimPlant::read()/write() were called with -- for a read, cb is
//     expected to FILL data[0..len) when it returns HANDLED; for a write,
//     data[0..len) holds the bytes the firmware wrote.
//
//     Return convention: 0 = PASS -- the hook declined this transaction;
//     the caller (sim_default_read()/sim_default_write(), see below, is
//     what a PASS-returning Python hook is expected to call itself to get
//     the real response before returning 0/1) -- 1 = HANDLED -- the hook
//     fully answered the transaction itself (for a read: it already wrote
//     data; for a write: it already decided what to do with the bytes,
//     including possibly nothing, i.e. "swallow this write").
//
//     Unlike a real I2CBus::read()/write() PASS/HANDLED distinction, THIS
//     file's dispatch to the hook does not itself re-run a default handler
//     on a 0 return -- see the wrapper lambdas below: whatever the Python
//     hook returns is returned verbatim as SimPlant::read()/write()'s own
//     result. A Python hook that wants pass-through behavior MUST call
//     sim_default_read()/sim_default_write() itself (see next) and return
//     ITS result -- there is no implicit second dispatch.
//
//   int sim_default_read(SimHandle h, uint16_t addr, uint8_t* data, int len);
//   int sim_default_write(SimHandle h, uint16_t addr, uint8_t* data, int len);
//     Thin call-throughs straight to SimPlant::defaultRead()/defaultWrite()
//     -- the pass-through a registered hook calls for "run the real
//     response" WITHOUT re-entering the hook (defaultRead()/defaultWrite()
//     never consult readHook_/writeHook_ -- see sim_plant.h/.cpp). This is
//     how a Python hook implements "observe or lightly perturb, but mostly
//     pass through": call sim_default_read(h, addr, data, len) to get the
//     real bytes, optionally mutate data in place, then return 1 (HANDLED).
//
// Source placement: HOST_BUILD-only test infrastructure, alongside
// sim_plant.{h,cpp}/sim_harness.h -- this file does NOT live in source/.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "sim_harness.h"

namespace {

TestSim::SimHarness* asHarness(void* h) { return static_cast<TestSim::SimHarness*>(h); }

}  // namespace

extern "C" {

using SimHandle = void*;
using SimHookFn = int (*)(void* ctx, uint16_t addr, uint8_t* data, int len);

// ---- Lifecycle ----

SimHandle sim_create(float trackWidth) {
  TestSim::SimHarness* harness = trackWidth > 0.0f ? new TestSim::SimHarness(trackWidth)
                                                    : new TestSim::SimHarness();
  harness->boot();
  return harness;
}

void sim_destroy(SimHandle h) { delete asHarness(h); }

int sim_booted(SimHandle h) { return asHarness(h)->booted() ? 1 : 0; }

int sim_cycle_count(SimHandle h) { return asHarness(h)->cycleCount(); }

// ---- Stepping ----

void sim_step(SimHandle h, int cycles) { asHarness(h)->step(cycles); }

// ---- Command injection ----

void sim_inject_twist(SimHandle h, float v_x, float omega, float duration, uint32_t corr) {
  asHarness(h)->injectTwist(v_x, omega, duration, corr);
}

void sim_inject_stop(SimHandle h, uint32_t corr) { asHarness(h)->injectStop(corr); }

void sim_inject_command(SimHandle h, const char* armoredLine) {
  asHarness(h)->injectCommand(armoredLine);
}

// ---- Telemetry drain ----

int sim_drain_tlm(SimHandle h, char* buf, int buflen) {
  std::vector<std::string> lines = asHarness(h)->drainRawTelemetry();
  std::string joined;
  for (size_t i = 0; i < lines.size(); ++i) {
    if (i != 0) joined += '\n';
    joined += lines[i];
  }
  if (buf != nullptr && buflen > 0) {
    std::snprintf(buf, static_cast<size_t>(buflen), "%s", joined.c_str());
  }
  return static_cast<int>(joined.size());
}

// ---- True pose ----

float sim_true_x(SimHandle h) { return asHarness(h)->trueX(); }
float sim_true_y(SimHandle h) { return asHarness(h)->trueY(); }
float sim_true_h(SimHandle h) { return asHarness(h)->trueHeading(); }

// ---- Fault-condition setters ----

void sim_set_wheel_disconnected(SimHandle h, int port, int disconnected) {
  asHarness(h)->plant().setDisconnected(port, disconnected != 0);
}

void sim_set_wheel_freeze(SimHandle h, int port, int freeze) {
  asHarness(h)->plant().freezePosition(port, freeze != 0);
}

void sim_set_wheel_dropout_rate(SimHandle h, int port, float fraction) {
  asHarness(h)->plant().setDropoutRate(port, fraction);
}

void sim_set_otos_drift(SimHandle h, float xDrift, float yDrift, float headingDrift) {
  asHarness(h)->plant().setOtosDrift(xDrift, yDrift, headingDrift);
}

// ---- Hook surface ----

void sim_set_read_hook(SimHandle h, SimHookFn cb, void* ctx) {
  TestSim::SimPlant& plant = asHarness(h)->plant();
  if (cb == nullptr) {
    plant.clearReadHook();
    return;
  }
  plant.setReadHook([cb, ctx](uint16_t addr, uint8_t* data, int len) {
    return cb(ctx, addr, data, len);
  });
}

void sim_set_write_hook(SimHandle h, SimHookFn cb, void* ctx) {
  TestSim::SimPlant& plant = asHarness(h)->plant();
  if (cb == nullptr) {
    plant.clearWriteHook();
    return;
  }
  plant.setWriteHook([cb, ctx](uint16_t addr, uint8_t* data, int len) {
    return cb(ctx, addr, data, len);
  });
}

int sim_default_read(SimHandle h, uint16_t addr, uint8_t* data, int len) {
  return asHarness(h)->plant().defaultRead(addr, data, len);
}

int sim_default_write(SimHandle h, uint16_t addr, uint8_t* data, int len) {
  return asHarness(h)->plant().defaultWrite(addr, data, len);
}

}  // extern "C"
