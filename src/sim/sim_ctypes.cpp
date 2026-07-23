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
//   void sim_set_true_pose(SimHandle h, float x, float y, float h_rad);  // [mm][mm][rad]
//     Plant teleport -- snaps the OtosPlant's ground-truth pose to
//     (x, y, h_rad) and resets both WheelPlant positions to 0 in the same
//     call (SimHarness::setTruePose() -> SimPlant::setTruePose()). Added
//     for the TestGUI Sim command-surface fix: Sim mode has no operator to
//     physically place the robot at the playfield centre the way real
//     hardware's "Set Robot @ 0,0" workflow assumes, so
//     host/robot_radio/io/sim_loop.py's set_true_pose() calls this instead.
//
// ---- Fault-condition setters ----
// Thin call-throughs to SimPlant's own knobs (sim_plant.h). port: 1 = left
// (Nezha motorId 1), 2 = right (motorId 2) -- same numbering the real wire
// frame's byte [2] carries.
//   void sim_set_wheel_disconnected(SimHandle h, int port, int disconnected);  // 1/0
//   void sim_set_wheel_freeze(SimHandle h, int port, int freeze);              // 1/0
//   void sim_set_wheel_dropout_rate(SimHandle h, int port, float fraction);    // [0,1]
//   void sim_set_otos_drift(SimHandle h, float xDrift, float yDrift, float headingDrift);  // [mm][mm][rad]
//   void sim_set_enc_scale_err(SimHandle h, int port, float fraction);  // fractional over/under-report (109-002)
//   void sim_set_otos_raw_scale_err(SimHandle h, float linearFraction, float angularFraction);  // fractional over/under-report, 0=perfect (109-007)
//   void sim_set_enc_tick_quant(SimHandle h, int port, float tickSizeMm);  // [mm] (109-007)
//   void sim_set_enc_slip(SimHandle h, int port, float rate, float magnitudeMm);  // [0,1] [mm] (109-007)
//
// ---- Tier-2 config-load surface (113-002) ----
// Thin call-through to SimHarness::configureMotor() -- the additive,
// one-shot "load a full boot config at runtime" surface for the per-motor
// vel_filt/fwd_sign fields that have no live Tier-1 wire arm.
//
// sim_configure_planner()/sim_read_planner_config()/
// sim_set_lead_compensation()/sim_set_yaw_rate_max()/sim_debug_heading_lead()
// -- DELETED (115-006, gut S1): msg::PlannerConfig and
// SimHarness::configurePlanner()/plannerConfig() no longer exist
// (Motion::Executor/App::Pilot/App::HeadingSource were deleted by 115-002's
// motion-stack excision) -- there is nothing left for any of these
// call-throughs to reach.
//   void sim_configure_motor(SimHandle h, int port, float velFiltAlpha, int fwdSign);
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

// Firmware version compiled into THIS shared library -- exported so the host
// (TestGUI) can display the version of the binary it actually LOADED, not the
// version sitting in the source tree. A running process keeps the old dylib
// mapped after a rebuild (dlopen caches by path), so "the sim is always built
// from this tree" is not something the GUI can assume. version_generated.h is
// emitted by gen_version.py and git-ignored, so guard the include and fall
// back to a dev sentinel when it is absent.
#if __has_include("types/version_generated.h")
#include "types/version_generated.h"
#endif
#ifndef FIRMWARE_VERSION_STR
#define FIRMWARE_VERSION_STR "0.0.0-dev"
#endif

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
  // Rest-encoder jitter (108-011) is enabled ONLY on this ctypes/hardware-
  // realistic path -- every Python consumer of the sim (the tour runner,
  // TestGUI's sim-mode transport) gets hardware-like encoders that never
  // hold a byte-identical stopped-wheel reading long enough to false-
  // positive Devices::MotorArmor's wedge-latch detector (kWedgeThreshold=10
  // consecutive identical reads) -- see wheel_plant.h's own "Rest-dither
  // tuning" comment for why. The plain C++ SimHarness/SimPlant construction
  // path (used directly by tests/sim/system/*.cpp scenario tests and
  // plant_harness.cpp) never calls this file, so it stays on WheelPlant's
  // deterministic default (jitter OFF) -- those tests assert an exact,
  // byte-stable stopped-wheel reportedPosition() and must not see jitter.
  harness->plant().setEncoderJitter(true);
  harness->boot();
  return harness;
}

void sim_destroy(SimHandle h) { delete asHarness(h); }

int sim_booted(SimHandle h) { return asHarness(h)->booted() ? 1 : 0; }

int sim_cycle_count(SimHandle h) { return asHarness(h)->cycleCount(); }

// Version string compiled into this library (see the FIRMWARE_VERSION_STR note
// near the includes). Stateless -- needs no SimHandle.
const char* sim_firmware_version() { return FIRMWARE_VERSION_STR; }

// sim_cycle_dt_us -- 118 ticket 003 (sim-cycle-must-match-firmware-period.md):
// exposes TestSim::SimHarness::kCycleDtUs (itself derived from firmware's own
// App::RobotLoop::kCycle, robot_loop.h) to Python so a ctypes caller can
// derive its OWN cadence constants from this one compiled-in value instead of
// an independently-hardcoded matching literal that can drift apart silently.
// Stateless -- needs no SimHandle (kCycleDtUs is a compile-time constant, not
// per-instance state).
int sim_cycle_dt_us() { return static_cast<int>(TestSim::SimHarness::kCycleDtUs); }

// Commanded per-wheel velocity (the velocity-PID SETPOINT) read DIRECTLY from
// the firmware's live NezhaMotor -- Path B (2026-07-17). cmd_vel is NOT on the
// wire (adding it to the primary Telemetry frame overflows the 186-byte
// envelope budget; it lives on the slower TelemetrySecondary). The sim can see
// this normally-invisible inner-loop command at full rate, which is exactly
// what TestGUI's "commanded vs actual" wheel-speed graph plots. Signed [mm/s].
float sim_cmd_vel_left(SimHandle h) { return asHarness(h)->motorLeft().velocityTarget(); }
float sim_cmd_vel_right(SimHandle h) { return asHarness(h)->motorRight().velocityTarget(); }

// Velocity-PID enable/disable on BOTH live NezhaMotors (stakeholder
// 2026-07-18, TestGUI "PID" checkbox next to the Test buttons) -- thin
// call-through to Devices::NezhaMotor::setPidEnabled(), the same sim-only
// direct-firmware-object surface as sim_cmd_vel_left/right() above (no wire
// arm exists for PID enable; DEV-family bench verbs are text-plane only).
// With PID off, a setVelocity()-staged command drives OPEN-LOOP: duty =
// Gains::kff [duty per mm/s] * velocityTarget_ with every feedback term
// bypassed (nezha_motor.cpp tick() step 4) -- Drive-driven motion
// (twist/Move) keeps moving at the feedforward-nominal speed, uncorrected.
void sim_set_pid_enabled(SimHandle h, int enabled) {
  asHarness(h)->motorLeft().setPidEnabled(enabled != 0);
  asHarness(h)->motorRight().setPidEnabled(enabled != 0);
}

// ---- Stepping ----

void sim_step(SimHandle h, int cycles) { asHarness(h)->step(cycles); }

// ---- Command injection ----

// sim_inject_twist -- BEHAVIOR-PRESERVING TRANSLATION (116-006, MOVE
// protocol cutover): App::Deadman and SimHarness::injectTwist() are both
// deleted -- every motion is now a bounded MOVE (arm 21), no separate
// deadman lease (protocol-set-point issue). This C ABI export's NAME and
// SIGNATURE stay unchanged (sim_loop.py's own ctypes binding needs no
// change) but its body now injects a MOVE that reproduces the deleted
// Twist+Deadman contract as closely as a bounded command can: a TWIST
// velocity variant, a TIME stop condition at `duration` (the deadman's own
// rearm window), `timeout` == `duration` too (nothing else can legitimately
// end a TIME-stop MOVE early), and `replace=true` (a fresh call always
// preempts/restarts the timer -- the deadman's own "every call sets a
// FRESH deadline ... re-arming, not stacking" contract, the deleted
// deadman.h's own arm() doc comment). `corr` doubles as both the
// enqueue-ack corr_id and the MOVE's own completion id -- this call site
// never distinguished the two.
void sim_inject_twist(SimHandle h, float v_x, float omega, float duration, uint32_t corr) {
  asHarness(h)->injectMove(v_x, /*v_y=*/0.0f, omega, TestSupport::MoveStopKind::kTime,
                            /*stopValue=*/duration, /*timeout=*/duration, /*replace=*/true,
                            /*id=*/corr, corr);
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

void sim_set_true_pose(SimHandle h, float x, float y, float h_rad) {
  asHarness(h)->setTruePose(x, y, h_rad);
}

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

void sim_set_enc_scale_err(SimHandle h, int port, float fraction) {
  asHarness(h)->plant().setEncScaleErr(port, fraction);
}

void sim_set_otos_raw_scale_err(SimHandle h, float linearFraction, float angularFraction) {
  asHarness(h)->plant().setOtosRawScaleErr(linearFraction, angularFraction);
}

void sim_set_enc_tick_quant(SimHandle h, int port, float tickSizeMm) {
  asHarness(h)->plant().setEncTickQuantization(port, tickSizeMm);
}

void sim_set_enc_slip(SimHandle h, int port, float rate, float magnitudeMm) {
  asHarness(h)->plant().setEncSlip(port, rate, magnitudeMm);
}

// sim_set_lead_compensation()/sim_set_yaw_rate_max()/sim_debug_heading_lead()
// -- DELETED (115-006, gut S1): SimHarness::setLeadCompensation()/
// setYawRateMax()/debugHeadingLead() no longer exist -- Motion::Executor/
// App::Pilot/App::HeadingSource were deleted by 115-002's motion-stack
// excision. See sim_harness.h's own header.

// ---- Tier-2 config-load surface (113-002) ----
//
// Thin call-through to SimHarness::configureMotor() -- see sim_harness.h's
// own doc comment on that method for the ADDITIVE contract (existing
// default-constructed SimHarness callers are unaffected; this export is the
// ONLY way a ctypes caller reaches it). Safe to call either before or after
// boot() (sim_create() already calls boot() unconditionally before
// returning a handle to the caller -- see this file's own header) since it
// does not touch Preamble's own boot sequencing.
//   void sim_configure_motor(SimHandle h, int port, float velFiltAlpha,
//     int fwdSign);
//     port: 1 = left, 2 = right (same convention as every other per-port
//     export above). Merges velFiltAlpha/fwdSign (the two MotorConfig
//     fields with no live Tier-1 wire arm) onto the target motor's FULL
//     current config (read live via NezhaMotor::config() -- every field,
//     not just velGains; see this function's own comment below for the
//     2026-07-22 regression the earlier gains-only merge caused) rather
//     than a blank MotorConfig{}, so this call cannot clobber what Tier
//     1's own MotorConfigPatch wire path already pushed.
//
// ---- Tier-2 config-load readback (113-007 test-only diagnostic) ----
// Thin call-through to SimHarness::motorConfig() -- the SAME test-only C++
// accessor ticket 002's own harness test (sim_harness_configure_harness.cpp)
// already exercises at the C++ level.
//
// sim_read_planner_config() -- DELETED (115-006, gut S1): SimHarness::
// plannerConfig() no longer exists (see this file's own header). Out-pointer
// style, mirroring the surviving hook surface's convention -- thin
// call-throughs, no logic of their own.
//   void sim_read_motor_config(SimHandle h, int port, float* velFiltAlpha,
//     int* fwdSign);
//     port: 1 = left, 2 = right. Returns whatever configureMotor() was last
//     called with for that port (SimHarness::motorConfig()'s own contract --
//     a default-constructed Devices::MotorConfig{} if configureMotor() was
//     never called for that port).

// sim_configure_planner() -- DELETED (115-006, gut S1): msg::PlannerConfig
// and SimHarness::configurePlanner() no longer exist. See this file's own
// header.

// port: 1 = left, 2 = right. Starts from the motor's FULL live config
// (NezhaMotor::config()) and overwrites ONLY port/velFiltAlpha/fwdSign --
// the fields this Tier-2 surface owns -- before the configureMotor()
// round trip. This function's original velGains-only merge (built on a
// blank MotorConfig{}) predates 114-001 Revision 1, which made
// MotorArmor::reconfigure() forward the WHOLE config to the wrapped
// NezhaMotor: from that revision on, every un-merged field
// (wheelTravelCalib/slewRate/outputDeadband/velDeadband/reversalDwell)
// was silently zeroed by this call, killing the encoder mm decode
// (nezha_motor.cpp gates position on wheelTravelCalib != 0) whenever
// Tier 2 landed AFTER a Tier-1 ConfigDelta push -- the exact 2026-07-22
// TestGUI-Sim wheels-never-move regression (exposed, not caused, by the
// same-day set_config fire-and-poll fix that made Tier 1 complete BEFORE
// Tier 2 for the first time). Full-config merge restores this function's
// own documented "cannot clobber what Tier 1 already pushed" contract for
// EVERY field, not just velGains.
void sim_configure_motor(SimHandle h, int port, float velFiltAlpha, int fwdSign) {
  TestSim::SimHarness* harness = asHarness(h);
  Devices::NezhaMotor& motor = (port == 2) ? harness->motorRight() : harness->motorLeft();
  Devices::MotorConfig cfg = motor.config();  // full live config -- merge, don't clobber
  cfg.port = static_cast<uint32_t>(port);
  cfg.velFiltAlpha = velFiltAlpha;
  cfg.fwdSign = fwdSign;
  harness->configureMotor(static_cast<uint32_t>(port), cfg);
}

// ---- Tier-2 config-load readback (113-007) ----
//
// Thin call-through to SimHarness::motorConfig() -- see this file's own
// header comment for why this ticket added a Python-reachable read
// direction (proving the FULL configure_from_robot() pipeline landed the
// right values, not just the C++ call site ticket 002's own harness test
// already covered).
//
// sim_read_planner_config() -- DELETED (115-006, gut S1): SimHarness::
// plannerConfig() no longer exists.

// port: 1 = left, 2 = right (same convention as sim_configure_motor() above).
void sim_read_motor_config(SimHandle h, int port, float* velFiltAlpha, int* fwdSign) {
  const Devices::MotorConfig& cfg = asHarness(h)->motorConfig(static_cast<uint32_t>(port));
  *velFiltAlpha = cfg.velFiltAlpha;
  *fwdSign = cfg.fwdSign;
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
