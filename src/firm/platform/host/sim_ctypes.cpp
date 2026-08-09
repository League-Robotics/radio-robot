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
//   void sim_inject_stop(SimHandle h, uint32_t corr);   // sends ESTOP -- see below
//   void sim_inject_wheels(SimHandle h, float vLeft, float vRight,
//                          float duration, uint32_t corr);
//   void sim_inject_command(SimHandle h, const char* frame, int len);
//     Raw, non-actuation escape hatch -- pushes ANY already-`<COMMAND>':'
//     <COBS+CRC bytes>`-framed wire line (124-005; was a bare COBS+CRC frame
//     body 123-002-124-004, an already-armored "*B..." line pre-123)
//     straight onto the inbound FakeTransport, for tests that need a wire
//     shape sim_inject_twist()/sim_inject_stop() don't cover. `frame`/`len`
//     is an EXPLICIT length, NOT NUL-terminated: COBS is now keyed on 0x0A
//     (wire_runtime.h item 8), not 0x00, so the line may legitimately
//     contain an embedded 0x00 byte -- a `strlen()`-recovered length would
//     silently truncate it (this is exactly the trap the pre-124-005 version
//     of this ABI fell into when the delimiter changed; see
//     `SimHarness::injectCommand()`, sim_harness.h). Build `frame` with
//     `robot_radio.io.wire_codec.encode_frame()`, PREFIXED with the ASCII
//     command name and ':' (the SAME codec every other binary command
//     producer uses) -- NOT the pre-123 `*B<base64>` shape.
//
// ---- Telemetry drain ----
//   int sim_drain_tlm(SimHandle h, uint8_t* buf, int buflen);
//     124-005 (protocol v5 Part A, "framing grammar cutover"): drains every
//     raw outbound LINE (`<COMMAND>':'<COBS+CRC bytes>`, e.g. "TLM:...")
//     captured since the LAST sim_drain_tlm() call on this handle -- 0x0A-free
//     by COBS construction (COBS is keyed on 0x0A now, wire_runtime.h item 8),
//     exactly what App::Transport::send() received per line -- and copies
//     them into `buf` with EXACTLY one '\n' (0x0A) byte appended after each
//     line -- the SAME trailing-delimiter convention the real wire itself
//     uses now (comms.h's Transport::send() doc comment), reproduced here
//     explicitly because SimHarness::drainRawTelemetry()'s own capture does
//     NOT include that trailing byte (it captures only the line
//     Comms::sendReply()/Telemetry::emitSecondary() built, before a real
//     transport would append its own delimiter). This makes the joined
//     buffer byte-for-byte the same shape multiple back-to-back real wire
//     lines would occupy on an actual serial/radio byte stream, so the
//     Python side demuxes it with the EXACT SAME '\n'-split + COBS-decode +
//     CRC-verify logic (`robot_radio.io.wire_codec.ByteStreamDemuxer`/
//     `decode_frame()`) it already needs for a real transport -- no separate
//     "sim ABI framing" convention to maintain. (123-002/003's own join byte
//     was 0x00, safe back then because COBS was keyed on 0x00 and every
//     captured frame was 0x00-free by construction; 124-005 re-keys BOTH the
//     COBS delimiter and this join byte to 0x0A together, so the property
//     still holds.)
//
//     Returns the TOTAL number of bytes across every captured line (each
//     line's own length + 1 for its trailing '\n' delimiter) -- mirroring
//     snprintf()'s own return-value convention so a caller can detect
//     truncation (return value > buflen means only a PREFIX of whole lines
//     was copied), except this is a raw byte count, not a string length: the
//     copy is a memcpy, never assumes or inserts a text NUL terminator of its
//     own beyond what the lines' own trailing delimiters already provide.
//     Never splits a line across the buflen boundary -- if a line would not
//     fit whole, it (and every line after it in this drain) is left
//     uncopied, though the drain has still CONSUMED it (same "drain always
//     advances regardless of whether buf was big enough" contract as
//     pre-124): pass a buffer sized generously (a handful of KB comfortably
//     covers a burst of frames from one step() call) to avoid this in
//     practice. buf may be NULL / buflen may be 0 to just drain-and-discard
//     (only the total byte count is computed). The Python side decodes each
//     demuxed frame with the exact same COBS+CRC codec a real robot's
//     replies go through (`robot_radio.io.wire_codec`).
//
// ---- Debug line drain (129-003, bench/Sim-only DBG channel) ----
//   int sim_drain_debug(SimHandle h, char* buf, int buflen);
//     Same drain contract as sim_drain_tlm() above (snprintf()-style
//     return-value convention, never splits a line across buflen, buf may
//     be NULL/buflen 0 to drain-and-discard), but sourced from
//     SimHarness::drainReliable() (the cleartext-plane capture READY/
//     STATUS/HELP/DEVICE/PONG already ride, comms.h's own Transport doc
//     comment) filtered down to ONLY lines starting "DBG:" -- the one
//     cleartext verb this sprint's App::debugf() (app/debug.h) ever emits
//     unsolicited, at an unbounded rate, that a Python caller (SimLoop.
//     drain_debug_lines()) wants to poll. A non-DBG reliable line (e.g. a
//     STATUS reply to some other test's own query) is silently NOT
//     returned here -- nothing else currently drains drainReliable()
//     through this C ABI, so nothing else's traffic is lost by this
//     filtering.
//   void sim_test_emit_debug(SimHandle h, const char* msg);
//     TEST-ONLY escape hatch: calls App::debugf("%s", msg) directly against
//     this handle's own installed sink (SimHarness's constructor already
//     wires App::setDebugSink(&comms_)), with no real subsystem call site
//     involved. Exists because this ticket lands the DBG CHANNEL itself,
//     ahead of tickets 006/007's actual debugf() call sites -- proves the
//     full setDebugSink()/debugf()/Comms::sendDebug()/sim_drain_debug()
//     round trip end to end without needing a real diagnostic to exist
//     yet. A no-op if this library was built without ROBOT_DEBUG/
//     HOST_BUILD (never true for this library -- HOST_BUILD=1 is always
//     defined here, see this file's own includes).
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

#include "app/debug.h"
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

// Commanded per-wheel velocity (the interim PID SETPOINT App::Drive stages
// -- 125-003: read from Drive's own driveTargetVelLeft/Right() accessor now,
// NOT Devices::Motor::velocityTarget(), which is deleted -- the velocity
// PID moved off NezhaMotor entirely, see drive.h's own header). cmd_vel is
// NOT on the wire at all (it never made it off TelemetrySecondary before
// that message was deleted outright, 124-009 -- see Types::RobotState::
// Wheel::cmdVelocity's own doc comment for the current, unwired state). The
// sim can see this normally-invisible inner-loop command at full rate,
// which is exactly what TestGUI's "commanded vs actual" wheel-speed graph
// plots. Signed [mm/s].
float sim_cmd_vel_left(SimHandle h) { return asHarness(h)->driveTargetVelLeft(); }
float sim_cmd_vel_right(SimHandle h) { return asHarness(h)->driveTargetVelRight(); }

// Velocity-PID enable/disable (stakeholder 2026-07-18, TestGUI "PID"
// checkbox next to the Test buttons) -- 125-003: NOW A NO-OP. The velocity
// PID this used to toggle on/off (Devices::NezhaMotor::setPidEnabled()) no
// longer exists on the Motor interface at all (Decision 2, sprint.md -- PID
// is a control decision, not hardware protection, and relocated to a
// motion-local wheel-velocity PID class -- itself deleted outright by
// 128-015, zero instantiations; App::Drive holds no controller of its own,
// see src/motion/DESIGN.md's "wheel control generations" note). This C ABI
// export is kept (not deleted) purely so host/robot_radio/io/sim_loop.py's
// ctypes symbol lookup at import time does not break -- it has no effect on
// firmware behavior; the one duty-stage controller that still exists
// (Motion::Planner's own, parked from the live tick by 128-015) has no
// enable/disable surface either.
void sim_set_pid_enabled(SimHandle, int) {}

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

// sim_inject_stop -- retargeted at ESTOP (command-ingestion-ring-buffered-
// comms-subsystem-routing-two-stops.md §2). The export's NAME and SIGNATURE
// are unchanged, so sim_loop.py's own ctypes binding and every caller of
// SimLoop.stop() keep working, but the WIRE COMMAND it sends is now ESTOP,
// not STOP. That is the behavior-preserving choice, not a change of intent:
// every existing caller of this entry point means "halt the drivetrain
// now," which is exactly what ESTOP still does and what STOP no longer
// does (STOP is now a planned, queued stop). A caller that genuinely wants
// the planned stop builds it through sim_inject_command().
void sim_inject_stop(SimHandle h, uint32_t corr) { asHarness(h)->injectEstop(corr); }

// sim_inject_wheels -- the dumb teleop primitive (§2), straight to
// App::Drive. `corr` doubles as the envelope corr_id and the command's own
// completion id, matching sim_inject_twist()'s established convention here.
void sim_inject_wheels(SimHandle h, float vLeft, float vRight, float duration, uint32_t corr) {
  asHarness(h)->injectWheels(vLeft, vRight, duration, /*id=*/corr, corr);
}

void sim_inject_command(SimHandle h, const char* armoredLine, int len) {
  asHarness(h)->injectCommand(armoredLine, static_cast<size_t>(len));
}

// ---- Telemetry drain ----

int sim_drain_tlm(SimHandle h, uint8_t* buf, int buflen) {
  std::vector<std::string> frames = asHarness(h)->drainRawTelemetry();

  size_t total = 0;
  for (const std::string& frame : frames) total += frame.size() + 1;  // +1 for the trailing '\n'

  if (buf != nullptr && buflen > 0) {
    size_t copied = 0;
    const size_t cap = static_cast<size_t>(buflen);
    for (const std::string& frame : frames) {
      const size_t frameTotal = frame.size() + 1;
      if (copied + frameTotal > cap) break;  // never split a frame across the buffer boundary
      std::memcpy(buf + copied, frame.data(), frame.size());
      buf[copied + frame.size()] = '\n';
      copied += frameTotal;
    }
  }
  return static_cast<int>(total);
}

// ---- Debug line drain (129-003, bench/Sim-only DBG channel) ----

int sim_drain_debug(SimHandle h, char* buf, int buflen) {
  // System-test change: return the WHOLE reliable cleartext plane, not
  // just DBG: lines -- the Python side (SimLoop._drain_debug_into_queue)
  // routes DBG to the debug queue and everything else (READY/STATUS/PONG/
  // DEVICE replies) to its on_cleartext observer, so sim datasets carry
  // the same cleartext records hardware datasets do.
  std::vector<std::string> debugLines = asHarness(h)->drainReliable();

  size_t total = 0;
  for (const std::string& line : debugLines) total += line.size() + 1;  // +1 for the trailing '\n'

  if (buf != nullptr && buflen > 0) {
    size_t copied = 0;
    const size_t cap = static_cast<size_t>(buflen);
    for (const std::string& line : debugLines) {
      const size_t lineTotal = line.size() + 1;
      if (copied + lineTotal > cap) break;  // never split a line across the buffer boundary
      std::memcpy(buf + copied, line.data(), line.size());
      buf[copied + line.size()] = '\n';
      copied += lineTotal;
    }
  }
  return static_cast<int>(total);
}

// TEST-ONLY: see this file's own header comment on sim_drain_debug()'s
// section for why this exists ahead of tickets 006/007's real debugf()
// call sites. `msg` is passed through App::debugf()'s own "%s" formatting
// -- NOT %-interpreted itself, so a test string containing a literal '%'
// cannot be misread as a format specifier.
void sim_test_emit_debug(SimHandle h, const char* msg) {
  (void)h;  // App::debugf() routes through the process-global sink
            // App::setDebugSink() installed -- SimHarness's constructor
            // already did this for THIS handle's own comms_ (there is
            // only ever one live handle per test process in practice).
  App::debugf("%s", msg);
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
// (NezhaMotor::config()) and overwrites ONLY port/fwdSign -- the fields
// this Tier-2 surface owns -- before the configureMotor() round trip. This
// function's original velGains-only merge (built on a blank
// MotorConfig{}) predates 114-001 Revision 1, which made MotorArmor::
// reconfigure() forward the WHOLE config to the wrapped NezhaMotor: from
// that revision on, every un-merged field (wheelTravelCalib/slewRate/
// outputDeadband/reversalDwell) was silently zeroed by this call, killing
// the encoder mm decode (nezha_motor.cpp gates position on
// wheelTravelCalib != 0) whenever Tier 2 landed AFTER a Tier-1 ConfigDelta
// push. Full-config merge preserves this function's own documented "cannot
// clobber what Tier 1 already pushed" contract.
//
// `velFiltAlpha` (the parameter, kept for C ABI/ctypes-argtypes
// compatibility with host/robot_radio/io/sim_loop.py) is now UNUSED --
// 125-003 deleted Devices::MotorConfig::velFiltAlpha along with the EMA
// velocity estimator it fed (pending ticket 004's App::WheelObserver,
// which will need its own config surface for whatever replaces it).
void sim_configure_motor(SimHandle h, int port, float /*velFiltAlpha*/, int fwdSign) {
  TestSim::SimHarness* harness = asHarness(h);
  Devices::NezhaMotor& motor = (port == 2) ? harness->motorRight() : harness->motorLeft();
  Devices::MotorConfig cfg = motor.config();  // full live config -- merge, don't clobber
  cfg.port = static_cast<uint32_t>(port);
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
// `*velFiltAlpha` (parameter kept for C ABI compatibility, see
// sim_configure_motor()'s own comment) always reads back 0.0f -- there is
// no live Devices::MotorConfig::velFiltAlpha field left to report.
void sim_read_motor_config(SimHandle h, int port, float* velFiltAlpha, int* fwdSign) {
  const Devices::MotorConfig& cfg = asHarness(h)->motorConfig(static_cast<uint32_t>(port));
  *velFiltAlpha = 0.0f;
  *fwdSign = cfg.fwdSign;
}

// 125-007 (adjacent-sim-plant-rotation-calibration-for-angle-stop-move-
// overshoot.md): thin call-through to App::RobotLoop::setRotationCalibration()
// (robot_loop.h), the SAME boot-only turn-calibration seam main.cpp uses for
// real hardware (main.cpp reads drivetrainConfig.rotation_gain_pos/
// rotation_offset/rotation_gain_neg/rotation_offset_neg, converts the
// offsets deg->rad, and calls this exact method once at boot). Before this
// export existed, nothing in the sim path ever called
// setRotationCalibration() at all -- SimHarness's own App::RobotLoop kept
// the identity default (gain 1, offset 0) permanently, regardless of what a
// robot JSON's calibration.rotation_gain/rotation_offset_deg said, which is
// why editing those JSON fields alone was a silent no-op for
// square_tour.py --sim's ANGLE-stop overshoot. `robotLoop()` is already a
// public accessor (sim_harness.h) -- no new SimHarness method needed, this
// export just gives it a ctypes-callable C ABI shape like every other
// Tier-2 export in this file.
//
// offsetPos/offsetNeg are [rad] here, matching setRotationCalibration()'s
// own contract and its own (unit-free, `// [rad]`-tagged) parameter names
// -- the deg->rad conversion happens host-side (sim_boot_config.py's
// drivetrain_boot_config_for(), mirroring main.cpp's own conversion at its
// seam) so this export stays a pure passthrough, no unit-conversion logic
// of its own.
void sim_configure_drivetrain(SimHandle h, float gainPos, float offsetPos,  // [rad]
                              float gainNeg, float offsetNeg) {  // [rad]
  asHarness(h)->robotLoop().setRotationCalibration(gainPos, offsetPos, gainNeg, offsetNeg);
}

// App::Drive's own boot calibration -- the sim-side counterpart of main.cpp's
// setDutyPerSpeed()/setCrawlPulse() seam, which reads the same values out of
// Config::defaultDriveConfig(). That generated config is deliberately absent
// from the sim CMake target (src/firm/platform/host/CMakeLists.txt bakes the active robot
// JSON at ARM build time only), so the values arrive over ctypes instead,
// host-side, from sim_boot_config.py's own drive_boot_config_for() -- see that
// function's docstring for why the sim went without a drive calibration
// entirely until this export existed, what the `pid.kff` accident it replaces
// cost, and why main.cpp's third install (setWheelCorrection(), a
// hardware-gearbox linearization) is deliberately NOT mirrored here.
//
// Pure passthrough: no unit conversion, no defaulting. `drive()` is already a
// public accessor (sim_harness.h) -- no new SimHarness method needed.
void sim_configure_drive(SimHandle h, float dutyPerSpeedLeft, float dutyPerSpeedRight,
                         float crawlPulse) {
  App::Drive& drive = asHarness(h)->drive();
  drive.setDutyPerSpeed(dutyPerSpeedLeft, dutyPerSpeedRight);
  drive.setCrawlPulse(crawlPulse);
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
