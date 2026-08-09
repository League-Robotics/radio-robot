// debug.h -- App's bench/Sim-only debug message channel (DBG=18,
// protos/commands.proto). Bench diagnostics for duty-sweep and adaptive
// calibration work that needs to see internal state without a full
// telemetry decode.
//
// Compile gate: everything below is a REAL, callable API only when
// ROBOT_DEBUG is defined (a bench firmware build's own opt-in CMake
// option -- see CMakeLists.txt's own ROBOT_DEBUG block, mirroring the
// existing FAKE_OTOS pattern) or HOST_BUILD is defined (Sim/host tests
// always have it, per src/firm/platform/host/CMakeLists.txt's `-DHOST_BUILD=1`). The
// shipped ARM release build defines neither, so debugf()/DBG_EVERY()/
// DBG_MILLI() are inline no-ops there instead -- zero flash cost, zero
// wire traffic. The same macro gates Core::Comms::sendDebug() (comms.h/.cpp,
// this module's ONLY caller into Core::Comms) and debug.cpp's own function
// bodies, so there is never a mismatch between what this header declares
// and what debug.cpp defines.
//
// debugf() formats a printf-style line and hands it to the installed sink
// as one cleartext "DBG:<line>" wire line (Core::Comms::sendDebug()).
// setDebugSink() wires the sink: main.cpp passes &comms on the robot,
// TestSim::SimHarness does the same with its own comms_ member -- both
// already construct an Core::Comms, so this is a one-line addition at each
// composition root, not a second communications channel.
//
// newlib-nano (the ARM libc this firmware links) has NO printf float
// support -- a bare "%f" silently emits nothing. Any debugf()/DBG_EVERY()
// call that needs to report a float value must convert it to an integer
// milli-unit first with DBG_MILLI() and format it with %ld, e.g.
// `Core::debugf("v=%ld", DBG_MILLI(velocity))` prints "v=1234" for
// velocity == 1.234f.
#pragma once

#include <cstdint>

namespace Core {

class Comms;

#if defined(ROBOT_DEBUG) || defined(HOST_BUILD)

#ifndef ROBOT_DEBUG
// HOST_BUILD implies ROBOT_DEBUG (this file's own header) -- defined here
// so any code that checks `#ifdef ROBOT_DEBUG` specifically (not the
// `defined(ROBOT_DEBUG) || defined(HOST_BUILD)` pair this file uses
// throughout) still sees it under HOST_BUILD too.
#define ROBOT_DEBUG 1
#endif

// setDebugSink -- install where debugf() output goes. `sink` must outlive
// every debugf() call made after this -- true for the whole life of the
// process (ARM) or Sim session (TestSim::SimHarness), matching Comms's
// own lifetime. Passing nullptr disables output again (debugf() becomes a
// silent no-op).
void setDebugSink(Comms* sink);

// debugf -- printf-style debug line (see this file's own header for the
// DBG_MILLI() float caveat). A no-op -- `fmt` is never even formatted --
// if no sink has been installed yet, so a call before setDebugSink() (or
// in a unit test that never calls it) is always safe.
void debugf(const char* fmt, ...);

#else  // shipped ARM release build -- compiles to nothing

inline void setDebugSink(Comms*) {}
inline void debugf(const char*, ...) {}

#endif  // ROBOT_DEBUG || HOST_BUILD

}  // namespace Core

// DBG_EVERY(n, ...) -- call Core::debugf(__VA_ARGS__) on every Nth
// invocation of this macro AT THIS CALL SITE (a function-local static
// counter, one per macro expansion, not a shared global -- throttling one
// call site never affects another's cadence). `n` must be a positive
// compile-time constant. The whole macro compiles to an empty statement
// (no counter, no argument evaluation) when neither ROBOT_DEBUG nor
// HOST_BUILD is defined -- the counter itself must not cost flash in a
// release build, not just the print.
#if defined(ROBOT_DEBUG) || defined(HOST_BUILD)
#define DBG_EVERY(n, ...)                    \
  do {                                        \
    static uint32_t dbgEveryCount = 0;         \
    if ((dbgEveryCount % (n)) == 0) {           \
      Core::debugf(__VA_ARGS__);                  \
    }                                             \
    ++dbgEveryCount;                                \
  } while (0)
#else
#define DBG_EVERY(n, ...) \
  do {                     \
  } while (0)
#endif

// DBG_MILLI(x) -- see this file's own header: newlib-nano's printf has no
// float support, so any debugf()/DBG_EVERY() call reporting a float value
// converts it to an integer milli-unit with this macro first, then
// formats it with %ld. Rounds to nearest, ties away from zero -- the same
// round-half-away-from-zero idiom messages/*.h's generated pack*() fixed-
// point converters already use (gen_messages.py's (scale)-option codegen).
#define DBG_MILLI(x) \
  (static_cast<long>((x) >= 0.0f ? ((x) * 1000.0f + 0.5f) : ((x) * 1000.0f - 0.5f)))
