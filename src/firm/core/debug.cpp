// debug.cpp -- Core::debugf()'s only implementation. See debug.h's own file
// header for the ROBOT_DEBUG/HOST_BUILD compile gate: this whole file's
// body is guarded by the identical condition the header uses, so a
// shipped ARM release build (neither macro defined) compiles this
// translation unit down to nothing -- no Core::debugf_/Core::setDebugSink_
// symbols at all, not merely unused ones a linker has to garbage-collect.
#include "core/debug.h"

#if defined(ROBOT_DEBUG) || defined(HOST_BUILD)

#include <cstdarg>
#include <cstdio>

#include "core/comms.h"

namespace Core {

namespace {

// The one installed sink. A raw pointer, not owned -- setDebugSink()'s own
// doc comment covers the lifetime contract. nullptr means "no sink yet",
// the safe default before main.cpp/SimHarness wires one up.
Comms* debugSink = nullptr;

// kDebugMsgMaxBytes -- the formatted message body's own scratch buffer,
// BEFORE Comms::sendDebug() prepends the "DBG:" wire prefix. Sized well
// under SerialPort::kTxBufferCapacity (250 bytes, com/serial_port.h) with
// room left for that prefix plus the transport's own trailing '\n' -- a
// debugf() call that formats past this is truncated by vsnprintf(), never
// overflowed.
constexpr size_t kDebugMsgMaxBytes = 200;

}  // namespace

void setDebugSink(Comms* sink) { debugSink = sink; }

void debugf(const char* fmt, ...) {
  if (debugSink == nullptr) return;  // no sink installed yet -- silent no-op

  char buf[kDebugMsgMaxBytes];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  debugSink->sendDebug(buf);
}

}  // namespace Core

#endif  // ROBOT_DEBUG || HOST_BUILD
