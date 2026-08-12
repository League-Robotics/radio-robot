// fake_transport.h -- HOST_BUILD-only Hal::Transport double: an in-memory,
// FIFO-based fake a test can push complete inbound LINES into (so
// Core::Comms::pump() reads them exactly as if from a real serial/radio
// line) and read captured outbound lines back out of (every
// Core::Comms::sendReply()/Core::Telemetry emit call). Ticket 105-002
// (SUC-019); mirrors comms.h's own documentation density/style. 136-005
// re-based this off Hal::Transport (hal/transport.h) -- the interface
// moved out of core/comms.h's own former #ifndef HOST_BUILD block into
// hal/, and its two ARM-only adapter classes (Core::SerialTransport/
// RadioTransport) are gone outright now that Platform::MicroBitSerialPort/
// MicroBitRadioLink implement Hal::Transport directly.
//
// 124-005 (protocol v5 Part A, "framing grammar cutover"): the interface
// no longer distinguishes text/binary at all -- Core::FrameKind is gone,
// and readLine() now returns a plain bool (comms.h's own file header, pre-
// 136-005). This fake's inbound queue follows suit: ONE FIFO of raw
// lines (a `<COMMAND>[':' <data>]` string a real transport would deliver,
// `\n` already stripped), no per-entry kind tag. `enqueueInboundBinary()`
// is kept as a same-behavior ALIAS of `enqueueInbound()` (both just queue
// raw bytes now) rather than deleted outright -- every existing scenario
// across this tree's harnesses already calls it with a FULLY-PREFIXED line
// (`TestSupport::armor*Command()`/`wire_test_codec`'s `armor()` now
// prepend the ASCII command name themselves, see those files' own updated
// doc comments), so keeping the name avoids a purely-mechanical rename at
// every one of those call sites for zero behavioral gain.
//
// This is the ONE canonical FakeTransport for src/tests/sim/ -- several
// harnesses previously carried their own ad hoc copy (app_comms_harness.cpp,
// app_telemetry_harness.cpp, the latter also had a second QueueableFake
// Transport variant purely to add a queue where its base fake had none).
// Both are now built on this single class; later tickets (004's sim_api,
// 006's pytest scenarios) build on it too, per architecture-update.md
// Step 3's "FakeTransport (test-only, e.g. src/tests/sim/support/
// fake_transport.h)" entry.
//
// Design: two SEPARATE concerns, matching Transport's own two directions.
//   - Inbound: a test calls enqueueInbound()/enqueueInboundBinary() to
//     script complete lines; readLine() pops ONE per call and returns
//     false, non-blocking, the instant the queue is empty -- never
//     populating buf. That is Transport::readLine()'s own contract (one
//     complete line per call), NOT a statement about pump(): since
//     command-ingestion-ring-buffered-comms-subsystem-routing-two-stops.md
//     §1, Comms::pump() CALLS readLine() repeatedly and drains both
//     transports into a command ring, so a test that scripts N lines and
//     then steps once will have all N consumed by that step.
//   - Outbound: send() (async, drop-on-full on the real transports) and
//     sendReliable() (bounded-wait, must-not-drop) each append to their OWN
//     capture -- this fake never actually drops anything (a host test wants
//     to see EVERY outbound line, not exercise real backpressure); a test
//     drains/inspects sent()/sentReliable() after stepping the loop.
//
// std::deque for all three queues: this is HOST_BUILD test-only code, not
// the hot ARM path the project's no-heap-in-hot-path convention targets --
// i2c_bus_host.cpp's own scripted queues already set this precedent.
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>

#include "hal/transport.h"

namespace TestSupport {

class FakeTransport : public Hal::Transport {
 public:
  // Push one complete wire LINE into the inbound FIFO -- `\n` already
  // stripped, exactly what a real Transport::readLine() delivers. Text or
  // binary is no longer distinguished here (124-005) -- Core::Comms itself
  // decides that from the parsed `<COMMAND>` prefix. readLine() pops
  // lines oldest-first.
  void enqueueInbound(const char* line) { inbound_.emplace_back(line); }
  void enqueueInbound(const uint8_t* data, size_t len) {
    inbound_.emplace_back(reinterpret_cast<const char*>(data), len);
  }
  void enqueueInbound(const std::string& line) { inbound_.push_back(line); }

  // Alias of enqueueInbound() -- see this class's own file header for why
  // the name is kept rather than deleted (every existing call site already
  // passes a fully command-prefixed line). DELIBERATELY NO `const char*`
  // overload here (unlike pre-124-005): COBS is now keyed on 0x0A, not
  // 0x00 (wire_runtime.h item 8), so an armored binary line may legitimately
  // contain an embedded 0x00 byte -- recovering its length via strlen()
  // would silently truncate it. Callers must pass an explicit length (a
  // std::string, or a uint8_t*/len pair) -- never `.c_str()`.
  void enqueueInboundBinary(const uint8_t* data, size_t len) { enqueueInbound(data, len); }
  void enqueueInboundBinary(const std::string& frame) { enqueueInbound(frame); }

  // Non-blocking: pops the oldest queued inbound line into buf and returns
  // true, or returns false immediately when the queue is empty --
  // buf/*outLen left untouched in that case. Matches
  // Hal::Transport::readLine()'s own documented contract exactly (buf is
  // always NUL-terminated in addition to *outLen being set -- see that
  // method's own doc comment).
  bool readLine(char* buf, uint16_t cap, uint16_t* outLen) override {
    if (inbound_.empty()) return false;
    std::string line = std::move(inbound_.front());
    inbound_.pop_front();
    uint16_t copy = (line.size() < static_cast<size_t>(cap - 1)) ? static_cast<uint16_t>(line.size())
                                                                  : static_cast<uint16_t>(cap - 1);
    std::memcpy(buf, line.data(), copy);
    buf[copy] = '\0';
    if (outLen) *outLen = copy;
    return true;
  }

  void send(const uint8_t* data, uint16_t len) override {
    sent_.emplace_back(reinterpret_cast<const char*>(data), len);
  }
  void sendReliable(const char* msg) override { sentReliable_.emplace_back(msg); }

  // Outbound captures -- a test drains/inspects these after stepping the
  // loop. Two SEPARATE captures, matching send() vs. sendReliable()'s
  // distinct call sites (Telemetry's primary/secondary frames ride send();
  // the HELLO/PING/ID/VER cleartext replies ride sendReliable()). send()'s
  // capture holds the raw `<COMMAND>':'<COBS+CRC bytes>` line (124-005) --
  // NOT NUL-terminated text -- constructed via the explicit-length
  // std::string ctor above.
  const std::deque<std::string>& sent() const { return sent_; }
  const std::deque<std::string>& sentReliable() const { return sentReliable_; }

  size_t inboundSize() const { return inbound_.size(); }

 private:
  std::deque<std::string> inbound_;
  std::deque<std::string> sent_;
  std::deque<std::string> sentReliable_;
};

}  // namespace TestSupport
