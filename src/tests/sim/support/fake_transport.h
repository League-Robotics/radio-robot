// fake_transport.h -- HOST_BUILD-only App::Transport double: an in-memory,
// FIFO-based fake a test can push complete inbound frames into (so
// App::Comms::pump() reads them exactly as if from a real serial/radio
// line) and read captured outbound frames back out of (every
// App::Comms::sendReply()/App::Telemetry emit call). Ticket 105-002
// (SUC-019); mirrors comms.h's own documentation density/style.
//
// 123-002 (COBS+CRC framer integration): the wire's armor changed from
// text "*B<base64>\r\n" lines to a binary-clean, 0x00-delimited COBS+CRC
// frame demuxed from the HELLO/PING text rump on the same stream --
// App::Transport::readLine() now reports which FrameKind it delivered,
// and App::Transport::send() takes an explicit uint8_t*/len (no longer a
// NUL-terminated C string). This fake's queues/captures follow suit:
// enqueueInbound() (text, unchanged) gets a binary sibling
// (enqueueInboundBinary()), and sent()/sentReliable() hold std::string
// built from an explicit-length constructor so embedded arbitrary bytes
// (including 0x00, which a COBS+CRC frame never contains but this fake
// makes no assumption either way) survive the capture intact.
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
//     script complete frames; readLine() pops at most one per call
//     (matches Comms::pump()'s own "at most one line per call" contract)
//     and returns App::FrameKind::kNone, non-blocking, the instant the
//     queue is empty -- never populating buf.
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
#include <utility>

#include "app/comms.h"

namespace TestSupport {

class FakeTransport : public App::Transport {
 public:
  // Push one complete TEXT line into the inbound FIFO (HELLO/PING) --
  // readLine() pops it (oldest first, App::FrameKind::kText) on a later
  // call. Unchanged from pre-123.
  void enqueueInbound(const char* line) { inbound_.emplace_back(App::FrameKind::kText, std::string(line)); }

  // Push one complete BINARY frame body into the inbound FIFO -- the
  // still-COBS+CRC-encoded bytes a real transport would deliver with the
  // trailing 0x00 delimiter already stripped (matching
  // App::Transport::readLine()'s own kBinary contract). readLine() pops
  // it (oldest first, App::FrameKind::kBinary) on a later call.
  void enqueueInboundBinary(const uint8_t* data, size_t len) {
    inbound_.emplace_back(App::FrameKind::kBinary,
                           std::string(reinterpret_cast<const char*>(data), len));
  }

  // Convenience overload for a std::string-held frame (the shape
  // TestSupport::armor()/armorMoveCommand()/armorStopCommand() return) --
  // uses the string's own explicit size(), safe even for a deliberately
  // malformed test frame that embeds a literal 0x00 (a fault-injection
  // scenario constructing bad COBS/CRC bytes on purpose), unlike a
  // strlen()-based recovery which would silently truncate at the first
  // embedded 0x00.
  void enqueueInboundBinary(const std::string& frame) {
    enqueueInboundBinary(reinterpret_cast<const uint8_t*>(frame.data()), frame.size());
  }

  // Convenience overload for a NUL-terminated binary frame (safe because
  // COBS-encoded content is 0x00-free by construction, per
  // WireRuntime::cobsEncode()'s own contract) -- lets a caller holding a
  // bare `const char*`/`.c_str()` push it without recomputing its own
  // length. Prefer the std::string overload above when the frame might be
  // deliberately malformed (may embed a literal 0x00).
  void enqueueInboundBinary(const char* frame) {
    enqueueInboundBinary(reinterpret_cast<const uint8_t*>(frame), std::strlen(frame));
  }

  // Non-blocking: pops the oldest queued inbound frame into buf and
  // returns its FrameKind, or returns App::FrameKind::kNone immediately
  // when the queue is empty -- buf/*outLen left untouched in that case.
  // Matches App::Transport::readLine()'s own documented contract exactly
  // (buf is always NUL-terminated in addition to *outLen being set, safe
  // for both frame kinds -- see that method's own doc comment).
  App::FrameKind readLine(char* buf, uint16_t cap, uint16_t* outLen) override {
    if (inbound_.empty()) return App::FrameKind::kNone;
    std::pair<App::FrameKind, std::string> entry = std::move(inbound_.front());
    inbound_.pop_front();
    const std::string& line = entry.second;
    uint16_t copy = (line.size() < static_cast<size_t>(cap - 1)) ? static_cast<uint16_t>(line.size())
                                                                  : static_cast<uint16_t>(cap - 1);
    std::memcpy(buf, line.data(), copy);
    buf[copy] = '\0';
    if (outLen) *outLen = copy;
    return entry.first;
  }

  void send(const uint8_t* data, uint16_t len) override {
    sent_.emplace_back(reinterpret_cast<const char*>(data), len);
  }
  void sendReliable(const char* msg) override { sentReliable_.emplace_back(msg); }

  // Outbound captures -- a test drains/inspects these after stepping the
  // loop. Two SEPARATE captures, matching send() vs. sendReliable()'s
  // distinct call sites (Telemetry's primary/secondary frames ride send();
  // the HELLO/PING text-plane replies ride sendReliable()). send()'s
  // capture holds raw COBS+CRC frame bytes (123-002) -- NOT NUL-terminated
  // text -- constructed via the explicit-length std::string ctor above.
  const std::deque<std::string>& sent() const { return sent_; }
  const std::deque<std::string>& sentReliable() const { return sentReliable_; }

  size_t inboundSize() const { return inbound_.size(); }

 private:
  std::deque<std::pair<App::FrameKind, std::string>> inbound_;
  std::deque<std::string> sent_;
  std::deque<std::string> sentReliable_;
};

}  // namespace TestSupport
