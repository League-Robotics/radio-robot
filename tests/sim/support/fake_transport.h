// fake_transport.h -- HOST_BUILD-only App::Transport double: an in-memory,
// FIFO-based fake a test can push armored "*B..." command lines into (so
// App::Comms::pump() reads them exactly as if from a real serial/radio
// line) and read captured outbound armored lines back out of (every
// App::Comms::sendReply()/App::Telemetry emit call). Ticket 105-002
// (SUC-019); mirrors comms.h's own documentation density/style.
//
// This is the ONE canonical FakeTransport for tests/sim/ -- several
// harnesses previously carried their own ad hoc copy (app_comms_harness.cpp,
// app_telemetry_harness.cpp, the latter also had a second QueueableFake
// Transport variant purely to add a queue where its base fake had none).
// Both are now built on this single class; later tickets (004's sim_api,
// 006's pytest scenarios) build on it too, per architecture-update.md
// Step 3's "FakeTransport (test-only, e.g. tests/sim/support/
// fake_transport.h)" entry.
//
// Design: two SEPARATE concerns, matching Transport's own two directions.
//   - Inbound: a test calls enqueueInbound() to script complete armored
//     lines; readLine() pops at most one per call (matches Comms::pump()'s
//     own "at most one line per call" contract) and returns false, non-
//     blocking, the instant the queue is empty -- never populating buf.
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
#include <deque>
#include <string>

#include "app/comms.h"

namespace TestSupport {

class FakeTransport : public App::Transport {
 public:
  // Push one complete armored line into the inbound FIFO; readLine() pops
  // it (oldest first) on a later call.
  void enqueueInbound(const char* line) { inbound_.emplace_back(line); }

  // Non-blocking: pops the oldest queued inbound line into buf (NUL-
  // terminated) and returns true, or returns false immediately when the
  // queue is empty -- buf is left untouched in that case. Matches
  // Transport::readLine()'s own documented contract exactly.
  bool readLine(char* buf, uint16_t len) override {
    if (inbound_.empty()) return false;
    std::string line = std::move(inbound_.front());
    inbound_.pop_front();
    std::snprintf(buf, len, "%s", line.c_str());
    return true;
  }

  void send(const char* msg) override { sent_.emplace_back(msg); }
  void sendReliable(const char* msg) override { sentReliable_.emplace_back(msg); }

  // Outbound captures -- a test drains/inspects these after stepping the
  // loop. Two SEPARATE captures, matching send() vs. sendReliable()'s
  // distinct call sites (Telemetry's primary/secondary frames ride send();
  // the HELLO/PING text-plane replies ride sendReliable()).
  const std::deque<std::string>& sent() const { return sent_; }
  const std::deque<std::string>& sentReliable() const { return sentReliable_; }

  size_t inboundSize() const { return inbound_.size(); }

 private:
  std::deque<std::string> inbound_;
  std::deque<std::string> sent_;
  std::deque<std::string> sentReliable_;
};

}  // namespace TestSupport
