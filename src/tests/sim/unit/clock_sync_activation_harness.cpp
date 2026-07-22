// clock_sync_activation_harness.cpp -- off-hardware acceptance harness for
// sprint 117 ticket 001 (SUC-056)'s host-side activation proof. NOT a
// scenario-assertion harness like the other *_harness.cpp files in this
// directory -- this one has no assertions of its own. Its ONLY job is to
// exercise the REAL, compiled App::Comms::pumpTransport() PING handler (the
// exact same comms.cpp this sprint's app_comms_harness.cpp/test_app_comms.py
// pair also compiles) for a burst of PING exchanges, printing each reply
// line verbatim to stdout, one per line -- so a Python test
// (test_clock_sync_activation.py) can feed those REAL firmware-formatted
// reply lines into the host's own robot_radio.robot.clock_sync.ClockSync
// through its public ping_burst(send_fn) API and prove best_offset() goes
// non-None. This is the thing AC3 (001's own Acceptance Criteria) asks for:
// "drives a real ClockSync instance's ping_burst() against the firmware's
// new reply ... not just parse a hand-written fixture string" -- the reply
// text below is never hand-typed in Python, it comes out of the same
// Comms::pumpTransport() code path robot_loop.cpp calls every cycle.
//
// `now` advances by 20ms (kCycle, robot_loop.cpp) between each simulated
// PING, mirroring how the real robot's clock actually advances between
// cycles -- proves the `t=` field is a genuine per-call reading, not a
// frozen constant, the same property scenarioPingRepliesOkPongViaSendReliable
// (app_comms_harness.cpp) proves for a single call.
//
// Compiled by test_clock_sync_activation.py with -DHOST_BUILD against
// comms.cpp, wire.cpp, wire_runtime.cpp -- the exact same dependency set
// test_app_comms.py already uses (no new HOST_BUILD wiring introduced).
#include <cstdint>
#include <cstdio>

#include "app/comms.h"
#include "support/fake_transport.h"

using TestSupport::FakeTransport;

namespace {

constexpr int kPingCount = 5;         // matches ClockSync.ping_burst()'s own default n=5
constexpr uint32_t kCycleMs = 20;     // [ms] robot_loop.cpp's kCycle -- simulated inter-ping spacing
constexpr uint32_t kStartNowMs = 1000;  // [ms] arbitrary nonzero robot-clock starting point

}  // namespace

int main() {
  FakeTransport serialFake;
  FakeTransport radioFake;
  static char banner[] = "DEVICE:NEZHA2:robot:sim:1234";
  App::Comms comms(serialFake, radioFake, banner);

  uint32_t now = kStartNowMs;
  for (int i = 0; i < kPingCount; ++i) {
    serialFake.enqueueInbound("PING");

    App::Cmd cmd;
    comms.pump(cmd, now);

    // One real "OK pong t=<now>" reply per PING -- print it verbatim so the
    // Python side can feed it straight into ClockSync.ping_burst()'s own
    // send_fn contract (Callable[[str], str | None]) without touching or
    // re-deriving the reply text itself.
    if (!serialFake.sentReliable().empty()) {
      std::printf("%s\n", serialFake.sentReliable().back().c_str());
    } else {
      std::printf("\n");  // should be unreachable -- PING always replies
    }

    now += kCycleMs;
  }

  return 0;
}
