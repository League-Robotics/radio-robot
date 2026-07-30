// test_app_robot_loop_dedup_harness.cpp -- sprint 127 ticket 002's own
// off-hardware acceptance proof: verifies the `Move.id` dedup short-circuit
// App::RobotLoop::handleMove() already ships (alreadyAccepted()/
// recordAccepted()/acceptedMoveIds_, src/firm/app/robot_loop.cpp
// ~216-258), against all four rules the source issue's own Verification
// section names (clasi/sprints/127-host-side-path-planner-goto-and-path-
// following/issues/duplicate-move-enqueue-on-ack-loss-retry.md):
//
//   1. Ordinary duplicate suppressed -- a resent already-accepted non-zero
//      id does not enqueue twice; queue depth rises by exactly one across
//      both sends; both acks err==0; a DIFFERENT id still enqueues.
//   2. Move.id == 0 is exempt -- two id-0 moves both enqueue as distinct
//      entries (dedup never fires on the "unset" sentinel).
//   3. The window outlives completion -- a duplicate id arriving AFTER the
//      first instance has already run to completion is still suppressed
//      (the accepted-id ring is not evicted on queue-exit). Also exercises
//      the ordering subtlety the issue calls out explicitly: the id check
//      sits BEFORE any `replace` handling, so a duplicate carrying
//      replace=true is suppressed too, not honored as a real replace.
//   4. ERR_FULL rejections are not recorded -- a move rejected because the
//      queue was full (Motion::Planner::kQueueDepth == 5, "1 active + 4
//      pending") must be re-acceptable once the queue drains.
//
// CHARACTERIZATION, NOT A FIX -- this file writes tests against existing,
// already-shipped firmware behavior. It does not modify
// src/firm/app/robot_loop.{h,cpp}, any wire message, or any .proto
// definition; alreadyAccepted()/recordAccepted() are read-only references
// here, exactly as the ticket requires.
//
// Drives the real TestSim::SimHarness (src/sim/sim_harness.h) -- the dedup
// short-circuit lives one layer above Motion::Planner, in
// App::RobotLoop::handleMove() itself, so a bare Planner (the scaffolding
// test_app_robot_loop_replace_harness.cpp's cases 1-5 use) has no id memory
// whatsoever to exercise. Each scenario below gets its OWN fresh
// SimHarness, matching this tier's established convention (see e.g. cases
// 1-5 there, each with their own fresh Planner) -- no state leaks between
// rules.
//
// Mirrors this tier's established hand-rolled assertion plumbing (see
// motion_stop_condition_harness.cpp / app_robot_loop_harness.cpp /
// test_app_robot_loop_replace_harness.cpp): compile with the system C++
// compiler, run the resulting binary, assert it exits 0. Collected under
// src/tests/sim/unit/ -- already within pyproject.toml's
// testpaths = ["src/tests/sim"].
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "messages/envelope.h"
#include "motion/planner/planner.h"
#include "sim_harness.h"
#include "support/bench_test_config.h"

namespace {

using Motion::Planner;
using TestSupport::DecodedKind;
using TestSupport::DecodedLine;
using TestSupport::MoveStopKind;

// --- Hand-rolled assertion plumbing (see the other src/tests/sim/unit/
// *_harness.cpp files -- this tier's established convention: accumulate
// every failure and keep running, rather than CHECK()'s exit-on-first-
// failure, so one scenario's report is complete). ---------------------

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
  if (!condition) fail(what + " -- expected true, got false");
}

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %u, got %u", what.c_str(),
                  static_cast<unsigned>(expected), static_cast<unsigned>(actual));
    fail(buf);
  }
}

// --- Shared scenario helpers --------------------------------------------

// active() + pendingCount() -- the protocol-v5 5-deep queue's own total
// occupancy (planner.h's own kQueueDepth comment: "1 active + 4 pending").
int queueDepth(const Planner& p) { return (p.active() ? 1 : 0) + p.pendingCount(); }

std::vector<DecodedLine> onlyTelemetry(const std::vector<DecodedLine>& lines) {
  std::vector<DecodedLine> out;
  for (const auto& l : lines) {
    if (l.kind == DecodedKind::kTelemetry) out.push_back(l);
  }
  return out;
}

// anyAckMatches -- "was `corrId` ever acked OK (err==0) anywhere in
// `frames`" -- mirrors move_protocol_harness.cpp's own helper of the same
// name. Scans the bounded ack ring (packed uint32 corr_id<<4|err).
bool anyAckMatches(const std::vector<DecodedLine>& frames, uint32_t corrId) {
  constexpr uint32_t kAckErrBits = 4;
  constexpr uint32_t kAckErrMask = (1u << kAckErrBits) - 1;
  for (const auto& f : frames) {
    for (uint8_t i = 0; i < f.telemetry.acks_count; ++i) {
      const uint32_t packed = f.telemetry.acks_[i];
      if ((packed >> kAckErrBits) == corrId && (packed & kAckErrMask) == 0) return true;
    }
  }
  return false;
}

// findFreshAck -- the first ack ring entry in `frames` whose packed
// corr_id == `ackCorr`, regardless of err (unlike anyAckMatches() above).
// Every scenario below gives each Move a corrId/id pair that is unique
// across its OWN scenario, so a match against either an enqueue corr_id or
// a completion Move.id is unambiguous. Mirrors move_protocol_harness.cpp's
// own helper of the same name.
bool findFreshAck(const std::vector<DecodedLine>& frames, uint32_t ackCorr, uint32_t* errOut) {
  constexpr uint32_t kAckErrBits = 4;
  constexpr uint32_t kAckErrMask = (1u << kAckErrBits) - 1;
  for (const auto& f : frames) {
    for (uint8_t i = 0; i < f.telemetry.acks_count; ++i) {
      const uint32_t packed = f.telemetry.acks_[i];
      if ((packed >> kAckErrBits) != ackCorr) continue;
      if (errOut) *errOut = packed & kAckErrMask;
      return true;
    }
  }
  return false;
}

TestSim::SimHarness* freshBootedSim(TestSim::SimHarness* sim) {
  TestSupport::configureSimForBenchTest(*sim);
  sim->boot();
  sim->step(3);
  (void)sim->drainTelemetry();  // discard boot-preamble telemetry
  return sim;
}

// ===========================================================================
// Rule 1 -- ordinary duplicate suppressed. A resent already-accepted
// non-zero Move.id does not enqueue twice: queue depth rises by exactly
// one across BOTH sends of that id, both acks err==0 (acking success is
// deliberate -- the host's retry loop is waiting for an ack to clear
// `inflight`, and the move genuinely is enqueued/executing/complete), and
// a DIFFERENT id still enqueues normally afterward.
// ===========================================================================
void scenarioOrdinaryDuplicateSuppressed() {
  beginScenario("Rule 1: ordinary duplicate suppressed -- resent non-zero Move.id does not "
                "enqueue twice; a different id still enqueues");
  TestSim::SimHarness sim;
  freshBootedSim(&sim);

  constexpr uint32_t kMoveIdA = 800;
  constexpr uint32_t kCorrEnqueueA = 8001;
  constexpr uint32_t kCorrDuplicateA = 8002;
  constexpr uint32_t kMoveIdB = 801;
  constexpr uint32_t kCorrEnqueueB = 8003;

  // Move A: original accept.
  sim.injectMove(/*v_x=*/150.0f, /*v_y=*/0.0f, /*omega=*/0.0f, MoveStopKind::kDistance,
                 /*stopValue=*/5000.0f, /*timeout=*/60000.0f, /*replace=*/false, kMoveIdA,
                 kCorrEnqueueA);
  sim.step(3);
  std::vector<DecodedLine> framesA = onlyTelemetry(sim.drainTelemetry());
  checkTrue(anyAckMatches(framesA, kCorrEnqueueA), "Move A's own enqueue ack is OK (err==0)");
  checkTrue(sim.planner().active(), "Move A is active after its first accept");
  checkUintEq(sim.planner().activeMoveId(), kMoveIdA, "Move A's id is the one active");
  const int depthAfterA = queueDepth(sim.planner());
  checkUintEq(static_cast<uint32_t>(depthAfterA), 1u, "queue depth is 1 after the first accept of id A");

  // Resend the EXACT SAME id under a fresh corr_id -- the shape a retried
  // enqueue whose original ack was lost would carry (robot_loop.cpp's own
  // comment above alreadyAccepted()).
  sim.injectMove(/*v_x=*/150.0f, /*v_y=*/0.0f, /*omega=*/0.0f, MoveStopKind::kDistance,
                 /*stopValue=*/5000.0f, /*timeout=*/60000.0f, /*replace=*/false, kMoveIdA,
                 kCorrDuplicateA);
  sim.step(3);
  std::vector<DecodedLine> framesDup = onlyTelemetry(sim.drainTelemetry());
  checkTrue(anyAckMatches(framesDup, kCorrDuplicateA),
            "the duplicate resend's OWN corr_id still acks OK (err==0) -- success, not an error, "
            "so the host's retry loop clears inflight instead of aborting an already-executing move");
  const int depthAfterDup = queueDepth(sim.planner());
  checkUintEq(static_cast<uint32_t>(depthAfterDup), static_cast<uint32_t>(depthAfterA),
              "the duplicate resend does NOT raise queue depth further -- it did not enqueue a "
              "second time");
  checkUintEq(sim.planner().activeMoveId(), kMoveIdA, "still Move A active -- no second copy landed");

  // A genuinely different id still enqueues normally -- the dedup did not
  // globally jam the queue.
  sim.injectMove(/*v_x=*/150.0f, /*v_y=*/0.0f, /*omega=*/0.0f, MoveStopKind::kDistance,
                 /*stopValue=*/300.0f, /*timeout=*/60000.0f, /*replace=*/false, kMoveIdB,
                 kCorrEnqueueB);
  sim.step(3);
  std::vector<DecodedLine> framesB = onlyTelemetry(sim.drainTelemetry());
  checkTrue(anyAckMatches(framesB, kCorrEnqueueB), "a genuinely different id's enqueue ack is OK");
  const int depthAfterB = queueDepth(sim.planner());
  checkUintEq(static_cast<uint32_t>(depthAfterB), static_cast<uint32_t>(depthAfterA + 1),
              "a different id enqueues normally (+1 pending) -- confirms the dedup is keyed on id, "
              "not a blanket suppression");
  std::printf("  RULE1_DEPTH_AFTER_A=%d AFTER_DUP=%d AFTER_B=%d\n", depthAfterA, depthAfterDup,
              depthAfterB);
}

// ===========================================================================
// Rule 2 -- Move.id == 0 is exempt. Two moves both carrying id 0 must BOTH
// enqueue, as two DISTINCT pending entries -- dedup must never fire on the
// "unset / don't care" sentinel `move_id` defaults to in
// src/host/robot_radio/robot/protocol.py.
// ===========================================================================
void scenarioIdZeroExempt() {
  beginScenario("Rule 2: Move.id == 0 is exempt -- two id-0 moves both enqueue as distinct entries");
  TestSim::SimHarness sim;
  freshBootedSim(&sim);

  constexpr uint32_t kIdZero = 0;
  constexpr uint32_t kCorr1 = 8101;
  constexpr uint32_t kCorr2 = 8102;

  sim.injectMove(/*v_x=*/150.0f, /*v_y=*/0.0f, /*omega=*/0.0f, MoveStopKind::kDistance,
                 /*stopValue=*/5000.0f, /*timeout=*/60000.0f, /*replace=*/false, kIdZero, kCorr1);
  sim.step(3);
  std::vector<DecodedLine> frames1 = onlyTelemetry(sim.drainTelemetry());
  checkTrue(anyAckMatches(frames1, kCorr1), "first id-0 move's enqueue ack is OK");
  const int depthAfter1 = queueDepth(sim.planner());
  checkUintEq(static_cast<uint32_t>(depthAfter1), 1u, "queue depth is 1 after the first id-0 accept");

  // A SECOND id-0 move, replace=false -- if dedup fired on id 0 (the bug
  // this rule exists to catch), this would be silently suppressed and
  // depth would stay at 1.
  sim.injectMove(/*v_x=*/0.0f, /*v_y=*/0.0f, /*omega=*/2.0f, MoveStopKind::kAngle,
                 /*stopValue=*/1.57f, /*timeout=*/60000.0f, /*replace=*/false, kIdZero, kCorr2);
  sim.step(3);
  std::vector<DecodedLine> frames2 = onlyTelemetry(sim.drainTelemetry());
  checkTrue(anyAckMatches(frames2, kCorr2), "second id-0 move's enqueue ack is OK");
  const int depthAfter2 = queueDepth(sim.planner());
  checkUintEq(static_cast<uint32_t>(depthAfter2), 2u,
              "queue depth rises to 2 -- BOTH id-0 moves enqueued as distinct entries, confirming "
              "id 0 is never deduped (a deduped second send would have left depth at 1)");
  std::printf("  RULE2_DEPTH_AFTER_1=%d AFTER_2=%d\n", depthAfter1, depthAfter2);
}

// ===========================================================================
// Rule 3 -- the window outlives completion. A duplicate id arriving AFTER
// the first instance has already run to completion is still suppressed
// (the accepted-id ring is not evicted on queue-exit) -- the actual
// lost-ack case from the source issue, where the retry can land well after
// the original move finished. Also exercises the ordering subtlety the
// issue calls out: the id check sits BEFORE any `replace` handling, so a
// duplicate carrying replace=true is suppressed too, not honored as a real
// replace (a real replace here would look exactly like the run-1 runaway
// signature -- a fresh move activating on top of a completed one).
// ===========================================================================
void scenarioWindowOutlivesCompletion() {
  beginScenario("Rule 3: window outlives completion -- a duplicate id sent AFTER the first "
                "instance completed is still suppressed, even carrying replace=true");
  TestSim::SimHarness sim;
  freshBootedSim(&sim);

  constexpr uint32_t kMoveId = 900;
  constexpr uint32_t kCorrEnqueue = 9001;
  constexpr uint32_t kCorrResend = 9002;
  constexpr float kStopTime = 200.0f;  // [ms] short TIME stop -- 5 cycles @ RobotLoop::kCycle=40ms

  sim.injectMove(/*v_x=*/150.0f, /*v_y=*/0.0f, /*omega=*/0.0f, MoveStopKind::kTime, kStopTime,
                 /*timeout=*/5000.0f, /*replace=*/false, kMoveId, kCorrEnqueue);
  sim.step(3);
  std::vector<DecodedLine> framesEnqueue = onlyTelemetry(sim.drainTelemetry());
  checkTrue(anyAckMatches(framesEnqueue, kCorrEnqueue), "the original move's enqueue ack is OK");

  // Run well past the 200ms stop threshold so the move completes and drains.
  sim.step(20);  // 20 * 40ms = 800ms, comfortably past 200ms
  std::vector<DecodedLine> framesComplete = onlyTelemetry(sim.drainTelemetry());
  uint32_t completionErr = 1;
  checkTrue(findFreshAck(framesComplete, kMoveId, &completionErr),
            "the move's completion ack (packed corr_id == Move.id) reached the wire");
  checkUintEq(completionErr, 0u, "the completion ack's err is OK");
  checkTrue(!sim.planner().active(), "the planner is idle -- the move ran all the way to completion "
                                      "before the duplicate is sent");
  checkUintEq(static_cast<uint32_t>(queueDepth(sim.planner())), 0u, "queue is fully drained");

  // The duplicate, sent AFTER completion, carrying replace=true and wildly
  // different content (an Angle move instead of the original Distance
  // move) -- if the id check did not precede replace handling, or if the
  // accepted-id ring evicted on queue-exit, this would look exactly like a
  // fresh, legitimate move and activate.
  sim.injectMove(/*v_x=*/0.0f, /*v_y=*/0.0f, /*omega=*/3.0f, MoveStopKind::kAngle,
                 /*stopValue=*/3.14159265f, /*timeout=*/60000.0f, /*replace=*/true, kMoveId,
                 kCorrResend);
  sim.step(3);
  std::vector<DecodedLine> framesResend = onlyTelemetry(sim.drainTelemetry());
  checkTrue(anyAckMatches(framesResend, kCorrResend),
            "the post-completion duplicate's own corr_id still acks OK (err==0)");
  checkTrue(!sim.planner().active(),
            "the planner is STILL idle -- the post-completion duplicate did not activate a new move, "
            "even though it carried replace=true (the id check precedes replace handling)");
  checkUintEq(static_cast<uint32_t>(queueDepth(sim.planner())), 0u,
              "queue depth is still 0 -- nothing was enqueued by the duplicate resend");
}

// ===========================================================================
// Rule 4 -- ERR_FULL rejections are not recorded. A move rejected because
// the queue was full (Motion::Planner::kQueueDepth == 5, "1 active + 4
// pending") must NOT be recorded as accepted -- the host is entitled to
// retry it for real once the queue drains, and it must succeed then.
// ===========================================================================
void scenarioErrFullNotRecorded() {
  beginScenario("Rule 4: ERR_FULL rejections are not recorded -- a queue-full-rejected id "
                "enqueues normally once the queue drains");
  TestSim::SimHarness sim;
  freshBootedSim(&sim);

  // Fill the queue to its full 5-slot capacity (1 active + 4 pending) with
  // five short, DISTINCT-id TIME moves -- short so the queue naturally
  // drains later in this same scenario. All six commands below are
  // injected BEFORE a single step() call: sim_harness.h's own contract is
  // that every line injected before one step() is consumed within THAT
  // step, in order -- so move 6 is routed against a queue already filled
  // by moves 1-5, exactly as a real back-to-back burst would be.
  constexpr float kEachStopTime = 120.0f;  // [ms] short enough to drain within this scenario
  constexpr float kTimeout = 5000.0f;        // [ms]
  const uint32_t kFillIds[5] = {950, 951, 952, 953, 954};
  const uint32_t kFillCorrs[5] = {9501, 9502, 9503, 9504, 9505};
  for (int i = 0; i < 5; ++i) {
    sim.injectMove(/*v_x=*/150.0f, /*v_y=*/0.0f, /*omega=*/0.0f, MoveStopKind::kTime,
                   kEachStopTime, kTimeout, /*replace=*/false, kFillIds[i], kFillCorrs[i]);
  }
  constexpr uint32_t kRejectedId = 955;
  constexpr uint32_t kCorrReject = 9506;
  sim.injectMove(/*v_x=*/150.0f, /*v_y=*/0.0f, /*omega=*/0.0f, MoveStopKind::kTime, kEachStopTime,
                 kTimeout, /*replace=*/false, kRejectedId, kCorrReject);
  sim.step(3);

  std::vector<DecodedLine> framesFill = onlyTelemetry(sim.drainTelemetry());
  for (int i = 0; i < 5; ++i) {
    checkTrue(anyAckMatches(framesFill, kFillCorrs[i]),
              "fill move's enqueue ack is OK (id " + std::to_string(kFillIds[i]) + ")");
  }
  checkUintEq(static_cast<uint32_t>(queueDepth(sim.planner())), 5u,
              "queue is at full capacity (1 active + 4 pending) after the five fill moves");
  checkUintEq(sim.planner().activeMoveId(), kFillIds[0], "the first fill move is still the active one");

  uint32_t rejectErr = 0;
  checkTrue(findFreshAck(framesFill, kCorrReject, &rejectErr),
            "the 6th move's own corr_id shows up in the ack ring");
  checkUintEq(rejectErr, static_cast<uint32_t>(msg::ErrCode::ERR_FULL),
              "the 6th move is rejected with ERR_FULL -- the queue was genuinely full");

  // Let all five fill moves run to completion and drain (5 * 120ms = 600ms;
  // run comfortably longer).
  sim.step(40);  // 40 * 40ms = 1.6s
  (void)sim.drainTelemetry();
  checkTrue(!sim.planner().active(), "the planner is idle -- all five fill moves completed and drained");
  checkUintEq(static_cast<uint32_t>(queueDepth(sim.planner())), 0u, "queue is fully drained");

  // Re-send the id that was ERR_FULL-rejected. If it had been (wrongly)
  // recorded as accepted at rejection time, this resend would now be
  // deduped -- acked OK but never actually enqueued, and the planner would
  // stay idle. It must instead enqueue for real.
  constexpr uint32_t kCorrRetry = 9507;
  sim.injectMove(/*v_x=*/150.0f, /*v_y=*/0.0f, /*omega=*/0.0f, MoveStopKind::kTime, kEachStopTime,
                 kTimeout, /*replace=*/false, kRejectedId, kCorrRetry);
  sim.step(3);
  std::vector<DecodedLine> framesRetry = onlyTelemetry(sim.drainTelemetry());
  uint32_t retryErr = 1;
  checkTrue(findFreshAck(framesRetry, kCorrRetry, &retryErr),
            "the retried id's own corr_id shows up in the ack ring");
  checkUintEq(retryErr, 0u,
              "the retried id (previously ERR_FULL-rejected) now acks OK -- it was never recorded "
              "as accepted, so this is a genuine fresh accept, not a dedup no-op");
  checkTrue(sim.planner().active(), "the planner is active again -- the retried move genuinely enqueued");
  checkUintEq(sim.planner().activeMoveId(), kRejectedId,
              "the previously-rejected id is the one now active -- confirms real re-acceptance, not "
              "a dedup short-circuit masquerading as success");
  checkUintEq(static_cast<uint32_t>(queueDepth(sim.planner())), 1u, "queue depth is 1 after the real re-accept");
}

}  // namespace

int main() {
  scenarioOrdinaryDuplicateSuppressed();
  scenarioIdZeroExempt();
  scenarioWindowOutlivesCompletion();
  scenarioErrFullNotRecorded();

  if (g_failureCount == 0) {
    std::printf("OK: all 127-002 Move.id dedup verification scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the 127-002 dedup verification scenarios\n",
              g_failureCount);
  return 1;
}
