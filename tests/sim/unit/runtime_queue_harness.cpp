// runtime_queue_harness.cpp — off-hardware acceptance harness for ticket
// 087-001 (SUC-001/SUC-002/SUC-006): exercises Rt::Mailbox<T> (capacity-1,
// latest-wins) and Rt::WorkQueue<T, N> (FIFO, capacity N) directly against a
// plain POD test type AND a real msg:: type (msg::MotorCommand) — no fakes
// needed, both templates are dependency-free beyond <cstdint>.
//
// Mirrors drivetrain_harness.cpp's/motor_policy_harness.cpp's shape exactly
// (see either file's header for the pattern): #includes only
// source/runtime/queue.h and messages/motor.h (both dependency-free — no
// MicroBit.h, no I2CBus), compiles with the plain system C++ compiler — no
// CMake, no ARM toolchain. Hand-rolled assertions, prints PASS/FAIL, exits
// nonzero on any failure. Run by test_runtime_queue.py, which compiles and
// runs this binary via subprocess.

#include <cstdint>
#include <cstdio>
#include <string>

#include "messages/motor.h"
#include "runtime/queue.h"

namespace {

// --- Hand-rolled assertion plumbing (same tiny shape as
// drivetrain_harness.cpp/motor_policy_harness.cpp -- a handful of scenarios
// do not warrant a test framework dependency for a dependency-free host
// harness). ---

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

void checkFalse(bool condition, const std::string& what) {
  if (condition) fail(what + " -- expected false, got true");
}

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %u, got %u", what.c_str(),
                  static_cast<unsigned>(expected), static_cast<unsigned>(actual));
    fail(buf);
  }
}

// --- Plain POD test type (no msg:: dependency at all) ---
//
// Note: default member initializers make this a non-aggregate under C++11
// (the aggregate rules only started permitting them in C++14, and this
// harness is compiled to C++11 — the firmware's own target, see
// test_runtime_queue.py). An explicit two-arg constructor is added so the
// scenarios below can still write the terse PodPayload{tag, value} form.
struct PodPayload {
  int tag = 0;
  float value = 0.0f;

  PodPayload() = default;
  PodPayload(int tagIn, float valueIn) : tag(tagIn), value(valueIn) {}
};

bool podEq(const PodPayload& a, const PodPayload& b) {
  return a.tag == b.tag && a.value == b.value;
}

void checkPodEq(const PodPayload& actual, const PodPayload& expected,
                 const std::string& what) {
  if (!podEq(actual, expected)) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "%s -- expected {tag=%d, value=%g}, got {tag=%d, value=%g}",
                  what.c_str(), expected.tag, static_cast<double>(expected.value),
                  actual.tag, static_cast<double>(actual.value));
    fail(buf);
  }
}

// --- Rt::Mailbox<PodPayload> scenarios ---

// 1. A freshly constructed Mailbox is empty; take() on an empty mailbox is
// well-defined -- returns a default-constructed T and stays empty.
void scenarioMailboxEmptyAndDefaultTake() {
  beginScenario("Mailbox<PodPayload>: starts empty, take() on empty is well-defined");
  Rt::Mailbox<PodPayload> mbox;
  checkTrue(mbox.empty(), "freshly constructed mailbox starts empty");

  PodPayload taken = mbox.take();
  checkPodEq(taken, PodPayload{}, "take() on empty mailbox returns default-constructed T");
  checkTrue(mbox.empty(), "mailbox stays empty after a take() on empty");
}

// 2. post() marks the mailbox full; take() returns the posted value and
// clears the full flag (empty again).
void scenarioMailboxPostThenTakeClears() {
  beginScenario("Mailbox<PodPayload>: post() fills, take() returns value and clears");
  Rt::Mailbox<PodPayload> mbox;

  mbox.post(PodPayload{1, 10.0f});
  checkFalse(mbox.empty(), "post() marks the mailbox non-empty");

  PodPayload taken = mbox.take();
  checkPodEq(taken, PodPayload{1, 10.0f}, "take() returns the posted value");
  checkTrue(mbox.empty(), "take() clears the full flag");
}

// 3. post() overwrites any unread value -- latest-wins. A second post()
// before any take() replaces the first; take() only ever sees the latest.
void scenarioMailboxLatestWinsOverwrite() {
  beginScenario("Mailbox<PodPayload>: post() overwrites an unread value (latest-wins)");
  Rt::Mailbox<PodPayload> mbox;

  mbox.post(PodPayload{1, 1.0f});
  mbox.post(PodPayload{2, 2.0f});   // overwrites the unread first post
  mbox.post(PodPayload{3, 3.0f});   // overwrites again

  checkFalse(mbox.empty(), "mailbox is full after three posts");
  PodPayload taken = mbox.take();
  checkPodEq(taken, PodPayload{3, 3.0f}, "take() returns only the latest posted value");
  checkTrue(mbox.empty(), "mailbox empty after the single take()");
}

// 4. Mailbox<msg::MotorCommand> -- a real msg:: type, proving the template
// compiles and behaves identically with a non-POD payload carrying a
// setter/union (the acceptance criterion's "at least one real msg:: type").
void scenarioMailboxRealMsgType() {
  beginScenario("Mailbox<msg::MotorCommand>: real msg:: type compiles and round-trips");
  Rt::Mailbox<msg::MotorCommand> mbox;
  checkTrue(mbox.empty(), "freshly constructed Mailbox<msg::MotorCommand> starts empty");

  msg::MotorCommand cmd;
  cmd.setVelocity(42.0f);
  mbox.post(cmd);
  checkFalse(mbox.empty(), "post() marks the mailbox non-empty");

  msg::MotorCommand taken = mbox.take();
  checkTrue(taken.control_kind == msg::MotorCommand::ControlKind::VELOCITY,
            "taken command retains its VELOCITY control_kind");
  checkTrue(taken.control.velocity == 42.0f, "taken command retains its velocity value");
  checkTrue(mbox.empty(), "mailbox empty after take()");
}

// --- Rt::WorkQueue<PodPayload, N> scenarios ---

// 5. post() appends in FIFO order; take() pops front-first, in the order
// posted; size()/empty() are accurate throughout.
void scenarioWorkQueueFifoOrder() {
  beginScenario("WorkQueue<PodPayload,4>: post()/take() preserve FIFO order");
  Rt::WorkQueue<PodPayload, 4> q;
  checkTrue(q.empty(), "freshly constructed queue starts empty");
  checkUintEq(q.size(), 0, "freshly constructed queue has size 0");

  checkTrue(q.post(PodPayload{1, 1.0f}), "post() #1 succeeds (queue has room)");
  checkTrue(q.post(PodPayload{2, 2.0f}), "post() #2 succeeds (queue has room)");
  checkTrue(q.post(PodPayload{3, 3.0f}), "post() #3 succeeds (queue has room)");
  checkUintEq(q.size(), 3, "size() reflects three queued elements");
  checkFalse(q.empty(), "queue is non-empty with elements queued");

  checkPodEq(q.take(), PodPayload{1, 1.0f}, "take() #1 returns the first-posted element");
  checkPodEq(q.take(), PodPayload{2, 2.0f}, "take() #2 returns the second-posted element");
  checkPodEq(q.take(), PodPayload{3, 3.0f}, "take() #3 returns the third-posted element");
  checkTrue(q.empty(), "queue empty after draining every posted element");
  checkUintEq(q.size(), 0, "size() is 0 after draining every posted element");
}

// 6. post() returns false when at capacity N -- never silently overwrites
// or drops without signaling full; the existing contents are unaffected by
// the rejected post.
void scenarioWorkQueueFullRejectsPost() {
  beginScenario("WorkQueue<PodPayload,3>: post() returns false at capacity, no overwrite");
  Rt::WorkQueue<PodPayload, 3> q;

  checkTrue(q.post(PodPayload{1, 1.0f}), "post() #1 succeeds");
  checkTrue(q.post(PodPayload{2, 2.0f}), "post() #2 succeeds");
  checkTrue(q.post(PodPayload{3, 3.0f}), "post() #3 succeeds (queue now at capacity)");
  checkUintEq(q.size(), 3, "size() == capacity after filling the queue");

  checkFalse(q.post(PodPayload{4, 4.0f}), "post() #4 returns false -- queue is full");
  checkUintEq(q.size(), 3, "size() unchanged after a rejected post()");

  // The rejected post must not have silently overwritten anything -- the
  // front element is still the original first post.
  checkPodEq(q.take(), PodPayload{1, 1.0f},
             "front element after the rejected post is still the original first post");

  // Now there is room again -- a subsequent post() succeeds.
  checkTrue(q.post(PodPayload{5, 5.0f}), "post() succeeds again once room is freed by a take()");
  checkUintEq(q.size(), 3, "size() back at capacity after the freed-then-refilled slot");
}

// 7. peek(i) is non-destructive and matches take()'s eventual order --
// repeated peek() calls do not change size()/empty(), and peek() at every
// index in [0, size()) matches what take() would return if called that
// many times in a row.
void scenarioWorkQueuePeekNonDestructive() {
  beginScenario("WorkQueue<PodPayload,4>: peek(i) is non-destructive and order-matching");
  Rt::WorkQueue<PodPayload, 4> q;
  q.post(PodPayload{10, 10.0f});
  q.post(PodPayload{20, 20.0f});
  q.post(PodPayload{30, 30.0f});

  // peek() at every valid index, twice each, to prove it never mutates state.
  for (int pass = 0; pass < 2; ++pass) {
    const PodPayload* p0 = q.peek(0);
    const PodPayload* p1 = q.peek(1);
    const PodPayload* p2 = q.peek(2);
    checkTrue(p0 != nullptr, "peek(0) is non-null while an element exists at index 0");
    checkTrue(p1 != nullptr, "peek(1) is non-null while an element exists at index 1");
    checkTrue(p2 != nullptr, "peek(2) is non-null while an element exists at index 2");
    if (p0) checkPodEq(*p0, PodPayload{10, 10.0f}, "peek(0) matches the front element");
    if (p1) checkPodEq(*p1, PodPayload{20, 20.0f}, "peek(1) matches the middle element");
    if (p2) checkPodEq(*p2, PodPayload{30, 30.0f}, "peek(2) matches the back element");
    checkUintEq(q.size(), 3, "peek() never changes size()");
  }

  // Out-of-range peek() returns nullptr, does not crash, does not mutate.
  checkTrue(q.peek(3) == nullptr, "peek(size()) is out of range -- returns nullptr");
  checkTrue(q.peek(100) == nullptr, "peek() far out of range -- returns nullptr");
  checkUintEq(q.size(), 3, "size() unchanged after out-of-range peek() calls");

  // Now prove peek()'s order matches take()'s eventual order exactly.
  checkPodEq(q.take(), PodPayload{10, 10.0f}, "take() #1 matches what peek(0) reported");
  checkPodEq(q.take(), PodPayload{20, 20.0f}, "take() #2 matches what peek(1) reported (pre-take)");
  checkPodEq(q.take(), PodPayload{30, 30.0f}, "take() #3 matches what peek(2) reported (pre-take)");
}

// 8. size()/empty() stay accurate under an interleaved sequence of
// post()/take() calls, including wraparound past the underlying ring
// buffer's physical end (proves the index arithmetic, not just a
// fill-then-drain-once sequence).
void scenarioWorkQueueSizeAccountingInterleaved() {
  beginScenario("WorkQueue<PodPayload,3>: size()/empty() accurate under interleaved post/take, with wraparound");
  Rt::WorkQueue<PodPayload, 3> q;

  q.post(PodPayload{1, 1.0f});
  q.post(PodPayload{2, 2.0f});
  checkUintEq(q.size(), 2, "size() == 2 after two posts");

  checkPodEq(q.take(), PodPayload{1, 1.0f}, "take() drains the first element");
  checkUintEq(q.size(), 1, "size() == 1 after one take()");

  // Post two more -- tail_ wraps past the physical end of the 3-slot buffer
  // (head_ advanced to 1, so posting #3 and #4 wraps tail_ back to 0, then 1).
  q.post(PodPayload{3, 3.0f});
  q.post(PodPayload{4, 4.0f});
  checkUintEq(q.size(), 3, "size() == 3 after posting up to capacity across a wraparound");
  checkFalse(q.post(PodPayload{5, 5.0f}), "post() at capacity (post-wraparound) still returns false");

  checkPodEq(q.take(), PodPayload{2, 2.0f}, "take() after wraparound still returns in FIFO order (#2)");
  checkPodEq(q.take(), PodPayload{3, 3.0f}, "take() after wraparound still returns in FIFO order (#3)");
  checkUintEq(q.size(), 1, "size() == 1 with only #4 remaining");
  checkFalse(q.empty(), "queue non-empty with one element remaining");

  checkPodEq(q.take(), PodPayload{4, 4.0f}, "final take() drains the last remaining element (#4)");
  checkTrue(q.empty(), "queue empty after draining every element");
  checkUintEq(q.size(), 0, "size() == 0 once fully drained");

  // take() on a drained (empty) queue is well-defined too.
  PodPayload takenEmpty = q.take();
  checkPodEq(takenEmpty, PodPayload{}, "take() on an empty queue returns a default-constructed T");
  checkTrue(q.empty(), "queue stays empty after a take() on empty");
}

// 9. WorkQueue<msg::MotorCommand, N> -- a real msg:: type, proving the
// template compiles and behaves identically with a non-POD payload (the
// acceptance criterion's "at least one real msg:: type").
void scenarioWorkQueueRealMsgType() {
  beginScenario("WorkQueue<msg::MotorCommand,2>: real msg:: type compiles and preserves FIFO order");
  Rt::WorkQueue<msg::MotorCommand, 2> q;
  checkTrue(q.empty(), "freshly constructed WorkQueue<msg::MotorCommand,2> starts empty");

  msg::MotorCommand a;
  a.setVelocity(11.0f);
  msg::MotorCommand b;
  b.setDutyCycle(0.5f);

  checkTrue(q.post(a), "post() #1 succeeds");
  checkTrue(q.post(b), "post() #2 succeeds (queue now at capacity 2)");
  checkFalse(q.post(msg::MotorCommand{}), "post() #3 returns false -- queue at capacity");
  checkUintEq(q.size(), 2, "size() == 2 at capacity");

  const msg::MotorCommand* front = q.peek(0);
  checkTrue(front != nullptr, "peek(0) non-null on a non-empty queue");
  if (front) {
    checkTrue(front->control_kind == msg::MotorCommand::ControlKind::VELOCITY,
              "peek(0) reports the first-posted command's VELOCITY kind");
  }

  msg::MotorCommand takenA = q.take();
  checkTrue(takenA.control_kind == msg::MotorCommand::ControlKind::VELOCITY,
            "take() #1 returns the first-posted (VELOCITY) command");
  checkTrue(takenA.control.velocity == 11.0f, "take() #1 retains its velocity value");

  msg::MotorCommand takenB = q.take();
  checkTrue(takenB.control_kind == msg::MotorCommand::ControlKind::DUTY_CYCLE,
            "take() #2 returns the second-posted (DUTY_CYCLE) command");
  checkTrue(takenB.control.duty_cycle == 0.5f, "take() #2 retains its duty_cycle value");

  checkTrue(q.empty(), "queue empty after draining both posted commands");
}

}  // namespace

int main() {
  scenarioMailboxEmptyAndDefaultTake();
  scenarioMailboxPostThenTakeClears();
  scenarioMailboxLatestWinsOverwrite();
  scenarioMailboxRealMsgType();
  scenarioWorkQueueFifoOrder();
  scenarioWorkQueueFullRejectsPost();
  scenarioWorkQueuePeekNonDestructive();
  scenarioWorkQueueSizeAccountingInterleaved();
  scenarioWorkQueueRealMsgType();

  if (g_failureCount == 0) {
    std::printf("OK: all Rt::Mailbox/Rt::WorkQueue scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Rt::Mailbox/Rt::WorkQueue scenarios\n",
              g_failureCount);
  return 1;
}
