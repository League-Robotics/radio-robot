// queue.h -- Rt::Mailbox<T> / Rt::WorkQueue<T, N>: the two command-plane
// transport primitives sprint 087's Blackboard is built from (see
// clasi/sprints/087-two-plane-blackboard-synchronous-update-loop-
// configurator-and-command-queue-transport-greenfield/architecture-update.md,
// "The two command-plane primitives").
//
// The design's payload taxonomy is fixed to exactly two disciplines:
//   - Mailbox<T>    -- capacity 1, latest-wins. For ABSOLUTE setpoints: an
//                      unread older value is pure staleness, so post()
//                      overwrites it.
//   - WorkQueue<T,N> -- FIFO, capacity N. For DELTAS/commands that must all
//                      apply, in order. post() returns false when full so
//                      the caller can decide drop-vs-signal (never silently
//                      overwrites or drops without telling the caller).
//
// Both templates are dependency-free beyond <cstdint>: no MicroBit.h, no
// I2CBus, no msg:: types required to compile the template itself -- any
// trivially-copyable payload works, including a plain POD test type or a
// real msg:: type (see tests/sim/unit/runtime_queue_harness.cpp). No heap
// allocation: WorkQueue holds a fixed-size array member, Mailbox a single
// value member (docs/architecture/architecture-034.md SS11's
// no-heap-in-hot-path constraint).
//
// This ticket (087-001) creates only these two templates -- no Blackboard,
// no subsystem wiring, no command-family code (see the ticket's
// Description).
#pragma once

#include <cstdint>

namespace Rt {

// Mailbox<T> -- capacity 1, latest-wins. For ABSOLUTE setpoints: an unread
// older value is pure staleness, so post() overwrites it.
template <typename T>
class Mailbox {
 public:
  // Overwrites any unread value and marks the mailbox full.
  void post(const T& value) {
    value_ = value;
    full_ = true;
  }

  // True iff no unread value is currently held.
  bool empty() const { return !full_; }

  // Pops (destructively) the latest posted value and clears the full flag.
  // Well-defined on an empty mailbox: returns a default-constructed T and
  // leaves the mailbox empty.
  T take() {
    full_ = false;
    return value_;
  }

  // Non-destructive read: the currently-held value, or nullptr if empty.
  // Mirrors WorkQueue::peek()'s own non-destructive-inspection precedent
  // (095-007, test-support addition -- lets a test observe a just-posted
  // value before the next tick's take() drains it, without altering the
  // mailbox's own single-consumer contract for every other caller).
  const T* peek() const { return full_ ? &value_ : nullptr; }

 private:
  T value_ = {};
  bool full_ = false;
};

// WorkQueue<T, N> -- FIFO, capacity N. For DELTAS/commands that must all
// apply, in order. post() returns false when at capacity (never silently
// overwrites or drops without signaling full) -- the caller decides
// drop-vs-ERR.
template <typename T, uint32_t N>
class WorkQueue {
 public:
  // Appends in FIFO order. Returns false (no-op) when already at capacity N.
  bool post(const T& value) {
    if (count_ >= N) return false;
    buf_[tail_] = value;
    tail_ = (tail_ + 1) % N;
    ++count_;
    return true;
  }

  // True iff no elements are queued.
  bool empty() const { return count_ == 0; }

  // Pops (destructively) the front element in FIFO order. Well-defined on
  // an empty queue: returns a default-constructed T and leaves the queue
  // empty.
  T take() {
    if (count_ == 0) return T{};
    T value = buf_[head_];
    head_ = (head_ + 1) % N;
    --count_;
    return value;
  }

  // Non-destructive iteration: peek(0) is the front (next take()'s
  // result), peek(size()-1) is the back. Returns nullptr for any index
  // outside [0, size()).
  const T* peek(uint32_t i) const {
    if (i >= count_) return nullptr;
    return &buf_[(head_ + i) % N];
  }

  // Number of elements currently queued.
  uint32_t size() const { return count_; }

 private:
  T buf_[N] = {};
  uint32_t head_ = 0;
  uint32_t tail_ = 0;
  uint32_t count_ = 0;
};

}  // namespace Rt
