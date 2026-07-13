// measurement_ring.h — Devices::Sample<T> + Devices::MeasurementRing<T>: the
// 6-slot gap-write ring every measurement stream (each motor's encoder
// reading, OTOS pose, line, color) publishes through.
//
// Ticket DB-002 (device-bus-tickets.md). Part of the greenfield
// `source/devices/` subsystem (namespace `Devices`) described in
// clasi/issues/device-bus-fiber-owned-self-contained-device-subsystem.md's
// "Measurement rings (the 6-slot gap-write buffer)" section — this file
// implements that section's sketch, not a redesign of it.
//
// --- The gap-write protocol, exactly ---
//
// 6 PHYSICAL slots, 5 PUBLISHED. At any moment the "published window" is the
// 5 slots [head_, head_-1, head_-2, head_-3, head_-4] (indices mod kSlots);
// the one remaining slot — (head_ + 1) mod kSlots — is the WRITE GAP, and is
// never part of the published window (a window of 5 consecutive slots out
// of 6 physical slots always excludes exactly one — the gap). publish()
// therefore does exactly two things, in this order:
//   1. Write the new (value, stamp) into the gap slot — a full Sample<T>
//      struct store into a slot NO reader can observe yet (it is outside
//      every published index sample()/bracket()/latest() can produce).
//   2. Advance head_ to that same index with a single store. THIS is the
//      instant the sample becomes published — head_ is a single uint8_t, so
//      this step is one aligned store, atomic with respect to any reader
//      running between fiber yield points (the issue's "Concurrency
//      contract").
// The tail is implicit: it is always head_ - 4 (mod kSlots), never tracked
// as its own variable.
//
// --- Immutability guarantee ---
//
// No published slot is EVER mutated in place. A reader that copies a
// Sample<T> out of the ring (latest()/sample()/bracket() all return BY
// VALUE) holds a value that cannot change underneath it, no matter how many
// further publish() calls happen afterward — publish() only ever writes the
// CURRENT gap slot, and a slot does not become "the gap" again until 5 more
// publishes have advanced head_ past it and back around. This holds
// independent of the cooperative-scheduler argument (belt and suspenders —
// the issue is explicit this stays correct even under a preemptive writer,
// though none is being built here).
//
// --- Sample<T>::stamp width (OQ3) ---
//
// The issue's own sketch shows `uint32_t stamp` (system_timer_current_time_
// us() truncated to 32 bits). device-bus-tickets.md's "Resolved open
// questions" overrides this: stamp is `uint64_t` [us] so bracket()/lerp are
// wrap-free (a uint32_t microsecond counter wraps in ~71 minutes; a uint64_t
// one does not wrap on any timescale this firmware will ever run — the same
// reasoning otos.h's own DB-005 "Scope changes" note already applied to
// readDue()/tick()'s "now" parameter). This is a deliberate, documented
// deviation from the sketch, not an oversight.
//
// Pure host-clean C++: <cstdint> only. No bus, no CODAL, no yields anywhere
// in this file — publish()/latest()/sample()/bracket() are all plain struct
// stores/copies, matching the concurrency contract's rule 2 ("No yield
// inside a publish, a staged-input store, or a consumer-side sample copy").
#pragma once

#include <cstdint>

namespace Devices {

// Sample<T> — one timestamped reading. `valid` is false until the stream's
// first publish() (a freshly-constructed MeasurementRing has no history yet
// — every not-yet-written slot reports Sample<T>{} with valid == false
// rather than a garbage stamp/value).
template <typename T>
struct Sample {
  T value{};
  uint64_t stamp = 0;  // [us] fiber-read timestamp at publish() (OQ3)
  bool valid = false;
};

// MeasurementRing<T> — single-writer (the fiber), multi-reader gap-write
// ring. See this file's header comment for the exact publish() protocol,
// the immutability guarantee, and the stamp-width note.
template <typename T>
class MeasurementRing {
 public:
  static constexpr uint8_t kSlots = 6;
  static constexpr uint8_t kDepth = 5;  // published slots; kSlots - kDepth == 1 write gap

  // publish — fiber-only. Writes `value`/`stamp` into the current write-gap
  // slot, then advances head_ to that slot with a single store, publishing
  // it. Never mutates any already-published slot.
  void publish(const T& value, uint64_t stamp) {  // [us]
    const uint8_t gap = static_cast<uint8_t>((head_ + 1) % kSlots);
    slots_[gap] = Sample<T>{value, stamp, true};  // full struct store into the (still-unpublished) gap
    head_ = gap;                                  // single aligned store -- publishes it
  }

  // latest — the newest published sample (age 0). Sample<T>{} (valid ==
  // false) if publish() has never been called.
  Sample<T> latest() const { return sample(0); }

  // sample — age 0 = newest … kDepth-1 = oldest published. An age at or
  // past the depth of history published so far returns Sample<T>{} (valid
  // == false), NOT a wrapped-around read of a foreign stream's history — a
  // never-written slot's default-constructed valid == false is exactly that
  // signal. `age` is clamped to [0, kDepth-1] so it can never land on the
  // write gap itself (kDepth == kSlots - 1 by construction — see this
  // file's header comment).
  Sample<T> sample(uint8_t age) const {
    if (age >= kDepth) age = kDepth - 1;
    const uint8_t index = static_cast<uint8_t>((head_ + kSlots - age) % kSlots);
    return slots_[index];
  }

  // bracket — the two published samples with older.stamp <= t <= newer.
  // stamp, if any. Scans from newest to oldest (ages 0..kDepth-1); stops at
  // the first not-yet-published slot it meets, since publish() only ever
  // extends history forward in time — once one slot going backward is
  // invalid, every slot older than it is guaranteed invalid too. Returns
  // false (older/newer left untouched) if `t` is outside the published
  // window, including "newer than every sample" and "before any sample".
  bool bracket(uint64_t t, Sample<T>& older, Sample<T>& newer) const {  // [us]
    Sample<T> newerCandidate = sample(0);
    if (!newerCandidate.valid) return false;
    for (uint8_t age = 1; age < kDepth; ++age) {
      const Sample<T> olderCandidate = sample(age);
      if (!olderCandidate.valid) break;
      if (olderCandidate.stamp <= t && t <= newerCandidate.stamp) {
        older = olderCandidate;
        newer = newerCandidate;
        return true;
      }
      newerCandidate = olderCandidate;
    }
    return false;
  }

 private:
  Sample<T> slots_[kSlots] = {};
  uint8_t head_ = 0;  // index of the most-recently-published slot
};

}  // namespace Devices
