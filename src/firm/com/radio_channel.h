#pragma once
#include "MicroBit.h"

/**
 * RadioChannel — persistent radio-channel (nRF frequency band) storage.
 * Design/rationale: DESIGN.md.
 *
 * Group is always kGroup (10); only the channel is persisted, in the
 * micro:bit's flash-backed key-value store (uBit.storage). Range 0..35
 * (single base-36 LED-matrix digit); unset/out-of-range falls back to
 * kDefault (0).
 */
namespace radiochan {

constexpr int kMin     = 0;     // frequency band lower bound
constexpr int kMax     = 35;    // upper bound — single base-36 digit (0-9, A-Z)
constexpr int kDefault = 0;     // matches RadioRelay default channel
constexpr int kGroup   = 10;    // fixed radio group (never changes)

inline int clamp(int c) { return c < kMin ? kMin : (c > kMax ? kMax : c); }

/** Render a channel as a single base-36 character: 0-9 then A-Z (10 = 'A'). */
inline char toChar(int c) {
    c = clamp(c);
    return (c < 10) ? (char)('0' + c) : (char)('A' + (c - 10));
}

/** Read the persisted channel, or kDefault if unset/out of range. */
int load(MicroBitStorage& storage);

/** Persist a channel (clamped to [kMin, kMax]). */
void save(MicroBitStorage& storage, int channel);

}  // namespace radiochan
