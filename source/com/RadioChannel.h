#pragma once
#include "MicroBit.h"

/**
 * RadioChannel — persistent radio-channel (nRF frequency band) storage.
 *
 * The robot's radio group is ALWAYS 10 (kGroup); only the channel (frequency
 * band) is configurable.  The channel is persisted in the micro:bit's
 * flash-backed key-value store (uBit.storage) so it survives power cycles.
 *
 * Channel range is 0..35 so it renders as a single base-36 character on the
 * LED matrix (0-9 then A-Z, so channel 10 = 'A').  The nRF hardware supports
 * bands up to 83, but we only need a handful and one digit keeps the boot
 * display glanceable.  An unset or out-of-range stored value falls back to
 * kDefault (0), which matches the RadioRelay's default.
 *
 * Changing the channel over the radio breaks the link the instant the robot
 * re-tunes (the relay is still on the old channel), so the channel is normally
 * changed via the on-board buttons at boot or the `RF` command over USB serial.
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
