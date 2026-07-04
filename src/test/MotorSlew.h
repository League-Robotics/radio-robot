#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// MotorSlew.h — pure |ΔPWM| step-clamp helper (064-002).
//
// Deliberately dependency-free: no CODAL/MicroBit include, so this header
// compiles both into the firmware (via Motor.cpp) AND into the host
// (HOST_BUILD) simulation library, where it is exercised directly through
// the sim_motor_clamp_slew() C-ABI hook (tests/_infra/sim/sim_api.cpp) —
// see tests/simulation/unit/test_motor_slew.py.
//
// Context: Motor::setSpeed() (source/hal/real/Motor.cpp) writes the Nezha
// V2's 0x60 "run motor" register.  A stop (pct==0) or a direction reversal
// bypasses the write-rate throttle and writes *immediately* — for a
// reversal, with the FULL requested swing (e.g. -100 -> +100 is a 200-point
// step) in one transaction.  Stand-session evidence (arm 5 of the stress
// matrix, see clasi/issues/encoder-reset-while-moving-latches-readback.md)
// reproduced a persistent encoder-readback latch from this trigger alone,
// with the IRQ guard ON and no resets involved.  clampStep() bounds the
// magnitude of any single 0x60 write so a large reversal converges over
// several writes instead of one instant slam.
// ---------------------------------------------------------------------------

namespace MotorSlew {

/**
 * clampStep — bound |target - lastWritten| to at most maxDelta.
 *
 * Returns `target` unchanged if the swing already fits within maxDelta.
 * Otherwise returns `lastWritten` stepped by maxDelta toward `target`.
 *
 * This function has NO concept of a "stop" command — pct==0 is just another
 * target value here.  The caller (Motor::setSpeed()) is responsible for
 * special-casing pct==0 as an immediate, unclamped, full write (the sprint's
 * explicit safety exemption); this pure helper's contract does not include
 * that exemption.
 *
 * Pure function: no state, no I/O, safe to call from both the firmware
 * (hal/real/Motor.cpp) and host-side tests.
 */
inline int8_t clampStep(int8_t lastWritten, int8_t target, uint8_t maxDelta)
{
    int16_t delta = (int16_t)target - (int16_t)lastWritten;
    if (delta > (int16_t)maxDelta) {
        return (int8_t)((int16_t)lastWritten + (int16_t)maxDelta);
    }
    if (delta < -(int16_t)maxDelta) {
        return (int8_t)((int16_t)lastWritten - (int16_t)maxDelta);
    }
    return target;
}

}  // namespace MotorSlew
