#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// motor_slew.h — pure |ΔPWM| step-clamp helper.
//
// Ported verbatim from source_old/hal/real/MotorSlew.h (064-002) — same
// function body, snake_case filename per this tree's convention. Kept
// dependency-free (no CODAL/MicroBit include) so it stays a pure, testable
// function.
//
// Context: NezhaMotor's write path (nezha_motor.cpp) writes the Nezha V2's
// 0x60 "run motor" register. A stop (pct==0) or a direction reversal
// bypasses the write-rate throttle and writes *immediately* — for a
// reversal, with the FULL requested swing (e.g. -100 -> +100 is a 200-point
// step) in one transaction. Stand-session evidence reproduced a persistent
// encoder-readback latch from this trigger alone. clampStep() bounds the
// magnitude of any single 0x60 write so a large reversal converges over
// several writes instead of one instant slam.
//
// Sprint 077 (ticket 003) ports this unchanged, per architecture-update.md
// Design Rationale 2: the newer zero-dwell reversal fix (docs/knowledge/
// 2026-07-04-encoder-wedge.md) is validated but out of
// this sprint's locked scope — this slew-cap mitigation is what source_old
// ships today, and that is what gets ported.
// ---------------------------------------------------------------------------

namespace MotorSlew {

/**
 * clampStep — bound |target - lastWritten| to at most maxDelta.
 *
 * Returns `target` unchanged if the swing already fits within maxDelta.
 * Otherwise returns `lastWritten` stepped by maxDelta toward `target`.
 *
 * This function has NO concept of a "stop" command — pct==0 is just another
 * target value here. The caller is responsible for special-casing pct==0 as
 * an immediate, unclamped, full write (the sprint's explicit safety
 * exemption); this pure helper's contract does not include that exemption.
 *
 * Pure function: no state, no I/O.
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
