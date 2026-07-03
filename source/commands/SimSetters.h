#pragma once
// SimSetters.h -- shared, single-source-of-truth free functions over
// SimHardware& (069-005).
//
// Every kSimRegistry[] row in SimCommands.cpp and every legacy per-field
// ctypes function in tests/_infra/sim/sim_api.cpp that touches the same
// PhysicsWorld/SimOdometer knob call THESE functions -- never a duplicated
// `hal.plant().setXxx(...)`/`hal.simOdometer().setXxx(...)` call site copied
// into both files. That is the whole point of this header: one canonical
// place per knob, so the SIMSET wire surface and the ctypes C-ABI can never
// drift apart (architecture-update.md Design Rationale Decision 3; sprint's
// own stated requirement, Sprint Changes Summary item 1).
//
// Naming: plain per-side names (e.g. encoderScaleErrorL/R) for knobs whose
// wire/registry shape is inherently per-side; a side-parameterized function
// (e.g. encoderNoise(hal, side, v), motorOffset(hal, side, v)) ADDITIONALLY
// exists where a legacy ctypes function's C-ABI signature takes a runtime
// `side` (0=left, 1=right, other=both) -- both forms are trivial one-line
// forwards to the SAME underlying PhysicsWorld/SimOdometer method, so no
// logic is duplicated between them.
//
// Header-only (all `inline`) so no new translation unit needs to be added to
// either build: the ARM firmware target never includes this header (it is
// only reachable from SimCommands.cpp and sim_api.cpp, both host-build-only
// translation units), so no CMakeLists.txt exclusion filter is needed the
// way SimCommands.cpp itself needs one.

#include "hal/sim/SimHardware.h"

namespace simsetters {

// ---- Body-truth scrub (069-002) -------------------------------------------
inline void  bodyRotScrub(SimHardware& hal, float f) { hal.plant().setBodyRotationalScrub(f); }
inline float getBodyRotScrub(SimHardware& hal)       { return hal.plant().bodyRotationalScrub(); }

inline void  bodyLinScrub(SimHardware& hal, float f) { hal.plant().setBodyLinearScrub(f); }
inline float getBodyLinScrub(SimHardware& hal)       { return hal.plant().bodyLinearScrub(); }

// ---- Geometry / actuation ---------------------------------------------------
// SimHardware::setTrackwidth() forwards to both its own cached field and
// _plant.setTrackwidth(); trackwidth() reads back via _plant.trackwidthMm().
// (071-007 namespace-collision note: ONLY this wrapper's own function name --
// which mirrors the "trackwidthMm" SIMSET key 1:1 -- is renamed here.
// SimHardware::trackwidthMm()/setTrackwidth() and PhysicsWorld::trackwidthMm()
// keep their existing names: SimHardware.h is outside this ticket's file
// scope and PhysicsWorld::trackwidthMm() is also read directly by WorldView.h,
// another out-of-scope consumer. The kSimRegistry[] "trackwidthMm" key string
// itself is untouched throughout -- see architecture-update.md's
// Wire-Compatibility Exclusion Table.)
inline void  trackwidth(SimHardware& hal, float trackwidth) { hal.setTrackwidth(trackwidth); }  // [mm]
inline float getTrackwidth(SimHardware& hal)                { return hal.trackwidthMm(); }

// motorOffsetL/R -- per-side registry rows (SIMSET has no "both" key).
inline void  motorOffsetL(SimHardware& hal, float f) { hal.plant().setOffsetFactor(0, f); }
inline float getMotorOffsetL(SimHardware& hal)       { return hal.plant().offsetFactorL(); }

inline void  motorOffsetR(SimHardware& hal, float f) { hal.plant().setOffsetFactor(1, f); }
inline float getMotorOffsetR(SimHardware& hal)       { return hal.plant().offsetFactorR(); }

// motorOffset(side, f) -- side-parameterized pass-through, matching
// PhysicsWorld::setOffsetFactor's own (0=L,1=R,other=both) convention
// verbatim. This is what the legacy sim_set_motor_offset(h, side, factor)
// ctypes C-ABI (which takes a runtime side, including "both") forwards to.
inline void motorOffset(SimHardware& hal, int side, float f) { hal.plant().setOffsetFactor(side, f); }

// ---- Per-wheel encoder-report error (058-001 lineage, 069-004 getters) ----
inline void  encoderScaleErrorL(SimHardware& hal, float err) { hal.plant().setEncoderScaleError(0, err); }
inline float getEncoderScaleErrorL(SimHardware& hal)         { return hal.plant().encoderScaleErrL(); }

inline void  encoderScaleErrorR(SimHardware& hal, float err) { hal.plant().setEncoderScaleError(1, err); }
inline float getEncoderScaleErrorR(SimHardware& hal)         { return hal.plant().encoderScaleErrR(); }

inline void  encoderSlipL(SimHardware& hal, float frac) { hal.plant().setEncoderSlip(0, frac); }
inline float getEncoderSlipL(SimHardware& hal)          { return hal.plant().encoderSlipL(); }

inline void  encoderSlipR(SimHardware& hal, float frac) { hal.plant().setEncoderSlip(1, frac); }
inline float getEncoderSlipR(SimHardware& hal)          { return hal.plant().encoderSlipR(); }

inline void  encoderNoiseL(SimHardware& hal, float sigma) { hal.plant().setEncoderNoise(0, sigma); }  // [mm]
inline float getEncoderNoiseL(SimHardware& hal)           { return hal.plant().encoderNoiseL(); }

inline void  encoderNoiseR(SimHardware& hal, float sigma) { hal.plant().setEncoderNoise(1, sigma); }  // [mm]
inline float getEncoderNoiseR(SimHardware& hal)           { return hal.plant().encoderNoiseR(); }

// encoderNoise(side, sigma) -- side-parameterized pass-through, matching
// PhysicsWorld::setEncoderNoise's own (0=L,1=R,other=both) convention
// verbatim. This is what the legacy sim_set_encoder_noise(h, side, sigma_mm)
// ctypes C-ABI (runtime side, including "both") forwards to.
inline void encoderNoise(SimHardware& hal, int side, float sigma) { hal.plant().setEncoderNoise(side, sigma); }  // [mm]

// ---- OTOS sim-model error state (057-005/058-001 lineage, 069-004 getters) ----
inline void  otosLinScaleErr(SimHardware& hal, float err) { hal.simOdometer().setLinearScaleError(err); }
inline float getOtosLinScaleErr(SimHardware& hal)         { return hal.simOdometer().linearScaleError(); }

inline void  otosAngScaleErr(SimHardware& hal, float err) { hal.simOdometer().setAngularScaleError(err); }
inline float getOtosAngScaleErr(SimHardware& hal)         { return hal.simOdometer().angularScaleError(); }

// otosLinNoise/otosYawNoise -- canonicalized on SimOdometer::setLinearNoiseSigma()/
// setYawNoiseSigma() (NOT the setLinearNoise()/setYawNoise() back-compat
// aliases in SimOdometer.h, which write the identical field but were, before
// this ticket, a second textual call path only the ctypes function used).
inline void  otosLinNoise(SimHardware& hal, float sigma) { hal.simOdometer().setLinearNoiseSigma(sigma); }
inline float getOtosLinNoise(SimHardware& hal)           { return hal.simOdometer().linearNoiseSigma(); }

inline void  otosYawNoise(SimHardware& hal, float sigma) { hal.simOdometer().setYawNoiseSigma(sigma); }
inline float getOtosYawNoise(SimHardware& hal)           { return hal.simOdometer().yawNoiseSigma(); }

// ---- OTOS drift (per-second wire/ctypes value <-> per-tick internal value) --
// otosLinDriftMmS / otosYawDriftDegS -- the wire keys (and any future ctypes
// caller) are PER-SECOND; SimOdometer::setLinearDriftPerTick()/
// setYawDriftPerTick() (and linearDriftPerTick()/yawDriftPerTick()) are
// PER-TICK internally: tick() adds the FULL per-tick value once per call, and
// tick() fires once per RobotConfig::controlPeriod (source/types/Config.h;
// see SimOdometer::tick()'s unconditional `_odomX += _linearDriftPerTick` /
// `_odomH += _yawDriftPerTick`, source/hal/sim/SimOdometer.cpp). Conversion
// formula (both directions read the SAME live controlPeriod() via
// SimOdometer::controlPeriod(), so a runtime `SET ctrlPeriod=…` is honored
// immediately, per 067's live-reference rule):
//
//     per_tick   = per_second * (period / 1000.0f)   // period: [ms]
//     per_second = per_tick   * (1000.0f / period)
//
// otosYawDriftDegS is additionally deg<->rad converted: the wire key is
// degrees/second (issue-1's plumbing guidance), but setYawDriftPerTick()'s
// argument -- and the internal _yawDriftPerTick accumulator it feeds -- is
// radians.
//
// (071-007 namespace-collision note, mirroring the trackwidth one above: only
// these wrapper FUNCTION NAMES are renamed -- otosLinDriftMmS/otosYawDriftDegS
// as kSimRegistry[] KEY STRINGS are untouched throughout SimCommands.cpp.)
static const float kDegToRad = 3.14159265358979323846f / 180.0f;
static const float kRadToDeg = 180.0f / 3.14159265358979323846f;

inline void otosLinearDrift(SimHardware& hal, float v) {
    float period = static_cast<float>(hal.simOdometer().controlPeriod());  // [ms]
    hal.simOdometer().setLinearDriftPerTick(v * (period / 1000.0f));
}
inline float getOtosLinearDrift(SimHardware& hal) {
    float period = static_cast<float>(hal.simOdometer().controlPeriod());  // [ms]
    if (period <= 0.0f) return 0.0f;
    return hal.simOdometer().linearDriftPerTick() * (1000.0f / period);
}

inline void otosYawDrift(SimHardware& hal, float v) {
    float period = static_cast<float>(hal.simOdometer().controlPeriod());  // [ms]
    float rate   = v * kDegToRad;                                          // [rad/s]
    hal.simOdometer().setYawDriftPerTick(rate * (period / 1000.0f));
}
inline float getOtosYawDrift(SimHardware& hal) {
    float period = static_cast<float>(hal.simOdometer().controlPeriod());  // [ms]
    if (period <= 0.0f) return 0.0f;
    float rate = hal.simOdometer().yawDriftPerTick() * (1000.0f / period);  // [rad/s]
    return rate * kRadToDeg;
}

}  // namespace simsetters
