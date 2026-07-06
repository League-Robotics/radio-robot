// sim_odometer.h — Hal::SimOdometer: the simulated OTOS-style odometer leaf
// (sprint 081-003) — the FIRST concrete implementation of Hal::Odometer
// (source/hal/capability/odometer.h, previously declared-only; see that
// file's "Gap note").
//
// Ground-truth sampling: each tick() samples Hal::PhysicsWorld's true pose
// (x, y, h), differences it against the previous sample, recovers the
// body-frame forward arc by projecting the world-frame delta onto the
// plant's own midpoint heading (the exact inverse of the midpoint-arc
// integration PhysicsWorld::update() used to produce the delta), and
// integrates the (optionally errored) result into its OWN accumulator
// (odomX_/odomY_/odomH_). This mirrors source_old/hal/sim/SimOdometer.cpp's
// ground-truth-sampling model (ticket 066-001 lineage there) minus the
// lever-arm / bench-OTOS / lift / injected-pose machinery that model also
// carried — none of that has an equivalent wire surface in the new tree yet
// (odometer.h's own "Gap note": no Odometer Command/Config/Capabilities
// message exists, and no firmware consumer of Hal::Odometer exists this
// sprint either), so porting it forward here would be unvalidatable scope
// creep, not a faithful minimal port.
//
// Independent error accumulator (architecture-update.md (081) acceptance
// criterion): this class's noise/scale/drift knobs are entirely separate
// state from Hal::PhysicsWorld's encoder error model (setEncoderNoise() et
// al.) — the two error models NEVER share state, so a test can disable one
// and exercise the other in isolation.
//
// No CODAL dependency — excluded from the ARM firmware build by
// CMakeLists.txt's blanket ".*/hal/sim/.*" EXCLUDE REGEX, the same as every
// other source/hal/sim/ file.
#pragma once

#include <stdint.h>

#ifdef HOST_BUILD
#include <random>
#endif

#include "hal/capability/odometer.h"
#include "hal/sim/physics_world.h"
#include "messages/common.h"

namespace Hal {

class SimOdometer : public Odometer {
 public:
  explicit SimOdometer(const PhysicsWorld& plant);

  msg::PoseEstimate pose() const override;
  bool connected() const override;   // always true — no I2C link to fail

  // Samples the plant's true pose, differences it against the previous
  // sample, and integrates the (optionally errored) delta into this
  // object's own accumulator. now: [ms].
  void tick(uint32_t now) override;

  // --- Hal::Odometer's primitive setters (084-008) — see hal/capability/
  // odometer.h's apply()/configure() for the shared dispatch that calls
  // these. init()/resetTracking() both rebaseline the ground-truth sampling
  // accumulator (hasLastTick_) rather than moving odomX_/odomY_/odomH_ —
  // see sim_odometer.cpp's doc comments on each for why that is the honest
  // sim analog of OtosSensor::init()/resetTracking()'s real register writes
  // (source_old/hal/real/OtosSensor.cpp), neither of which touches the
  // POSITION_XL registers OZ/OV do. setPose() is the one primitive that DOES
  // move the accumulator (used by both OZ, via an all-zero Pose2D, and OV).
  void init() override;
  void resetTracking() override;
  void setPose(const msg::Pose2D& pose) override;
  void setLinearScalar(float scalar) override;
  void setAngularScalar(float scalar) override;

  // Calibration mirrors (test / future SIMGET-style read-back) — 084-008:
  // two NEW fields, kept independent of the error-injection knobs below
  // (Decision 5's Consequences: "the calibration surface and the
  // error-injection surface never share state"). Store-and-echo only this
  // sprint — see linearScalar_/angularScalar_'s own field comments.
  float linearScalar() const { return linearScalar_; }
  float angularScalar() const { return angularScalar_; }

  // --- Error-knob setters — each mutates exactly ONE field below; mirrored
  // 1:1 by sim_setters.h's free Hal:: functions (the canonical call sites
  // ticket 004's ctypes ABI uses). All default to zero -> a fresh
  // SimOdometer is perfect (no behaviour change). ---
  void setLinearNoiseSigma(float sigma);     // [mm] mutates linearNoiseSigma_
  void setYawNoiseSigma(float sigma);        // [rad] mutates yawNoiseSigma_
  void setLinearScaleError(float err);       // mutates linearScaleErr_ (fractional)
  void setAngularScaleError(float err);      // mutates angularScaleErr_ (fractional)
  void setLinearDriftPerTick(float drift);   // [mm] mutates linearDriftPerTick_
  void setYawDriftPerTick(float drift);      // [rad] mutates yawDriftPerTick_

  // Mirror accessors (test / future SIMGET-style read-back).
  float linearNoiseSigma()    const { return linearNoiseSigma_; }
  float yawNoiseSigma()       const { return yawNoiseSigma_; }
  float linearScaleError()    const { return linearScaleErr_; }
  float angularScaleError()   const { return angularScaleErr_; }
  float linearDriftPerTick()  const { return linearDriftPerTick_; }
  float yawDriftPerTick()     const { return yawDriftPerTick_; }

  // Accumulated pose (test accessors mirroring pose()'s own fields).
  float odomX() const { return odomX_; }
  float odomY() const { return odomY_; }
  float odomH() const { return odomH_; }

 private:
  const PhysicsWorld& plant_;   // ground-truth read access only

  float odomX_ = 0.0f;
  float odomY_ = 0.0f;
  float odomH_ = 0.0f;

  float velV_     = 0.0f;   // [mm/s] body-frame forward
  float velOmega_ = 0.0f;   // [rad/s]

  // Ground-truth sampling baseline: the plant's truePose*() value as of the
  // previous tick() call.
  float prevTrueX_ = 0.0f;
  float prevTrueY_ = 0.0f;
  float prevTrueH_ = 0.0f;

  uint32_t lastTick_ = 0;   // [ms]
  bool hasLastTick_ = false;

  // Deterministic + stochastic error model. All zero by default -> a fresh
  // SimOdometer is perfect (the zero-error determinism gate).
  float linearNoiseSigma_   = 0.0f;   // [mm]
  float yawNoiseSigma_      = 0.0f;   // [rad]
  float linearScaleErr_     = 0.0f;   // fractional scale error on linear delta
  float angularScaleErr_    = 0.0f;   // fractional scale error on angular delta
  float linearDriftPerTick_ = 0.0f;   // [mm] additive drift per tick
  float yawDriftPerTick_    = 0.0f;   // [rad] additive drift per tick

  // Calibration surface (084-008, OL/OA) — deliberately separate fields from
  // the error-injection knobs immediately above (Decision 5's Consequences).
  // Store-and-echo only: no scale error is modeled in sim for these to
  // meaningfully correct, so neither field is read by tick() anywhere.
  float linearScalar_  = 0.0f;   // OL
  float angularScalar_ = 0.0f;   // OA

#ifdef HOST_BUILD
  // Independent noise stream from PhysicsWorld's own rngL_/rngR_ (different
  // seed, matching source_old's SimOdometer/PhysicsWorld precedent of
  // separate generators per error model).
  std::mt19937 rng_{43u};
#endif
};

}  // namespace Hal
