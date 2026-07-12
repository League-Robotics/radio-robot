// null_odometer.h — Hal::NullOdometer: the Null Object for Hal::Odometer
// (ticket 090-003, issue null-odometer-object.md).
//
// Subsystems::Hardware::odometer()'s base-class default used to return
// `nullptr` for an owner with no real odometer leaf of its own, which smeared
// "no device" handling across every caller as an `if (odometer != nullptr)`
// guard (source/runtime/main_loop.cpp's three branches, source/main.cpp's
// bb.otosPresent snapshot, source/runtime/configurator.cpp's config guard —
// see each file's own 090-003 comments). This class collapses that: the base
// default now returns a NullOdometer instead, so `hardware_.odometer()` NEVER
// returns null and every one of those guards drops to its unconditional form.
//
// Every primitive is an inert no-op/discard:
//   - pose() returns a default-constructed msg::PoseEstimate — identity pose
//     (all-zero Pose2D/BodyTwist3), stamp.valid == false (ValueSet's own
//     default), matching Hal::SimOdometer/Hal::OtosOdometer's own existing
//     "this is not a fresh sample" idiom rather than inventing a new one —
//     Subsystems::PoseEstimator::tick() already gates fusion on
//     `otosObs->stamp.valid` (source/subsystems/pose_estimator.cpp), so a
//     NullOdometer's pose() is inert there for free.
//   - connected() is always false — there is no device to be connected to.
//   - tick()/init()/resetTracking()/setPose()/setLinearScalar()/
//     setAngularScalar() all discard — there is nothing to drive.
//   - fusableThisPass() overrides the base's flag-based logic (090-002)
//     entirely: a NullOdometer is never fusable, full stop, regardless of
//     the (nonexistent, since apply()'s primitives above are no-ops) reset
//     bookkeeping the base class tracks in its private resetAppliedThisPass_.
//
// Headers-only, matching capability/odometer.h's own convention — no
// null_odometer.cpp. Deliberately lives in hal/capability/, NOT hal/sim/
// (architecture-update.md Decision 3): it has zero HOST_BUILD/PhysicsWorld
// dependency and must be reachable from a real-hardware ARM build (it is
// Subsystems::Hardware's own base-class default, callable from
// Subsystems::NezhaHardware's inheritance chain even though NezhaHardware
// itself always overrides odometer()) — hal/sim/ is excluded from the ARM
// build by CMakeLists.txt's blanket EXCLUDE REGEX, which would make this
// class unreachable from firmware if it lived there.
#pragma once

#include <stdint.h>

#include "hal/capability/odometer.h"
#include "messages/common.h"
#include "messages/drivetrain.h"

namespace Hal {

class NullOdometer : public Odometer {
 public:
  msg::PoseEstimate pose() const override { return msg::PoseEstimate(); }
  bool connected() const override { return false; }
  // 099-002: mirrors connected()'s own always-false -- there is no device
  // at all, so nothing was ever detected present either. Not reachable by
  // any currently-constructed owner (this class's own file header) --
  // overridden here anyway, rather than left to Hal::Odometer's `true`
  // convenience default, for the same "every primitive is an inert no-op"
  // completeness this class's header already promises.
  bool present() const override { return false; }

  void tick(uint32_t now) override { (void)now; }   // [ms] no device to sample

  void init() override {}
  void resetTracking() override {}
  void setPose(const msg::Pose2D& pose) override { (void)pose; }
  void setLinearScalar(float scalar) override { (void)scalar; }
  void setAngularScalar(float scalar) override { (void)scalar; }

  // Overrides ticket 090-002's flag-based logic (Odometer::fusableThisPass())
  // entirely — there is no device, so there is never anything fusable. Never
  // touches (and does not need) the base's private resetAppliedThisPass_
  // bookkeeping; apply()'s primitive dispatch above is all no-ops anyway.
  bool fusableThisPass() override { return false; }
};

}  // namespace Hal
