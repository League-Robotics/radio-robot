// odometer.h — the Odometer faceplate (e.g. an OTOS-style optical
// odometry sensor).
//
// 084-008 fills the gap this file's own header used to flag: protos/
// odometer.proto now exists (OdometerCommand{oneof: init | zero |
// reset_tracking | set_pose}, OdometerConfig{linear_scalar,
// angular_scalar}), so this faceplate gains a real message plane —
// apply()/configure() — matching the same primitive-setters + shared-
// dispatch discipline capability/motor.h's own apply()/configure() already
// established (Hal::Motor::apply() switches over MotorCommand's oneof and
// calls the leaf's primitive setDutyCycle()/setVelocity()/etc.; the two
// methods below do the same over OdometerCommand/OdometerConfig against the
// five primitives declared just below them). capabilities() still does not
// exist — OdometerCommand's four actions are never capability-gated (every
// odometer, real or simulated, supports all four), unlike Motor's five-way
// control-mode oneof, so there is nothing to gate the way
// motorCommandAllowed() gates Motor's.
//
// State: msg::PoseEstimate (pose + twist + freshness stamp) — unchanged,
// still the honest, reused-not-duplicated fit.
#pragma once

#include <stdint.h>

#include "messages/common.h"
#include "messages/drivetrain.h"
#include "messages/odometer.h"

namespace Hal {

class Odometer {
 public:
  virtual ~Odometer() = default;
  virtual void begin() {}

  // Primitive getter — the real read.
  virtual msg::PoseEstimate pose() const = 0;
  virtual bool connected() const = 0;

  // present() (099-002, architecture-update-r1.md Decision 2): true once a
  // chip/device was ever detected as physically present at boot --
  // permanent for the life of the object, unlike connected()'s live,
  // re-evaluated-every-tick() bus health. A CALLER deciding whether to
  // schedule this leaf a bus slot at all (Subsystems::NezhaHardware::
  // tick()'s scheduled-slot branch) or seeding a boot-time diagnostic
  // (bb.otosPresent, source/main.cpp / tests/_infra/sim/sim_api.cpp) wants
  // THIS query, never connected() -- see Hal::OtosOdometer::present()'s own
  // doc comment (otos_odometer.h) for the full contract and the regression
  // this distinction fixes. Virtual with a convenience default of `true`
  // (mirrors begin()'s own "no caller needs polymorphic X semantics for
  // every hypothetical owner this sprint" default, above): Hal::OtosOdometer
  // is the only leaf with a real boot-time detection step to report here
  // and overrides this with its own initialized_-backed logic;
  // Hal::SimOdometer has no physical chip to ever fail to detect (the same
  // rationale as its own hardcoded connected()==true) and is content with
  // this base default, needing no sim-side file change of its own.
  virtual bool present() const { return true; }

  virtual void tick(uint32_t now) = 0;   // [ms]

  // --- Primitive setters (084-008) — one per OdometerCommand action /
  // OdometerConfig field. apply()/configure() below dispatch onto these,
  // never onto a leaf's own storage directly — mirrors capability/motor.h's
  // setDutyCycle()/setVelocity()/etc. split exactly. ---
  virtual void init() = 0;                            // OI — re-init signal processing / tracking
  virtual void resetTracking() = 0;                   // OR — reset Kalman/tracking state
  virtual void setPose(const msg::Pose2D& pose) = 0;  // OZ (zero pose) / OV — set world-frame position
  virtual void setLinearScalar(float scalar) = 0;     // OL
  virtual void setAngularScalar(float scalar) = 0;    // OA

  // Message plane — apply()/configure() are concrete (defined once, below),
  // built on the primitives above; no leaf overrides either (same
  // discipline as capability/motor.h's apply()/configure()).
  void apply(const msg::OdometerCommand& command);
  void configure(const msg::OdometerConfig& config);

  // applySetPose() (090-002) — the SetPose -> Pose2D -> OdometerCommand
  // translation that used to be inlined at main_loop.cpp's SI/otosSetPoseIn
  // drain site. Concrete, built on apply()'s existing SET_POSE dispatch
  // (never duplicates the reset-flag bookkeeping apply() already does) —
  // same "built on primitives, not on leaf storage" discipline as apply()/
  // configure() above.
  void applySetPose(const msg::SetPose& pose);

  // fusableThisPass() (090-002) — READ-AND-CLEAR, SINGLE-CALLER query.
  // Mirrors this codebase's hasEvent()/takeEvent() one-shot-signal
  // convention (e.g. Subsystems::Planner) rather than a plain getter: the
  // very act of calling this method clears the transient it reports, so it
  // may be called AT MOST ONCE per pass. Its one sanctioned caller is
  // Rt::MainLoop::tick()'s poseEstimator_.tick() fusion gate — calling it a
  // second time in the same pass would wrongly consume the signal and make
  // the second caller (or the first, if called again next pass before a new
  // reset) see a stale `true` that hides a still-pending skip, or a
  // spuriously-cleared `false` that should have been reported. Do NOT poll
  // this from more than one call site.
  //
  // Returns false for exactly the one call immediately following a reset
  // applied THIS pass via applySetPose() or apply(const msg::OdometerCommand&)
  // (covering all four reset actions — INIT/ZERO/RESET_TRACKING/SET_POSE);
  // true otherwise. See apply()'s own reset-flag bookkeeping below for why
  // all four actions participate, not only SET_POSE.
  virtual bool fusableThisPass();

  // Message plane — declared, not defined (no caller needs this yet;
  // dev_loop.cpp/telemetry_commands.cpp both read pose() directly instead —
  // see capability/gripper.h's file header for the "declared, not defined"
  // mechanism this relies on).
  msg::PoseEstimate state() const;

 private:
  // resetAppliedThisPass_ (090-002) — set true by apply()'s dispatch for
  // every one of the four reset actions (INIT/ZERO/RESET_TRACKING/SET_POSE)
  // and cleared by fusableThisPass()'s read-and-clear. Private: only this
  // base class's own apply()/fusableThisPass() bodies ever touch it — no
  // leaf needs (or is allowed) to set or clear it directly, so there is
  // nothing for a leaf override to bypass (apply() itself is not virtual;
  // see this class's own file header).
  bool resetAppliedThisPass_ = false;
};

// --- apply()/configure(): the shared message plane, defined once here (same
// headers-only style as capability/motor.h's apply()/state() — no
// capability/odometer.cpp exists). ---

inline void Odometer::apply(const msg::OdometerCommand& command) {
  // 090-002: every action arm below sets resetAppliedThisPass_ = true — all
  // four OdometerCommand actions (INIT/ZERO/RESET_TRACKING/SET_POSE) are
  // one-shot odometer resets that main_loop.cpp's odometerResetThisPass used
  // to track by hand; see fusableThisPass()'s own doc comment for the
  // consumer side of this flag. This runs in the BASE class's own (non-
  // virtual) apply(), so it fires regardless of what a leaf's setPose()/
  // init()/resetTracking() override does — no leaf can bypass it.
  switch (command.action_kind) {
    case msg::OdometerCommand::ActionKind::INIT:
      init();
      resetAppliedThisPass_ = true;
      break;
    case msg::OdometerCommand::ActionKind::ZERO:
      // Real-hardware effect is setPositionRaw(0, 0, 0) (docs/protocol-v2.md
      // §11's "OZ" section) — the SAME primitive SET_POSE uses, just with an
      // all-zero Pose2D rather than the caller-supplied one.
      setPose(msg::Pose2D());
      resetAppliedThisPass_ = true;
      break;
    case msg::OdometerCommand::ActionKind::RESET_TRACKING:
      resetTracking();
      resetAppliedThisPass_ = true;
      break;
    case msg::OdometerCommand::ActionKind::SET_POSE:
      setPose(command.action.set_pose);
      resetAppliedThisPass_ = true;
      break;
    case msg::OdometerCommand::ActionKind::NONE:
    default:
      break;
  }
}

inline void Odometer::configure(const msg::OdometerConfig& config) {
  setLinearScalar(config.linear_scalar);
  setAngularScalar(config.angular_scalar);
}

inline void Odometer::applySetPose(const msg::SetPose& pose) {
  // SetPose -> Pose2D -> OdometerCommand translation, ported verbatim from
  // main_loop.cpp's former inline otosSetPoseIn drain (090-002). Dispatches
  // through apply() rather than calling setPose() directly so the SET_POSE
  // arm's resetAppliedThisPass_ bookkeeping above runs exactly once, from
  // exactly one place.
  msg::Pose2D otosPose;
  otosPose.x = pose.x;
  otosPose.y = pose.y;
  otosPose.h = pose.h;
  msg::OdometerCommand cmd;
  cmd.setSetPose(otosPose);
  apply(cmd);
}

inline bool Odometer::fusableThisPass() {
  bool wasReset = resetAppliedThisPass_;
  resetAppliedThisPass_ = false;
  return !wasReset;
}

}  // namespace Hal
