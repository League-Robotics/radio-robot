// configurator.cpp -- App::Configurator implementation. See configurator.h
// for the module's boundary.
#include "app/configurator.h"

#include "app/boot_calibration.h"
#include "config/boot_config.h"
#include "messages/wire.h"

namespace App {

namespace {

// isLiveConfigurable() -- the per-ConfigGroupTarget re-appliability table
// (132-008, the-configuration-object.md's "boot-only vs live" boundary).
// See configurator.h's own class-level doc comment for the full table with
// per-target notes; this is just the yes/no gate applyGroup() consults
// BEFORE decoding anything, so a boot-only push leaves config_ untouched
// rather than silently no-op'ing after a successful decode.
bool isLiveConfigurable(msg::ConfigGroupTarget target) {
  switch (target) {
    case msg::ConfigGroupTarget::DRIVE:
    case msg::ConfigGroupTarget::WHEEL_CONTROL:
    case msg::ConfigGroupTarget::MOTORS:
    case msg::ConfigGroupTarget::OTOS:
    case msg::ConfigGroupTarget::ESTIMATOR:
      return true;
    case msg::ConfigGroupTarget::GEOMETRY:
    case msg::ConfigGroupTarget::PLANNER:
    case msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED:
      return false;
  }
  return false;
}

// Present-field merges for persisted tuning. Gains mirror onto both sides;
// travel_calib is side-selected and merged by apply() itself.
void mergeMotorGainsPatch(msg::MotorConfigPatch& slot,
                          const msg::MotorConfigPatch& incoming) {
  if (incoming.kp.has) slot.kp = incoming.kp;
  if (incoming.ki.has) slot.ki = incoming.ki;
  if (incoming.kff.has) slot.kff = incoming.kff;
  if (incoming.i_max.has) slot.i_max = incoming.i_max;
  if (incoming.kaw.has) slot.kaw = incoming.kaw;
}

// `init` is a one-shot trigger, never persisted.
void mergeOtosPatch(msg::OtosConfigPatch& slot,
                    const msg::OtosConfigPatch& incoming) {
  if (incoming.linear_scale.has) slot.linear_scale = incoming.linear_scale;
  if (incoming.angular_scale.has) slot.angular_scale = incoming.angular_scale;
  if (incoming.offset_x.has) slot.offset_x = incoming.offset_x;
  if (incoming.offset_y.has) slot.offset_y = incoming.offset_y;
  if (incoming.offset_yaw.has) slot.offset_yaw = incoming.offset_yaw;
}

}  // namespace

Configurator::Configurator(Drive& drive, Devices::Motor& motorL,
                           Devices::Motor& motorR, Devices::Otos& otos,
                           Motion::Planner& planner,
                           Config::TuningStore* tuningStore)
    : drive_(drive),
      motorL_(motorL),
      motorR_(motorR),
      otos_(otos),
      planner_(planner),
      tuningStore_(tuningStore) {}

uint32_t Configurator::apply(const msg::CommandEnvelope& env) {
  const msg::ConfigDelta& config = env.cmd.config;

  if (config.patch_kind == msg::ConfigDelta::PatchKind::OTOS) {
    const msg::OtosConfigPatch& patch = config.patch.otos;

    applyOtosPatch(patch);
    mergeOtosPatch(persistedTuning_.otos, patch);
    persistTuningIfChanged();
    return 0;
  }

  if (config.patch_kind == msg::ConfigDelta::PatchKind::ESTIMATOR) {
    const msg::EstimatorConfigPatch& patch = config.patch.estimator;

    // weight_heading_otos/weight_omega_otos/staleness_ms: accepted on the
    // wire, applied nowhere -- there is no live consumer for them, and
    // (132-010) there permanently isn't one to wire to -- see
    // install(ConfigGroupTarget::ESTIMATOR)'s own comment below for the
    // full reasoning (App::StateEstimator deleted 128-016; its candidate
    // successor's call site deleted 130-009 pending estimator-v2). This
    // OLD patch surface itself is retired outright by ticket 013, not
    // touched here. Only the shaper-ceiling fields below still reach a
    // live consumer.

    // Shaper wire keys retarget the planner's live profile ceilings. Read
    // the planner's current limits, merge the present fields onto them, and
    // push the whole set back -- applyShaperLimits() takes all six.
    float aMax = planner_.limits().ceilings.aMax;
    float aDecel = planner_.limits().ceilings.aDecel;
    float alphaMax = planner_.limits().ceilings.alphaMax;
    float alphaDecel = planner_.limits().ceilings.alphaDecel;
    float jerkMax = planner_.limits().ceilings.jerkMax;
    float yawJerkMax = planner_.limits().ceilings.yawJerkMax;
    if (patch.a_max.has) aMax = patch.a_max.val;
    if (patch.a_decel.has) aDecel = patch.a_decel.val;
    if (patch.alpha_max.has) alphaMax = patch.alpha_max.val;
    if (patch.alpha_decel.has) alphaDecel = patch.alpha_decel.val;
    if (patch.j_max.has) jerkMax = patch.j_max.val;
    if (patch.yaw_jerk_max.has) yawJerkMax = patch.yaw_jerk_max.val;
    planner_.applyShaperLimits(aMax, aDecel, alphaMax, alphaDecel, jerkMax,
                               yawJerkMax);
    return 0;
  }

  if (config.patch_kind != msg::ConfigDelta::PatchKind::MOTOR) {
    return static_cast<uint32_t>(msg::ErrCode::ERR_UNIMPLEMENTED);
  }

  const msg::MotorConfigPatch& patch = config.patch.motor;

  // Gains mirror onto both sides; travel_calib only to the addressed side.
  mergeMotorGainsPatch(persistedTuning_.motorL, patch);
  mergeMotorGainsPatch(persistedTuning_.motorR, patch);
  if (patch.travel_calib.has) {
    msg::MotorConfigPatch& target = (patch.side == msg::BoundMotorSide::LEFT)
                                        ? persistedTuning_.motorL
                                        : persistedTuning_.motorR;
    target.travel_calib = patch.travel_calib;
  }
  persistedTuning_.motorL.side = msg::BoundMotorSide::LEFT;
  persistedTuning_.motorR.side = msg::BoundMotorSide::RIGHT;

  applyMotorConfigPatch(persistedTuning_.motorL);
  applyMotorConfigPatch(persistedTuning_.motorR);
  persistTuningIfChanged();
  return 0;
}

void Configurator::applyMotorConfigPatch(const msg::MotorConfigPatch& patch) {
  // `kff` does NOT retarget App::Drive's duty-per-speed calibration. It used
  // to (`drive_.setDutyPerSpeed(patch.kff.val, patch.kff.val)`), and that was
  // a destructive overload of one wire key onto two unrelated quantities: the
  // host pushes `pid.kff` from the robot JSON's `control.vel_kff` (the
  // velocity PID's feedforward gain, 0.0008 for tovez -- calibration/push.py),
  // while App::Drive's scale is `control.duty_per_speed_left/right`
  // (0.00187325, boot-baked by Config::defaultDriveConfig()). Every
  // connect-time calibration push therefore replaced a calibrated 0.00187325
  // with 0.0008, and the wheels ran at 43% of the commanded speed -- measured
  // 80 mm/s against a commanded 200 mm/s, which also timed out every turn leg
  // of a tour. The per-wheel duty scale is boot calibration and has no live
  // wire arm (boot_config.h's own DriveBootConfig comment says exactly that);
  // it stays boot-baked.

  // pid.* wire keys retarget App::Drive's unified wheel-speed controller
  // (130-005 repoint, wheel-speed-controller-moves-into-drive.md Phase 3 /
  // sprint 130 Architecture Decision 3): closes the silent no-op these keys
  // used to be onto Motion::Planner's parked M4 duty stage
  // (applyVelGains()/velKff/velKp/velKi/velIMax, inert since 128-015 --
  // nothing ever actuated from it, see planner.h's own commandedDutyLeft/
  // Right() doc comment).
  //
  // `ControlGains` (drive.h) has 5 fields -- kp/ki/iMax/kaff/pidMax --
  // exactly like `MotorConfigPatch` has 5 -- kp/ki/i_max/kff/kaw. kp/ki/
  // i_max keep both their wire-key NAME and their gain SEMANTICS, now
  // landing on Drive instead of the planner. kff/kaw have no Stage-B
  // counterpart of their own name, but both were ALREADY fully dead on
  // this path (kff's old duty-per-speed retarget was removed above; kaw
  // was never read by any apply site at all, on any prior sprint) --
  // repurposed to carry the two ControlGains members that would otherwise
  // have no wire key: kaff (accel feedforward) and pidMax (total fast-loop
  // authority). The wire's field NUMBERS/NAMES are unchanged (a routing
  // change, not a wire-grammar change -- sprint 130's own Out-of-Scope
  // line) -- only what C++ code the values land in changes.
  Drive::ControlGains gains = drive_.controlGains();
  if (patch.kp.has) gains.kp = patch.kp.val;
  if (patch.ki.has) gains.ki = patch.ki.val;
  if (patch.i_max.has) gains.iMax = patch.i_max.val;
  if (patch.kff.has) gains.kaff = patch.kff.val;    // pid.kff -> Stage B's kaff
  if (patch.kaw.has) gains.pidMax = patch.kaw.val;  // pid.kaw -> Stage B's pidMax
  drive_.setControlGains(gains);

  if (patch.travel_calib.has) {
    if (patch.side == msg::BoundMotorSide::LEFT) {
      motorL_.applyTravelCalib(patch.travel_calib.val);
    } else {
      motorR_.applyTravelCalib(patch.travel_calib.val);
    }
  }
}

void Configurator::applyOtosPatch(const msg::OtosConfigPatch& patch) {
  if (patch.linear_scale.has) otos_.setLinearScalar(patch.linear_scale.val);
  if (patch.angular_scale.has) otos_.setAngularScalar(patch.angular_scale.val);

  // setOffset writes x/y/heading together: read-merge-write so absent
  // fields keep the chip's current values.
  if (patch.offset_x.has || patch.offset_y.has || patch.offset_yaw.has) {
    float x = 0.0f, y = 0.0f, heading = 0.0f;
    otos_.getOffset(x, y, heading);
    if (patch.offset_x.has) x = patch.offset_x.val;
    if (patch.offset_y.has) y = patch.offset_y.val;
    if (patch.offset_yaw.has) heading = patch.offset_yaw.val;
    otos_.setOffset(x, y, heading);
  }

  if (patch.init) otos_.init();
}

// Change-detection debounce: only write flash when the serialized snapshot
// actually differs from the last one written.
void Configurator::persistTuningIfChanged() {
  if (tuningStore_ == nullptr) return;

  Config::Blob blob = Config::serializeSnapshot(persistedTuning_);
  if (blob == lastPersistedBlob_) return;

  tuningStore_->save(Config::kConfigSchemaVersion, blob);
  lastPersistedBlob_ = blob;
}

void Configurator::reapplyPersistedTuning(const Config::TuningSnapshot& snapshot) {
  applyMotorConfigPatch(snapshot.motorL);
  applyMotorConfigPatch(snapshot.motorR);
  applyOtosPatch(snapshot.otos);

  persistedTuning_ = snapshot;
  lastPersistedBlob_ = Config::serializeSnapshot(persistedTuning_);
}

// loadBaked() -- see configurator.h's own doc comment. One assignment per
// msg::ConfigGroupTarget, straight from the generated, robot-JSON-baked
// defaults (132-005) -- no derivation of its own.
void Configurator::loadBaked() {
  config_.geometry = Config::defaultGeometryGroup();
  config_.motors = Config::defaultMotorsGroup();
  config_.drive = Config::defaultDriveGroup();
  config_.wheelControl = Config::defaultWheelControlGroup();
  config_.planner = Config::defaultPlannerGroup();
  config_.otos = Config::defaultOtosGroup();
  config_.estimator = Config::defaultEstimatorGroup();
}

// install() -- see configurator.h's own doc comment. PLANNER stays an
// inline call (boot-only, no install(target) case exists for it -- the
// per-target re-appliability table, configurator.h). DRIVE/WHEEL_CONTROL
// now call drive_.configure(config_) -- the SAME call install(DRIVE)/
// install(WHEEL_CONTROL) uses (132-008) -- rather than re-deriving
// Stage A/B/C inline a second time; this retarget is 132-009's own job
// (configurator.h's "tickets 009/010's job" note, now resolved).
//
// dutyPerSpeed decision (132-009, a stakeholder-decision REVERSAL -- see
// this file's own git history / the ticket for the full writeup):
// REVERSED. Before this ticket, this line read
// `drive_.setDutyPerSpeed(Drive::kDutyPerSpeed, Drive::kDutyPerSpeed)` --
// "MEASURED, NOT CONFIGURED" (stakeholder, 2026-07-31), a hardcoded C++
// constant applied identically to EVERY robot regardless of which JSON was
// baked, deliberately ignoring config_.drive.duty_per_speed_left/right.
// That decision solved a real problem (duty_per_speed and wheel_gain had
// been circularly fitted against each other -- data/robots/tovez.json's own
// `_wheel_correction_note`) but at the cost of two standing violations:
// (1) the 2026-08-03 configuration-discipline rule
// (.claude/rules/configuration-discipline.md) -- "every value the robot
// uses comes from the file, no exceptions, at production boot" -- and (2)
// this project's own §6 "no defaults, always configured" boot posture
// (drive.h's dutyPerSpeedLeft_/Right_ doc comment: "Baking a value here is
// what made one robot's gearboxes every robot's"), which a single baked
// C++ literal shared across tovez/togov/tovez_nocal violates just as
// directly as a hand-rolled default would. The design's own resolution
// (the-configuration-object.md) is to keep dutyPerSpeed a ONE-population-
// scale value with ALL per-wheel deviation expressed in wheel_gain/
// intercept instead -- i.e. fix the circular-fit risk by CONVENTION (the
// file's own left/right fields are measured together and kept equal,
// never re-fit against wheel_gain residuals -- duty_sweep.py's own
// "constant-free saturation reading" cross-check already enforces this in
// practice), not by removing the value from the file entirely. Sourcing it
// from config_.drive here is therefore BEHAVIOR-PRESERVING, not a new
// calibration decision: data/robots/tovez.json/tovez_nocal.json's
// duty_per_speed_left/right were corrected (132-009) from a stale
// 0.00187325 (the ~1.6x error the-configuration-object.md's Cause section
// cites) to 0.001182 -- the value that was ALREADY running on every robot
// via the hardcoded constant this replaces, so no robot's actual boot
// behavior changes the moment this lands. Drive::kDutyPerSpeed itself is
// KEPT (drive.h) as the documented measurement this value traces to and
// duty_sweep.py's own cross-check anchor, but is no longer read here.
// togov.json's own duty_per_speed (its own 2026-07-27 sweep, a different
// number) is deliberately left untouched -- see that file's own
// _drive_calibration_note; it now finally takes effect if togov is ever
// built as the active robot, which was silently impossible before this
// ticket.
void Configurator::install() {
  planner_.applyShaperLimits(config_.planner.a_max, config_.planner.a_decel,
                              config_.planner.alpha_max, config_.planner.alpha_decel,
                              config_.planner.jerk_max, config_.planner.yaw_jerk_max);

  drive_.setDutyPerSpeed(config_.drive.duty_per_speed_left, config_.drive.duty_per_speed_right);
  drive_.configure(config_);
}

// applyGroup() -- see configurator.h's own doc comment. Boot-only targets
// are refused BEFORE any decode is attempted (config_ untouched).
//
// Decodes into a local SCRATCH value of the group's own type, never
// directly into config_'s member -- msg::wire::decode(<Group>&, ...)
// unconditionally memsets its `out` argument to zero before decoding (the
// SAME full-object-zero rationale decode(CommandEnvelope&, ...)/
// decode(Telemetry&, ...) document, wire.cpp), so decoding straight into
// config_.drive (etc.) would zero the CURRENTLY LIVE group immediately,
// before it is even known whether the incoming push is valid. A decode
// failure partway through (malformed bytes / an out-of-bounds field,
// including a NaN payload -- wire.cpp's validateBounds() rejects NaN
// explicitly, 132-008) would then leave config_ in a half-zeroed,
// half-new state -- neither the old values nor a fully-decoded new set.
// Committing the scratch value into config_ only after `r.ok` is what
// actually delivers the "no partial commit" property: a rejected push
// leaves config_ completely untouched, not merely mostly-untouched. On
// success the whole group is REPLACED in one assignment (no patch, no
// presence flags, no merge, per the-configuration-object.md).
msg::ErrCode Configurator::applyGroup(msg::ConfigGroupTarget target, const uint8_t* wire,
                                      size_t len) {
  if (!isLiveConfigurable(target)) return msg::ErrCode::ERR_NOT_LIVE;

  const auto wireLen = static_cast<uint16_t>(len);
  switch (target) {
    case msg::ConfigGroupTarget::DRIVE: {
      msg::Drive decoded;
      const msg::wire::Result r = msg::wire::decode(decoded, wire, wireLen);
      if (!r.ok) return r.code;
      config_.drive = decoded;
      break;
    }
    case msg::ConfigGroupTarget::WHEEL_CONTROL: {
      msg::WheelControl decoded;
      const msg::wire::Result r = msg::wire::decode(decoded, wire, wireLen);
      if (!r.ok) return r.code;
      config_.wheelControl = decoded;
      break;
    }
    case msg::ConfigGroupTarget::MOTORS: {
      msg::Motors decoded;
      const msg::wire::Result r = msg::wire::decode(decoded, wire, wireLen);
      if (!r.ok) return r.code;
      config_.motors = decoded;
      break;
    }
    case msg::ConfigGroupTarget::OTOS: {
      msg::Otos decoded;
      const msg::wire::Result r = msg::wire::decode(decoded, wire, wireLen);
      if (!r.ok) return r.code;
      config_.otos = decoded;
      break;
    }
    case msg::ConfigGroupTarget::ESTIMATOR: {
      msg::Estimator decoded;
      const msg::wire::Result r = msg::wire::decode(decoded, wire, wireLen);
      if (!r.ok) return r.code;
      config_.estimator = decoded;
      break;
    }
    case msg::ConfigGroupTarget::GEOMETRY:
    case msg::ConfigGroupTarget::PLANNER:
    case msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED:
      // Unreachable: isLiveConfigurable() already filtered these out above.
      // Kept as an explicit case (not folded into `default`) so a future
      // ConfigGroupTarget value added to the enum without a matching
      // isLiveConfigurable()/applyGroup() arm fails to compile
      // (-Wswitch/-Werror) rather than silently falling through here.
      return msg::ErrCode::ERR_NOT_LIVE;
  }

  return install(target);
}

// install(target) -- see configurator.h's own doc comment and the
// per-target re-appliability table there for what each branch reaches and
// why. Called only after applyGroup() has already decoded into config_ for
// a live target (or directly by a test) -- never mutates config_ itself.
msg::ErrCode Configurator::install(msg::ConfigGroupTarget target) {
  switch (target) {
    case msg::ConfigGroupTarget::DRIVE:
    case msg::ConfigGroupTarget::WHEEL_CONTROL:
      // Drive::configure() (132-007) pulls BOTH groups' fields in one
      // pass -- Stage A wheel correction + crawl pulse from config_.drive,
      // Stage B/C gains/bounds from config_.wheelControl -- so DRIVE and
      // WHEEL_CONTROL share this one call. Neither re-clamps state that is
      // already running out of newly-lowered bounds (the running PID
      // integrator, the learned adaptation bias) -- a live push can leave
      // that state stale until the next steady-gate accumulation; not this
      // ticket's fix (the-configuration-object.md's own caveat on the
      // "safe" setters).
      drive_.configure(config_);
      return msg::ErrCode::ERR_NONE;

    case msg::ConfigGroupTarget::MOTORS: {
      // Guarded per side (App::configureMotor(), 132-007, boot_calibration.
      // h/.cpp): refuses -- and applies nothing -- while that side reports
      // itself in motion (velocity()/appliedDuty() both nonzero). The
      // refusal is surfaced, not swallowed: ERR_BUSY tells a caller to
      // retry at rest rather than the push silently doing nothing.
      const bool okLeft = App::configureMotor(motorL_, config_, /*isLeft=*/true);
      const bool okRight = App::configureMotor(motorR_, config_, /*isLeft=*/false);
      if (!okLeft || !okRight) return msg::ErrCode::ERR_BUSY;
      return msg::ErrCode::ERR_NONE;
    }

    case msg::ConfigGroupTarget::OTOS:
      // App::configureOtos() (132-007, domain fix 132-010) now converts
      // linear_scale/angular_scale through Devices::scaleToRegister()
      // before calling setLinearScalar()/setAngularScalar() -- the SAME
      // conversion RealOtos::begin() applies to the baked value at boot,
      // closing trap 3 (the-configuration-object.md): a live push and a
      // boot bake now agree on what a given multiplier means.
      // applyOtosPatch() (the OLD patch surface, above) still has the
      // pass-through bug -- that surface is retired outright by ticket
      // 013, not patched here.
      App::configureOtos(otos_, config_);
      return msg::ErrCode::ERR_NONE;

    case msg::ConfigGroupTarget::ESTIMATOR:
      // PERMANENT, not a gap: closing trap 2 (the-configuration-object.md)
      // means making this dead end EXPLICIT, not inventing a call site.
      // App::StateEstimator was deleted outright as dead code (sprint 128
      // ticket 016); its one candidate successor, Motion::PoseTracker::
      // blendHeading() (src/motion/planner/estimation.h), had its only
      // call site AND its own config fields (PlannerLimits::
      // headingOtosWeight/otosStaleness) deleted outright by 130-009 in
      // favor of a from-scratch fusion redesign tracked at
      // clasi/issues/later/estimator-v2-otos-fusion-sim-first.md. Wiring
      // ESTIMATOR to either would mean resurrecting logic 130-009
      // deliberately retired, or building estimator-v2 itself -- neither
      // is this ticket's job, and the sprint's own module boundary
      // (sprint.md Step 3) is explicit that this sprint gives existing
      // setters a wire path, it does not invent new firmware logic. So
      // Configurator holds no estimator-shaped reference at all.
      // config_.estimator is still decoded and read-back-correct --
      // applyGroup() above already ran before install() is ever reached --
      // but there is nothing here to fan it out TO, permanently, until an
      // estimator-v2 ticket gives it a real consumer. ERR_UNIMPLEMENTED
      // reports that honestly instead of acking OK for a push that lands
      // nowhere -- the exact silent no-op this sprint exists to close, not
      // reproduce under a new name.
      return msg::ErrCode::ERR_UNIMPLEMENTED;

    case msg::ConfigGroupTarget::GEOMETRY:
    case msg::ConfigGroupTarget::PLANNER:
    case msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED:
      // GEOMETRY/PLANNER are boot-only (isLiveConfigurable() above) --
      // applyGroup() never reaches install() for either. Reachable
      // directly by a test or a future caller that skips applyGroup(), so
      // this stays a real rejection rather than an unreachable-assumed
      // default.
      return msg::ErrCode::ERR_NOT_LIVE;
  }
  return msg::ErrCode::ERR_NOT_LIVE;
}

}  // namespace App
