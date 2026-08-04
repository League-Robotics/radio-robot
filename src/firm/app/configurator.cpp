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
    // wire, applied nowhere -- there is no live consumer for them. Only
    // the shaper-ceiling fields below still reach a live consumer.

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

// install() -- see configurator.h's own doc comment. Ports
// installShaperLimits()/installDriveCalibration()/installWheelController()'s
// bodies (boot_calibration.cpp, pre-132-006) unchanged in BEHAVIOR, reading
// config_'s NEW group fields (populated by loadBaked(), immediately above)
// instead of the OLD resolve()-computed Motion::PlannerLimits/
// Config::DriveBootConfig/Config::WheelControllerBootConfig structs those
// free functions used to take as parameters. Both old and new structs are
// baked from the SAME robot-JSON `_require()` paths (132-005's own
// completion note), so this is a relocation, not a behavior change.
void Configurator::install() {
  planner_.applyShaperLimits(config_.planner.a_max, config_.planner.a_decel,
                              config_.planner.alpha_max, config_.planner.alpha_decel,
                              config_.planner.jerk_max, config_.planner.yaw_jerk_max);

  // MEASURED, NOT CONFIGURED (stakeholder, 2026-07-31): one baked constant
  // for both wheels, deliberately ignoring config_.drive.duty_per_speed_left/
  // right -- see Drive::kDutyPerSpeed's own doc comment (drive.h). Ticket
  // 009 owns whether this reverses; not this ticket's call to make.
  drive_.setDutyPerSpeed(Drive::kDutyPerSpeed, Drive::kDutyPerSpeed);
  drive_.setWheelCorrection(
      config_.drive.wheel_gain_left_accel, config_.drive.wheel_intercept_left_accel,
      config_.drive.wheel_gain_left_decel, config_.drive.wheel_intercept_left_decel,
      config_.drive.wheel_gain_right_accel, config_.drive.wheel_intercept_right_accel,
      config_.drive.wheel_gain_right_decel, config_.drive.wheel_intercept_right_decel);
  drive_.setCrawlPulse(config_.drive.crawl_pulse);

  Drive::ControlGains gains;
  gains.kp = config_.wheelControl.pid_kp;
  gains.ki = config_.wheelControl.pid_ki;
  gains.iMax = config_.wheelControl.pid_i_max;
  gains.kaff = config_.wheelControl.pid_kaff;
  gains.pidMax = config_.wheelControl.pid_max;
  drive_.setControlGains(gains);

  Drive::AdaptationBounds bounds;
  bounds.vMin = config_.wheelControl.v_min;
  bounds.biasMax = config_.wheelControl.bias_max;
  bounds.tauAdapt = config_.wheelControl.tau_adapt;
  bounds.aSteady = config_.wheelControl.a_steady;
  bounds.deficitThreshold = config_.wheelControl.deficit_threshold;
  bounds.deficitWindow = config_.wheelControl.deficit_window;
  drive_.setAdaptationBounds(bounds);
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
      // App::configureOtos() (132-007) passes linear_scale/angular_scale
      // straight through to setLinearScalar()/setAngularScalar() -- the
      // SAME multiplier-vs-register domain mismatch applyOtosPatch()
      // (the OLD patch surface, above) already has (trap 3,
      // the-configuration-object.md: RealOtos::begin() converts the same
      // multiplier through scaleToRegister() first; this live path does
      // not, yet). Reconciling the two is ticket 010's job, not this
      // ticket's -- see that ticket's own file.
      App::configureOtos(otos_, config_);
      return msg::ErrCode::ERR_NONE;

    case msg::ConfigGroupTarget::ESTIMATOR:
      // No live consumer exists yet: App::StateEstimator was deleted
      // outright as dead code (sprint 128 ticket 016), and this class
      // holds no reference to a replacement (ticket 010 adds one).
      // config_.estimator is still decoded and read-back-correct --
      // applyGroup() above already ran before install() is ever reached --
      // but there is nothing here to fan it out TO yet. ERR_UNIMPLEMENTED
      // reports that honestly instead of acking OK for a push that lands
      // nowhere (trap 2, the-configuration-object.md) -- the exact silent
      // no-op this sprint exists to close, not reproduce under a new name.
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
