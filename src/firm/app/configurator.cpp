// configurator.cpp -- App::Configurator implementation. See configurator.h
// for the module's boundary.
#include "app/configurator.h"

namespace App {

namespace {

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
    float aMax = planner_.limits().aMax;
    float aDecel = planner_.limits().aDecel;
    float alphaMax = planner_.limits().alphaMax;
    float alphaDecel = planner_.limits().alphaDecel;
    float jerkMax = planner_.limits().jerkMax;
    float yawJerkMax = planner_.limits().yawJerkMax;
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

}  // namespace App
