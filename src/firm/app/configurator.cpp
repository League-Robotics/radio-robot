// configurator.cpp -- App::Configurator implementation. See configurator.h
// for the module's boundary.
#include "app/configurator.h"

#include <cmath>

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
    case msg::ConfigGroupTarget::PLANNER_SHAPER:
      // PLANNER_SHAPER (132-017 split): the six shaper-ceiling fields
      // carry their own re-appliable setter (Motion::Planner::
      // applyShaperLimits()) -- see configurator.h's re-appliability
      // table for why this was split OUT of PLANNER rather than left
      // boot-only alongside the rest of that group.
      return true;
    case msg::ConfigGroupTarget::NAVIGATOR:
      // 135-004: Motion::Navigator holds its NavigatorLimits by reference
      // (no setter needed at all) -- see configurator.h's re-appliability
      // table NAVIGATOR row.
      return true;
    case msg::ConfigGroupTarget::GEOMETRY:
    case msg::ConfigGroupTarget::PLANNER:
    case msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED:
      return false;
  }
  return false;
}

}  // namespace

// stampSource()/configSource() -- 133-006 provenance. See configurator.h's
// own doc comments for the placement rule (stamp at the mutation site, never
// at a call site) and for why this is a side table rather than a field of
// Config::Robot.
void Configurator::stampSource(msg::ConfigGroupTarget target, msg::ConfigSource source) {
  const auto slot = static_cast<size_t>(target);
  if (slot == 0 || slot >= kGroupSourceSlots) return;
  groupSource_[slot] = source;
}

msg::ConfigSource Configurator::configSource(msg::ConfigGroupTarget target) const {
  const auto slot = static_cast<size_t>(target);
  if (slot == 0 || slot >= kGroupSourceSlots) return msg::ConfigSource::CONFIG_SOURCE_UNSPECIFIED;
  return groupSource_[slot];
}

Configurator::Configurator(Drive& drive, Devices::Motor& motorL,
                           Devices::Motor& motorR, Devices::Otos& otos,
                           Motion::Planner& planner,
                           Motion::NavigatorLimits& navigatorLimits,
                           Config::TuningStore* tuningStore)
    : drive_(drive),
      motorL_(motorL),
      motorR_(motorR),
      otos_(otos),
      planner_(planner),
      navigatorLimits_(navigatorLimits),
      tuningStore_(tuningStore) {}

// persistIfEligible() -- 132-013 (patch-surface retirement): called from
// applyGroup()/applyField() after install(target) returns ERR_NONE.
// Snapshots config_'s CURRENT values for `target`'s persisted subset
// straight out of config_ (no merge -- config_ already holds the union of
// every baked default and every live push so far, exactly the property
// applyGroup()'s own "no patch, no presence flags, no merge" whole-group
// replacement guarantees), marks that group's own "tuned" flag, and writes
// through persistTuningIfChanged(). A no-op for any target outside the
// precedent-persisted set (configurator.h's re-appliability table's own
// PERSISTENCE SCOPE note documents exactly which targets and why).
void Configurator::persistIfEligible(msg::ConfigGroupTarget target) {
  switch (target) {
    case msg::ConfigGroupTarget::WHEEL_CONTROL:
      persistedTuning_.wheelControlTuned = true;
      persistedTuning_.wheelControlPidKp = config_.wheelControl.pid_kp;
      persistedTuning_.wheelControlPidKi = config_.wheelControl.pid_ki;
      persistedTuning_.wheelControlPidIMax = config_.wheelControl.pid_i_max;
      persistedTuning_.wheelControlPidKaff = config_.wheelControl.pid_kaff;
      persistedTuning_.wheelControlPidMax = config_.wheelControl.pid_max;
      break;

    case msg::ConfigGroupTarget::MOTORS:
      persistedTuning_.motorsTravelCalibTuned = true;
      persistedTuning_.motorsTravelCalibLeft = config_.motors.travel_calib_left;
      persistedTuning_.motorsTravelCalibRight = config_.motors.travel_calib_right;
      break;

    case msg::ConfigGroupTarget::OTOS:
      persistedTuning_.otosTuned = true;
      persistedTuning_.otosOffsetX = config_.otos.offset_x;
      persistedTuning_.otosOffsetY = config_.otos.offset_y;
      persistedTuning_.otosOffsetYaw = config_.otos.offset_yaw;
      persistedTuning_.otosLinearScale = config_.otos.linear_scale;
      persistedTuning_.otosAngularScale = config_.otos.angular_scale;
      break;

    case msg::ConfigGroupTarget::DRIVE:
    case msg::ConfigGroupTarget::ESTIMATOR:
    case msg::ConfigGroupTarget::GEOMETRY:
    case msg::ConfigGroupTarget::PLANNER:
    case msg::ConfigGroupTarget::PLANNER_SHAPER:
    case msg::ConfigGroupTarget::NAVIGATOR:
    case msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED:
      // Not in the persisted-tuning precedent set -- configurator.h's own
      // re-appliability table's PERSISTENCE SCOPE note. PLANNER_SHAPER
      // (132-017, live but never persisted) is here on purpose, same as
      // DRIVE above -- see that row's own doc comment. NAVIGATOR (135-004)
      // is the same shape again -- live, never persisted.
      return;
  }

  persistTuningIfChanged();
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

// reapplyPersistedTuning() -- see configurator.h's own doc comment.
// Writes each TOUCHED group's persisted fields into config_, then fans it
// out via the SAME install(target) a live wire push uses -- an untouched
// group (its own "tuned" flag false) is left exactly as loadBaked()/
// install() already set it; a zero-initialized snapshot field is never
// written over a real baked default.
void Configurator::reapplyPersistedTuning(const Config::TuningSnapshot& snapshot) {
  if (snapshot.wheelControlTuned) {
    config_.wheelControl.pid_kp = snapshot.wheelControlPidKp;
    config_.wheelControl.pid_ki = snapshot.wheelControlPidKi;
    config_.wheelControl.pid_i_max = snapshot.wheelControlPidIMax;
    config_.wheelControl.pid_kaff = snapshot.wheelControlPidKaff;
    config_.wheelControl.pid_max = snapshot.wheelControlPidMax;
    // 133-006: PERSISTED, not BAKED -- these values came out of flash, not
    // out of the baked file, and reporting them as BAKED would be exactly
    // the "robot running tuned values the read-back denies" dishonesty this
    // provenance exists to remove. See robot_config.proto's ConfigSource.
    stampSource(msg::ConfigGroupTarget::WHEEL_CONTROL,
                msg::ConfigSource::CONFIG_SOURCE_PERSISTED);
    install(msg::ConfigGroupTarget::WHEEL_CONTROL);
  }

  if (snapshot.motorsTravelCalibTuned) {
    config_.motors.travel_calib_left = snapshot.motorsTravelCalibLeft;
    config_.motors.travel_calib_right = snapshot.motorsTravelCalibRight;
    // Motors are at rest this early in boot (before any command is ever
    // routed) -- install(MOTORS)'s ERR_BUSY guard should never trip here;
    // the return value is intentionally not checked, matching this
    // function's own pre-132-013 best-effort shape (void, no error path).
    stampSource(msg::ConfigGroupTarget::MOTORS, msg::ConfigSource::CONFIG_SOURCE_PERSISTED);
    install(msg::ConfigGroupTarget::MOTORS);
  }

  if (snapshot.otosTuned) {
    config_.otos.offset_x = snapshot.otosOffsetX;
    config_.otos.offset_y = snapshot.otosOffsetY;
    config_.otos.offset_yaw = snapshot.otosOffsetYaw;
    config_.otos.linear_scale = snapshot.otosLinearScale;
    config_.otos.angular_scale = snapshot.otosAngularScale;
    stampSource(msg::ConfigGroupTarget::OTOS, msg::ConfigSource::CONFIG_SOURCE_PERSISTED);
    install(msg::ConfigGroupTarget::OTOS);
  }

  persistedTuning_ = snapshot;
  lastPersistedBlob_ = Config::serializeSnapshot(persistedTuning_);
}

// loadBaked() -- see configurator.h's own doc comment. One assignment per
// msg::ConfigGroupTarget, straight from the generated, robot-JSON-baked
// defaults (132-005) -- no derivation of its own.
void Configurator::loadBaked(const Config::WheelCorrection* wheelCorrectionOverride) {
  config_.geometry = Config::defaultGeometryGroup();
  config_.motors = Config::defaultMotorsGroup();
  config_.drive = Config::defaultDriveGroup();
  config_.wheelControl = Config::defaultWheelControlGroup();
  config_.planner = Config::defaultPlannerGroup();
  config_.plannerShaper = Config::defaultPlannerShaperGroup();
  config_.otos = Config::defaultOtosGroup();
  config_.estimator = Config::defaultEstimatorGroup();
  config_.navigator = Config::defaultNavigatorGroup();

  // 133-005: the wheel-correction override, applied AFTER the bake so it
  // wins over whatever data/robots/*.json currently holds -- see this
  // method's own declaration comment and BootOverrides::wheelCorrection
  // (app/boot_wiring.h). Deliberately overwrites config_ itself, not just
  // the Drive install: read-back (config()/GetConfig(DRIVE)) then reports
  // what this robot is ACTUALLY calibrated with, which is the whole point
  // of the configuration object.
  if (wheelCorrectionOverride != nullptr) {
    config_.drive.wheel_gain_left_accel = wheelCorrectionOverride->gainLeftAccel;
    config_.drive.wheel_intercept_left_accel = wheelCorrectionOverride->interceptLeftAccel;
    config_.drive.wheel_gain_left_decel = wheelCorrectionOverride->gainLeftDecel;
    config_.drive.wheel_intercept_left_decel = wheelCorrectionOverride->interceptLeftDecel;
    config_.drive.wheel_gain_right_accel = wheelCorrectionOverride->gainRightAccel;
    config_.drive.wheel_intercept_right_accel = wheelCorrectionOverride->interceptRightAccel;
    config_.drive.wheel_gain_right_decel = wheelCorrectionOverride->gainRightDecel;
    config_.drive.wheel_intercept_right_decel = wheelCorrectionOverride->interceptRightDecel;
  }

  // 133-006 provenance: everything this function established is BAKED --
  // including a `wheelCorrectionOverride` DRIVE, which is a composition-root
  // boot value (App::composeRobot()'s sim-plant identity fit, boot_wiring.h),
  // not a runtime push. BAKED here means "came from boot-time composition",
  // which is the distinction a caller asking "is the robot running what I
  // pushed" actually needs; the override is deliberately NOT given a
  // provenance of its own.
  //
  // This one loop is also the whole of "reset behaviour": a reset re-runs
  // loadBaked(), which re-stamps every group BAKED, so a previously-LIVE
  // group stops claiming to be live with no separate reset handling. Slot 0
  // (CONFIG_GROUP_UNSPECIFIED) is not a group and is deliberately skipped.
  for (size_t slot = 1; slot < kGroupSourceSlots; ++slot) {
    groupSource_[slot] = msg::ConfigSource::CONFIG_SOURCE_BAKED;
  }
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
  // 132-017 split: the six shaper ceilings now live on config_.plannerShaper
  // (a LIVE ConfigGroupTarget), not config_.planner (the boot-only
  // remainder) -- see configurator.h's re-appliability table.
  planner_.applyShaperLimits(config_.plannerShaper.a_max, config_.plannerShaper.a_decel,
                              config_.plannerShaper.alpha_max, config_.plannerShaper.alpha_decel,
                              config_.plannerShaper.jerk_max, config_.plannerShaper.yaw_jerk_max);

  drive_.setDutyPerSpeed(config_.drive.duty_per_speed_left, config_.drive.duty_per_speed_right);
  drive_.configure(config_);

  // 135-004: the SAME call install(NAVIGATOR) makes -- a live push and a
  // boot bake share one code path, same precedent as PLANNER_SHAPER above.
  App::configureNavigator(navigatorLimits_, config_);
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
    case msg::ConfigGroupTarget::PLANNER_SHAPER: {
      msg::PlannerShaper decoded;
      const msg::wire::Result r = msg::wire::decode(decoded, wire, wireLen);
      if (!r.ok) return r.code;
      config_.plannerShaper = decoded;
      break;
    }
    case msg::ConfigGroupTarget::NAVIGATOR: {
      msg::Navigator decoded;
      const msg::wire::Result r = msg::wire::decode(decoded, wire, wireLen);
      if (!r.ok) return r.code;
      config_.navigator = decoded;
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

  // 133-006: stamp LIVE the moment config_ is committed, BEFORE install().
  // Deliberately not conditioned on install() succeeding: by this point the
  // group HAS been replaced, so a read-back returns the pushed values, and
  // the honest answer to "where did what I am reading come from" is "a live
  // push" whether or not the fan-out then reached a subsystem. ESTIMATOR is
  // the standing example -- its install() is a permanent ERR_UNIMPLEMENTED,
  // yet config_.estimator holds exactly what was pushed.
  stampSource(target, msg::ConfigSource::CONFIG_SOURCE_LIVE);

  const msg::ErrCode result = install(target);
  if (result == msg::ErrCode::ERR_NONE) persistIfEligible(target);
  return result;
}

// applyField() -- see configurator.h's own doc comment. Boot-only targets
// are refused BEFORE any field lookup (same isLiveConfigurable() gate
// applyGroup() uses above); a non-finite value is refused BEFORE
// msg::wire::setField() ever runs -- validateBounds() (wire.cpp) only
// guards min/max/abs_max, not finiteness, so a NaN/Inf check has to happen
// here, at the one call site, rather than inside the generated engine
// (132-008's own documented NaN trap, closed there for applyGroup()'s
// decode path and closed here for this single-field path the same way).
msg::ErrCode Configurator::applyField(msg::ConfigGroupTarget target, uint16_t fieldNumber,
                                      float value) {
  if (!isLiveConfigurable(target)) return msg::ErrCode::ERR_NOT_LIVE;
  if (!std::isfinite(value)) return msg::ErrCode::ERR_BADARG;

  msg::wire::Result r{false, fieldNumber, msg::ErrCode::ERR_BADARG};
  switch (target) {
    case msg::ConfigGroupTarget::DRIVE:
      r = msg::wire::setField(config_.drive, fieldNumber, value);
      break;
    case msg::ConfigGroupTarget::WHEEL_CONTROL:
      r = msg::wire::setField(config_.wheelControl, fieldNumber, value);
      break;
    case msg::ConfigGroupTarget::MOTORS:
      r = msg::wire::setField(config_.motors, fieldNumber, value);
      break;
    case msg::ConfigGroupTarget::OTOS:
      r = msg::wire::setField(config_.otos, fieldNumber, value);
      break;
    case msg::ConfigGroupTarget::ESTIMATOR:
      r = msg::wire::setField(config_.estimator, fieldNumber, value);
      break;
    case msg::ConfigGroupTarget::PLANNER_SHAPER:
      r = msg::wire::setField(config_.plannerShaper, fieldNumber, value);
      break;
    case msg::ConfigGroupTarget::NAVIGATOR:
      r = msg::wire::setField(config_.navigator, fieldNumber, value);
      break;
    case msg::ConfigGroupTarget::GEOMETRY:
    case msg::ConfigGroupTarget::PLANNER:
    case msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED:
      // Unreachable: isLiveConfigurable() already filtered these out above.
      // Kept as an explicit case (not folded into `default`), same
      // discipline applyGroup()'s own switch documents -- a future
      // ConfigGroupTarget added without a matching arm here fails to
      // compile (-Wswitch/-Werror) rather than silently falling through.
      return msg::ErrCode::ERR_NOT_LIVE;
  }
  if (!r.ok) return r.code;

  // 133-006: stamp LIVE only once the single-field write actually landed
  // (`r.ok`) -- every rejection path above returns with config_ untouched,
  // so an unknown field / non-finite value / out-of-bounds push must not
  // make the group claim it is live. Same "stamp at the mutation, before
  // install()" rule applyGroup() documents above.
  stampSource(target, msg::ConfigSource::CONFIG_SOURCE_LIVE);

  const msg::ErrCode result = install(target);
  if (result == msg::ErrCode::ERR_NONE) persistIfEligible(target);
  return result;
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

    case msg::ConfigGroupTarget::PLANNER_SHAPER:
      // 132-017 split: the shaper ceilings' own re-appliable setter, the
      // SAME Motion::Planner::applyShaperLimits() call install() (the
      // no-arg boot fan-out, above) already makes -- a live push and a
      // boot bake share one code path.
      planner_.applyShaperLimits(config_.plannerShaper.a_max, config_.plannerShaper.a_decel,
                                  config_.plannerShaper.alpha_max, config_.plannerShaper.alpha_decel,
                                  config_.plannerShaper.jerk_max, config_.plannerShaper.yaw_jerk_max);
      return msg::ErrCode::ERR_NONE;

    case msg::ConfigGroupTarget::NAVIGATOR:
      // 135-004: writes straight into the NavigatorLimits Motion::Navigator
      // already holds a reference to -- the SAME call install() (the
      // no-arg boot fan-out, above) makes -- no setter, by construction
      // (configurator.h's re-appliability table NAVIGATOR row).
      App::configureNavigator(navigatorLimits_, config_);
      return msg::ErrCode::ERR_NONE;

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

// encodeSnapshot() -- see configurator.h's own doc comment. Deliberately
// NOT gated by isLiveConfigurable(): read-back is honest for every
// ConfigGroupTarget, including the boot-only ones applyGroup()/install()
// reject as writes (the-configuration-object.md's own testing note: "GET
// still works for boot-only groups even though SET is rejected").
msg::ErrCode Configurator::encodeSnapshot(msg::ConfigGroupTarget target,
                                          msg::ConfigSnapshot& out) const {
  out.target = target;
  // 133-006: provenance rides the REPLY, not the config struct. Set before
  // the switch so every success path carries it; the ERR_BADARG path below
  // returns without a body, and reporting a source for a group that was
  // never encoded would be its own small lie -- hence the reset there.
  out.source = configSource(target);
  uint16_t len = 0;
  switch (target) {
    case msg::ConfigGroupTarget::GEOMETRY:
      len = msg::wire::encode(config_.geometry, out.body_, sizeof(out.body_));
      break;
    case msg::ConfigGroupTarget::MOTORS:
      len = msg::wire::encode(config_.motors, out.body_, sizeof(out.body_));
      break;
    case msg::ConfigGroupTarget::DRIVE:
      len = msg::wire::encode(config_.drive, out.body_, sizeof(out.body_));
      break;
    case msg::ConfigGroupTarget::WHEEL_CONTROL:
      len = msg::wire::encode(config_.wheelControl, out.body_, sizeof(out.body_));
      break;
    case msg::ConfigGroupTarget::PLANNER:
      len = msg::wire::encode(config_.planner, out.body_, sizeof(out.body_));
      break;
    case msg::ConfigGroupTarget::PLANNER_SHAPER:
      len = msg::wire::encode(config_.plannerShaper, out.body_, sizeof(out.body_));
      break;
    case msg::ConfigGroupTarget::OTOS:
      len = msg::wire::encode(config_.otos, out.body_, sizeof(out.body_));
      break;
    case msg::ConfigGroupTarget::ESTIMATOR:
      len = msg::wire::encode(config_.estimator, out.body_, sizeof(out.body_));
      break;
    case msg::ConfigGroupTarget::NAVIGATOR:
      len = msg::wire::encode(config_.navigator, out.body_, sizeof(out.body_));
      break;
    case msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED:
    default:
      // Kept as an explicit default (not folded away) so a future
      // ConfigGroupTarget value added with no matching arm here fails
      // loudly (ERR_BADARG) rather than reading stale/zeroed body bytes.
      out.source = msg::ConfigSource::CONFIG_SOURCE_UNSPECIFIED;
      return msg::ErrCode::ERR_BADARG;
  }
  out.body_count = static_cast<uint8_t>(len);
  return msg::ErrCode::ERR_NONE;
}

}  // namespace App
