// configurator.h -- Core::Configurator: the CONFIG command's whole
// lifecycle (command-ingestion-ring-buffered-comms-subsystem-routing-two-
// stops.md §6). One of the four routing destinations Core::RobotLoop::
// routeCommand() dispatches to, alongside Core::DifferentialDrive (WHEELS) and
// Motion::Planner (MOVE/STOP).
//
// 132-006 (the-configuration-object.md, sprint 132 "configuration
// discipline"): Configurator now OWNS the one Config::Robot instance --
// `config_` below. `loadBaked()` populates it from the generated,
// robot-JSON-baked Config::default*Group() functions (config/boot_config.h,
// 132-005); `config()` returns it for read-back; `install()` fans its
// re-appliable groups out to the subsystems that own them, replacing
// RobotGraph::Resolved's old job of feeding the same four install*
// Calibration() free functions (boot_calibration.h) -- see boot_wiring.h/
// .cpp for the composition-root side of this split.
//
// 132-008 adds the LIVE wire push path: `applyGroup(target, wire, len)`
// decodes a whole group straight into `config_` (no patch, no presence
// flags, no merge -- ticket 002's generated msg::Geometry/Motors/DifferentialDrive/
// WheelControl/Planner/Otos/Estimator structs ARE the wire codec targets,
// decoded through gen_messages.py-emitted `msg::wire::decode(<Group>&, ...)`
// overloads, wire.h/wire.cpp) and `install(target)` fans out just that one
// group. `applyField(target, fieldNumber, value)` (132-012) is the
// single-field counterpart: reuses the SAME re-appliability gate and the
// SAME generated field-table engine (gen_messages.py-emitted
// `msg::wire::setField(<Group>&, ...)` overloads, one per applyGroup()'s
// own decode() family) to write exactly one already-live field, addressed
// by (target, protobuf field number) rather than a whole-group payload --
// the dev-mode single-value push `.claude/rules/configuration-discipline.md`
// carves out for bench tuning.
//
// Every `ConfigGroupTarget` declares whether it is safely re-appliable at
// runtime or boot-only (the-configuration-object.md's "boot-only vs live"
// section) -- config that acks OK and silently does nothing is worse than
// config that is rejected. `applyGroup()`/`install(target)` consult this
// table (`isLiveConfigurable()`, configurator.cpp) before touching
// anything:
//
//   | target        | live? | install(target) reaches                        | notes |
//   |---------------|-------|--------------------------------------------------|-------|
//   | GEOMETRY      | NO    | -- (ERR_NOT_LIVE)                                 | trackWidth has no post-construction setter anywhere (DifferentialDrive/Odometry/Planner all bake it in at construction); rotation calibration installs once, at boot, via RobotLoop::configure() called directly from boot_wiring.cpp, not through this class |
//   | PLANNER       | NO    | -- (ERR_NOT_LIVE)                                 | vMax/omegaMax/controlPeriod/actuationDelay/landing.*/headingHoldGain have no post-construction setter -- the ONLY reason this group is boot-only. SPLIT (132-017) from the shaper ceilings below, which used to sit in this SAME group and were dragged down to boot-only by that coarse grouping even though they have their own re-appliable setter |
//   | PLANNER_SHAPER| yes   | `Motion::Planner::applyShaperLimits()` (132-017)  | a_max/a_decel/alpha_max/alpha_decel/jerk_max/yaw_jerk_max -- split OUT of PLANNER (132-017, JSON reshape ticket, stakeholder-sanctioned mid-sprint scope addition) because this exact setter was ALREADY one of the-configuration-object.md's eight safely-re-appliable setters; the boot-time no-arg install() also still calls this same setter, so a wire push and a boot bake use the identical code path. NOT persisted -- same "no old curated live-tuning message for this field set" reasoning as DRIVE below; a live-pushed shaper ceiling reverts to the baked JSON default on reboot until re-pushed |
//   | DRIVE         | yes   | `DifferentialDrive::configure(config_)` (132-007)             | Stage A per-wheel gain/intercept + crawl pulse, via the existing `setWheelCorrection()`/`setCrawlPulse()`. NOT persisted (132-013) -- no old curated live-tuning wire message ever existed for these fields, so the persistence precedent leaves them boot-only-persisted (i.e. reset to the baked JSON default every reboot until live-tuned again) |
//   | WHEEL_CONTROL | yes   | `DifferentialDrive::configure(config_)` (132-007)             | SAME call as DRIVE -- DifferentialDrive::configure() pulls Stage B/C gains/bounds from `config_.wheelControl` in the same pass it reads `config_.drive` from; one re-appliable unit from DifferentialDrive's own point of view, two ConfigGroupTargets on the wire. PERSISTED (132-013) -- these 5 fields (pid_kp/pid_ki/pid_i_max/pid_kaff/pid_max) are the direct successor of the old curated Motor live-tuning message's kp/ki/i_max/kff/kaw, which already persisted |
//   | MOTORS        | yes   | `Core::configureMotor()` x2 (132-007), GUARDED     | refuses (returns false, applies nothing) while that side reports itself in motion -- surfaced as `ERR_BUSY`, never swallowed, so a caller can retry at rest. PARTIALLY PERSISTED (132-013) -- only travel_calib_left/travel_calib_right (the one MotorConfig field Core::configureMotor() still live-applies, boot_calibration.h's own doc comment), matching the old curated Motor live-tuning message's travel_calib precedent exactly; fwd_sign/output_deadband/reversal_dwell/vel_* never persisted before and still don't |
//   | OTOS          | yes   | `Core::configureOtos()` (132-007/010)              | trap 3 CLOSED (132-010): `linear_scale`/`angular_scale` are converted through `Hardware::scaleToRegister()` before reaching `setLinearScalar()`/`setAngularScalar()`, matching `RealOtos::begin()`'s own boot-time conversion -- a live push and a boot bake now agree on what a given multiplier means. PERSISTED IN FULL (132-013) -- offset_x/offset_y/offset_yaw/linear_scale/angular_scale mirror the old curated Otos live-tuning message's own 5 scale/offset fields exactly (its 6th field, `init`, was a fire-and-forget trigger with no Config::Robot-shaped successor and was never persisted either) |
//   | ESTIMATOR     | yes   | -- (`ERR_UNIMPLEMENTED`, PERMANENT)               | trap 2 CLOSED (132-010) by making the dead end EXPLICIT rather than inventing a consumer: `Core::StateEstimator` was deleted outright as dead code (sprint 128 ticket 016), and its one candidate successor -- `Motion::PoseTracker::blendHeading()` (`src/firm/motion/planner/estimation.h`) -- had its only call site AND its own config fields (`PlannerLimits::headingOtosWeight`/`otosStaleness`) deleted outright by 130-009, in favor of a from-scratch fusion redesign tracked at `clasi/issues/later/estimator-v2-otos-fusion-sim-first.md`. Building a real consumer today would mean either resurrecting logic 130-009 deliberately retired, or building estimator-v2 itself -- both out of this ticket's scope (and the latter explicitly deferred by its own tracked issue). `Configurator` therefore holds NO estimator-shaped reference; `config_.estimator` still decodes correctly for read-back (`applyGroup()` never skips the decode for a live-classified target), but `install(ESTIMATOR)` returns `ERR_UNIMPLEMENTED` permanently until estimator-v2 gives it something real to call. NEVER PERSISTED (132-013, following the old curated Estimator live-tuning message's own explicit precedent -- "a reboot always reverts to the baked JSON default") -- unaffected by ESTIMATOR's own permanent ERR_UNIMPLEMENTED above; the two are independent facts that happen to agree |
//
//   | NAVIGATOR     | yes   | writes `config_.navigator`'s fields directly into the `Motion::NavigatorLimits` this class holds a reference to (135-004) | NOT a setter call -- `Motion::Navigator` holds its `NavigatorLimits` by const reference (navigator.h), so `tick()` sees a live push on its very next call with no re-apply step to invoke. `Core::configureNavigator()` (app/boot_calibration.h) does the field-by-field copy, sourcing `trackWidth` from `config_.effectiveTrackWidth()` rather than a `config_.navigator` field (robot_config.proto's Navigator message deliberately has none). NOT persisted -- same "no old curated live-tuning message for this field set" reasoning DRIVE/PLANNER_SHAPER's own rows give |
//
// PERSISTENCE SCOPE (132-013, patch-surface retirement -- sprint.md Out of
// Scope's own explicit ticket-013 acceptance criterion): the reshaped
// `Config::TuningSnapshot` (config/persisted_tuning.h) persists exactly
// WHEEL_CONTROL (in full) + MOTORS.travel_calib_left/right + OTOS (in
// full) -- the SAME set the old curated-message merge accumulator already
// persisted, renamed onto their new Config::Robot homes, neither expanded
// nor contracted. DRIVE (Stage A per-wheel correction, this sprint's OWN
// headline new live-wire capability) and ESTIMATOR are deliberately left
// OUT, matching the precedent that only what a live push could already
// change persisted -- Stage A never had a wire arm before this sprint, and
// Estimator never persisted even when it did have one. GEOMETRY/PLANNER
// are boot-only (never live-pushed at all) and were never candidates.
// PLANNER_SHAPER (132-017, split out of PLANNER) is a live target but is
// likewise left OUT of the persisted set, for the same "no old curated
// live-tuning message ever persisted this field set" reasoning DRIVE's own
// row above documents -- not an oversight.
//
// Owns three things that used to be scattered across RobotLoop (132-013,
// patch-surface retirement -- REVISED from the four-thing list this
// comment used to carry: item 2 below, "present-field merges," is GONE,
// not merely renumbered -- see the git history of this file for the
// pre-132-013 four-thing version):
//   1. The one Config::Robot instance -- loadBaked()/config()/install(),
//      above.
//   2. The persisted-tuning snapshot and its change-detection write policy
//      (flash is written only when the serialized blob actually differs).
//      Reshaped (132-013, config/persisted_tuning.h) from a shape modeled
//      on the old curated live-tuning wire messages, with per-FIELD
//      Opt<T> presence, into a flat, Config::Robot-groups-shaped snapshot
//      with per-GROUP (not per-field) presence -- see persisted_tuning.h's
//      own doc comment for the full reshape and the re-appliability
//      table's own PERSISTENCE SCOPE note above for WHICH groups.
//   3. Pushing values into the subsystems that OWN them: motor gains and
//      per-wheel calibration -> Core::DifferentialDrive and the two Hal::Motor
//      leaves, shaper ceilings -> Motion::Planner, OTOS scalars and
//      offsets -> the OTOS leaf. The OLD curated Estimator live-tuning
//      message's own weight_heading_otos/weight_omega_otos/staleness_ms
//      fields still decode correctly for read-back (config_.estimator,
//      via applyGroup()) but land nowhere on install(), same as
//      `install(ESTIMATOR)`'s own permanent `ERR_UNIMPLEMENTED` above --
//      see the re-appliability table's ESTIMATOR row for the full 132-010
//      reasoning. There is no separate "present-field merge" responsibility
//      any more (the whole curated per-target live-tuning message family
//      and its routing enum are deleted outright, 132-013) -- applyGroup()/
//      applyField() write straight into config_, no patch, no presence
//      flags, no merge, and item 2 above snapshots config_'s own current
//      values rather than accumulating a separate merge state.
//
// Named `Configurator`, not `Config`, because `Config::` is already a
// namespace in this tree (config/persisted_tuning.h) -- a class of that
// name would collide with it.
//
// Boundary: inside -- what a decoded SetConfigGroup/SetConfigField MEANS
// and where its values land; outside -- decoding it (Core::Comms), routing
// it (RobotLoop), and acking it (RobotLoop, from this class's returned
// error code).
#pragma once

#include <cstddef>
#include <cstdint>

#include "core/differential_drive.h"
#include "config/boot_config.h"
#include "config/persisted_tuning.h"
#include "config/robot.h"
#include "hal/motor.h"
#include "hardware/generic/real_otos.h"
#include "messages/envelope.h"
#include "messages/robot_config.h"
#include "motion/navigator/arc_solver.h"
#include "motion/planner/planner.h"

// Config::Robot -- the ONE owned configuration object (issue: the-
// configuration-object.md, sprint 132) -- moved to its own header,
// config/robot.h, by 132-007 (see that header's own doc comment for why:
// breaking a circular #include with app/drive.h, and because the SHAPE
// of Config::Robot is config/'s own data model, not something this class
// owns). Populated at boot by Configurator::loadBaked() (config/
// boot_config.h's generated Config::default*Group() functions, 132-005).
// Decoding a wire push directly into this same object is
// Configurator::applyGroup() (132-008, below); applyField() (single-field
// push) is 132-012.

namespace Core {

class Configurator {
 public:
  // All references are already-constructed modules; the composition root
  // owns construction and wiring order. tuningStore may be null (sim/test
  // roots): persistence disabled, everything else unchanged.
  //
  // navigatorLimits (135-004): the SAME Motion::NavigatorLimits object
  // Motion::Navigator was constructed with a reference to (boot_wiring.h's
  // RobotGraph) -- install(NAVIGATOR) writes straight into it, per this
  // class's own re-appliability table NAVIGATOR row above.
  Configurator(DifferentialDrive& drive, Hal::Motor& motorL, Hal::Motor& motorR,
               Hal::Otos& otos, Motion::Planner& planner,
               Motion::NavigatorLimits& navigatorLimits,
               Config::TuningStore* tuningStore = nullptr);

  // reapplyPersistedTuning() -- main.cpp's own post-boot step
  // (RobotGraph::loadPersistedTuning(), boot_wiring.cpp): writes a loaded
  // TuningSnapshot's persisted fields into config_ (only for a group whose
  // own per-group "tuned" flag is set -- an untouched group's config_
  // value stays whatever loadBaked()/install() already put there, never
  // stomped with a zero-initialized snapshot field), then fans each
  // touched group out via the SAME install(target) a live wire push uses.
  // Finally seeds persistedTuning_/lastPersistedBlob_ from the snapshot so
  // the first live push after boot is change-detected against what is
  // actually in flash, not an empty baseline.
  void reapplyPersistedTuning(const Config::TuningSnapshot& snapshot);

  // config() -- read-back: one call, whole truth (the-configuration-
  // object.md). Reflects whatever loadBaked()/applyGroup() last wrote.
  const Config::Robot& config() const { return config_; }

  // loadBaked() -- populates config_ from the generated, robot-JSON-baked
  // Config::default*Group() functions (config/boot_config.h, 132-005) --
  // one assignment per msg::ConfigGroupTarget, no derivation of its own
  // (every value here mirrors the robot JSON's raw fields, per the
  // issue's "the object holds RAW file values" rule). Idempotent; safe to
  // call again (e.g. a future reload).
  //
  // wheelCorrectionOverride (133-005, nullptr = no override, the hardware
  // default): when non-null, config_.drive's eight wheel_gain_*/
  // wheel_intercept_* fields are replaced by it AFTER the bake lands, so
  // the whole downstream fan-out -- install(DRIVE) here, and every later
  // DifferentialDrive::configure(config_) -- sees the override rather than the baked
  // robot-JSON value. This is the ONE seam that decides what a given
  // composition root's DifferentialDrive is calibrated with, which is exactly why the
  // override belongs here and not at a post-construction setter a
  // refactor can forget to re-apply. Only Core::composeRobot() passes one,
  // and only for the reason BootOverrides::wheelCorrection documents
  // (app/boot_wiring.h).
  void loadBaked(const Config::WheelCorrection* wheelCorrectionOverride = nullptr);

  // install() -- the boot-time fan-out: pushes config_'s re-appliable
  // groups into the subsystems that own them. The applyShaperLimits() call
  // here reads config_.plannerShaper (132-017 split) -- PLANNER itself
  // (the boot-only remainder) has no post-construction setter and is
  // reached only here, at boot, via loadBaked()/install(); PLANNER_SHAPER
  // additionally has an install(target) case below for a LIVE per-target
  // push, since the same setter is safely re-appliable. DRIVE/WHEEL_CONTROL
  // are now retargeted (132-009) onto the SAME
  // `drive_.configure(config_)` call install(DRIVE)/install(WHEEL_CONTROL)
  // use below, rather than re-deriving Stage A/B/C inline a second time.
  // Deliberately NOT RobotLoop's own geometry/rotation configure()
  // (RobotLoop is not a reference this class holds) -- 132-007 gave
  // RobotLoop its own `configure(const Config::Robot&)` entry point
  // instead, and boot_wiring.cpp calls it directly, right after
  // loadBaked(), reading config() for its data exactly the way this method
  // does. MOTORS/OTOS boot values are installed even earlier, at
  // CONSTRUCTION (boot_wiring.cpp's own bakeBootValues() feeds
  // Hal::Motor's/RealOtos's constructors directly) -- this method does
  // not re-touch them at boot; only a LIVE push reaches configureMotor()/
  // configureOtos() (install(target) below).
  //
  // dutyPerSpeed (132-009): now sourced from config_.drive.duty_per_speed_
  // left/right (the active robot JSON) instead of the hardcoded
  // DifferentialDrive::kDutyPerSpeed literal -- see configurator.cpp's own doc comment
  // on this method for the full reversal reasoning (a stakeholder decision
  // change, not an implementation detail). Still boot-only: neither
  // DifferentialDrive::configure() nor install(DRIVE) touch dutyPerSpeed live -- the
  // wire's DRIVE group never carried it and still does not.
  //
  // This is the BOOT-TIME fan-out (every group, once, at construction) --
  // for a LIVE per-target push see install(ConfigGroupTarget) below.
  void install();

  // applyGroup() -- 132-008: the live wire push path (the-configuration-
  // object.md: "a push, applyGroup(...) -> install(target)"). Decodes
  // `wire`/`len` (one group's own encoded body, e.g. a SetConfigGroup.body)
  // straight into config_'s matching member -- no patch, no presence
  // flags, no merge, reusing gen_messages.py's generated
  // msg::wire::decode(<Group>&, ...) family (wire.h/wire.cpp) for the
  // decode/bounds-validate pass, then fans it out via install(target).
  //
  // Boot-only targets (GEOMETRY/PLANNER, see the re-appliability table
  // above) are refused with ERR_NOT_LIVE BEFORE any decode is attempted --
  // config_ is left untouched by a rejected push. A malformed or
  // out-of-bounds payload (ERR_DECODE/ERR_RANGE/ERR_BADARG, from the
  // decode call) also leaves config_ untouched for that target: decodeInto()
  // validates inline and this function does not partially commit a
  // half-decoded group.
  //
  // On success (install(target) returns ERR_NONE), calls
  // persistIfEligible(target) (132-013) -- a no-op for a target outside
  // the persisted-tuning precedent set, a flash write (if the serialized
  // snapshot actually changed) for WHEEL_CONTROL/MOTORS/OTOS.
  msg::ErrCode applyGroup(msg::ConfigGroupTarget target, const uint8_t* wire, size_t len);

  // applyField() -- 132-012 (SetConfigField / the-configuration-object.md's
  // own worked design, "Updating one value: (target, field number,
  // value)"): the single-field counterpart of applyGroup() above -- writes
  // exactly ONE field inside ONE already-live group, addressed by (target,
  // protobuf field number), rather than replacing the whole group.
  //
  // Reuses applyGroup()'s SAME re-appliability gate (`isLiveConfigurable()`
  // -- a boot-only target is refused with ERR_NOT_LIVE BEFORE any field
  // lookup, config_ untouched) and the SAME generated field-table engine
  // applyGroup()'s decode() family uses for the lookup/bounds-validate/
  // write pass (`msg::wire::setField(<Group>&, ...)`, wire.h/wire.cpp's
  // 132-012 addition -- "the loop minus tag decoding," per the design doc).
  //
  // `value` is rejected as ERR_BADARG if it is not finite (NaN or +/-inf)
  // BEFORE `msg::wire::setField()` ever runs: `validateBounds()`'s `<`/`>`
  // bound comparisons are both false for NaN, so an unchecked NaN would
  // otherwise pass every bound a field declares and land in config_ (the
  // same documented trap 132-008 closed for applyGroup()'s own decode
  // path). This keeps config_ untouched on every rejection path --
  // unknown field (ERR_BADARG), non-finite value (ERR_BADARG), and
  // out-of-bounds (ERR_RANGE) are all checked before the single scalar
  // write, and the write itself is one field wide, so there is no
  // multi-field decode to partially commit the way applyGroup()'s
  // whole-group decode can.
  //
  // On success, fans out via install(target) -- the SAME call applyGroup()
  // makes -- so a single-field push takes effect exactly the way a
  // whole-group push does (including MOTORS's ERR_BUSY-while-moving guard
  // and ESTIMATOR's permanent ERR_UNIMPLEMENTED, both install(target)'s
  // own job, unaffected by which caller reached it), and calls
  // persistIfEligible(target) (132-013) exactly like applyGroup() does --
  // a single-field push through a persisted group snapshots that group's
  // now-current full state, same as a whole-group push would.
  msg::ErrCode applyField(msg::ConfigGroupTarget target, uint16_t fieldNumber, float value);

  // install(target) -- 132-008: fans ONE already-decoded group out to the
  // subsystem(s) that own it (the re-appliability table above documents
  // exactly which setter each target reaches, and the two known gaps --
  // OTOS's scale-domain mismatch, ESTIMATOR's missing consumer -- both
  // ticket 010's job). Returns ERR_NOT_LIVE for a boot-only target (should
  // not be reached directly -- applyGroup() already filters -- but kept
  // total rather than assuming a caller always pre-checks), ERR_BUSY if a
  // guarded subsystem (MOTORS) refuses because it is in motion,
  // ERR_UNIMPLEMENTED if the target has no live consumer yet (ESTIMATOR),
  // ERR_NONE on success. Distinct from the boot-time, no-argument
  // install() above, which fans out every group once, at construction.
  msg::ErrCode install(msg::ConfigGroupTarget target);

  // configSource() -- 133-006: `target`'s PROVENANCE, i.e. where the values
  // config() currently reports for it came from (msg::ConfigSource --
  // BAKED / LIVE / PERSISTED; see robot_config.proto's own enum comment).
  // Read-only; the value is written by the three paths that mutate config_
  // and nowhere else (see stampSource() below).
  //
  // Returns CONFIG_SOURCE_UNSPECIFIED for CONFIG_GROUP_UNSPECIFIED and for
  // any out-of-range target -- total, like every other accessor here.
  msg::ConfigSource configSource(msg::ConfigGroupTarget target) const;

  // encodeSnapshot() -- 132-011: read-back's own encode step (the-
  // configuration-object.md: "GetConfig reads the object straight back
  // out"). Fills `out` with `target`'s CURRENT value from config_,
  // reusing the SAME generated per-group codec applyGroup()'s decode()
  // counterpart already uses (msg::wire::encode(<Group>&, ...),
  // gen_messages.py's 132-011 encode-direction addition).
  //
  // 133-006: also fills `out.source` from configSource(target) above, so a
  // read-back answers "what is this group running" and "where did it come
  // from" in the same round trip. Provenance rides the REPLY, never the
  // config struct -- see robot_config.proto's ConfigSource enum for why
  // (a `source` field inside a group would land in the robot JSON and
  // break sprint 132's read-back-equals-file property).
  //
  // NOT gated by isLiveConfigurable()/the re-appliability table above --
  // that table governs WRITES (a boot-only group has no safe runtime
  // setter to re-apply); it says nothing about whether reading the value
  // back out is safe, and it always is. GEOMETRY/PLANNER (boot-only for
  // applyGroup()/install()) read back exactly like every live target.
  //
  // Returns ERR_BADARG for CONFIG_GROUP_UNSPECIFIED or any target this
  // switch does not recognize -- `out` is left with `target` set but its
  // `body`/`body_count` at their default (untouched) state, mirroring
  // applyGroup()'s own "reject before touching anything meaningful"
  // discipline in the read direction. `msg::wire::encode()` returning 0
  // is NOT itself an error signal here (0 is the legitimate encoding of
  // an all-default-valued group, proto3 implicit presence) -- see that
  // function's own doc comment (wire.h).
  msg::ErrCode encodeSnapshot(msg::ConfigGroupTarget target, msg::ConfigSnapshot& out) const;

 private:
  // stampSource() -- 133-006: the ONE writer of groupSource_. Called from
  // exactly the three functions that mutate config_ -- loadBaked() (every
  // group BAKED), reapplyPersistedTuning() (each restored group
  // PERSISTED), and applyGroup()/applyField() (the pushed group LIVE) --
  // and from nowhere else.
  //
  // Stamping at the MUTATION SITE rather than at call sites is the whole
  // point: config_ is private and only configurator.cpp writes it, so
  // "every write is stamped" is a property a reader can verify by reading
  // one file, not a convention that rots. A future mutation path that
  // forgets to stamp leaves a stale provenance -- which is why any new
  // writer of config_ belongs in this file, next to the others.
  //
  // Out-of-range/UNSPECIFIED targets are ignored (no slot to write).
  void stampSource(msg::ConfigGroupTarget target, msg::ConfigSource source);

  // persistIfEligible() -- 132-013: called from applyGroup()/applyField()
  // after install(target) succeeds. Snapshots config_'s CURRENT values for
  // `target`'s persisted subset (the re-appliability table's PERSISTENCE
  // SCOPE note above -- WHEEL_CONTROL in full, MOTORS.travel_calib only,
  // OTOS in full) into persistedTuning_, marks that group's own "tuned"
  // flag, and calls persistTuningIfChanged(). A no-op for any other target
  // (DRIVE/ESTIMATOR/GEOMETRY/PLANNER never persist -- see the table).
  void persistIfEligible(msg::ConfigGroupTarget target);

  // Flash write policy: save only when the serialized snapshot changed.
  void persistTuningIfChanged();

  DifferentialDrive& drive_;
  Hal::Motor& motorL_;
  Hal::Motor& motorR_;
  Hal::Otos& otos_;
  Motion::Planner& planner_;
  Motion::NavigatorLimits& navigatorLimits_;

  // Persisted live-tuning: per-group snapshot of config_'s own current
  // values for the persisted subset (config/persisted_tuning.h), plus the
  // last blob actually written (change-detection baseline).
  Config::TuningStore* tuningStore_ = nullptr;
  Config::TuningSnapshot persistedTuning_ = {};
  Config::Blob lastPersistedBlob_ = {};

  // The one owned configuration object (132-006) -- see loadBaked()/
  // config()/install() above.
  Config::Robot config_ = {};

  // Per-group provenance (133-006), indexed by ConfigGroupTarget's own
  // integer value -- deliberately a SIDE TABLE here rather than a field of
  // Config::Robot: the config object's shape is the robot JSON's shape
  // (config/robot.h), and a runtime-assigned value has no business in a
  // file. Slot 0 (CONFIG_GROUP_UNSPECIFIED) is never a real group and
  // stays CONFIG_SOURCE_UNSPECIFIED forever.
  //
  // Zero-initialized to CONFIG_SOURCE_UNSPECIFIED, which is the honest
  // answer for a Configurator whose loadBaked() has not run yet -- the
  // values in config_ at that point are C++ zero-initialization, not the
  // baked file.
  static constexpr size_t kGroupSourceSlots =
      static_cast<size_t>(msg::ConfigGroupTarget::NAVIGATOR) + 1;
  msg::ConfigSource groupSource_[kGroupSourceSlots] = {};
};

}  // namespace Core
