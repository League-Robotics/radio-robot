// configurator.h -- App::Configurator: the CONFIG command's whole
// lifecycle (command-ingestion-ring-buffered-comms-subsystem-routing-two-
// stops.md §6). One of the four routing destinations App::RobotLoop::
// routeCommand() dispatches to, alongside App::Drive (WHEELS) and
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
// flags, no merge -- ticket 002's generated msg::Geometry/Motors/Drive/
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
//   | GEOMETRY      | NO    | -- (ERR_NOT_LIVE)                                 | trackWidth has no post-construction setter anywhere (Drive/Odometry/Planner all bake it in at construction); rotation calibration installs once, at boot, via RobotLoop::configure() called directly from boot_wiring.cpp, not through this class |
//   | PLANNER       | NO    | -- (ERR_NOT_LIVE)                                 | vMax/omegaMax/controlPeriod/actuationDelay/landing.*/headingHoldGain have no post-construction setter; only the (already-live) shaper ceilings are re-appliable, and those ride the boot-time no-arg install(), not a per-target push |
//   | DRIVE         | yes   | `Drive::configure(config_)` (132-007)             | Stage A per-wheel gain/intercept + crawl pulse, via the existing `setWheelCorrection()`/`setCrawlPulse()` |
//   | WHEEL_CONTROL | yes   | `Drive::configure(config_)` (132-007)             | SAME call as DRIVE -- Drive::configure() pulls Stage B/C gains/bounds from `config_.wheelControl` in the same pass it reads `config_.drive` from; one re-appliable unit from Drive's own point of view, two ConfigGroupTargets on the wire |
//   | MOTORS        | yes   | `App::configureMotor()` x2 (132-007), GUARDED     | refuses (returns false, applies nothing) while that side reports itself in motion -- surfaced as `ERR_BUSY`, never swallowed, so a caller can retry at rest |
//   | OTOS          | yes   | `App::configureOtos()` (132-007/010)              | trap 3 CLOSED (132-010): `linear_scale`/`angular_scale` are converted through `Devices::scaleToRegister()` before reaching `setLinearScalar()`/`setAngularScalar()`, matching `RealOtos::begin()`'s own boot-time conversion -- a live push and a boot bake now agree on what a given multiplier means. `apply()`'s OLD `applyOtosPatch()` still has the pass-through bug -- that surface is retired outright by ticket 013, not patched here |
//   | ESTIMATOR     | yes   | -- (`ERR_UNIMPLEMENTED`, PERMANENT)               | trap 2 CLOSED (132-010) by making the dead end EXPLICIT rather than inventing a consumer: `App::StateEstimator` was deleted outright as dead code (sprint 128 ticket 016), and its one candidate successor -- `Motion::PoseTracker::blendHeading()` (`src/motion/planner/estimation.h`) -- had its only call site AND its own config fields (`PlannerLimits::headingOtosWeight`/`otosStaleness`) deleted outright by 130-009, in favor of a from-scratch fusion redesign tracked at `clasi/issues/later/estimator-v2-otos-fusion-sim-first.md`. Building a real consumer today would mean either resurrecting logic 130-009 deliberately retired, or building estimator-v2 itself -- both out of this ticket's scope (and the latter explicitly deferred by its own tracked issue). `Configurator` therefore holds NO estimator-shaped reference; `config_.estimator` still decodes correctly for read-back (`applyGroup()` never skips the decode for a live-classified target), but `install(ESTIMATOR)` returns `ERR_UNIMPLEMENTED` permanently until estimator-v2 gives it something real to call |
//
// Owns four things that used to be scattered across RobotLoop:
//   1. The one Config::Robot instance -- loadBaked()/config()/install(),
//      above.
//   2. Present-field patch merges -- a ConfigDelta carries only the fields
//      the host set, so applying one is a merge onto the cumulative
//      snapshot, never an overwrite. (Unchanged by 132-006 -- this is the
//      OLD *ConfigPatch wire surface, retired by ticket 013, not this
//      ticket.)
//   3. The persisted-tuning snapshot and its change-detection write policy
//      (flash is written only when the serialized blob actually differs).
//   4. Pushing values into the subsystems that OWN them: motor gains and
//      per-wheel calibration -> App::Drive and the two Devices::Motor
//      leaves, shaper ceilings -> Motion::Planner, OTOS scalars and
//      offsets -> the OTOS leaf. The OLD ESTIMATOR patch surface's own
//      weight_heading_otos/weight_omega_otos/staleness_ms fields are still
//      accepted on the wire (that surface is unchanged here -- retired
//      outright by ticket 013) but land nowhere, same as
//      `install(ESTIMATOR)`'s own permanent `ERR_UNIMPLEMENTED` above --
//      see apply()'s own ESTIMATOR-branch comment and the re-appliability
//      table's ESTIMATOR row for the full 132-010 reasoning.
//
// Named `Configurator`, not `Config`, because `Config::` is already a
// namespace in this tree (config/persisted_tuning.h) -- a class of that
// name would collide with it.
//
// Boundary: inside -- what a decoded ConfigDelta MEANS and where its values
// land; outside -- decoding it (App::Comms), routing it (RobotLoop), and
// acking it (RobotLoop, from this class's returned error code).
#pragma once

#include <cstdint>

#include "app/drive.h"
#include "config/persisted_tuning.h"
#include "config/robot.h"
#include "devices/motor.h"
#include "devices/otos.h"
#include "messages/envelope.h"
#include "messages/robot_config.h"
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
// push) is still ticket 012's job.

namespace App {

class Configurator {
 public:
  // All references are already-constructed modules; the composition root
  // owns construction and wiring order. tuningStore may be null (sim/test
  // roots): persistence disabled, everything else unchanged.
  Configurator(Drive& drive, Devices::Motor& motorL, Devices::Motor& motorR,
               Devices::Otos& otos, Motion::Planner& planner,
               Config::TuningStore* tuningStore = nullptr);

  // Apply one decoded CONFIG command. Returns the msg::ErrCode to ack with
  // -- 0 on success, ERR_UNIMPLEMENTED for a declared-but-unwired patch
  // arm. RobotLoop's whole CONFIG handler is this call plus the ack.
  uint32_t apply(const msg::CommandEnvelope& env);

  // Apply a loaded TuningSnapshot through the SAME per-kind appliers a live
  // patch uses, and seed the write-policy baseline so the first live patch
  // after boot is compared against what is actually in flash.
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
  void loadBaked();

  // install() -- the boot-time fan-out: pushes config_'s re-appliable
  // groups into the subsystems that own them. PLANNER stays an inline
  // applyShaperLimits() call (boot-only -- no install(target) case exists
  // for it). DRIVE/WHEEL_CONTROL are now retargeted (132-009) onto the SAME
  // `drive_.configure(config_)` call install(DRIVE)/install(WHEEL_CONTROL)
  // use below, rather than re-deriving Stage A/B/C inline a second time.
  // Deliberately NOT RobotLoop's own geometry/rotation configure()
  // (RobotLoop is not a reference this class holds) -- 132-007 gave
  // RobotLoop its own `configure(const Config::Robot&)` entry point
  // instead, and boot_wiring.cpp calls it directly, right after
  // loadBaked(), reading config() for its data exactly the way this method
  // does. MOTORS/OTOS boot values are installed even earlier, at
  // CONSTRUCTION (boot_wiring.cpp's own bakeBootValues() feeds
  // Devices::Motor's/RealOtos's constructors directly) -- this method does
  // not re-touch them at boot; only a LIVE push reaches configureMotor()/
  // configureOtos() (install(target) below).
  //
  // dutyPerSpeed (132-009): now sourced from config_.drive.duty_per_speed_
  // left/right (the active robot JSON) instead of the hardcoded
  // Drive::kDutyPerSpeed literal -- see configurator.cpp's own doc comment
  // on this method for the full reversal reasoning (a stakeholder decision
  // change, not an implementation detail). Still boot-only: neither
  // Drive::configure() nor install(DRIVE) touch dutyPerSpeed live -- the
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
  // own job, unaffected by which caller reached it).
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

  // encodeSnapshot() -- 132-011: read-back's own encode step (the-
  // configuration-object.md: "GetConfig reads the object straight back
  // out"). Fills `out` with `target`'s CURRENT value from config_,
  // reusing the SAME generated per-group codec applyGroup()'s decode()
  // counterpart already uses (msg::wire::encode(<Group>&, ...),
  // gen_messages.py's 132-011 encode-direction addition).
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
  // Per-kind appliers, shared by apply() and reapplyPersistedTuning().
  void applyMotorConfigPatch(const msg::MotorConfigPatch& patch);
  void applyOtosPatch(const msg::OtosConfigPatch& patch);

  // Flash write policy: save only when the serialized snapshot changed.
  void persistTuningIfChanged();

  Drive& drive_;
  Devices::Motor& motorL_;
  Devices::Motor& motorR_;
  Devices::Otos& otos_;
  Motion::Planner& planner_;

  // Persisted live-tuning: cumulative merge of every tuned field, plus the
  // last blob actually written (change-detection baseline).
  Config::TuningStore* tuningStore_ = nullptr;
  Config::TuningSnapshot persistedTuning_ = {};
  Config::Blob lastPersistedBlob_ = {};

  // The one owned configuration object (132-006) -- see loadBaked()/
  // config()/install() above.
  Config::Robot config_ = {};
};

}  // namespace App
