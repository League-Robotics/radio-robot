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
// .cpp for the composition-root side of this split. Full per-group wire
// decode (`applyGroup()`/`applyField()`) and per-target re-appliability
// enforcement are later tickets (008/012); this ticket's `install()` is
// the boot-time fan-out only, and may not yet cover every group
// `config_` holds -- see install()'s own doc comment.
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
//      offsets -> the OTOS leaf. The ESTIMATOR patch's own
//      weight_heading_otos/weight_omega_otos/staleness_ms fields are
//      accepted on the wire but land nowhere -- there is no live consumer
//      for them -- see apply()'s own ESTIMATOR-branch comment.
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
#include "motion/planner/planner.h"

// Config::Robot -- the ONE owned configuration object (issue: the-
// configuration-object.md, sprint 132) -- moved to its own header,
// config/robot.h, by 132-007 (see that header's own doc comment for why:
// breaking a circular #include with app/drive.h, and because the SHAPE
// of Config::Robot is config/'s own data model, not something this class
// owns). Populated at boot by Configurator::loadBaked() (config/
// boot_config.h's generated Config::default*Group() functions, 132-005).
// Decoding a wire push directly into this same object
// (Configurator::applyGroup()) is tickets 008/012's job, not this
// ticket's -- Configurator::install() here only reads it back out for
// the boot-time fan-out.

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
  // object.md). Reflects whatever loadBaked()/applyGroup() last wrote;
  // applyGroup() itself is not built until ticket 008.
  const Config::Robot& config() const { return config_; }

  // loadBaked() -- populates config_ from the generated, robot-JSON-baked
  // Config::default*Group() functions (config/boot_config.h, 132-005) --
  // one assignment per msg::ConfigGroupTarget, no derivation of its own
  // (every value here mirrors the robot JSON's raw fields, per the
  // issue's "the object holds RAW file values" rule). Idempotent; safe to
  // call again (e.g. a future reload).
  void loadBaked();

  // install() -- the boot-time fan-out: pushes config_'s re-appliable
  // groups into the subsystems that own them, through the SAME setters
  // the old free-function install*Calibration() family (boot_calibration.h)
  // used -- installShaperLimits()/installDriveCalibration()/
  // installWheelController()'s bodies, ported here reading config_ instead
  // of a resolve()-computed struct. Deliberately NOT RobotLoop's own
  // geometry/rotation configure() (RobotLoop is not a reference this
  // class holds) -- 132-007 gave RobotLoop its own
  // `configure(const Config::Robot&)` entry point instead, and
  // boot_wiring.cpp calls it directly, right after loadBaked(), reading
  // config() for its data exactly the way this method does. This
  // method's own body is also NOT yet retargeted to call the new
  // Drive::configure()/Motion::Planner-adapter/Motor-adapter/Otos-adapter
  // entry points 132-007 added -- that retarget is tickets 008/009/010's
  // job (applyGroup()'s per-target dispatch, Drive::kDutyPerSpeed-vs-JSON,
  // OTOS/ESTIMATOR correctness); this is still a straightforward
  // relocation of what already worked.
  void install();

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
