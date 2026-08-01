// configurator.h -- App::Configurator: the CONFIG command's whole
// lifecycle (command-ingestion-ring-buffered-comms-subsystem-routing-two-
// stops.md §6). One of the four routing destinations App::RobotLoop::
// routeCommand() dispatches to, alongside App::Drive (WHEELS) and
// Motion::Planner (MOVE/STOP).
//
// Owns three things that used to be scattered across RobotLoop:
//   1. Present-field patch merges -- a ConfigDelta carries only the fields
//      the host set, so applying one is a merge onto the cumulative
//      snapshot, never an overwrite.
//   2. The persisted-tuning snapshot and its change-detection write policy
//      (flash is written only when the serialized blob actually differs).
//   3. Pushing values into the subsystems that OWN them: motor gains and
//      per-wheel calibration -> App::Drive and the two Devices::Motor
//      leaves, shaper ceilings -> Motion::Planner, OTOS scalars and
//      offsets -> the OTOS leaf. The ESTIMATOR patch's own
//      weight_heading_otos/weight_omega_otos/staleness_ms fields are
//      accepted on the wire but land nowhere (128-016,
//      robot-state-pose-needs-exactly-one-writer.md): they used to feed
//      Motion::StateEstimator, deleted this ticket as a per-cycle
//      computation with no consumer -- see apply()'s own ESTIMATOR-branch
//      comment.
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
#include "devices/motor.h"
#include "devices/otos.h"
#include "messages/envelope.h"
#include "motion/planner/planner.h"

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
};

}  // namespace App
