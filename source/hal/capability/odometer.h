// odometer.h — the Odometer faceplate (e.g. an OTOS-style optical
// odometry sensor). Declaration only this ticket — see
// capability/gripper.h's file header for the "declared, not defined"
// mechanism; the same applies here.
//
// Gap note (flagged for the team lead / a follow-up ticket): unlike the
// other four unimplemented faceplates, ticket 002 (protos + message regen)
// generated no dedicated Odometer{Command,Config,Capabilities} message at
// all — no protos/odometer.proto exists, only the shared msg::PoseEstimate
// observation type (protos/common.proto), which DrivetrainState already
// reuses for its fused/encoder/optical fields. This faceplate is built
// against that gap as follows:
//  - State: msg::PoseEstimate (pose + twist + freshness stamp) — an honest
//    fit, reused rather than duplicated.
//  - Command: none. An odometer is a pure sensor; there is nothing to
//    apply() until a leaf needs one (e.g. a pose-reset command), so no
//    apply() exists on this faceplate at all.
//  - Config / Capabilities: none generated, so neither configure() nor
//    capabilities() appears here. A later ticket that adds a concrete
//    odometer leaf (OTOS wrapper or similar) and finds it needs tunable
//    parameters should add a dedicated proto at that point rather than
//    retrofitting one now with no leaf to validate it against.
#pragma once

#include <stdint.h>

#include "messages/common.h"

namespace Hal {

class Odometer {
 public:
  virtual ~Odometer() = default;
  virtual void begin() {}

  // Primitive getter — the real read.
  virtual msg::PoseEstimate pose() const = 0;
  virtual bool connected() const = 0;

  // Faceplate verb (no Config/Capabilities message exists — see file
  // header).
  virtual void tick(uint32_t now) = 0;   // [ms]

  // Message plane — declared, not defined (no concrete leaf this sprint).
  // No apply(): read-only sensor, no Command message.
  msg::PoseEstimate state() const;
};

}  // namespace Hal
