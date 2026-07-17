// cmd.h -- Motion::Cmd: a normalized arc command, Motion::Executor's own
// internal representation of one decoded msg::Move (envelope.proto).
//
// This is a plain value type -- no validation/clamping logic lives here
// (Executor::enqueue() does the classification: degenerate/DISTANCE/TIMED).
// fromMove() is a field-for-field copy, deliberately not a "smart"
// constructor -- the wire shape (msg::Move) and this struct are the same
// shape on purpose, so there is exactly one place (this file) that would
// need to change if they ever diverged.
#pragma once

#include <cstdint>

#include "messages/envelope.h"

namespace Motion {

struct Cmd {
  float distance = 0.0f;      // [mm] signed arc length along the path
  float deltaHeading = 0.0f;  // [rad] signed heading change over the arc
  float vMax = 0.0f;          // [mm/s] linear ceiling (DISTANCE) / signed target (TIMED)
  float omega = 0.0f;         // [rad/s] signed target yaw rate (TIMED only)
  float time = 0.0f;          // [ms] 0 = distance-bounded; >0 = TIMED (total duration)
  bool replace = false;       // replace last queued (or the active cmd if queue empty)
  uint32_t id = 0;            // host correlation id -- completion events echo this

  // isTimed -- msg::Move's own mode discriminant (envelope.proto's own doc
  // comment): time > 0 selects TIMED mode. time <= 0 is DISTANCE mode
  // UNLESS isDegenerate() below is also true.
  bool isTimed() const { return time > 0.0f; }

  // isDegenerate -- zero distance AND zero heading delta AND not TIMED:
  // nothing for the executor to do. Classified BEFORE isTimed()/DISTANCE
  // dispatch in Executor::enqueue() -- acked TRIVIAL, never queued.
  bool isDegenerate() const {
    return distance == 0.0f && deltaHeading == 0.0f && time <= 0.0f;
  }

  // isPivot -- 109-005: DISTANCE mode (not TIMED, not degenerate -- both
  // already ruled out by the classification order in Executor::enqueue())
  // with zero linear distance: the rotational channel is the dominant (and
  // only planned) channel, driven directly to `deltaHeading`. The
  // complementary DISTANCE case (`distance != 0`, an arc or a straight leg)
  // has no equivalent named helper -- Executor::activate() just tests
  // `!isTimed() && !isPivot()` inline, since "arc" isn't a third thing to
  // classify, just "not a pivot".
  bool isPivot() const { return !isTimed() && distance == 0.0f; }
};

inline Cmd fromMove(const msg::Move& move) {
  Cmd cmd;
  cmd.distance = move.distance;
  cmd.deltaHeading = move.delta_heading;
  cmd.vMax = move.v_max;
  cmd.omega = move.omega;
  cmd.time = move.time;
  cmd.replace = move.replace;
  cmd.id = move.id;
  return cmd;
}

}  // namespace Motion
