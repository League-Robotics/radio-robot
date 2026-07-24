// otos_sample.h -- App::applyOtosSample(): the minimal OTOS-only perception
// step. Split out of odometry.h/.cpp (122-002, motion-library extraction) --
// Motion::Odometry (now src/motion/odometry.{h,cpp}) may not depend on
// Devices::Otos or App::Telemetry::Frame, so this base-side (hardware- and
// telemetry-adjacent) free function stays behind in src/firm/app/ while the
// dead-reckoning integration it used to share a file with moved to
// src/motion/.
//
// Minimal-OTOS-only-perception: this file only ever samples OTOS -- line/
// color sampling (115-005, gut S1) is wired directly into
// App::RobotLoop::updateLineColor(), not through a shared perception class
// here or a round-robin scheduler; each sensor is its own bounded, rate-
// limited step, not a unified abstraction. applyOtosSample() samples the
// Otos leaf and copies the full reading (position, heading, AND the
// measured velocities, per telemetry.proto's OtosReading) straight into a
// Telemetry::Frame -- no perception class, no round-robin scheduler.
// Otos::tick()'s OWN internal rate limiting (kReadPeriod, otos.h) is left
// completely unchanged; applyOtosSample() is safe to call every cycle
// because a too-soon call is already a documented no-bus-traffic no-op
// inside Otos::tick() itself. Bus discipline (never calling this from
// inside a motor request->collect window) is the loop's job; this
// function itself is a single bounded call with no internal sleeps.
#pragma once

#include <cstdint>

#include "app/telemetry.h"
#include "devices/otos.h"

namespace App {

// applyOtosSample() -- see file header. Samples `otos` (rate-limited
// internally by its own readDue()/kReadPeriod, unchanged) and copies the
// full reading (x, y, heading, v_x, v_y, omega) plus the burst's own read
// time into `frame`'s `otos` field; call this BEFORE the caller's next
// Telemetry::setFrame(frame)/emit() -- it must reach Telemetry before that
// cycle's frame is built.
//
// `frame.otosPresent` -- 115-005: the new telemetry.proto flags bit 0
// (otos_present) is documented as "OtosReading fresh THIS frame", a
// tighter contract than the old (pre-115) hasOtos, which mirrored
// otos.present() (a chip was EVER detected at boot) so a rate-limit-skipped
// cycle wouldn't flip it off. Now that OtosReading carries its own `time`
// field, freshness itself is the signal a caller needs -- frame.otosPresent
// is therefore `otos.present() && otos.poseFresh()`: true only on a cycle
// this function's own otos.tick() call actually refreshed the cached pose.
// `frame.otosConnected` mirrors the leaf's own live, per-tick connected(),
// unchanged from before. No pose fusion happens here -- the robot does not
// fuse; the raw OTOS pose rides to the host verbatim for host-side fusion.
// `frame.otos` itself is only overwritten when otosPresent is true --
// otherwise it is left exactly as the caller last staged it (Telemetry's
// own "last staged snapshot" contract), even though the flags bit that
// gates its validity will correctly read false that frame.
void applyOtosSample(Devices::Otos& otos, uint64_t now, Telemetry::Frame& frame);  // [us]

}  // namespace App
