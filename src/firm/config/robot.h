// robot.h -- Config::Robot: the ONE owned, whole-robot configuration
// object (issue: the-configuration-object.md, sprint 132 "configuration
// discipline"). Composes the seven msg:: robot-config groups generated
// from src/protos/robot_config.proto (messages/robot_config.h), one
// member per msg::ConfigGroupTarget, by value -- Identity/Connection/
// Vision (host-only) are deliberately NOT here, matching the issue's own
// "the object holds RAW file values" framing: this mirrors only the
// groups that ever cross the wire or feed a converter.
//
// 132-006 first built this struct inline inside app/configurator.h.
// 132-007 (subsystem configure() entry points + derived-value methods)
// moved it here, into its own header, for two reasons:
//   1. Derived-value methods (below) are consumed by App::Drive/
//      RobotLoop/the boot_calibration.h Config::Robot-consuming adapters
//      -- all of which configurator.h's own #include "app/drive.h" would
//      put in a circular #include with configurator.h if Config::Robot
//      stayed there (drive.h would need "app/configurator.h" for the
//      type, configurator.h already needs "app/drive.h" for Drive&).
//      A standalone header with no App:: dependency of its own breaks
//      the cycle for every current and future consumer, not just Drive.
//   2. It is genuinely config/'s own data model (a sibling to
//      config/boot_config.h), not something Configurator itself owns the
//      SHAPE of -- Configurator owns an INSTANCE of it (config_) and the
//      dispatch around that instance, which is a different
//      responsibility (see app/configurator.h's own header).
//
// Dependency floor: only messages/robot_config.h, which itself pulls in
// only messages/common.h, which pulls in only <stdint.h> -- no heap, no
// STL, no App::/Devices:: type in reach from this header, transitively.
//
// NOT extended to devices/: despite that dependency floor being exactly
// what the issue's own "the one place this collides with layering"
// section recommends relaxing the devices isolation invariant for
// (src/firm/DESIGN.md §5 / src/firm/devices/DESIGN.md §3: "devices/ must
// not include messages/... or config/..."), this header deliberately
// stays OUTSIDE that relaxation. devices/otos.h's own "Scope changes"
// header section already documents, as a considered design choice, why
// Devices::OtosConfig exists instead of reusing Config::OtosBootConfig
// ("Devices-local so this leaf never includes config/") -- relaxing the
// invariant for this header would contradict that documented rationale
// in place, not just add an exception next to it. 132-007 therefore
// keeps Devices::Motor/Devices::Otos on their existing narrow structs and
// gives the App:: layer (boot_calibration.h's configureMotor()/
// configureOtos(), alongside its existing toDeviceMotorConfig()) the
// Config::Robot-consuming entry point instead -- see that file's own
// header for the full rationale. src/motion/planner/ is stricter still
// (src/motion/DESIGN.md §3: "No Devices::*, App::*, or bus/timing
// collaborator anywhere in this tree" -- Config::* included, explicitly,
// by name, elsewhere in that same section) -- Motion::Planner::
// configure() is therefore likewise NOT a member method; boot_
// calibration.h's configurePlanner() is the adapter, reusing the
// existing applyShaperLimits() setter.
//
// Holds RAW file values only (the-configuration-object.md's "the object
// holds RAW file values" rule) -- e.g. geometry.trackwidth is the
// configured trackwidth, not the scrub-corrected effective track width
// Drive/Odometry/Planner actually want. Derived quantities are METHODS
// below, computed once, so every consumer necessarily agrees (replacing
// today's boot_calibration.cpp:25-29/:63-65/:77-81 fan-out to several
// places with no way to check they still agree).
#pragma once

#include "messages/robot_config.h"

namespace Config {

struct Robot {
  msg::Geometry geometry;
  msg::Motors motors;
  msg::Drive drive;
  msg::WheelControl wheelControl;
  msg::Planner planner;
  msg::Otos otos;
  msg::Estimator estimator;

  // effectiveTrackWidth -- physical separation corrected for SCRUB. Ports
  // App::effectiveTrackWidth()'s formula (boot_calibration.cpp:25-29)
  // onto this object exactly, as a METHOD rather than a stored field
  // (the-configuration-object.md's own worked example) -- read-back stays
  // meaningful because config() serialized only ever carries the raw
  // geometry.trackwidth/rotational_slip fields, never this derived one.
  //
  // Ideal differential kinematics say omega = (vR - vL) / b, but a
  // skid-steer robot drags its wheels sideways through a turn and
  // rotates LESS than that for a given wheel differential --
  // rotational_slip is the measured ratio of actual to ideal rotation,
  // so every kinematic use of the track wants b / slip, not b. slip == 0
  // is the "uncalibrated" sentinel (robot_config.proto's own `{0} u
  // [0.5, 1.0]` domain comment on Geometry.rotational_slip), meaning
  // apply no correction.
  float effectiveTrackWidth() const {  // [mm]
    return geometry.rotational_slip > 0.0f
               ? geometry.trackwidth / geometry.rotational_slip
               : geometry.trackwidth;
  }

  // rotationOffsetPos/rotationOffsetNeg -- the robot JSON's measured
  // turn-response offset, per direction of commanded rotation, DEGREES
  // on the wire/JSON side (what a human reads and what the camera
  // measurement produced) converted to RADIANS here -- matching
  // Motion::Move::threshold and RobotLoop::setRotationCalibration()'s
  // own units. Ports App::installRotationCalibration()'s conversion
  // (boot_calibration.cpp:77-81) exactly, one method per direction.
  float rotationOffsetPos() const {  // [rad]
    return geometry.rotation_offset * kDegToRad;
  }
  float rotationOffsetNeg() const {  // [rad]
    return geometry.rotation_offset_neg * kDegToRad;
  }

  // velocityFilterWeight -- EMA smoothing weight for the planner's plant
  // velocity filter, floored per App::bootPlannerLimits()'s own
  // long-standing sanity check (boot_calibration.cpp:63-65, ported here
  // verbatim as a method): a near-zero or unconfigured alpha would leave
  // the filter frozen at its first sample forever, so anything at or
  // below the 0.05 floor is treated as "unset" and replaced with 1.0 (no
  // smoothing) rather than silently freezing the estimate. Sourced from
  // motors.vel_filt_alpha (Config::Robot's own populated field for this
  // quantity) -- NOT yet wired to replace bootPlannerLimits()'s own
  // inline copy of this same formula at its construction-time call site
  // (that call site reads a different, pre-Config::Robot msg::
  // DrivetrainConfig field that is never populated with a nonzero value
  // today, so retargeting it is a real behavior change, out of this
  // ticket's named scope -- see 132-007's own ticket file).
  float velocityFilterWeight() const {  // dimensionless EMA weight
    return motors.vel_filt_alpha > 0.05f ? motors.vel_filt_alpha : 1.0f;
  }

 private:
  static constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
};

}  // namespace Config
