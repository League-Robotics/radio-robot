// boot_calibration.h -- App:: free functions that convert the generated
// Config::boot_config.cpp bake into the App:: / Motion:: types the
// composition root wires together, and install that calibration onto an
// already-constructed graph.
//
// Deliberately a SEPARATE translation unit from boot_wiring.{h,cpp} (which
// holds composeRobot()/RobotGraph -- the graph CONSTRUCTION): every
// function here reads config/boot_config.h (the generated, robot-JSON-
// baked constants), while boot_wiring.cpp's own graph-construction code
// does not need to be recompiled/relinked just because a caller only wants
// one small conversion helper (e.g. effectiveTrackWidth()). Splitting the
// two avoids dragging boot_wiring.cpp's wider App::/Motion:: graph
// dependencies into a test that only wants a calibration conversion, and
// vice versa -- see 130-002's own ticket note on the "four-source-list
// trap": a caller that links boot_calibration.cpp without boot_wiring.cpp
// pulls in config/boot_config.cpp's generated robot bake, nothing else.
//
// Every function here is a straight extraction of main.cpp's own pre-130-
// 002 inline conversion/install code (toDeviceMotorConfig()/
// toPlannerLimits() and the DriveBootConfig/rotation-calibration install
// blocks) -- see git history around 45cd56df and earlier for the
// pre-extraction shape. Behavior is unchanged; only the location moved.
#pragma once

#include "app/drive.h"
#include "app/robot_loop.h"
#include "config/boot_config.h"
#include "devices/device_config.h"
#include "devices/motor.h"
#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "motion/planner/planner.h"
#include "motion/planner/planner_types.h"

namespace App {

// toDeviceMotorConfig -- converts the boot config's wire-plane
// msg::MotorConfig into the Devices-local MotorConfig NezhaMotor's
// constructor needs. Lives here (not devices/) because the devices/
// isolation invariant (DESIGN.md) forbids that leaf layer from including
// messages/ or config/.
//
// vel_gains/vel_filt_alpha/min_duty are NOT copied here -- Devices::
// MotorConfig has no such fields (the velocity PID they fed has been
// deleted outright -- see drive.h's own header). vel_filt_alpha has no
// live consumer at all -- the wire field itself is untouched (no protocol
// change), simply unread by firmware for now.
Devices::MotorConfig toDeviceMotorConfig(const msg::MotorConfig& src);

// effectiveTrackWidth -- physical separation corrected for SCRUB.
//
// Ideal differential kinematics say omega = (vR - vL) / b, but a skid-steer
// robot drags its wheels sideways through a turn and rotates LESS than that
// for a given wheel differential. rotational_slip is the measured ratio of
// actual to ideal rotation, so every kinematic use of the track wants
// b / slip, not b.
//
// `trackwidth` stays the caliper-measured wheel separation and must NOT be
// bent to absorb scrub -- that would destroy the one value in the robot
// JSON that is independently verifiable, and hide the scrub instead of
// measuring it.
//
// slip == 0 is the "uncalibrated" sentinel (config.proto's `{0} u
// [0.5, 1.0]` domain), meaning apply no correction.
float effectiveTrackWidth(const msg::DrivetrainConfig& drivetrainConfig);  // [mm]

// bootPlannerLimits -- converts Config::defaultPlannerLimits() (the
// plant-validated tuning baked from the active robot JSON's own `planner`
// section) into a Motion::PlannerLimits, with trackWidth/
// velocityFilterWeight sourced from drivetrainConfig/trackWidth (the two
// PlannerLimits fields NOT carried by Config::PlannerBootConfig -- see
// that struct's own doc comment).
Motion::PlannerLimits bootPlannerLimits(const msg::DrivetrainConfig& drivetrainConfig,
                                        float trackWidth);  // [mm]

// installShaperLimits -- marks shaping CONFIGURED through the same
// applyShaperLimits() entry the wire push uses, with `limits`'s own
// validated ceilings, so kFlagFaultShapingDisabled stays quiet.
void installShaperLimits(Motion::Planner& planner, const Motion::PlannerLimits& limits);

// installRotationCalibration -- the robot JSON's measured turn response
// (`actual = gain*commanded + offset`, per direction), installed onto
// robotLoop. Degrees on the wire/JSON side (what a human reads and what
// the camera measurement produced), radians inside, matching
// Motion::Move::threshold -- see RobotLoop::setRotationCalibration().
void installRotationCalibration(RobotLoop& robotLoop,
                                const msg::DrivetrainConfig& drivetrainConfig);

// installDriveCalibration -- installs this robot's own wheel calibration
// (command-ingestion-ring-buffered-comms-subsystem-routing-two-stops.md
// §6) onto drive: the measured plant-inverse duty-per-speed constant
// (App::Drive::kDutyPerSpeed -- MEASURED, NOT CONFIGURED, stakeholder
// 2026-07-31; deliberately NOT driveConfig.dutyPerSpeedLeft/Right, see
// Drive::kDutyPerSpeed's own doc comment), the per-wheel gain/intercept
// correction, and the crawl-pulse amplitude.
void installDriveCalibration(Drive& drive, const Config::DriveBootConfig& driveConfig);

}  // namespace App
