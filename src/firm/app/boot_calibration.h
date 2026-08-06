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

#include "config/robot.h"
#include "devices/device_config.h"
#include "devices/motor.h"
#include "devices/otos.h"
#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "motion/navigator/arc_solver.h"
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

// installShaperLimits/installDriveCalibration/installWheelController --
// DELETED, 132-015 (dead-code sweep, the-configuration-object.md). All
// three had already lost their only call site by 132-006/132-009
// (Configurator::install(), configurator.cpp, does the equivalent
// fan-out inline instead, reading Config::Robot directly) -- this
// ticket's own fresh grep re-confirmed zero remaining callers and
// deleted them, alongside DriveBootConfig/WheelControllerBootConfig
// (config/boot_config.h, deleted the same ticket -- see that file's own
// note for the struct-level half of this cleanup).
//
// installRotationCalibration -- DELETED (132-007). Superseded by
// RobotLoop::configure(const Config::Robot&) (robot_loop.h), which reads
// config/robot.h's rotationOffsetPos()/rotationOffsetNeg() derived
// methods instead of doing its own degrees->radians conversion inline;
// boot_wiring.cpp's constructor calls robotLoop_.configure(configurator_.
// config()) directly now, in place of this function's only call site.

// --- Config::Robot-consuming entry points (132-007, the-configuration-
// object.md's "subsystems take the whole object" pattern) for the THREE
// subsystems that cannot take a Config::Robot& as a member method
// themselves --
//   - Motion::Planner (src/motion/planner/): that tree's own, narrower
//     dependency rule (src/motion/DESIGN.md §3) forbids ANY
//     App::/Devices::/Config:: dependency, no exception -- "No Devices::*,
//     App::*, or bus/timing collaborator anywhere in this tree."
//   - Devices::Motor / Devices::Otos (src/firm/devices/): the devices
//     isolation invariant (src/firm/DESIGN.md §5) forbids devices/ from
//     including messages/ or config/ headers, and (otos.h's own "Scope
//     changes" section) Devices::Otos deliberately keeps its OWN narrow
//     Devices::OtosConfig for exactly this reason today.
// toDeviceMotorConfig() above already lives here, in App::, for the
// identical reason -- these three functions are that same pattern
// extended to Config::Robot: App:: is the one layer that can see both a
// Config:: type and the lower-layer subsystem's own API, so the
// conversion/apply happens here, not down in the subsystem itself. Each
// reuses an EXISTING setter -- no new firmware control logic.

// configurePlanner -- reuses Motion::Planner::applyShaperLimits(), the
// SAME setter Configurator::install() (configurator.cpp) already calls,
// reading config.planner's six shaper-ceiling fields.
void configurePlanner(Motion::Planner& planner, const Config::Robot& config);

// configureMotor -- reuses Devices::Motor::applyTravelCalib(), the ONE
// MotorConfig field this interface still live-applies post-construction
// (motor.h's own doc comment), side-selected exactly like RobotLoop's
// own CONFIG merge path already does (isLeft picks motors.
// travel_calib_left vs. travel_calib_right). Guarded the same way
// NezhaMotor::reconfigure() is guarded (nezha_motor.cpp): refuses
// (returns false, applies nothing) while the motor reports itself in
// motion via its own public velocity()/appliedDuty() accessors -- the
// SAME "is moving" signal reconfigure()'s own atRest check uses, read
// through the public Devices::Motor interface rather than a
// leaf-private field, since this function operates on the interface,
// not a concrete leaf. Configurator maps a `false` return to ERR_BUSY
// (ticket 009's own job); this function's scope is only the bool.
[[nodiscard]] bool configureMotor(Devices::Motor& motor, const Config::Robot& config,
                                  bool isLeft);

// configureOtos -- reuses setLinearScalar()/setAngularScalar()/
// setOffset(), the SAME setters Configurator::applyOtosPatch() (the old
// patch surface, configurator.cpp) already call. Trap 3's multiplier-
// vs-register domain mismatch is CLOSED here (132-010): linear_scale/
// angular_scale are converted through Devices::scaleToRegister() (otos.h,
// a free function since 132-010 -- see its own doc comment) before
// reaching the chip-level setters, the same conversion RealOtos::begin()
// applies to the baked value at boot, so a live push and a boot bake now
// agree on what a given multiplier means.
void configureOtos(Devices::Otos& otos, const Config::Robot& config);

// configureNavigator -- 135-004: writes config.navigator's fields DIRECTLY
// into `limits` -- not a setter call, because Motion::Navigator holds its
// NavigatorLimits by const reference (navigator.h's own design: `tick()`
// reads `limits_` fresh every call, so there is no "re-apply" step to
// invoke at all -- a write here takes effect on the very next tick()).
// `limits` is therefore RobotGraph's own owned Motion::NavigatorLimits
// object (boot_wiring.h), the SAME one Motion::Navigator was constructed
// with a reference to -- never a temporary.
//
// trackWidth is deliberately NOT copied from config.navigator (that
// message has no such field, by design -- robot_config.proto's Navigator
// message header comment): sourced from config.effectiveTrackWidth()
// instead, the SAME scrub-corrected value Drive/Odometry/PlannerLimits
// already use, so there is exactly one place trackwidth/rotational_slip
// are combined, never a second copy that can drift from it.
void configureNavigator(Motion::NavigatorLimits& limits, const Config::Robot& config);

}  // namespace App
