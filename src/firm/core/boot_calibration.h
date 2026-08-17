// boot_calibration.h -- Core:: free functions that convert the generated
// Config::boot_config.cpp bake into the Core::/Control:: types the
// composition root wires together, and install that calibration onto an
// already-constructed graph.
//
// Deliberately a SEPARATE translation unit from boot_wiring.{h,cpp} (which
// holds composeRobot()/RobotGraph -- the graph CONSTRUCTION): every
// function here reads config/boot_config.h (the generated, robot-JSON-
// baked constants), while boot_wiring.cpp's own graph-construction code
// does not need to be recompiled/relinked just because a caller only wants
// one small conversion helper. Splitting the two avoids dragging
// boot_wiring.cpp's wider Core:: graph dependencies into a test that only
// wants a calibration conversion, and vice versa.
//
// EXPLORATORY-KERNEL REWRITE (2026-08-15,
// clasi/issues/differentialdrive-one-class-one-fiber-exploratory-worktree.md):
// bootPlannerLimits()/configurePlanner()/configureNavigator() and their
// Motion::* return/parameter types are DELETED along with the whole
// src/firm/motion/ tree -- Motion::Planner/Motion::Navigator no longer
// exist, so there is nothing left for either function to configure.
// toDeviceMotorConfig() drops its `wheelTravelCalib` line (the leaf is
// counts-native now, Hal::MotorConfig has no such field) and gains
// `writeThrottle`, derived from the kernel's own cycle period.
// buildDriveKernelConfig() is NEW: the one place a whole
// Config::Robot (DRIVE + WHEEL_CONTROL groups, plus MOTORS.travel_calib_*
// for the mm<->counts rebake) becomes a Control::DifferentialDrive::Config
// -- the kernel's own config object, in counts. configureMotor() is
// DELETED outright: its entire job was pushing `travel_calib` onto the
// motor leaf via the now-deleted Hal::Motor::applyTravelCalib(); travel
// calibration lives at THIS layer now (feeding buildDriveKernelConfig()),
// not on the leaf, so there is nothing left for it to do.
#pragma once

#include "config/robot.h"
#include "control/differential_drive.h"
#include "hal/device_config.h"
#include "hal/motor.h"
#include "hardware/generic/real_otos.h"
#include "messages/motor.h"

namespace Core {

// kKernelCyclePeriod -- the wheel kernel's own fiber cadence.
// TODO(config): not yet a JSON key. The exploratory tree bakes this as a
// plain C++ constant rather than adding a robot-JSON field for it --
// promoting it to `wheel_control.cycle_period` (or similar) in
// gen_boot_config.py is real, wanted follow-up work once this kernel
// design is no longer exploratory (configuration-discipline.md: "we have
// to be able to configure everything we bake" -- this is the one
// documented exception, and it is documented for exactly that reason).
constexpr uint32_t kKernelCyclePeriod = 24;  // [ms]

// toDeviceMotorConfig -- converts the boot config's wire-plane
// msg::MotorConfig into the Devices-local MotorConfig NezhaMotor's
// constructor needs. Lives here (not devices/) because the devices/
// isolation invariant (DESIGN.md) forbids that leaf layer from including
// messages/ or config/.
//
// wheelTravelCalib is NOT copied (Hal::MotorConfig has no such field any
// more -- the leaf is counts-native, 2026-08-15). writeThrottle is NEW:
// replaces the leaf's old hand-synced kMinWriteIntervalUs literal
// (nezha_motor.cpp's own historical TRAP note) with a config-derived
// value, `(kKernelCyclePeriod - 5) * 1000` [us] -- the same "cycle minus a
// few ms of jitter margin" shape the old literal encoded, now tracking the
// KERNEL's cycle period (the only fiber writing duty) instead of the
// long-gone RobotLoop::kCycle it used to silently assume.
//
// vel_gains/vel_filt_alpha/min_duty are NOT copied here -- Devices::
// MotorConfig has no such fields (the velocity PID they fed was deleted
// long before this rewrite). vel_filt_alpha has no live consumer at all --
// the wire field itself is untouched (no protocol change), simply unread
// by firmware for now.
Hal::MotorConfig toDeviceMotorConfig(const msg::MotorConfig& src);

// buildDriveKernelConfig -- the ONE place a whole Config::Robot becomes a
// Control::DifferentialDrive::Config: DRIVE (Stage A wheel correction +
// crawl pulse) and WHEEL_CONTROL (Stage B/C gains/bounds + stall/deficit
// windows), converted from the robot JSON's mm/mm-s domain into the
// kernel's native counts/counts-s domain via MOTORS.travel_calib_left/
// right (mean, per the "one population-scale value" convention
// duty_per_speed already follows -- see configurator.cpp's own doc
// comment on that convention). `fullDutyVelocity` is the one PER-WHEEL
// conversion (duty_per_speed_left/right each divide by their OWN wheel's
// travel_calib before being averaged into one kernel-wide plant gain --
// the kernel has no per-wheel fullDutyVelocity slot, matching
// duty_per_speed's own existing single-value convention).
//
// twistHoldGain has no robot-JSON key yet (no wire field exists for it) --
// left at 0 (off); a bench session sets it live via
// DifferentialDrive::setTwistHoldGain() directly. maxDuty likewise has no
// robot-JSON key -- left at Config's own 100.0f default (full authority).
// cyclePeriod is always kKernelCyclePeriod, never robot-JSON-sourced (see
// that constant's own TODO(config) note).
//
// Called from Configurator::install() every time DRIVE, WHEEL_CONTROL, or
// MOTORS (travel_calib) changes -- see configurator.h's re-appliability
// table -- so a live push of any of the three actually reaches the
// kernel, per configuration-discipline.md's "every value in the file
// reaches the robot" half.
Control::DifferentialDrive::Config buildDriveKernelConfig(const Config::Robot& config);

// configureOtos -- reuses setLinearScalar()/setAngularScalar()/setOffset(),
// the SAME setters both boot and a live OTOS config push have always
// called. linear_scale/angular_scale are converted through
// Hardware::scaleToRegister() (otos.h) before reaching the chip-level
// setters -- the SAME conversion RealOtos::begin() applies to the baked
// value at boot, so a live push and a boot bake agree on what a given
// multiplier means. Unaffected by the exploratory-kernel rewrite; kept
// here verbatim (motion/'s deletion has nothing to do with OTOS).
void configureOtos(Hal::Otos& otos, const Config::Robot& config);

}  // namespace Core
