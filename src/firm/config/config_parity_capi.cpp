// config_parity_capi.cpp -- see config_parity_capi.h for the contract this
// file implements and why it exists.
//
// Every offset below is looked up by FIELD NAME via offsetof(), exactly
// like plannerLimitsOffsets() (src/motion/planner/capi.cpp:93-117). That
// choice is what makes the guard catch a mid-struct insertion rather than
// just a size change: offsetof(T, someField) always reports wherever
// `someField` REALLY sits in the current robot_config.h, so if a field is
// inserted ahead of it, every offset after the insertion point shifts
// automatically -- this file does not need editing for that drift to show
// up at runtime. A field RENAME or DELETION in robot_config.h instead fails
// this file's own compile (offsetof() on a nonexistent member), which is an
// even earlier catch than the Python harness gets to see.
#include "config_parity_capi.h"

#include <cstddef>

#include "messages/robot_config.h"

namespace {

// Writes min(count, total) entries from `table` into `out`; always returns
// the true total. Shared by every export below (capi.cpp's own version of
// this loop is inlined once, since it has a single call site there; this
// file has eight, so it is factored out).
uint32_t writeCapped(const uint32_t* table, uint32_t total, uint32_t* out, uint32_t count) {
  const uint32_t n = count < total ? count : total;
  for (uint32_t i = 0; i < n; ++i) out[i] = table[i];
  return total;
}

}  // namespace

extern "C" {

uint32_t configParityStructSizes(uint32_t* out, uint32_t count) {
  static const uint32_t kSizes[] = {
      sizeof(msg::Geometry),       sizeof(msg::Motors),  sizeof(msg::Drive),
      sizeof(msg::WheelControl),   sizeof(msg::Planner), sizeof(msg::PlannerShaper),
      sizeof(msg::Otos),           sizeof(msg::Estimator),
  };
  static_assert(sizeof(kSizes) / sizeof(kSizes[0]) ==
                    static_cast<uint32_t>(ConfigParityGroup::Count),
                "kSizes must carry exactly one entry per ConfigParityGroup, "
                "in ConfigParityGroup order");
  return writeCapped(kSizes, static_cast<uint32_t>(ConfigParityGroup::Count), out, count);
}

uint32_t configParityFieldOffsets(uint32_t group, uint32_t* out, uint32_t count) {
  switch (static_cast<ConfigParityGroup>(group)) {
    case ConfigParityGroup::Geometry: {
      using T = msg::Geometry;
      static const uint32_t kOffsets[] = {
          offsetof(T, trackwidth),        offsetof(T, rotational_slip),
          offsetof(T, rotation_gain_pos), offsetof(T, rotation_offset),
          offsetof(T, rotation_gain_neg), offsetof(T, rotation_offset_neg),
      };
      return writeCapped(kOffsets, sizeof(kOffsets) / sizeof(kOffsets[0]), out, count);
    }
    case ConfigParityGroup::Motors: {
      using T = msg::Motors;
      static const uint32_t kOffsets[] = {
          offsetof(T, travel_calib_left),  offsetof(T, travel_calib_right),
          offsetof(T, fwd_sign_left),      offsetof(T, fwd_sign_right),
          offsetof(T, output_deadband),    offsetof(T, reversal_dwell),
          offsetof(T, vel_kp),             offsetof(T, vel_ki),
          offsetof(T, vel_kff),            offsetof(T, vel_i_max),
          offsetof(T, vel_kaw),            offsetof(T, vel_filt_alpha),
      };
      return writeCapped(kOffsets, sizeof(kOffsets) / sizeof(kOffsets[0]), out, count);
    }
    case ConfigParityGroup::Drive: {
      using T = msg::Drive;
      static const uint32_t kOffsets[] = {
          offsetof(T, duty_per_speed_left),         offsetof(T, duty_per_speed_right),
          offsetof(T, crawl_pulse),                 offsetof(T, wheel_gain_left_accel),
          offsetof(T, wheel_intercept_left_accel),  offsetof(T, wheel_gain_left_decel),
          offsetof(T, wheel_intercept_left_decel),  offsetof(T, wheel_gain_right_accel),
          offsetof(T, wheel_intercept_right_accel), offsetof(T, wheel_gain_right_decel),
          offsetof(T, wheel_intercept_right_decel),
      };
      return writeCapped(kOffsets, sizeof(kOffsets) / sizeof(kOffsets[0]), out, count);
    }
    case ConfigParityGroup::WheelControl: {
      using T = msg::WheelControl;
      static const uint32_t kOffsets[] = {
          offsetof(T, v_min),             offsetof(T, bias_max),
          offsetof(T, tau_adapt),         offsetof(T, a_steady),
          offsetof(T, deficit_threshold), offsetof(T, deficit_window),
          offsetof(T, pid_kp),            offsetof(T, pid_ki),
          offsetof(T, pid_i_max),         offsetof(T, pid_kaff),
          offsetof(T, pid_max),            offsetof(T, pos_err_max),  // 133-002
      };
      return writeCapped(kOffsets, sizeof(kOffsets) / sizeof(kOffsets[0]), out, count);
    }
    case ConfigParityGroup::Planner: {
      // a_max/a_decel/alpha_max/alpha_decel/jerk_max/yaw_jerk_max --
      // MOVED, 132-017: now ConfigParityGroup::PlannerShaper's own fields
      // (below), split out because they carry their own re-appliable
      // setter (Motion::Planner::applyShaperLimits()) unlike the rest of
      // this group. shaper_a_max/shaper_a_decel/shaper_alpha_max/
      // shaper_alpha_decel/shaper_j_max/shaper_yaw_jerk_max -- DELETED,
      // 132-015 (dead-code sweep; robot_config.proto's Planner message
      // now `reserved`s those field numbers instead of declaring them,
      // see that message's own trailing comment).
      using T = msg::Planner;
      static const uint32_t kOffsets[] = {
          offsetof(T, v_max),                 offsetof(T, omega_max),
          offsetof(T, control_period),        offsetof(T, actuation_delay),
          offsetof(T, settle_rest_velocity),  offsetof(T, settle_rest_omega),
          offsetof(T, settle_epsilon_linear), offsetof(T, settle_epsilon_angular),
          offsetof(T, heading_hold_gain),     offsetof(T, decel_plan_fraction),
          // 134-003 terminal fine-align. align_max_nudges is the first
          // non-float field this group has ever carried (int32); the parity
          // check compares OFFSETS, not types, so the int32 sits in the
          // table exactly like its float siblings.
          offsetof(T, align_tol),             offsetof(T, align_max_nudges),
      };
      return writeCapped(kOffsets, sizeof(kOffsets) / sizeof(kOffsets[0]), out, count);
    }
    case ConfigParityGroup::PlannerShaper: {
      using T = msg::PlannerShaper;
      static const uint32_t kOffsets[] = {
          offsetof(T, a_max),   offsetof(T, a_decel),
          offsetof(T, alpha_max), offsetof(T, alpha_decel),
          offsetof(T, jerk_max),  offsetof(T, yaw_jerk_max),
      };
      return writeCapped(kOffsets, sizeof(kOffsets) / sizeof(kOffsets[0]), out, count);
    }
    case ConfigParityGroup::Otos: {
      using T = msg::Otos;
      static const uint32_t kOffsets[] = {
          offsetof(T, offset_x),   offsetof(T, offset_y),
          offsetof(T, offset_yaw), offsetof(T, linear_scale),
          offsetof(T, angular_scale),
      };
      return writeCapped(kOffsets, sizeof(kOffsets) / sizeof(kOffsets[0]), out, count);
    }
    case ConfigParityGroup::Estimator: {
      using T = msg::Estimator;
      static const uint32_t kOffsets[] = {
          offsetof(T, weight_heading_otos), offsetof(T, weight_omega_otos),
          offsetof(T, staleness),
      };
      return writeCapped(kOffsets, sizeof(kOffsets) / sizeof(kOffsets[0]), out, count);
    }
    case ConfigParityGroup::Count:
      break;  // not a real group -- falls through to the 0 return below
  }
  return 0;
}

}  // extern "C"
