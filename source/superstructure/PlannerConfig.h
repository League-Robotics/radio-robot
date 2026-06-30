#pragma once
// =============================================================================
// PlannerConfig.h — toPlannerConfig projection declaration (ticket 059-001).
//
// Projects a RobotConfig into a msg::PlannerConfig for use by
// MotionController2::configure() and the Phase 3 command-bus initialization.
//
// Motion-limits scope only: aMax, aDecel, vBodyMax, yawRateMax, yawAccMax,
// jMax, yawJerkMax, arriveTolMm, turnInPlaceGate, turnThresholdMm, doneTolMm,
// minSpeedMms.
//
// Drive geometry and PID gains are NOT mapped here — those belong to
// toDriveConfig() in DriveConfig.cpp.
// =============================================================================

#include "types/Config.h"      // RobotConfig
#include "messages/planner.h"  // msg::PlannerConfig

// Declared in global namespace to match the DriveConfig.cpp convention and
// keep projection functions as free functions.
msg::PlannerConfig toPlannerConfig(const RobotConfig& rc);
