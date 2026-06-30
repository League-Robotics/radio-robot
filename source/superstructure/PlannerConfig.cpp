// PlannerConfig.cpp — toPlannerConfig projection (ticket 059-001).
//
// Projects a RobotConfig into a msg::PlannerConfig.
// Motion-limits scope only. Drive geometry and PID gains live in DriveConfig.cpp.

#include "superstructure/PlannerConfig.h"
#include "types/Config.h"
#include "messages/planner.h"

msg::PlannerConfig toPlannerConfig(const RobotConfig& rc)
{
    msg::PlannerConfig cfg;

    // Linear acceleration limits (reused from RobotConfig.aMax / aDecel).
    cfg.setAMax(rc.aMax);
    cfg.setADecel(rc.aDecel);

    // Body speed ceiling.
    cfg.setVBodyMax(rc.vBodyMax);

    // Yaw rate and acceleration limits.
    // RobotConfig.yawRateMax is in deg/s; msg::PlannerConfig.yaw_rate_max
    // carries the same unit (consumers convert as needed).
    cfg.setYawRateMax(rc.yawRateMax);
    cfg.setYawAccMax(rc.yawAccMax);

    // Jerk limits (0 = trapezoid profile, no S-curve).
    cfg.setJMax(rc.jMax);
    cfg.setYawJerkMax(rc.yawJerkMax);

    // Go-to arrival tolerance (mm).
    cfg.setArriveTolMm(rc.arriveTolMm);

    // Turn-in-place gate: bearing threshold in degrees below which the robot
    // goes straight to the goal rather than rotating first.
    cfg.setTurnInPlaceGate(rc.turnInPlaceGate);

    // Legacy go-to tolerances (retained for backward compat / future use).
    cfg.setTurnThresholdMm(rc.turnThresholdMm);
    cfg.setDoneTolMm(rc.doneTolMm);

    // Minimum speed floor (mm/s).  RobotConfig stores as int32_t.
    cfg.setMinSpeedMms((float)rc.minSpeedMms);

    return cfg;
}
