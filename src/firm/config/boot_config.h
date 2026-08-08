#pragma once

#include <stdint.h>

#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "messages/robot_config.h"

namespace Config {

constexpr uint32_t kMotorConfigCount = 4;

void defaultMotorConfigs(msg::MotorConfig* out);

msg::DrivetrainConfig defaultDrivetrainConfig();

extern const char kRobotProfileName[];

extern const char kDrivetrainType[];

// Radio channel (nRF frequency band, 0..83) main.cpp passes to
// Radio::begin(). Baked from the robot JSON's connection.radio_channel
// (default 0 -- the historical hard-coded value); BAKE-ONLY by design, no
// wire target: re-tuning a live link drops it, and the field exists so two
// robots sharing a bench never share a channel unintentionally (see
// robot_config.proto's Connection.radio_channel doc comment).
extern const int kRadioChannel;  // [nRF frequency band]

struct OtosBootConfig {
  float offsetX = 0.0f;  // [mm] mounting offset from chassis centre to sensor
  float offsetY = 0.0f;  // [mm]
  float offsetYaw = 0.0f;  // [rad] mounting yaw offset
  float linearScale = 1.0f;
  float angularScale = 1.0f;
};

OtosBootConfig defaultOtosBootConfig();

struct EstimatorBootConfig {
  float headingOtos = 0.0f;  // [0..1] blend weight: body heading vs OTOS heading
  float omegaOtos = 0.0f;  // [0..1] blend weight: body omega vs OTOS omega
  uint32_t staleness = 200;  // [ms] max OTOS reading age still eligible to blend
};

EstimatorBootConfig defaultEstimatorConfig();

struct ShaperBootConfig {
  float aMax = 0.0f;  // [mm/s^2] linear accel-ramp ceiling
  float aDecel = 0.0f;  // [mm/s^2] linear decel-taper ceiling
  float alphaMax = 0.0f;  // [rad/s^2] angular accel-ramp ceiling
  float alphaDecel = 0.0f;  // [rad/s^2] angular decel-taper ceiling
  float jMax = 0.0f;  // [mm/s^3] linear jerk ceiling
  float yawJerkMax = 0.0f;  // [rad/s^3] angular jerk ceiling
};

ShaperBootConfig defaultShaperConfig();

struct DriveBootConfig {
  float dutyPerSpeedLeft = 0.0f;  // [duty/(mm/s)] 1 / measured plant gain, left
  float dutyPerSpeedRight = 0.0f;  // [duty/(mm/s)] 1 / measured plant gain, right
  float crawlPulse = 0.0f;  // [-1, 1] sub-breakaway pulse amplitude; 0 = off

  float gainLeftAccel = 1.0f;
  float interceptLeftAccel = 0.0f;  // [mm/s] its intercept
  float gainLeftDecel = 1.0f;
  float interceptLeftDecel = 0.0f;  // [mm/s] its intercept
  float gainRightAccel = 1.0f;
  float interceptRightAccel = 0.0f;  // [mm/s] its intercept
  float gainRightDecel = 1.0f;
  float interceptRightDecel = 0.0f;  // [mm/s] its intercept
};

DriveBootConfig defaultDriveConfig();

struct WheelControllerBootConfig {
  float vMin = 0.0f;  // [mm/s] speed floor
  float biasMax = 0.0f;  // [mm/s] Stage C trim authority clamp
  float tauAdapt = 0.0f;  // [s] Stage C adaptation time constant; <=0 disables
  float aSteady = 0.0f;  // [mm/s^2] steady-state gate
  float deficitThreshold = 0.0f;  // [mm/s] deficit-flag error threshold
  float deficitWindow = 0.0f;  // [ms] deficit-flag sustain window

  float kp = 0.0f;  // [1] dimensionless proportional gain
  float ki = 0.0f;  // [1/s] integral gain
  float iMax = 0.0f;  // [mm/s] integrator clamp
  float kaff = 0.0f;  // [s] accel feedforward
  float pidMax = 0.0f;  // [mm/s] total fast-loop authority clamp

  // Stall detection -- see robot_config.proto's WheelControl for why this is
  // a different fault from deficit* and from wheelFrozen, and for why the
  // encoder-health precondition is load-bearing.
  float stallSpeed = 0.0f;  // [mm/s] measured speed at or below this is not turning
  float stallDemand = 0.0f;  // [mm/s] commanded speed above this is genuinely asking for motion
  float stallWindow = 0.0f;  // [ms] sustain time before the stall latches; 0 = detector off
};

WheelControllerBootConfig defaultWheelControllerConfig();

struct PlannerBootConfig {
  float vMax = 0.0f;  // [mm/s] linear velocity ceiling
  float aMax = 0.0f;  // [mm/s^2] linear accel-ramp ceiling
  float aDecel = 0.0f;  // [mm/s^2] linear decel-taper ceiling
  float omegaMax = 0.0f;  // [rad/s] angular velocity ceiling
  float alphaMax = 0.0f;  // [rad/s^2] angular accel-ramp ceiling
  float alphaDecel = 0.0f;  // [rad/s^2] angular decel-taper ceiling
  float jerkMax = 0.0f;  // [mm/s^3] linear jerk ceiling
  float yawJerkMax = 0.0f;  // [rad/s^3] angular jerk ceiling

  float controlPeriod = 0.0f;  // [ms]
  float actuationDelay = 0.0f;  // [ms] command-staged-to-wheels latency

  float settleRestVelocity = 0.0f;  // [mm/s]
  float settleRestOmega = 0.0f;  // [rad/s]
  float settleEpsilonLinear = 0.0f;  // [mm]
  float settleEpsilonAngular = 0.0f;  // [rad]
  float headingHoldGain = 0.0f;  // [1/s] rad/s of correction per rad of error

  float decelPlanFraction = 0.0f;  // [1] fraction of decel ceiling to plan brake-start against

  float alignTol = 0.0f;         // [rad] fine-align residual tolerance
  int32_t alignMaxNudges = 0;    // corrective pivots one Move may spend
};

// Commanded->actual values App::Drive::setWheelCorrection() installs.
struct WheelCorrection {
  float gainLeftAccel = 1.0f;
  float interceptLeftAccel = 0.0f;   // [mm/s]
  float gainLeftDecel = 1.0f;
  float interceptLeftDecel = 0.0f;   // [mm/s]
  float gainRightAccel = 1.0f;
  float interceptRightAccel = 0.0f;  // [mm/s]
  float gainRightDecel = 1.0f;
  float interceptRightDecel = 0.0f;  // [mm/s]
};

PlannerBootConfig defaultPlannerLimits();

msg::Geometry defaultGeometryGroup();
msg::Motors defaultMotorsGroup();
msg::Drive defaultDriveGroup();
msg::WheelControl defaultWheelControlGroup();
msg::Planner defaultPlannerGroup();
msg::PlannerShaper defaultPlannerShaperGroup();
msg::Otos defaultOtosGroup();
msg::Estimator defaultEstimatorGroup();
msg::Navigator defaultNavigatorGroup();

}
