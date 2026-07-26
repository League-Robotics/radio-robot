// robot_state.h -- MIRROR of the sprint-124 RobotState blackboard sketch
// (clasi/sprints/124-.../issues/robot-state-blackboard-one-struct-for-all-
// shared-state-and-telemetry.md). The real header lands in
// src/firm/types/robot_state.h during sprint 124; at the joint checkpoint
// this file is DELETED and the build points at the real one (the swap the
// motion-planner sketch §2 prescribes). Until then this is the planner's
// only view of the robot: dependency-free, trivially copyable, no
// pointers, no heap -- directly mirrorable as a Python ctypes.Structure.
//
// Field lists follow the issue sketch; the estimate section is filled out
// with the ZOH bases the sketch calls for (value + velocity + basisTime,
// so "predict to t" is a pure function over a copied state).
#pragma once

#include <cstdint>

struct RobotState {
  struct Time {
    uint32_t cycleStart = 0;   // [ms] writer: loop
    uint32_t cycleBusy = 0;    // [ms]
    uint32_t cyclePeriod = 0;  // [ms]
  } time;

  struct Wheel {
    float position = 0.0f;     // [mm] sensed, cumulative; writer: Drive::update
    float velocity = 0.0f;     // [mm/s] signed, sensed (raw -- noisy)
    uint32_t sampleTime = 0;   // [ms] when the sensed pair was collected
    bool connected = false;
    float cmdVelocity = 0.0f;  // [mm/s] signed; writer: the planner (via update())
  } wheelLeft, wheelRight;

  struct Otos {
    bool present = false;      // fresh this cycle
    bool connected = false;
    float x = 0.0f;            // [mm]
    float y = 0.0f;            // [mm]
    float heading = 0.0f;      // [rad]
    float vx = 0.0f;           // [mm/s]
    float vy = 0.0f;           // [mm/s]
    float omega = 0.0f;        // [rad/s]
    uint32_t time = 0;         // [ms]
  } otos;

  struct Perception {
    uint32_t line = 0;
    uint32_t color = 0;
    bool lineFresh = false;
    bool colorFresh = false;
  } perception;

  struct Pose {
    float x = 0.0f;        // [mm]
    float y = 0.0f;        // [mm]
    float heading = 0.0f;  // [rad]
    float vx = 0.0f;       // [mm/s] body-frame
    float vy = 0.0f;       // [mm/s] body-frame
    float omega = 0.0f;    // [rad/s]
  } pose;

  // ZOH bases -- value + velocity + basisTime per peer, so any consumer
  // holding a (copied) state can extrapolate to an arbitrary instant.
  struct WheelBasis {
    float distance = 0.0f;   // [mm] cumulative wheel travel at basisTime
    float velocity = 0.0f;   // [mm/s] signed, filtered
    uint32_t basisTime = 0;  // [ms]
    bool valid = false;
  };
  struct BodyBasis {
    float x = 0.0f;          // [mm]
    float y = 0.0f;          // [mm]
    float heading = 0.0f;    // [rad]
    float v_x = 0.0f;        // [mm/s] body-frame
    float v_y = 0.0f;        // [mm/s] body-frame
    float omega = 0.0f;      // [rad/s]
    uint32_t basisTime = 0;  // [ms]
    bool valid = false;
  };
  struct Estimate {
    BodyBasis body;
    WheelBasis wheelLeft, wheelRight;
    float innovationHeading = 0.0f;  // [rad] OTOS minus predicted, at blend time
    float innovationOmega = 0.0f;    // [rad/s]
    bool innovationsValid = false;
  } estimate;

  struct Command {
    bool moveActive = false;
    uint32_t activeMoveId = 0;
  } command;

  struct Health {
    uint32_t commsMalformedCount = 0;
    bool deadmanExpired = false;
  } health;
};
