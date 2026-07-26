// duty_plant.h -- TestPlanner::DutyPlant: a realistic duty->wheel plant,
// constants MEASURED on the real robot (plant_id.py, 2026-07-26, wheels
// free on the stand): steady-state gain ~1370 mm/s per unit duty with ~7%
// L/R asymmetry (L 1420 / R 1320), first-order time constant ~230 ms, and
// the brick's own per-write duty slew (25%/write). Encoder positions
// quantize at ~0.07 mm (a tenth of a motor-shaft degree through the wheel
// travel calibration); reported velocity carries deterministic zig-zag
// noise. The plant is the HONEST test bed for the co-located one-loop
// duty topology: sense (quantized, one interval old) -> plan+PID -> duty
// applied over the CURRENT interval.
#pragma once

#include <cmath>
#include <cstdint>

#include "types/robot_state.h"

namespace TestPlanner {

struct DutyWheel {
  float gain;              // [mm/s per duty] steady-state
  float applied = 0.0f;    // [duty] after the brick's slew
  float velocity = 0.0f;   // [mm/s] true plant velocity
  double position = 0.0;   // [mm] true accumulated travel

  void step(float dutyCommand, float dt, float tau, float slewPerStep) {
    // Brick-side duty slew: the applied duty steps toward the command by
    // at most slewPerStep per cycle (nezha writeShapedDuty's cap).
    const float delta = dutyCommand - applied;
    const float slewed = (delta > slewPerStep)    ? applied + slewPerStep
                         : (delta < -slewPerStep) ? applied - slewPerStep
                                                  : dutyCommand;
    applied = slewed;
    // First-order response toward the applied duty's steady-state speed.
    const float target = applied * gain;
    velocity += (target - velocity) * (dt / tau);
    position += static_cast<double>(velocity) * dt;
  }
};

class DutyPlant {
 public:
  static constexpr float kGainLeft = 1420.0f;   // [mm/s per duty] measured
  static constexpr float kGainRight = 1320.0f;  // [mm/s per duty] measured
  static constexpr float kTau = 0.23f;          // [s] measured
  static constexpr float kSlewPerStep = 0.25f;  // [duty/cycle] brick cap
  static constexpr float kQuantum = 0.0716f;    // [mm] tenth-deg via calib
  static constexpr float kVelocityNoise = 15.0f;  // [mm/s] zig-zag amplitude

  DutyWheel left{kGainLeft};
  DutyWheel right{kGainRight};
  bool noisy = true;

  // Advance one interval under the staged duty pair, then publish the
  // samples the NEXT tick will see (quantized position, noisy velocity,
  // fresh sampleTime -- fresh every cycle, per the measured encoder
  // characterization).
  void step(Types::RobotState& state, float dutyLeft, float dutyRight,
            float dt, uint32_t sampleTime) {  // [s] [ms]
    left.step(dutyLeft, dt, kTau, kSlewPerStep);
    right.step(dutyRight, dt, kTau, kSlewPerStep);
    ++stepCount_;
    publish(state.wheelLeft, left, sampleTime);
    publish(state.wheelRight, right, sampleTime);
  }

  float truePath() const {  // [mm] signed mean travel
    return 0.5f * static_cast<float>(left.position + right.position);
  }
  float trueHeading(float trackWidth) const {  // [rad]
    return static_cast<float>(right.position - left.position) / trackWidth;
  }

 private:
  void publish(Types::RobotState::Wheel& wheel, const DutyWheel& plant,
               uint32_t sampleTime) {
    const float quantized =
        std::round(static_cast<float>(plant.position) / kQuantum) * kQuantum;
    wheel.position = quantized;
    const float noise =
        noisy ? ((stepCount_ % 2 == 0) ? kVelocityNoise : -kVelocityNoise)
              : 0.0f;
    wheel.velocity = plant.velocity + noise;
    wheel.sampleTime = sampleTime;
    wheel.connected = true;
  }

  int stepCount_ = 0;
};

}  // namespace TestPlanner
