// pilot.cpp -- App::Pilot implementation. See pilot.h's file header for
// the module's boundary and cycle-placement contract.
#include "app/pilot.h"

namespace App {

void Pilot::tick(uint32_t now) {  // [ms]
  uint32_t dt = hasLastTick_ ? (now - lastTick_) : 0;
  lastTick_ = now;
  hasLastTick_ = true;

  Motion::Executor::Twist twist = executor_.tick(dt);
  if (executor_.state() != Motion::State::kIdle) {
    drive_.setTwist(twist.v, twist.omega);
  }
}

}  // namespace App
