// otos_sample.cpp -- App::applyOtosSample() implementation. See
// otos_sample.h's file header for the module's boundary and rationale
// (split out of odometry.cpp, 122-002).
#include "app/otos_sample.h"

namespace App {

void applyOtosSample(Devices::Otos& otos, uint64_t now, Telemetry::Frame& frame) {
  otos.tick(now);
  frame.otosConnected = otos.connected();
  frame.otosPresent = otos.present() && otos.poseFresh();
  if (frame.otosPresent) {
    Devices::PoseReading reading = otos.pose();
    frame.otos.x = reading.x;
    frame.otos.y = reading.y;
    frame.otos.heading = reading.heading;
    frame.otos.v_x = reading.v_x;
    frame.otos.v_y = reading.v_y;
    frame.otos.omega = reading.omega;
    frame.otos.time = static_cast<uint32_t>(now / 1000);  // [us] -> [ms]
  }
}

}  // namespace App
