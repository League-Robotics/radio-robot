// kinematics.cpp -- Kinematics::Model's one concrete method.
#include "kinematics/kinematics.h"

#include <math.h>

namespace Kinematics {

void Model::saturate(const float wheels[], float wheelSpeedMax,
                     float steerHeadroom, float out[]) const {
  const int n = wheelCount();
  const float ceiling = wheelSpeedMax - steerHeadroom;

  float maxAbs = 0.0f;
  for (int i = 0; i < n; ++i) {
    const float a = fabsf(wheels[i]);
    if (a > maxAbs) maxAbs = a;
  }

  // Pass-through below the ceiling. Note `out` may alias `wheels`, so the
  // copy is still required rather than skippable.
  if (maxAbs > ceiling) {
    const float s = ceiling / maxAbs;
    for (int i = 0; i < n; ++i) out[i] = s * wheels[i];
  } else {
    for (int i = 0; i < n; ++i) out[i] = wheels[i];
  }
}

}  // namespace Kinematics
