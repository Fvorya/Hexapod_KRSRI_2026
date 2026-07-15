#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "config.h"
#include <math.h>

class InverseKinematics {
public:
  static bool solve(float x, float y, float z, float& coxaDeg, float& femurDeg, float& tibiaDeg);
};

#endif
