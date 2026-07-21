#ifndef LEGINVERSEKINEMATICS_H
#define LEGINVERSEKINEMATICS_H

#include "config.h"
#include <math.h>
#include "types.h"

class LegInverseKinematics {
public:
  static bool solve(float x, float y, float z, float& coxaDeg, float& femurDeg, float& tibiaDeg);
};

#endif
