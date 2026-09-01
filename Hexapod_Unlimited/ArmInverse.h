#ifndef ARMINVERSE_H
#define ARMINVERSE_H

#include "config.h"
#include <math.h>
#include "types.h"

class ArmInverse {
public:
  static bool solve(float x, float y, float& shoulderDeg, float& elbowDeg);
};

#endif
