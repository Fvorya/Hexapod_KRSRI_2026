#include "ArmInverse.h"

bool ArmInverse::solve(float x, float y, float& shoulderDeg, float& elbowDeg) {
  bool inRange = true;
  
  float x2 = pow(x, 2);
  float y2 = pow(y, 2);
  float upperArm2 = pow(UPPERARM_LENGTH, 2);
  float foreArm2 = pow(FOREARM_LENGTH, 2);

  float cosElbow = clampf((x2 + y2 - upperArm2 - foreArm2) / (2 * UPPERARM_LENGTH * FOREARM_LENGTH), -1.0f, 1.0f);
  float elbowRad = atan2f(sqrtf(1 - pow(cosElbow, 2)), cosElbow);
  elbowDeg = rad2deg(elbowRad);

  float upper1 = atan2f(y, x);
  float upper2 = atan2f(FOREARM_LENGTH * sinf(elbowRad), UPPERARM_LENGTH + FOREARM_LENGTH * cosf(elbowRad));
  float shoulderRad = upper1 - upper2;
  shoulderDeg = rad2deg(shoulderRad);

  if (sqrtf(x2 + y2) > (UPPERARM_LENGTH + FOREARM_LENGTH)) {
      inRange = false; 
  }
  
  return inRange;
}
