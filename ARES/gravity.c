#include "main.h"
#include "math.h"

#include "tof.h"

const float x_zero = 5;
const float y_zero = 5;
const float x_factor = 0.1;
const float y_factor = 0.05;

struct Potential {
  float x;
  float left;
  float right;
  float wplus;
  float wminus;
} potential;

void calcPotential(void) {
  potential.x = tof[TOF_FRONT];
  potential.left = 0.5 * (tof[TOF_LEFT_FRONT] + tof[TOF_LEFT_BACK]);
  potential.right = 0.5 * (tof[TOF_RIGHT_FRONT] + tof[TOF_RIGHT_BACK]);
  potential.wplus = (tof[TOF_LEFT_FRONT] + tof[TOF_RIGHT_BACK]);
  potential.wminus = (tof[TOF_LEFT_BACK] + tof[TOF_RIGHT_FRONT]);
}

#include "chassis.h"

void gravityAction() {
  float x, y, w;
  x = x_factor * fabs(potential.x - x) * (potential.x - x_zero);
  float left =
      y_factor * fabs(potential.left - y_zero) * (potential.left - y_zero);
  float right =
      y_factor * fabs(potential.right - y_zero) * (potential.right - y_zero);
  y = left - right;
  if (potential.wplus > potential.wminus) {
    w = potential.wplus;
  } else {
    w = -potential.wminus;
  }
  Chassis_setSpeed(x, y, w,2);
}
