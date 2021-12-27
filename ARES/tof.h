#ifndef TOF_H
#define TOF_H

#include "main.h"

enum tofOrientation {
  TOF_FRONT = 1,
  TOF_LEFT_FRONT = 0,
  TOF_LEFT_BACK = 4,
  TOF_RIGHT_BACK = 3,
  TOF_RIGHT_FRONT = 2,
  TOF_BACK = 5,
};

extern int tof[6];

#define TOF_DISTANCE_MM 230

#endif
