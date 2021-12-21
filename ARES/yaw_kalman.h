#ifndef YAW_KALMAN_H
#define YAW_KALMAN_H
#incldue "main.h"

typedef struct YawKalman
{
  float theta;
  float omega;
  float pTheta;
  float pOmega;
} YawKalman;

extern void YawKalman_init(YawKalman *kalman,float thetaGuess);
extern void YawKalman_predict(YawKalman *kalman, float omega, float qTheta,float qOmega,float dt);
extern void YawKalman_update(YawKalman *kalman, float theta, float rTheta);
extern float YawKalman_getAngle(YawKalman *kalman);

#endif
