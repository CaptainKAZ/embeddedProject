#include "yaw_kalman.h"

void YawKalman_init(YawKalman *kalman, float thetaGuess) {
  kalman->pTheta = 1;
  kalman->pOmega = 1;
  kalman->theta = thetaGuess;
  kalman->omega = 0;
}

void YawKalman_predict(YawKalman *kalman, float omega, float qTheta,
                       float qOmega, float dt) {
  // predict state ahead
  kalman->theta = kalman->theta + 0.5 * (omega + kalman->omega) * dt;
  kalman->omega = omega;
  // predict error covariance
  kalman->pTheta = kalman->pOmega * dt * dt / 4 + kalman->pTheta + qTheta;
  kalman->pOmega = qOmega;
}

void YawKalman_update(YawKalman *kalman, float theta, float rTheta) {
  // compute kalman gain
  float k = kalman->pTheta / (kalman->pTheta + rTheta);
  // update estimate with measurement
  kalman->theta = kalman->theta + k * (theta - kalman->theta);
  // update error covariance
  kalman->pTheta = (1 - k) * kalman->pTheta;
}

float YawKalman_getAngle(YawKalman *kalman) { return kalman->theta; }
