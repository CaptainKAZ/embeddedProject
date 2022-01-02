#include "PID.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "main.h"
#include "math.h"
#include "tof.h"
#include "yaw_kalman.h"
extern YawKalman kalman;

int8_t stage = -1;

PID_Controller yawPID;
PID_Controller omegaPID;
PID_Controller xPID;
PID_Controller yPID;

PID_Param yawPIDParam = {
    .Int_type = NORMAL_INT, .kB = 0, .kD = 0, .kI = 0, .kP = 10, .N = 0};
PID_Param omegaPIDParam = {
    .Int_type = NORMAL_INT, .kB = 0, .kD = 0, .kI = 0, .kP = 0.01, .N = 0};
PID_Param xPIDParam = {
    .Int_type = NORMAL_INT, .kB = 0, .kD = 0, .kI = 0, .kP = 0.002, .N = 0};
PID_Param yPIDParam = {
    .Int_type = NORMAL_INT, .kB = 0, .kD = 0.0011, .kI = 0.0, .kP = 0.0035, .N = 0};

PID_Constraint yawConstraint = {
    .inMin = 0, .inMax = 0, .outMin = -720, .outMax = 720};
PID_Constraint omegaConstraint = {
    .inMin = -0, .inMax = 0, .outMin = -0.8, .outMax = 0.8};
PID_Constraint xConstraint = {
    .inMin = -0, .inMax = 0, .outMin = -1.0, .outMax = 1.0};
PID_Constraint yConstraint = {
    .inMin = -0, .inMax = 0, .outMin = -0.8, .outMax = 0.8};

int initted_flag = 0;
int control_task_init() {
  if (kalman.initted == 0) {
    return 0;
  }
  if (initted_flag == 0) {
    PID_ControllerInit(&yawPID, PIDPOS_CONTROLLER, &yawConstraint, &yawPIDParam,
                       0.1f);
    PID_ControllerInit(&omegaPID, PIDPOS_CONTROLLER, &omegaConstraint,
                       &omegaPIDParam, 0.1f);
    PID_ControllerInit(&xPID, PIDPOS_CONTROLLER, &xConstraint, &xPIDParam,
                       0.1f);
    PID_ControllerInit(&yPID, PIDPOS_CONTROLLER, &yConstraint, &yPIDParam,
                       0.1f);
    initted_flag = 1;
  }

  return initted_flag;
}

float xTarget = 0, yTarget = 0, yawTarget;
float chassisX = 0, chassisY = 0, chassisW = 0;

void control_task_update(void) {
  if (stage == -1) {
    chassisX = 0;
    chassisY = 0;
    chassisW = 0;
    if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {
      stage = 0;
    }
  }
  if (stage == 0) {
    //第一个直
    yawTarget = 0;
    xTarget = 40;
    yTarget = 0;
    chassisX = -PID_ControllerUpdate(&xPID, xTarget, tof[TOF_FRONT]);
    chassisY = PID_ControllerUpdate(
        &yPID, yTarget, 130 - (tof[TOF_RIGHT_FRONT] + tof[TOF_RIGHT_BACK]) / 2);
    float set_omega = PID_ControllerUpdate(&yawPID, yawTarget, kalman.theta);
    chassisW = PID_ControllerUpdate(
        &omegaPID, 0,
        180 / 3.141592653589793238462643383279 *
            atan2(tof[TOF_RIGHT_FRONT] - tof[TOF_RIGHT_BACK], 230));
    if (tof[TOF_FRONT] < 600 &&
        fabsf(180 / 3.141592653589793238462643383279 *
              atan2(tof[TOF_RIGHT_FRONT] - tof[TOF_RIGHT_BACK], 230)) < 10) {
      yawTarget = kalman.theta + 90;
      stage = 1;
    }
  } else if (stage == 1) {
    //第一个左转
    // yawTarget = 90;
    xTarget = 0;
    yTarget = 0;
    chassisX = 0.5;
    chassisY = PID_ControllerUpdate(
        &yPID, yTarget, 116 - (tof[TOF_RIGHT_FRONT] + tof[TOF_RIGHT_BACK]) / 2);
    float set_omega = PID_ControllerUpdate(&yawPID, yawTarget, kalman.theta);
    chassisW = PID_ControllerUpdate(&omegaPID, set_omega, kalman.omega);
    if (fabsf(kalman.theta - yawTarget) < 10) {
      stage = 2;
    }
  } else if (stage == 2) {
    //左移
    // yawTarget = 0;
    xTarget = 40;
    yTarget = 0;
    chassisX = -PID_ControllerUpdate(&xPID, xTarget, tof[TOF_FRONT]);
    chassisY = PID_ControllerUpdate(
        &yPID, yTarget, 116 - (tof[TOF_RIGHT_FRONT] + tof[TOF_RIGHT_BACK]) / 2);
    float set_omega = PID_ControllerUpdate(&yawPID, yawTarget, kalman.theta);
    chassisW = PID_ControllerUpdate(
        &omegaPID, 0,
        180 / 3.141592653589793238462643383279 *
            atan2(tof[TOF_RIGHT_FRONT] - tof[TOF_RIGHT_BACK], 230));
    if (tof[TOF_FRONT] < 590 &&
        fabsf(180 / 3.141592653589793238462643383279 *
              atan2(tof[TOF_RIGHT_FRONT] - tof[TOF_RIGHT_BACK], 230)) < 10) {
      yawTarget = kalman.theta + 90;
      stage = 3;
    }
  } else if (stage == 3) {
    //第二个左转
    // yawTarget = 180;
    xTarget = 0;
    yTarget = 0;
    chassisX = 0.4;
    chassisY = PID_ControllerUpdate(
        &yPID, yTarget, 116 - (tof[TOF_RIGHT_FRONT] + tof[TOF_RIGHT_BACK]) / 2);
    float set_omega = PID_ControllerUpdate(&yawPID, yawTarget, kalman.theta);
    chassisW = PID_ControllerUpdate(&omegaPID, set_omega, kalman.omega);
    if (fabsf(kalman.theta - yawTarget) < 5) {
      stage = 4;
    }
  } else if (stage == 4) {
    //往回走
    xTarget = 40;
    yTarget = 0;
    chassisX = -PID_ControllerUpdate(&xPID, xTarget, tof[TOF_FRONT]);
    chassisY = PID_ControllerUpdate(
        &yPID, yTarget, 116 - (tof[TOF_RIGHT_FRONT] + tof[TOF_RIGHT_BACK]) / 2);
    float set_omega = PID_ControllerUpdate(&yawPID, yawTarget, kalman.theta);
    chassisW = PID_ControllerUpdate(
        &omegaPID, 0,
        180 / 3.141592653589793238462643383279 *
            atan2(tof[TOF_RIGHT_FRONT] - tof[TOF_RIGHT_BACK], 230));
    if (tof[TOF_FRONT] < 480 &&
        fabsf(180 / 3.141592653589793238462643383279 *
              atan2(tof[TOF_LEFT_BACK] - tof[TOF_LEFT_FRONT], 230)) < 5) {
      yawTarget = kalman.theta - 90;
      stage = 5;
      //while (1) osDelay(1);
    }
  } else if (stage == 5) {
    //第一个右转
    xTarget = 0;
    yTarget = 0;
    chassisX = 0.6;
    chassisY = PID_ControllerUpdate(
        &yPID, yTarget, -116 + (tof[TOF_LEFT_FRONT] + tof[TOF_LEFT_BACK]) / 2);
    float set_omega = PID_ControllerUpdate(&yawPID, yawTarget, kalman.theta);
    chassisW = PID_ControllerUpdate(&omegaPID, set_omega, kalman.omega);
    if (fabsf(kalman.theta - yawTarget) < 5) {
      stage = 6;
    }
  } else if (stage == 6) {
    //第二个往右走
    yawTarget = 90;
    xTarget = 40;
    yTarget = 0;
    chassisX = -PID_ControllerUpdate(&xPID, xTarget, tof[TOF_FRONT]);
    chassisY = PID_ControllerUpdate(
        &yPID, yTarget, -116 + (tof[TOF_LEFT_FRONT] + tof[TOF_LEFT_BACK]) / 2);
    float set_omega = PID_ControllerUpdate(&yawPID, yawTarget, kalman.theta);
    chassisW = PID_ControllerUpdate(&omegaPID, set_omega, kalman.omega);
    if (tof[TOF_FRONT] < 520 && fabsf(kalman.theta - yawTarget) <5) {
      stage = 7;
      
    }
  } else if (stage == 7) {
    //第二个右拐
    yawTarget = 0;
    xTarget = 0;
    yTarget = 0;
    chassisX = 0.7;
    chassisY =- PID_ControllerUpdate(
        &yPID, yTarget, -116 + (tof[TOF_LEFT_FRONT] + tof[TOF_LEFT_BACK]) / 2);
    float set_omega = PID_ControllerUpdate(&yawPID, yawTarget, kalman.theta);
    chassisW = PID_ControllerUpdate(&omegaPID, set_omega, kalman.omega);
    if (fabsf(kalman.theta - yawTarget) < 5) {
      stage = 8;
    }
  } else if (stage == 8) {
    //找颜色
    yawTarget = 0;
    yawTarget = 0;
    xTarget = 0;
    chassisX = 0.4;
    chassisY = PID_ControllerUpdate(
        &yPID, yTarget, 110 - (tof[TOF_RIGHT_FRONT]));
    float set_omega = PID_ControllerUpdate(&yawPID, yawTarget, kalman.theta);
    chassisW = PID_ControllerUpdate(&omegaPID, set_omega, kalman.omega);
    if(HAL_GPIO_ReadPin(RGB_DETECT_GPIO_Port, RGB_DETECT_Pin) == GPIO_PIN_RESET){
      stage = 9;
      while(1){
        osDelay(1);
      }
    }
  }

  Chassis_setSpeed(chassisX, chassisY, chassisW, 10);
  // Chassis_setSpeed(0, 0, 0.5, 10);
}
