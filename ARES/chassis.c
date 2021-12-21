#include "chassis.h"
#include <math.h>
#include <stdio.h>
#include "cmsis_os.h"

Chassis chassis = {.r = 1};

char chassisCommand[] = "#000P1500T1000!";

void Chassis_setSpeed(float xSpeed, float ySpeed,
                      float wSpeed, int32_t timeout) {
  chassis.xSpeed = xSpeed;
  chassis.ySpeed = ySpeed;
  chassis.wSpeed = wSpeed;
  chassis.timeout = timeout;
}

void chassis_task(void) {
  osDelay(1000);
  for (;;) {
    chassis.wheelSpeed[FRONT_LEFT] =
        chassis.xSpeed - chassis.ySpeed - chassis.r * chassis.wSpeed;
    chassis.wheelSpeed[FRONT_RIGHT] =
        chassis.xSpeed + chassis.ySpeed + chassis.r * chassis.wSpeed;
    chassis.wheelSpeed[BACK_LEFT] =
        chassis.xSpeed + chassis.ySpeed - chassis.r * chassis.wSpeed;
    chassis.wheelSpeed[BACK_RIGHT] =
        chassis.xSpeed - chassis.ySpeed + chassis.r * chassis.wSpeed;
    float maxAbs = 0;
    for (int i = 0; i < 4; i++) {
      if (fabsf(chassis.wheelSpeed[i]) > maxAbs) {
        maxAbs = fabsf(chassis.wheelSpeed[i]);
      }
    }
    for (int i = 0; i < 4; i++) {
      chassis.wheelSpeed[i] = chassis.wheelSpeed[i] / maxAbs;
    }
    if (chassis.timeout > 0) {
      chassis.timeout--;
      for (int i = 0; i < 4; i++) {
        int pwm = 1500;
        pwm+=chassis.wheelSpeed[i]*1000;
        sprintf(chassisCommand, "#%03dP%4dT%4d!", i, pwm, 0);
        HAL_UART_Transmit(&huart2, (uint8_t *)chassisCommand,
                          sizeof(chassisCommand), 1);
      }
    }
    osDelay(10);
  }
}
