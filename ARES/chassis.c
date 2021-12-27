#include "chassis.h"

#include <math.h>
#include <stdio.h>

#include "cmsis_os.h"

extern int control_task_init(void);
extern void control_task_update(void);

Chassis chassis = {.r = 1};

char chassisCommand[] = "#000P1500T1000!\r\n";

void Chassis_setSpeed(float xSpeed, float ySpeed, float wSpeed,
                      int32_t timeout) {
  chassis.xSpeed = xSpeed;
  chassis.ySpeed = ySpeed;
  chassis.wSpeed = wSpeed;
  chassis.timeout = timeout;
}
static uint32_t lastTime = 0;
void chassis_task(void) {
  osDelay(4000);
  // for(;;) osDelay(1);
  while (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) != GPIO_PIN_RESET) {
    osDelay(1);
  }

  for (;;) {
    lastTime = xTaskGetTickCount();
    if (control_task_init()) {
      control_task_update();
    } else {
      chassis.wSpeed = 0.3;
      chassis.timeout = 1;
    }
    chassis.wheelSpeed[FRONT_LEFT] =
        chassis.xSpeed + chassis.ySpeed - chassis.r * chassis.wSpeed;
    chassis.wheelSpeed[FRONT_RIGHT] =
        -chassis.xSpeed + chassis.ySpeed - chassis.r * chassis.wSpeed;
    chassis.wheelSpeed[BACK_LEFT] =
        chassis.xSpeed - chassis.ySpeed - chassis.r * chassis.wSpeed;
    chassis.wheelSpeed[BACK_RIGHT] =
        -chassis.xSpeed - chassis.ySpeed - chassis.r * chassis.wSpeed;
    float maxAbs = 0;
    for (int i = 0; i < 4; i++) {
      if (fabsf(chassis.wheelSpeed[i]) > maxAbs) {
        maxAbs = fabsf(chassis.wheelSpeed[i]);
      }
    }
    // if (maxAbs > 1) {
    //   for (int i = 0; i < 4; i++) {
    //     chassis.wheelSpeed[i] /= maxAbs;
    //   }
    // }
    for (uint8_t i = 0; i < 4; i++) {
      if (chassis.wheelSpeed[i] > 1) {
        chassis.wheelSpeed[i] = 1;
      }
      if (chassis.wheelSpeed[i] < -1) {
        chassis.wheelSpeed[i] = -1;
      }
    }
    if (chassis.timeout > 0) {
      chassis.timeout--;
      for (int i = 0; i < 4; i++) {
        int pwm = 1500;
        pwm += chassis.wheelSpeed[i] * 1000;
        sprintf(chassisCommand, "#%03dP%04dT%04d!\r\n", i, pwm, 40);
        HAL_UART_Transmit(&huart2, (uint8_t *)chassisCommand,
                          sizeof(chassisCommand) - 1, 20);
      }
    }
    vTaskDelayUntil(&lastTime, 20);
  }
}
