#include <math.h>

#include "cmsis_os.h"
#include "data_builder.h"
#include "eMPL_outputs.h"
#include "feedback_task.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "main.h"
#include "mltypes.h"
#include "mpl.h"
#include "mpu.h"
#include "quaternion_supervisor.h"
#include "yaw_kalman.h"

int tof[5];

struct mpu_convet {
  float gyro_sens;
  uint16_t accel_sens;
} mpu_convet;

struct mpu_data {
  float gyro[3];
  float accel[3];
  float compass[2];
  uint32_t timeStamp[3];
  uint32_t dt[3];
  short gyro_short[3], accel_short[3], compass_short[3];
  float gyro_bias[3], accel_bias[3], compass_center[2], compass_range[2];
} mpu_data;

void mpuInit(void) {
  struct int_param_s int_param;
  long gyro_bias[3], accel_bias[3];
  mpu_init(&int_param);
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  // mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(1000);
  mpu_set_compass_sample_rate(100);
  mpu_set_lpf(188);
  mpu_set_gyro_fsr(1000);
  mpu_set_accel_fsr(2);
  mpu_get_accel_sens(&mpu_convet.accel_sens);
  mpu_get_gyro_sens(&mpu_convet.gyro_sens);
  mpu_run_6500_self_test(gyro_bias, accel_bias, 0);
  mpu_data.gyro_bias[0] = (float)gyro_bias[0] / 65536;
  mpu_data.gyro_bias[1] = (float)gyro_bias[1] / 65536;
  mpu_data.gyro_bias[2] = (float)gyro_bias[2] / 65536;
  mpu_data.accel_bias[0] = (float)accel_bias[0] / 65536;
  mpu_data.accel_bias[1] = (float)accel_bias[1] / 65536;
  mpu_data.accel_bias[2] = (float)accel_bias[2] / 65536;
}

struct MagMeidanFilter {
  float mag_x[5];  // save the time info
  float mag_x_shorted[5];
  float mag_y[5];
  float mag_y_shorted[5];
} magMeidanFilter;

void mpuFilterMag(void) {
  // kick out the outdate data
  for (int i = 4; i > 0; i--) {
    magMeidanFilter.mag_x[i] = magMeidanFilter.mag_x[i - 1];
    magMeidanFilter.mag_y[i] = magMeidanFilter.mag_y[i - 1];
  }
  magMeidanFilter.mag_x[0] = mpu_data.compass[0];
  magMeidanFilter.mag_y[0] = mpu_data.compass[1];
  // copy data to sort buffer
  for (int i = 0; i < 5; i++) {
    magMeidanFilter.mag_x_shorted[i] = magMeidanFilter.mag_x[i];
    magMeidanFilter.mag_y_shorted[i] = magMeidanFilter.mag_y[i];
  }
  // bubble sort
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4 - i; j++) {
      if (magMeidanFilter.mag_x_shorted[j] >
          magMeidanFilter.mag_x_shorted[j + 1]) {
        float temp = magMeidanFilter.mag_x_shorted[j];
        magMeidanFilter.mag_x_shorted[j] = magMeidanFilter.mag_x_shorted[j + 1];
        magMeidanFilter.mag_x_shorted[j + 1] = temp;
      }
    }
  }
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4 - i; j++) {
      if (magMeidanFilter.mag_y_shorted[j] >
          magMeidanFilter.mag_y_shorted[j + 1]) {
        float temp = magMeidanFilter.mag_y_shorted[j];
        magMeidanFilter.mag_y_shorted[j] = magMeidanFilter.mag_y_shorted[j + 1];
        magMeidanFilter.mag_y_shorted[j + 1] = temp;
      }
    }
  }
  mpu_data.compass[0] = magMeidanFilter.mag_x_shorted[2];
  mpu_data.compass[1] = magMeidanFilter.mag_y_shorted[2];
}

void mpuCompensateMag(void) {
  mpu_data.compass[0] = (mpu_data.compass[0] - mpu_data.compass_center[0]) /
                        mpu_data.compass_range[0];
  mpu_data.compass[1] = (mpu_data.compass[1] - mpu_data.compass_center[1]) /
                        mpu_data.compass_range[1];
}

void mpuGetData(void) {
  short gyro_short[3], accel_short[3], compass_short[3];
  unsigned long timestamp;
  mpu_get_accel_reg(accel_short, &timestamp);
  mpu_get_gyro_reg(gyro_short, &timestamp);
  mpu_get_compass_reg(compass_short, &timestamp);
  if (gyro_short[0] != mpu_data.gyro_short[0] ||
      gyro_short[1] != mpu_data.gyro_short[1] ||
      gyro_short[2] != mpu_data.gyro_short[2]) {
    mpu_data.gyro_short[0] = gyro_short[0];
    mpu_data.gyro_short[1] = gyro_short[1];
    mpu_data.gyro_short[2] = gyro_short[2];
    mpu_data.gyro[0] = (float)gyro_short[0] / mpu_convet.gyro_sens;
    mpu_data.gyro[1] = (float)gyro_short[1] / mpu_convet.gyro_sens;
    mpu_data.gyro[2] = (float)gyro_short[2] / mpu_convet.gyro_sens;
    mpu_data.dt[0] = xTaskGetTickCount() - mpu_data.timeStamp[0];
    mpu_data.timeStamp[0] = xTaskGetTickCount();
  }
  if (accel_short[0] != mpu_data.accel_short[0] ||
      accel_short[1] != mpu_data.accel_short[1] ||
      accel_short[2] != mpu_data.accel_short[2]) {
    mpu_data.accel_short[0] = accel_short[0];
    mpu_data.accel_short[1] = accel_short[1];
    mpu_data.accel_short[2] = accel_short[2];
    mpu_data.accel[0] = (float)accel_short[0] / mpu_convet.accel_sens;
    mpu_data.accel[1] = (float)accel_short[1] / mpu_convet.accel_sens;
    mpu_data.accel[2] = (float)accel_short[2] / mpu_convet.accel_sens;
    mpu_data.dt[1] = xTaskGetTickCount() - mpu_data.timeStamp[1];
    mpu_data.timeStamp[1] = xTaskGetTickCount();
  }
  if (compass_short[0] != mpu_data.compass_short[0] ||
      compass_short[1] != mpu_data.compass_short[1] ||
      compass_short[2] != mpu_data.compass_short[2]) {
    mpu_data.compass_short[0] = compass_short[0];
    mpu_data.compass_short[1] = compass_short[1];
    mpu_data.compass[0] = mpu_data.compass_short[0];
    mpu_data.compass[1] = mpu_data.compass_short[1];
    mpu_data.compass_short[2] = compass_short[2];
    mpu_data.dt[2] = xTaskGetTickCount() - mpu_data.timeStamp[2];
    mpu_data.timeStamp[2] = xTaskGetTickCount();
    mpuFilterMag();
    mpuCompensateMag();
  }
}

void mpuCalAccelGyro(uint32_t ms) {
  uint32_t base = xTaskGetTickCount();
  float coef = (ms / 3) / (1 + ms / 3);
  while (xTaskGetTickCount() - base < ms) {
    mpuGetData();
    mpu_data.accel_bias[0] =
        (1 - coef) * mpu_data.accel[0] + coef * mpu_data.accel_bias[0];
    mpu_data.accel_bias[1] =
        (1 - coef) * mpu_data.accel[1] + coef * mpu_data.accel_bias[1];
    mpu_data.accel_bias[2] =
        0.0001f * (mpu_data.accel[2] - 1) + coef * mpu_data.accel_bias[2];
    mpu_data.gyro_bias[0] =
        (1 - coef) * mpu_data.gyro[0] + coef * mpu_data.gyro_bias[0];
    mpu_data.gyro_bias[1] =
        (1 - coef) * mpu_data.gyro[1] + coef * mpu_data.gyro_bias[1];
    mpu_data.gyro_bias[2] =
        (1 - coef) * mpu_data.gyro[2] + coef * mpu_data.gyro_bias[2];
    osDelay(1);
    if (xTaskGetTickCount() % 50 == 0)
      HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  }
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
}

int mpuCalCompassXY() {
  // static float xmin = 999, xmax = -999, ymin = 999, ymax = -999;
  // static float rLong = 1;
  // if (mpu_data.compass_short[0] < xmin)
  //   xmin = 0.6 * xmin + 0.4 * mpu_data.compass_short[0];
  // if (mpu_data.compass_short[0] > xmax)
  //   xmax = 0.6 * xmax + 0.4 * mpu_data.compass_short[0];
  // if (mpu_data.compass_short[1] < ymin)
  //   ymin = 0.6 * ymin + 0.4 * mpu_data.compass_short[1];
  // if (mpu_data.compass_short[1] > ymax)
  //   ymax = 0.6 * ymax + 0.4 * mpu_data.compass_short[1];
  // mpu_data.compass_center[0] = (xmax + xmin) / 2;
  // mpu_data.compass_center[1] = (ymax + ymin) / 2;
  // mpu_data.compass_range[1] = mpu_data.compass_range[0] =
  //     (xmax - xmin) / 4 + (ymax - ymin) / 4;
  // ;
  // mpuCompensateMag();
  // float theta = atan2(mpu_data.compass[1], mpu_data.compass[0]);
  // if (xmax - xmin > 400 && ymax - ymin > 400) {
  //   float r = sqrtf(mpu_data.compass[0] * mpu_data.compass[0] +
  //                   mpu_data.compass[1] * mpu_data.compass[1]);

  //   rLong = (rLong * 0.99f + r * 0.01f);
  //   xmin += (1 - rLong) * 0.01f;
  //   ymin += (1 - rLong) * 0.01f;
  //   xmax -= (1 - rLong) * 0.01f;
  //   ymax -= (1 - rLong) * 0.01f;
  //   return 1;
  // } else {
  //   return 0;
  // }
  mpuGetData();
  //mpu_data.compass_center[0] = 262.31f;
  //mpu_data.compass_center[1] = 25.1827f;
  //mpu_data.compass_range[0] = mpu_data.compass_range[1] = 478.2034f;
  mpu_data.compass_center[0] = 254.518f;
  mpu_data.compass_center[1] = 77.03f;
  mpu_data.compass_range[0] = mpu_data.compass_range[1] = 459.516f;
  //mpu_data.compass_range[0] = mpu_data.compass_range[1] = 1.0f;
  return 1;
}

#include "tof.h"

struct TofYaw {
  float tof_base_angle[5];
  int stage;
  float leftAngle;
  float rightAngle;
  float estimatedAngle;
  uint8_t usable;
} tofYaw = {
    .tof_base_angle = {0, 90, 180, 90, 0},
    .stage = 0,
    .leftAngle = 0,
    .rightAngle = 0,
    .usable = 0,
};

int updateTofYaw(void) {
  tofYaw.leftAngle =
      180 / M_PI *
      atan2(tof[TOF_LEFT_FRONT] - tof[TOF_LEFT_BACK], TOF_DISTANCE_MM);
  tofYaw.rightAngle =
      180 / M_PI *
      atan2(tof[TOF_RIGHT_FRONT] - tof[TOF_RIGHT_BACK], TOF_DISTANCE_MM);
  if (fabs(tofYaw.leftAngle - tofYaw.rightAngle) < 7.0f&&tofYaw.usable) {
    tofYaw.usable = 1;
    tofYaw.estimatedAngle = (tofYaw.leftAngle + tofYaw.rightAngle) / 2+tofYaw.tof_base_angle[tofYaw.stage];
    return 1;
  }else{
    tofYaw.usable = 0;
    return 0;
  }
}

float yaw_bias;
void getInitAngle(uint32_t time){
  uint32_t base = xTaskGetTickCount();
  while(xTaskGetTickCount()-base<time){
    mpuGetData();
    float yaw_bias_temp =
        180 / M_PI * atan2(mpu_data.compass[1], mpu_data.compass[0]) - 0;
    yaw_bias = 0.99 * yaw_bias + 0.01 * yaw_bias_temp;
    if (xTaskGetTickCount() % 50 == 0)
      HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    osDelay(1);
  }
}

YawKalman kalman;
float mag_yaw;

void yaw_task() {
  mpu_data.compass_range[0] = mpu_data.compass_range[1] = 1.0f;
  osDelay(19);
  mpuInit();
  YawKalman_init(&kalman, 0);
  mpuCalAccelGyro(5000);
  feedback_register(&mpu_data.compass[0], 0);
  feedback_register(&mpu_data.compass[1], 1);
  while (!mpuCalCompassXY()) {
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
    osDelay(1);
  }
  tofYaw.usable = 1;
  // while(!updateTofYaw()){
  //   osDelay(1);
  // }
  tofYaw.usable = 0;
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
  yaw_bias =
      180 / M_PI * atan2(mpu_data.compass[1], mpu_data.compass[0]) - 0;
  getInitAngle(1000);
  kalman.initted = 1;
  for (;;) {
    osDelay(1);
    mpuGetData();
    
    if (mpu_data.timeStamp[0] == xTaskGetTickCount()) {
      YawKalman_predict(&kalman, mpu_data.gyro[2] - mpu_data.gyro_bias[2],
                        0.0001, 0.0001, 0.001);
    }
    if (mpu_data.timeStamp[2] == xTaskGetTickCount()) {
      float mag_yaw_new =
          180 / M_PI * atan2(mpu_data.compass[1], mpu_data.compass[0]) -
          yaw_bias;
      while (mag_yaw_new - mag_yaw > 180) mag_yaw_new -= 360;
      while (mag_yaw_new - mag_yaw < -180) mag_yaw_new += 360;
      mag_yaw = mag_yaw_new;
      //YawKalman_update(&kalman, mag_yaw, 40);
    }
    // if(updateTofYaw()){
    //   YawKalman_update(&kalman, tofYaw.estimatedAngle, 2);
    // }
  }
}
