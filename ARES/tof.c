#include "tof.h"

#include "cmsis_os.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "myiic.h"
uint8_t tofAddr[5] = {0x30, 0x31, 0x32, 0x33, 0x34};
uint32_t xshutGPIO[5] = {XSHUT0_Pin, XSHUT1_Pin, XSHUT2_Pin, XSHUT3_Pin,
                         XSHUT4_Pin};
GPIO_TypeDef *xshutPort[5] = {XSHUT0_GPIO_Port, XSHUT1_GPIO_Port,
                              XSHUT2_GPIO_Port, XSHUT3_GPIO_Port,
                              XSHUT4_GPIO_Port};
VL53L0X_Dev_t tofDev[5];

#define XSHUT_LOW(dev) \
  HAL_GPIO_WritePin(dev.XSHUT_Port, dev.XSHUT_Pin, GPIO_PIN_RESET)
#define XSHUT_HIGH(dev) \
  HAL_GPIO_WritePin(dev.XSHUT_Port, dev.XSHUT_Pin, GPIO_PIN_SET)

void tofCalibrationInit(VL53L0X_Dev_t *dev) {
  uint8_t tempId = dev->I2cDevAddr;
  dev->I2cDevAddr = 0x29;
  while (VL53L0X_DataInit(dev)) {
    osDelay(20);
  }
  while (VL53L0X_SetDeviceAddress(dev, tempId)) {
    osDelay(20);
  }
  dev->I2cDevAddr = tempId;
  while (VL53L0X_StaticInit(dev)) {
    osDelay(20);
  }

  while (
      VL53L0X_PerformRefSpadManagement(dev, &dev->calibrationData.refSpadCount,
                                       &dev->calibrationData.isApertureSpads)) {
    osDelay(20);
  }
  while (VL53L0X_PerformRefCalibration(dev, &dev->calibrationData.VhvSettings,
                                       &dev->calibrationData.PhaseCal)) {
    osDelay(20);
  }
  uint32_t time = xTaskGetTickCount();
  while (xTaskGetTickCount() - time < 3000) {
    if (xTaskGetTickCount() % 100 == 0) {
      HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    }
    osDelay(1);
  }
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
  VL53L0X_PerformOffsetCalibration(dev, 100 << 16,
                                   &dev->calibrationData.offsetMM);
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
  osDelay(200);
  time = xTaskGetTickCount();
  while (xTaskGetTickCount() - time < 3000) {
    if (xTaskGetTickCount() % 100 == 0) {
      HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    }
    osDelay(1);
  }
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
  VL53L0X_PerformXTalkCalibration(dev, 450 << 16,
                                  &dev->calibrationData.XTalkCalDistance);
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
  osDelay(200);
}

void tofMiniumCalibration(VL53L0X_Dev_t *dev) {
  uint8_t tempId = dev->I2cDevAddr;
  dev->I2cDevAddr = 0x29;
  int8_t status;

  VL53L0X_WaitDeviceBooted(dev);  
  while ((status = VL53L0X_DataInit(dev))) {
    osDelay(20);
  }
  while (VL53L0X_SetDeviceAddress(dev, tempId)) {
    osDelay(20);
  }
  osDelay(10);
  dev->I2cDevAddr = tempId;
  while (VL53L0X_StaticInit(dev)) {
    osDelay(20);
  }
  while (
      VL53L0X_PerformRefSpadManagement(dev, &dev->calibrationData.refSpadCount,
                                       &dev->calibrationData.isApertureSpads)) {
    osDelay(20);
  }
  while (VL53L0X_PerformRefCalibration(dev, &dev->calibrationData.VhvSettings,
                                       &dev->calibrationData.PhaseCal)) {
    osDelay(20);
  }
}

void tofStart(VL53L0X_Dev_t *dev) {
  // VL53L0X_SetReferenceSpads(dev, dev->calibrationData.refSpadCount,
  //                           dev->calibrationData.isApertureSpads);
  // VL53L0X_SetRefCalibration(dev, dev->calibrationData.VhvSettings,
  //                           dev->calibrationData.PhaseCal);
  // VL53L0X_SetOffsetCalibrationDataMicroMeter(dev,
  //                                            dev->calibrationData.offsetMM);
  // VL53L0X_SetXTalkCompensationRateMegaCps(
  //     dev, dev->calibrationData.XTalkCalDistance);
  int8_t status = 0;
  status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  while (!status) {
    osDelay(20);
  }
  while (!VL53L0X_StartMeasurement(dev)) {
    osDelay(20);
  }
}

void tofPolling(VL53L0X_Dev_t *dev) {
  uint8_t dataReady = 0;
  VL53L0X_GetMeasurementDataReady(dev, &dataReady);
  if (dataReady) {
    VL53L0X_GetRangingMeasurementData(dev, &dev->measurementData);
    VL53L0X_ClearInterruptMask(dev, 0);
  }
}

uint8_t i2cDeviceList[100];
uint8_t i2cDeviceCount = 0;
extern void TofUart_init();

void tof_task() {
  //osDelay(2000);
  TofUart_init();
  vTaskSuspend(NULL);
  for (uint8_t i = 0; i < 5; i++) {
    tofDev[i].I2cDevAddr = tofAddr[i];
    #ifndef MY_IIC
    tofDev[i].hi2c = &hi2c1;
    #endif
    tofDev[i].XSHUT_Port = xshutPort[i];
    tofDev[i].XSHUT_Pin = xshutGPIO[i];
  }
  IIC_Init();
  osDelay(100);
  for (uint8_t i = 0; i < 5; i++) {
    XSHUT_LOW(tofDev[i]);
  }
  osDelay(30);
  for (uint8_t i = 0; i < 5; i++) {
    XSHUT_HIGH(tofDev[i]);
  }
  for (uint8_t i = 0; i < 5; i++) {
    XSHUT_LOW(tofDev[i]);
  }

  for (uint8_t i = 0; i < 5; i++) {
    XSHUT_HIGH(tofDev[i]);
    #ifndef MY_IIC
    while (HAL_OK!=HAL_I2C_IsDeviceReady(&hi2c1, 0x29<<1, 10, 10)){
      osDelay(10);
    }
    #endif
     HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    tofMiniumCalibration(&tofDev[i]);
    osDelay(5);
  }
#ifndef MY_IIC
  for (uint8_t i = 0; i < 100; i++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 10, 10) == HAL_OK) {
      i2cDeviceList[i2cDeviceCount++] = i;
    }
  }

  for (uint8_t i = 0; i < 5; i++) {
  while (HAL_OK!=HAL_I2C_IsDeviceReady(tofDev[i].hi2c,
  tofDev[i].I2cDevAddr<<1, 10, 10)){
    osDelay(10);
  }
  }
#endif
  for (uint8_t i = 0; i < 5; i++) {
    tofMiniumCalibration(&tofDev[i]);
  }
  for (uint8_t i = 0; i < 5; i++) {
    tofStart(&tofDev[i]);
  }
  for (;;) {
    for (uint8_t i = 0; i < 5; i++) {
      tofPolling(&tofDev[i]);
    }
    osDelay(5);
  }
}
