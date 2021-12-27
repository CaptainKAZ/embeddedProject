
#include "cmsis_os.h"
#include "vl53l0x_platform.h"
#define STATUS_OK 0x00
#define STATUS_FAIL 0x01
#include "myiic.h"
#ifndef MY_IIC
#include "i2c.h"
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata,
                                 uint32_t count) {
  char retries = 0;
  int ret = 0;
  unsigned short retry_in_mlsec = 10;

tryWriteAgain:
  ret = 0;
  ret = HAL_I2C_Mem_Write(Dev->hi2c, Dev->I2cDevAddr << 1, index,
                          I2C_MEMADD_SIZE_8BIT, pdata, count, 100);

  if (ret && retry_in_mlsec) {
    if (retries++ > 4) return ret;

    osDelay(retry_in_mlsec);
    goto tryWriteAgain;
  }
  //osDelay(10);
  return ret;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata,
                                uint32_t count) {
  char retries = 0;
  int ret = 0;
  unsigned short retry_in_mlsec = 10;

tryWriteAgain:
  ret = 0;
  ret = HAL_I2C_Mem_Read(Dev->hi2c, Dev->I2cDevAddr << 1, index,
                         I2C_MEMADD_SIZE_8BIT, pdata, count, 100);

  if (ret && retry_in_mlsec) {
    if (retries++ > 4) return ret;

    osDelay(retry_in_mlsec);
    goto tryWriteAgain;
  }
  //osDelay(10);
  return ret;
}

#else

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata,
                                 uint32_t count) {
  uint8_t i;
  IIC_Start();
  IIC_Send_Byte((Dev->I2cDevAddr << 1) | 0);  //发送器件地址+写命令
  if (IIC_Wait_Ack())                         //等待应答
  {
    IIC_Stop();
    return 1;
  }
  IIC_Send_Byte(index);  //写寄存器地址
  IIC_Wait_Ack();        //等待应答
  for (i = 0; i < count; i++) {
    IIC_Send_Byte(pdata[i]);  //发送数据
    if (IIC_Wait_Ack())       //等待ACK
    {
      IIC_Stop();
      return 1;
    }
  }
  IIC_Stop();
  return 0;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata,
                                uint32_t count){
  IIC_Start();
  IIC_Send_Byte((Dev->I2cDevAddr << 1) | 0);  //发送器件地址+写命令
  if (IIC_Wait_Ack())              //等待应答
  {
    IIC_Stop();
    return 1;
  }
  IIC_Send_Byte(index);  //写寄存器地址
  IIC_Wait_Ack();      //等待应答
  IIC_Start();
  IIC_Send_Byte((Dev->I2cDevAddr << 1) | 1);  //发送器件地址+读命令
  IIC_Wait_Ack();                  //等待应答
  while (count) {
    if (count == 1)
      *pdata = IIC_Read_Byte(0);  //读数据,发送nACK
    else
      *pdata = IIC_Read_Byte(1);  //读数据,发送ACK
    count--;
    pdata++;
  }
  IIC_Stop();  //产生一个停止条件
  return 0;
                                }

#endif
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data) {
  return VL53L0X_WriteMulti(Dev, index, &data, 1);
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data) {
  uint8_t buffer[sizeof(uint16_t)];

  // Split 16-bit word into MS and LS uint8_t
  buffer[0] = (uint8_t)(data >> 8);
  buffer[1] = (uint8_t)(data & 0x00FF);
  return VL53L0X_WriteMulti(Dev, index, buffer, sizeof(uint16_t));
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data) {
  uint8_t buffer[sizeof(uint32_t)];

  // Split 32-bit word into MS ... LS uint8_t
  buffer[0] = (uint8_t)(data >> 24);
  buffer[1] = (uint8_t)(data >> 16);
  buffer[2] = (uint8_t)(data >> 8);
  buffer[3] = (uint8_t)(data & 0xFF);
  return VL53L0X_WriteMulti(Dev, index, buffer, sizeof(uint32_t));
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data) {
  return VL53L0X_ReadMulti(Dev, index, data, 1);
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data) {
  uint8_t buffer[sizeof(uint16_t)];
  VL53L0X_Error Status =
      VL53L0X_ReadMulti(Dev, index, buffer, sizeof(uint16_t));
  *data = (buffer[0] << 8) | buffer[1];
  return Status;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data) {
  uint8_t buffer[sizeof(uint32_t)];
  VL53L0X_Error Status =
      VL53L0X_ReadMulti(Dev, index, buffer, sizeof(uint32_t));
  *data = (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
  return Status;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index,
                                 uint8_t AndData, uint8_t OrData) {
  uint8_t data;
  VL53L0X_Error Status = VL53L0X_RdByte(Dev, index, &data);
  if (Status == HAL_OK) {
    data = (data & AndData) | OrData;
    Status = VL53L0X_WrByte(Dev, index, data);
  }
  return Status;
}

#include "cmsis_os.h"

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev) {
  osDelay(50);
  return 0;
}
