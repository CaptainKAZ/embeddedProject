#include "iic_mpu9250.h"
#include "cmsis_os.h"
#ifndef MY_IIC
#include "i2c.h"

#define mdelay osDelay

unsigned long ST_Sensors_I2C_WriteRegister(unsigned char Address,
                                           unsigned char RegisterAddr,
                                           unsigned short RegisterLen,
                                           const unsigned char *RegisterValue);

unsigned long ST_Sensors_I2C_ReadRegister(unsigned char Address,
                                          unsigned char RegisterAddr,
                                          unsigned short RegisterLen,
                                          unsigned char *RegisterValue);

static unsigned short RETRY_IN_MLSEC = 55;

void Set_I2C_Retry(unsigned short ml_sec) { RETRY_IN_MLSEC = ml_sec; }

unsigned short Get_I2C_Retry() { return RETRY_IN_MLSEC; }

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  __HAL_RCC_I2C2_FORCE_RESET();
  HAL_I2C_DeInit(&hi2c2);
  MX_I2C2_Init();
}

int Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr,
                              unsigned short len,
                              const unsigned char *data_ptr) {
  char retries = 0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();

tryWriteAgain:
  ret = 0;
  ret = ST_Sensors_I2C_WriteRegister(slave_addr, reg_addr, len, data_ptr);

  if (ret && retry_in_mlsec) {
    if (retries++ > 4) return ret;

    mdelay(retry_in_mlsec);
    goto tryWriteAgain;
  }
  return ret;
}

int Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr,
                             unsigned short len, unsigned char *data_ptr) {
  char retries = 0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();

tryReadAgain:
  ret = 0;
  ret = ST_Sensors_I2C_ReadRegister(slave_addr, reg_addr, len, data_ptr);

  if (ret && retry_in_mlsec) {
    if (retries++ > 4) return ret;

    mdelay(retry_in_mlsec);
    goto tryReadAgain;
  }
  return ret;
}

/**
  * @brief  Writes a Byte to a given register to the sensors through the
            control interface (I2C)
  * @param  RegisterAddr: The address (location) of the register to be written.
  * @param  RegisterValue: the Byte value to be written into destination
  register.
  * @retval 0 if correct communication, else wrong communication
  */
unsigned long ST_Sensors_I2C_WriteRegister(unsigned char Address,
                                           unsigned char RegisterAddr,
                                           unsigned short RegisterLen,
                                           const unsigned char *RegisterValue) {
                                             return HAL_I2C_Mem_Write(&hi2c2, Address<<1, RegisterAddr,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t *)RegisterValue, RegisterLen,
                             100);
}

unsigned long ST_Sensors_I2C_ReadRegister(unsigned char Address,
                                          unsigned char RegisterAddr,
                                          unsigned short RegisterLen,
                                          unsigned char *RegisterValue) {
    return HAL_I2C_Mem_Read(&hi2c2, Address<<1, RegisterAddr, I2C_MEMADD_SIZE_8BIT,
                     RegisterValue, RegisterLen, 100);
}


#else
#include "main.h"
#include "myiic.h"
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf) {
  uint8_t i;
  IIC_Start();
  IIC_Send_Byte((addr << 1) | 0);  //发送器件地址+写命令
  if (IIC_Wait_Ack())              //等待应答
  {
    IIC_Stop();
    return 1;
  }
  IIC_Send_Byte(reg);  //写寄存器地址
  IIC_Wait_Ack();      //等待应答
  for (i = 0; i < len; i++) {
    IIC_Send_Byte(buf[i]);  //发送数据
    if (IIC_Wait_Ack())     //等待ACK
    {
      IIC_Stop();
      return 1;
    }
  }
  IIC_Stop();
  return 0;
}

// IIC连续读
// addr:器件地址
// reg:要读取的寄存器地址
// len:要读取的长度
// buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf) {
  IIC_Start();
  IIC_Send_Byte((addr << 1) | 0);  //发送器件地址+写命令
  if (IIC_Wait_Ack())              //等待应答
  {
    IIC_Stop();
    return 1;
  }
  IIC_Send_Byte(reg);  //写寄存器地址
  IIC_Wait_Ack();      //等待应答
  IIC_Start();
  IIC_Send_Byte((addr << 1) | 1);  //发送器件地址+读命令
  IIC_Wait_Ack();                  //等待应答
  while (len) {
    if (len == 1)
      *buf = IIC_Read_Byte(0);  //读数据,发送nACK
    else
      *buf = IIC_Read_Byte(1);  //读数据,发送ACK
    len--;
    buf++;
  }
  IIC_Stop();  //产生一个停止条件
  return 0;
}
#endif
void mpl_getTick(unsigned long *tick) {
  *tick = (unsigned long)xTaskGetTickCount();
}
