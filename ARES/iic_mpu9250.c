#include "i2c.h"
#include "cmsis_os.h"
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
    return HAL_I2C_Mem_Write(&hi2c2, Address, RegisterAddr,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t *)RegisterValue, RegisterLen,
                             10);
}

unsigned long ST_Sensors_I2C_ReadRegister(unsigned char Address,
                                          unsigned char RegisterAddr,
                                          unsigned short RegisterLen,
                                          unsigned char *RegisterValue) {
    return HAL_I2C_Mem_Read(&hi2c2, Address, RegisterAddr, I2C_MEMADD_SIZE_8BIT,
                     RegisterValue, RegisterLen, 10);
}

void mpl_getTick(unsigned long *tick) {
  *tick = (unsigned long)xTaskGetTickCount();
}
