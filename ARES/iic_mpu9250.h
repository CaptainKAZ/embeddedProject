#ifndef IIC_MPU9250_H
#define IIC_MPU9250_H
#include "main.h"
//#define MY_IIC
#ifndef MY_IIC
extern int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                     unsigned char reg_addr, unsigned short len,
                                     const unsigned char *data_ptr);

extern int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                                    unsigned char reg_addr, unsigned short len,
                                    unsigned char *data_ptr);
#else
#define Sensors_I2C_WriteRegister MPU_Write_Len
#define Sensors_I2C_ReadRegister MPU_Read_Len
extern uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len,
                             uint8_t *buf);
extern uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len,
                            uint8_t *buf);

#endif
extern void mpl_getTick(unsigned long *tick);
#endif
