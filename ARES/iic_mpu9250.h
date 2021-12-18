#ifndef IIC_MPU9250_H
#define IIC_MPU9250_H
#include "main.h"

extern int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                     unsigned char reg_addr, unsigned short len,
                                     const unsigned char *data_ptr);

extern int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                                    unsigned char reg_addr, unsigned short len,
                                    unsigned char *data_ptr);
extern void mpl_getTick(unsigned long *tick);

#endif
