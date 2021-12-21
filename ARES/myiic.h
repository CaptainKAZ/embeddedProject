#ifndef _MYIIC_H
#define _MYIIC_H
#include "main.h"

// IO方向设置
#define SDA_IN()                     \
  {                                  \
    GPIOH->MODER &= ~(3 << (5 * 2)); \
    GPIOH->MODER |= 0 << 5 * 2;      \
  }  // PH5输入模式
#define SDA_OUT()                    \
  {                                  \
    GPIOH->MODER &= ~(3 << (5 * 2)); \
    GPIOH->MODER |= 1 << 5 * 2;      \
  }  // PH5输出模式
// IO操作
#define IIC_SCL(n)                                        \
  (n ? HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET) \
     : HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_RESET))  // SCL
#define IIC_SDA(n)                                        \
  (n ? HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET) \
     : HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_RESET))  // SDA
#define READ_SDA HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_5)          //输入SDA

//IIC所有操作函数
void IIC_Init(void);                   //初始化IIC的IO口
void IIC_Start(void);                  //发送IIC开始信号
void IIC_Stop(void);                   //发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);       // IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);  // IIC读取一个字节
uint8_t IIC_Wait_Ack(void);                // IIC等待ACK信号
void IIC_Ack(void);                    //IIC发送ACK信号
void IIC_NAck(void);                   //IIC不发送ACK信号

void IIC_Write_One_Byte(uint8_t daddr, uint8_t addr, uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr, uint8_t addr);
#endif
