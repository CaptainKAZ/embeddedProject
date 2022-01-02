#ifndef _CHASSIS_H
#define _CHASSIS_H
#endif


#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F7开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/6/10
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//LED端口定义
void usart1_send_char(u8 c);
void VelocityClosedLoopControl(float vel_x, float vel_y, float angvel_yaw);
void VelocityControl(int ID,int vel,int timeInMs);
void ChassisMove(float vel_x,float vel_y,float angvel_yaw);
void MazeVelocityControl(float vel_x, float vel_y, float angvel_yaw);

