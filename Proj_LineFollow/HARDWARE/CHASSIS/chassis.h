#ifndef _CHASSIS_H
#define _CHASSIS_H
#endif


#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F7������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/6/10
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//LED�˿ڶ���
void usart1_send_char(u8 c);
void VelocityClosedLoopControl(float vel_x, float vel_y, float angvel_yaw);
void VelocityControl(int ID,int vel,int timeInMs);
void ChassisMove(float vel_x,float vel_y,float angvel_yaw);
void MazeVelocityControl(float vel_x, float vel_y, float angvel_yaw);

