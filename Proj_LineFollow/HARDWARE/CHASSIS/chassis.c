#include "chassis.h"
#include "usart.h"
#include "inv_mpu.h"
//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F7开发板
//LED驱动代码
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/11/27
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////

//初始化PB0,PB1为输出.并使能这两个口的时钟
//LED IO初始化
float yawP = 0.25f;
float yawI = 0.006f;
float yawD = 0.1f;
float yawLineP = 0.245f;
float yawLineI = 0.00005f;
float yawLineD = 1.4f;


float wheelDiameter = 0.1f;
float ChassisDiagonalHalfDiameter = 0.12f;
float motorMaxRPS = 3.0f;


float pitch, roll, yaw; //欧拉角

float yawErrorLast = 0.0f,yawDesire = 0.0f, yawError = 0.0f, yawIntegral = 0.0f, controlValue = 0.0f;
float yawErrorLast2 = 0.0f,yawDesire2 = 0.0f, yawError2 = 0.0f, yawIntegral2 = 100.0f, controlValue2 = 0.0f;

void VelocityClosedLoopControl(float vel_x, float vel_y, float angvel_yaw)
{
  yawDesire2 = yawDesire2 + 0.06f * angvel_yaw;
   
  if (1)
  {
    yawError2 = yawDesire2 + yaw;
    while (yawError2 > 180.0f)
    {
      yawError2 -= 360.0f;
    }
    while (yawError2 < -180.0f)
    {
      yawError2 += 360.0f;
    }
    yawIntegral2 = yawIntegral2 + yawError2;
    if (yawIntegral2 * yawI > 1.5f)
    {
      yawIntegral2 = 1.5f / yawI;
    }
    else if (yawIntegral2 * yawI < -1.5f)
    {
      yawIntegral2 = -1.5f / yawI;
    }
    controlValue2 = yawError2 * yawP + yawIntegral2 * yawI + (yawError2-yawErrorLast2)*yawD;
    if (controlValue2 > 8.0f)
    {
      controlValue2 = 8.0f;
    }
    else if (controlValue2 < -8.0f)
    {
      controlValue2 = -8.0f;
    }
    ChassisMove(vel_x, vel_y, controlValue2);
    yawErrorLast2 = yawError2;
  }
}

void MazeVelocityControl(float vel_x, float vel_y, float yawError)
{

    while (yawError > 180.0f)
    {
      yawError -= 360.0f;
    }
    while (yawError < -180.0f)
    {
      yawError += 360.0f;
    }
    yawIntegral = yawIntegral + yawError;
    if (yawIntegral * yawLineI > 1.5f)
    {
      yawIntegral = 1.5f / yawLineI;
    }
    else if (yawIntegral * yawLineI < -1.5f)
    {
      yawIntegral = -1.5f / yawLineI;
    }
    controlValue = yawError * yawLineP + yawIntegral * yawLineI + (yawError-yawErrorLast)*yawLineD;
    if (controlValue > 10.0f)
    {
      controlValue = 10.0f;
    }
    else if (controlValue < -10.0f)
    {
      controlValue = -10.0f;
    }
    ChassisMove(vel_x, vel_y, controlValue);
    yawErrorLast = yawError;
  
}

// Vel需要为500-2500
// TimeInMs 需要为10-900
void VelocityControl(int ID, int vel, int timeInMs)
{
  printf("#00");
  printf("%d", ID);
  printf("P");
  if (vel < 1000)
  {
    printf("0");
  }
  if (vel > 2500)
  {
    printf("%d", 2500);
  }
  else if (vel < 500)
  {
    printf("%d", 500);
  }
  else
  {
    printf("%d", vel);
  }
  printf("T");
  if (timeInMs < 100)
  {
    printf("00");
  }
  else if (timeInMs < 1000)
  {
    printf("0");
  }
  printf("%d", timeInMs);
  printf("!\r\n");
}

// x,y方向单位为M/S,x velocity is forward,, y velocity is rightwards.
void ChassisMove(float vel_x, float vel_y, float angvel_yaw)
{
  float wheelRound = wheelDiameter * 3.14159f;
  float rps_x = vel_x / wheelRound;
  float rps_y = vel_y / wheelRound;
  float rps_ang = angvel_yaw * ChassisDiagonalHalfDiameter / wheelRound;
  int LF, LR, RF, RR;
  float LFFactor = 1.0f, LRFactor = 1.0f, RFFactor = 1.0f, RRFactor = 1.0f;
  LF = 1500 + (int)(LFFactor * 1000.0f / motorMaxRPS * (rps_x + rps_y - rps_ang));
  LR = 1500 + (int)(LRFactor * 1000.0f / motorMaxRPS * (rps_x - rps_y - rps_ang));
  RF = 1500 + (int)(RFFactor * 1000.0f / motorMaxRPS * (-rps_x + rps_y - rps_ang));
  RR = 1500 + (int)(RRFactor * 1000.0f / motorMaxRPS * (-rps_x - rps_y - rps_ang));
  VelocityControl(0, LR, 100);
  VelocityControl(1, RR, 100);
  VelocityControl(2, RF, 100);
  VelocityControl(3, LF, 100);
}
