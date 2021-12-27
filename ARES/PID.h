/**
 * ****************************(C) COPYRIGHT 2021
 * ARES@SUSTech****************************
 *                                 ___     ____   ______ _____
 *                                /   |   / __ \ / ____// ___/
 *                               / /| |  / /_/ // __/   \__ \
 *                              / ___ | / _, _// /___  ___/ /
 *                             /_/  |_|/_/ |_|/_____/ /____/
 *                        Association of Robotics Engineers at SUSTech
 *
 * @file     PID.h
 * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
 * @brief    实现在通用控制器框架下的PID控制
 * @version  0.1
 * @date     2021-02-13
 *
 * ****************************(C) COPYRIGHT 2021
 * ARES@SUSTech****************************
 */
#ifndef PID_H
#define PID_H
#include "main.h"

typedef enum {
  NORMAL_INT,
  CLAMPING_INT,
  BACK_CALCULATION_INT,
} PID_IntegratorType;

typedef enum PID_Type {
  NONE_CONTROLLER = 0,  //未初始化
  PIDPOS_CONTROLLER,    //位置式PID控制
  PIDINC_CONTROLLER,    //增量式PID控制
  CONTROLLER_TYPE_NUM,
} PID_Type;

typedef struct {
  float kP;
  float kI;
  float kD;
  float kB;                     //反馈抗饱和系数
  float max_Iout;               //积分器钳位
  PID_IntegratorType Int_type;  //积分器形式
  float N;                      //一阶滤波常数，即为截止频率
} PID_Param;

typedef struct {
  float outMax;
  float outMin;
  float inMax;
  float inMin;
} PID_Constraint;

typedef struct {
  PID_Type type;
  PID_Param param;
  PID_Constraint constraint;
  float timeout;  //积分器超时 单位为秒
  float lastTime;
  float dt;
  float Pout;
  float Iout;
  float Dout[2];
  float DInt;
  float out[2];
  float err[3];
  float Ierr[2];
} PID_Controller;

extern void PID_ControllerInit(PID_Controller *self, PID_Type type,
                               PID_Constraint *constrain, PID_Param *param,
                               float timeout);
extern void PID_ControllerSetParam(PID_Controller *self, PID_Param *param);
extern void PID_Clear(PID_Controller *self);
extern float PID_ControllerUpdate(PID_Controller *self, float set, float ref);
#endif
