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
 * @file     PID.c
 * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
 * @brief    实现在通用控制器框架下的PID控制
 * @version  0.1
 * @date     2021-02-13
 *
 * ****************************(C) COPYRIGHT 2021
 * ARES@SUSTech****************************
 */
#include "PID.h"

#include <string.h>

#include "algorithm/user_lib.h"
#include "cmsis_os.h"
#include "math.h"
#define PID ((PID_Controller *)self)
#define PIDPARAM ((PID_Param *)(&(self)->param))

#define PID_NEEDINT(self)                                                    \
  ((PIDPARAM->Int_type == CLAMPING_INT)                                      \
       ? (!((__fabsf(PID->out[1]) >= PID->constraint.outMax) &&              \
            (SIGN(PID->out[1]) ^ (SIGN(PIDPARAM->kI) ^ SIGN(PID->err[0]))))) \
       : 1)

#define PID_Ierr(self)                                          \
  PIDPARAM->Int_type == BACK_CALCULATION_INT                    \
      ? PIDPARAM->kB *(PID->out[0] - PID->out[1]) + PID->err[0] \
      : PID->err[0]

void PID_Clear(PID_Controller *self) {
  self->err[0] = self->err[1] = self->err[2] = 0;
  self->Ierr[0] = self->Ierr[1] = 0;
  self->Dout[0] = self->Dout[1] = self->Iout = self->Pout = 0;
  self->out[1] = self->out[0] = 0;
  self->DInt = 0;
  PID->dt = 0;
  PID->lastTime = xTaskGetTickCount();
}

float PID_ControllerUpdate(PID_Controller *self, float set, float ref) {
  if (NULL == self && NULL == set && NULL == ref) {
    return 0;
  }
  if (self->type != PIDPOS_CONTROLLER && self->type != PIDINC_CONTROLLER) {
    return 0;
  }

  PID->err[2] = PID->err[1];
  PID->err[1] = PID->err[0];
  PID->err[0] = set - ref;
  if (self->constraint.inMin < self->constraint.outMin) {
    while (PID->err[0] <
           -(self->constraint.outMin - self->constraint.inMin) / 2) {
      PID->err[0] += self->constraint.outMin - self->constraint.inMin;
    }
    while (PID->err[0] >=
           (self->constraint.outMin - self->constraint.inMin) / 2) {
      PID->err[0] -= self->constraint.outMin - self->constraint.inMin;
    }
  }

  PID->Ierr[1] = PID->Ierr[0];
  PID->Ierr[0] = PID_Ierr(self);

  PID->dt = 0.001f * xTaskGetTickCount() - PID->lastTime;
  PID->lastTime = 0.001f * xTaskGetTickCount();

  if (PID->dt > PID->timeout || PID->dt == 0) {
    PID_Clear(PID);
    return 0;
  }

  switch (self->type) {
    case PIDPOS_CONTROLLER:
      PID->Pout = PIDPARAM->kP * PID->err[0];
      if (PID_NEEDINT(self)) {
        PID->Iout +=
            PIDPARAM->kI * 0.5f * (PID->Ierr[0] + PID->Ierr[1]) * PID->dt;
        CLAMP(PID->Iout, -PIDPARAM->max_Iout, PIDPARAM->max_Iout);
      }
      if (PIDPARAM->N != 0) {
        PID->Dout[1] = PID->Dout[0];
        PID->Dout[0] = PIDPARAM->kD * (PIDPARAM->N * PID->err[0] - PID->DInt);
        PID->DInt += 0.5f * PID->dt * (PID->Dout[0] + PID->Dout[1]);
      } else {
        PID->Dout[0] = PIDPARAM->kD * (PID->err[0] - PID->err[1]) / PID->dt;
      }
      PID->out[0] = PID->out[1] = PID->Pout + PID->Iout + PID->Dout[0];

      CLAMP(PID->out[0], self->constraint.outMin, self->constraint.outMax);
      return isnan(PID->out[0]) ? 0 : PID->out[0];
    case PIDINC_CONTROLLER:
      PID->Pout = PIDPARAM->kP * (PID->err[0] - PID->err[1]);
      PID->Iout = PIDPARAM->kI * (PID->err[0]);
      CLAMP(PID->Iout, -PIDPARAM->max_Iout, PIDPARAM->max_Iout);
      if (PIDPARAM->N == 0) {
        PID->Dout[0] =
            PIDPARAM->kD * (PID->err[0] - 2 * PID->err[1] + PID->err[2]);
      } else {
        PID->Dout[1] = PID->Dout[0];
        PID->Dout[0] = PIDPARAM->kD *
                       (PIDPARAM->N * (PID->err[0] - PID->err[1]) - PID->DInt);
        PID->DInt += 0.5f * PID->dt * (PID->Dout[0] + PID->Dout[1]);
      }
      PID->out[0] +=
          PID->dt * 0.5f * (PID->out[1] + PID->Pout + PID->Iout + PID->Dout[0]);
      PID->out[1] = PID->Pout + PID->Iout + PID->Dout[0];

      CLAMP(PID->out[0], self->constraint.outMin, self->constraint.outMax);
      return isnan(PID->out[0]) ? 0 : PID->out[0];
    default:
      return 0;
  }
}

void PID_ControllerSetParam(PID_Controller *self, PID_Param *param) {
  if (NULL == self || param == NULL) {
    return;
  }
  if (self->type != PIDPOS_CONTROLLER && self->type != PIDINC_CONTROLLER) {
    return;
  }
  memcpy(&self->param, param, sizeof(PID_Param));
}

void PID_ControllerInit(PID_Controller *self, PID_Type type,
                        PID_Constraint *constrain, PID_Param *param,
                        float timeout) {
  if (NULL == self || NULL == constrain || NULL == param) {
    return;
  }
  self->type = type;
  memcpy(&self->constraint, constrain, sizeof(PID_Constraint));
  memcpy(&self->param, param, sizeof(PID_Param));
  self->timeout = timeout;
  PID_Clear(self);
}
