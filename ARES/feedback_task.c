/**
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     feedback_task.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    状态反馈任务
  * @version  0.1
  * @date     2021-03-12
  * 
  * ****************************(C) COPYRIGHT 2021 ARES@SUSTech****************************
  */
#include "feedback_task.h"
#include "string.h"
#include "usart.h"
static float *  feedback_pointer[FEEDBACK_CHANNEL_NUM];
static uint8_t tx_buf[(FEEDBACK_CHANNEL_NUM + 1) * sizeof(float) + 4];

uint8_t feedback_register(float *ptr, uint8_t channel) {
  if (feedback_pointer[channel] == NULL) {
    feedback_pointer[channel] = ptr;
    return 0;
  }
  return 1;
}

void feedback_task() {
  vTaskDelay(303);
  
  while (1) {
    for (uint8_t i = 0; i < FEEDBACK_CHANNEL_NUM; i++) {
      if (feedback_pointer[i] != NULL) {
        memcpy(&tx_buf[i * sizeof(float)], feedback_pointer[i], sizeof(float));
      }else{
        memset(&tx_buf[i * sizeof(float)], 0, sizeof(float));
      }
    }
    float time = (float)xTaskGetTickCount();
    memcpy(&tx_buf[sizeof(float)*FEEDBACK_CHANNEL_NUM], &time, sizeof(float));
    tx_buf[(FEEDBACK_CHANNEL_NUM + 1) * sizeof(float) + 0] = 0x00;
    tx_buf[(FEEDBACK_CHANNEL_NUM + 1) * sizeof(float) + 1] = 0x00;
    tx_buf[(FEEDBACK_CHANNEL_NUM + 1) * sizeof(float) + 2] = 0x80;
    tx_buf[(FEEDBACK_CHANNEL_NUM + 1) * sizeof(float) + 3] = 0x7f;
    HAL_UART_Transmit_DMA(&huart1, tx_buf, sizeof(tx_buf));
    vTaskDelay(1);
  }
}
