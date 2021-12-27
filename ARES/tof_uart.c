#include "feedback_task.h"
#include "main.h"
#include "string.h"
#include "tof.h"
#include "usart.h"

#define TOF_UART huart2
#define TOF_UART_RX_BUFFER_SIZE 128
uint8_t tof_uart_count = 0;
uint8_t tof_uart_rx_buffer[TOF_UART_RX_BUFFER_SIZE];

int tofBias[6] = {8, 18, 9, 1, 32,0};

float tof_float[5];

void TofUart_init(void) {
  feedback_register(&tof_float[0], 2);
  feedback_register(&tof_float[1], 3);
  feedback_register(&tof_float[2], 4);
  feedback_register(&tof_float[3], 5);
  feedback_register(&tof_float[4], 6);
  __HAL_UART_ENABLE_IT(&TOF_UART, UART_IT_IDLE);
}

void UsartReceive_IDLE(UART_HandleTypeDef *huart) {
  uint32_t temp;
  if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET))  //如果收到一帧数据
  {
    __HAL_UART_CLEAR_IDLEFLAG(huart);  //清除标志位
    HAL_UART_DMAStop(huart);           //暂停DMA
    temp = huart->hdmarx->Instance->NDTR;
    tof_uart_count = TOF_UART_RX_BUFFER_SIZE - temp;  //计算数据长度
    if (tof_uart_count == TOF_UART_RX_BUFFER_SIZE) {
      memset(tof_uart_rx_buffer, 0, TOF_UART_RX_BUFFER_SIZE);
      tof_uart_count = 0;
    }

    if (tof_uart_count >= 5) {
      if (tof_uart_rx_buffer[tof_uart_count - 1] == '\n' &&
          tof_uart_rx_buffer[tof_uart_count - 2] == '\r') {
        if (tof_uart_rx_buffer[tof_uart_count - 5] >= 'A' &&
            tof_uart_rx_buffer[tof_uart_count - 5] <= 'F') {
          uint8_t id = tof_uart_rx_buffer[tof_uart_count - 5] - 'A';
          tof[id] = tof_uart_rx_buffer[tof_uart_count - 4] << 8 |
                    tof_uart_rx_buffer[tof_uart_count - 3];
          tof[id] -= tofBias[id];
          if(tof[id]>1000){
            tof[id] = 1000;
          }
          tof_float[id] = tof[id];
        }
        memset(tof_uart_rx_buffer, 0, TOF_UART_RX_BUFFER_SIZE);
        tof_uart_count = 0;
      }
    }
    HAL_UART_Receive_DMA(&huart2, tof_uart_rx_buffer,
                         TOF_UART_RX_BUFFER_SIZE);  //从串口缓存接收数据
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == &TOF_UART) {
    HAL_UART_Receive_DMA(&huart2, tof_uart_rx_buffer,
                         TOF_UART_RX_BUFFER_SIZE);  //从串口缓存接收数据
  }
}
