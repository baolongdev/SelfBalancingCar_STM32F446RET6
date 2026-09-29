/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    uart_log.h
  * @brief   Lightweight UART logger utility.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __UART_LOG_H
#define __UART_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"
#include <stdint.h>

/* Logger mức C, phù hợp cho module thuần C hoặc lúc chưa muốn dùng queue bất đồng bộ C++. */
void UartLog_Init(UART_HandleTypeDef *huart);
void UartLog_SetEnabled(uint8_t enabled);
uint8_t UartLog_IsEnabled(void);
void UartLog_Print(const char *text);
void UartLog_Printf(const char *fmt, ...);
void UartLog_PrintTimestamped(const char *tag, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* __UART_LOG_H */
