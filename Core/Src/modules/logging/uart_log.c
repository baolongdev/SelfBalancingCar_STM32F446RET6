/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    uart_log.c
  * @brief   Lightweight UART logger utility implementation.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "modules/logging/uart_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define UART_LOG_BUFFER_SIZE        192U
#define UART_LOG_TX_TIMEOUT_MS      50U

static UART_HandleTypeDef *s_log_uart = NULL;
static uint8_t s_log_enabled = 1U;

static void UartLog_WriteRaw(const char *text, uint16_t length)
{
  if ((s_log_uart == NULL) || (s_log_enabled == 0U) || (text == NULL) || (length == 0U))
  {
    return;
  }

  /* Bản logger này truyền blocking, phù hợp log ngắn và ít tần suất. */
  (void)HAL_UART_Transmit(s_log_uart, (uint8_t *)text, length, UART_LOG_TX_TIMEOUT_MS);
}

void UartLog_Init(UART_HandleTypeDef *huart)
{
  s_log_uart = huart;
}

void UartLog_SetEnabled(uint8_t enabled)
{
  s_log_enabled = (enabled != 0U) ? 1U : 0U;
}

uint8_t UartLog_IsEnabled(void)
{
  return s_log_enabled;
}

void UartLog_Print(const char *text)
{
  uint16_t len;

  if (text == NULL)
  {
    return;
  }

  len = (uint16_t)strlen(text);
  UartLog_WriteRaw(text, len);
}

void UartLog_Printf(const char *fmt, ...)
{
  va_list args;
  int written;
  char buffer[UART_LOG_BUFFER_SIZE];

  if (fmt == NULL)
  {
    return;
  }

  va_start(args, fmt);
  written = vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  if (written <= 0)
  {
    return;
  }

  if (written >= (int)sizeof(buffer))
  {
    written = (int)sizeof(buffer) - 1;
  }

  UartLog_WriteRaw(buffer, (uint16_t)written);
}

void UartLog_PrintTimestamped(const char *tag, const char *fmt, ...)
{
  va_list args;
  int written;
  int prefix_len;
  uint32_t now;
  char buffer[UART_LOG_BUFFER_SIZE];
  const char *safe_tag = (tag != NULL) ? tag : "LOG";

  if (fmt == NULL)
  {
    return;
  }

  /* Prefix timestamp giúp so thời gian giữa các sự kiện mà không cần debugger. */
  now = HAL_GetTick();
  prefix_len = snprintf(buffer, sizeof(buffer), "[%8lu ms][%s] ", now, safe_tag);
  if ((prefix_len <= 0) || (prefix_len >= (int)sizeof(buffer)))
  {
    return;
  }

  va_start(args, fmt);
  written = vsnprintf(&buffer[prefix_len], sizeof(buffer) - (uint32_t)prefix_len, fmt, args);
  va_end(args);

  if (written < 0)
  {
    return;
  }

  if ((prefix_len + written) >= (int)sizeof(buffer))
  {
    written = (int)sizeof(buffer) - prefix_len - 1;
  }

  UartLog_WriteRaw(buffer, (uint16_t)(prefix_len + written));
}

