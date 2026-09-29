/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    led_manager.h
  * @brief   LED status manager.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __LED_MANAGER_H
#define __LED_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef enum
{
  LED_MODE_OFF = 0,
  LED_MODE_ON,
  LED_MODE_BLINK_SLOW,
  LED_MODE_BLINK_FAST,
  /* BLINK_CUSTOM cho phép app tự quyết định chu kỳ để biểu diễn từng state riêng. */
  LED_MODE_BLINK_CUSTOM
} LedMode_t;

/* Module LED rất nhỏ, nhưng tách riêng để app chỉ cần set mode thay vì tự quản thời gian nháy. */
void LedManager_Init(void);
void LedManager_Update(void);
void LedManager_SetMode(LedMode_t mode);
LedMode_t LedManager_GetMode(void);
void LedManager_SetCustomBlink(uint32_t period_ms);
void LedManager_Toggle(void);
const char *LedManager_ModeName(LedMode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* __LED_MANAGER_H */
