/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    led_manager.c
  * @brief   LED status manager implementation.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "modules/control/led_manager.h"

#define LED_BLINK_SLOW_PERIOD_MS      700U
#define LED_BLINK_FAST_PERIOD_MS      150U

typedef struct
{
  LedMode_t mode;
  uint32_t custom_period_ms;
  uint32_t last_toggle_tick;
} LedRuntime_t;

static LedRuntime_t s_led = {LED_MODE_OFF, 300U, 0U};

static void Led_Write(uint8_t on)
{
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LedManager_Init(void)
{
  s_led.mode = LED_MODE_OFF;
  s_led.custom_period_ms = 300U;
  s_led.last_toggle_tick = HAL_GetTick();
  Led_Write(0U);
}

void LedManager_Update(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t period_ms = 0U;

  if (s_led.mode == LED_MODE_OFF)
  {
    Led_Write(0U);
    return;
  }
  if (s_led.mode == LED_MODE_ON)
  {
    Led_Write(1U);
    return;
  }

  if (s_led.mode == LED_MODE_BLINK_SLOW)
  {
    period_ms = LED_BLINK_SLOW_PERIOD_MS;
  }
  else if (s_led.mode == LED_MODE_BLINK_FAST)
  {
    period_ms = LED_BLINK_FAST_PERIOD_MS;
  }
  else
  {
    period_ms = (s_led.custom_period_ms > 0U) ? s_led.custom_period_ms : 300U;
  }

  if ((now - s_led.last_toggle_tick) >= period_ms)
  {
    /* LED nháy theo kiểu non-blocking để không chặn main loop. */
    LedManager_Toggle();
    s_led.last_toggle_tick = now;
  }
}

void LedManager_SetMode(LedMode_t mode)
{
  s_led.mode = mode;
  s_led.last_toggle_tick = HAL_GetTick();
  if (mode == LED_MODE_OFF)
  {
    Led_Write(0U);
  }
  else if (mode == LED_MODE_ON)
  {
    Led_Write(1U);
  }
}

LedMode_t LedManager_GetMode(void)
{
  return s_led.mode;
}

void LedManager_SetCustomBlink(uint32_t period_ms)
{
  s_led.custom_period_ms = (period_ms > 0U) ? period_ms : 300U;
}

void LedManager_Toggle(void)
{
  HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
}

const char *LedManager_ModeName(LedMode_t mode)
{
  switch (mode)
  {
    case LED_MODE_OFF:
      return "OFF";
    case LED_MODE_ON:
      return "ON";
    case LED_MODE_BLINK_SLOW:
      return "BLINK_SLOW";
    case LED_MODE_BLINK_FAST:
      return "BLINK_FAST";
    case LED_MODE_BLINK_CUSTOM:
      return "BLINK_CUSTOM";
    default:
      return "UNKNOWN";
  }
}

