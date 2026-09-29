/*
 * logger_throughput_ratelimit_example.cpp
 * Stress UART logger with fast loop + rate-limit
 */

#include "modules/logging/uart_logger.hpp"

static uint32_t s_last_hb = 0U;

void LoggerExample_Init(void)
{
  UartLogger::instance().init(&huart1);
  UartLogger::instance().setLevel(LogLevel::Debug);
  LOGI("LOG", "logger throughput test start");
}

void LoggerExample_Loop(void)
{
  uint32_t now = HAL_GetTick();

  /* High-frequency debug logs are rate-limited by key */
  UartLogger::instance().logRateLimited(LogLevel::Debug, "FAST", 0x2001U, 20U,
                                        "tick=%lu", (unsigned long)now);

  if ((now - s_last_hb) >= 1000U)
  {
    s_last_hb = now;
    LOGI("HB", "dropped=%lu", (unsigned long)UartLogger::instance().droppedCount());
  }

  UartLogger::instance().process();
}

