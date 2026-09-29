#ifndef UART_LOGGER_HPP
#define UART_LOGGER_HPP

#include "usart.h"
#include <stddef.h>
#include <stdint.h>

enum class LogLevel : uint8_t
{
  Error = 0,
  Warn,
  Info,
  Debug
};

/* Logger C++ có buffer vòng + truyền UART interrupt để giảm việc block main loop. */
class UartLogger
{
public:
  static UartLogger &instance();

  void init(UART_HandleTypeDef *huart);
  void setEnabled(uint8_t enabled);
  uint8_t isEnabled() const;
  void setLevel(LogLevel level);
  LogLevel level() const;

  void process();
  void flush();

  void log(LogLevel level, const char *tag, const char *fmt, ...);
  void logRateLimited(LogLevel level, const char *tag, uint32_t key, uint32_t interval_ms, const char *fmt, ...);

  void onTxCompleteFromISR(UART_HandleTypeDef *huart);
  void onErrorFromISR(UART_HandleTypeDef *huart);

  uint32_t droppedCount() const;

private:
  UartLogger();

  struct RateLimitSlot
  {
    /* Mỗi key đại diện cho một nhóm log rate-limited. */
    uint8_t used;
    uint32_t key;
    uint32_t last_tick;
  };

  uint8_t canLog(LogLevel level) const;
  uint8_t allowRate(uint32_t key, uint32_t interval_ms, uint32_t now_ms);
  void writeMessage(LogLevel level, const char *tag, const char *msg);
  uint8_t enqueue(const char *text, size_t len);
  void kickTx();
  static const char *levelName(LogLevel level);

  static const uint16_t kBufferSize = 1024U;
  static const uint8_t kRateSlotCount = 16U;

  UART_HandleTypeDef *huart_;
  volatile uint16_t head_;
  volatile uint16_t tail_;
  volatile uint16_t tx_len_;
  volatile uint8_t tx_busy_;
  volatile uint32_t dropped_count_;
  uint8_t enabled_;
  LogLevel min_level_;
  RateLimitSlot rate_slots_[kRateSlotCount];
  uint8_t buffer_[kBufferSize];
};

#define LOGE(TAG, FMT, ...) UartLogger::instance().log(LogLevel::Error, TAG, FMT, ##__VA_ARGS__)
#define LOGW(TAG, FMT, ...) UartLogger::instance().log(LogLevel::Warn, TAG, FMT, ##__VA_ARGS__)
#define LOGI(TAG, FMT, ...) UartLogger::instance().log(LogLevel::Info, TAG, FMT, ##__VA_ARGS__)
#define LOGD(TAG, FMT, ...) UartLogger::instance().log(LogLevel::Debug, TAG, FMT, ##__VA_ARGS__)

#endif /* UART_LOGGER_HPP */
