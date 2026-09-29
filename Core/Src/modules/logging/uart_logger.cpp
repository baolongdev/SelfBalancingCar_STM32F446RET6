#include "modules/logging/uart_logger.hpp"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace
{
static uint32_t EnterCritical(void)
{
  /* Queue log được truy cập cả từ main loop lẫn ISR, nên cần critical section ngắn. */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void ExitCritical(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}
}

UartLogger::UartLogger()
  : huart_(NULL),
    head_(0U),
    tail_(0U),
    tx_len_(0U),
    tx_busy_(0U),
    dropped_count_(0U),
    enabled_(1U),
    min_level_(LogLevel::Info)
{
  uint8_t i;

  for (i = 0U; i < kRateSlotCount; i++)
  {
    rate_slots_[i].used = 0U;
    rate_slots_[i].key = 0U;
    rate_slots_[i].last_tick = 0U;
  }
}

UartLogger &UartLogger::instance()
{
  static UartLogger s_instance;
  return s_instance;
}

void UartLogger::init(UART_HandleTypeDef *huart)
{
  uint32_t primask = EnterCritical();

  huart_ = huart;
  head_ = 0U;
  tail_ = 0U;
  tx_len_ = 0U;
  tx_busy_ = 0U;
  dropped_count_ = 0U;

  ExitCritical(primask);
}

void UartLogger::setEnabled(uint8_t enabled)
{
  enabled_ = (enabled != 0U) ? 1U : 0U;
}

uint8_t UartLogger::isEnabled() const
{
  return enabled_;
}

void UartLogger::setLevel(LogLevel level)
{
  min_level_ = level;
}

LogLevel UartLogger::level() const
{
  return min_level_;
}

void UartLogger::process()
{
  kickTx();
}

void UartLogger::flush()
{
  while (1)
  {
    uint16_t head;
    uint16_t tail;

    process();

    {
      uint32_t primask = EnterCritical();
      head = head_;
      tail = tail_;
      ExitCritical(primask);
    }

    if (head == tail)
    {
      break;
    }
  }
}

void UartLogger::log(LogLevel level, const char *tag, const char *fmt, ...)
{
  va_list args;
  int written;
  char message[192];

  if ((fmt == NULL) || (canLog(level) == 0U))
  {
    return;
  }

  va_start(args, fmt);
  written = vsnprintf(message, sizeof(message), fmt, args);
  va_end(args);

  if (written <= 0)
  {
    return;
  }

  message[sizeof(message) - 1U] = '\0';
  writeMessage(level, tag, message);
}

void UartLogger::logRateLimited(LogLevel level, const char *tag, uint32_t key, uint32_t interval_ms, const char *fmt, ...)
{
  va_list args;
  int written;
  char message[192];
  uint32_t now_ms;

  if ((fmt == NULL) || (canLog(level) == 0U))
  {
    return;
  }

  now_ms = HAL_GetTick();
  if (allowRate(key, interval_ms, now_ms) == 0U)
  {
    return;
  }

  va_start(args, fmt);
  written = vsnprintf(message, sizeof(message), fmt, args);
  va_end(args);

  if (written <= 0)
  {
    return;
  }

  message[sizeof(message) - 1U] = '\0';
  writeMessage(level, tag, message);
}

void UartLogger::onTxCompleteFromISR(UART_HandleTypeDef *huart)
{
  uint32_t primask;

  if ((huart == NULL) || (huart != huart_))
  {
    return;
  }

  primask = EnterCritical();
  tail_ = (uint16_t)((tail_ + tx_len_) % kBufferSize);
  tx_len_ = 0U;
  tx_busy_ = 0U;
  ExitCritical(primask);
}

void UartLogger::onErrorFromISR(UART_HandleTypeDef *huart)
{
  uint32_t primask;

  if ((huart == NULL) || (huart != huart_))
  {
    return;
  }

  primask = EnterCritical();
  tx_len_ = 0U;
  tx_busy_ = 0U;
  dropped_count_++;
  ExitCritical(primask);
}

uint32_t UartLogger::droppedCount() const
{
  return dropped_count_;
}

uint8_t UartLogger::canLog(LogLevel level) const
{
  if (enabled_ == 0U)
  {
    return 0U;
  }

  if ((huart_ == NULL) || ((uint8_t)level > (uint8_t)min_level_))
  {
    return 0U;
  }

  return 1U;
}

uint8_t UartLogger::allowRate(uint32_t key, uint32_t interval_ms, uint32_t now_ms)
{
  uint8_t i;
  uint8_t free_index = 0xFFU;

  if (interval_ms == 0U)
  {
    return 1U;
  }

  /* Dùng slot cố định để rate-limit mà không phải cấp phát động. */
  for (i = 0U; i < kRateSlotCount; i++)
  {
    if ((rate_slots_[i].used != 0U) && (rate_slots_[i].key == key))
    {
      if ((now_ms - rate_slots_[i].last_tick) < interval_ms)
      {
        return 0U;
      }

      rate_slots_[i].last_tick = now_ms;
      return 1U;
    }

    if ((rate_slots_[i].used == 0U) && (free_index == 0xFFU))
    {
      free_index = i;
    }
  }

  if (free_index != 0xFFU)
  {
    rate_slots_[free_index].used = 1U;
    rate_slots_[free_index].key = key;
    rate_slots_[free_index].last_tick = now_ms;
    return 1U;
  }

  return 1U;
}

void UartLogger::writeMessage(LogLevel level, const char *tag, const char *msg)
{
  int prefix_len;
  uint32_t now_ms;
  char line[256];
  const char *safe_tag = (tag != NULL) ? tag : "LOG";

  now_ms = HAL_GetTick();
  prefix_len = snprintf(line, sizeof(line), "[%8lu ms][%s][%s] %s\r\n",
                        now_ms,
                        levelName(level),
                        safe_tag,
                        (msg != NULL) ? msg : "");

  if (prefix_len <= 0)
  {
    return;
  }

  if ((size_t)prefix_len >= sizeof(line))
  {
    line[sizeof(line) - 1U] = '\0';
    prefix_len = (int)sizeof(line) - 1;
  }

  (void)enqueue(line, (size_t)prefix_len);
  kickTx();
}

uint8_t UartLogger::enqueue(const char *text, size_t len)
{
  uint16_t head;
  uint16_t tail;
  uint16_t free_space;
  size_t i;
  uint32_t primask;

  if ((text == NULL) || (len == 0U))
  {
    return 0U;
  }

  primask = EnterCritical();
  head = head_;
  tail = tail_;

  if (head >= tail)
  {
    free_space = (uint16_t)(kBufferSize - (head - tail) - 1U);
  }
  else
  {
    free_space = (uint16_t)(tail - head - 1U);
  }

  if (len > (size_t)free_space)
  {
    /* Không chặn app khi đầy buffer; bỏ log mới và tăng bộ đếm dropped. */
    dropped_count_++;
    ExitCritical(primask);
    return 0U;
  }

  for (i = 0U; i < len; i++)
  {
    buffer_[head] = (uint8_t)text[i];
    head = (uint16_t)((head + 1U) % kBufferSize);
  }

  head_ = head;
  ExitCritical(primask);

  return 1U;
}

void UartLogger::kickTx()
{
  uint16_t head;
  uint16_t tail;
  uint16_t len;
  uint32_t primask;

  if ((huart_ == NULL) || (enabled_ == 0U))
  {
    return;
  }

  primask = EnterCritical();

  if (tx_busy_ != 0U)
  {
    ExitCritical(primask);
    return;
  }

  head = head_;
  tail = tail_;
  if (head == tail)
  {
    ExitCritical(primask);
    return;
  }

  if (head > tail)
  {
    len = (uint16_t)(head - tail);
  }
  else
  {
    /* Buffer vòng có thể bị wrap, nên mỗi lần chỉ bắn một đoạn liên tục. */
    len = (uint16_t)(kBufferSize - tail);
  }

  tx_len_ = len;
  tx_busy_ = 1U;
  ExitCritical(primask);

  if (HAL_UART_Transmit_IT(huart_, (uint8_t *)&buffer_[tail], len) != HAL_OK)
  {
    primask = EnterCritical();
    tx_busy_ = 0U;
    tx_len_ = 0U;
    dropped_count_++;
    ExitCritical(primask);
  }
}

const char *UartLogger::levelName(LogLevel level)
{
  switch (level)
  {
    case LogLevel::Error:
      return "ERR";
    case LogLevel::Warn:
      return "WRN";
    case LogLevel::Info:
      return "INF";
    case LogLevel::Debug:
      return "DBG";
    default:
      return "UNK";
  }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  UartLogger::instance().onTxCompleteFromISR(huart);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  UartLogger::instance().onErrorFromISR(huart);
}

