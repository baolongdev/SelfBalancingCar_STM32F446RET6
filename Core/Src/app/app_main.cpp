#include "app/app_main.h"

#include "i2c.h"
#include "modules/control/led_manager.h"
#include "modules/control/balance_manager.hpp"
#include "modules/control/motor.h"
#include "modules/control/wheel_encoder.h"
#include "modules/logging/uart_logger.hpp"
#include "modules/sensors/icm20948.h"
#include "tim.h"
#include "usart.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stddef.h>

namespace
{
MotorController_t g_motor_controller;
WheelEncoder_t g_left_encoder;
WheelEncoder_t g_right_encoder;
ICM20948_t g_imu;
uint32_t g_next_imu_sample_ms;
uint32_t g_last_imu_sample_ms;
BalanceManager g_balance_manager;
BalanceInput g_balance_input{};
uint8_t g_balance_enable_requested;
volatile uint8_t g_uart_command_ready;
uint8_t g_uart_command_overflow;
uint16_t g_uart_command_length;
char g_uart_command_buffer[256];
volatile uint16_t g_uart_rx_head;
volatile uint16_t g_uart_rx_tail;
volatile uint32_t g_uart_rx_dropped;
uint8_t g_uart_rx_ring[512];
static const float kComplementaryGyroWeight = 0.985f;
/* Mechanical convention: car upright on its two wheels = 0 deg, lying flat =
   approximately +/-90 deg. */
static const float kBalanceAngleSign = 1.0f;
static const float kBalanceGyroSign = 1.0f;
/* Fill these after the encoder timer/pin mapping is confirmed on the PCB. */
static const float kEncoderCountsPerRev = 0.0f;
static const float kWheelCircumferenceM = 0.0f;

static const uint32_t kBalancePersistAddress = 0x08060000UL;
static const uint32_t kBalancePersistMagic = 0x42414C31UL; /* BAL1 */
static const uint32_t kBalancePersistVersion = 1UL;

struct BalancePersistRecord
{
  uint32_t magic;
  uint32_t version;
  uint32_t algorithm;
  float target_angle_deg;
  float max_output_percent;
  float boost_start_deg;
  float boost_max_output_percent;
  float tilt_cutoff_deg;
  float tilt_recover_deg;
  uint32_t stale_timeout_ms;
  uint32_t invert_output;
  float kp;
  float ki;
  float kd;
  float integral_limit;
  float angle_gain;
  float angular_rate_gain;
  float speed_gain;
  float position_gain;
  uint32_t min_start_percent;
  uint32_t min_run_percent;
  uint32_t accel_up_percent_per_s;
  uint32_t accel_down_percent_per_s;
  uint32_t crc;
};

static_assert((sizeof(BalancePersistRecord) % sizeof(uint32_t)) == 0U,
              "Balance persist record must be word aligned");

static uint32_t BalancePersistCrc(const BalancePersistRecord &record)
{
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&record);
  const size_t length = offsetof(BalancePersistRecord, crc);
  uint32_t crc = 2166136261UL;
  for (size_t i = 0U; i < length; ++i)
  {
    crc ^= bytes[i];
    crc *= 16777619UL;
  }
  return crc;
}

static BalancePersistRecord BalancePersistFromConfig(const BalanceConfig &config,
                                                     const MotorController_t &motor)
{
  BalancePersistRecord record{};
  record.magic = kBalancePersistMagic;
  record.version = kBalancePersistVersion;
  record.algorithm = static_cast<uint32_t>(config.algorithm);
  record.target_angle_deg = config.target_angle_deg;
  record.max_output_percent = config.max_output_percent;
  record.boost_start_deg = config.boost_start_deg;
  record.boost_max_output_percent = config.boost_max_output_percent;
  record.tilt_cutoff_deg = config.tilt_cutoff_deg;
  record.tilt_recover_deg = config.tilt_recover_deg;
  record.stale_timeout_ms = config.stale_timeout_ms;
  record.invert_output = config.invert_output;
  record.kp = config.pid.kp;
  record.ki = config.pid.ki;
  record.kd = config.pid.kd;
  record.integral_limit = config.pid.integral_limit;
  record.angle_gain = config.state_feedback.angle_gain;
  record.angular_rate_gain = config.state_feedback.angular_rate_gain;
  record.speed_gain = config.state_feedback.speed_gain;
  record.position_gain = config.state_feedback.position_gain;
  record.min_start_percent = motor.min_start_percent;
  record.min_run_percent = motor.min_run_percent;
  record.accel_up_percent_per_s = motor.custom_slew.accel_up_per_s;
  record.accel_down_percent_per_s = motor.custom_slew.accel_down_per_s;
  record.crc = BalancePersistCrc(record);
  return record;
}

static uint8_t BalancePersistRecordValid(const BalancePersistRecord &record)
{
  return (uint8_t)(record.magic == kBalancePersistMagic &&
                   record.version == kBalancePersistVersion &&
                   record.algorithm <= static_cast<uint32_t>(BalanceAlgorithm::StateFeedback) &&
                   isfinite(record.target_angle_deg) &&
                   isfinite(record.max_output_percent) &&
                   isfinite(record.boost_start_deg) &&
                   isfinite(record.boost_max_output_percent) &&
                   isfinite(record.tilt_cutoff_deg) &&
                   isfinite(record.tilt_recover_deg) &&
                   isfinite(record.kp) && isfinite(record.ki) && isfinite(record.kd) &&
                   isfinite(record.integral_limit) &&
                   record.crc == BalancePersistCrc(record));
}

static uint8_t BalancePersistLoad(BalanceConfig *config,
                                  uint8_t *min_start,
                                  uint8_t *min_run,
                                  uint16_t *accel_up,
                                  uint16_t *accel_down)
{
  const BalancePersistRecord &record =
    *reinterpret_cast<const BalancePersistRecord *>(kBalancePersistAddress);
  if ((config == NULL) || (min_start == NULL) || (min_run == NULL) ||
      (accel_up == NULL) || (accel_down == NULL) ||
      (BalancePersistRecordValid(record) == 0U))
  {
    return 0U;
  }
  config->algorithm = static_cast<BalanceAlgorithm>(record.algorithm);
  config->target_angle_deg = record.target_angle_deg;
  config->max_output_percent = record.max_output_percent;
  config->boost_start_deg = record.boost_start_deg;
  config->boost_max_output_percent = record.boost_max_output_percent;
  config->tilt_cutoff_deg = record.tilt_cutoff_deg;
  config->tilt_recover_deg = record.tilt_recover_deg;
  config->stale_timeout_ms = record.stale_timeout_ms;
  config->invert_output = (uint8_t)(record.invert_output != 0U);
  config->pid = {record.kp, record.ki, record.kd, record.integral_limit};
  config->state_feedback = {record.angle_gain, record.angular_rate_gain,
                             record.speed_gain, record.position_gain};
  *min_start = (uint8_t)((record.min_start_percent > 100U) ? 100U : record.min_start_percent);
  *min_run = (uint8_t)((record.min_run_percent > 100U) ? 100U : record.min_run_percent);
  *accel_up = (uint16_t)((record.accel_up_percent_per_s > 65535U) ?
                         65535U : record.accel_up_percent_per_s);
  *accel_down = (uint16_t)((record.accel_down_percent_per_s > 65535U) ?
                           65535U : record.accel_down_percent_per_s);
  return 1U;
}

static HAL_StatusTypeDef BalancePersistSave(const BalanceConfig &config,
                                             const MotorController_t &motor)
{
  BalancePersistRecord record = BalancePersistFromConfig(config, motor);
  FLASH_EraseInitTypeDef erase{};
  uint32_t erase_error = 0U;
  HAL_StatusTypeDef status = HAL_FLASH_Unlock();
  if (status != HAL_OK) return status;
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Sector = FLASH_SECTOR_7;
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  status = HAL_FLASHEx_Erase(&erase, &erase_error);
  if (status == HAL_OK)
  {
    const uint32_t *words = reinterpret_cast<const uint32_t *>(&record);
    for (uint32_t i = 0U; i < (sizeof(record) / sizeof(uint32_t)); ++i)
    {
      status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                 kBalancePersistAddress + (i * sizeof(uint32_t)),
                                 words[i]);
      if (status != HAL_OK) break;
    }
  }
  HAL_FLASH_Lock();
  return status;
}

static void SetStatusLedForBalance(const BalanceOutput &balance_output,
                                    int16_t pwm_left_command_percent,
                                    int16_t pwm_right_command_percent)
{
  LedMode_t mode = LED_MODE_OFF;

  if ((balance_output.faults & BALANCE_FAULT_IMU_INVALID) != 0U ||
      (balance_output.faults & BALANCE_FAULT_IMU_STALE) != 0U ||
      (balance_output.faults & BALANCE_FAULT_TILT_LIMIT) != 0U)
  {
    /* LED đơn: nháy nhanh thay cho cảnh báo đỏ khi mất I2C/IMU hoặc tilt. */
    mode = LED_MODE_BLINK_FAST;
  }
  else if (balance_output.active == 0U)
  {
    mode = LED_MODE_OFF;
  }
  else if ((abs((int)pwm_left_command_percent) <= 1) &&
           (abs((int)pwm_right_command_percent) <= 1) &&
           (fabsf(balance_output.angle_error_deg) <= 0.2f))
  {
    /* Gần target và lệnh PWM gần như bằng 0: sáng ổn định. */
    mode = LED_MODE_ON;
  }
  else
  {
    /* Lệnh PWM càng cao thì LED càng chớp nhanh; PWM thấp thì chớp chậm. */
    const int pwm_abs_left = abs((int)pwm_left_command_percent);
    const int pwm_abs_right = abs((int)pwm_right_command_percent);
    const int pwm_abs = (pwm_abs_left > pwm_abs_right) ? pwm_abs_left : pwm_abs_right;
    const int clamped_pwm = (pwm_abs > 100) ? 100 : pwm_abs;
    const uint32_t period_ms = (uint32_t)(900 - (clamped_pwm * 8));
    LedManager_SetCustomBlink(period_ms);
    mode = LED_MODE_BLINK_CUSTOM;
  }

  if (LedManager_GetMode() != mode)
  {
    LedManager_SetMode(mode);
  }
}

static void BalanceLogConfig()
{
  const BalanceConfig &c = g_balance_manager.config();
  LOGI("BALCFG", "alg=%s en=%u target_cdeg=%ld max_cpercent=%ld boost_start_cdeg=%ld boost_max_cpercent=%ld cutoff_cdeg=%ld recover_cdeg=%ld kp_milli=%ld ki_milli=%ld kd_milli=%ld ilim_milli=%ld ag_milli=%ld rg_milli=%ld sg_milli=%ld pg_milli=%ld invert=%u",
       BalanceManager::AlgorithmName(c.algorithm),
       (unsigned int)g_balance_manager.enabled(),
       (long)(c.target_angle_deg * 100.0f), (long)(c.max_output_percent * 100.0f),
       (long)(c.boost_start_deg * 100.0f), (long)(c.boost_max_output_percent * 100.0f),
       (long)(c.tilt_cutoff_deg * 100.0f), (long)(c.tilt_recover_deg * 100.0f),
       (long)(c.pid.kp * 1000.0f), (long)(c.pid.ki * 1000.0f),
       (long)(c.pid.kd * 1000.0f), (long)(c.pid.integral_limit * 1000.0f),
       (long)(c.state_feedback.angle_gain * 1000.0f),
       (long)(c.state_feedback.angular_rate_gain * 1000.0f),
       (long)(c.state_feedback.speed_gain * 1000.0f),
       (long)(c.state_feedback.position_gain * 1000.0f),
       (unsigned int)c.invert_output);
}

static uint8_t ParseFloat(const char *text, float *value)
{
  char *end = NULL;
  if ((text == NULL) || (value == NULL) || (*text == '\0')) return 0U;
  *value = strtof(text, &end);
  return (uint8_t)((end != text) && (*end == '\0') && isfinite(*value));
}

static uint8_t CommandValue(const char *token, float *value)
{
  const char *equal = (token != NULL) ? strchr(token, '=') : NULL;
  return (equal != NULL) ? ParseFloat(equal + 1, value) : 0U;
}

static void MotorLogConfig()
{
  LOGI("MOTORCFG", "min_start=%u min_run=%u accel_up=%u accel_down=%u timeout_ms=%lu mode=%u faults=0x%08lx",
       (unsigned int)g_motor_controller.min_start_percent,
       (unsigned int)g_motor_controller.min_run_percent,
       (unsigned int)g_motor_controller.custom_slew.accel_up_per_s,
       (unsigned int)g_motor_controller.custom_slew.accel_down_per_s,
       (unsigned long)g_motor_controller.command_timeout_ms,
       (unsigned int)g_motor_controller.drive_mode,
       (unsigned long)g_motor_controller.fault_flags);
}

static void ProcessMotorCommand()
{
  char *token = strtok(NULL, " \t\r\n");
  if (token == NULL || strcmp(token, "GET") == 0)
  {
    MotorLogConfig();
    return;
  }
  if (strcmp(token, "STOP") == 0)
  {
    MotorController_Stop(&g_motor_controller);
    LOGI("MOTOR", "stop");
    return;
  }
  if (g_balance_manager.enabled() != 0U)
  {
    LOGW("MOTOR", "reject manual motor command while balance enabled");
    return;
  }
  if (strcmp(token, "CFG") == 0)
  {
    uint8_t min_start = g_motor_controller.min_start_percent;
    uint8_t min_run = g_motor_controller.min_run_percent;
    uint16_t accel_up = g_motor_controller.custom_slew.accel_up_per_s;
    uint16_t accel_down = g_motor_controller.custom_slew.accel_down_per_s;
    uint32_t timeout = g_motor_controller.command_timeout_ms;
    uint8_t invalid_value = 0U;
    float value = 0.0f;
    while ((token = strtok(NULL, " \t\r\n")) != NULL)
    {
      if (strncmp(token, "min_start=", 10) == 0)
      {
        if ((CommandValue(token, &value) == 0U) || (value < 0.0f) || (value > 100.0f)) invalid_value = 1U;
        else min_start = (uint8_t)value;
      }
      else if (strncmp(token, "min_run=", 8) == 0)
      {
        if ((CommandValue(token, &value) == 0U) || (value < 0.0f) || (value > 100.0f)) invalid_value = 1U;
        else min_run = (uint8_t)value;
      }
      else if (strncmp(token, "accel_up=", 9) == 0)
      {
        if ((CommandValue(token, &value) == 0U) || (value <= 0.0f) || (value > 65535.0f)) invalid_value = 1U;
        else accel_up = (uint16_t)value;
      }
      else if (strncmp(token, "accel_down=", 11) == 0)
      {
        if ((CommandValue(token, &value) == 0U) || (value <= 0.0f) || (value > 65535.0f)) invalid_value = 1U;
        else accel_down = (uint16_t)value;
      }
      else if (strncmp(token, "timeout_ms=", 11) == 0)
      {
        if ((CommandValue(token, &value) == 0U) || (value < 0.0f) || (value > 5000.0f)) invalid_value = 1U;
        else timeout = (uint32_t)value;
      }
    }
    if (invalid_value != 0U || min_start > 100U || min_run > 100U || min_start < min_run ||
        accel_up == 0U || accel_down == 0U || timeout < 100U || timeout > 5000U)
    {
      LOGW("MOTOR", "reject invalid motor config");
      return;
    }
    MotorController_SetMinDrivePercent(&g_motor_controller, min_start, min_run);
    MotorController_SetCustomSlew(&g_motor_controller, accel_up, accel_down);
    MotorController_SetCommandTimeout(&g_motor_controller, timeout);
    MotorLogConfig();
    return;
  }

  char *speed_token = strtok(NULL, " \t\r\n");
  if (speed_token == NULL)
  {
    LOGW("MOTOR", "missing speed");
    return;
  }
  float speed_value = 0.0f;
  if ((ParseFloat(speed_token, &speed_value) == 0U) ||
      (speed_value < 0.0f) || (speed_value > 100.0f))
  {
    LOGW("MOTOR", "reject invalid speed");
    return;
  }
  const uint8_t speed = (uint8_t)speed_value;
  if (strcmp(token, "FWD") == 0) MotorController_Forward(&g_motor_controller, speed);
  else if (strcmp(token, "BACK") == 0) MotorController_Backward(&g_motor_controller, speed);
  else if (strcmp(token, "LEFT") == 0) MotorController_TurnLeft(&g_motor_controller, speed);
  else if (strcmp(token, "RIGHT") == 0) MotorController_TurnRight(&g_motor_controller, speed);
  else { LOGW("MOTOR", "unknown command"); return; }
  LOGI("MOTOR", "command=%s speed=%u", token, (unsigned int)speed);
}

static void ProcessBalanceCommand(char *command)
{
  char *token = strtok(command, " \t\r\n");
  if (token == NULL || strcmp(token, "BAL") != 0) return;

  token = strtok(NULL, " \t\r\n");
  if (token == NULL || strcmp(token, "GET") == 0 || strcmp(token, "?") == 0)
  {
    BalanceLogConfig();
    return;
  }

  if (strcmp(token, "MOTOR") == 0)
  {
    ProcessMotorCommand();
    return;
  }

  if (strcmp(token, "ENABLE") == 0)
  {
    char *value = strtok(NULL, " \t\r\n");
    g_balance_enable_requested = (uint8_t)((value != NULL && atoi(value) != 0) ? 1U : 0U);
    g_balance_manager.setEnabled(g_balance_enable_requested);
    if (g_balance_enable_requested == 0U) MotorController_Stop(&g_motor_controller);
    LOGI("BALCMD", "enabled=%u", (unsigned int)g_balance_manager.enabled());
    BalanceLogConfig();
    return;
  }

  if (strcmp(token, "DEFAULT") == 0)
  {
    if (g_balance_manager.enabled() != 0U)
    {
      LOGW("BALCMD", "reject default while balance enabled");
      return;
    }
    g_balance_manager.setConfig(BalanceManager::DefaultConfig());
    BalanceLogConfig();
    return;
  }

  if (strcmp(token, "SAVE") == 0)
  {
    if (g_balance_manager.enabled() != 0U)
    {
      LOGW("BALSTORE", "reject save while balance enabled");
      return;
    }
    const HAL_StatusTypeDef status = BalancePersistSave(g_balance_manager.config(),
                                                        g_motor_controller);
    if (status == HAL_OK)
    {
      LOGI("BALSTORE", "saved config to flash sector 7");
    }
    else
    {
      LOGE("BALSTORE", "flash save failed status=%u", (unsigned int)status);
    }
    return;
  }

  if (strcmp(token, "APPLY") != 0)
  {
    LOGW("BALCMD", "unknown command");
    return;
  }
  if (g_balance_manager.enabled() != 0U)
  {
    LOGW("BALCMD", "reject apply while balance enabled");
    return;
  }
  BalanceConfig config = g_balance_manager.config();
  uint8_t invalid_value = 0U;
  uint8_t invalid_algorithm = 0U;
  float value = 0.0f;
  while ((token = strtok(NULL, " \t\r\n")) != NULL)
  {
    if (strncmp(token, "alg=", 4) == 0)
    {
      const char *name = token + 4;
      if (strcmp(name, "PID") == 0) config.algorithm = BalanceAlgorithm::PID;
      else if (strcmp(name, "PD") == 0) config.algorithm = BalanceAlgorithm::PD;
      else if (strcmp(name, "STATE_FB") == 0) config.algorithm = BalanceAlgorithm::StateFeedback;
      else if (strcmp(name, "DISABLED") == 0) config.algorithm = BalanceAlgorithm::Disabled;
      else invalid_algorithm = 1U;
    }
    else if (strncmp(token, "kp=", 3) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.pid.kp) == 0U);
    else if (strncmp(token, "ki=", 3) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.pid.ki) == 0U);
    else if (strncmp(token, "kd=", 3) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.pid.kd) == 0U);
    else if (strncmp(token, "ilim=", 5) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.pid.integral_limit) == 0U);
    else if (strncmp(token, "ag=", 3) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.state_feedback.angle_gain) == 0U);
    else if (strncmp(token, "rg=", 3) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.state_feedback.angular_rate_gain) == 0U);
    else if (strncmp(token, "sg=", 3) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.state_feedback.speed_gain) == 0U);
    else if (strncmp(token, "pg=", 3) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.state_feedback.position_gain) == 0U);
    else if (strncmp(token, "target=", 7) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.target_angle_deg) == 0U);
    else if (strncmp(token, "max=", 4) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.max_output_percent) == 0U);
    else if (strncmp(token, "boost_start=", 12) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.boost_start_deg) == 0U);
    else if (strncmp(token, "boost_max=", 10) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.boost_max_output_percent) == 0U);
    else if (strncmp(token, "cutoff=", 7) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.tilt_cutoff_deg) == 0U);
    else if (strncmp(token, "recover=", 8) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.tilt_recover_deg) == 0U);
    else if (strncmp(token, "invert=", 7) == 0)
    {
      if (CommandValue(token, &value) == 0U) invalid_value = 1U;
      else config.invert_output = (uint8_t)(value != 0.0f);
    }
  }

  if ((invalid_value != 0U) || (invalid_algorithm != 0U) ||
      config.max_output_percent < 1.0f || config.max_output_percent > 100.0f ||
      config.boost_start_deg < 1.0f || config.boost_start_deg >= config.tilt_cutoff_deg ||
      config.boost_max_output_percent < config.max_output_percent ||
      config.boost_max_output_percent > 100.0f ||
      config.tilt_cutoff_deg < 5.0f || config.tilt_cutoff_deg > 89.0f ||
      config.tilt_recover_deg < 0.5f || config.tilt_recover_deg >= config.tilt_cutoff_deg ||
      config.pid.kp < 0.0f || config.pid.ki < 0.0f || config.pid.kd < 0.0f ||
      config.pid.integral_limit < 0.0f || config.state_feedback.angle_gain < 0.0f ||
      config.state_feedback.angular_rate_gain < 0.0f || config.state_feedback.speed_gain < 0.0f ||
      config.state_feedback.position_gain < 0.0f)
  {
    LOGW("BALCMD", "reject invalid config");
    return;
  }
  g_balance_manager.setConfig(config);
  LOGI("BALCMD", "config applied in RAM");
  BalanceLogConfig();
}

static void ProcessUartCommands()
{
  if (g_uart_command_ready == 0U) return;
  __disable_irq();
  char command[sizeof(g_uart_command_buffer)];
  memcpy(command, g_uart_command_buffer, sizeof(command));
  g_uart_command_ready = 0U;
  __enable_irq();
  ProcessBalanceCommand(command);
}

static void PollUartCommandInput()
{
  while (g_uart_command_ready == 0U && g_uart_rx_tail != g_uart_rx_head)
  {
    const uint8_t byte = g_uart_rx_ring[g_uart_rx_tail];
    g_uart_rx_tail = (uint16_t)((g_uart_rx_tail + 1U) % sizeof(g_uart_rx_ring));
    if (byte == '\n' || byte == '\r')
    {
      if (g_uart_command_overflow != 0U)
      {
        LOGW("UART", "command too long");
      }
      else if (g_uart_command_length > 0U)
      {
        g_uart_command_buffer[g_uart_command_length] = '\0';
        g_uart_command_ready = 1U;
      }
      g_uart_command_length = 0U;
      g_uart_command_overflow = 0U;
    }
    else if (g_uart_command_overflow == 0U &&
             g_uart_command_length < (sizeof(g_uart_command_buffer) - 1U))
    {
      g_uart_command_buffer[g_uart_command_length++] = (char)byte;
    }
    else
    {
      g_uart_command_overflow = 1U;
      g_uart_command_length = 0U;
    }
  }
}
}

extern "C" void AppMain_UartRxByteFromISR(uint8_t byte)
{
  const uint16_t next = (uint16_t)((g_uart_rx_head + 1U) % sizeof(g_uart_rx_ring));
  if (next == g_uart_rx_tail)
  {
    g_uart_rx_dropped++;
    return;
  }
  g_uart_rx_ring[g_uart_rx_head] = byte;
  g_uart_rx_head = next;
}

extern "C" void AppMain_UpdateWheelEncoderTicks(int32_t left_ticks,
                                                  int32_t right_ticks,
                                                  uint32_t timestamp_ms)
{
  WheelEncoder_UpdateCount(&g_left_encoder, left_ticks, timestamp_ms);
  WheelEncoder_UpdateCount(&g_right_encoder, right_ticks, timestamp_ms);
}

extern "C" void AppMain_Init(void)
{
  UartLogger &logger = UartLogger::instance();
  logger.init(&huart1);
  logger.setEnabled(1U);
  logger.setLevel(LogLevel::Debug);
  g_uart_command_length = 0U;
  g_uart_command_ready = 0U;
  g_uart_command_overflow = 0U;
  g_uart_rx_head = 0U;
  g_uart_rx_tail = 0U;
  g_uart_rx_dropped = 0U;
  WheelEncoder_Init(&g_left_encoder, kEncoderCountsPerRev, kWheelCircumferenceM);
  WheelEncoder_Init(&g_right_encoder, kEncoderCountsPerRev, kWheelCircumferenceM);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  HAL_StatusTypeDef imu_status = ICM20948_Init(&g_imu, &hi2c1, ICM20948_ADDRESS_AD0_LOW);
  if (imu_status != HAL_OK)
  {
    imu_status = ICM20948_Init(&g_imu, &hi2c1, ICM20948_ADDRESS_AD0_HIGH);
  }
  if (imu_status != HAL_OK)
  {
    LOGE("IMU", "ICM-20948 not detected at 0x68/0x69");
    /* Không khóa CPU tại đây: giữ UART telemetry sống để chẩn đoán nguồn,
       địa chỉ I2C hoặc dây PB6/PB7. ICM20948_Read() sẽ trả lỗi an toàn. */
    g_imu.initialized = 0U;
  }
  else
  {
    LOGI("IMU", "ICM-20948 ready");
    if (ICM20948_CalibrateGyro(&g_imu, 100U, 2U) == HAL_OK)
    {
      LOGI("IMU", "gyro bias calibrated bx_cdps=%ld by_cdps=%ld bz_cdps=%ld",
           (long)(g_imu.gyro_bias_dps[0] * 100.0f),
           (long)(g_imu.gyro_bias_dps[1] * 100.0f),
           (long)(g_imu.gyro_bias_dps[2] * 100.0f));
    }
    else
    {
      LOGW("IMU", "gyro calibration failed; using raw bias correction");
    }
  }
  g_next_imu_sample_ms = HAL_GetTick();
  g_last_imu_sample_ms = g_next_imu_sample_ms;

  /* Load the last UART-saved tuning before the balance manager starts. */
  BalanceConfig startup_balance_config = BalanceManager::DefaultConfig();
  uint8_t saved_min_start = 60U;
  uint8_t saved_min_run = 18U;
  uint16_t saved_accel_up = 9000U;
  uint16_t saved_accel_down = 13000U;
  const uint8_t saved_config = BalancePersistLoad(&startup_balance_config,
                                                  &saved_min_start,
                                                  &saved_min_run,
                                                  &saved_accel_up,
                                                  &saved_accel_down);
  LOGI("BALSTORE", "startup_config=%u", (unsigned int)saved_config);

  /* Balance starts automatically after a valid, safe IMU sample. A manual
     BAL ENABLE 0 command clears the request and keeps it disabled. */
  g_balance_manager.init(startup_balance_config);
  g_balance_manager.setAlgorithm(BalanceAlgorithm::PID);
  g_balance_enable_requested = 1U;
  g_balance_manager.setEnabled(0U);

  LedManager_Init();
  LedManager_SetMode(LED_MODE_OFF);

  MotorController_InitDefault(&g_motor_controller);
  /* Hiệu chỉnh mapping theo chiều lắp motor thực tế: đảo motor trái để
     Forward/Backward và TurnLeft/TurnRight khớp với hướng xe. */
  MotorController_SetInversion(&g_motor_controller, 1U, 0U);
  MotorController_SetStopMode(&g_motor_controller, MOTOR_STOP_BRAKE);
  /* Balance cần đảo chiều nhanh để chống quán tính; không dùng profile
     reverse dài của chế độ manual vì 80 ms sẽ bỏ lỡ nhiều chu kỳ 250 Hz. */
  MotorController_SetDirectionSafety(&g_motor_controller, 5U, 0U);
  /* Tăng lực khởi động vừa phải và giảm độ trễ ramp khi góc lệch tăng nhanh. */
  MotorController_SetMinDrivePercent(&g_motor_controller,
                                     (saved_config != 0U) ? saved_min_start : 60U,
                                     (saved_config != 0U) ? saved_min_run : 18U);
  MotorController_SetCustomSlew(&g_motor_controller,
                                (saved_config != 0U) ? saved_accel_up : 9000U,
                                (saved_config != 0U) ? saved_accel_down : 13000U);
  MotorController_SetCommandTimeout(&g_motor_controller, 600U);

  if (MotorController_Start(&g_motor_controller) != HAL_OK)
  {
    LOGE("INIT", "Motor controller start failed");
    /* Giữ main loop và UART chạy để xem lỗi phần cứng/PWM; motor vẫn ở trạng
       thái dừng vì Start() đã gọi MotorController_Stop(). */
  }

  MotorController_Stop(&g_motor_controller);
  LOGI("INIT", "Motor-only control ready");
}

extern "C" void AppMain_Loop(void)
{
  UartLogger &logger = UartLogger::instance();
  uint32_t now_ms = HAL_GetTick();
  PollUartCommandInput();
  ProcessUartCommands();

  /* 250 Hz sensor loop: 4 ms period, independent of motor/log processing. */
  if ((int32_t)(now_ms - g_next_imu_sample_ms) >= 0)
  {
    g_next_imu_sample_ms = now_ms + 4U;

    if (ICM20948_Read(&g_imu) == HAL_OK)
    {
      const uint32_t previous_sample_ms = g_last_imu_sample_ms;
      g_last_imu_sample_ms = now_ms;
      g_balance_input.timestamp_ms = now_ms;
      g_balance_input.dt_s = (float)(now_ms - previous_sample_ms) / 1000.0f;
    g_balance_input.imu_valid = 1U;
      (void)ICM20948_UpdateComplementaryFilter(&g_imu, g_balance_input.dt_s,
                                                kComplementaryGyroWeight);
      /* Roll theo quy ước cơ khí của xe dùng filtered_roll; tốc độ góc tương
         ứng vẫn là gyro-X. */
      g_balance_input.angle_deg = kBalanceAngleSign * g_imu.filtered_roll_deg;
      g_balance_input.angular_rate_dps = kBalanceGyroSign * g_imu.gyro_dps[0];
      g_balance_input.left_speed = WheelEncoder_GetSpeed(&g_left_encoder);
      g_balance_input.right_speed = WheelEncoder_GetSpeed(&g_right_encoder);
      g_balance_input.left_position = WheelEncoder_GetPosition(&g_left_encoder);
      g_balance_input.right_position = WheelEncoder_GetPosition(&g_right_encoder);
      g_balance_input.battery_voltage = 0.0f;

      /* Tự kích hoạt khi IMU hợp lệ và xe còn trong vùng an toàn. Khi góc
         vượt cutoff, BalanceManager sẽ latch fault và phía dưới dừng motor. */
      if (g_balance_enable_requested != 0U &&
           fabsf(g_balance_manager.config().target_angle_deg - g_imu.filtered_roll_deg) <
                g_balance_manager.config().tilt_cutoff_deg)
      {
        g_balance_manager.setEnabled(1U);
      }

      const BalanceOutput balance_output = g_balance_manager.update(g_balance_input);
      SetStatusLedForBalance(balance_output,
                              balance_output.left_motor_percent,
                              balance_output.right_motor_percent);
      if (balance_output.active != 0U)
      {
        MotorController_SetDriveMode(&g_motor_controller, MOTOR_DRIVE_MODE_CUSTOM);
        MotorController_Set(&g_motor_controller,
                            balance_output.left_motor_percent,
                            balance_output.right_motor_percent);
      }
      else
      {
        /* Fault, tilt limit hoặc IMU chưa sẵn sàng phải cắt lệnh ngay,
           không chờ command watchdog. */
        MotorController_Stop(&g_motor_controller);
      }

      logger.logRateLimited(
        LogLevel::Info,
        "IMU",
        0x20948U,
        200U,
        "roll_cdeg=%ld pitch_cdeg=%ld gyroX_cdps=%ld gyroY_cdps=%ld gyroZ_cdps=%ld ax_mg=%ld ay_mg=%ld az_mg=%ld",
         (long)(g_imu.filtered_roll_deg * 100.0f),
        (long)(g_imu.pitch_deg * 100.0f),
        (long)(g_imu.gyro_dps[0] * 100.0f),
        (long)(g_imu.gyro_dps[1] * 100.0f),
        (long)(g_imu.gyro_dps[2] * 100.0f),
        (long)(g_imu.accel_g[0] * 1000.0f),
        (long)(g_imu.accel_g[1] * 1000.0f),
        (long)(g_imu.accel_g[2] * 1000.0f));

      /* BAL telemetry cần luôn có ở mức Info để không phụ thuộc cấu hình
         logger Debug của bản firmware đang chạy. */
      logger.logRateLimited(
        LogLevel::Info,
        "BAL",
        0xBA1A1U,
        200U,
        "alg=%s en=%u err_cdeg=%ld out_cpercent=%ld pwmL_percent=%ld pwmR_percent=%ld fault=0x%08lx",
        BalanceManager::AlgorithmName(balance_output.algorithm),
        (unsigned int)g_balance_manager.enabled(),
        (long)(balance_output.angle_error_deg * 100.0f),
        (long)(balance_output.correction_percent * 100.0f),
        (long)g_motor_controller.applied_left_percent,
        (long)g_motor_controller.applied_right_percent,
        (unsigned long)balance_output.faults);
    }
    else
    {
      LedManager_SetMode(LED_MODE_BLINK_FAST);
      logger.logRateLimited(LogLevel::Error, "IMU", 0x20949U, 500U,
                            "ICM-20948 read failed st=%u i2c_err=0x%08lx",
                            (unsigned int)g_imu.last_status,
                            (unsigned long)g_imu.last_i2c_error);
    }
  }

  if (g_balance_manager.enabled() != 0U &&
      (now_ms - g_last_imu_sample_ms) > g_balance_manager.config().stale_timeout_ms)
  {
    g_balance_input.timestamp_ms = now_ms;
    g_balance_input.dt_s = (float)(now_ms - g_last_imu_sample_ms) / 1000.0f;
    g_balance_input.imu_valid = 0U;
    (void)g_balance_manager.update(g_balance_input);
    MotorController_Stop(&g_motor_controller);
    LedManager_SetMode(LED_MODE_BLINK_FAST);
  }

  MotorController_Update(&g_motor_controller);
  LedManager_Update();
  logger.process();
}

extern "C" void AppMain_SetDriveCommand(AppDriveCommand_t command, uint8_t speed_percent)
{
  MotorController_SetDriveMode(&g_motor_controller, MOTOR_DRIVE_MODE_CUSTOM);

  switch (command)
  {
    case APP_DRIVE_FORWARD:
      MotorController_Forward(&g_motor_controller, speed_percent);
      break;
    case APP_DRIVE_BACKWARD:
      MotorController_Backward(&g_motor_controller, speed_percent);
      break;
    case APP_DRIVE_TURN_LEFT:
      MotorController_TurnLeft(&g_motor_controller, speed_percent);
      break;
    case APP_DRIVE_TURN_RIGHT:
      MotorController_TurnRight(&g_motor_controller, speed_percent);
      break;
    case APP_DRIVE_STOP:
    default:
      MotorController_Stop(&g_motor_controller);
      break;
  }
}

extern "C" void AppMain_Forward(uint8_t speed_percent)
{
  AppMain_SetDriveCommand(APP_DRIVE_FORWARD, speed_percent);
}

extern "C" void AppMain_Backward(uint8_t speed_percent)
{
  AppMain_SetDriveCommand(APP_DRIVE_BACKWARD, speed_percent);
}

extern "C" void AppMain_TurnLeft(uint8_t speed_percent)
{
  AppMain_SetDriveCommand(APP_DRIVE_TURN_LEFT, speed_percent);
}

extern "C" void AppMain_TurnRight(uint8_t speed_percent)
{
  AppMain_SetDriveCommand(APP_DRIVE_TURN_RIGHT, speed_percent);
}

extern "C" void AppMain_Stop(void)
{
  AppMain_SetDriveCommand(APP_DRIVE_STOP, 0U);
}

extern "C" void AppMain_SetBalanceEnabled(uint8_t enabled)
{
  g_balance_enable_requested = (enabled != 0U) ? 1U : 0U;
  g_balance_manager.setEnabled(g_balance_enable_requested);
  if (g_balance_enable_requested == 0U)
  {
    MotorController_Stop(&g_motor_controller);
  }
}

extern "C" void AppMain_SetBalanceAlgorithm(uint8_t algorithm)
{
  if (algorithm <= (uint8_t)BalanceAlgorithm::StateFeedback)
  {
    g_balance_manager.setAlgorithm((BalanceAlgorithm)algorithm);
  }
}
