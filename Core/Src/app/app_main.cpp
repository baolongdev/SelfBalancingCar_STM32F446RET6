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
uint32_t g_uart_rx_dropped_reported;
uint8_t g_uart_rx_ring[512];
uint8_t g_imu_fail_count;
uint32_t g_last_imu_recovery_ms;
uint32_t g_last_imu_health_ms;
/* Bo dem chan doan, in kem log loi IMU de biet loi xay ra thuong xuyen co nao. */
uint32_t g_imu_read_errors;
uint32_t g_imu_bus_recoveries;
uint32_t g_imu_reinits;
/* Log nhanh [T] de tune PID (0 = tat). Log [IMU]/[BAL] 200 ms qua cham de
   thay dao dong 2-8 Hz. */
uint16_t g_fast_log_ms;

/* Dead-reckoning 2D khi chua co encoder: yaw = tich phan gyro Z (xe dung,
   truc Z huong len, CCW duong); van toc = k * PWM trung binh hai banh (motor
   chay theo dien ap nen PWM on dinh ~ van toc). k can calibrate: cho xe chay
   thang mot doan do bang thuoc roi chinh "BAL ODOM k=". Chieu "+x" = chieu
   PWM duong. Sai so van toc ~20-30% -> chi de xem huong troi, khong do duong. */
struct Odometry
{
  float yaw_deg;
  float x_m;
  float y_m;
  float speed_mps;
};
Odometry g_odom;
float g_odom_mps_per_percent = 0.008f;

/* Giu huong bang gyro Z: lech PWM hai banh de xe khong xoay/di vong do hai
   motor khong deu. Mac dinh bat (kp=0.4, kd=0.04); tat bang "BAL YAW kp=0 kd=0".
   yaw_sign dao chieu neu lap motor nguoc (tu kiem tra bang log). */
float g_yaw_kp;
float g_yaw_kd;
float g_yaw_sign = 1.0f;
float g_yaw_target_deg;
uint8_t g_yaw_was_active;
static const float kYawMaxDiffPercent = 25.0f;
/* Sai lech yaw lon -> gan nhu chac chan sai dau: tu tat de xe khong xoay vong. */
static const float kYawRunawayDeg = 60.0f;

/* Calibrate gyro bias lien tuc khi xe nam yen (motor dung): bias luc boot co
   the sai neu xe dang bi cam, va bias troi theo nhiet do. */
struct GyroCalibWindow
{
  float sum[3];
  float min_dps[3];
  float max_dps[3];
  uint16_t count;
};
GyroCalibWindow g_gyro_calib;
static const uint16_t kGyroCalibSamples = 500U; /* 2 s o 250 Hz */
static const float kGyroCalibMaxSpreadDps = 3.0f;
static const uint32_t kImuSamplePeriodMs = 4U;
/* Moi lan doc loi do bus: giai phong bus ngay (nhanh, khong dong toi chip).
   Neu van loi lien tiep kImuFailuresBeforeRecovery lan: init lai ca IMU. */
static const uint8_t kImuFailuresBeforeRecovery = 5U;
static const uint32_t kImuRecoveryIntervalMs = 500U;
/* Chu ky kiem tra chip con song (WHO_AM_I + PWR_MGMT_1). */
static const uint32_t kImuHealthCheckMs = 500U;
/* Do tren xe that: luc can bang, goc tinh rieng tu accel dao dong +/-15-26 deg
   (gia toc ngang cua xe). 0.985 (tau ~0.27 s) de goc loc bi keo lech vai deg
   moi khi xe tang toc -> xe giu sai do nghieng va chay luon. 0.998 (tau ~2 s)
   giam anh huong ~7 lan; gyro bias da tu calibrate nen troi nho. */
static const float kComplementaryGyroWeight = 0.998f;
/* |a| lech khoi 1 g qua nguong nay -> chi dung gyro cho mau do. */
static const float kAccelTrustBandG = 0.1f;
/* Mechanical convention: car upright on its two wheels = 0 deg, lying flat =
   approximately +/-90 deg. */
static const float kBalanceAngleSign = 1.0f;
static const float kBalanceGyroSign = 1.0f;
/* Bu lech lap dat IMU (deg), cong vao goc roll. Calibrate bang "BAL IMU ZERO"
   khi giu xe thang dung theo thuoc thuy; luu flash cung BAL SAVE. */
float g_roll_offset_deg = 0.0f;
static const float kRollOffsetMaxDeg = 15.0f;
/* Fill these after the encoder timer/pin mapping is confirmed on the PCB. */
static const float kEncoderCountsPerRev = 0.0f;
static const float kWheelCircumferenceM = 0.0f;

static const uint32_t kBalancePersistAddress = 0x08060000UL;
static const uint32_t kBalancePersistMagic = 0x42414C31UL; /* BAL1 */
static const uint32_t kBalancePersistVersion = 5UL;

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
  float angle_gain;
  float angular_rate_gain;
  float speed_gain;
  float position_gain;
  float drift_gain_deg_per_percent;
  float drift_tau_s;
  float drift_max_deg;
  float drift_trim_rate;
  uint32_t min_start_percent;
  uint32_t min_run_percent;
  uint32_t accel_up_percent_per_s;
  uint32_t accel_down_percent_per_s;
  float imu_roll_offset_deg;
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
  record.angle_gain = config.state_feedback.angle_gain;
  record.angular_rate_gain = config.state_feedback.angular_rate_gain;
  record.speed_gain = config.state_feedback.speed_gain;
  record.position_gain = config.state_feedback.position_gain;
  record.drift_gain_deg_per_percent = config.drift_gain_deg_per_percent;
  record.drift_tau_s = config.drift_tau_s;
  record.drift_max_deg = config.drift_max_deg;
  record.drift_trim_rate = config.drift_trim_rate;
  record.min_start_percent = motor.min_start_percent;
  record.min_run_percent = motor.min_run_percent;
  record.accel_up_percent_per_s = motor.custom_slew.accel_up_per_s;
  record.accel_down_percent_per_s = motor.custom_slew.accel_down_per_s;
  record.imu_roll_offset_deg = g_roll_offset_deg;
  record.crc = BalancePersistCrc(record);
  return record;
}

static uint8_t GainValid(float gain)
{
  return (uint8_t)(isfinite(gain) && (gain >= 0.0f));
}

/* Dung chung cho BAL APPLY va khi load tu flash. Cac so sanh viet o dang
   duong de NaN luon bi tu choi. */
static uint8_t BalanceConfigValid(const BalanceConfig &c)
{
  return (uint8_t)(c.algorithm <= BalanceAlgorithm::StateFeedback &&
                   fabsf(c.target_angle_deg) <= 30.0f &&
                   c.max_output_percent >= 1.0f && c.max_output_percent <= 100.0f &&
                   c.tilt_cutoff_deg >= 5.0f && c.tilt_cutoff_deg <= 89.0f &&
                   c.boost_start_deg >= 1.0f && c.boost_start_deg < c.tilt_cutoff_deg &&
                   c.boost_max_output_percent >= c.max_output_percent &&
                   c.boost_max_output_percent <= 100.0f &&
                   c.tilt_recover_deg >= 0.5f && c.tilt_recover_deg < c.tilt_cutoff_deg &&
                   c.stale_timeout_ms >= 5U && c.stale_timeout_ms <= 200U &&
                   GainValid(c.pid.kp) && GainValid(c.pid.ki) && GainValid(c.pid.kd) &&
                   GainValid(c.state_feedback.angle_gain) &&
                   GainValid(c.state_feedback.angular_rate_gain) &&
                   GainValid(c.state_feedback.speed_gain) &&
                   GainValid(c.state_feedback.position_gain) &&
                   c.drift_gain_deg_per_percent >= 0.0f && c.drift_gain_deg_per_percent <= 0.5f &&
                   c.drift_tau_s >= 0.05f && c.drift_tau_s <= 20.0f &&
                   c.drift_max_deg >= 0.0f && c.drift_max_deg <= 15.0f &&
                   c.drift_trim_rate >= 0.0f && c.drift_trim_rate <= 0.1f);
}

static uint8_t MotorConfigValid(uint32_t min_start, uint32_t min_run,
                                uint32_t accel_up, uint32_t accel_down)
{
  return (uint8_t)(min_start <= 100U && min_run <= min_start &&
                   accel_up > 0U && accel_up <= 65535U &&
                   accel_down > 0U && accel_down <= 65535U);
}

static uint8_t BalancePersistRecordValid(const BalancePersistRecord &record)
{
  return (uint8_t)(record.magic == kBalancePersistMagic &&
                   record.version == kBalancePersistVersion &&
                   record.algorithm <= static_cast<uint32_t>(BalanceAlgorithm::StateFeedback) &&
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
  BalanceConfig loaded = *config;
  loaded.algorithm = static_cast<BalanceAlgorithm>(record.algorithm);
  loaded.target_angle_deg = record.target_angle_deg;
  loaded.max_output_percent = record.max_output_percent;
  loaded.boost_start_deg = record.boost_start_deg;
  loaded.boost_max_output_percent = record.boost_max_output_percent;
  loaded.tilt_cutoff_deg = record.tilt_cutoff_deg;
  loaded.tilt_recover_deg = record.tilt_recover_deg;
  loaded.stale_timeout_ms = record.stale_timeout_ms;
  loaded.invert_output = (uint8_t)(record.invert_output != 0U);
  loaded.pid = {record.kp, record.ki, record.kd};
  loaded.state_feedback = {record.angle_gain, record.angular_rate_gain,
                           record.speed_gain, record.position_gain};
  loaded.drift_gain_deg_per_percent = record.drift_gain_deg_per_percent;
  loaded.drift_tau_s = record.drift_tau_s;
  loaded.drift_max_deg = record.drift_max_deg;
  loaded.drift_trim_rate = record.drift_trim_rate;
  /* Record co CRC dung van co the chua gia tri ngoai mien hop le (vd firmware
     cu luu khi chua kiem tra du) -> bo qua toan bo, dung default. */
  if ((BalanceConfigValid(loaded) == 0U) ||
      (MotorConfigValid(record.min_start_percent, record.min_run_percent,
                        record.accel_up_percent_per_s, record.accel_down_percent_per_s) == 0U))
  {
    return 0U;
  }
  *config = loaded;
  *min_start = (uint8_t)record.min_start_percent;
  *min_run = (uint8_t)record.min_run_percent;
  *accel_up = (uint16_t)record.accel_up_percent_per_s;
  *accel_down = (uint16_t)record.accel_down_percent_per_s;
  if (isfinite(record.imu_roll_offset_deg) &&
      (fabsf(record.imu_roll_offset_deg) <= kRollOffsetMaxDeg))
  {
    g_roll_offset_deg = record.imu_roll_offset_deg;
  }
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
  /* Xoa sector 128 KB lam CPU dung ~1-2 s (fetch tu flash bi stall, ke ca
     ISR): UART RX mat byte trong luc nay. Chi goi khi balance da tat. */
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

/* LedManager_SetMode() reset pha nhay; chi goi khi mode thuc su doi, neu
   khong goi lien tuc moi 4 ms se lam LED khong bao gio toggle. */
static void SetLedMode(LedMode_t mode)
{
  if (LedManager_GetMode() != mode)
  {
    LedManager_SetMode(mode);
  }
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

  SetLedMode(mode);
}

static void BalanceLogConfig()
{
  const BalanceConfig &c = g_balance_manager.config();
  LOGI("BALCFG", "alg=%s en=%u target_cdeg=%ld max_cpercent=%ld boost_start_cdeg=%ld boost_max_cpercent=%ld cutoff_cdeg=%ld recover_cdeg=%ld kp_milli=%ld ki_milli=%ld kd_milli=%ld ag_milli=%ld rg_milli=%ld sg_milli=%ld pg_milli=%ld invert=%u drift_milli=%ld drift_tau_ms=%ld drift_max_cdeg=%ld trim_rate_milli=%ld",
       BalanceManager::AlgorithmName(c.algorithm),
       (unsigned int)g_balance_manager.enabled(),
       (long)(c.target_angle_deg * 100.0f), (long)(c.max_output_percent * 100.0f),
       (long)(c.boost_start_deg * 100.0f), (long)(c.boost_max_output_percent * 100.0f),
       (long)(c.tilt_cutoff_deg * 100.0f), (long)(c.tilt_recover_deg * 100.0f),
       (long)(c.pid.kp * 1000.0f), (long)(c.pid.ki * 1000.0f),
       (long)(c.pid.kd * 1000.0f),
       (long)(c.state_feedback.angle_gain * 1000.0f),
       (long)(c.state_feedback.angular_rate_gain * 1000.0f),
       (long)(c.state_feedback.speed_gain * 1000.0f),
       (long)(c.state_feedback.position_gain * 1000.0f),
       (unsigned int)c.invert_output,
       (long)(c.drift_gain_deg_per_percent * 1000.0f),
       (long)(c.drift_tau_s * 1000.0f),
       (long)(c.drift_max_deg * 100.0f),
       (long)(c.drift_trim_rate * 1000.0f));
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
  /* CFG (min_start/min_run/slew) duoc phep khi dang balance de tune live. */
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

  /* Khi balance dang duoc yeu cau (ke ca luc cho xe dung len de tu enable),
     main loop se dung motor moi chu ky nen lenh tay khong co tac dung. */
  if (g_balance_enable_requested != 0U)
  {
    LOGW("MOTOR", "reject manual motor command while balance requested (BAL ENABLE 0 first)");
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
    AppMain_SetBalanceEnabled((uint8_t)((value != NULL && atoi(value) != 0) ? 1U : 0U));
    LOGI("BALCMD", "requested=%u enabled=%u", (unsigned int)g_balance_enable_requested,
         (unsigned int)g_balance_manager.enabled());
    BalanceLogConfig();
    return;
  }

  if (strcmp(token, "DEFAULT") == 0)
  {
    g_balance_manager.setConfig(BalanceManager::DefaultConfig());
    BalanceLogConfig();
    return;
  }

  if (strcmp(token, "IMU") == 0)
  {
    float new_offset = g_roll_offset_deg;
    uint8_t changed = 0U;
    while ((token = strtok(NULL, " \t\r\n")) != NULL)
    {
      float value = 0.0f;
      if (strcmp(token, "ZERO") == 0)
      {
        /* Goc hien tai (chua bu) -> 0: giu xe thang dung theo thuoc thuy. */
        new_offset = -(kBalanceAngleSign * g_imu.filtered_roll_deg);
        changed = 1U;
      }
      else if (strncmp(token, "offset=", 7) == 0)
      {
        if (CommandValue(token, &value) == 0U)
        {
          LOGW("BALCMD", "reject imu offset");
          return;
        }
        new_offset = value;
        changed = 1U;
      }
    }
    if (!(fabsf(new_offset) <= kRollOffsetMaxDeg))
    {
      LOGW("BALCMD", "reject imu offset (|offset| <= 15 deg)");
      return;
    }
    if (changed != 0U)
    {
      /* Doi he quy chieu goc: dich target cung luong de diem can bang vat ly
         khong doi (xe khong giat khi dang chay). */
      BalanceConfig config = g_balance_manager.config();
      const float delta = new_offset - g_roll_offset_deg;
      config.target_angle_deg += delta;
      if (BalanceConfigValid(config) != 0U)
      {
        g_balance_manager.setConfig(config);
      }
      g_roll_offset_deg = new_offset;
    }
    LOGI("BALCMD", "imu roll_offset_cdeg=%ld raw_roll_cdeg=%ld angle_cdeg=%ld target_cdeg=%ld",
         (long)(g_roll_offset_deg * 100.0f),
         (long)(kBalanceAngleSign * g_imu.filtered_roll_deg * 100.0f),
         (long)((kBalanceAngleSign * g_imu.filtered_roll_deg + g_roll_offset_deg) * 100.0f),
         (long)(g_balance_manager.config().target_angle_deg * 100.0f));
    return;
  }

  if (strcmp(token, "ODOM") == 0)
  {
    while ((token = strtok(NULL, " \t\r\n")) != NULL)
    {
      float value = 0.0f;
      if (strcmp(token, "RESET") == 0)
      {
        g_odom.x_m = 0.0f;
        g_odom.y_m = 0.0f;
        g_odom.yaw_deg = 0.0f;
        g_yaw_target_deg = 0.0f;
      }
      else if (strncmp(token, "k=", 2) == 0)
      {
        if ((CommandValue(token, &value) == 0U) || (value <= 0.0f) || (value > 0.1f))
        {
          LOGW("BALCMD", "reject odom k (0 < k <= 0.1 m/s per %%)");
          return;
        }
        g_odom_mps_per_percent = value;
      }
    }
    LOGI("BALCMD", "odom k_umps=%ld x_mm=%ld y_mm=%ld yaw_cdeg=%ld",
         (long)(g_odom_mps_per_percent * 1000000.0f), (long)(g_odom.x_m * 1000.0f),
         (long)(g_odom.y_m * 1000.0f), (long)(g_odom.yaw_deg * 100.0f));
    return;
  }

  if (strcmp(token, "YAW") == 0)
  {
    float kp = g_yaw_kp;
    float kd = g_yaw_kd;
    float sign = g_yaw_sign;
    uint8_t invalid_value = 0U;
    while ((token = strtok(NULL, " \t\r\n")) != NULL)
    {
      if (strncmp(token, "kp=", 3) == 0) invalid_value |= (uint8_t)(CommandValue(token, &kp) == 0U);
      else if (strncmp(token, "kd=", 3) == 0) invalid_value |= (uint8_t)(CommandValue(token, &kd) == 0U);
      else if (strncmp(token, "sign=", 5) == 0) invalid_value |= (uint8_t)(CommandValue(token, &sign) == 0U);
      else if (strcmp(token, "HOLD") == 0) g_yaw_target_deg = g_odom.yaw_deg;
    }
    if ((invalid_value != 0U) || !(kp >= 0.0f && kp <= 5.0f) || !(kd >= 0.0f && kd <= 2.0f))
    {
      LOGW("BALCMD", "reject yaw config (0<=kp<=5, 0<=kd<=2)");
      return;
    }
    g_yaw_kp = kp;
    g_yaw_kd = kd;
    g_yaw_sign = (sign < 0.0f) ? -1.0f : 1.0f;
    g_yaw_target_deg = g_odom.yaw_deg;
    LOGI("BALCMD", "yaw kp_milli=%ld kd_milli=%ld sign=%d target_cdeg=%ld",
         (long)(g_yaw_kp * 1000.0f), (long)(g_yaw_kd * 1000.0f), (int)g_yaw_sign,
         (long)(g_yaw_target_deg * 100.0f));
    return;
  }

  if (strcmp(token, "LOG") == 0)
  {
    char *value = strtok(NULL, " \t\r\n");
    float ms = 0.0f;
    if ((value == NULL) || (ParseFloat(value, &ms) == 0U) ||
        ((ms != 0.0f) && ((ms < 4.0f) || (ms > 1000.0f))))
    {
      LOGW("BALCMD", "usage: BAL LOG <0|4..1000 ms>");
      return;
    }
    g_fast_log_ms = (uint16_t)ms;
    LOGI("BALCMD", "fast_log_ms=%u", (unsigned int)g_fast_log_ms);
    return;
  }

  if (strcmp(token, "SAVE") == 0)
  {
    if (g_balance_manager.enabled() != 0U)
    {
      /* Xoa flash lam CPU dung ~1-2 s, xe se nga neu dang chay. */
      LOGW("BALSTORE", "reject save while balancing (BAL ENABLE 0 first)");
      return;
    }
    /* Gop diem can bang da hoc vao target de lan bat sau khong phai hoc lai. */
    BalanceConfig save_config = g_balance_manager.config();
    const float learned_trim = g_balance_manager.trimDeg();
    save_config.target_angle_deg += learned_trim;
    if (BalanceConfigValid(save_config) == 0U)
    {
      save_config = g_balance_manager.config();
    }
    else
    {
      g_balance_manager.setConfig(save_config);
      g_balance_manager.setTrimDeg(0.0f);
      LOGI("BALSTORE", "folded trim_cdeg=%ld into target_cdeg=%ld",
           (long)(learned_trim * 100.0f), (long)(save_config.target_angle_deg * 100.0f));
    }
    const HAL_StatusTypeDef status = BalancePersistSave(save_config, g_motor_controller);
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
  /* APPLY duoc phep khi dang balance de tune live; config da duoc validate
     day du truoc khi ap dung. */
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
    else if (strncmp(token, "ag=", 3) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.state_feedback.angle_gain) == 0U);
    else if (strncmp(token, "rg=", 3) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.state_feedback.angular_rate_gain) == 0U);
    else if (strncmp(token, "sg=", 3) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.state_feedback.speed_gain) == 0U);
    else if (strncmp(token, "pg=", 3) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.state_feedback.position_gain) == 0U);
    else if (strncmp(token, "drift=", 6) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.drift_gain_deg_per_percent) == 0U);
    else if (strncmp(token, "drift_tau=", 10) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.drift_tau_s) == 0U);
    else if (strncmp(token, "drift_max=", 10) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.drift_max_deg) == 0U);
    else if (strncmp(token, "trim_rate=", 10) == 0) invalid_value |= (uint8_t)(CommandValue(token, &config.drift_trim_rate) == 0U);
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
      (BalanceConfigValid(config) == 0U))
  {
    LOGW("BALCMD", "reject invalid config");
    return;
  }
  g_balance_manager.setConfig(config);
  LOGI("BALCMD", "config applied in RAM");
  BalanceLogConfig();
}

static HAL_StatusTypeDef InitImu()
{
  /* Thu dia chi da dung truoc do truoc, roi moi thu dia chi con lai. */
  const uint16_t first = (g_imu.address == ICM20948_ADDRESS_AD0_HIGH) ?
                         ICM20948_ADDRESS_AD0_HIGH : ICM20948_ADDRESS_AD0_LOW;
  const uint16_t second = (first == ICM20948_ADDRESS_AD0_LOW) ?
                          ICM20948_ADDRESS_AD0_HIGH : ICM20948_ADDRESS_AD0_LOW;
  HAL_StatusTypeDef status = ICM20948_Init(&g_imu, &hi2c1, first);
  if (status != HAL_OK)
  {
    status = ICM20948_Init(&g_imu, &hi2c1, second);
  }
  return status;
}

/* Loi tu HAL (timeout/NACK/bus error) co the de slave giu SDA hoac de co
   BUSY bi ket; neu khong giai phong, lan doc sau se cho BUSY toi 25 ms. Loi
   "du lieu dung" / "chip reset" khong phai loi bus nen khong can. */
static void RecoverBusAfterReadError()
{
  if ((g_imu.last_status == ICM20948_STATUS_STUCK) ||
      (g_imu.last_status == ICM20948_STATUS_NOT_ALIVE))
  {
    return;
  }
  (void)I2C1_BusRecover();
  g_imu_bus_recoveries++;
}

/* Chi chay khi balance dang duoc yeu cau (motor da bi stale-timeout cat) de
   khong pha lenh motor tay khi test khong co IMU. Init IMU block ~100 ms. */
static void TryRecoverImu(uint32_t now_ms)
{
  if ((g_balance_enable_requested == 0U) ||
      (g_imu_fail_count < kImuFailuresBeforeRecovery) ||
      ((now_ms - g_last_imu_recovery_ms) < kImuRecoveryIntervalMs))
  {
    return;
  }
  g_last_imu_recovery_ms = now_ms;

  float gyro_bias[3];
  memcpy(gyro_bias, g_imu.gyro_bias_dps, sizeof(gyro_bias));
  MotorController_Stop(&g_motor_controller);
  (void)I2C1_BusRecover();
  g_imu_reinits++;
  if (InitImu() == HAL_OK)
  {
    /* Init() xoa struct; giu bias cu vi xe co the dang chuyen dong, khong
       calibrate lai duoc. */
    memcpy(g_imu.gyro_bias_dps, gyro_bias, sizeof(gyro_bias));
    g_imu_fail_count = 0U;
    LOGI("IMU", "recovered after I2C bus reset");
  }
  else
  {
    g_imu.initialized = 0U;
    LOGW("IMU", "recovery failed st=%u i2c_err=0x%08lx",
         (unsigned int)g_imu.last_status, (unsigned long)g_imu.last_i2c_error);
  }
}

static void GyroCalibReset()
{
  g_gyro_calib.count = 0U;
}

static void GyroCalibFeed()
{
  for (uint8_t axis = 0U; axis < 3U; axis++)
  {
    const float raw_dps = g_imu.gyro_dps[axis] + g_imu.gyro_bias_dps[axis];
    if (g_gyro_calib.count == 0U)
    {
      g_gyro_calib.sum[axis] = 0.0f;
      g_gyro_calib.min_dps[axis] = raw_dps;
      g_gyro_calib.max_dps[axis] = raw_dps;
    }
    if (raw_dps < g_gyro_calib.min_dps[axis]) g_gyro_calib.min_dps[axis] = raw_dps;
    if (raw_dps > g_gyro_calib.max_dps[axis]) g_gyro_calib.max_dps[axis] = raw_dps;
    g_gyro_calib.sum[axis] += raw_dps;
    if ((g_gyro_calib.max_dps[axis] - g_gyro_calib.min_dps[axis]) > kGyroCalibMaxSpreadDps)
    {
      /* Xe dang bi cam/lac: bat dau lai cua so. */
      GyroCalibReset();
      return;
    }
  }

  g_gyro_calib.count++;
  if (g_gyro_calib.count < kGyroCalibSamples) return;

  for (uint8_t axis = 0U; axis < 3U; axis++)
  {
    g_imu.gyro_bias_dps[axis] = g_gyro_calib.sum[axis] / (float)g_gyro_calib.count;
  }
  GyroCalibReset();
  UartLogger::instance().logRateLimited(LogLevel::Info, "IMU", 0xB1A5U, 10000U,
                                        "gyro bias updated bx_cdps=%ld by_cdps=%ld bz_cdps=%ld",
                                        (long)(g_imu.gyro_bias_dps[0] * 100.0f),
                                        (long)(g_imu.gyro_bias_dps[1] * 100.0f),
                                        (long)(g_imu.gyro_bias_dps[2] * 100.0f));
}

static void ProcessUartCommands()
{
  if (g_uart_command_ready == 0U) return;
  /* g_uart_command_buffer chi duoc ghi trong main loop (PollUartCommandInput),
     khong dung chung voi ISR nen khong can tat IRQ. */
  char command[sizeof(g_uart_command_buffer)];
  memcpy(command, g_uart_command_buffer, sizeof(command));
  g_uart_command_ready = 0U;
  ProcessBalanceCommand(command);
}

static void PollUartCommandInput()
{
  const uint32_t dropped = g_uart_rx_dropped;
  if (dropped != g_uart_rx_dropped_reported)
  {
    UartLogger::instance().logRateLimited(LogLevel::Warn, "UART", 0x0A11DU, 1000U,
                                          "rx ring overflow dropped=%lu",
                                          (unsigned long)dropped);
    g_uart_rx_dropped_reported = dropped;
  }

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
  g_uart_rx_dropped_reported = 0U;
  g_imu_fail_count = 0U;
  g_last_imu_recovery_ms = 0U;
  g_last_imu_health_ms = 0U;
  g_imu_read_errors = 0U;
  g_imu_bus_recoveries = 0U;
  g_imu_reinits = 0U;
  g_fast_log_ms = 0U;
  g_odom = Odometry{};
  /* Dau +1 da kiem chung tren xe: xe tu xoay -4.8 deg/s -> +0.15 deg/s. */
  g_yaw_kp = 0.4f;
  g_yaw_kd = 0.04f;
  g_yaw_target_deg = 0.0f;
  g_yaw_was_active = 0U;
  GyroCalibReset();
  WheelEncoder_Init(&g_left_encoder, kEncoderCountsPerRev, kWheelCircumferenceM);
  WheelEncoder_Init(&g_right_encoder, kEncoderCountsPerRev, kWheelCircumferenceM);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  const HAL_StatusTypeDef imu_status = InitImu();
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
    HAL_StatusTypeDef calib_status = HAL_BUSY;
    for (uint8_t attempt = 0U; (attempt < 3U) && (calib_status == HAL_BUSY); ++attempt)
    {
      calib_status = ICM20948_CalibrateGyro(&g_imu, 250U, 2U);
    }
    if (calib_status == HAL_OK)
    {
      LOGI("IMU", "gyro bias calibrated bx_cdps=%ld by_cdps=%ld bz_cdps=%ld",
           (long)(g_imu.gyro_bias_dps[0] * 100.0f),
           (long)(g_imu.gyro_bias_dps[1] * 100.0f),
           (long)(g_imu.gyro_bias_dps[2] * 100.0f));
    }
    else
    {
      /* Bias se duoc hoc lai tu dong khi xe nam yen (GyroCalibFeed). */
      LOGW("IMU", "gyro calibration failed st=%u (car moving?); will retry when still",
           (unsigned int)calib_status);
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
    /* Cong don chu ky de khong troi pha; neu tre qua mot chu ky (vd flash
       save, khoi phuc IMU) thi dong bo lai thay vi chay bu lien tiep. */
    g_next_imu_sample_ms += kImuSamplePeriodMs;
    if ((int32_t)(now_ms - g_next_imu_sample_ms) >= 0)
    {
      g_next_imu_sample_ms = now_ms + kImuSamplePeriodMs;
    }

    HAL_StatusTypeDef imu_read_status = ICM20948_Read(&g_imu);
    if ((imu_read_status == HAL_OK) &&
        ((now_ms - g_last_imu_health_ms) >= kImuHealthCheckMs))
    {
      /* Chip bi sut ap/reset se ve che do ngu: I2C van OK nhung du lieu sai,
         nen kiem tra dinh ky va coi nhu loi doc de kich hoat init lai. */
      g_last_imu_health_ms = now_ms;
      if (ICM20948_CheckAlive(&g_imu) != HAL_OK)
      {
        imu_read_status = HAL_ERROR;
        if (g_imu.last_status == ICM20948_STATUS_NOT_ALIVE)
        {
          g_imu_fail_count = kImuFailuresBeforeRecovery;
        }
      }
    }

    if (imu_read_status == HAL_OK)
    {
      g_imu_fail_count = 0U;
      const uint32_t previous_sample_ms = g_last_imu_sample_ms;
      g_last_imu_sample_ms = now_ms;
      g_balance_input.timestamp_ms = now_ms;
      g_balance_input.dt_s = (float)(now_ms - previous_sample_ms) / 1000.0f;
      g_balance_input.imu_valid = 1U;
      const float accel_norm_g = sqrtf((g_imu.accel_g[0] * g_imu.accel_g[0]) +
                                       (g_imu.accel_g[1] * g_imu.accel_g[1]) +
                                       (g_imu.accel_g[2] * g_imu.accel_g[2]));
      const float gyro_weight = (fabsf(accel_norm_g - 1.0f) > kAccelTrustBandG) ?
                                1.0f : kComplementaryGyroWeight;
      (void)ICM20948_UpdateComplementaryFilter(&g_imu, g_balance_input.dt_s, gyro_weight);
      /* Roll theo quy ước cơ khí của xe dùng filtered_roll; tốc độ góc tương
         ứng vẫn là gyro-X. */
      g_balance_input.angle_deg = kBalanceAngleSign * g_imu.filtered_roll_deg + g_roll_offset_deg;
      g_balance_input.angular_rate_dps = kBalanceGyroSign * g_imu.gyro_dps[0];
      g_balance_input.left_speed = WheelEncoder_GetSpeed(&g_left_encoder);
      g_balance_input.right_speed = WheelEncoder_GetSpeed(&g_right_encoder);
      g_balance_input.left_position = WheelEncoder_GetPosition(&g_left_encoder);
      g_balance_input.right_position = WheelEncoder_GetPosition(&g_right_encoder);
      g_balance_input.battery_voltage = 0.0f;
      g_balance_input.motor_command_percent =
        0.5f * (float)(g_motor_controller.applied_left_percent +
                       g_motor_controller.applied_right_percent);

      /* Tự kích hoạt chỉ khi xe đã gần thẳng đứng (trong tilt_recover_deg),
         cùng ngưỡng với lúc nhả tilt latch, để motor không bật mạnh ngay khi
         nhấc xe từ tư thế nằm. Khi góc vượt cutoff, BalanceManager sẽ latch
         fault và phía dưới dừng motor. */
      if (g_balance_enable_requested != 0U && g_balance_manager.enabled() == 0U &&
          fabsf(g_balance_manager.config().target_angle_deg - g_balance_input.angle_deg) <=
                g_balance_manager.config().tilt_recover_deg)
      {
        g_balance_manager.setEnabled(1U);
      }

      const BalanceOutput balance_output = g_balance_manager.update(g_balance_input);
      SetStatusLedForBalance(balance_output,
                              balance_output.left_motor_percent,
                              balance_output.right_motor_percent);
      if ((balance_output.active == 0U) &&
          (g_motor_controller.applied_left_percent == 0) &&
          (g_motor_controller.applied_right_percent == 0))
      {
        GyroCalibFeed();
      }
      else
      {
        GyroCalibReset();
      }

      if (g_fast_log_ms != 0U)
      {
        logger.logRateLimited(LogLevel::Info, "T", 0x7E1E0U, g_fast_log_ms,
                              "a=%ld g=%ld o=%ld l=%d r=%d f=%lx s=%ld",
                              (long)(balance_output.angle_error_deg * 100.0f),
                              (long)(g_balance_input.angular_rate_dps * 100.0f),
                              (long)(balance_output.correction_percent * 100.0f),
                              (int)g_motor_controller.applied_left_percent,
                              (int)g_motor_controller.applied_right_percent,
                              (unsigned long)balance_output.faults,
                              (long)(balance_output.target_shift_deg * 100.0f));
      }

      /* Odometry: chi tich phan khi xe dang tu can bang (dung tren banh). */
      const float yaw_rate_dps = g_imu.gyro_dps[2];
      if (balance_output.active != 0U)
      {
        const float dt = g_balance_input.dt_s;
        const float yaw_rad = g_odom.yaw_deg * 0.0174532925f;
        g_odom.yaw_deg += yaw_rate_dps * dt;
        g_odom.speed_mps = g_odom_mps_per_percent * g_balance_input.motor_command_percent;
        g_odom.x_m += g_odom.speed_mps * cosf(yaw_rad) * dt;
        g_odom.y_m += g_odom.speed_mps * sinf(yaw_rad) * dt;
      }
      else
      {
        g_odom.speed_mps = 0.0f;
      }

      if (balance_output.active != 0U)
      {
        float yaw_diff = 0.0f;
        if (g_yaw_was_active == 0U)
        {
          /* Vua dung len: giu huong hien tai, khong quay ve huong cu. */
          g_yaw_target_deg = g_odom.yaw_deg;
        }
        if ((g_yaw_kp > 0.0f) || (g_yaw_kd > 0.0f))
        {
          const float yaw_error = g_yaw_target_deg - g_odom.yaw_deg;
          if (fabsf(yaw_error) > kYawRunawayDeg)
          {
            g_yaw_kp = 0.0f;
            g_yaw_kd = 0.0f;
            LOGW("YAW", "yaw hold disabled: error %ld cdeg (wrong sign? try BAL YAW sign=-1)",
                 (long)(yaw_error * 100.0f));
          }
          else
          {
            yaw_diff = g_yaw_sign * ((g_yaw_kp * yaw_error) - (g_yaw_kd * yaw_rate_dps));
            if (yaw_diff > kYawMaxDiffPercent) yaw_diff = kYawMaxDiffPercent;
            if (yaw_diff < -kYawMaxDiffPercent) yaw_diff = -kYawMaxDiffPercent;
          }
        }
        /* yaw_diff > 0: banh phai nhanh hon -> xe quay CCW (yaw tang) neu
           chieu +PWM la chieu tien; nguoc lai dung sign=-1. */
        MotorController_SetDriveMode(&g_motor_controller, MOTOR_DRIVE_MODE_CUSTOM);
        MotorController_Set(&g_motor_controller,
                            (int16_t)((float)balance_output.left_motor_percent - yaw_diff),
                            (int16_t)((float)balance_output.right_motor_percent + yaw_diff));
      }
      else if (g_balance_enable_requested != 0U)
      {
        /* Fault, tilt limit hoặc IMU chưa sẵn sàng phải cắt lệnh ngay,
           không chờ command watchdog. Khi balance bị tắt hẳn (BAL ENABLE 0)
           thì để yên lệnh motor tay. */
        MotorController_Stop(&g_motor_controller);
      }
      g_yaw_was_active = balance_output.active;

      logger.logRateLimited(
        LogLevel::Info,
        "IMU",
        0x20948U,
        200U,
        "roll_cdeg=%ld pitch_cdeg=%ld gyroX_cdps=%ld gyroY_cdps=%ld gyroZ_cdps=%ld ax_mg=%ld ay_mg=%ld az_mg=%ld",
         (long)(g_balance_input.angle_deg * 100.0f),
        (long)(g_imu.pitch_deg * 100.0f),
        (long)(g_imu.gyro_dps[0] * 100.0f),
        (long)(g_imu.gyro_dps[1] * 100.0f),
        (long)(g_imu.gyro_dps[2] * 100.0f),
        (long)(g_imu.accel_g[0] * 1000.0f),
        (long)(g_imu.accel_g[1] * 1000.0f),
        (long)(g_imu.accel_g[2] * 1000.0f));

      logger.logRateLimited(LogLevel::Info, "ODO", 0x0D0E0U, 100U,
                            "x_mm=%ld y_mm=%ld yaw_cdeg=%ld v_mmps=%ld yawrate_cdps=%ld",
                            (long)(g_odom.x_m * 1000.0f), (long)(g_odom.y_m * 1000.0f),
                            (long)(g_odom.yaw_deg * 100.0f), (long)(g_odom.speed_mps * 1000.0f),
                            (long)(yaw_rate_dps * 100.0f));

      /* BAL telemetry cần luôn có ở mức Info để không phụ thuộc cấu hình
         logger Debug của bản firmware đang chạy. */
      logger.logRateLimited(
        LogLevel::Info,
        "BAL",
        0xBA1A1U,
        200U,
        "alg=%s en=%u err_cdeg=%ld out_cpercent=%ld pwmL_percent=%ld pwmR_percent=%ld fault=0x%08lx trim_cdeg=%ld shift_cdeg=%ld",
        BalanceManager::AlgorithmName(balance_output.algorithm),
        (unsigned int)g_balance_manager.enabled(),
        (long)(balance_output.angle_error_deg * 100.0f),
        (long)(balance_output.correction_percent * 100.0f),
        (long)g_motor_controller.applied_left_percent,
        (long)g_motor_controller.applied_right_percent,
        (unsigned long)balance_output.faults,
        (long)(balance_output.trim_deg * 100.0f),
        (long)(balance_output.target_shift_deg * 100.0f));
    }
    else
    {
      /* Motor khong bi cat ngay o day: mot lan loi le te se do stale timeout
         (20 ms) xu ly, tranh brake giat 1 chu ky. Timeout I2C ngan (3 ms)
         nen loop khong bi treo lau voi PWM cu. */
      if (g_imu_fail_count < 255U) g_imu_fail_count++;
      g_imu_read_errors++;
      SetLedMode(LED_MODE_BLINK_FAST);
      /* st: 1=HAL_ERROR 2=BUSY 3=TIMEOUT 0x10=du lieu dung 0x11=chip reset.
         i2c_err: 0x1 BERR 0x2 ARLO 0x4 AF(NACK) 0x8 OVR 0x20 TIMEOUT. */
      logger.logRateLimited(LogLevel::Error, "IMU", 0x20949U, 500U,
                            "ICM-20948 read failed st=0x%02x i2c_err=0x%08lx streak=%u errors=%lu bus_resets=%lu reinits=%lu",
                            (unsigned int)g_imu.last_status,
                            (unsigned long)g_imu.last_i2c_error,
                            (unsigned int)g_imu_fail_count,
                            (unsigned long)g_imu_read_errors,
                            (unsigned long)g_imu_bus_recoveries,
                            (unsigned long)g_imu_reinits);
      RecoverBusAfterReadError();
      TryRecoverImu(now_ms);
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
    SetLedMode(LED_MODE_BLINK_FAST);
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
  /* Chi ghi nhan yeu cau; AppMain_Loop se enable that khi xe da gan thang
     dung, de khong bo qua nguong an toan tilt_recover_deg. */
  g_balance_enable_requested = (enabled != 0U) ? 1U : 0U;
  if (g_balance_enable_requested == 0U)
  {
    g_balance_manager.setEnabled(0U);
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
