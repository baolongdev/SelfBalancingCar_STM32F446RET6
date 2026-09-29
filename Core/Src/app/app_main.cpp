#include "app/app_main.h"

#include "i2c.h"
#include "modules/control/led_manager.h"
#include "modules/control/balance_manager.hpp"
#include "modules/control/motor.h"
#include "modules/logging/uart_logger.hpp"
#include "modules/sensors/icm20948.h"
#include "tim.h"
#include "usart.h"

namespace
{
MotorController_t g_motor_controller;
ICM20948_t g_imu;
uint32_t g_next_imu_sample_ms;
uint32_t g_last_imu_sample_ms;
BalanceManager g_balance_manager;
BalanceInput g_balance_input{};
}

extern "C" void AppMain_Init(void)
{
  UartLogger &logger = UartLogger::instance();
  logger.init(&huart1);
  logger.setEnabled(1U);
  logger.setLevel(LogLevel::Debug);

  HAL_StatusTypeDef imu_status = ICM20948_Init(&g_imu, &hi2c1, ICM20948_ADDRESS_AD0_LOW);
  if (imu_status != HAL_OK)
  {
    imu_status = ICM20948_Init(&g_imu, &hi2c1, ICM20948_ADDRESS_AD0_HIGH);
  }
  if (imu_status != HAL_OK)
  {
    LOGE("IMU", "ICM-20948 not detected at 0x68/0x69");
    Error_Handler();
  }
  LOGI("IMU", "ICM-20948 ready");
  g_next_imu_sample_ms = HAL_GetTick();
  g_last_imu_sample_ms = g_next_imu_sample_ms;

  /* Balance control is deliberately disabled until axis direction and offsets
     are verified with the chassis lifted from the ground. */
  g_balance_manager.init(BalanceConfig::DefaultConfig());
  g_balance_manager.setAlgorithm(BalanceAlgorithm::PID);
  g_balance_manager.setEnabled(0U);

  LedManager_Init();
  LedManager_SetMode(LED_MODE_OFF);

  MotorController_InitDefault(&g_motor_controller);
  MotorController_SetStopMode(&g_motor_controller, MOTOR_STOP_BRAKE);
  MotorController_SetDirectionSafety(&g_motor_controller, 80U, 40U);
  MotorController_SetMinDrivePercent(&g_motor_controller, 18U, 10U);
  MotorController_SetCommandTimeout(&g_motor_controller, 600U);

  if (MotorController_Start(&g_motor_controller) != HAL_OK)
  {
    LOGE("INIT", "Motor controller start failed");
    Error_Handler();
  }

  MotorController_Stop(&g_motor_controller);
  LOGI("INIT", "Motor-only control ready");
}

extern "C" void AppMain_Loop(void)
{
  UartLogger &logger = UartLogger::instance();
  uint32_t now_ms = HAL_GetTick();

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
      g_balance_input.angle_deg = g_imu.pitch_deg;
      g_balance_input.angular_rate_dps = g_imu.gyro_dps[1];
      g_balance_input.left_speed = 0.0f;
      g_balance_input.right_speed = 0.0f;
      g_balance_input.left_position = 0.0f;
      g_balance_input.right_position = 0.0f;
      g_balance_input.battery_voltage = 0.0f;

      const BalanceOutput balance_output = g_balance_manager.update(g_balance_input);
      if (balance_output.active != 0U)
      {
        MotorController_SetDriveMode(&g_motor_controller, MOTOR_DRIVE_MODE_CUSTOM);
        MotorController_Set(&g_motor_controller,
                            balance_output.left_motor_percent,
                            balance_output.right_motor_percent);
      }

      logger.logRateLimited(
        LogLevel::Info,
        "IMU",
        0x20948U,
        200U,
        "roll=%.2f pitch=%.2f gyroY=%.2f ax=%.3f ay=%.3f az=%.3f",
        (double)g_imu.roll_deg,
        (double)g_imu.pitch_deg,
        (double)g_imu.gyro_dps[1],
        (double)g_imu.accel_g[0],
        (double)g_imu.accel_g[1],
        (double)g_imu.accel_g[2]);

      logger.logRateLimited(
        LogLevel::Debug,
        "BAL",
        0xBA1A1U,
        200U,
        "alg=%s en=%u err=%.2f out=%.2f fault=0x%08lx",
        BalanceManager::AlgorithmName(balance_output.algorithm),
        (unsigned int)g_balance_manager.enabled(),
        (double)balance_output.angle_error_deg,
        (double)balance_output.correction_percent,
        (unsigned long)balance_output.faults);
    }
    else
    {
      logger.logRateLimited(LogLevel::Error, "IMU", 0x20949U, 500U, "ICM-20948 read failed");
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
  g_balance_manager.setEnabled(enabled);
  if (enabled == 0U)
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
