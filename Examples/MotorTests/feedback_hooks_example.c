/*
 * Feedback hooks sample:
 * - battery voltage compensation
 * - speed feedback for closed-loop PI
 * - current feedback for stall detection
 * Copy into Core/Src/main.c USER CODE regions.
 */

/* USER CODE BEGIN Includes */
#include "modules/control/motor.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
static MotorController_t motor_controller;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
/* Replace these with your real sensor/ADC interfaces. */
static uint16_t App_ReadBatteryMv(void) { return 7400U; }
static int16_t App_ReadLeftSpeedPercent(void) { return 0; }
static int16_t App_ReadRightSpeedPercent(void) { return 0; }
static uint16_t App_ReadLeftCurrentMa(void) { return 0U; }
static uint16_t App_ReadRightCurrentMa(void) { return 0U; }
/* USER CODE END 0 */

/* USER CODE BEGIN 2 */
MotorBatteryCompConfig_t bat_cfg;
MotorClosedLoopConfig_t cl_cfg;
MotorStallConfig_t stall_cfg;

MotorController_InitDefault(&motor_controller);

bat_cfg.enabled = 1U;
bat_cfg.nominal_mv = 7400U;
bat_cfg.min_mv = 6200U;
bat_cfg.max_mv = 8400U;
bat_cfg.min_scale_permille = 900U;
bat_cfg.max_scale_permille = 1300U;
MotorController_SetBatteryCompConfig(&motor_controller, &bat_cfg);

cl_cfg.enabled = 1U;
cl_cfg.kp = 0.30f;
cl_cfg.ki = 0.80f;
cl_cfg.integral_limit = 40.0f;
cl_cfg.integral_left = 0.0f;
cl_cfg.integral_right = 0.0f;
cl_cfg.feedback_left_percent = 0;
cl_cfg.feedback_right_percent = 0;
MotorController_SetClosedLoopConfig(&motor_controller, &cl_cfg);

stall_cfg.enabled = 1U;
stall_cfg.auto_stop_on_stall = 1U;
stall_cfg.pwm_threshold_percent = 45U;
stall_cfg.speed_threshold_percent = 8U;
stall_cfg.detect_ms = 120U;
stall_cfg.current_threshold_ma = 1800U;
stall_cfg.current_left_ma = 0U;
stall_cfg.current_right_ma = 0U;
stall_cfg.left_stall_tick = 0U;
stall_cfg.right_stall_tick = 0U;
MotorController_SetStallConfig(&motor_controller, &stall_cfg);

if (MotorController_Start(&motor_controller) != HAL_OK)
{
  Error_Handler();
}

MotorController_Charge(&motor_controller, 70U);
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
MotorController_UpdateBatteryMv(&motor_controller, App_ReadBatteryMv());
MotorController_UpdateSpeedFeedback(&motor_controller, App_ReadLeftSpeedPercent(), App_ReadRightSpeedPercent());
MotorController_UpdateCurrentFeedback(&motor_controller, App_ReadLeftCurrentMa(), App_ReadRightCurrentMa());

MotorController_Update(&motor_controller);
/* USER CODE END 3 */

