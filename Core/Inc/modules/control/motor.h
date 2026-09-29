/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.h
  * @brief   Motor controller interface for differential-drive motors.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"
#include <stdint.h>

typedef struct
{
  /* Một motor gồm 2 chân điều hướng và 1 kênh PWM điều tốc. */
  GPIO_TypeDef *r_en_port;
  uint16_t r_en_pin;
  GPIO_TypeDef *l_en_port;
  uint16_t l_en_pin;
  TIM_HandleTypeDef *htim;
  uint32_t pwm_channel;
  uint8_t invert_direction;
} Motor_t;

typedef enum
{
  MOTOR_STOP_COAST = 0,
  MOTOR_STOP_BRAKE
} MotorStopMode_t;

typedef enum
{
  MOTOR_DRIVE_MODE_SEARCH = 0,
  MOTOR_DRIVE_MODE_ATTACK,
  MOTOR_DRIVE_MODE_CUSTOM
} MotorDriveMode_t;

typedef enum
{
  MOTOR_PROFILE_STATE_IDLE = 0,
  MOTOR_PROFILE_STATE_RAMP_UP,
  MOTOR_PROFILE_STATE_CRUISE,
  MOTOR_PROFILE_STATE_BRAKE_HOLD
} MotorProfileState_t;

typedef enum
{
  MOTOR_FAULT_NONE        = 0U,
  MOTOR_FAULT_ESTOP       = (1U << 0),
  MOTOR_FAULT_WATCHDOG    = (1U << 1),
  MOTOR_FAULT_STALL_LEFT  = (1U << 2),
  MOTOR_FAULT_STALL_RIGHT = (1U << 3)
} MotorFaultFlags_t;

typedef struct
{
  uint16_t accel_up_per_s;
  uint16_t accel_down_per_s;
} MotorSlewConfig_t;

typedef struct
{
  uint8_t enabled;
  uint16_t nominal_mv;
  uint16_t min_mv;
  uint16_t max_mv;
  uint16_t min_scale_permille;
  uint16_t max_scale_permille;
} MotorBatteryCompConfig_t;

typedef struct
{
  uint8_t enabled;
  float kp;
  float ki;
  float integral_limit;
  float integral_left;
  float integral_right;
  int16_t feedback_left_percent;
  int16_t feedback_right_percent;
} MotorClosedLoopConfig_t;

typedef struct
{
  uint8_t enabled;
  uint8_t auto_stop_on_stall;
  uint8_t pwm_threshold_percent;
  uint8_t speed_threshold_percent;
  uint32_t detect_ms;
  uint16_t current_threshold_ma;
  uint16_t current_left_ma;
  uint16_t current_right_ma;
  uint32_t left_stall_tick;
  uint32_t right_stall_tick;
} MotorStallConfig_t;

typedef struct
{
  MotorProfileState_t state;
  uint8_t active;
  uint8_t target_percent;
  uint32_t ramp_up_ms;
  uint32_t run_ms;
  uint32_t brake_ms;
  uint32_t state_tick;
} MotorMotionProfile_t;

typedef struct
{
  Motor_t left;
  Motor_t right;
  uint32_t pwm_max;

  /* Mỗi drive mode có profile tăng/giảm tốc riêng để xe phản ứng khác nhau
     giữa lúc tìm kiếm, tấn công và chạy lệnh custom. */
  MotorStopMode_t stop_mode;
  MotorDriveMode_t drive_mode;
  MotorSlewConfig_t search_slew;
  MotorSlewConfig_t attack_slew;
  MotorSlewConfig_t custom_slew;

  uint32_t command_timeout_ms;
  uint32_t reverse_deadtime_ms;
  uint32_t reverse_brake_ms;
  uint8_t min_start_percent;
  uint8_t min_run_percent;

  uint16_t battery_mv;
  MotorBatteryCompConfig_t battery_comp;
  MotorClosedLoopConfig_t closed_loop;
  MotorStallConfig_t stall;
  MotorMotionProfile_t profile;

  int16_t target_left_percent;
  int16_t target_right_percent;
  int16_t applied_left_percent;
  int16_t applied_right_percent;

  uint32_t left_reverse_unlock_tick;
  uint32_t right_reverse_unlock_tick;
  uint32_t left_brake_until_tick;
  uint32_t right_brake_until_tick;
  uint32_t last_command_tick;
  uint32_t last_update_tick;

  uint8_t e_stop_active;
  uint32_t fault_flags;
} MotorController_t;

/* API điều khiển mức cao cho app.
   App chỉ cần set target, còn update() sẽ xử lý slew, đảo chiều an toàn, watchdog và stall. */
void MotorController_InitDefault(MotorController_t *controller);
HAL_StatusTypeDef MotorController_Start(MotorController_t *controller);
void MotorController_Update(MotorController_t *controller);

void MotorController_Set(MotorController_t *controller, int16_t left_percent, int16_t right_percent);
void MotorController_Forward(MotorController_t *controller, uint8_t percent);
void MotorController_Backward(MotorController_t *controller, uint8_t percent);
void MotorController_TurnLeft(MotorController_t *controller, uint8_t percent);
void MotorController_TurnRight(MotorController_t *controller, uint8_t percent);
void MotorController_Charge(MotorController_t *controller, uint8_t percent);
void MotorController_Retreat(MotorController_t *controller, uint8_t percent);
void MotorController_PivotLeft(MotorController_t *controller, uint8_t percent);
void MotorController_PivotRight(MotorController_t *controller, uint8_t percent);

void MotorController_Stop(MotorController_t *controller);
void MotorController_Coast(MotorController_t *controller);
void MotorController_Brake(MotorController_t *controller);
void MotorController_SetStopMode(MotorController_t *controller, MotorStopMode_t mode);

void MotorController_SetMaxPwm(MotorController_t *controller, uint32_t pwm_max);
void MotorController_SetInversion(MotorController_t *controller, uint8_t left_invert, uint8_t right_invert);
void MotorController_SetDriveMode(MotorController_t *controller, MotorDriveMode_t mode);
void MotorController_SetCustomSlew(MotorController_t *controller, uint16_t accel_up_per_s, uint16_t accel_down_per_s);
void MotorController_SetDirectionSafety(MotorController_t *controller, uint32_t reverse_deadtime_ms, uint32_t reverse_brake_ms);
void MotorController_SetMinDrivePercent(MotorController_t *controller, uint8_t min_start_percent, uint8_t min_run_percent);
void MotorController_SetCommandTimeout(MotorController_t *controller, uint32_t timeout_ms);

void MotorController_SetBatteryCompConfig(MotorController_t *controller, const MotorBatteryCompConfig_t *config);
void MotorController_UpdateBatteryMv(MotorController_t *controller, uint16_t battery_mv);
void MotorController_SetClosedLoopConfig(MotorController_t *controller, const MotorClosedLoopConfig_t *config);
void MotorController_UpdateSpeedFeedback(MotorController_t *controller, int16_t left_speed_percent, int16_t right_speed_percent);
void MotorController_SetStallConfig(MotorController_t *controller, const MotorStallConfig_t *config);
void MotorController_UpdateCurrentFeedback(MotorController_t *controller, uint16_t left_current_ma, uint16_t right_current_ma);

void MotorController_ProfileStart(MotorController_t *controller, uint8_t target_percent, uint32_t ramp_up_ms, uint32_t run_ms, uint32_t brake_ms);
void MotorController_ProfileCancel(MotorController_t *controller);
uint8_t MotorController_ProfileIsBusy(const MotorController_t *controller);

void MotorController_EmergencyStop(MotorController_t *controller);
void MotorController_EmergencyRelease(MotorController_t *controller);
uint8_t MotorController_IsEmergencyStopped(const MotorController_t *controller);

uint32_t MotorController_GetFaults(const MotorController_t *controller);
void MotorController_ClearFaults(MotorController_t *controller, uint32_t fault_mask);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
