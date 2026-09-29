/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.c
  * @brief   Motor controller implementation for differential-drive motors.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "modules/control/motor.h"

#define MOTOR_PERCENT_MIN   (-100)
#define MOTOR_PERCENT_MAX   (100)

typedef enum
{
  MOTOR_SIDE_MODE_NORMAL = 0,
  MOTOR_SIDE_MODE_COAST,
  MOTOR_SIDE_MODE_BRAKE
} MotorSideMode_t;

static int16_t Motor_ClampPercent(int16_t percent)
{
  if (percent > MOTOR_PERCENT_MAX)
  {
    return MOTOR_PERCENT_MAX;
  }
  if (percent < MOTOR_PERCENT_MIN)
  {
    return MOTOR_PERCENT_MIN;
  }
  return percent;
}

static uint8_t Motor_ClampPercentU8(uint8_t percent)
{
  if (percent > 100U)
  {
    return 100U;
  }
  return percent;
}

static int16_t Motor_AbsInt16(int16_t value)
{
  return (value < 0) ? (int16_t)(-value) : value;
}

static int8_t Motor_Sign(int16_t value)
{
  if (value > 0)
  {
    return 1;
  }
  if (value < 0)
  {
    return -1;
  }
  return 0;
}

static void Motor_WriteDir(const Motor_t *motor, int8_t direction)
{
  uint8_t forward = (direction > 0) ? 1U : 0U;

  if (motor->invert_direction != 0U)
  {
    forward = (uint8_t)!forward;
  }

  if (direction == 0)
  {
    HAL_GPIO_WritePin(motor->r_en_port, motor->r_en_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->l_en_port, motor->l_en_pin, GPIO_PIN_RESET);
    return;
  }

  HAL_GPIO_WritePin(motor->r_en_port, motor->r_en_pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(motor->l_en_port, motor->l_en_pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void Motor_CoastSingle(const Motor_t *motor)
{
  __HAL_TIM_SET_COMPARE(motor->htim, motor->pwm_channel, 0U);
  Motor_WriteDir(motor, 0);
}

static void Motor_BrakeSingle(const Motor_t *motor)
{
  __HAL_TIM_SET_COMPARE(motor->htim, motor->pwm_channel, 0U);
  HAL_GPIO_WritePin(motor->r_en_port, motor->r_en_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor->l_en_port, motor->l_en_pin, GPIO_PIN_SET);
}

static uint32_t Motor_GetEffectivePwmMax(const MotorController_t *controller, const Motor_t *motor)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(motor->htim);

  if (controller->pwm_max == 0U)
  {
    return arr;
  }
  if (controller->pwm_max > arr)
  {
    return arr;
  }
  return controller->pwm_max;
}

static uint32_t Motor_GetCompScalePermille(const MotorController_t *controller)
{
  uint32_t mv;
  uint32_t scale;

  if ((controller->battery_comp.enabled == 0U) ||
      (controller->battery_mv == 0U) ||
      (controller->battery_comp.nominal_mv == 0U))
  {
    return 1000U;
  }

  mv = controller->battery_mv;
  if ((controller->battery_comp.min_mv > 0U) && (mv < controller->battery_comp.min_mv))
  {
    mv = controller->battery_comp.min_mv;
  }
  if ((controller->battery_comp.max_mv > 0U) && (mv > controller->battery_comp.max_mv))
  {
    mv = controller->battery_comp.max_mv;
  }

  if (mv == 0U)
  {
    return 1000U;
  }

  scale = ((uint32_t)controller->battery_comp.nominal_mv * 1000U) / mv;

  if ((controller->battery_comp.min_scale_permille > 0U) &&
      (scale < controller->battery_comp.min_scale_permille))
  {
    scale = controller->battery_comp.min_scale_permille;
  }
  if ((controller->battery_comp.max_scale_permille > 0U) &&
      (scale > controller->battery_comp.max_scale_permille))
  {
    scale = controller->battery_comp.max_scale_permille;
  }

  return scale;
}

static uint32_t Motor_PercentToPwm(const MotorController_t *controller, const Motor_t *motor, uint8_t percent_abs)
{
  uint32_t pwm_max = Motor_GetEffectivePwmMax(controller, motor);
  uint32_t base = (pwm_max * percent_abs) / 100U;
  uint32_t scale = Motor_GetCompScalePermille(controller);
  uint32_t compensated = (base * scale) / 1000U;

  if (compensated > pwm_max)
  {
    compensated = pwm_max;
  }
  return compensated;
}

static void Motor_SetOutputImmediate(const MotorController_t *controller, const Motor_t *motor, int16_t percent)
{
  int16_t limited = Motor_ClampPercent(percent);
  uint8_t abs_percent = (uint8_t)Motor_AbsInt16(limited);
  uint32_t pwm;

  if (limited == 0)
  {
    Motor_CoastSingle(motor);
    return;
  }

  Motor_WriteDir(motor, Motor_Sign(limited));
  pwm = Motor_PercentToPwm(controller, motor, abs_percent);
  __HAL_TIM_SET_COMPARE(motor->htim, motor->pwm_channel, pwm);
}

static const MotorSlewConfig_t *Motor_GetSlewConfig(const MotorController_t *controller)
{
  if (controller->drive_mode == MOTOR_DRIVE_MODE_ATTACK)
  {
    return &controller->attack_slew;
  }
  if (controller->drive_mode == MOTOR_DRIVE_MODE_CUSTOM)
  {
    return &controller->custom_slew;
  }
  return &controller->search_slew;
}

static int16_t Motor_SlewStep(int16_t current, int16_t target, uint16_t accel_up_per_s, uint16_t accel_down_per_s, uint32_t dt_ms)
{
  int16_t delta = target - current;
  uint16_t rate;
  uint32_t step;
  int16_t abs_current = Motor_AbsInt16(current);
  int16_t abs_target = Motor_AbsInt16(target);

  if ((delta == 0) || (dt_ms == 0U))
  {
    return current;
  }

  rate = (abs_target > abs_current) ? accel_up_per_s : accel_down_per_s;
  if (rate == 0U)
  {
    return target;
  }

  step = ((uint32_t)rate * dt_ms) / 1000U;
  if (step == 0U)
  {
    step = 1U;
  }

  if (delta > 0)
  {
    if ((uint32_t)delta > step)
    {
      return (int16_t)(current + (int16_t)step);
    }
  }
  else
  {
    if ((uint32_t)(-delta) > step)
    {
      return (int16_t)(current - (int16_t)step);
    }
  }
  return target;
}

static int16_t Motor_ApplyMinPercent(const MotorController_t *controller, int16_t command_percent, uint8_t was_running)
{
  int16_t limited = Motor_ClampPercent(command_percent);
  int16_t abs_value;
  uint8_t threshold;

  if (limited == 0)
  {
    return 0;
  }

  threshold = was_running ? controller->min_run_percent : controller->min_start_percent;
  if (threshold == 0U)
  {
    return limited;
  }

  abs_value = Motor_AbsInt16(limited);
  if (abs_value < (int16_t)threshold)
  {
    abs_value = threshold;
  }

  return (limited > 0) ? abs_value : (int16_t)(-abs_value);
}

static MotorSideMode_t Motor_CheckReverseTransition(MotorController_t *controller,
                                                    const Motor_t *motor,
                                                    int16_t applied_percent,
                                                    int16_t requested_percent,
                                                    uint32_t *unlock_tick,
                                                    uint32_t *brake_until_tick,
                                                    uint32_t now,
                                                    int16_t *safe_command)
{
  int8_t applied_sign;
  int8_t request_sign;

  *safe_command = requested_percent;
  if ((requested_percent == 0) || (applied_percent == 0))
  {
    return MOTOR_SIDE_MODE_NORMAL;
  }

  applied_sign = Motor_Sign(applied_percent);
  request_sign = Motor_Sign(requested_percent);
  if (applied_sign == request_sign)
  {
    return MOTOR_SIDE_MODE_NORMAL;
  }

  if ((controller->reverse_deadtime_ms == 0U) && (controller->reverse_brake_ms == 0U))
  {
    return MOTOR_SIDE_MODE_NORMAL;
  }

  if ((now >= *unlock_tick) || (*unlock_tick == 0U))
  {
    /* Khi đổi dấu lệnh, motor không được đảo chiều tức thì.
       Tùy cấu hình sẽ có một pha brake/coast ngắn để bảo vệ cầu H và cơ khí. */
    *unlock_tick = now + controller->reverse_deadtime_ms;
    *brake_until_tick = now + controller->reverse_brake_ms;
  }

  *safe_command = 0;
  if (now < *unlock_tick)
  {
    if ((controller->reverse_brake_ms > 0U) && (now < *brake_until_tick))
    {
      Motor_BrakeSingle(motor);
      return MOTOR_SIDE_MODE_BRAKE;
    }
    Motor_CoastSingle(motor);
    return MOTOR_SIDE_MODE_COAST;
  }

  return MOTOR_SIDE_MODE_NORMAL;
}

static void Motor_ClearProfileState(MotorController_t *controller)
{
  controller->profile.active = 0U;
  controller->profile.state = MOTOR_PROFILE_STATE_IDLE;
}

static void Motor_ProcessProfile(MotorController_t *controller, uint32_t now, uint8_t *brake_override)
{
  uint32_t elapsed;
  uint32_t speed;

  *brake_override = 0U;
  if (controller->profile.active == 0U)
  {
    return;
  }

  elapsed = now - controller->profile.state_tick;
  controller->last_command_tick = now;

  switch (controller->profile.state)
  {
    case MOTOR_PROFILE_STATE_RAMP_UP:
      if (controller->profile.ramp_up_ms == 0U)
      {
        speed = controller->profile.target_percent;
      }
      else if (elapsed >= controller->profile.ramp_up_ms)
      {
        speed = controller->profile.target_percent;
      }
      else
      {
        speed = ((uint32_t)controller->profile.target_percent * elapsed) / controller->profile.ramp_up_ms;
      }

      controller->target_left_percent = (int16_t)speed;
      controller->target_right_percent = (int16_t)speed;

      if ((controller->profile.ramp_up_ms == 0U) || (elapsed >= controller->profile.ramp_up_ms))
      {
        controller->profile.state_tick = now;
        if (controller->profile.run_ms > 0U)
        {
          controller->profile.state = MOTOR_PROFILE_STATE_CRUISE;
        }
        else if (controller->profile.brake_ms > 0U)
        {
          controller->profile.state = MOTOR_PROFILE_STATE_BRAKE_HOLD;
        }
        else
        {
          controller->target_left_percent = 0;
          controller->target_right_percent = 0;
          Motor_ClearProfileState(controller);
        }
      }
      break;

    case MOTOR_PROFILE_STATE_CRUISE:
      controller->target_left_percent = (int16_t)controller->profile.target_percent;
      controller->target_right_percent = (int16_t)controller->profile.target_percent;
      if (elapsed >= controller->profile.run_ms)
      {
        controller->profile.state_tick = now;
        if (controller->profile.brake_ms > 0U)
        {
          controller->profile.state = MOTOR_PROFILE_STATE_BRAKE_HOLD;
        }
        else
        {
          controller->target_left_percent = 0;
          controller->target_right_percent = 0;
          Motor_ClearProfileState(controller);
        }
      }
      break;

    case MOTOR_PROFILE_STATE_BRAKE_HOLD:
      *brake_override = 1U;
      if (elapsed >= controller->profile.brake_ms)
      {
        controller->target_left_percent = 0;
        controller->target_right_percent = 0;
        Motor_ClearProfileState(controller);
      }
      break;

    case MOTOR_PROFILE_STATE_IDLE:
    default:
      Motor_ClearProfileState(controller);
      break;
  }
}

static void Motor_ApplyClosedLoop(MotorController_t *controller, int16_t *left, int16_t *right, uint32_t dt_ms)
{
  float dt_s;
  float err;
  float correction;
  float limit;

  if (controller->closed_loop.enabled == 0U)
  {
    controller->closed_loop.integral_left = 0.0f;
    controller->closed_loop.integral_right = 0.0f;
    return;
  }

  if (dt_ms == 0U)
  {
    dt_ms = 1U;
  }
  dt_s = (float)dt_ms / 1000.0f;
  limit = controller->closed_loop.integral_limit;

  err = (float)(*left - controller->closed_loop.feedback_left_percent);
  controller->closed_loop.integral_left += err * dt_s;
  if (controller->closed_loop.integral_left > limit)
  {
    controller->closed_loop.integral_left = limit;
  }
  if (controller->closed_loop.integral_left < -limit)
  {
    controller->closed_loop.integral_left = -limit;
  }
  correction = (controller->closed_loop.kp * err) + (controller->closed_loop.ki * controller->closed_loop.integral_left);
  *left = Motor_ClampPercent((int16_t)(*left + (int16_t)correction));

  err = (float)(*right - controller->closed_loop.feedback_right_percent);
  controller->closed_loop.integral_right += err * dt_s;
  if (controller->closed_loop.integral_right > limit)
  {
    controller->closed_loop.integral_right = limit;
  }
  if (controller->closed_loop.integral_right < -limit)
  {
    controller->closed_loop.integral_right = -limit;
  }
  correction = (controller->closed_loop.kp * err) + (controller->closed_loop.ki * controller->closed_loop.integral_right);
  *right = Motor_ClampPercent((int16_t)(*right + (int16_t)correction));
}

static void Motor_ProcessWatchdog(MotorController_t *controller, uint32_t now)
{
  if (controller->command_timeout_ms == 0U)
  {
    return;
  }

  if ((controller->target_left_percent == 0) &&
      (controller->target_right_percent == 0) &&
      (controller->profile.active == 0U))
  {
    return;
  }

  if ((now - controller->last_command_tick) > controller->command_timeout_ms)
  {
    /* Nếu app ngừng cập nhật lệnh quá lâu, watchdog sẽ kéo target về 0 và dừng an toàn. */
    controller->fault_flags |= MOTOR_FAULT_WATCHDOG;
    Motor_ClearProfileState(controller);
    controller->target_left_percent = 0;
    controller->target_right_percent = 0;
    MotorController_Stop(controller);
  }
}

static void Motor_ProcessStallSide(MotorController_t *controller,
                                   int16_t applied_percent,
                                   int16_t feedback_percent,
                                   uint16_t current_ma,
                                   uint32_t *stall_tick,
                                   uint32_t fault_bit,
                                   uint32_t now)
{
  uint8_t stall_condition;

  if (controller->stall.enabled == 0U)
  {
    *stall_tick = 0U;
    return;
  }

  stall_condition = (uint8_t)(
    (Motor_AbsInt16(applied_percent) >= (int16_t)controller->stall.pwm_threshold_percent) &&
    (Motor_AbsInt16(feedback_percent) <= (int16_t)controller->stall.speed_threshold_percent) &&
    (current_ma >= controller->stall.current_threshold_ma)
  );

  /* Stall chỉ được xác nhận khi đồng thời:
     PWM đang lớn, tốc độ phản hồi thấp và dòng kéo cao trong đủ thời gian detect_ms. */
  if (stall_condition == 0U)
  {
    *stall_tick = 0U;
    return;
  }

  if (*stall_tick == 0U)
  {
    *stall_tick = now;
    return;
  }

  if ((now - *stall_tick) >= controller->stall.detect_ms)
  {
    controller->fault_flags |= fault_bit;
    if (controller->stall.auto_stop_on_stall != 0U)
    {
      Motor_ClearProfileState(controller);
      controller->target_left_percent = 0;
      controller->target_right_percent = 0;
      MotorController_Stop(controller);
    }
  }
}

void MotorController_InitDefault(MotorController_t *controller)
{
  if (controller == NULL)
  {
    return;
  }

  controller->left.r_en_port = MOTOR_LEFT_R_EN_GPIO_Port;
  controller->left.r_en_pin = MOTOR_LEFT_R_EN_Pin;
  controller->left.l_en_port = MOTOR_LEFT_L_EN_GPIO_Port;
  controller->left.l_en_pin = MOTOR_LEFT_L_EN_Pin;
  controller->left.htim = &htim2;
  controller->left.pwm_channel = TIM_CHANNEL_2;
  controller->left.invert_direction = 0U;

  controller->right.r_en_port = MOTOR_RIGHT_R_EN_GPIO_Port;
  controller->right.r_en_pin = MOTOR_RIGHT_R_EN_Pin;
  controller->right.l_en_port = MOTOR_RIGHT_L_EN_GPIO_Port;
  controller->right.l_en_pin = MOTOR_RIGHT_L_EN_Pin;
  controller->right.htim = &htim2;
  controller->right.pwm_channel = TIM_CHANNEL_3;
  controller->right.invert_direction = 0U;

  controller->pwm_max = 0U;
  controller->stop_mode = MOTOR_STOP_COAST;
  controller->drive_mode = MOTOR_DRIVE_MODE_SEARCH;
  controller->search_slew.accel_up_per_s = 80U;
  controller->search_slew.accel_down_per_s = 120U;
  controller->attack_slew.accel_up_per_s = 260U;
  controller->attack_slew.accel_down_per_s = 350U;
  controller->custom_slew.accel_up_per_s = 120U;
  controller->custom_slew.accel_down_per_s = 160U;
  controller->command_timeout_ms = 400U;
  controller->reverse_deadtime_ms = 80U;
  controller->reverse_brake_ms = 40U;
  controller->min_start_percent = 18U;
  controller->min_run_percent = 10U;

  controller->battery_mv = 0U;
  controller->battery_comp.enabled = 1U;
  controller->battery_comp.nominal_mv = 7400U;
  controller->battery_comp.min_mv = 6200U;
  controller->battery_comp.max_mv = 8400U;
  controller->battery_comp.min_scale_permille = 900U;
  controller->battery_comp.max_scale_permille = 1300U;

  controller->closed_loop.enabled = 0U;
  controller->closed_loop.kp = 0.0f;
  controller->closed_loop.ki = 0.0f;
  controller->closed_loop.integral_limit = 50.0f;
  controller->closed_loop.integral_left = 0.0f;
  controller->closed_loop.integral_right = 0.0f;
  controller->closed_loop.feedback_left_percent = 0;
  controller->closed_loop.feedback_right_percent = 0;

  controller->stall.enabled = 0U;
  controller->stall.auto_stop_on_stall = 1U;
  controller->stall.pwm_threshold_percent = 45U;
  controller->stall.speed_threshold_percent = 8U;
  controller->stall.detect_ms = 120U;
  controller->stall.current_threshold_ma = 1800U;
  controller->stall.current_left_ma = 0U;
  controller->stall.current_right_ma = 0U;
  controller->stall.left_stall_tick = 0U;
  controller->stall.right_stall_tick = 0U;

  controller->profile.active = 0U;
  controller->profile.state = MOTOR_PROFILE_STATE_IDLE;
  controller->profile.target_percent = 0U;
  controller->profile.ramp_up_ms = 0U;
  controller->profile.run_ms = 0U;
  controller->profile.brake_ms = 0U;
  controller->profile.state_tick = 0U;

  controller->target_left_percent = 0;
  controller->target_right_percent = 0;
  controller->applied_left_percent = 0;
  controller->applied_right_percent = 0;

  controller->left_reverse_unlock_tick = 0U;
  controller->right_reverse_unlock_tick = 0U;
  controller->left_brake_until_tick = 0U;
  controller->right_brake_until_tick = 0U;
  controller->last_command_tick = HAL_GetTick();
  controller->last_update_tick = HAL_GetTick();
  controller->e_stop_active = 0U;
  controller->fault_flags = MOTOR_FAULT_NONE;
}

HAL_StatusTypeDef MotorController_Start(MotorController_t *controller)
{
  HAL_StatusTypeDef left_status;
  HAL_StatusTypeDef right_status;

  if (controller == NULL)
  {
    return HAL_ERROR;
  }

  left_status = HAL_TIM_PWM_Start(controller->left.htim, controller->left.pwm_channel);
  right_status = HAL_TIM_PWM_Start(controller->right.htim, controller->right.pwm_channel);
  MotorController_Stop(controller);

  if ((left_status != HAL_OK) || (right_status != HAL_OK))
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

void MotorController_Update(MotorController_t *controller)
{
  uint32_t now;
  uint32_t dt_ms;
  uint8_t brake_override;
  int16_t desired_left;
  int16_t desired_right;
  int16_t safe_left;
  int16_t safe_right;
  int16_t next_left;
  int16_t next_right;
  MotorSideMode_t left_mode;
  MotorSideMode_t right_mode;
  const MotorSlewConfig_t *slew;

  if (controller == NULL)
  {
    return;
  }

  now = HAL_GetTick();
  dt_ms = now - controller->last_update_tick;
  if (dt_ms == 0U)
  {
    dt_ms = 1U;
  }

  if (controller->e_stop_active != 0U)
  {
    MotorController_Brake(controller);
    controller->last_update_tick = now;
    return;
  }

  Motor_ProcessProfile(controller, now, &brake_override);
  if (brake_override != 0U)
  {
    MotorController_Brake(controller);
    controller->last_update_tick = now;
    return;
  }

  Motor_ProcessWatchdog(controller, now);

  desired_left = controller->target_left_percent;
  desired_right = controller->target_right_percent;
  /* Thứ tự xử lý chính:
     target -> closed loop -> chặn đảo chiều -> slew/min-percent -> xuất PWM thật. */
  Motor_ApplyClosedLoop(controller, &desired_left, &desired_right, dt_ms);

  left_mode = Motor_CheckReverseTransition(controller, &controller->left,
                                           controller->applied_left_percent, desired_left,
                                           &controller->left_reverse_unlock_tick,
                                           &controller->left_brake_until_tick,
                                           now, &safe_left);
  right_mode = Motor_CheckReverseTransition(controller, &controller->right,
                                            controller->applied_right_percent, desired_right,
                                            &controller->right_reverse_unlock_tick,
                                            &controller->right_brake_until_tick,
                                            now, &safe_right);

  slew = Motor_GetSlewConfig(controller);

  if (left_mode == MOTOR_SIDE_MODE_NORMAL)
  {
    safe_left = Motor_ApplyMinPercent(controller, safe_left, (controller->applied_left_percent != 0));
    next_left = Motor_SlewStep(controller->applied_left_percent, safe_left,
                               slew->accel_up_per_s, slew->accel_down_per_s, dt_ms);
    if (safe_left != 0)
    {
      next_left = Motor_ApplyMinPercent(controller, next_left, (controller->applied_left_percent != 0));
    }
    Motor_SetOutputImmediate(controller, &controller->left, next_left);
    controller->applied_left_percent = next_left;
  }
  else
  {
    controller->applied_left_percent = 0;
  }

  if (right_mode == MOTOR_SIDE_MODE_NORMAL)
  {
    safe_right = Motor_ApplyMinPercent(controller, safe_right, (controller->applied_right_percent != 0));
    next_right = Motor_SlewStep(controller->applied_right_percent, safe_right,
                                slew->accel_up_per_s, slew->accel_down_per_s, dt_ms);
    if (safe_right != 0)
    {
      next_right = Motor_ApplyMinPercent(controller, next_right, (controller->applied_right_percent != 0));
    }
    Motor_SetOutputImmediate(controller, &controller->right, next_right);
    controller->applied_right_percent = next_right;
  }
  else
  {
    controller->applied_right_percent = 0;
  }

  Motor_ProcessStallSide(controller,
                         controller->applied_left_percent,
                         controller->closed_loop.feedback_left_percent,
                         controller->stall.current_left_ma,
                         &controller->stall.left_stall_tick,
                         MOTOR_FAULT_STALL_LEFT,
                         now);
  Motor_ProcessStallSide(controller,
                         controller->applied_right_percent,
                         controller->closed_loop.feedback_right_percent,
                         controller->stall.current_right_ma,
                         &controller->stall.right_stall_tick,
                         MOTOR_FAULT_STALL_RIGHT,
                         now);

  controller->last_update_tick = now;
}

void MotorController_Set(MotorController_t *controller, int16_t left_percent, int16_t right_percent)
{
  if (controller == NULL)
  {
    return;
  }

  Motor_ClearProfileState(controller);
  controller->target_left_percent = Motor_ClampPercent(left_percent);
  controller->target_right_percent = Motor_ClampPercent(right_percent);
  controller->last_command_tick = HAL_GetTick();
}

void MotorController_Forward(MotorController_t *controller, uint8_t percent)
{
  int16_t speed = Motor_ClampPercent((int16_t)percent);
  MotorController_Set(controller, speed, speed);
}

void MotorController_Backward(MotorController_t *controller, uint8_t percent)
{
  int16_t speed = Motor_ClampPercent((int16_t)percent);
  MotorController_Set(controller, (int16_t)-speed, (int16_t)-speed);
}

void MotorController_TurnLeft(MotorController_t *controller, uint8_t percent)
{
  int16_t speed = Motor_ClampPercent((int16_t)percent);
  MotorController_Set(controller, (int16_t)-speed, speed);
}

void MotorController_TurnRight(MotorController_t *controller, uint8_t percent)
{
  int16_t speed = Motor_ClampPercent((int16_t)percent);
  MotorController_Set(controller, speed, (int16_t)-speed);
}

void MotorController_Charge(MotorController_t *controller, uint8_t percent)
{
  MotorController_SetDriveMode(controller, MOTOR_DRIVE_MODE_ATTACK);
  MotorController_Forward(controller, percent);
}

void MotorController_Retreat(MotorController_t *controller, uint8_t percent)
{
  MotorController_SetDriveMode(controller, MOTOR_DRIVE_MODE_ATTACK);
  MotorController_Backward(controller, percent);
}

void MotorController_PivotLeft(MotorController_t *controller, uint8_t percent)
{
  MotorController_SetDriveMode(controller, MOTOR_DRIVE_MODE_ATTACK);
  MotorController_TurnLeft(controller, percent);
}

void MotorController_PivotRight(MotorController_t *controller, uint8_t percent)
{
  MotorController_SetDriveMode(controller, MOTOR_DRIVE_MODE_ATTACK);
  MotorController_TurnRight(controller, percent);
}

void MotorController_Coast(MotorController_t *controller)
{
  if (controller == NULL)
  {
    return;
  }

  Motor_ClearProfileState(controller);
  controller->target_left_percent = 0;
  controller->target_right_percent = 0;
  controller->applied_left_percent = 0;
  controller->applied_right_percent = 0;
  Motor_CoastSingle(&controller->left);
  Motor_CoastSingle(&controller->right);
  controller->last_command_tick = HAL_GetTick();
}

void MotorController_Brake(MotorController_t *controller)
{
  if (controller == NULL)
  {
    return;
  }

  Motor_ClearProfileState(controller);
  controller->target_left_percent = 0;
  controller->target_right_percent = 0;
  controller->applied_left_percent = 0;
  controller->applied_right_percent = 0;
  Motor_BrakeSingle(&controller->left);
  Motor_BrakeSingle(&controller->right);
  controller->last_command_tick = HAL_GetTick();
}

void MotorController_Stop(MotorController_t *controller)
{
  if (controller == NULL)
  {
    return;
  }

  if (controller->stop_mode == MOTOR_STOP_BRAKE)
  {
    MotorController_Brake(controller);
  }
  else
  {
    MotorController_Coast(controller);
  }
}

void MotorController_SetStopMode(MotorController_t *controller, MotorStopMode_t mode)
{
  if (controller == NULL)
  {
    return;
  }
  controller->stop_mode = mode;
}

void MotorController_SetMaxPwm(MotorController_t *controller, uint32_t pwm_max)
{
  if (controller == NULL)
  {
    return;
  }
  controller->pwm_max = pwm_max;
}

void MotorController_SetInversion(MotorController_t *controller, uint8_t left_invert, uint8_t right_invert)
{
  if (controller == NULL)
  {
    return;
  }
  controller->left.invert_direction = (left_invert != 0U) ? 1U : 0U;
  controller->right.invert_direction = (right_invert != 0U) ? 1U : 0U;
}

void MotorController_SetDriveMode(MotorController_t *controller, MotorDriveMode_t mode)
{
  if (controller == NULL)
  {
    return;
  }
  controller->drive_mode = mode;
}

void MotorController_SetCustomSlew(MotorController_t *controller, uint16_t accel_up_per_s, uint16_t accel_down_per_s)
{
  if (controller == NULL)
  {
    return;
  }
  controller->custom_slew.accel_up_per_s = accel_up_per_s;
  controller->custom_slew.accel_down_per_s = accel_down_per_s;
}

void MotorController_SetDirectionSafety(MotorController_t *controller, uint32_t reverse_deadtime_ms, uint32_t reverse_brake_ms)
{
  if (controller == NULL)
  {
    return;
  }
  controller->reverse_deadtime_ms = reverse_deadtime_ms;
  controller->reverse_brake_ms = reverse_brake_ms;
}

void MotorController_SetMinDrivePercent(MotorController_t *controller, uint8_t min_start_percent, uint8_t min_run_percent)
{
  if (controller == NULL)
  {
    return;
  }
  controller->min_start_percent = Motor_ClampPercentU8(min_start_percent);
  controller->min_run_percent = Motor_ClampPercentU8(min_run_percent);
}

void MotorController_SetCommandTimeout(MotorController_t *controller, uint32_t timeout_ms)
{
  if (controller == NULL)
  {
    return;
  }
  controller->command_timeout_ms = timeout_ms;
}

void MotorController_SetBatteryCompConfig(MotorController_t *controller, const MotorBatteryCompConfig_t *config)
{
  if ((controller == NULL) || (config == NULL))
  {
    return;
  }
  controller->battery_comp = *config;
}

void MotorController_UpdateBatteryMv(MotorController_t *controller, uint16_t battery_mv)
{
  if (controller == NULL)
  {
    return;
  }
  controller->battery_mv = battery_mv;
}

void MotorController_SetClosedLoopConfig(MotorController_t *controller, const MotorClosedLoopConfig_t *config)
{
  if ((controller == NULL) || (config == NULL))
  {
    return;
  }
  controller->closed_loop.enabled = config->enabled;
  controller->closed_loop.kp = config->kp;
  controller->closed_loop.ki = config->ki;
  controller->closed_loop.integral_limit = config->integral_limit;
  controller->closed_loop.integral_left = 0.0f;
  controller->closed_loop.integral_right = 0.0f;
}

void MotorController_UpdateSpeedFeedback(MotorController_t *controller, int16_t left_speed_percent, int16_t right_speed_percent)
{
  if (controller == NULL)
  {
    return;
  }
  controller->closed_loop.feedback_left_percent = Motor_ClampPercent(left_speed_percent);
  controller->closed_loop.feedback_right_percent = Motor_ClampPercent(right_speed_percent);
}

void MotorController_SetStallConfig(MotorController_t *controller, const MotorStallConfig_t *config)
{
  if ((controller == NULL) || (config == NULL))
  {
    return;
  }
  controller->stall.enabled = config->enabled;
  controller->stall.auto_stop_on_stall = config->auto_stop_on_stall;
  controller->stall.pwm_threshold_percent = Motor_ClampPercentU8(config->pwm_threshold_percent);
  controller->stall.speed_threshold_percent = Motor_ClampPercentU8(config->speed_threshold_percent);
  controller->stall.detect_ms = config->detect_ms;
  controller->stall.current_threshold_ma = config->current_threshold_ma;
  controller->stall.left_stall_tick = 0U;
  controller->stall.right_stall_tick = 0U;
}

void MotorController_UpdateCurrentFeedback(MotorController_t *controller, uint16_t left_current_ma, uint16_t right_current_ma)
{
  if (controller == NULL)
  {
    return;
  }
  controller->stall.current_left_ma = left_current_ma;
  controller->stall.current_right_ma = right_current_ma;
}

void MotorController_ProfileStart(MotorController_t *controller, uint8_t target_percent, uint32_t ramp_up_ms, uint32_t run_ms, uint32_t brake_ms)
{
  uint32_t now;

  if (controller == NULL)
  {
    return;
  }

  now = HAL_GetTick();
  controller->profile.active = 1U;
  controller->profile.state = MOTOR_PROFILE_STATE_RAMP_UP;
  controller->profile.target_percent = Motor_ClampPercentU8(target_percent);
  controller->profile.ramp_up_ms = ramp_up_ms;
  controller->profile.run_ms = run_ms;
  controller->profile.brake_ms = brake_ms;
  controller->profile.state_tick = now;
  controller->last_command_tick = now;

  if (controller->profile.target_percent == 0U)
  {
    Motor_ClearProfileState(controller);
    MotorController_Stop(controller);
  }
}

void MotorController_ProfileCancel(MotorController_t *controller)
{
  if (controller == NULL)
  {
    return;
  }
  Motor_ClearProfileState(controller);
  controller->target_left_percent = 0;
  controller->target_right_percent = 0;
}

uint8_t MotorController_ProfileIsBusy(const MotorController_t *controller)
{
  if (controller == NULL)
  {
    return 0U;
  }
  return controller->profile.active;
}

void MotorController_EmergencyStop(MotorController_t *controller)
{
  if (controller == NULL)
  {
    return;
  }

  controller->e_stop_active = 1U;
  controller->fault_flags |= MOTOR_FAULT_ESTOP;
  MotorController_Brake(controller);
}

void MotorController_EmergencyRelease(MotorController_t *controller)
{
  if (controller == NULL)
  {
    return;
  }

  controller->e_stop_active = 0U;
  MotorController_Coast(controller);
}

uint8_t MotorController_IsEmergencyStopped(const MotorController_t *controller)
{
  if (controller == NULL)
  {
    return 0U;
  }
  return controller->e_stop_active;
}

uint32_t MotorController_GetFaults(const MotorController_t *controller)
{
  if (controller == NULL)
  {
    return MOTOR_FAULT_NONE;
  }
  return controller->fault_flags;
}

void MotorController_ClearFaults(MotorController_t *controller, uint32_t fault_mask)
{
  if (controller == NULL)
  {
    return;
  }
  controller->fault_flags &= ~fault_mask;
}

