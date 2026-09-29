/*
 * Safety and fault handling sample
 * Copy into Core/Src/main.c USER CODE regions.
 */

/* USER CODE BEGIN Includes */
#include "modules/control/motor.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
static MotorController_t motor_controller;
static uint32_t last_mode_tick = 0U;
static uint8_t mode = 0U;
/* USER CODE END PV */

/* USER CODE BEGIN 2 */
MotorController_InitDefault(&motor_controller);
MotorController_SetStopMode(&motor_controller, MOTOR_STOP_BRAKE);
MotorController_SetCommandTimeout(&motor_controller, 300U);
MotorController_SetDirectionSafety(&motor_controller, 100U, 50U);
if (MotorController_Start(&motor_controller) != HAL_OK)
{
  Error_Handler();
}
last_mode_tick = HAL_GetTick();
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
uint32_t faults;

/* Emergency button on PB0 (active-low by default in many boards). */
if (HAL_GPIO_ReadPin(BTN_B0_GPIO_Port, BTN_B0_Pin) == GPIO_PIN_RESET)
{
  MotorController_EmergencyStop(&motor_controller);
}
else if (MotorController_IsEmergencyStopped(&motor_controller) != 0U)
{
  MotorController_EmergencyRelease(&motor_controller);
  MotorController_ClearFaults(&motor_controller, MOTOR_FAULT_ESTOP);
}

/* Heartbeat motion command to avoid watchdog timeout. */
if ((HAL_GetTick() - last_mode_tick) >= 250U)
{
  last_mode_tick = HAL_GetTick();
  if (mode == 0U)
  {
    MotorController_Forward(&motor_controller, 35U);
    mode = 1U;
  }
  else
  {
    MotorController_TurnRight(&motor_controller, 25U);
    mode = 0U;
  }
}

MotorController_Update(&motor_controller);

/* Fault indicator */
faults = MotorController_GetFaults(&motor_controller);
if (faults != MOTOR_FAULT_NONE)
{
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);

  /* Example recovery for watchdog fault */
  if ((faults & MOTOR_FAULT_WATCHDOG) != 0U)
  {
    MotorController_ClearFaults(&motor_controller, MOTOR_FAULT_WATCHDOG);
  }
}
else
{
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
}
/* USER CODE END 3 */

