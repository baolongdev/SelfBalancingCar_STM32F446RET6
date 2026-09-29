/*
 * Non-blocking motion profile sample
 * Profile: ramp up -> run -> brake hold -> stop
 * Copy into Core/Src/main.c USER CODE regions.
 */

/* USER CODE BEGIN Includes */
#include "modules/control/motor.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
static MotorController_t motor_controller;
static uint32_t profile_restart_tick = 0U;
static uint8_t profile_wait_restart = 0U;
/* USER CODE END PV */

/* USER CODE BEGIN 2 */
MotorController_InitDefault(&motor_controller);
MotorController_SetInversion(&motor_controller, 0U, 0U);
MotorController_SetMaxPwm(&motor_controller, 0U); /* 0 => auto from ARR */
if (MotorController_Start(&motor_controller) != HAL_OK)
{
  Error_Handler();
}
MotorController_ProfileStart(&motor_controller, 50U, 1500U, 2000U, 250U);
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
MotorController_Update(&motor_controller);

if (MotorController_ProfileIsBusy(&motor_controller) == 0U)
{
  if (profile_wait_restart == 0U)
  {
    profile_wait_restart = 1U;
    profile_restart_tick = HAL_GetTick();
  }
  else if ((HAL_GetTick() - profile_restart_tick) >= 1000U)
  {
    MotorController_ProfileStart(&motor_controller, 50U, 1500U, 2000U, 250U);
    profile_wait_restart = 0U;
  }
}
/* USER CODE END 3 */

