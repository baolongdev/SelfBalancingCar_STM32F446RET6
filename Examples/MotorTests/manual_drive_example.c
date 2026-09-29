/*
 * Manual drive test sample
 * Copy into Core/Src/main.c USER CODE regions.
 */

/* USER CODE BEGIN Includes */
#include "modules/control/motor.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
static MotorController_t motor_controller;
static uint32_t phase_tick = 0U;
static uint8_t phase = 0U;
/* USER CODE END PV */

/* USER CODE BEGIN 2 */
MotorController_InitDefault(&motor_controller);
MotorController_SetInversion(&motor_controller, 0U, 0U);
MotorController_SetMaxPwm(&motor_controller, 0U); /* 0 => auto from ARR */
MotorController_SetCommandTimeout(&motor_controller, 0U); /* disable watchdog for this simple test */
if (MotorController_Start(&motor_controller) != HAL_OK)
{
  Error_Handler();
}
MotorController_Stop(&motor_controller);
phase_tick = HAL_GetTick();
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
MotorController_Update(&motor_controller);

switch (phase)
{
  case 0:
    MotorController_Forward(&motor_controller, 40);
    phase_tick = HAL_GetTick();
    phase = 1U;
    break;
  case 1:
    if ((HAL_GetTick() - phase_tick) >= 1200U)
    {
      MotorController_Backward(&motor_controller, 35);
      phase_tick = HAL_GetTick();
      phase = 2U;
    }
    break;
  case 2:
    if ((HAL_GetTick() - phase_tick) >= 1000U)
    {
      MotorController_TurnLeft(&motor_controller, 35);
      phase_tick = HAL_GetTick();
      phase = 3U;
    }
    break;
  case 3:
    if ((HAL_GetTick() - phase_tick) >= 700U)
    {
      MotorController_TurnRight(&motor_controller, 35);
      phase_tick = HAL_GetTick();
      phase = 4U;
    }
    break;
  case 4:
    if ((HAL_GetTick() - phase_tick) >= 700U)
    {
      MotorController_Brake(&motor_controller);
      phase_tick = HAL_GetTick();
      phase = 5U;
    }
    break;
  case 5:
    if ((HAL_GetTick() - phase_tick) >= 250U)
    {
      MotorController_Stop(&motor_controller);
      phase_tick = HAL_GetTick();
      phase = 6U;
    }
    break;
  default:
    if ((HAL_GetTick() - phase_tick) >= 1000U)
    {
      phase = 0U;
    }
    break;
}
/* USER CODE END 3 */

