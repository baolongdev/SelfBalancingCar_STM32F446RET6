#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* API điều khiển motor mức cao cho xe. */
typedef enum
{
  APP_DRIVE_STOP = 0,
  APP_DRIVE_FORWARD,
  APP_DRIVE_BACKWARD,
  APP_DRIVE_TURN_LEFT,
  APP_DRIVE_TURN_RIGHT
} AppDriveCommand_t;

void AppMain_Init(void);
void AppMain_Loop(void);

/* Nhóm hàm manual phục vụ kiểm tra motor/debug.
   Khi balance manager được enable, output cân bằng sẽ được ưu tiên. */
void AppMain_SetDriveCommand(AppDriveCommand_t command, uint8_t speed_percent);
void AppMain_Forward(uint8_t speed_percent);
void AppMain_Backward(uint8_t speed_percent);
void AppMain_TurnLeft(uint8_t speed_percent);
void AppMain_TurnRight(uint8_t speed_percent);
void AppMain_Stop(void);
void AppMain_SetBalanceEnabled(uint8_t enabled);
void AppMain_SetBalanceAlgorithm(uint8_t algorithm);
void AppMain_UartRxByteFromISR(uint8_t byte);
void AppMain_UpdateWheelEncoderTicks(int32_t left_ticks, int32_t right_ticks,
                                     uint32_t timestamp_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_H */
