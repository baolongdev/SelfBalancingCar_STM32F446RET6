#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  int32_t count;
  int32_t previous_count;
  float counts_per_rev;
  float wheel_circumference_m;
  float position_m;
  float speed_mps;
  uint32_t last_update_ms;
  uint8_t configured;
} WheelEncoder_t;

void WheelEncoder_Init(WheelEncoder_t *encoder, float counts_per_rev,
                       float wheel_circumference_m);
void WheelEncoder_UpdateCount(WheelEncoder_t *encoder, int32_t count,
                              uint32_t timestamp_ms);
float WheelEncoder_GetSpeed(const WheelEncoder_t *encoder);
float WheelEncoder_GetPosition(const WheelEncoder_t *encoder);

#ifdef __cplusplus
}
#endif

#endif
