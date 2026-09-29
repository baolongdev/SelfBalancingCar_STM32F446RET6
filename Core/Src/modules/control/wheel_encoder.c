#include "modules/control/wheel_encoder.h"

void WheelEncoder_Init(WheelEncoder_t *encoder, float counts_per_rev,
                       float wheel_circumference_m)
{
  if (encoder == 0) return;
  encoder->count = 0;
  encoder->previous_count = 0;
  encoder->counts_per_rev = counts_per_rev;
  encoder->wheel_circumference_m = wheel_circumference_m;
  encoder->position_m = 0.0f;
  encoder->speed_mps = 0.0f;
  encoder->last_update_ms = 0U;
  encoder->configured = (uint8_t)(counts_per_rev > 0.0f && wheel_circumference_m > 0.0f);
}

void WheelEncoder_UpdateCount(WheelEncoder_t *encoder, int32_t count,
                              uint32_t timestamp_ms)
{
  if ((encoder == 0) || (encoder->configured == 0U)) return;
  if (encoder->last_update_ms != 0U && timestamp_ms > encoder->last_update_ms)
  {
    const float dt_s = (float)(timestamp_ms - encoder->last_update_ms) / 1000.0f;
    const int32_t delta = count - encoder->count;
    const float meters_per_count = encoder->wheel_circumference_m / encoder->counts_per_rev;
    encoder->speed_mps = ((float)delta * meters_per_count) / dt_s;
  }
  encoder->previous_count = encoder->count;
  encoder->count = count;
  encoder->position_m = (float)count * encoder->wheel_circumference_m /
                        encoder->counts_per_rev;
  encoder->last_update_ms = timestamp_ms;
}

float WheelEncoder_GetSpeed(const WheelEncoder_t *encoder)
{
  return (encoder != 0) ? encoder->speed_mps : 0.0f;
}

float WheelEncoder_GetPosition(const WheelEncoder_t *encoder)
{
  return (encoder != 0) ? encoder->position_m : 0.0f;
}
