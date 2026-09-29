#ifndef ICM20948_H
#define ICM20948_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ICM20948_ADDRESS_AD0_LOW   (0x68U << 1)
#define ICM20948_ADDRESS_AD0_HIGH  (0x69U << 1)
#define ICM20948_WHO_AM_I_VALUE    0xEAU

typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint16_t address;
  int16_t accel_raw[3];
  int16_t gyro_raw[3];
  float accel_g[3];
  float gyro_dps[3];
  float gyro_bias_dps[3];
  float roll_deg;
  float pitch_deg;
  float filtered_roll_deg;
  float filtered_pitch_deg;
  uint8_t filter_initialized;
  uint8_t initialized;
  uint8_t last_status;
  uint32_t last_i2c_error;
} ICM20948_t;

HAL_StatusTypeDef ICM20948_Init(ICM20948_t *imu, I2C_HandleTypeDef *hi2c, uint16_t address);
HAL_StatusTypeDef ICM20948_Read(ICM20948_t *imu);
HAL_StatusTypeDef ICM20948_CalibrateGyro(ICM20948_t *imu, uint16_t samples, uint32_t sample_delay_ms);
HAL_StatusTypeDef ICM20948_UpdateComplementaryFilter(ICM20948_t *imu, float dt_s, float gyro_weight);
uint8_t ICM20948_IsReady(ICM20948_t *imu);

#ifdef __cplusplus
}
#endif

#endif /* ICM20948_H */
