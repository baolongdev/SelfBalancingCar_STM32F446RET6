#include "modules/sensors/icm20948.h"

#include <math.h>
#include <string.h>

#define ICM20948_REG_BANK_SEL       0x7FU
#define ICM20948_BANK0              0x00U
#define ICM20948_BANK2              0x20U

#define ICM20948_REG_WHO_AM_I      0x00U
#define ICM20948_REG_PWR_MGMT_1    0x06U
#define ICM20948_REG_PWR_MGMT_2    0x07U
#define ICM20948_REG_ACCEL_XOUT_H  0x2DU

#define ICM20948_REG_GYRO_SMPLRT_DIV  0x00U
#define ICM20948_REG_GYRO_CONFIG_1    0x01U
#define ICM20948_REG_ACCEL_SMPLRT_DIV_1 0x10U
#define ICM20948_REG_ACCEL_SMPLRT_DIV_2 0x11U
#define ICM20948_REG_ACCEL_CONFIG      0x14U

#define ICM20948_TIMEOUT_MS         100U
#define ICM20948_ACCEL_LSB_PER_G    16384.0f
#define ICM20948_GYRO_LSB_PER_DPS   131.0f
#define ICM20948_RAD_TO_DEG         57.2957795f

static HAL_StatusTypeDef ICM20948_SelectBank(ICM20948_t *imu, uint8_t bank)
{
  uint8_t value = (uint8_t)(bank & 0x30U);
  return HAL_I2C_Mem_Write(imu->hi2c, imu->address, ICM20948_REG_BANK_SEL,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U, ICM20948_TIMEOUT_MS);
}

static HAL_StatusTypeDef ICM20948_Write(ICM20948_t *imu, uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(imu->hi2c, imu->address, reg, I2C_MEMADD_SIZE_8BIT,
                           &value, 1U, ICM20948_TIMEOUT_MS);
}

static HAL_StatusTypeDef ICM20948_ReadRegisters(ICM20948_t *imu, uint8_t reg,
                                                  uint8_t *data, uint16_t length)
{
  return HAL_I2C_Mem_Read(imu->hi2c, imu->address, reg, I2C_MEMADD_SIZE_8BIT,
                          data, length, ICM20948_TIMEOUT_MS);
}

static HAL_StatusTypeDef ICM20948_InitFailure(ICM20948_t *imu)
{
  imu->last_status = (uint8_t)HAL_ERROR;
  imu->last_i2c_error = (imu->hi2c != NULL) ? HAL_I2C_GetError(imu->hi2c) : HAL_I2C_ERROR_NONE;
  return HAL_ERROR;
}

static int16_t ICM20948_Int16(const uint8_t *data)
{
  return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

HAL_StatusTypeDef ICM20948_Init(ICM20948_t *imu, I2C_HandleTypeDef *hi2c, uint16_t address)
{
  uint8_t who_am_i = 0U;

  if ((imu == NULL) || (hi2c == NULL))
  {
    if (imu != NULL)
    {
      imu->last_status = (uint8_t)HAL_ERROR;
      imu->last_i2c_error = HAL_I2C_ERROR_NONE;
    }
    return HAL_ERROR;
  }

  memset(imu, 0, sizeof(*imu));
  imu->hi2c = hi2c;
  imu->address = address;
  imu->last_status = HAL_OK;
  imu->last_i2c_error = HAL_I2C_ERROR_NONE;

  if (HAL_I2C_IsDeviceReady(imu->hi2c, imu->address, 3U, ICM20948_TIMEOUT_MS) != HAL_OK)
  {
    return ICM20948_InitFailure(imu);
  }

  if (ICM20948_SelectBank(imu, ICM20948_BANK0) != HAL_OK ||
      ICM20948_ReadRegisters(imu, ICM20948_REG_WHO_AM_I, &who_am_i, 1U) != HAL_OK ||
      who_am_i != ICM20948_WHO_AM_I_VALUE)
  {
    return ICM20948_InitFailure(imu);
  }

  if (ICM20948_Write(imu, ICM20948_REG_PWR_MGMT_1, 0x80U) != HAL_OK)
  {
    return ICM20948_InitFailure(imu);
  }
  HAL_Delay(100U);

  if (ICM20948_Write(imu, ICM20948_REG_PWR_MGMT_1, 0x01U) != HAL_OK ||
      ICM20948_Write(imu, ICM20948_REG_PWR_MGMT_2, 0x00U) != HAL_OK ||
      ICM20948_SelectBank(imu, ICM20948_BANK2) != HAL_OK)
  {
    return ICM20948_InitFailure(imu);
  }

  /* Gyro: 250 dps, 1 kHz output; accel: 2 g, 1 kHz output. */
  if (ICM20948_Write(imu, ICM20948_REG_GYRO_SMPLRT_DIV, 0x00U) != HAL_OK ||
      ICM20948_Write(imu, ICM20948_REG_GYRO_CONFIG_1, 0x01U) != HAL_OK ||
      ICM20948_Write(imu, ICM20948_REG_ACCEL_SMPLRT_DIV_1, 0x00U) != HAL_OK ||
      ICM20948_Write(imu, ICM20948_REG_ACCEL_SMPLRT_DIV_2, 0x00U) != HAL_OK ||
      ICM20948_Write(imu, ICM20948_REG_ACCEL_CONFIG, 0x01U) != HAL_OK ||
      ICM20948_SelectBank(imu, ICM20948_BANK0) != HAL_OK)
  {
    return ICM20948_InitFailure(imu);
  }

  imu->initialized = 1U;
  imu->last_status = (uint8_t)HAL_OK;
  imu->last_i2c_error = HAL_I2C_ERROR_NONE;
  return HAL_OK;
}

HAL_StatusTypeDef ICM20948_Read(ICM20948_t *imu)
{
  uint8_t data[14];
  HAL_StatusTypeDef status;

  if ((imu == NULL) || (imu->initialized == 0U))
  {
    if (imu != NULL)
    {
      imu->last_status = (uint8_t)HAL_ERROR;
      imu->last_i2c_error = (imu->hi2c != NULL) ? HAL_I2C_GetError(imu->hi2c) : HAL_I2C_ERROR_NONE;
    }
    return HAL_ERROR;
  }

  status = ICM20948_SelectBank(imu, ICM20948_BANK0);
  if (status != HAL_OK)
  {
    imu->last_status = (uint8_t)status;
    imu->last_i2c_error = HAL_I2C_GetError(imu->hi2c);
    return status;
  }

  status = ICM20948_ReadRegisters(imu, ICM20948_REG_ACCEL_XOUT_H, data, sizeof(data));
  if (status != HAL_OK)
  {
    imu->last_status = (uint8_t)status;
    imu->last_i2c_error = HAL_I2C_GetError(imu->hi2c);
    return status;
  }

  imu->last_status = HAL_OK;
  imu->last_i2c_error = HAL_I2C_ERROR_NONE;

  for (uint8_t axis = 0U; axis < 3U; axis++)
  {
    imu->accel_raw[axis] = ICM20948_Int16(&data[axis * 2U]);
    imu->gyro_raw[axis] = ICM20948_Int16(&data[6U + axis * 2U]);
    imu->accel_g[axis] = (float)imu->accel_raw[axis] / ICM20948_ACCEL_LSB_PER_G;
    imu->gyro_dps[axis] = ((float)imu->gyro_raw[axis] / ICM20948_GYRO_LSB_PER_DPS) - imu->gyro_bias_dps[axis];
  }

  imu->roll_deg = atan2f(imu->accel_g[1],
                         sqrtf((imu->accel_g[0] * imu->accel_g[0]) +
                               (imu->accel_g[2] * imu->accel_g[2]))) * ICM20948_RAD_TO_DEG;
  imu->pitch_deg = atan2f(imu->accel_g[0],
                          sqrtf((imu->accel_g[1] * imu->accel_g[1]) +
                                (imu->accel_g[2] * imu->accel_g[2]))) * ICM20948_RAD_TO_DEG;

  return HAL_OK;
}

HAL_StatusTypeDef ICM20948_CalibrateGyro(ICM20948_t *imu, uint16_t samples, uint32_t sample_delay_ms)
{
  float sum[3] = {0.0f, 0.0f, 0.0f};
  if ((imu == NULL) || (imu->initialized == 0U) || (samples == 0U)) return HAL_ERROR;

  imu->filter_initialized = 0U;
  for (uint16_t sample = 0U; sample < samples; sample++)
  {
    HAL_StatusTypeDef status = ICM20948_Read(imu);
    if (status != HAL_OK) return status;
    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
      sum[axis] += (float)imu->gyro_raw[axis] / ICM20948_GYRO_LSB_PER_DPS;
    }
    if (sample_delay_ms != 0U) HAL_Delay(sample_delay_ms);
  }
  for (uint8_t axis = 0U; axis < 3U; axis++)
  {
    imu->gyro_bias_dps[axis] = sum[axis] / (float)samples;
  }
  imu->filter_initialized = 0U;
  return HAL_OK;
}

HAL_StatusTypeDef ICM20948_UpdateComplementaryFilter(ICM20948_t *imu, float dt_s, float gyro_weight)
{
  if ((imu == NULL) || (imu->initialized == 0U) || (dt_s <= 0.0f) || (dt_s > 0.1f)) return HAL_ERROR;
  if (gyro_weight < 0.0f) gyro_weight = 0.0f;
  if (gyro_weight > 1.0f) gyro_weight = 1.0f;

  if (imu->filter_initialized == 0U)
  {
    imu->filtered_roll_deg = imu->roll_deg;
    imu->filtered_pitch_deg = imu->pitch_deg;
    imu->filter_initialized = 1U;
  }
  else
  {
    const float gyro_roll = imu->filtered_roll_deg + imu->gyro_dps[0] * dt_s;
    const float gyro_pitch = imu->filtered_pitch_deg + imu->gyro_dps[1] * dt_s;
    imu->filtered_roll_deg = gyro_weight * gyro_roll + (1.0f - gyro_weight) * imu->roll_deg;
    imu->filtered_pitch_deg = gyro_weight * gyro_pitch + (1.0f - gyro_weight) * imu->pitch_deg;
  }
  return HAL_OK;
}

uint8_t ICM20948_IsReady(ICM20948_t *imu)
{
  return (uint8_t)((imu != NULL) && (imu->initialized != 0U));
}
