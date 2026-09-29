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

static int16_t ICM20948_Int16(const uint8_t *data)
{
  return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

HAL_StatusTypeDef ICM20948_Init(ICM20948_t *imu, I2C_HandleTypeDef *hi2c, uint16_t address)
{
  uint8_t who_am_i = 0U;

  if ((imu == NULL) || (hi2c == NULL))
  {
    return HAL_ERROR;
  }

  memset(imu, 0, sizeof(*imu));
  imu->hi2c = hi2c;
  imu->address = address;

  if (HAL_I2C_IsDeviceReady(imu->hi2c, imu->address, 3U, ICM20948_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (ICM20948_SelectBank(imu, ICM20948_BANK0) != HAL_OK ||
      ICM20948_ReadRegisters(imu, ICM20948_REG_WHO_AM_I, &who_am_i, 1U) != HAL_OK ||
      who_am_i != ICM20948_WHO_AM_I_VALUE)
  {
    return HAL_ERROR;
  }

  if (ICM20948_Write(imu, ICM20948_REG_PWR_MGMT_1, 0x80U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(100U);

  if (ICM20948_Write(imu, ICM20948_REG_PWR_MGMT_1, 0x01U) != HAL_OK ||
      ICM20948_Write(imu, ICM20948_REG_PWR_MGMT_2, 0x00U) != HAL_OK ||
      ICM20948_SelectBank(imu, ICM20948_BANK2) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* Gyro: 250 dps, 1 kHz output; accel: 2 g, 1 kHz output. */
  if (ICM20948_Write(imu, ICM20948_REG_GYRO_SMPLRT_DIV, 0x00U) != HAL_OK ||
      ICM20948_Write(imu, ICM20948_REG_GYRO_CONFIG_1, 0x01U) != HAL_OK ||
      ICM20948_Write(imu, ICM20948_REG_ACCEL_SMPLRT_DIV_1, 0x00U) != HAL_OK ||
      ICM20948_Write(imu, ICM20948_REG_ACCEL_SMPLRT_DIV_2, 0x00U) != HAL_OK ||
      ICM20948_Write(imu, ICM20948_REG_ACCEL_CONFIG, 0x01U) != HAL_OK ||
      ICM20948_SelectBank(imu, ICM20948_BANK0) != HAL_OK)
  {
    return HAL_ERROR;
  }

  imu->initialized = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef ICM20948_Read(ICM20948_t *imu)
{
  uint8_t data[14];

  if ((imu == NULL) || (imu->initialized == 0U))
  {
    return HAL_ERROR;
  }

  if (ICM20948_SelectBank(imu, ICM20948_BANK0) != HAL_OK ||
      ICM20948_ReadRegisters(imu, ICM20948_REG_ACCEL_XOUT_H, data, sizeof(data)) != HAL_OK)
  {
    return HAL_ERROR;
  }

  for (uint8_t axis = 0U; axis < 3U; axis++)
  {
    imu->accel_raw[axis] = ICM20948_Int16(&data[axis * 2U]);
    imu->gyro_raw[axis] = ICM20948_Int16(&data[6U + axis * 2U]);
    imu->accel_g[axis] = (float)imu->accel_raw[axis] / ICM20948_ACCEL_LSB_PER_G;
    imu->gyro_dps[axis] = (float)imu->gyro_raw[axis] / ICM20948_GYRO_LSB_PER_DPS;
  }

  imu->roll_deg = atan2f(imu->accel_g[1],
                         sqrtf((imu->accel_g[0] * imu->accel_g[0]) +
                               (imu->accel_g[2] * imu->accel_g[2]))) * ICM20948_RAD_TO_DEG;
  imu->pitch_deg = atan2f(imu->accel_g[0],
                          sqrtf((imu->accel_g[1] * imu->accel_g[1]) +
                                (imu->accel_g[2] * imu->accel_g[2]))) * ICM20948_RAD_TO_DEG;

  return HAL_OK;
}

uint8_t ICM20948_IsReady(ICM20948_t *imu)
{
  return (uint8_t)((imu != NULL) && (imu->initialized != 0U));
}
