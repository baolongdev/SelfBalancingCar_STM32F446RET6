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
/* Doc 14 byte o 100 kHz mat ~1.7 ms. Timeout ngan de loi I2C khong treo
   main loop (va giu PWM cu) qua vai ms. */
#define ICM20948_IO_TIMEOUT_MS      5U
/* Accel/gyro luon co nhieu LSB; 50 frame giong het nhau (200 ms o 250 Hz)
   nghia la chip da dung cap nhat du lieu. */
#define ICM20948_STUCK_LIMIT        50U
#define ICM20948_PWR_MGMT_1_RUN     0x01U
/* Gyro can ~35 ms khoi dong sau khi thoat sleep; doc som se ra 0/rac va lam
   sai bias calibrate. */
#define ICM20948_GYRO_STARTUP_MS    50U
/* Bo qua vai mau dau va tu choi calibrate neu xe dang bi cam/di chuyen. */
#define ICM20948_CALIB_DISCARD      10U
#define ICM20948_CALIB_MAX_SPREAD_DPS 4.0f
#define ICM20948_ACCEL_LSB_PER_G    16384.0f
/* Gyro FS = +/-2000 dps (xem GYRO_CONFIG_1 ben duoi) de tranh clip khi xe
   do/nga that voi toc do goc > 250 dps. */
#define ICM20948_GYRO_LSB_PER_DPS   16.4f
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
  const uint32_t timeout_ms = (imu->initialized != 0U) ? ICM20948_IO_TIMEOUT_MS : ICM20948_TIMEOUT_MS;
  return HAL_I2C_Mem_Read(imu->hi2c, imu->address, reg, I2C_MEMADD_SIZE_8BIT,
                          data, length, timeout_ms);
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

  /* Gyro: 2000 dps (tranh clip khi roll/pitch doi nhanh), 1 kHz output;
     accel: 2 g, 1 kHz output. GYRO_FCHOICE=1 (bit0), GYRO_FS_SEL=11 (bits 2:1). */
  if (ICM20948_Write(imu, ICM20948_REG_GYRO_SMPLRT_DIV, 0x00U) != HAL_OK ||
      ICM20948_Write(imu, ICM20948_REG_GYRO_CONFIG_1, 0x07U) != HAL_OK ||
      ICM20948_Write(imu, ICM20948_REG_ACCEL_SMPLRT_DIV_1, 0x00U) != HAL_OK ||
      ICM20948_Write(imu, ICM20948_REG_ACCEL_SMPLRT_DIV_2, 0x00U) != HAL_OK ||
      ICM20948_Write(imu, ICM20948_REG_ACCEL_CONFIG, 0x01U) != HAL_OK ||
      ICM20948_SelectBank(imu, ICM20948_BANK0) != HAL_OK)
  {
    return ICM20948_InitFailure(imu);
  }

  HAL_Delay(ICM20948_GYRO_STARTUP_MS);

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

  /* Init() luon ket thuc o bank 0 va khong co code nao khac doi bank, nen
     khong can ghi REG_BANK_SEL moi lan doc. */
  status = ICM20948_ReadRegisters(imu, ICM20948_REG_ACCEL_XOUT_H, data, sizeof(data));
  if (status != HAL_OK)
  {
    imu->last_status = (uint8_t)status;
    imu->last_i2c_error = HAL_I2C_GetError(imu->hi2c);
    return status;
  }

  if (memcmp(data, imu->last_frame, sizeof(data)) == 0)
  {
    if (imu->stuck_samples < 0xFFFFU) imu->stuck_samples++;
    if (imu->stuck_samples >= ICM20948_STUCK_LIMIT)
    {
      imu->last_status = ICM20948_STATUS_STUCK;
      imu->last_i2c_error = HAL_I2C_ERROR_NONE;
      return HAL_ERROR;
    }
  }
  else
  {
    imu->stuck_samples = 0U;
    memcpy(imu->last_frame, data, sizeof(data));
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

  /* Quy ước balance: xe đứng trên hai bánh = 0 deg, xe nằm ngang = +/-90 deg.
     Theo log thực tế, tư thế xe nằm có gravity trên +Y, nên dùng roll chuẩn
     theo Y; khi xe đứng, gravity chuyển về trục Z và góc tiến về 0 deg. */
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
  float min_dps[3] = {0.0f, 0.0f, 0.0f};
  float max_dps[3] = {0.0f, 0.0f, 0.0f};
  if ((imu == NULL) || (imu->initialized == 0U) || (samples == 0U)) return HAL_ERROR;

  imu->filter_initialized = 0U;
  for (uint16_t sample = 0U; sample < (uint16_t)(samples + ICM20948_CALIB_DISCARD); sample++)
  {
    HAL_StatusTypeDef status = ICM20948_Read(imu);
    if (status != HAL_OK) return status;
    if (sample >= ICM20948_CALIB_DISCARD)
    {
      for (uint8_t axis = 0U; axis < 3U; axis++)
      {
        const float dps = (float)imu->gyro_raw[axis] / ICM20948_GYRO_LSB_PER_DPS;
        if ((sample == ICM20948_CALIB_DISCARD) || (dps < min_dps[axis])) min_dps[axis] = dps;
        if ((sample == ICM20948_CALIB_DISCARD) || (dps > max_dps[axis])) max_dps[axis] = dps;
        sum[axis] += dps;
      }
    }
    if (sample_delay_ms != 0U) HAL_Delay(sample_delay_ms);
  }
  for (uint8_t axis = 0U; axis < 3U; axis++)
  {
    /* Xe bi cam/di chuyen: giu bias cu thay vi hoc sai. */
    if ((max_dps[axis] - min_dps[axis]) > ICM20948_CALIB_MAX_SPREAD_DPS) return HAL_BUSY;
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

HAL_StatusTypeDef ICM20948_CheckAlive(ICM20948_t *imu)
{
  uint8_t regs[7];
  HAL_StatusTypeDef status;

  if ((imu == NULL) || (imu->initialized == 0U)) return HAL_ERROR;

  /* Doc lien 0x00 (WHO_AM_I) .. 0x06 (PWR_MGMT_1) trong mot transaction. */
  status = ICM20948_ReadRegisters(imu, ICM20948_REG_WHO_AM_I, regs, sizeof(regs));
  if (status != HAL_OK)
  {
    imu->last_status = (uint8_t)status;
    imu->last_i2c_error = HAL_I2C_GetError(imu->hi2c);
    return status;
  }
  if ((regs[0] != ICM20948_WHO_AM_I_VALUE) ||
      (regs[ICM20948_REG_PWR_MGMT_1] != ICM20948_PWR_MGMT_1_RUN))
  {
    imu->last_status = ICM20948_STATUS_NOT_ALIVE;
    imu->last_i2c_error = HAL_I2C_ERROR_NONE;
    return HAL_ERROR;
  }
  return HAL_OK;
}

uint8_t ICM20948_IsReady(ICM20948_t *imu)
{
  return (uint8_t)((imu != NULL) && (imu->initialized != 0U));
}
