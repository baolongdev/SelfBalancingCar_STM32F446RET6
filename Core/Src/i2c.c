#include "i2c.h"

I2C_HandleTypeDef hi2c1;

/* Bo loc so tren SDA/SCL: bo xung nhieu ngan hon 4 chu ky PCLK1 (~110 ns o
   36 MHz) do motor PWM ghep vao day IMU. */
#define I2C1_DIGITAL_FILTER   4U

/* 100 kHz (Standard mode) thay vi 400 kHz: doc 14 byte mat ~1.7 ms, van du
   cho chu ky 4 ms, nhung bien do timing/nhieu lon hon nhieu khi day dai va
   pull-up yeu (canh len cham). */
#define I2C1_CLOCK_HZ         100000U

static HAL_StatusTypeDef I2C1_InitPeripheral(void)
{
  HAL_StatusTypeDef status = HAL_I2C_Init(&hi2c1);
  if (status == HAL_OK)
  {
    status = HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
  }
  if (status == HAL_OK)
  {
    status = HAL_I2CEx_ConfigDigitalFilter(&hi2c1, I2C1_DIGITAL_FILTER);
  }
  return status;
}

void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = I2C1_CLOCK_HZ;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0U;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0U;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (I2C1_InitPeripheral() != HAL_OK)
  {
    Error_Handler();
  }
}

static void I2C1_RecoveryDelay(void)
{
  /* ~5 us o 72 MHz: xung SCL thu cong ~100 kHz. */
  for (uint32_t i = 0U; i < 120U; ++i)
  {
    __NOP();
  }
}

HAL_StatusTypeDef I2C1_BusRecover(void)
{
  GPIO_InitTypeDef gpio = {0};

  (void)HAL_I2C_DeInit(&hi2c1);

  /* Slave co the dang giu SDA thap (transaction bi cat giua chung do timeout
     hoac nhieu): xung SCL toi da 9 lan cho toi khi SDA nha, roi tao STOP. */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);
  I2C1_RecoveryDelay();

  for (uint8_t i = 0U; (i < 9U) && (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET); ++i)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    I2C1_RecoveryDelay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    I2C1_RecoveryDelay();
  }

  /* STOP: SDA len cao trong khi SCL dang cao. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  I2C1_RecoveryDelay();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  I2C1_RecoveryDelay();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  I2C1_RecoveryDelay();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  I2C1_RecoveryDelay();

  /* hi2c1.Init van giu cau hinh sau DeInit; MspInit tra chan ve AF_OD.
     HAL_I2C_Init lam SWRST nen phai cau hinh lai bo loc. */
  return I2C1_InitPeripheral();
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (i2cHandle->Instance == I2C1)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    /* Canh xuong cham hon -> it ringing tren day dai; 100 kHz khong can toc
       do cao. */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *i2cHandle)
{
  if (i2cHandle->Instance == I2C1)
  {
    __HAL_RCC_I2C1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
  }
}
