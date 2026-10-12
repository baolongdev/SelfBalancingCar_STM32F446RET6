#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

void MX_I2C1_Init(void);
/* Giai phong bus (9 xung SCL + STOP) va init lai I2C1; khong treo CPU khi loi. */
HAL_StatusTypeDef I2C1_BusRecover(void);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */
