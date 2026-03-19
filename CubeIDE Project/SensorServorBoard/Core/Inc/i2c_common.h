#ifndef I2C_COMMON_H_
#define I2C_COMMON_H_

#include "stm32f4xx_hal.h"

#define I2C_TIMEOUT 100  /* ms — finite timeout for all I2C ops */

/**
 * @brief  Recover a stuck I2C bus (STM32F4 errata workaround).
 *         Performs software reset, restores 100kHz standard mode config.
 * @param  hi2c  Pointer to I2C handle
 */
void i2c_recover(I2C_HandleTypeDef *hi2c);

#endif /* I2C_COMMON_H_ */
