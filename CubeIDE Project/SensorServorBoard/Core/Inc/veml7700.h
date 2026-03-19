#ifndef VEML7700_H_
#define VEML7700_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* VEML7700 I2C Address (8-bit) */
#define VEML7700_ADDR_8BIT        0x20  /* 0x10 << 1 */

/* VEML7700 Registers */
#define VEML7700_REG_ALS_CONF     0x00
#define VEML7700_REG_ALS_WH       0x01  /* High threshold */
#define VEML7700_REG_ALS_WL       0x02  /* Low threshold */
#define VEML7700_REG_PSM          0x03  /* Power save mode */
#define VEML7700_REG_ALS_DATA     0x04
#define VEML7700_REG_WHITE_DATA   0x05
#define VEML7700_REG_ALS_INT      0x06  /* Interrupt status */

/* Gain settings (bits 12:11 of ALS_CONF) */
#define VEML7700_GAIN_1           0x00  /* x1 */
#define VEML7700_GAIN_2           0x01  /* x2 */
#define VEML7700_GAIN_1_8         0x02  /* x1/8 */
#define VEML7700_GAIN_1_4         0x03  /* x1/4 */

/* Integration time settings (bits 9:6 of ALS_CONF) */
#define VEML7700_IT_25MS          0x0C
#define VEML7700_IT_50MS          0x08
#define VEML7700_IT_100MS         0x00
#define VEML7700_IT_200MS         0x01
#define VEML7700_IT_400MS         0x02
#define VEML7700_IT_800MS         0x03

/**
 * @brief  Initialize VEML7700: gain x2, integration time 200ms, power on.
 *         Blocks for first integration cycle.
 */
void LightSensor_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Read ambient light level in lux (float precision).
 *         Uses proper resolution calculation based on gain and IT.
 * @return Lux value as float. Returns 0.0f on I2C error.
 */
float LightSensor_ReadLux(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Read raw ALS count (16-bit).
 * @return Raw ALS register value. Returns 0 on I2C error.
 */
uint16_t LightSensor_ReadRaw(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Read white channel raw count (16-bit).
 * @return Raw white register value. Returns 0 on I2C error.
 */
uint16_t LightSensor_ReadWhite(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Legacy API — returns lux as uint16_t (truncated).
 *         Maintained for backward compatibility with existing code.
 */
uint16_t LightSensor_Read(I2C_HandleTypeDef *hi2c);

#endif /* VEML7700_H_ */
