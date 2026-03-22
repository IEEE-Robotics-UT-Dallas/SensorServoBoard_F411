#ifndef VL53L0X_H_
#define VL53L0X_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* VL53L0X Registers */
#define VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS 0x8A
#define VL53L0X_REG_SYSRANGE_START           0x00
#define VL53L0X_REG_RESULT_RANGE_STATUS      0x14

/* Default VL53L0X I2C Address (8-bit) */
#define VL53L0X_DEFAULT_ADDR 0x52

/**
 * @brief  Change the I2C address of a VL53L0X sensor.
 * @retval HAL_OK on success, HAL error code on failure.
 */
HAL_StatusTypeDef VL53L0X_SetAddress(I2C_HandleTypeDef *hi2c, uint8_t old_addr, uint8_t new_addr);

/**
 * @brief  Initialize a VL53L0X sensor at the given address.
 */
void VL53L0X_Init(I2C_HandleTypeDef *hi2c, uint8_t addr);

/**
 * @brief  Read distance in mm. Returns 0xFFFF on error.
 */
uint16_t VL53L0X_ReadDistance(I2C_HandleTypeDef *hi2c, uint8_t addr);

#endif /* VL53L0X_H_ */
