#ifndef SENSOR_DRIVERS_H_
#define SENSOR_DRIVERS_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* I2C Handles (Assumed to be defined elsewhere, e.g., main.c or i2c.c) */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

/* VL53L0X Registers */
#define VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS 0x8A
#define VL53L0X_REG_SYSRANGE_START           0x00
#define VL53L0X_REG_RESULT_RANGE_STATUS      0x14

/* Default VL53L0X I2C Address (8-bit) */
#define VL53L0X_DEFAULT_ADDR 0x52

/* Magnetometer (QMC5883L) Registers & Address */
#define QMC5883L_ADDR_8BIT   0x1A /* 0x0D << 1 */
#define QMC5883L_REG_OUT_X_L 0x00
#define QMC5883L_REG_CTRL1   0x09

/* Light Sensor (BH1750) Registers & Address */
#define BH1750_ADDR_8BIT      0x46 /* 0x23 << 1 */
#define BH1750_CMD_POWER_ON   0x01
#define BH1750_CMD_CONT_H_RES 0x10

/* Data Structures */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} MagData_t;

/* Function Prototypes */

/* ToF Sensor Functions */
void VL53L0X_SetAddress(I2C_HandleTypeDef *hi2c, uint8_t old_addr, uint8_t new_addr);
void VL53L0X_Init(I2C_HandleTypeDef *hi2c, uint8_t addr);
uint16_t VL53L0X_ReadDistance(I2C_HandleTypeDef *hi2c, uint8_t addr);

/* Magnetometer Functions */
void Mag_Init(I2C_HandleTypeDef *hi2c);
MagData_t Mag_Read(I2C_HandleTypeDef *hi2c);

/* Light Sensor Functions */
void LightSensor_Init(I2C_HandleTypeDef *hi2c);
uint16_t LightSensor_Read(I2C_HandleTypeDef *hi2c);

#endif /* SENSOR_DRIVERS_H_ */
