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

/* Magnetometer (MLX90393) Commands & Address */
#define MLX90393_ADDR_8BIT    0x18  /* 0x0C << 1 */
#define MLX90393_CMD_RT       0xF0  /* Reset */
#define MLX90393_CMD_EX       0x80  /* Exit mode */
#define MLX90393_CMD_SM_XYZ   0x3E  /* Start single measurement XYZ */
#define MLX90393_CMD_RM_XYZ   0x4E  /* Read measurement XYZ */

/* Light Sensor (VEML7700) Registers & Address */
#define VEML7700_ADDR_8BIT        0x20  /* 0x10 << 1 */
#define VEML7700_REG_ALS_CONF     0x00
#define VEML7700_REG_ALS_DATA     0x04
#define VEML7700_ALS_GAIN_1       0x0000  /* Gain x1 */
#define VEML7700_ALS_IT_100MS     0x0000  /* Integration time 100ms */
#define VEML7700_ALS_SD_ON        0x0000  /* Power on (SD=0) */

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
