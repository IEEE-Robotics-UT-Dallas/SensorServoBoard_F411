#include "sensor_drivers.h"
#include "cmsis_os.h" // For osDelay

/* ------------------------------------------------------------------------- */
/* ToF Sensor (VL53L0X) Functions                                            */
/* ------------------------------------------------------------------------- */

void VL53L0X_SetAddress(I2C_HandleTypeDef *hi2c, uint8_t old_addr, uint8_t new_addr) {
    /* The VL53L0X expects a 7-bit address for this register write */
    uint8_t new_addr_7bit = new_addr >> 1;
    HAL_I2C_Mem_Write(hi2c, old_addr, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, I2C_MEMADD_SIZE_8BIT, &new_addr_7bit, 1, HAL_MAX_DELAY);
}

void VL53L0X_Init(I2C_HandleTypeDef *hi2c, uint8_t addr) {
    /* Skeleton initialization for VL53L0X */
    /* In a full driver, this would involve downloading tuning settings */
    /* Here we just ensure the device is accessible and do a basic setup */
    uint8_t val = 0x01;
    HAL_I2C_Mem_Write(hi2c, addr, 0x88, I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);
}

uint16_t VL53L0X_ReadDistance(I2C_HandleTypeDef *hi2c, uint8_t addr) {
    uint8_t val = 0x01;
    uint8_t data[2];
    uint16_t distance = 0;

    /* Start single ranging measurement */
    HAL_I2C_Mem_Write(hi2c, addr, VL53L0X_REG_SYSRANGE_START, I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);

    /* Wait for measurement to complete (polling status register) */
    do {
        if (HAL_I2C_Mem_Read(hi2c, addr, VL53L0X_REG_RESULT_RANGE_STATUS, I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY) != HAL_OK) {
            return 0xFFFF; /* Error */
        }
        osDelay(1);
    } while ((val & 0x01) == 0);

    /* Read distance (14 bytes starting from 0x14, distance is at offset 10 and 11) */
    /* For simplicity, we read just the 2 bytes of distance at 0x1E */
    if (HAL_I2C_Mem_Read(hi2c, addr, 0x1E, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY) == HAL_OK) {
        distance = (data[0] << 8) | data[1];
    } else {
        distance = 0xFFFF;
    }

    return distance;
}

/* ------------------------------------------------------------------------- */
/* Magnetometer (QMC5883L) Functions                                         */
/* ------------------------------------------------------------------------- */

void Mag_Init(I2C_HandleTypeDef *hi2c) {
    /* Initialize QMC5883L */
    /* Set Mode to Continuous, ODR to 50Hz, RNG to 8G, OSR to 512 */
    uint8_t ctrl1 = 0x1D; 
    HAL_I2C_Mem_Write(hi2c, QMC5883L_ADDR_8BIT, QMC5883L_REG_CTRL1, I2C_MEMADD_SIZE_8BIT, &ctrl1, 1, HAL_MAX_DELAY);
}

MagData_t Mag_Read(I2C_HandleTypeDef *hi2c) {
    MagData_t mag_data = {0, 0, 0};
    uint8_t data[6];

    if (HAL_I2C_Mem_Read(hi2c, QMC5883L_ADDR_8BIT, QMC5883L_REG_OUT_X_L, I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY) == HAL_OK) {
        mag_data.x = (int16_t)((data[1] << 8) | data[0]);
        mag_data.y = (int16_t)((data[3] << 8) | data[2]);
        mag_data.z = (int16_t)((data[5] << 8) | data[4]);
    }

    return mag_data;
}

/* ------------------------------------------------------------------------- */
/* Light Sensor (BH1750) Functions                                           */
/* ------------------------------------------------------------------------- */

void LightSensor_Init(I2C_HandleTypeDef *hi2c) {
    /* Initialize BH1750 */
    uint8_t cmd = BH1750_CMD_POWER_ON;
    HAL_I2C_Master_Transmit(hi2c, BH1750_ADDR_8BIT, &cmd, 1, HAL_MAX_DELAY);
    osDelay(10);
    cmd = BH1750_CMD_CONT_H_RES;
    HAL_I2C_Master_Transmit(hi2c, BH1750_ADDR_8BIT, &cmd, 1, HAL_MAX_DELAY);
}

uint16_t LightSensor_Read(I2C_HandleTypeDef *hi2c) {
    uint8_t data[2];
    uint16_t lux = 0;

    if (HAL_I2C_Master_Receive(hi2c, BH1750_ADDR_8BIT, data, 2, HAL_MAX_DELAY) == HAL_OK) {
        lux = (data[0] << 8) | data[1];
        lux = lux / 1.2; /* Convert to lux according to datasheet */
    }

    return lux;
}
