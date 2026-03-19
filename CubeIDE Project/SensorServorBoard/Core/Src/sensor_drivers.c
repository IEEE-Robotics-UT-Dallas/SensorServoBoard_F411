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
/* Magnetometer (MLX90393) Functions                                         */
/* ------------------------------------------------------------------------- */

void Mag_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t cmd;
    uint8_t status;

    /* Reset the MLX90393 */
    cmd = MLX90393_CMD_RT;
    HAL_I2C_Master_Transmit(hi2c, MLX90393_ADDR_8BIT, &cmd, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(hi2c, MLX90393_ADDR_8BIT, &status, 1, HAL_MAX_DELAY);
    osDelay(2);

    /* Exit any current mode */
    cmd = MLX90393_CMD_EX;
    HAL_I2C_Master_Transmit(hi2c, MLX90393_ADDR_8BIT, &cmd, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(hi2c, MLX90393_ADDR_8BIT, &status, 1, HAL_MAX_DELAY);
}

MagData_t Mag_Read(I2C_HandleTypeDef *hi2c) {
    MagData_t mag_data = {0, 0, 0};
    uint8_t cmd;
    uint8_t status;
    uint8_t buf[7]; /* status + X(2) + Y(2) + Z(2) */

    /* Start single measurement for XYZ */
    cmd = MLX90393_CMD_SM_XYZ;
    HAL_I2C_Master_Transmit(hi2c, MLX90393_ADDR_8BIT, &cmd, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(hi2c, MLX90393_ADDR_8BIT, &status, 1, HAL_MAX_DELAY);

    /* Wait for conversion */
    osDelay(20);

    /* Read measurement */
    cmd = MLX90393_CMD_RM_XYZ;
    HAL_I2C_Master_Transmit(hi2c, MLX90393_ADDR_8BIT, &cmd, 1, HAL_MAX_DELAY);
    if (HAL_I2C_Master_Receive(hi2c, MLX90393_ADDR_8BIT, buf, 7, HAL_MAX_DELAY) == HAL_OK) {
        /* buf[0]=status, buf[1..6]=X_H,X_L,Y_H,Y_L,Z_H,Z_L (big-endian) */
        mag_data.x = (int16_t)((buf[1] << 8) | buf[2]);
        mag_data.y = (int16_t)((buf[3] << 8) | buf[4]);
        mag_data.z = (int16_t)((buf[5] << 8) | buf[6]);
    }

    return mag_data;
}

/* ------------------------------------------------------------------------- */
/* Light Sensor (VEML7700) Functions                                         */
/* ------------------------------------------------------------------------- */

void LightSensor_Init(I2C_HandleTypeDef *hi2c) {
    /* Configure: Gain x1, Integration Time 100ms, Power on */
    uint8_t config[2] = {0x00, 0x00}; /* little-endian: low byte first */
    HAL_I2C_Mem_Write(hi2c, VEML7700_ADDR_8BIT, VEML7700_REG_ALS_CONF,
                      I2C_MEMADD_SIZE_8BIT, config, 2, HAL_MAX_DELAY);
    osDelay(5);
}

uint16_t LightSensor_Read(I2C_HandleTypeDef *hi2c) {
    uint8_t data[2];
    uint16_t lux = 0;

    if (HAL_I2C_Mem_Read(hi2c, VEML7700_ADDR_8BIT, VEML7700_REG_ALS_DATA,
                         I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY) == HAL_OK) {
        uint16_t raw = (data[1] << 8) | data[0]; /* little-endian */
        lux = (uint16_t)(raw * 0.0576f);
    }

    return lux;
}
