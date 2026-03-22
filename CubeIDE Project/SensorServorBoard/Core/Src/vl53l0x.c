#include "vl53l0x.h"
#include "i2c_common.h"
#include "cmsis_os.h"

HAL_StatusTypeDef VL53L0X_SetAddress(I2C_HandleTypeDef *hi2c, uint8_t old_addr, uint8_t new_addr)
{
    uint8_t new_addr_7bit = new_addr >> 1;
    return HAL_I2C_Mem_Write(hi2c, old_addr, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS,
                             I2C_MEMADD_SIZE_8BIT, &new_addr_7bit, 1, I2C_TIMEOUT);
}

void VL53L0X_Init(I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    uint8_t val = 0x01;
    if (HAL_I2C_Mem_Write(hi2c, addr, 0x88, I2C_MEMADD_SIZE_8BIT,
                          &val, 1, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c);
    }
}

uint16_t VL53L0X_ReadDistance(I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    uint8_t val = 0x01;
    uint8_t data[2];

    if (HAL_I2C_Mem_Write(hi2c, addr, VL53L0X_REG_SYSRANGE_START,
                          I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c);
        return 0xFFFF;
    }

    for (int i = 0; i < 200; i++) {
        if (HAL_I2C_Mem_Read(hi2c, addr, VL53L0X_REG_RESULT_RANGE_STATUS,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT) != HAL_OK) {
            i2c_recover(hi2c);
            return 0xFFFF;
        }
        if (val & 0x01) break;
        osDelay(1);
    }
    if (!(val & 0x01)) return 0xFFFF;

    if (HAL_I2C_Mem_Read(hi2c, addr, 0x1E, I2C_MEMADD_SIZE_8BIT,
                         data, 2, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c);
        return 0xFFFF;
    }

    return (data[0] << 8) | data[1];
}
