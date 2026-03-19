#include "mlx90393.h"
#include "i2c_common.h"
#include "cmsis_os.h"

/* Write a 16-bit register: [WR, data_hi, data_lo, addr<<2] → 1 status byte */
static HAL_StatusTypeDef mlx_write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t val)
{
    uint8_t buf[4] = { MLX90393_CMD_WR, (uint8_t)(val >> 8), (uint8_t)val, (uint8_t)(reg << 2) };
    uint8_t status;
    if (HAL_I2C_Master_Transmit(hi2c, MLX90393_ADDR_8BIT, buf, 4, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c); return HAL_ERROR;
    }
    if (HAL_I2C_Master_Receive(hi2c, MLX90393_ADDR_8BIT, &status, 1, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c); return HAL_ERROR;
    }
    return HAL_OK;
}

void Mag_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t cmd, status;

    /* Exit any active mode first — sensor may be in burst from previous MCU run */
    cmd = MLX90393_CMD_EX;
    HAL_I2C_Master_Transmit(hi2c, MLX90393_ADDR_8BIT, &cmd, 1, I2C_TIMEOUT);
    HAL_I2C_Master_Receive(hi2c, MLX90393_ADDR_8BIT, &status, 1, I2C_TIMEOUT);
    osDelay(10);

    /* Reset from idle — 1 byte response */
    cmd = MLX90393_CMD_RT;
    if (HAL_I2C_Master_Transmit(hi2c, MLX90393_ADDR_8BIT, &cmd, 1, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c); return;
    }
    if (HAL_I2C_Master_Receive(hi2c, MLX90393_ADDR_8BIT, &status, 1, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c); return;
    }
    osDelay(200);
    i2c_recover(hi2c);
    osDelay(10);

    /* Register 2: OSR=0, DIG_FILT=2 */
    if (mlx_write_reg(hi2c, 0x02, 0x0008) != HAL_OK) return;
    osDelay(2);

    /* Register 1: BURST_SEL=XYZ, BURST_DATA_RATE=0 */
    if (mlx_write_reg(hi2c, 0x01, 0x03C0) != HAL_OK) return;
    osDelay(2);

    /* Start burst mode */
    cmd = MLX90393_CMD_SB_XYZ;
    if (HAL_I2C_Master_Transmit(hi2c, MLX90393_ADDR_8BIT, &cmd, 1, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c); return;
    }
    if (HAL_I2C_Master_Receive(hi2c, MLX90393_ADDR_8BIT, &status, 1, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c); return;
    }
    osDelay(10);
}

MagData_t Mag_Read(I2C_HandleTypeDef *hi2c)
{
    MagData_t mag_data = {0, 0, 0};
    uint8_t cmd, buf[7];

    cmd = MLX90393_CMD_RM_XYZ;
    if (HAL_I2C_Master_Transmit(hi2c, MLX90393_ADDR_8BIT, &cmd, 1, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c); return mag_data;
    }
    if (HAL_I2C_Master_Receive(hi2c, MLX90393_ADDR_8BIT, buf, 7, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c); return mag_data;
    }

    mag_data.x = (int16_t)((buf[1] << 8) | buf[2]);
    mag_data.y = (int16_t)((buf[3] << 8) | buf[4]);
    mag_data.z = (int16_t)((buf[5] << 8) | buf[6]);
    return mag_data;
}
