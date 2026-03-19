#include "sensor_drivers.h"
#include "cmsis_os.h"
#include <string.h>

#define I2C_TIMEOUT 100  /* ms — finite timeout for all I2C ops */

/* ── I2C Recovery (STM32F4 errata workaround) ──────────────────────────── */

static void i2c_recover(I2C_HandleTypeDef *hi2c)
{
    I2C_TypeDef *reg = hi2c->Instance;
    reg->CR1 &= ~I2C_CR1_PE;
    reg->SR1 = 0;
    reg->CR1 |= I2C_CR1_SWRST;
    reg->CR1 &= ~I2C_CR1_SWRST;
    /* APB1 = 48 MHz, 100 kHz standard mode */
    reg->CR2  = 48;
    reg->CCR  = 240;       /* 48 MHz / (2 * 240) = 100 kHz */
    reg->TRISE = 49;       /* (48 + 1) */
    reg->OAR1 = 0x4000;    /* bit 14 must be kept at 1 */
    reg->CR1 |= I2C_CR1_PE;
    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Lock  = HAL_UNLOCKED;
}

/* ── ToF Sensor (VL53L0X) ──────────────────────────────────────────────── */

void VL53L0X_SetAddress(I2C_HandleTypeDef *hi2c, uint8_t old_addr, uint8_t new_addr)
{
    uint8_t new_addr_7bit = new_addr >> 1;
    if (HAL_I2C_Mem_Write(hi2c, old_addr, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS,
                          I2C_MEMADD_SIZE_8BIT, &new_addr_7bit, 1, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c);
    }
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

    /* Poll for measurement complete — bounded to 200 ms max */
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

/* ── Magnetometer (MLX90393) ───────────────────────────────────────────── */

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

    /* Reset */
    cmd = MLX90393_CMD_RT;
    if (HAL_I2C_Master_Transmit(hi2c, MLX90393_ADDR_8BIT, &cmd, 1, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c); return;
    }
    if (HAL_I2C_Master_Receive(hi2c, MLX90393_ADDR_8BIT, &status, 1, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c); return;
    }
    osDelay(50);

    /* Exit any active mode */
    cmd = MLX90393_CMD_EX;
    if (HAL_I2C_Master_Transmit(hi2c, MLX90393_ADDR_8BIT, &cmd, 1, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c); return;
    }
    HAL_I2C_Master_Receive(hi2c, MLX90393_ADDR_8BIT, &status, 1, I2C_TIMEOUT);
    osDelay(5);

    /*
     * Register 2 (0x02): OSR + DIG_FILT + RES
     *   OSR=0 (fastest), DIG_FILT=2 (light filtering), RES=0 (default)
     *   Bits [1:0] = OSR = 00
     *   Bits [4:2] = DIG_FILT = 010
     *   → conversion time ~5ms per XYZ → ~200Hz capable
     *   Value: 0x0008 (DIG_FILT=2, OSR=0)
     */
    mlx_write_reg(hi2c, 0x02, 0x0008);
    osDelay(2);

    /*
     * Register 1 (0x01): BURST_DATA_RATE + BURST_SEL
     *   BURST_SEL[3:0] = 0xE (XYZ enabled, no T)
     *   BURST_DATA_RATE[5:0] = 0 (no extra delay between bursts)
     *   TRIG_INT_SEL = 0, COMM_MODE = 00 (I2C)
     *   Value: 0x00E0  (BURST_SEL=0xE in bits [9:6])
     */
    mlx_write_reg(hi2c, 0x01, 0x03C0);
    osDelay(2);

    /* Start burst mode — sensor now converts continuously */
    cmd = MLX90393_CMD_SB_XYZ;
    if (HAL_I2C_Master_Transmit(hi2c, MLX90393_ADDR_8BIT, &cmd, 1, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c); return;
    }
    if (HAL_I2C_Master_Receive(hi2c, MLX90393_ADDR_8BIT, &status, 1, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c); return;
    }
    osDelay(10);  /* let first conversion complete */
}

MagData_t Mag_Read(I2C_HandleTypeDef *hi2c)
{
    MagData_t mag_data = {0, 0, 0};
    uint8_t cmd, buf[7];

    /* In burst mode, just read the latest measurement */
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

/* ── Light Sensor (VEML7700) ───────────────────────────────────────────── */

void LightSensor_Init(I2C_HandleTypeDef *hi2c)
{
    /* ALS_CONF: gain x2 (bit 11), IT=200ms (bits 7:6 = 01), power on */
    uint8_t config[2] = {0x40, 0x08};  /* low byte: IT=200ms, high byte: gain x2 */
    if (HAL_I2C_Mem_Write(hi2c, VEML7700_ADDR_8BIT, VEML7700_REG_ALS_CONF,
                          I2C_MEMADD_SIZE_8BIT, config, 2, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c);
        return;
    }
    osDelay(250);  /* wait for first integration cycle */
}

uint16_t LightSensor_Read(I2C_HandleTypeDef *hi2c)
{
    uint8_t data[2] = {0, 0};

    if (HAL_I2C_Mem_Read(hi2c, VEML7700_ADDR_8BIT, VEML7700_REG_ALS_DATA,
                         I2C_MEMADD_SIZE_8BIT, data, 2, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c);
        return 0;
    }

    uint16_t raw = (data[1] << 8) | data[0];
    /* Resolution = 0.0288 lux/count at gain x2, IT=200ms */
    return (uint16_t)(raw * 0.0288f);
}
