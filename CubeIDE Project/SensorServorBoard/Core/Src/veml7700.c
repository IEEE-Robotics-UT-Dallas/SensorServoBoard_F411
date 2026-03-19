#include "veml7700.h"
#include "i2c_common.h"
#include "cmsis_os.h"

/*
 * Resolution table from Vishay VEML7700 app note (Document #84323).
 * Maximum resolution = 0.0036 lux/count at gain x2, IT=800ms.
 * Resolution scales inversely with gain and integration time.
 *
 * resolution = 0.0036 * (800 / IT_ms) * (2 / gain)
 *
 * For our default config (gain x2, IT 200ms):
 *   0.0036 * (800/200) * (2/2) = 0.0144 lux/count
 */

/* Current configuration (set during init) */
static float current_resolution = 0.0144f;  /* gain x2, IT=200ms */

static uint16_t veml7700_read_register(I2C_HandleTypeDef *hi2c, uint8_t reg)
{
    uint8_t data[2] = {0, 0};
    if (HAL_I2C_Mem_Read(hi2c, VEML7700_ADDR_8BIT, reg,
                         I2C_MEMADD_SIZE_8BIT, data, 2, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c);
        return 0;
    }
    return (uint16_t)((data[1] << 8) | data[0]);
}

void LightSensor_Init(I2C_HandleTypeDef *hi2c)
{
    /*
     * ALS_CONF register (16-bit, little-endian):
     *   Bit 0     : ALS_SD     = 0 (power on)
     *   Bit 1     : ALS_INT_EN = 0 (no interrupt)
     *   Bits 5:4  : ALS_PERS   = 00 (1 sample)
     *   Bits 9:6  : ALS_IT     = 0001 (200ms)
     *   Bits 12:11: ALS_GAIN   = 01 (x2)
     *
     * Low byte:  IT[3:0]<<6 | PERS<<4 | INT_EN<<1 | SD
     *            = (0x01 << 6) | 0 | 0 | 0 = 0x40
     * High byte: GAIN<<3 = 0x01 << 3 = 0x08
     */
    uint8_t config[2] = {0x40, 0x08};
    if (HAL_I2C_Mem_Write(hi2c, VEML7700_ADDR_8BIT, VEML7700_REG_ALS_CONF,
                          I2C_MEMADD_SIZE_8BIT, config, 2, I2C_TIMEOUT) != HAL_OK) {
        i2c_recover(hi2c);
        return;
    }

    /* Resolution for gain x2, IT=200ms */
    current_resolution = 0.0036f * (800.0f / 200.0f) * (2.0f / 2.0f);

    /* Wait for first integration + startup (Vishay recommends >= IT + 2.5ms) */
    osDelay(250);
}

uint16_t LightSensor_ReadRaw(I2C_HandleTypeDef *hi2c)
{
    return veml7700_read_register(hi2c, VEML7700_REG_ALS_DATA);
}

uint16_t LightSensor_ReadWhite(I2C_HandleTypeDef *hi2c)
{
    return veml7700_read_register(hi2c, VEML7700_REG_WHITE_DATA);
}

float LightSensor_ReadLux(I2C_HandleTypeDef *hi2c)
{
    uint16_t raw = LightSensor_ReadRaw(hi2c);
    float lux = current_resolution * (float)raw;

    /*
     * Non-linear correction from Vishay app note (page 5):
     * For lux > 100, apply polynomial correction:
     *   lux_corrected = 6.0135e-13 * lux^4 - 9.3924e-9 * lux^3
     *                 + 8.1488e-5 * lux^2 + 1.0023 * lux
     */
    if (lux > 100.0f) {
        lux = (((6.0135e-13f * lux - 9.3924e-9f) * lux + 8.1488e-5f) * lux + 1.0023f) * lux;
    }

    return lux;
}

uint16_t LightSensor_Read(I2C_HandleTypeDef *hi2c)
{
    return (uint16_t)LightSensor_ReadLux(hi2c);
}
