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
    if (i2c3_mem_read_dma(VEML7700_ADDR_8BIT, reg, data, 2) != HAL_OK) {
        return 0;
    }
    return (uint16_t)((data[1] << 8) | data[0]);
}

void LightSensor_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t config[2] = {0x40, 0x08};
    if (i2c3_mem_write_dma(VEML7700_ADDR_8BIT, VEML7700_REG_ALS_CONF,
                            config, 2) != HAL_OK) {
        i2c_recover(hi2c);
        return;
    }

    current_resolution = 0.0036f * (800.0f / 200.0f) * (2.0f / 2.0f);
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
