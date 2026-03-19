#include "i2c_common.h"

/**
 * @brief  GPIO-level I2C bus recovery for I2C3 (PA8=SCL, PB4=SDA).
 *         Toggles SCL 9+ times to clock out a stuck slave holding SDA low,
 *         then generates a STOP condition before re-enabling the peripheral.
 */
static void i2c3_bus_unstick(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* 1. Switch SCL (PA8) to push-pull output */
    gpio.Pin   = GPIO_PIN_8;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

    /* 2. Switch SDA (PB4) to input so we can read it */
    gpio.Pin  = GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* 3. Toggle SCL up to 16 times until SDA releases */
    for (int i = 0; i < 16; i++) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET)
            break;  /* SDA is free */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        for (volatile int d = 0; d < 100; d++);  /* ~5µs at 96MHz */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        for (volatile int d = 0; d < 100; d++);
    }

    /* 4. Generate STOP: SDA low → SCL high → SDA high */
    gpio.Pin  = GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);  /* SDA low */
    for (volatile int d = 0; d < 100; d++);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);    /* SCL high */
    for (volatile int d = 0; d < 100; d++);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);    /* SDA high = STOP */
    for (volatile int d = 0; d < 100; d++);

    /* 5. Restore AF open-drain for I2C3 */
    gpio.Pin       = GPIO_PIN_8;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin       = GPIO_PIN_4;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_I2C3;
    HAL_GPIO_Init(GPIOB, &gpio);
}

void i2c_recover(I2C_HandleTypeDef *hi2c)
{
    I2C_TypeDef *reg = hi2c->Instance;

    /* Disable peripheral before bus recovery */
    reg->CR1 &= ~I2C_CR1_PE;

    /* GPIO-level bus unstick (only for I2C3 — PA8/PB4) */
    if (reg == I2C3) {
        i2c3_bus_unstick();
    }

    /* Software reset of I2C peripheral */
    reg->SR1 = 0;
    reg->CR1 |= I2C_CR1_SWRST;
    reg->CR1 &= ~I2C_CR1_SWRST;

    /* APB1 = 48 MHz, 100 kHz standard mode */
    reg->CR2   = 48;
    reg->CCR   = 240;       /* 48 MHz / (2 * 240) = 100 kHz */
    reg->TRISE = 49;        /* (48 + 1) */
    reg->OAR1  = 0x4000;    /* bit 14 must be kept at 1 */
    reg->CR1  |= I2C_CR1_PE;

    hi2c->State     = HAL_I2C_STATE_READY;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    hi2c->Mode      = HAL_I2C_MODE_NONE;
    hi2c->Lock      = HAL_UNLOCKED;
}
