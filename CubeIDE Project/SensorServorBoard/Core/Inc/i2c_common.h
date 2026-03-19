#ifndef I2C_COMMON_H_
#define I2C_COMMON_H_

#include "stm32f4xx_hal.h"

#define I2C_TIMEOUT 100  /* ms — finite timeout for all I2C ops */

/**
 * @brief  Recover a stuck I2C bus (STM32F4 errata workaround).
 *         Performs software reset, restores 100kHz standard mode config.
 * @param  hi2c  Pointer to I2C handle
 */
void i2c_recover(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Initialize I2C3 DMA transfer synchronization.
 *         Must be called after RTOS scheduler starts, before DMA transfers.
 */
void i2c3_dma_init(void);

/**
 * @brief  DMA-based I2C3 master transmit with RTOS synchronization.
 *         CPU is free during transfer; calling task blocks on semaphore.
 * @return HAL_OK on success, HAL_ERROR/HAL_TIMEOUT on failure.
 */
HAL_StatusTypeDef i2c3_transmit_dma(uint16_t addr, uint8_t *data, uint16_t size);

/**
 * @brief  DMA-based I2C3 master receive with RTOS synchronization.
 * @return HAL_OK on success, HAL_ERROR/HAL_TIMEOUT on failure.
 */
HAL_StatusTypeDef i2c3_receive_dma(uint16_t addr, uint8_t *data, uint16_t size);

/**
 * @brief  DMA-based I2C3 memory read (register-addressed) with RTOS sync.
 * @return HAL_OK on success, HAL_ERROR/HAL_TIMEOUT on failure.
 */
HAL_StatusTypeDef i2c3_mem_read_dma(uint16_t addr, uint8_t reg,
                                     uint8_t *data, uint16_t size);

/**
 * @brief  DMA-based I2C3 memory write with RTOS sync.
 * @return HAL_OK on success, HAL_ERROR/HAL_TIMEOUT on failure.
 */
HAL_StatusTypeDef i2c3_mem_write_dma(uint16_t addr, uint8_t reg,
                                      uint8_t *data, uint16_t size);

#endif /* I2C_COMMON_H_ */
