/*
 * hal_stubs.h — Minimal HAL type definitions for host-side unit testing.
 * Intercepts #include "stm32f4xx_hal.h" and "main.h" and "cmsis_os.h"
 * so the real STM32 headers are never pulled in on x86.
 */
#ifndef HAL_STUBS_H_
#define HAL_STUBS_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- HAL Status ---- */
typedef enum {
    HAL_OK      = 0x00U,
    HAL_ERROR   = 0x01U,
    HAL_BUSY    = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

/* ---- I2C Handle (minimal) ---- */
typedef struct {
    uint32_t dummy;
} I2C_HandleTypeDef;

/* ---- Timer Handle (minimal) ---- */
typedef struct {
    uint32_t Instance_placeholder;
    struct {
        uint32_t CCR1;
        uint32_t CCR2;
        uint32_t CCR3;
        uint32_t CCR4;
    } ChannelValues;
} TIM_HandleTypeDef;

/* ---- UART Handle (minimal) ---- */
typedef struct {
    uint32_t dummy;
} UART_HandleTypeDef;

/* ---- Timer channel defines ---- */
#define TIM_CHANNEL_1  0x00000000U
#define TIM_CHANNEL_2  0x00000004U
#define TIM_CHANNEL_3  0x00000008U
#define TIM_CHANNEL_4  0x0000000CU

/* ---- I2C memory address size ---- */
#define I2C_MEMADD_SIZE_8BIT   0x00000001U
#define I2C_MEMADD_SIZE_16BIT  0x00000010U

/* ---- HAL max delay ---- */
#define HAL_MAX_DELAY 0xFFFFFFFFU

/* ---- HAL I2C functions (to be mocked) ---- */
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                     uint16_t MemAddress, uint16_t MemAddSize,
                                     uint8_t *pData, uint16_t Size, uint32_t Timeout);

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                    uint16_t MemAddress, uint16_t MemAddSize,
                                    uint8_t *pData, uint16_t Size, uint32_t Timeout);

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                           uint8_t *pData, uint16_t Size, uint32_t Timeout);

HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                                          uint8_t *pData, uint16_t Size, uint32_t Timeout);

/* ---- HAL TIM ---- */
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/*
 * __HAL_TIM_SET_COMPARE — redefined as a real function for testability.
 * The real HAL uses a macro that writes directly to a register.
 */
void __HAL_TIM_SET_COMPARE_func(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t Compare);
#define __HAL_TIM_SET_COMPARE(htim, channel, compare) \
    __HAL_TIM_SET_COMPARE_func((htim), (channel), (compare))

/* ---- CMSIS-OS2 stubs ---- */
typedef void* osMutexId_t;
typedef uint32_t osSemaphoreId_t;

typedef struct {
    const char *name;
    uint32_t attr_bits;
    void *cb_mem;
    uint32_t cb_size;
} osSemaphoreAttr_t;

#define osWaitForever 0xFFFFFFFFU

typedef enum {
    osOK = 0
} osStatus_t;

void osDelay(uint32_t ticks);
osStatus_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout);
osStatus_t osMutexRelease(osMutexId_t mutex_id);
uint32_t HAL_GetTick(void);

/* ---- Error handler ---- */
void Error_Handler(void);

/* ---- FreeRTOS time control for tests ---- */
void hal_stub_set_tick_overflow(long overflow_count);
void hal_stub_set_tick_entering(uint32_t entering);

#ifdef __cplusplus
}
#endif

#endif /* HAL_STUBS_H_ */
