/*
 * hal_stubs.c — Minimal HAL function stubs for host-side unit testing.
 * These provide default no-op implementations; CMock-generated mocks
 * override the I2C functions when linked into sensor_drivers tests.
 */
#include "hal_stubs.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>
#include <string.h>

/* Global timer/I2C handles referenced by the production code */
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

/* Shared data externs */
#include "shared_data.h"
#include "double_buffer.h"
sensor_double_buffer_t g_sensor_db;

/* ---- Tracking for __HAL_TIM_SET_COMPARE ---- */
static TIM_HandleTypeDef *last_tim_htim;
static uint32_t last_tim_channel;
static uint32_t last_tim_compare;
static int tim_set_compare_call_count;

void __HAL_TIM_SET_COMPARE_func(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t Compare) {
    last_tim_htim = htim;
    last_tim_channel = Channel;
    last_tim_compare = Compare;
    tim_set_compare_call_count++;
}

TIM_HandleTypeDef* hal_stub_get_last_tim_htim(void) { return last_tim_htim; }
uint32_t hal_stub_get_last_tim_channel(void)        { return last_tim_channel; }
uint32_t hal_stub_get_last_tim_compare(void)        { return last_tim_compare; }
int      hal_stub_get_tim_call_count(void)           { return tim_set_compare_call_count; }

void hal_stub_reset_tim_tracking(void) {
    last_tim_htim = NULL;
    last_tim_channel = 0;
    last_tim_compare = 0;
    tim_set_compare_call_count = 0;
}

/* ---- CMSIS-OS stubs ---- */
void osDelay(uint32_t ticks) { (void)ticks; }
osStatus_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout) {
    (void)mutex_id; (void)timeout; return osOK;
}
osStatus_t osMutexRelease(osMutexId_t mutex_id) {
    (void)mutex_id; return osOK;
}
uint32_t HAL_GetTick(void) {
    return 0;
}

/* ---- HAL TIM stubs ---- */
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel) {
    (void)htim; (void)Channel; return HAL_OK;
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim) { (void)htim; }
void Error_Handler(void) { }

/* ==== FreeRTOS heap_4-compatible pvPortMalloc / vPortFree ====
 * Mimics the heap_4 BlockLink_t header so that getBlockSize() in
 * custom_memory_manager.c works correctly on host.
 */
typedef struct StubBlockLink {
    struct StubBlockLink *pxNextFreeBlock;
    size_t xBlockSize;
} StubBlockLink_t;

static const size_t stub_xHeapStructSize =
    (sizeof(StubBlockLink_t) + ((size_t)(portBYTE_ALIGNMENT - 1)))
    & ~((size_t)portBYTE_ALIGNMENT_MASK);

static const size_t stub_xBlockAllocatedBit =
    ((size_t)1) << ((sizeof(size_t) * 8) - 1);

void *pvPortMalloc(size_t xWantedSize)
{
    size_t totalSize = stub_xHeapStructSize + xWantedSize;
    uint8_t *raw = (uint8_t *)malloc(totalSize);
    if (raw == NULL) return NULL;

    StubBlockLink_t *header = (StubBlockLink_t *)raw;
    header->pxNextFreeBlock = NULL;
    header->xBlockSize = totalSize | stub_xBlockAllocatedBit;

    return raw + stub_xHeapStructSize;
}

void vPortFree(void *pv)
{
    if (pv == NULL) return;
    uint8_t *raw = (uint8_t *)pv - stub_xHeapStructSize;
    free(raw);
}

/* ==== FreeRTOS task/time stubs ==== */
static long     stub_tick_overflow  = 0;
static uint32_t stub_tick_entering  = 0;

void hal_stub_set_tick_overflow(long overflow_count)  { stub_tick_overflow = overflow_count; }
void hal_stub_set_tick_entering(uint32_t entering)    { stub_tick_entering = entering; }

TickType_t xTaskGetTickCount(void)
{
    return (TickType_t)stub_tick_entering;
}

/* ---- HAL FLASH stubs ---- */
static uint8_t mock_flash[128 * 1024]; // 128KB for Sector 7
void* mock_flash_ptr = (void*)mock_flash;

HAL_StatusTypeDef HAL_FLASH_Unlock(void) { return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Lock(void) { return HAL_OK; }

HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError) {
    (void)pEraseInit;
    (void)SectorError;
    memset(mock_flash, 0xFF, sizeof(mock_flash));
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint32_t Data) {
    (void)TypeProgram;
    // Address is absolute in production, but we'll treat it as relative to mock_flash_ptr in our test setup
    // or we'll use the pointer directly if we redefine CONFIG_FLASH_ADDR
    uint32_t offset = Address - (uint32_t)(uintptr_t)mock_flash_ptr;
    if (offset + 4 <= sizeof(mock_flash)) {
        memcpy(&mock_flash[offset], &Data, 4);
        return HAL_OK;
    }
    return HAL_ERROR;
}

void* hal_stub_get_flash_ptr(void) { return mock_flash_ptr; }

void vTaskSetTimeOutState(TimeOut_t *pxTimeOut)
{
    pxTimeOut->xOverflowCount = (BaseType_t)stub_tick_overflow;
    pxTimeOut->xTimeOnEntering = (TickType_t)stub_tick_entering;
}
