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
shared_sensor_data_t g_sensor_data;
osMutexId_t sensor_data_mutex;

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

void vTaskSetTimeOutState(TimeOut_t *pxTimeOut)
{
    pxTimeOut->xOverflowCount = (BaseType_t)stub_tick_overflow;
    pxTimeOut->xTimeOnEntering = (TickType_t)stub_tick_entering;
}
