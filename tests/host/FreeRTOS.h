/*
 * FreeRTOS.h — Minimal stub for host-side unit testing.
 * Intercepts #include "FreeRTOS.h" so the real FreeRTOS headers are never pulled in.
 */
#ifndef FREERTOS_H_STUB
#define FREERTOS_H_STUB

#include <stdint.h>
#include <stddef.h>

/* ---- Kernel configuration (matches project FreeRTOSConfig.h) ---- */
#define configTICK_RATE_HZ          ((TickType_t)1000)

/* ---- Port constants (matches Cortex-M4 port) ---- */
#define portBYTE_ALIGNMENT          8
#define portBYTE_ALIGNMENT_MASK     0x0007
#define portMAX_DELAY               0xFFFFFFFFUL

/* ---- Basic types ---- */
typedef uint32_t TickType_t;
typedef long     BaseType_t;
typedef unsigned long UBaseType_t;

/* ---- Heap API ---- */
void *pvPortMalloc(size_t xWantedSize);
void  vPortFree(void *pv);

#endif /* FREERTOS_H_STUB */
