/*
 * task.h — Minimal stub for host-side unit testing.
 * Intercepts #include "task.h" so the real FreeRTOS task header is never pulled in.
 */
#ifndef TASK_H_STUB
#define TASK_H_STUB

#include "FreeRTOS.h"

/* ---- TimeOut structure (used by clock_gettime in microros_time.c) ---- */
typedef struct {
    BaseType_t xOverflowCount;
    TickType_t xTimeOnEntering;
} TimeOut_t;

/* ---- Task API stubs ---- */
TickType_t xTaskGetTickCount(void);
void       vTaskSetTimeOutState(TimeOut_t *pxTimeOut);

#endif /* TASK_H_STUB */
