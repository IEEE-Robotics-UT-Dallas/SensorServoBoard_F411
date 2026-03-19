#ifndef DOUBLE_BUFFER_H_
#define DOUBLE_BUFFER_H_

#include "shared_data.h"
#include <stdint.h>

/* Timestamped sensor snapshot */
typedef struct {
    shared_sensor_data_t data;
    uint32_t timestamp_ms;   /* HAL_GetTick() at snapshot time */
    uint8_t  sequence;       /* Incrementing counter for freshness detection */
} sensor_snapshot_t;

/* Double-buffered sensor data with critical-section-protected swap */
typedef struct sensor_double_buffer {
    sensor_snapshot_t snapshots[2];
    volatile uint8_t write_idx;     /* Writer's current buffer index (0 or 1) */
} sensor_double_buffer_t;

/* Global double buffer instance */
extern sensor_double_buffer_t g_sensor_db;

/**
 * @brief  Initialize double buffer: zero both snapshots, create semaphore.
 */
void DoubleBuffer_Init(sensor_double_buffer_t *db);

/**
 * @brief  Get pointer to the current write buffer.
 *         Writer fills this, then calls DoubleBuffer_Swap().
 *         NEVER blocks.
 */
sensor_snapshot_t* DoubleBuffer_GetWriteBuffer(sensor_double_buffer_t *db);

/**
 * @brief  Swap write and read buffers (atomic index flip).
 *         After swap, the reader sees the data the writer just finished.
 *         Increments sequence counter and sets timestamp.
 */
void DoubleBuffer_Swap(sensor_double_buffer_t *db);

/**
 * @brief  Get pointer to the current read buffer (last completed snapshot).
 *         NEVER blocks. Always returns a consistent snapshot.
 */
const sensor_snapshot_t* DoubleBuffer_GetReadBuffer(const sensor_double_buffer_t *db);

#endif /* DOUBLE_BUFFER_H_ */
