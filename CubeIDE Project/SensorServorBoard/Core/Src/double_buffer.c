#include "double_buffer.h"
#include "stm32f4xx_hal.h"
#include <string.h>

sensor_double_buffer_t g_sensor_db;

void DoubleBuffer_Init(sensor_double_buffer_t *db)
{
    memset(db->snapshots, 0, sizeof(db->snapshots));
    db->write_idx = 0;
}

sensor_snapshot_t* DoubleBuffer_GetWriteBuffer(sensor_double_buffer_t *db)
{
    return &db->snapshots[db->write_idx];
}

void DoubleBuffer_Swap(sensor_double_buffer_t *db)
{
    /* Stamp the write buffer before swapping */
    db->snapshots[db->write_idx].timestamp_ms = HAL_GetTick();
    db->snapshots[db->write_idx].sequence++;

    /* Atomic index flip under interrupt disable */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    db->write_idx ^= 1;
    __set_PRIMASK(primask);

    /* Pre-copy: seed new write buffer with previous data for partial updates */
    db->snapshots[db->write_idx].data = db->snapshots[db->write_idx ^ 1].data;
}

const sensor_snapshot_t* DoubleBuffer_GetReadBuffer(const sensor_double_buffer_t *db)
{
    /* Read buffer is always the one the writer is NOT currently using */
    return &db->snapshots[db->write_idx ^ 1];
}
