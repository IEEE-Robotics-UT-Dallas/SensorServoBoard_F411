#ifndef SHARED_DATA_H_
#define SHARED_DATA_H_

#include <stdint.h>
#include <stddef.h>
#include "cmsis_os.h"
#include "sensor_drivers.h"

/* ---- Sensor data struct (used by sensor tasks, mutex-protected) ---- */

typedef struct {
    uint16_t tof_distances[5];
    MagData_t mag_data;
    uint16_t light_lux;
} shared_sensor_data_t;

/* Forward-declare double buffer (defined in double_buffer.h) */
struct sensor_double_buffer;
extern struct sensor_double_buffer g_sensor_db;

/* ---- Telemetry index map (Float32MultiArray layout for /telemetry) ---- */

#define IDX_TOF_0   0   /* tof_distances[0] – range in meters */
#define IDX_TOF_1   1
#define IDX_TOF_2   2
#define IDX_TOF_3   3
#define IDX_TOF_4   4
#define IDX_MAG_X   5   /* mag_data.x */
#define IDX_MAG_Y   6   /* mag_data.y */
#define IDX_MAG_Z   7   /* mag_data.z */
#define IDX_LIGHT   8   /* light_lux */
#define TELEMETRY_SIZE 9

/* Convert shared_sensor_data_t → flat float telemetry array */
void shared_data_to_telemetry(const shared_sensor_data_t *data,
                              float *out, size_t out_size);

#endif /* SHARED_DATA_H_ */
