#include "shared_data.h"

void shared_data_to_telemetry(const shared_sensor_data_t *data,
                              float *out, size_t out_size)
{
    if (data == NULL || out == NULL || out_size < TELEMETRY_SIZE) {
        return;
    }

    /* ToF distances: uint16 mm → float meters */
    for (int i = 0; i < 5; i++) {
        out[IDX_TOF_0 + i] = (float)data->tof_distances[i] / 1000.0f;
    }

    /* Magnetometer: int16 raw → float */
    out[IDX_MAG_X] = (float)data->mag_data.x;
    out[IDX_MAG_Y] = (float)data->mag_data.y;
    out[IDX_MAG_Z] = (float)data->mag_data.z;

    /* Ambient light: uint16 lux → float */
    out[IDX_LIGHT] = (float)data->light_lux;
}
