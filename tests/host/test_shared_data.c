/*
 * test_shared_data.c — Pure unit tests for shared_data_to_telemetry().
 * No mocks needed: the function is purely computational.
 */
#include "unity.h"
#include "shared_data.h"
#include <string.h>
#include <math.h>
#include <float.h>

static shared_sensor_data_t data;
static float out[TELEMETRY_SIZE];

void setUp(void) {
    memset(&data, 0, sizeof(data));
    memset(out, 0xFF, sizeof(out)); /* fill with garbage */
}

void tearDown(void) { }

/* ---- Normal values ---- */
void test_normal_values(void) {
    data.tof_distances[0] = 1000;
    data.tof_distances[1] = 2000;
    data.tof_distances[2] = 3000;
    data.tof_distances[3] = 4000;
    data.tof_distances[4] = 5000;
    data.mag_data.x = 100;
    data.mag_data.y = -200;
    data.mag_data.z = 300;
    data.light_lux = 500;

    shared_data_to_telemetry(&data, out, TELEMETRY_SIZE);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f,   out[IDX_TOF_0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f,   out[IDX_TOF_1]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.0f,   out[IDX_TOF_2]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.0f,   out[IDX_TOF_3]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f,   out[IDX_TOF_4]);
    TEST_ASSERT_FLOAT_WITHIN(0.1f,   100.0f, out[IDX_MAG_X]);
    TEST_ASSERT_FLOAT_WITHIN(0.1f,  -200.0f, out[IDX_MAG_Y]);
    TEST_ASSERT_FLOAT_WITHIN(0.1f,   300.0f, out[IDX_MAG_Z]);
    TEST_ASSERT_FLOAT_WITHIN(0.1f,   500.0f, out[IDX_LIGHT]);
}

/* ---- Zero values ---- */
void test_zero_values(void) {
    shared_data_to_telemetry(&data, out, TELEMETRY_SIZE);

    for (int i = 0; i < TELEMETRY_SIZE; i++) {
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, out[i]);
    }
}

/* ---- Max values ---- */
void test_max_values(void) {
    for (int i = 0; i < 5; i++) data.tof_distances[i] = 65535;
    data.mag_data.x = INT16_MAX;
    data.mag_data.y = INT16_MIN;
    data.mag_data.z = INT16_MAX;
    data.light_lux = UINT16_MAX;

    shared_data_to_telemetry(&data, out, TELEMETRY_SIZE);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 65.535f,     out[IDX_TOF_0]);
    TEST_ASSERT_FLOAT_WITHIN(0.1f,  32767.0f,    out[IDX_MAG_X]);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -32768.0f,    out[IDX_MAG_Y]);
    TEST_ASSERT_FLOAT_WITHIN(0.1f,  65535.0f,    out[IDX_LIGHT]);
}

/* ---- NULL data pointer ---- */
void test_null_data_pointer(void) {
    float sentinel = -999.0f;
    out[0] = sentinel;
    shared_data_to_telemetry(NULL, out, TELEMETRY_SIZE);
    TEST_ASSERT_EQUAL_FLOAT(sentinel, out[0]); /* output untouched */
}

/* ---- NULL output pointer ---- */
void test_null_output_pointer(void) {
    shared_data_to_telemetry(&data, NULL, TELEMETRY_SIZE);
    /* no crash = pass */
}

/* ---- out_size too small ---- */
void test_out_size_too_small(void) {
    float sentinel = -999.0f;
    out[0] = sentinel;
    shared_data_to_telemetry(&data, out, TELEMETRY_SIZE - 1);
    TEST_ASSERT_EQUAL_FLOAT(sentinel, out[0]); /* output untouched */
}

/* ---- ToF conversion accuracy: 1500mm → 1.5m ---- */
void test_tof_conversion_accuracy(void) {
    data.tof_distances[0] = 1500;
    shared_data_to_telemetry(&data, out, TELEMETRY_SIZE);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.5f, out[IDX_TOF_0]);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_normal_values);
    RUN_TEST(test_zero_values);
    RUN_TEST(test_max_values);
    RUN_TEST(test_null_data_pointer);
    RUN_TEST(test_null_output_pointer);
    RUN_TEST(test_out_size_too_small);
    RUN_TEST(test_tof_conversion_accuracy);
    return UNITY_END();
}
