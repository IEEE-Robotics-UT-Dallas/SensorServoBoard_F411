/*
 * test_sensor_drivers.c — Tests for VL53L0X, Mag, LightSensor drivers.
 * Uses CMock-generated mocks for HAL I2C functions.
 */
#include "unity.h"
#include "cmock.h"
#include "mock_mock_hal_i2c.h"
#include "sensor_drivers.h"
#include <string.h>

static I2C_HandleTypeDef test_hi2c;

void setUp(void) {
    mock_mock_hal_i2c_Init();
    memset(&test_hi2c, 0, sizeof(test_hi2c));
}

void tearDown(void) {
    mock_mock_hal_i2c_Verify();
    mock_mock_hal_i2c_Destroy();
}

/* ======================================================================== */
/* VL53L0X_SetAddress                                                       */
/* ======================================================================== */

void test_VL53L0X_SetAddress_writes_register_0x8A(void) {
    uint8_t old_addr = 0x52;
    uint8_t new_addr = 0x60;

    HAL_I2C_Mem_Write_ExpectAndReturn(&test_hi2c, old_addr,
        VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, I2C_MEMADD_SIZE_8BIT,
        NULL, 1, HAL_MAX_DELAY, HAL_OK);
    HAL_I2C_Mem_Write_IgnoreArg_pData();

    VL53L0X_SetAddress(&test_hi2c, old_addr, new_addr);
}

/* ======================================================================== */
/* VL53L0X_ReadDistance                                                      */
/* ======================================================================== */

/* Combined callback: call 0 = status register (ready), call 1 = distance data */
static uint16_t expected_distance_val;
static HAL_StatusTypeDef read_distance_cb(I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
    uint8_t *pData, uint16_t Size, uint32_t Timeout, int cmock_num_calls)
{
    (void)hi2c; (void)DevAddress; (void)MemAddSize; (void)Timeout;
    if (cmock_num_calls == 0) {
        /* Status register poll: bit 0 set = measurement complete */
        *pData = 0x01;
    } else {
        /* Distance data: big-endian 2 bytes */
        pData[0] = (expected_distance_val >> 8) & 0xFF;
        pData[1] = expected_distance_val & 0xFF;
    }
    return HAL_OK;
}

void test_VL53L0X_ReadDistance_returns_correct_value(void) {
    uint8_t addr = 0x52;
    expected_distance_val = 1234;

    /* Expect: write to start ranging */
    HAL_I2C_Mem_Write_ExpectAnyArgsAndReturn(HAL_OK);

    /* Stub all Mem_Read calls through our combined callback */
    HAL_I2C_Mem_Read_StubWithCallback(read_distance_cb);

    uint16_t dist = VL53L0X_ReadDistance(&test_hi2c, addr);
    TEST_ASSERT_EQUAL_UINT16(1234, dist);
}

void test_VL53L0X_ReadDistance_returns_error_on_i2c_fail(void) {
    uint8_t addr = 0x52;

    HAL_I2C_Mem_Write_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_I2C_Mem_Read_ExpectAnyArgsAndReturn(HAL_ERROR);

    uint16_t dist = VL53L0X_ReadDistance(&test_hi2c, addr);
    TEST_ASSERT_EQUAL_UINT16(0xFFFF, dist);
}

/* ======================================================================== */
/* Mag_Read                                                                  */
/* ======================================================================== */

/*
 * MLX90393 Mag_Read issues 4 HAL calls:
 *   0: Master_Transmit (SM_XYZ command)
 *   1: Master_Receive  (status, 1 byte)
 *   2: Master_Transmit (RM_XYZ command)
 *   3: Master_Receive  (7 bytes: status + X_H,X_L,Y_H,Y_L,Z_H,Z_L big-endian)
 */
static int mag_rx_call_count;

static HAL_StatusTypeDef mag_rx_cb(I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress, uint8_t *pData, uint16_t Size,
    uint32_t Timeout, int cmock_num_calls)
{
    (void)hi2c; (void)DevAddress; (void)Timeout; (void)cmock_num_calls;
    mag_rx_call_count++;
    if (Size == 1) {
        /* Status byte responses */
        pData[0] = 0x00;
    } else if (Size == 7) {
        /* RM response: status + big-endian XYZ */
        pData[0] = 0x00; /* status */
        /* x=100  = 0x0064 */
        pData[1] = 0x00; pData[2] = 0x64;
        /* y=-200 = 0xFF38 */
        pData[3] = 0xFF; pData[4] = 0x38;
        /* z=300  = 0x012C */
        pData[5] = 0x01; pData[6] = 0x2C;
    }
    return HAL_OK;
}

void test_Mag_Read_parses_big_endian_xyz(void) {
    mag_rx_call_count = 0;

    /* SM transmit + SM receive + RM transmit + RM receive */
    HAL_I2C_Master_Transmit_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_I2C_Master_Receive_StubWithCallback(mag_rx_cb);
    HAL_I2C_Master_Transmit_ExpectAnyArgsAndReturn(HAL_OK);

    MagData_t mag = Mag_Read(&test_hi2c);
    TEST_ASSERT_EQUAL_INT16(100,  mag.x);
    TEST_ASSERT_EQUAL_INT16(-200, mag.y);
    TEST_ASSERT_EQUAL_INT16(300,  mag.z);
}

void test_Mag_Read_returns_zero_on_failure(void) {
    /* SM transmit + SM receive OK, RM transmit OK, RM receive FAILS */
    HAL_I2C_Master_Transmit_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_I2C_Master_Receive_ExpectAnyArgsAndReturn(HAL_OK); /* SM status */
    HAL_I2C_Master_Transmit_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_I2C_Master_Receive_ExpectAnyArgsAndReturn(HAL_ERROR); /* RM fails */

    MagData_t mag = Mag_Read(&test_hi2c);
    TEST_ASSERT_EQUAL_INT16(0, mag.x);
    TEST_ASSERT_EQUAL_INT16(0, mag.y);
    TEST_ASSERT_EQUAL_INT16(0, mag.z);
}

/* ======================================================================== */
/* LightSensor_Read                                                          */
/* ======================================================================== */

static HAL_StatusTypeDef light_data_cb(I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
    uint8_t *pData, uint16_t Size, uint32_t Timeout, int cmock_num_calls)
{
    (void)hi2c; (void)DevAddress; (void)MemAddress; (void)MemAddSize;
    (void)Size; (void)Timeout; (void)cmock_num_calls;
    /* raw = 17370 (0x43DA), little-endian → 17370*0.0576 = 1000.51 → 1000 */
    pData[0] = 0xDA; /* low byte */
    pData[1] = 0x43; /* high byte */
    return HAL_OK;
}

void test_LightSensor_Read_converts_to_lux(void) {
    HAL_I2C_Mem_Read_StubWithCallback(light_data_cb);

    uint16_t lux = LightSensor_Read(&test_hi2c);
    /* 17370 * 0.0576 = 1000.512 → truncated to 1000 */
    TEST_ASSERT_EQUAL_UINT16(1000, lux);
}

void test_LightSensor_Read_returns_zero_on_failure(void) {
    HAL_I2C_Mem_Read_ExpectAnyArgsAndReturn(HAL_ERROR);

    uint16_t lux = LightSensor_Read(&test_hi2c);
    TEST_ASSERT_EQUAL_UINT16(0, lux);
}

/* ======================================================================== */
/* LightSensor_Init                                                          */
/* ======================================================================== */

void test_LightSensor_Init_writes_als_config(void) {
    HAL_I2C_Mem_Write_ExpectAnyArgsAndReturn(HAL_OK);
    LightSensor_Init(&test_hi2c);
}

/* ======================================================================== */
/* Mag_Init                                                                  */
/* ======================================================================== */

void test_Mag_Init_sends_reset_and_exit(void) {
    /* RT: transmit + receive, EX: transmit + receive = 4 HAL calls */
    HAL_I2C_Master_Transmit_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_I2C_Master_Receive_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_I2C_Master_Transmit_ExpectAnyArgsAndReturn(HAL_OK);
    HAL_I2C_Master_Receive_ExpectAnyArgsAndReturn(HAL_OK);
    Mag_Init(&test_hi2c);
}

/* ======================================================================== */
/* VL53L0X_Init                                                              */
/* ======================================================================== */

void test_VL53L0X_Init_writes_register(void) {
    HAL_I2C_Mem_Write_ExpectAnyArgsAndReturn(HAL_OK);
    VL53L0X_Init(&test_hi2c, 0x52);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_VL53L0X_SetAddress_writes_register_0x8A);
    RUN_TEST(test_VL53L0X_ReadDistance_returns_correct_value);
    RUN_TEST(test_VL53L0X_ReadDistance_returns_error_on_i2c_fail);
    RUN_TEST(test_Mag_Read_parses_big_endian_xyz);
    RUN_TEST(test_Mag_Read_returns_zero_on_failure);
    RUN_TEST(test_LightSensor_Read_converts_to_lux);
    RUN_TEST(test_LightSensor_Read_returns_zero_on_failure);
    RUN_TEST(test_LightSensor_Init_writes_als_config);
    RUN_TEST(test_Mag_Init_sends_reset_and_exit);
    RUN_TEST(test_VL53L0X_Init_writes_register);
    return UNITY_END();
}
