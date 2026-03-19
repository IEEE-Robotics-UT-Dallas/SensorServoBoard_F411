/*
 * on_target_tests.c — Automated on-target test suite for SensorServoBoard F411
 *
 * Build with -DHW_TEST to replace normal firmware.
 * Runs automated pass/fail tests over UART (500000 baud, PB6 TX / PB7 RX).
 * Reports results in Unity-style format for easy parsing.
 *
 * Current hardware: UART, MLX90393 (mag), VEML7700 (light) on I2C3
 * ToF sensors: not yet connected (tests skipped)
 */

#ifdef HW_TEST

#include "main.h"
#include "sensor_drivers.h"
#include "servo_control.h"
#include "tof_manager.h"
#include "shared_data.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart6;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/* ── UART Output (USART6 on PA11/PA12 via USB-C) ──────────────── */

static void uart_print(const char *str)
{
    HAL_UART_Transmit(&huart6, (uint8_t *)str, strlen(str), 100);
}

static void uart_printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    uart_print(buf);
}

/* ── Test Framework ────────────────────────────────────────────── */

static int tests_run    = 0;
static int tests_passed = 0;
static int tests_failed = 0;
static int tests_skipped = 0;
static int current_line = 0;

#define RUN_TEST(func) do { \
    current_line = __LINE__; \
    func(); \
    tests_run++; \
} while(0)

#define TEST_PASS(name) do { \
    tests_passed++; \
    uart_printf("  PASS: %s\r\n", name); \
} while(0)

#define TEST_FAIL(name, msg) do { \
    tests_failed++; \
    uart_printf("  FAIL: %s — %s\r\n", name, msg); \
} while(0)

#define TEST_SKIP(name, reason) do { \
    tests_skipped++; \
    uart_printf("  SKIP: %s — %s\r\n", name, reason); \
} while(0)

#define ASSERT_TRUE(name, cond) do { \
    if (cond) { TEST_PASS(name); } \
    else { TEST_FAIL(name, "expected true"); } \
} while(0)

#define ASSERT_EQ(name, actual, expected) do { \
    if ((actual) == (expected)) { TEST_PASS(name); } \
    else { \
        char _msg[80]; \
        snprintf(_msg, sizeof(_msg), "expected %d, got %d", (int)(expected), (int)(actual)); \
        TEST_FAIL(name, _msg); \
    } \
} while(0)

#define ASSERT_RANGE(name, val, lo, hi) do { \
    int _v = (int)(val); \
    if (_v >= (int)(lo) && _v <= (int)(hi)) { TEST_PASS(name); } \
    else { \
        char _msg[80]; \
        snprintf(_msg, sizeof(_msg), "value %d not in [%d, %d]", _v, (int)(lo), (int)(hi)); \
        TEST_FAIL(name, _msg); \
    } \
} while(0)

#define ASSERT_NEQ(name, actual, unexpected) do { \
    if ((actual) != (unexpected)) { TEST_PASS(name); } \
    else { \
        char _msg[80]; \
        snprintf(_msg, sizeof(_msg), "got unexpected %d", (int)(unexpected)); \
        TEST_FAIL(name, _msg); \
    } \
} while(0)

static void test_group(const char *name)
{
    uart_printf("\r\n── %s ──\r\n", name);
}

/* ── HAL & Clock Tests ─────────────────────────────────────────── */

static void test_hal_tick_running(void)
{
    uint32_t t1 = HAL_GetTick();
    HAL_Delay(10);
    uint32_t t2 = HAL_GetTick();
    ASSERT_TRUE("HAL tick incrementing", (t2 - t1) >= 8 && (t2 - t1) <= 20);
}

static void test_system_clock(void)
{
    uint32_t hclk = HAL_RCC_GetHCLKFreq();
    /* Should be ~96 MHz (PLLM=14, PLLN=215, PLLP=4, HSE=25MHz) */
    ASSERT_TRUE("HCLK > 90 MHz", hclk > 90000000);
    ASSERT_TRUE("HCLK < 100 MHz", hclk < 100000000);
}

static void test_freertos_tick(void)
{
    uint32_t t1 = osKernelGetTickCount();
    osDelay(50);
    uint32_t t2 = osKernelGetTickCount();
    ASSERT_TRUE("FreeRTOS tick running (50ms delay)", (t2 - t1) >= 45 && (t2 - t1) <= 60);
}

/* ── UART Tests ────────────────────────────────────────────────── */

static void test_uart_initialized(void)
{
    ASSERT_TRUE("USART6 state ready", huart6.gState == HAL_UART_STATE_READY);
}

static void test_uart_tx(void)
{
    uint8_t data[] = "UART_TX_OK";
    HAL_StatusTypeDef res = HAL_UART_Transmit(&huart6, data, sizeof(data) - 1, 100);
    uart_print("\r\n");
    ASSERT_EQ("USART6 TX returns HAL_OK", res, HAL_OK);
}

/* ── I2C Bus Tests ─────────────────────────────────────────────── */

static void test_i2c1_initialized(void)
{
    ASSERT_TRUE("I2C1 peripheral enabled",
                hi2c1.Instance->CR1 & I2C_CR1_PE);
}

static void test_i2c2_initialized(void)
{
    ASSERT_TRUE("I2C2 peripheral enabled",
                hi2c2.Instance->CR1 & I2C_CR1_PE);
}

static void test_i2c3_initialized(void)
{
    ASSERT_TRUE("I2C3 peripheral enabled",
                hi2c3.Instance->CR1 & I2C_CR1_PE);
}

static int i2c_count_devices(I2C_HandleTypeDef *hi2c)
{
    int count = 0;
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 1, 5) == HAL_OK)
            count++;
    }
    return count;
}

/* Reset I2C peripheral to clear bus errors (BERR, ARLO, AF, etc.) */
static void i2c_reset(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_DeInit(hi2c);
    osDelay(10);
    HAL_I2C_Init(hi2c);
    osDelay(10);
}

static void test_i2c3_has_devices(void)
{
    int n = i2c_count_devices(&hi2c3);
    uart_printf("    (found %d device(s) on I2C3)\r\n", n);
    ASSERT_TRUE("I2C3 has ≥1 device (mag+light)", n >= 1);
}

/* ── MLX90393 Magnetometer Tests ───────────────────────────────── */

static void test_mag_present(void)
{
    HAL_StatusTypeDef res = HAL_I2C_IsDeviceReady(&hi2c3, MLX90393_ADDR_8BIT, 3, 50);
    ASSERT_EQ("MLX90393 responds at 0x0C", res, HAL_OK);
}

static void test_mag_init(void)
{
    Mag_Init(&hi2c3);
    osDelay(50);
    TEST_PASS("Mag_Init() completed");
}

static void test_mag_read_nonzero(void)
{
    MagData_t m = Mag_Read(&hi2c3);
    int any_nonzero = (m.x != 0) || (m.y != 0) || (m.z != 0);
    uart_printf("    (X=%d Y=%d Z=%d)\r\n", m.x, m.y, m.z);
    ASSERT_TRUE("Mag_Read returns nonzero data", any_nonzero);
}

static void test_mag_read_stable(void)
{
    MagData_t m1 = Mag_Read(&hi2c3);
    osDelay(100);
    MagData_t m2 = Mag_Read(&hi2c3);

    int dx = abs(m1.x - m2.x);
    int dy = abs(m1.y - m2.y);
    int dz = abs(m1.z - m2.z);
    uart_printf("    (delta: X=%d Y=%d Z=%d)\r\n", dx, dy, dz);
    /* Stationary board — readings should be within 500 counts */
    ASSERT_TRUE("Mag readings stable (delta < 500)", dx < 500 && dy < 500 && dz < 500);
}

static void test_mag_read_in_range(void)
{
    MagData_t m = Mag_Read(&hi2c3);
    /* MLX90393 raw values: int16_t range, typical earth field ~ few hundred counts */
    ASSERT_TRUE("Mag X in int16 range", m.x > -32000 && m.x < 32000);
    ASSERT_TRUE("Mag Y in int16 range", m.y > -32000 && m.y < 32000);
    ASSERT_TRUE("Mag Z in int16 range", m.z > -32000 && m.z < 32000);
}

/* ── VEML7700 Light Sensor Tests ───────────────────────────────── */

static void test_light_present(void)
{
    HAL_StatusTypeDef res = HAL_I2C_IsDeviceReady(&hi2c3, VEML7700_ADDR_8BIT, 3, 50);
    ASSERT_EQ("VEML7700 responds at 0x10", res, HAL_OK);
}

static void test_light_init(void)
{
    LightSensor_Init(&hi2c3);
    osDelay(200);
    TEST_PASS("LightSensor_Init() completed");
}

static void test_light_read_plausible(void)
{
    uint16_t lux = LightSensor_Read(&hi2c3);
    uart_printf("    (lux = %u)\r\n", lux);
    /* Indoor: 50-1000 lux typical, 0 = dark, 65535 = saturated */
    ASSERT_TRUE("Light reading < 65535 (not saturated)", lux < 65535);
}

static void test_light_read_stable(void)
{
    uint16_t l1 = LightSensor_Read(&hi2c3);
    osDelay(200);
    uint16_t l2 = LightSensor_Read(&hi2c3);
    int delta = abs((int)l1 - (int)l2);
    uart_printf("    (l1=%u l2=%u delta=%d)\r\n", l1, l2, delta);
    /* Stable lighting — readings within 20% or 50 lux */
    int threshold = (l1 / 5) + 50;
    ASSERT_TRUE("Light readings stable", delta < threshold);
}

/* ── Timer / PWM Tests ─────────────────────────────────────────── */

static void test_tim2_config(void)
{
    ASSERT_TRUE("TIM2 prescaler set (≈1µs/tick)", htim2.Init.Prescaler > 50);
    ASSERT_EQ("TIM2 period = 19999 (50Hz)", htim2.Init.Period, 19999);
}

static void test_tim3_config(void)
{
    ASSERT_TRUE("TIM3 prescaler set (≈1µs/tick)", htim3.Init.Prescaler > 50);
    ASSERT_EQ("TIM3 period = 19999 (50Hz)", htim3.Init.Period, 19999);
}

static void test_servo_init_and_center(void)
{
    ServoControl_Init();
    osDelay(50);

    /* After init, all servos should be at 1500µs (center) */
    uint32_t ccr = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
    ASSERT_EQ("Servo 1 centered at 1500µs", ccr, 1500);
}

static void test_servo_set_angle(void)
{
    ServoControl_SetAngle(SERVO_CTRL_1, 0.0f);
    osDelay(10);
    uint32_t ccr0 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);

    ServoControl_SetAngle(SERVO_CTRL_1, 180.0f);
    osDelay(10);
    uint32_t ccr180 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);

    /* 0° = 500µs, 180° = 2500µs */
    ASSERT_EQ("Servo angle 0° → 500µs", ccr0, 500);
    ASSERT_EQ("Servo angle 180° → 2500µs", ccr180, 2500);

    /* Restore center */
    ServoControl_SetAngle(SERVO_CTRL_1, 90.0f);
}

/* ── GPIO / XSHUT Tests ────────────────────────────────────────── */

static void test_xshut_pins_controllable(void)
{
    /* PB0 — ToF 1 XSHUT */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    osDelay(5);
    GPIO_PinState low = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    osDelay(5);
    GPIO_PinState high = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);

    ASSERT_TRUE("XSHUT PB0 toggles LOW→HIGH", low == GPIO_PIN_RESET && high == GPIO_PIN_SET);
}

/* ── shared_data Tests ─────────────────────────────────────────── */

static void test_shared_data_telemetry(void)
{
    shared_sensor_data_t data = {0};
    data.tof_distances[0] = 1000;
    data.tof_distances[1] = 2000;
    data.mag_data.x = 100;
    data.mag_data.y = -200;
    data.mag_data.z = 300;
    data.light_lux = 500;

    float telem[TELEMETRY_SIZE];
    shared_data_to_telemetry(&data, telem, TELEMETRY_SIZE);

    ASSERT_TRUE("telemetry[0] = tof_0 in meters",
                fabsf(telem[IDX_TOF_0] - 1.0f) < 0.01f);
    ASSERT_TRUE("telemetry[5] = mag_x",
                fabsf(telem[IDX_MAG_X] - 100.0f) < 0.01f);
    ASSERT_TRUE("telemetry[8] = light_lux",
                fabsf(telem[IDX_LIGHT] - 500.0f) < 0.01f);
}

/* ── Memory Tests ──────────────────────────────────────────────── */

static void test_heap_available(void)
{
    size_t free_heap = xPortGetFreeHeapSize();
    uart_printf("    (free heap: %u bytes)\r\n", (unsigned)free_heap);
    ASSERT_TRUE("Heap > 4KB free", free_heap > 4096);
}

/* ── ToF Tests (skipped — no hardware) ─────────────────────────── */

static void test_tof_skipped(void)
{
    TEST_SKIP("ToF I2C1 sensor scan", "ToF sensors not connected");
    TEST_SKIP("ToF I2C2 sensor scan", "ToF sensors not connected");
    TEST_SKIP("ToF_Init_All()", "ToF sensors not connected");
    TEST_SKIP("ToF_ReadDistance()", "ToF sensors not connected");
}

/* ── Test Runner ───────────────────────────────────────────────── */

void HW_Test_Run(void)
{
    osDelay(500);

    uart_printf("\r\n");
    uart_printf("╔══════════════════════════════════════════════╗\r\n");
    uart_printf("║  SensorServoBoard F411 — On-Target Tests    ║\r\n");
    uart_printf("║  UART: 500000 baud  |  %s     ║\r\n", __DATE__);
    uart_printf("╚══════════════════════════════════════════════╝\r\n");

    /* ── System ── */
    test_group("System & Clock");
    RUN_TEST(test_hal_tick_running);
    RUN_TEST(test_system_clock);
    RUN_TEST(test_freertos_tick);
    RUN_TEST(test_heap_available);

    /* ── UART ── */
    test_group("UART");
    RUN_TEST(test_uart_initialized);
    RUN_TEST(test_uart_tx);

    /* ── I2C Buses ── */
    test_group("I2C Buses");
    RUN_TEST(test_i2c1_initialized);
    RUN_TEST(test_i2c2_initialized);
    RUN_TEST(test_i2c3_initialized);
    RUN_TEST(test_i2c3_has_devices);

    /* ── Magnetometer ── */
    test_group("MLX90393 Magnetometer (I2C3)");
    i2c_reset(&hi2c3);
    RUN_TEST(test_mag_present);
    RUN_TEST(test_mag_init);
    RUN_TEST(test_mag_read_nonzero);
    RUN_TEST(test_mag_read_stable);
    RUN_TEST(test_mag_read_in_range);

    /* ── Light Sensor ── */
    test_group("VEML7700 Light Sensor (I2C3)");
    i2c_reset(&hi2c3);
    RUN_TEST(test_light_present);
    RUN_TEST(test_light_init);
    RUN_TEST(test_light_read_plausible);
    RUN_TEST(test_light_read_stable);

    /* ── Timers & Servos ── */
    test_group("Timers & PWM");
    RUN_TEST(test_tim2_config);
    RUN_TEST(test_tim3_config);
    RUN_TEST(test_servo_init_and_center);
    RUN_TEST(test_servo_set_angle);

    /* ── GPIO ── */
    test_group("GPIO (XSHUT)");
    RUN_TEST(test_xshut_pins_controllable);

    /* ── Data Conversion ── */
    test_group("Shared Data / Telemetry");
    RUN_TEST(test_shared_data_telemetry);

    /* ── Skipped (no hardware) ── */
    test_group("ToF Sensors (not connected)");
    RUN_TEST(test_tof_skipped);

    /* ── Summary ── */
    uart_printf("\r\n");
    uart_printf("══════════════════════════════════════════════\r\n");
    uart_printf("  %d tests: %d passed, %d failed, %d skipped\r\n",
                tests_run, tests_passed, tests_failed, tests_skipped);
    if (tests_failed == 0) {
        uart_printf("  ✓ ALL TESTS PASSED\r\n");
    } else {
        uart_printf("  ✗ %d FAILURE(S)\r\n", tests_failed);
    }
    uart_printf("══════════════════════════════════════════════\r\n");

    /* Blink or idle */
    for (;;) {
        osDelay(1000);
    }
}

#endif /* HW_TEST */
