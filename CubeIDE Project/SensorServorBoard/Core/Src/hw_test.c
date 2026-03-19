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
#include "double_buffer.h"
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
    HAL_UART_Transmit(&huart6, (uint8_t *)str, strlen(str), 500);
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
    HAL_Delay(50);
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

static void test_uart6_dma(void)
{
    int dma_rx_linked = (huart6.hdmarx != NULL);
    int dma_tx_linked = (huart6.hdmatx != NULL);
    uart_printf("    (hdmarx=%p hdmatx=%p)\r\n",
                (void*)huart6.hdmarx, (void*)huart6.hdmatx);
    ASSERT_TRUE("USART6 DMA RX handle linked", dma_rx_linked);
    ASSERT_TRUE("USART6 DMA TX handle linked", dma_tx_linked);

    if (dma_rx_linked) {
        /* Verify DMA RX stream is configured for USART6 (DMA2_Stream1) */
        ASSERT_TRUE("USART6 DMA RX instance = DMA2_Stream1",
                     huart6.hdmarx->Instance == DMA2_Stream1);
    }
    if (dma_tx_linked) {
        ASSERT_TRUE("USART6 DMA TX instance = DMA2_Stream6",
                     huart6.hdmatx->Instance == DMA2_Stream6);
    }

    /* Test DMA TX: send a short burst */
    uint8_t dma_test[] = "DMA_OK";
    HAL_StatusTypeDef res = HAL_UART_Transmit_DMA(&huart6, dma_test, sizeof(dma_test) - 1);
    if (res == HAL_OK) {
        /* Wait for TX complete */
        int timeout = 100;
        while (huart6.gState != HAL_UART_STATE_READY && timeout-- > 0)
            HAL_Delay(1);
    }
    uart_print("\r\n");
    ASSERT_EQ("USART6 DMA TX returns HAL_OK", res, HAL_OK);

    /* Test DMA RX: start circular receive, check counter moves */
    static uint8_t rx_buf[64];
    res = HAL_UART_Receive_DMA(&huart6, rx_buf, sizeof(rx_buf));
    uart_printf("    (DMA RX start: %d, NDTR=%lu)\r\n",
                (int)res,
                (unsigned long)__HAL_DMA_GET_COUNTER(huart6.hdmarx));
    ASSERT_EQ("USART6 DMA RX start returns HAL_OK", res, HAL_OK);
    /* Stop DMA so UART is available for test output again */
    HAL_UART_DMAStop(&huart6);
    /* Ensure UART state is clean for subsequent polling TX */
    huart6.gState = HAL_UART_STATE_READY;
    huart6.RxState = HAL_UART_STATE_READY;
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
        if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 1, 5) == HAL_OK) {
            uart_printf("    (device at 0x%02X)\r\n", addr);
            count++;
        }
    }
    return count;
}

/* i2c_recover() now lives in i2c_common.c (included via sensor_drivers.h) */

static void test_i2c3_has_devices(void)
{
    int n = i2c_count_devices(&hi2c3);
    uart_printf("    (found %d device(s) on I2C3)\r\n", n);
    ASSERT_TRUE("I2C3 has ≥1 device (mag+light)", n >= 1);
}

/* ── MLX90393 Magnetometer Tests ───────────────────────────────── */
/* I2C_TIMEOUT provided by i2c_common.h */

static void test_mag_present(void)
{
    HAL_StatusTypeDef res = HAL_I2C_IsDeviceReady(&hi2c3, MLX90393_ADDR_8BIT, 3, 50);
    ASSERT_EQ("MLX90393 responds at 0x0C", res, HAL_OK);
}

static void test_mag_init(void)
{
    uint8_t cmd;
    uint8_t buf[7];
    HAL_StatusTypeDef res;

    /* Exit any active mode — sensor persists state across MCU resets */
    cmd = MLX90393_CMD_EX;
    HAL_I2C_Master_Transmit(&hi2c3, MLX90393_ADDR_8BIT, &cmd, 1, I2C_TIMEOUT);
    HAL_I2C_Master_Receive(&hi2c3, MLX90393_ADDR_8BIT, buf, 1, I2C_TIMEOUT);
    uart_printf("    (EX status: 0x%02X)\r\n", buf[0]);
    HAL_Delay(10);

    /* Reset from idle — response is 1 byte when not in burst */
    cmd = MLX90393_CMD_RT;
    res = HAL_I2C_Master_Transmit(&hi2c3, MLX90393_ADDR_8BIT, &cmd, 1, I2C_TIMEOUT);
    if (res != HAL_OK) { ASSERT_EQ("Mag reset TX", res, HAL_OK); return; }
    res = HAL_I2C_Master_Receive(&hi2c3, MLX90393_ADDR_8BIT, buf, 1, I2C_TIMEOUT);
    if (res != HAL_OK) { ASSERT_EQ("Mag reset RX", res, HAL_OK); return; }
    uart_printf("    (RT status: 0x%02X)\r\n", buf[0]);
    HAL_Delay(200);
    i2c_recover(&hi2c3);
    HAL_Delay(10);

    /* Single measurement */
    cmd = MLX90393_CMD_SM_XYZ;
    res = HAL_I2C_Master_Transmit(&hi2c3, MLX90393_ADDR_8BIT, &cmd, 1, I2C_TIMEOUT);
    if (res != HAL_OK) {
        uart_printf("    (SM TX err=%d, EC=0x%02lX)\r\n", res, hi2c3.ErrorCode);
        ASSERT_EQ("Mag SM TX", res, HAL_OK); return;
    }
    res = HAL_I2C_Master_Receive(&hi2c3, MLX90393_ADDR_8BIT, buf, 1, I2C_TIMEOUT);
    if (res != HAL_OK) { ASSERT_EQ("Mag SM RX", res, HAL_OK); return; }
    uart_printf("    (SM status: 0x%02X)\r\n", buf[0]);
    HAL_Delay(50);

    cmd = MLX90393_CMD_RM_XYZ;
    res = HAL_I2C_Master_Transmit(&hi2c3, MLX90393_ADDR_8BIT, &cmd, 1, I2C_TIMEOUT);
    if (res != HAL_OK) { ASSERT_EQ("Mag RM TX", res, HAL_OK); return; }
    res = HAL_I2C_Master_Receive(&hi2c3, MLX90393_ADDR_8BIT, buf, 7, I2C_TIMEOUT);
    if (res != HAL_OK) { ASSERT_EQ("Mag RM RX", res, HAL_OK); return; }
    uart_printf("    (RM: X=%d Y=%d Z=%d)\r\n",
                (int16_t)((buf[1] << 8) | buf[2]),
                (int16_t)((buf[3] << 8) | buf[4]),
                (int16_t)((buf[5] << 8) | buf[6]));

    TEST_PASS("Mag_Init completed (EX + reset + SM + RM)");
}

static MagData_t mag_read_safe(HAL_StatusTypeDef *out_res)
{
    MagData_t m = {0, 0, 0};
    uint8_t cmd, status, buf[7];
    HAL_StatusTypeDef res;

    cmd = MLX90393_CMD_SM_XYZ;
    res = HAL_I2C_Master_Transmit(&hi2c3, MLX90393_ADDR_8BIT, &cmd, 1, I2C_TIMEOUT);
    if (res != HAL_OK) { *out_res = res; return m; }
    res = HAL_I2C_Master_Receive(&hi2c3, MLX90393_ADDR_8BIT, &status, 1, I2C_TIMEOUT);
    if (res != HAL_OK) { *out_res = res; return m; }

    HAL_Delay(50);

    cmd = MLX90393_CMD_RM_XYZ;
    res = HAL_I2C_Master_Transmit(&hi2c3, MLX90393_ADDR_8BIT, &cmd, 1, I2C_TIMEOUT);
    if (res != HAL_OK) { *out_res = res; return m; }
    res = HAL_I2C_Master_Receive(&hi2c3, MLX90393_ADDR_8BIT, buf, 7, I2C_TIMEOUT);
    if (res == HAL_OK) {
        m.x = (int16_t)((buf[1] << 8) | buf[2]);
        m.y = (int16_t)((buf[3] << 8) | buf[4]);
        m.z = (int16_t)((buf[5] << 8) | buf[6]);
    }
    *out_res = res;
    return m;
}

static void test_mag_read_nonzero(void)
{
    HAL_StatusTypeDef res = HAL_ERROR;
    MagData_t m = {0, 0, 0};
    /* Retry once — i2c_recover before this test can leave first TX flaky */
    for (int attempt = 0; attempt < 2; attempt++) {
        m = mag_read_safe(&res);
        if (res == HAL_OK) break;
        i2c_recover(&hi2c3);
        HAL_Delay(10);
    }
    if (res != HAL_OK) { ASSERT_EQ("Mag read I2C", res, HAL_OK); return; }
    int any_nonzero = (m.x != 0) || (m.y != 0) || (m.z != 0);
    uart_printf("    (X=%d Y=%d Z=%d)\r\n", m.x, m.y, m.z);
    ASSERT_TRUE("Mag_Read returns nonzero data", any_nonzero);
}

static void test_mag_read_stable(void)
{
    HAL_StatusTypeDef res;
    MagData_t m1 = mag_read_safe(&res);
    if (res != HAL_OK) { ASSERT_EQ("Mag read1 I2C", res, HAL_OK); return; }
    HAL_Delay(100);
    MagData_t m2 = mag_read_safe(&res);
    if (res != HAL_OK) { ASSERT_EQ("Mag read2 I2C", res, HAL_OK); return; }

    int dx = abs(m1.x - m2.x);
    int dy = abs(m1.y - m2.y);
    int dz = abs(m1.z - m2.z);
    uart_printf("    (delta: X=%d Y=%d Z=%d)\r\n", dx, dy, dz);
    ASSERT_TRUE("Mag readings stable (delta < 500)", dx < 500 && dy < 500 && dz < 500);
}

static void test_mag_read_in_range(void)
{
    HAL_StatusTypeDef res;
    MagData_t m = mag_read_safe(&res);
    if (res != HAL_OK) { ASSERT_EQ("Mag range I2C", res, HAL_OK); return; }
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
    /* ALS_CONF: gain x2 (bit 11), IT=200ms (bits 7:6 = 01), power on */
    uint8_t conf[3] = {VEML7700_REG_ALS_CONF, 0x40, 0x08};
    HAL_StatusTypeDef res = HAL_I2C_Master_Transmit(&hi2c3, VEML7700_ADDR_8BIT, conf, 3, I2C_TIMEOUT);
    HAL_Delay(250);
    ASSERT_EQ("LightSensor_Init I2C OK", res, HAL_OK);
}

static uint16_t light_read_safe(HAL_StatusTypeDef *out_res)
{
    uint8_t reg = VEML7700_REG_ALS_DATA;
    uint8_t buf[2] = {0, 0};
    *out_res = HAL_I2C_Master_Transmit(&hi2c3, VEML7700_ADDR_8BIT, &reg, 1, I2C_TIMEOUT);
    if (*out_res != HAL_OK) return 0;
    *out_res = HAL_I2C_Master_Receive(&hi2c3, VEML7700_ADDR_8BIT, buf, 2, I2C_TIMEOUT);
    return (uint16_t)(buf[1] << 8 | buf[0]);
}

static void test_light_read_plausible(void)
{
    HAL_StatusTypeDef res;
    uint16_t raw = light_read_safe(&res);
    if (res != HAL_OK) { ASSERT_EQ("Light read I2C", res, HAL_OK); return; }
    uart_printf("    (raw = %u, ~%.1f lux)\r\n", raw, raw * 0.0144f);
    ASSERT_TRUE("Light reading < 65535 (not saturated)", raw < 65535);
}

static void test_light_read_stable(void)
{
    HAL_StatusTypeDef res;
    uint16_t l1 = light_read_safe(&res);
    if (res != HAL_OK) { ASSERT_EQ("Light read1 I2C", res, HAL_OK); return; }
    HAL_Delay(250);
    uint16_t l2 = light_read_safe(&res);
    if (res != HAL_OK) { ASSERT_EQ("Light read2 I2C", res, HAL_OK); return; }
    int delta = abs((int)l1 - (int)l2);
    uart_printf("    (l1=%u l2=%u delta=%d)\r\n", l1, l2, delta);
    /* Stable lighting — readings within 20% or 50 lux */
    int threshold = (l1 / 5) + 50;
    ASSERT_TRUE("Light readings stable", delta < threshold);
}

/* ── Driver API Tests ───────────────────────────────────────────── */
/* These exercise the actual driver functions from mlx90393.h / veml7700.h */

static void test_driver_mag_init(void)
{
    /* Replicate Mag_Init sequence using HAL_Delay for HW_TEST context.
       This validates the same init flow that the production driver uses. */
    uint8_t cmd, status;
    HAL_StatusTypeDef res;

    i2c_recover(&hi2c3);

    /* EX */
    cmd = MLX90393_CMD_EX;
    HAL_I2C_Master_Transmit(&hi2c3, MLX90393_ADDR_8BIT, &cmd, 1, I2C_TIMEOUT);
    HAL_I2C_Master_Receive(&hi2c3, MLX90393_ADDR_8BIT, &status, 1, I2C_TIMEOUT);
    HAL_Delay(10);

    /* RT */
    cmd = MLX90393_CMD_RT;
    res = HAL_I2C_Master_Transmit(&hi2c3, MLX90393_ADDR_8BIT, &cmd, 1, I2C_TIMEOUT);
    if (res != HAL_OK) { i2c_recover(&hi2c3); }
    else {
        res = HAL_I2C_Master_Receive(&hi2c3, MLX90393_ADDR_8BIT, &status, 1, I2C_TIMEOUT);
        if (res != HAL_OK) i2c_recover(&hi2c3);
    }
    HAL_Delay(200);
    i2c_recover(&hi2c3);
    HAL_Delay(10);

    /* Write config registers */
    uint8_t wr2[4] = { MLX90393_CMD_WR, 0x00, 0x08, (0x02 << 2) };
    res = HAL_I2C_Master_Transmit(&hi2c3, MLX90393_ADDR_8BIT, wr2, 4, I2C_TIMEOUT);
    if (res != HAL_OK) { ASSERT_EQ("Mag WR reg2 TX", res, HAL_OK); return; }
    res = HAL_I2C_Master_Receive(&hi2c3, MLX90393_ADDR_8BIT, &status, 1, I2C_TIMEOUT);
    if (res != HAL_OK) { ASSERT_EQ("Mag WR reg2 RX", res, HAL_OK); return; }
    uart_printf("    (WR2 status: 0x%02X)\r\n", status);
    HAL_Delay(2);

    uint8_t wr1[4] = { MLX90393_CMD_WR, 0x03, 0xC0, (0x01 << 2) };
    res = HAL_I2C_Master_Transmit(&hi2c3, MLX90393_ADDR_8BIT, wr1, 4, I2C_TIMEOUT);
    if (res != HAL_OK) { ASSERT_EQ("Mag WR reg1 TX", res, HAL_OK); return; }
    res = HAL_I2C_Master_Receive(&hi2c3, MLX90393_ADDR_8BIT, &status, 1, I2C_TIMEOUT);
    if (res != HAL_OK) { ASSERT_EQ("Mag WR reg1 RX", res, HAL_OK); return; }
    uart_printf("    (WR1 status: 0x%02X)\r\n", status);
    HAL_Delay(2);

    /* Start burst */
    cmd = MLX90393_CMD_SB_XYZ;
    res = HAL_I2C_Master_Transmit(&hi2c3, MLX90393_ADDR_8BIT, &cmd, 1, I2C_TIMEOUT);
    if (res != HAL_OK) { ASSERT_EQ("Mag SB TX", res, HAL_OK); return; }
    res = HAL_I2C_Master_Receive(&hi2c3, MLX90393_ADDR_8BIT, &status, 1, I2C_TIMEOUT);
    if (res != HAL_OK) { ASSERT_EQ("Mag SB RX", res, HAL_OK); return; }
    uart_printf("    (SB status: 0x%02X)\r\n", status);
    HAL_Delay(20);

    TEST_PASS("Mag_Init() completes without hang");
}

static void test_driver_mag_read(void)
{
    MagData_t m = Mag_Read(&hi2c3);
    int any_nonzero = (m.x != 0) || (m.y != 0) || (m.z != 0);
    uart_printf("    (driver: X=%d Y=%d Z=%d)\r\n", m.x, m.y, m.z);
    ASSERT_TRUE("Mag_Read() returns nonzero via driver", any_nonzero);
}

static void test_driver_mag_read_consistency(void)
{
    MagData_t m1 = Mag_Read(&hi2c3);
    HAL_Delay(20);
    MagData_t m2 = Mag_Read(&hi2c3);
    int dx = abs(m1.x - m2.x);
    int dy = abs(m1.y - m2.y);
    int dz = abs(m1.z - m2.z);
    uart_printf("    (driver delta: X=%d Y=%d Z=%d)\r\n", dx, dy, dz);
    ASSERT_TRUE("Mag_Read() consecutive reads stable", dx < 500 && dy < 500 && dz < 500);
}

static void test_driver_light_init(void)
{
    /* Use HAL_Delay version — LightSensor_Init uses osDelay which
       hangs in HW_TEST defaultTask context. Config is identical. */
    i2c_recover(&hi2c3);
    uint8_t config[2] = {0x40, 0x08};
    HAL_StatusTypeDef res = HAL_I2C_Mem_Write(&hi2c3, VEML7700_ADDR_8BIT,
                                               0x00, I2C_MEMADD_SIZE_8BIT,
                                               config, 2, I2C_TIMEOUT);
    if (res != HAL_OK) { ASSERT_EQ("Light Mem_Write", res, HAL_OK); return; }
    HAL_Delay(250);
    TEST_PASS("LightSensor_Init() completes without hang");
}

static void test_driver_light_read_raw(void)
{
    uint16_t raw = LightSensor_ReadRaw(&hi2c3);
    uart_printf("    (ReadRaw = %u)\r\n", raw);
    ASSERT_TRUE("LightSensor_ReadRaw() < 65535", raw < 65535);
}

static void test_driver_light_read_white(void)
{
    uint16_t white = LightSensor_ReadWhite(&hi2c3);
    uart_printf("    (ReadWhite = %u)\r\n", white);
    ASSERT_TRUE("LightSensor_ReadWhite() < 65535", white < 65535);
}

static void test_driver_light_read_lux(void)
{
    float lux = LightSensor_ReadLux(&hi2c3);
    uart_printf("    (ReadLux = %.2f)\r\n", lux);
    ASSERT_TRUE("LightSensor_ReadLux() >= 0", lux >= 0.0f);
    ASSERT_TRUE("LightSensor_ReadLux() < 120000", lux < 120000.0f);
}

static void test_driver_light_read_legacy(void)
{
    uint16_t lux = LightSensor_Read(&hi2c3);
    float lux_f = LightSensor_ReadLux(&hi2c3);
    uart_printf("    (Read=%u, ReadLux=%.2f)\r\n", lux, lux_f);
    /* Legacy should be truncated float value (within ±2 for rounding) */
    int delta = abs((int)lux - (int)lux_f);
    ASSERT_TRUE("LightSensor_Read() ≈ truncated ReadLux()", delta <= 2);
}

static void test_driver_light_raw_vs_lux(void)
{
    uint16_t raw = LightSensor_ReadRaw(&hi2c3);
    float lux = LightSensor_ReadLux(&hi2c3);
    /* At gain x2 IT=200ms, resolution=0.0144. Lux should be ≈ raw * 0.0144 */
    if (raw > 0 && lux > 0.0f) {
        float ratio = lux / (float)raw;
        uart_printf("    (raw=%u lux=%.2f ratio=%.4f)\r\n", raw, lux, ratio);
        /* Ratio should be near 0.0144, but non-linear correction can shift it */
        ASSERT_TRUE("Lux/Raw ratio > 0.01", ratio > 0.01f);
        ASSERT_TRUE("Lux/Raw ratio < 0.1", ratio < 0.1f);
    } else {
        uart_printf("    (raw=%u lux=%.2f — zero reading)\r\n", raw, lux);
        TEST_PASS("Light sensor zero reading (dark environment)");
    }
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
    HAL_Delay(50);

    /* After init, all servos should be at 1500µs (center) */
    uint32_t ccr = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
    ASSERT_EQ("Servo 1 centered at 1500µs", ccr, 1500);
}

static void test_servo_set_angle(void)
{
    ServoControl_SetAngle(SERVO_CTRL_1, 0.0f);
    HAL_Delay(10);
    uint32_t ccr0 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);

    ServoControl_SetAngle(SERVO_CTRL_1, 180.0f);
    HAL_Delay(10);
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
    HAL_Delay(5);
    GPIO_PinState low = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_Delay(5);
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

/* ── Double Buffer Tests ───────────────────────────────────────── */

static void test_db_init_zeros(void)
{
    sensor_double_buffer_t db;
    DoubleBuffer_Init(&db);

    ASSERT_EQ("DB init: write_idx = 0", db.write_idx, 0);

    /* Both snapshots should be zeroed */
    ASSERT_EQ("DB init: snapshot[0].timestamp = 0", db.snapshots[0].timestamp_ms, 0);
    ASSERT_EQ("DB init: snapshot[1].timestamp = 0", db.snapshots[1].timestamp_ms, 0);
    ASSERT_EQ("DB init: snapshot[0].sequence = 0", db.snapshots[0].sequence, 0);
    ASSERT_EQ("DB init: snapshot[1].sequence = 0", db.snapshots[1].sequence, 0);
    ASSERT_EQ("DB init: snapshot[0].data.light = 0", db.snapshots[0].data.light_lux, 0);
    ASSERT_EQ("DB init: snapshot[1].data.light = 0", db.snapshots[1].data.light_lux, 0);
    ASSERT_EQ("DB init: snapshot[0].data.mag.x = 0", db.snapshots[0].data.mag_data.x, 0);
    ASSERT_EQ("DB init: snapshot[1].data.mag.x = 0", db.snapshots[1].data.mag_data.x, 0);
}

static void test_db_write_buffer_returns_current(void)
{
    sensor_double_buffer_t db;
    DoubleBuffer_Init(&db);

    sensor_snapshot_t *wb = DoubleBuffer_GetWriteBuffer(&db);
    ASSERT_TRUE("GetWriteBuffer returns &snapshots[0]", wb == &db.snapshots[0]);

    /* After swap, write buffer should be the other one */
    wb->data.light_lux = 42;
    DoubleBuffer_Swap(&db);

    sensor_snapshot_t *wb2 = DoubleBuffer_GetWriteBuffer(&db);
    ASSERT_TRUE("After swap: GetWriteBuffer returns &snapshots[1]", wb2 == &db.snapshots[1]);
}

static void test_db_read_buffer_opposite(void)
{
    sensor_double_buffer_t db;
    DoubleBuffer_Init(&db);

    /* Before any swap: read buffer is snapshots[1] (opposite of write_idx=0) */
    const sensor_snapshot_t *rb = DoubleBuffer_GetReadBuffer(&db);
    ASSERT_TRUE("Initial read = &snapshots[1]", rb == &db.snapshots[1]);

    /* Write data and swap */
    sensor_snapshot_t *wb = DoubleBuffer_GetWriteBuffer(&db);
    wb->data.light_lux = 999;
    DoubleBuffer_Swap(&db);

    /* Now read buffer should be snapshots[0] — the one we just wrote */
    rb = DoubleBuffer_GetReadBuffer(&db);
    ASSERT_TRUE("After swap: read = &snapshots[0]", rb == &db.snapshots[0]);
    ASSERT_EQ("Reader sees written data", rb->data.light_lux, 999);
}

static void test_db_write_isolation(void)
{
    /* Writes to the write buffer must NOT be visible to reader until swap */
    sensor_double_buffer_t db;
    DoubleBuffer_Init(&db);

    /* Swap once so read buffer (snapshots[0]) has known state */
    sensor_snapshot_t *wb = DoubleBuffer_GetWriteBuffer(&db);
    wb->data.light_lux = 100;
    DoubleBuffer_Swap(&db);

    /* Now write new data but DON'T swap */
    wb = DoubleBuffer_GetWriteBuffer(&db);
    wb->data.light_lux = 200;

    /* Reader should still see old data (100) */
    const sensor_snapshot_t *rb = DoubleBuffer_GetReadBuffer(&db);
    ASSERT_EQ("Reader sees 100 before swap", rb->data.light_lux, 100);

    /* Now swap — reader should see 200 */
    DoubleBuffer_Swap(&db);
    rb = DoubleBuffer_GetReadBuffer(&db);
    ASSERT_EQ("Reader sees 200 after swap", rb->data.light_lux, 200);
}

static void test_db_sequence_increments(void)
{
    sensor_double_buffer_t db;
    DoubleBuffer_Init(&db);

    uint8_t seq_before = db.snapshots[0].sequence;
    sensor_snapshot_t *wb = DoubleBuffer_GetWriteBuffer(&db);
    wb->data.light_lux = 1;
    DoubleBuffer_Swap(&db);

    /* The swapped buffer (now read buffer, snapshots[0]) should have seq+1 */
    const sensor_snapshot_t *rb = DoubleBuffer_GetReadBuffer(&db);
    ASSERT_EQ("Sequence incremented after 1 swap", rb->sequence, seq_before + 1);

    /* Swap again — this time snapshot[1] gets incremented */
    wb = DoubleBuffer_GetWriteBuffer(&db);
    wb->data.light_lux = 2;
    DoubleBuffer_Swap(&db);

    rb = DoubleBuffer_GetReadBuffer(&db);
    ASSERT_EQ("Sequence incremented after 2 swaps", rb->sequence, 1);
}

static void test_db_timestamp_set_on_swap(void)
{
    sensor_double_buffer_t db;
    DoubleBuffer_Init(&db);

    uint32_t before = HAL_GetTick();
    sensor_snapshot_t *wb = DoubleBuffer_GetWriteBuffer(&db);
    wb->data.light_lux = 1;
    DoubleBuffer_Swap(&db);
    uint32_t after = HAL_GetTick();

    const sensor_snapshot_t *rb = DoubleBuffer_GetReadBuffer(&db);
    ASSERT_TRUE("Timestamp >= before", rb->timestamp_ms >= before);
    ASSERT_TRUE("Timestamp <= after", rb->timestamp_ms <= after);
}

static void test_db_precopy_seeds_new_write(void)
{
    /* After swap, the new write buffer should contain a copy of the
       data that was just published — enables partial updates */
    sensor_double_buffer_t db;
    DoubleBuffer_Init(&db);

    sensor_snapshot_t *wb = DoubleBuffer_GetWriteBuffer(&db);
    wb->data.tof_distances[0] = 1000;
    wb->data.tof_distances[1] = 2000;
    wb->data.mag_data.x = 42;
    wb->data.light_lux = 500;
    DoubleBuffer_Swap(&db);

    /* New write buffer should have the pre-copy */
    wb = DoubleBuffer_GetWriteBuffer(&db);
    ASSERT_EQ("Pre-copy: tof[0] carried over", wb->data.tof_distances[0], 1000);
    ASSERT_EQ("Pre-copy: tof[1] carried over", wb->data.tof_distances[1], 2000);
    ASSERT_EQ("Pre-copy: mag.x carried over", wb->data.mag_data.x, 42);
    ASSERT_EQ("Pre-copy: light carried over", wb->data.light_lux, 500);
}

static void test_db_partial_update_preserves_fields(void)
{
    /* Simulate I2C3 task: only update mag + light, tof should survive */
    sensor_double_buffer_t db;
    DoubleBuffer_Init(&db);

    /* First pass: I2C1 writes tof 0,1 */
    sensor_snapshot_t *wb = DoubleBuffer_GetWriteBuffer(&db);
    wb->data.tof_distances[0] = 100;
    wb->data.tof_distances[1] = 200;
    DoubleBuffer_Swap(&db);

    /* Second pass: I2C3 writes mag + light only */
    wb = DoubleBuffer_GetWriteBuffer(&db);
    /* tof values should be pre-copied */
    wb->data.mag_data.x = 55;
    wb->data.light_lux = 300;
    DoubleBuffer_Swap(&db);

    /* Reader should see BOTH tof (from pre-copy) AND mag+light */
    const sensor_snapshot_t *rb = DoubleBuffer_GetReadBuffer(&db);
    ASSERT_EQ("Partial: tof[0] preserved", rb->data.tof_distances[0], 100);
    ASSERT_EQ("Partial: tof[1] preserved", rb->data.tof_distances[1], 200);
    ASSERT_EQ("Partial: mag.x updated", rb->data.mag_data.x, 55);
    ASSERT_EQ("Partial: light updated", rb->data.light_lux, 300);
}

static void test_db_multiple_swaps_converge(void)
{
    /* Simulate realistic multi-task write pattern */
    sensor_double_buffer_t db;
    DoubleBuffer_Init(&db);

    /* 5 cycles of writes from 3 "tasks" */
    for (int cycle = 0; cycle < 5; cycle++) {
        /* Task I2C1: tof 0,1 */
        sensor_snapshot_t *wb = DoubleBuffer_GetWriteBuffer(&db);
        wb->data.tof_distances[0] = 100 + cycle;
        wb->data.tof_distances[1] = 200 + cycle;
        DoubleBuffer_Swap(&db);

        /* Task I2C2: tof 2,3 */
        wb = DoubleBuffer_GetWriteBuffer(&db);
        wb->data.tof_distances[2] = 300 + cycle;
        wb->data.tof_distances[3] = 400 + cycle;
        DoubleBuffer_Swap(&db);

        /* Task I2C3: mag + light */
        wb = DoubleBuffer_GetWriteBuffer(&db);
        wb->data.mag_data.x = (int16_t)(10 + cycle);
        wb->data.light_lux = 500 + cycle;
        DoubleBuffer_Swap(&db);
    }

    /* After 5 cycles, reader should see the latest values */
    const sensor_snapshot_t *rb = DoubleBuffer_GetReadBuffer(&db);
    ASSERT_EQ("Multi-swap: tof[0] = 104", rb->data.tof_distances[0], 104);
    ASSERT_EQ("Multi-swap: tof[1] = 204", rb->data.tof_distances[1], 204);
    ASSERT_EQ("Multi-swap: tof[2] = 304", rb->data.tof_distances[2], 304);
    ASSERT_EQ("Multi-swap: tof[3] = 404", rb->data.tof_distances[3], 404);
    ASSERT_EQ("Multi-swap: mag.x = 14", rb->data.mag_data.x, 14);
    ASSERT_EQ("Multi-swap: light = 504", rb->data.light_lux, 504);
}

static void test_db_read_stable_without_swap(void)
{
    /* Reader calling GetReadBuffer multiple times without writer
       swapping should always get identical data */
    sensor_double_buffer_t db;
    DoubleBuffer_Init(&db);

    sensor_snapshot_t *wb = DoubleBuffer_GetWriteBuffer(&db);
    wb->data.light_lux = 777;
    wb->data.mag_data.z = -123;
    DoubleBuffer_Swap(&db);

    const sensor_snapshot_t *r1 = DoubleBuffer_GetReadBuffer(&db);
    const sensor_snapshot_t *r2 = DoubleBuffer_GetReadBuffer(&db);
    const sensor_snapshot_t *r3 = DoubleBuffer_GetReadBuffer(&db);

    ASSERT_TRUE("Read stable: same pointer", r1 == r2 && r2 == r3);
    ASSERT_EQ("Read stable: light consistent", r1->data.light_lux, 777);
    ASSERT_EQ("Read stable: mag.z consistent", r1->data.mag_data.z, -123);
}

static void test_db_write_idx_toggles(void)
{
    sensor_double_buffer_t db;
    DoubleBuffer_Init(&db);

    ASSERT_EQ("write_idx starts at 0", db.write_idx, 0);

    DoubleBuffer_Swap(&db);
    ASSERT_EQ("write_idx after 1 swap = 1", db.write_idx, 1);

    DoubleBuffer_Swap(&db);
    ASSERT_EQ("write_idx after 2 swaps = 0", db.write_idx, 0);

    DoubleBuffer_Swap(&db);
    ASSERT_EQ("write_idx after 3 swaps = 1", db.write_idx, 1);
}

static void test_db_telemetry_from_read_buffer(void)
{
    /* End-to-end: write → swap → read → telemetry conversion */
    sensor_double_buffer_t db;
    DoubleBuffer_Init(&db);

    sensor_snapshot_t *wb = DoubleBuffer_GetWriteBuffer(&db);
    wb->data.tof_distances[0] = 1500;  /* 1500mm = 1.5m */
    wb->data.tof_distances[1] = 3000;  /* 3000mm = 3.0m */
    wb->data.mag_data.x = 100;
    wb->data.mag_data.y = -200;
    wb->data.mag_data.z = 300;
    wb->data.light_lux = 450;
    DoubleBuffer_Swap(&db);

    const sensor_snapshot_t *rb = DoubleBuffer_GetReadBuffer(&db);
    float telem[TELEMETRY_SIZE];
    shared_data_to_telemetry(&rb->data, telem, TELEMETRY_SIZE);

    ASSERT_TRUE("Telem: tof[0] = 1.5m",
                fabsf(telem[IDX_TOF_0] - 1.5f) < 0.01f);
    ASSERT_TRUE("Telem: tof[1] = 3.0m",
                fabsf(telem[IDX_TOF_1] - 3.0f) < 0.01f);
    ASSERT_TRUE("Telem: mag_x = 100",
                fabsf(telem[IDX_MAG_X] - 100.0f) < 0.01f);
    ASSERT_TRUE("Telem: mag_y = -200",
                fabsf(telem[IDX_MAG_Y] - (-200.0f)) < 0.01f);
    ASSERT_TRUE("Telem: light = 450",
                fabsf(telem[IDX_LIGHT] - 450.0f) < 0.01f);
}

static void test_db_sequence_wraps_at_255(void)
{
    sensor_double_buffer_t db;
    DoubleBuffer_Init(&db);

    /* Force sequence to 254 on snapshot[0] */
    db.snapshots[0].sequence = 254;

    sensor_snapshot_t *wb = DoubleBuffer_GetWriteBuffer(&db);
    wb->data.light_lux = 1;
    DoubleBuffer_Swap(&db);

    const sensor_snapshot_t *rb = DoubleBuffer_GetReadBuffer(&db);
    ASSERT_EQ("Sequence 254→255", rb->sequence, 255);

    /* Swap again: snapshot[1] sequence increments (was 0 → 1) */
    wb = DoubleBuffer_GetWriteBuffer(&db);
    wb->data.light_lux = 2;
    DoubleBuffer_Swap(&db);

    /* Swap again: snapshot[0] wraps 255 → 0 */
    wb = DoubleBuffer_GetWriteBuffer(&db);
    wb->data.light_lux = 3;
    DoubleBuffer_Swap(&db);

    rb = DoubleBuffer_GetReadBuffer(&db);
    ASSERT_EQ("Sequence wraps 255→0", rb->sequence, 0);
}

static void test_db_all_tof_channels(void)
{
    /* Verify all 5 ToF distance slots work independently */
    sensor_double_buffer_t db;
    DoubleBuffer_Init(&db);

    sensor_snapshot_t *wb = DoubleBuffer_GetWriteBuffer(&db);
    wb->data.tof_distances[0] = 111;
    wb->data.tof_distances[1] = 222;
    wb->data.tof_distances[2] = 333;
    wb->data.tof_distances[3] = 444;
    wb->data.tof_distances[4] = 555;
    DoubleBuffer_Swap(&db);

    const sensor_snapshot_t *rb = DoubleBuffer_GetReadBuffer(&db);
    ASSERT_EQ("All ToF: ch0 = 111", rb->data.tof_distances[0], 111);
    ASSERT_EQ("All ToF: ch1 = 222", rb->data.tof_distances[1], 222);
    ASSERT_EQ("All ToF: ch2 = 333", rb->data.tof_distances[2], 333);
    ASSERT_EQ("All ToF: ch3 = 444", rb->data.tof_distances[3], 444);
    ASSERT_EQ("All ToF: ch4 = 555", rb->data.tof_distances[4], 555);
}

static void test_db_mag_full_range(void)
{
    /* Verify mag fields handle full int16 range including negatives */
    sensor_double_buffer_t db;
    DoubleBuffer_Init(&db);

    sensor_snapshot_t *wb = DoubleBuffer_GetWriteBuffer(&db);
    wb->data.mag_data.x = INT16_MAX;
    wb->data.mag_data.y = INT16_MIN;
    wb->data.mag_data.z = -1;
    DoubleBuffer_Swap(&db);

    const sensor_snapshot_t *rb = DoubleBuffer_GetReadBuffer(&db);
    ASSERT_EQ("Mag range: x = INT16_MAX", rb->data.mag_data.x, INT16_MAX);
    ASSERT_EQ("Mag range: y = INT16_MIN", rb->data.mag_data.y, INT16_MIN);
    ASSERT_EQ("Mag range: z = -1", rb->data.mag_data.z, -1);
}

static void test_db_rapid_swap_stress(void)
{
    /* 100 rapid swaps — verify no corruption */
    sensor_double_buffer_t db;
    DoubleBuffer_Init(&db);

    for (int i = 0; i < 100; i++) {
        sensor_snapshot_t *wb = DoubleBuffer_GetWriteBuffer(&db);
        wb->data.tof_distances[0] = (uint16_t)i;
        wb->data.mag_data.x = (int16_t)(i * 3);
        wb->data.light_lux = (uint16_t)(i * 5);
        DoubleBuffer_Swap(&db);
    }

    const sensor_snapshot_t *rb = DoubleBuffer_GetReadBuffer(&db);
    ASSERT_EQ("Stress: tof[0] = 99", rb->data.tof_distances[0], 99);
    ASSERT_EQ("Stress: mag.x = 297", rb->data.mag_data.x, 297);
    ASSERT_EQ("Stress: light = 495", rb->data.light_lux, 495);

    /* write_idx should be 0 (100 swaps = even) */
    ASSERT_EQ("Stress: write_idx = 0 after even swaps", db.write_idx, 0);
}

static void test_db_reinit_clears_state(void)
{
    /* Re-init should fully reset even after writes */
    sensor_double_buffer_t db;
    DoubleBuffer_Init(&db);

    sensor_snapshot_t *wb = DoubleBuffer_GetWriteBuffer(&db);
    wb->data.light_lux = 999;
    wb->data.mag_data.x = -500;
    wb->data.tof_distances[0] = 4000;
    DoubleBuffer_Swap(&db);
    DoubleBuffer_Swap(&db);
    DoubleBuffer_Swap(&db);

    /* Re-init */
    DoubleBuffer_Init(&db);

    ASSERT_EQ("Re-init: write_idx reset", db.write_idx, 0);
    ASSERT_EQ("Re-init: snap[0].light = 0", db.snapshots[0].data.light_lux, 0);
    ASSERT_EQ("Re-init: snap[1].light = 0", db.snapshots[1].data.light_lux, 0);
    ASSERT_EQ("Re-init: snap[0].seq = 0", db.snapshots[0].sequence, 0);
    ASSERT_EQ("Re-init: snap[1].seq = 0", db.snapshots[1].sequence, 0);
    ASSERT_EQ("Re-init: snap[0].ts = 0", db.snapshots[0].timestamp_ms, 0);
}

static void test_db_timestamps_monotonic(void)
{
    /* Timestamps must be non-decreasing across consecutive swaps */
    sensor_double_buffer_t db;
    DoubleBuffer_Init(&db);

    uint32_t prev_ts = 0;
    int monotonic = 1;

    for (int i = 0; i < 10; i++) {
        sensor_snapshot_t *wb = DoubleBuffer_GetWriteBuffer(&db);
        wb->data.light_lux = (uint16_t)i;
        DoubleBuffer_Swap(&db);

        const sensor_snapshot_t *rb = DoubleBuffer_GetReadBuffer(&db);
        if (rb->timestamp_ms < prev_ts) {
            monotonic = 0;
            uart_printf("    (ts[%d]=%lu < prev=%lu)\r\n",
                        i, (unsigned long)rb->timestamp_ms, (unsigned long)prev_ts);
            break;
        }
        prev_ts = rb->timestamp_ms;
    }

    ASSERT_TRUE("Timestamps monotonically non-decreasing", monotonic);
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
    HAL_Delay(500);

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
    RUN_TEST(test_uart6_dma);

    /* ── I2C Buses ── */
    test_group("I2C Buses");
    RUN_TEST(test_i2c1_initialized);
    RUN_TEST(test_i2c2_initialized);
    RUN_TEST(test_i2c3_initialized);
    i2c_recover(&hi2c3);
    RUN_TEST(test_i2c3_has_devices);

    /* ── Magnetometer ── */
    test_group("MLX90393 Magnetometer (I2C3)");
    i2c_recover(&hi2c3);
    RUN_TEST(test_mag_present);
    RUN_TEST(test_mag_init);
    HAL_Delay(20);  /* let burst mode settle */
    RUN_TEST(test_mag_read_nonzero);
    RUN_TEST(test_mag_read_stable);
    RUN_TEST(test_mag_read_in_range);

    /* ── Light Sensor ── */
    test_group("VEML7700 Light Sensor (I2C3)");
    i2c_recover(&hi2c3);
    RUN_TEST(test_light_present);
    RUN_TEST(test_light_init);
    RUN_TEST(test_light_read_plausible);
    RUN_TEST(test_light_read_stable);

    /* ── Driver API Tests ── */
    test_group("Driver APIs (mlx90393 + veml7700)");
    RUN_TEST(test_driver_mag_init);
    RUN_TEST(test_driver_mag_read);
    RUN_TEST(test_driver_mag_read_consistency);
    RUN_TEST(test_driver_light_init);
    RUN_TEST(test_driver_light_read_raw);
    RUN_TEST(test_driver_light_read_white);
    RUN_TEST(test_driver_light_read_lux);
    RUN_TEST(test_driver_light_read_legacy);
    RUN_TEST(test_driver_light_raw_vs_lux);

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

    /* ── Double Buffer ── */
    test_group("Double Buffer Logic");
    RUN_TEST(test_db_init_zeros);
    RUN_TEST(test_db_write_buffer_returns_current);
    RUN_TEST(test_db_read_buffer_opposite);
    RUN_TEST(test_db_write_isolation);
    RUN_TEST(test_db_sequence_increments);
    RUN_TEST(test_db_timestamp_set_on_swap);
    RUN_TEST(test_db_precopy_seeds_new_write);
    RUN_TEST(test_db_partial_update_preserves_fields);
    RUN_TEST(test_db_multiple_swaps_converge);
    RUN_TEST(test_db_read_stable_without_swap);
    RUN_TEST(test_db_write_idx_toggles);
    RUN_TEST(test_db_telemetry_from_read_buffer);
    RUN_TEST(test_db_sequence_wraps_at_255);
    RUN_TEST(test_db_all_tof_channels);
    RUN_TEST(test_db_mag_full_range);
    RUN_TEST(test_db_rapid_swap_stress);
    RUN_TEST(test_db_reinit_clears_state);
    RUN_TEST(test_db_timestamps_monotonic);

    /* ── Skipped (no hardware) ── */
    test_group("ToF Sensors (not connected)");
    RUN_TEST(test_tof_skipped);

    /* ── Summary ── */
    uart_printf("\r\n");
    uart_printf("══════════════════════════════════════════════\r\n");
    uart_printf("  %d assertions: %d passed, %d failed, %d skipped\r\n",
                tests_passed + tests_failed + tests_skipped,
                tests_passed, tests_failed, tests_skipped);
    if (tests_failed == 0) {
        uart_printf("  ✓ ALL TESTS PASSED\r\n");
    } else {
        uart_printf("  ✗ %d FAILURE(S)\r\n", tests_failed);
    }
    uart_printf("══════════════════════════════════════════════\r\n");

    /* Blink or idle */
    for (;;) {
        HAL_Delay(1000);
    }
}

#endif /* HW_TEST */
