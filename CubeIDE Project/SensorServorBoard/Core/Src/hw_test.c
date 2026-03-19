/*
 * hw_test.c — Interactive hardware test for SensorServoBoard F411
 *
 * Build with -DHW_TEST (and without -DMICRO_ROS_ENABLED) to use USART1
 * as a debug console. Tests: I2C scan, ToF sensors, magnetometer,
 * light sensor, servo sweep, and GPIO/XSHUT pins.
 *
 * Connect a USB-UART adapter to PB6(TX)/PB7(RX) at 500000 baud.
 */

#ifdef HW_TEST

#include "main.h"
#include "sensor_drivers.h"
#include "servo_control.h"
#include "tof_manager.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

/* Printf redirect over UART */
static void uart_print(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
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

static uint8_t uart_getchar(uint32_t timeout_ms)
{
    uint8_t ch = 0;
    HAL_UART_Receive(&huart1, &ch, 1, timeout_ms);
    return ch;
}

/* ---- I2C Bus Scan ---- */
static void test_i2c_scan(I2C_HandleTypeDef *hi2c, const char *name)
{
    uart_printf("\r\n--- I2C Scan: %s ---\r\n", name);
    uart_printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\r\n");

    int found = 0;
    for (uint8_t row = 0; row < 8; row++) {
        uart_printf("%02X: ", row << 4);
        for (uint8_t col = 0; col < 16; col++) {
            uint8_t addr = (row << 4) | col;
            if (addr < 0x03 || addr > 0x77) {
                uart_print("   ");
                continue;
            }
            HAL_StatusTypeDef res = HAL_I2C_IsDeviceReady(hi2c, addr << 1, 1, 10);
            if (res == HAL_OK) {
                uart_printf("%02X ", addr);
                found++;
            } else {
                uart_print("-- ");
            }
        }
        uart_print("\r\n");
    }
    uart_printf("Found %d device(s)\r\n", found);
}

/* ---- XSHUT GPIO Test ---- */
static void test_xshut_pins(void)
{
    uart_printf("\r\n--- XSHUT Pin Test ---\r\n");
    uart_printf("Toggling XSHUT pins (PB0, PB1, PB12, PB13)...\r\n");

    const struct { GPIO_TypeDef *port; uint16_t pin; const char *name; } xshut[] = {
        { GPIOB, GPIO_PIN_0,  "PB0  (ToF1 XSHUT)" },
        { GPIOB, GPIO_PIN_1,  "PB1  (ToF2 XSHUT)" },
        { GPIOB, GPIO_PIN_12, "PB12 (ToF3 XSHUT)" },
        { GPIOB, GPIO_PIN_13, "PB13 (ToF4 XSHUT)" },
    };

    for (int i = 0; i < 4; i++) {
        /* Pull LOW (sensor off) */
        HAL_GPIO_WritePin(xshut[i].port, xshut[i].pin, GPIO_PIN_RESET);
        osDelay(50);
        GPIO_PinState low = HAL_GPIO_ReadPin(xshut[i].port, xshut[i].pin);

        /* Pull HIGH (sensor on) */
        HAL_GPIO_WritePin(xshut[i].port, xshut[i].pin, GPIO_PIN_SET);
        osDelay(50);
        GPIO_PinState high = HAL_GPIO_ReadPin(xshut[i].port, xshut[i].pin);

        uart_printf("  %s: LOW=%d HIGH=%d %s\r\n",
                     xshut[i].name, low, high,
                     (low == 0 && high == 1) ? "[OK]" : "[FAIL]");
    }
}

/* ---- Servo Sweep Test ---- */
static void test_servos(void)
{
    uart_printf("\r\n--- Servo Sweep Test ---\r\n");
    uart_printf("Sweeping all 5 servos: 0 -> 90 -> 180 -> 90\r\n");

    ServoControl_Init();

    const float angles[] = { 0.0f, 90.0f, 180.0f, 90.0f };
    for (int a = 0; a < 4; a++) {
        uart_printf("  Angle: %.0f deg ... ", angles[a]);
        for (int s = 0; s < 5; s++) {
            ServoControl_SetAngle((ServoCtrl_ID_t)s, angles[a]);
        }
        uart_printf("set\r\n");
        osDelay(800);
    }
    uart_printf("Servos at 90 (center)\r\n");
}

/* ---- Individual Servo Test ---- */
static void test_servo_individual(void)
{
    uart_printf("\r\n--- Individual Servo Test ---\r\n");
    uart_printf("Testing each servo one at a time...\r\n");

    ServoControl_Init();
    const char *names[] = {"PA1/TIM2_CH2", "PA2/TIM2_CH3", "PA3/TIM2_CH4",
                           "PA5/TIM2_CH1", "PA6/TIM3_CH1"};

    for (int s = 0; s < 5; s++) {
        uart_printf("  Servo %d (%s): 0->180->90 ", s + 1, names[s]);
        ServoControl_SetAngle((ServoCtrl_ID_t)s, 0.0f);
        osDelay(500);
        ServoControl_SetAngle((ServoCtrl_ID_t)s, 180.0f);
        osDelay(500);
        ServoControl_SetAngle((ServoCtrl_ID_t)s, 90.0f);
        osDelay(300);
        uart_printf("[DONE]\r\n");
    }
}

/* ---- ToF Sensor Read Test ---- */
static void test_tof_sensors(void)
{
    uart_printf("\r\n--- ToF Sensor Test ---\r\n");

    /* First scan I2C buses to see what's there */
    test_i2c_scan(&hi2c1, "I2C1 (PB8/PB9) - ToF 1,2");
    test_i2c_scan(&hi2c2, "I2C2 (PB10/PB3) - ToF 3,4");
    test_i2c_scan(&hi2c3, "I2C3 (PA8/PB4) - ToF 5, Mag, Light");

    uart_printf("\r\nInitializing ToF sensors...\r\n");
    ToF_Init_All();
    osDelay(100);

    uart_printf("Reading distances (5 samples each):\r\n");
    const char *tof_names[] = {"ToF1(I2C1)", "ToF2(I2C1)", "ToF3(I2C2)",
                                "ToF4(I2C2)", "ToF5(I2C3)"};
    for (int sample = 0; sample < 5; sample++) {
        uart_printf("  Sample %d: ", sample + 1);
        for (int t = 0; t < 5; t++) {
            uint16_t dist = ToF_ReadDistance((ToF_ID_t)t);
            uart_printf("%s=%4dmm ", tof_names[t], dist);
        }
        uart_printf("\r\n");
        osDelay(100);
    }
}

/* ---- Magnetometer Test ---- */
static void test_magnetometer(void)
{
    uart_printf("\r\n--- Magnetometer Test (QMC5883L on I2C3) ---\r\n");

    /* Check if device responds */
    if (HAL_I2C_IsDeviceReady(&hi2c3, QMC5883L_ADDR_8BIT, 3, 50) != HAL_OK) {
        uart_printf("  [FAIL] QMC5883L not found at 0x%02X\r\n", QMC5883L_ADDR_8BIT >> 1);
        return;
    }
    uart_printf("  [OK] QMC5883L found at 0x%02X\r\n", QMC5883L_ADDR_8BIT >> 1);

    Mag_Init(&hi2c3);
    osDelay(50);

    uart_printf("  Reading 5 samples:\r\n");
    for (int i = 0; i < 5; i++) {
        MagData_t m = Mag_Read(&hi2c3);
        uart_printf("    X=%6d  Y=%6d  Z=%6d\r\n", m.x, m.y, m.z);
        osDelay(100);
    }
}

/* ---- Light Sensor Test ---- */
static void test_light_sensor(void)
{
    uart_printf("\r\n--- Light Sensor Test (BH1750 on I2C3) ---\r\n");

    if (HAL_I2C_IsDeviceReady(&hi2c3, BH1750_ADDR_8BIT, 3, 50) != HAL_OK) {
        uart_printf("  [FAIL] BH1750 not found at 0x%02X\r\n", BH1750_ADDR_8BIT >> 1);
        return;
    }
    uart_printf("  [OK] BH1750 found at 0x%02X\r\n", BH1750_ADDR_8BIT >> 1);

    LightSensor_Init(&hi2c3);
    osDelay(200);

    uart_printf("  Reading 5 samples:\r\n");
    for (int i = 0; i < 5; i++) {
        uint16_t lux = LightSensor_Read(&hi2c3);
        uart_printf("    Light: %5d lux\r\n", lux);
        osDelay(200);
    }
}

/* ---- Main Test Menu ---- */
void HW_Test_Run(void)
{
    osDelay(500);

    uart_printf("\r\n");
    uart_printf("============================================\r\n");
    uart_printf("  SensorServoBoard F411 — Hardware Test\r\n");
    uart_printf("  UART: 500000 baud on PB6(TX)/PB7(RX)\r\n");
    uart_printf("============================================\r\n");

    for (;;) {
        uart_printf("\r\n");
        uart_printf("  [1] I2C scan (all buses)\r\n");
        uart_printf("  [2] XSHUT GPIO toggle\r\n");
        uart_printf("  [3] Servo sweep (all)\r\n");
        uart_printf("  [4] Servo test (individual)\r\n");
        uart_printf("  [5] ToF sensors read\r\n");
        uart_printf("  [6] Magnetometer read\r\n");
        uart_printf("  [7] Light sensor read\r\n");
        uart_printf("  [a] Run ALL tests\r\n");
        uart_printf("  [r] Repeat last\r\n");
        uart_printf("> ");

        static uint8_t last_choice = '1';
        uint8_t ch = uart_getchar(30000);
        if (ch == 0) continue;
        if (ch == 'r' || ch == 'R') ch = last_choice;
        else last_choice = ch;

        uart_printf("%c\r\n", ch);

        switch (ch) {
            case '1':
                test_i2c_scan(&hi2c1, "I2C1 (PB8/PB9)");
                test_i2c_scan(&hi2c2, "I2C2 (PB10/PB3)");
                test_i2c_scan(&hi2c3, "I2C3 (PA8/PB4)");
                break;
            case '2': test_xshut_pins(); break;
            case '3': test_servos(); break;
            case '4': test_servo_individual(); break;
            case '5': test_tof_sensors(); break;
            case '6': test_magnetometer(); break;
            case '7': test_light_sensor(); break;
            case 'a': case 'A':
                test_xshut_pins();
                test_i2c_scan(&hi2c1, "I2C1 (PB8/PB9)");
                test_i2c_scan(&hi2c2, "I2C2 (PB10/PB3)");
                test_i2c_scan(&hi2c3, "I2C3 (PA8/PB4)");
                test_tof_sensors();
                test_magnetometer();
                test_light_sensor();
                test_servos();
                break;
            default:
                uart_printf("Unknown command: '%c'\r\n", ch);
                break;
        }
    }
}

#endif /* HW_TEST */
