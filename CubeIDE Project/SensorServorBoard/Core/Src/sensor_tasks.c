#include "sensor_tasks.h"
#include "tof_manager.h"
#include "servo_manager.h"
#include "sensor_drivers.h"
#include "shared_data.h"
#include "double_buffer.h"
#include "i2c_common.h"
#include "cmsis_os.h"

// Define the GPIO Port and Pins for XSHUT
#define XSHUT_PORT_1_2 GPIOB
#define XSHUT_PIN_TOF1 GPIO_PIN_0
#define XSHUT_PIN_TOF2 GPIO_PIN_1

#define XSHUT_PORT_3_4 GPIOB
#define XSHUT_PIN_TOF3 GPIO_PIN_12
#define XSHUT_PIN_TOF4 GPIO_PIN_13

void StartI2C1Task(void *argument) {
    // 1. Reset sensors on I2C1 (Pull XSHUT LOW)
    HAL_GPIO_WritePin(XSHUT_PORT_1_2, XSHUT_PIN_TOF1 | XSHUT_PIN_TOF2, GPIO_PIN_RESET);
    osDelay(10); // Give them time to turn off

    // 2. Turn on ToF 1
    HAL_GPIO_WritePin(XSHUT_PORT_1_2, XSHUT_PIN_TOF1, GPIO_PIN_SET);
    osDelay(15);
    // 3. Init at default, then change ToF 1 address
    VL53L0X_Init(&hi2c1, TOF_DEFAULT_ADDR);
    VL53L0X_SetAddress(&hi2c1, TOF_DEFAULT_ADDR, TOF_1_ADDR);
    osDelay(20);

    // 4. Turn on ToF 2
    HAL_GPIO_WritePin(XSHUT_PORT_1_2, XSHUT_PIN_TOF2, GPIO_PIN_SET);
    osDelay(15);
    // 5. Init at default, then change ToF 2 address
    VL53L0X_Init(&hi2c1, TOF_DEFAULT_ADDR);
    VL53L0X_SetAddress(&hi2c1, TOF_DEFAULT_ADDR, TOF_2_ADDR);
    osDelay(20);

    for(;;) {
        uint16_t d1 = VL53L0X_ReadDistance(&hi2c1, TOF_1_ADDR);
        uint16_t d2 = VL53L0X_ReadDistance(&hi2c1, TOF_2_ADDR);
        
        sensor_snapshot_t *wb = DoubleBuffer_GetWriteBuffer(&g_sensor_db);
        wb->data.tof_distances[0] = d1;
        wb->data.tof_distances[1] = d2;
        DoubleBuffer_Swap(&g_sensor_db);
        
        osDelay(50); // Run at 20Hz
    }
}

void StartI2C2Task(void *argument) {
    // 1. Reset sensors on I2C2 (Pull XSHUT LOW)
    HAL_GPIO_WritePin(XSHUT_PORT_3_4, XSHUT_PIN_TOF3 | XSHUT_PIN_TOF4, GPIO_PIN_RESET);
    osDelay(10); // Give them time to turn off

    // 2. Turn on ToF 3
    HAL_GPIO_WritePin(XSHUT_PORT_3_4, XSHUT_PIN_TOF3, GPIO_PIN_SET);
    osDelay(15);
    VL53L0X_Init(&hi2c2, TOF_DEFAULT_ADDR);
    VL53L0X_SetAddress(&hi2c2, TOF_DEFAULT_ADDR, TOF_3_ADDR);
    osDelay(20);

    // 3. Turn on ToF 4
    HAL_GPIO_WritePin(XSHUT_PORT_3_4, XSHUT_PIN_TOF4, GPIO_PIN_SET);
    osDelay(15);
    VL53L0X_Init(&hi2c2, TOF_DEFAULT_ADDR);
    VL53L0X_SetAddress(&hi2c2, TOF_DEFAULT_ADDR, TOF_4_ADDR);
    osDelay(20);

    for(;;) {
        uint16_t d3 = VL53L0X_ReadDistance(&hi2c2, TOF_3_ADDR);
        uint16_t d4 = VL53L0X_ReadDistance(&hi2c2, TOF_4_ADDR);
        
        sensor_snapshot_t *wb = DoubleBuffer_GetWriteBuffer(&g_sensor_db);
        wb->data.tof_distances[2] = d3;
        wb->data.tof_distances[3] = d4;
        DoubleBuffer_Swap(&g_sensor_db);
        
        osDelay(50); // Run at 20Hz
    }
}

void StartI2C3Task(void *argument) {
    /* Initialize DMA synchronization semaphore */
    i2c3_dma_init();

    /* Initialize double buffer (before any sensor init) */
    DoubleBuffer_Init(&g_sensor_db);

    /* Mag + Light are on I2C3; ToF 5 not connected yet */
    Mag_Init(&hi2c3);
    LightSensor_Init(&hi2c3);

    /* Moving average buffers for magnetometer (4 samples) */
    #define MAG_AVG_N 4
    int16_t mag_x_buf[MAG_AVG_N] = {0};
    int16_t mag_y_buf[MAG_AVG_N] = {0};
    int16_t mag_z_buf[MAG_AVG_N] = {0};
    uint8_t mag_idx = 0;
    uint8_t mag_count = 0;  /* ramp up until buffer full */

    uint8_t light_divider = 0;  /* read light every 20th cycle (~200ms at 10ms loop) */

    for(;;) {
        MagData_t raw = Mag_Read(&hi2c3);

        /* Store into ring buffer */
        mag_x_buf[mag_idx] = raw.x;
        mag_y_buf[mag_idx] = raw.y;
        mag_z_buf[mag_idx] = raw.z;
        mag_idx = (mag_idx + 1) % MAG_AVG_N;
        if (mag_count < MAG_AVG_N) mag_count++;

        /* Compute average */
        int32_t sx = 0, sy = 0, sz = 0;
        for (uint8_t i = 0; i < mag_count; i++) {
            sx += mag_x_buf[i]; sy += mag_y_buf[i]; sz += mag_z_buf[i];
        }
        MagData_t mag;
        mag.x = (int16_t)(sx / mag_count);
        mag.y = (int16_t)(sy / mag_count);
        mag.z = (int16_t)(sz / mag_count);

        /* Read light every 20th cycle (~200ms matches integration time) */
        sensor_snapshot_t *wb = DoubleBuffer_GetWriteBuffer(&g_sensor_db);
        uint16_t lux = wb->data.light_lux;  /* keep previous */
        if (++light_divider >= 20) {
            light_divider = 0;
            lux = LightSensor_Read(&hi2c3);
        }

        wb->data.mag_data = mag;
        wb->data.light_lux = lux;
        DoubleBuffer_Swap(&g_sensor_db);

        osDelay(10);  /* ~100Hz read rate (burst mode — no conversion wait) */
    }
}

void StartServoTask(void *argument) {
    // 1. Initialize PWM Timers
    Servo_Init();
    
    float angle = 0.0f;
    float step = 5.0f;

    // 2. Main servo control loop
    for(;;) {
        // Example Sweep
        Servo_SetAngle(SERVO_1, angle);
        Servo_SetAngle(SERVO_2, angle);
        Servo_SetAngle(SERVO_3, angle);
        Servo_SetAngle(SERVO_4, angle);
        Servo_SetAngle(SERVO_5, angle);
        
        angle += step;
        if (angle >= 180.0f || angle <= 0.0f) {
            step = -step;
        }
        
        osDelay(20); // 50Hz update rate
    }
}
