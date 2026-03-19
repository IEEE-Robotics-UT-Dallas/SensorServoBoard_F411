#include "sensor_tasks.h"
#include "tof_manager.h"
#include "servo_manager.h"
#include "sensor_drivers.h"
#include "shared_data.h"
#include "cmsis_os.h" // FreeRTOS API

// Globals to store sensor readings
shared_sensor_data_t g_sensor_data;
osMutexId_t sensor_data_mutex;

// Define the GPIO Port and Pins for XSHUT
#define XSHUT_PORT_1_2 GPIOB
#define XSHUT_PIN_TOF1 GPIO_PIN_0
#define XSHUT_PIN_TOF2 GPIO_PIN_1

#define XSHUT_PORT_3_4 GPIOB
#define XSHUT_PIN_TOF3 GPIO_PIN_12
#define XSHUT_PIN_TOF4 GPIO_PIN_13

void StartI2C1Task(void *argument) {
    // Create mutex if not already created
    if (sensor_data_mutex == NULL) {
        const osMutexAttr_t mutex_attr = {
            .name = "sensorDataMutex",
            .attr_bits = osMutexRecursive,
            .cb_mem = NULL,
            .cb_size = 0U
        };
        sensor_data_mutex = osMutexNew(&mutex_attr);
    }

    // 1. Reset sensors on I2C1 (Pull XSHUT LOW)
    HAL_GPIO_WritePin(XSHUT_PORT_1_2, XSHUT_PIN_TOF1 | XSHUT_PIN_TOF2, GPIO_PIN_RESET);
    osDelay(10); // Give them time to turn off

    // 2. Turn on ToF 1
    HAL_GPIO_WritePin(XSHUT_PORT_1_2, XSHUT_PIN_TOF1, GPIO_PIN_SET);
    osDelay(5); // Wait for boot
    // 3. Change ToF 1 Address
    VL53L0X_SetAddress(&hi2c1, TOF_DEFAULT_ADDR, TOF_1_ADDR);
    VL53L0X_Init(&hi2c1, TOF_1_ADDR);

    // 4. Turn on ToF 2
    HAL_GPIO_WritePin(XSHUT_PORT_1_2, XSHUT_PIN_TOF2, GPIO_PIN_SET);
    osDelay(5);
    // 5. Change ToF 2 Address
    VL53L0X_SetAddress(&hi2c1, TOF_DEFAULT_ADDR, TOF_2_ADDR);
    VL53L0X_Init(&hi2c1, TOF_2_ADDR);

    for(;;) {
        uint16_t d1 = VL53L0X_ReadDistance(&hi2c1, TOF_1_ADDR);
        uint16_t d2 = VL53L0X_ReadDistance(&hi2c1, TOF_2_ADDR);
        
        if (sensor_data_mutex != NULL) {
            osMutexAcquire(sensor_data_mutex, osWaitForever);
            g_sensor_data.tof_distances[0] = d1;
            g_sensor_data.tof_distances[1] = d2;
            osMutexRelease(sensor_data_mutex);
        }
        
        osDelay(50); // Run at 20Hz
    }
}

void StartI2C2Task(void *argument) {
    // 1. Reset sensors on I2C2 (Pull XSHUT LOW)
    HAL_GPIO_WritePin(XSHUT_PORT_3_4, XSHUT_PIN_TOF3 | XSHUT_PIN_TOF4, GPIO_PIN_RESET);
    osDelay(10); // Give them time to turn off

    // 2. Turn on ToF 3
    HAL_GPIO_WritePin(XSHUT_PORT_3_4, XSHUT_PIN_TOF3, GPIO_PIN_SET);
    osDelay(5);
    VL53L0X_SetAddress(&hi2c2, TOF_DEFAULT_ADDR, TOF_3_ADDR);
    VL53L0X_Init(&hi2c2, TOF_3_ADDR);

    // 3. Turn on ToF 4
    HAL_GPIO_WritePin(XSHUT_PORT_3_4, XSHUT_PIN_TOF4, GPIO_PIN_SET);
    osDelay(5);
    VL53L0X_SetAddress(&hi2c2, TOF_DEFAULT_ADDR, TOF_4_ADDR);
    VL53L0X_Init(&hi2c2, TOF_4_ADDR);

    for(;;) {
        uint16_t d3 = VL53L0X_ReadDistance(&hi2c2, TOF_3_ADDR);
        uint16_t d4 = VL53L0X_ReadDistance(&hi2c2, TOF_4_ADDR);
        
        if (sensor_data_mutex != NULL) {
            osMutexAcquire(sensor_data_mutex, osWaitForever);
            g_sensor_data.tof_distances[2] = d3;
            g_sensor_data.tof_distances[3] = d4;
            osMutexRelease(sensor_data_mutex);
        }
        
        osDelay(50); // Run at 20Hz
    }
}

void StartI2C3Task(void *argument) {
    /* Create mutex if not already created (I2C1 task may be disabled) */
    if (sensor_data_mutex == NULL) {
        const osMutexAttr_t mutex_attr = {
            .name = "sensorDataMutex",
            .attr_bits = osMutexRecursive,
            .cb_mem = NULL,
            .cb_size = 0U
        };
        sensor_data_mutex = osMutexNew(&mutex_attr);
    }

    /* Mag + Light are on I2C3; ToF 5 not connected yet */
    Mag_Init(&hi2c3);
    LightSensor_Init(&hi2c3);

    for(;;) {
        MagData_t mag = Mag_Read(&hi2c3);
        uint16_t lux = LightSensor_Read(&hi2c3);

        if (sensor_data_mutex != NULL) {
            osMutexAcquire(sensor_data_mutex, osWaitForever);
            g_sensor_data.mag_data = mag;
            g_sensor_data.light_lux = lux;
            osMutexRelease(sensor_data_mutex);
        }

        osDelay(50);
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
