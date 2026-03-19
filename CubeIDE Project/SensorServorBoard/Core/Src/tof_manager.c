#include "tof_manager.h"
#include "sensor_drivers.h"

// Define the GPIO Port and Pins for XSHUT (Update with your exact labels from CubeMX)
#define XSHUT_PORT_1_2 GPIOB
#define XSHUT_PIN_TOF1 GPIO_PIN_0
#define XSHUT_PIN_TOF2 GPIO_PIN_1

#define XSHUT_PORT_3_4 GPIOB
#define XSHUT_PIN_TOF3 GPIO_PIN_12
#define XSHUT_PIN_TOF4 GPIO_PIN_13

// Note: ToF 5 does not have an XSHUT pin assigned in the prompt.
// We assume it's hardwired or uses the default address since it's the only ToF on I2C3.

void ToF_Init_All(void) {
    // 1. Reset all sensors (Pull XSHUT LOW)
    HAL_GPIO_WritePin(XSHUT_PORT_1_2, XSHUT_PIN_TOF1 | XSHUT_PIN_TOF2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(XSHUT_PORT_3_4, XSHUT_PIN_TOF3 | XSHUT_PIN_TOF4, GPIO_PIN_RESET);
    HAL_Delay(10); // Give them time to turn off

    // --- I2C1 BOOT SEQUENCE ---
    // 2. Turn on ToF 1
    HAL_GPIO_WritePin(XSHUT_PORT_1_2, XSHUT_PIN_TOF1, GPIO_PIN_SET);
    HAL_Delay(5); // Wait for boot
    // 3. Change ToF 1 Address
    VL53L0X_SetAddress(&hi2c1, TOF_DEFAULT_ADDR, TOF_1_ADDR);
    VL53L0X_Init(&hi2c1, TOF_1_ADDR);

    // 4. Turn on ToF 2
    HAL_GPIO_WritePin(XSHUT_PORT_1_2, XSHUT_PIN_TOF2, GPIO_PIN_SET);
    HAL_Delay(5);
    // 5. Change ToF 2 Address
    VL53L0X_SetAddress(&hi2c1, TOF_DEFAULT_ADDR, TOF_2_ADDR);
    VL53L0X_Init(&hi2c1, TOF_2_ADDR);

    // --- I2C2 BOOT SEQUENCE ---
    // 6. Turn on ToF 3
    HAL_GPIO_WritePin(XSHUT_PORT_3_4, XSHUT_PIN_TOF3, GPIO_PIN_SET);
    HAL_Delay(5);
    VL53L0X_SetAddress(&hi2c2, TOF_DEFAULT_ADDR, TOF_3_ADDR);
    VL53L0X_Init(&hi2c2, TOF_3_ADDR);

    // 7. Turn on ToF 4
    HAL_GPIO_WritePin(XSHUT_PORT_3_4, XSHUT_PIN_TOF4, GPIO_PIN_SET);
    HAL_Delay(5);
    VL53L0X_SetAddress(&hi2c2, TOF_DEFAULT_ADDR, TOF_4_ADDR);
    VL53L0X_Init(&hi2c2, TOF_4_ADDR);

    // --- I2C3 BOOT SEQUENCE ---
    // ToF 5 (and Mag/Light sensor) are on I2C3. 
    // If ToF 5 is the only ToF on this bus, it can remain at TOF_DEFAULT_ADDR.
    VL53L0X_Init(&hi2c3, TOF_DEFAULT_ADDR);
}

uint16_t ToF_ReadDistance(ToF_ID_t tof_id) {
    switch (tof_id) {
        case TOF_1: return VL53L0X_ReadDistance(&hi2c1, TOF_1_ADDR);
        case TOF_2: return VL53L0X_ReadDistance(&hi2c1, TOF_2_ADDR);
        case TOF_3: return VL53L0X_ReadDistance(&hi2c2, TOF_3_ADDR);
        case TOF_4: return VL53L0X_ReadDistance(&hi2c2, TOF_4_ADDR);
        case TOF_5: return VL53L0X_ReadDistance(&hi2c3, TOF_DEFAULT_ADDR);
        default: return 0xFFFF;
    }
}
