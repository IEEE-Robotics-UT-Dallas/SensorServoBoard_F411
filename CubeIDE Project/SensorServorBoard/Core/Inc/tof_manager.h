#ifndef TOF_MANAGER_H_
#define TOF_MANAGER_H_

#include "main.h"

// I2C Handles (to be defined in main.c by CubeMX)
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

// ToF Sensor IDs
typedef enum {
    TOF_1 = 0, // I2C1
    TOF_2 = 1, // I2C1
    TOF_3 = 2, // I2C2
    TOF_4 = 3, // I2C2
    TOF_5 = 4  // I2C3
} ToF_ID_t;

// Default VL53L0X/VL53L1X address (8-bit shifted)
#define TOF_DEFAULT_ADDR 0x52

// New unique addresses for the sensors on shared buses
#define TOF_1_ADDR 0x54
#define TOF_2_ADDR 0x56
#define TOF_3_ADDR 0x58
#define TOF_4_ADDR 0x5A
#define TOF_5_ADDR 0x5C // Or leave default if it's the only ToF on I2C3

// Boot sequence and initialization
void ToF_Init_All(void);

// Read distance placeholder
uint16_t ToF_ReadDistance(ToF_ID_t tof_id);

#endif /* TOF_MANAGER_H_ */
