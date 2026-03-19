#ifndef SENSOR_DRIVERS_H_
#define SENSOR_DRIVERS_H_

/*
 * Umbrella header — includes all individual sensor drivers.
 * Existing code that includes "sensor_drivers.h" continues to work unchanged.
 */

#include "i2c_common.h"
#include "vl53l0x.h"
#include "mlx90393.h"
#include "veml7700.h"

/* I2C Handles (defined in main.c) */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

#endif /* SENSOR_DRIVERS_H_ */
