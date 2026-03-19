#ifndef MLX90393_H_
#define MLX90393_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* MLX90393 I2C Address (8-bit) */
#define MLX90393_ADDR_8BIT    0x18  /* 0x0C << 1 */

/* MLX90393 Commands */
#define MLX90393_CMD_RT       0xF0  /* Reset */
#define MLX90393_CMD_EX       0x80  /* Exit mode */
#define MLX90393_CMD_SB_XYZ   0x1E  /* Start burst measurement XYZ */
#define MLX90393_CMD_SM_XYZ   0x3E  /* Start single measurement XYZ */
#define MLX90393_CMD_RM_XYZ   0x4E  /* Read measurement XYZ */
#define MLX90393_CMD_WR       0x60  /* Write register */
#define MLX90393_CMD_RR       0x50  /* Read register */

/* Magnetometer data */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} MagData_t;

/**
 * @brief  Initialize MLX90393 in burst mode (continuous XYZ at ~200Hz).
 *         Configures OSR=0, DIG_FILT=2 for ~5ms conversion time.
 */
void Mag_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Read latest XYZ measurement from burst mode.
 * @return MagData_t with x, y, z fields (raw counts). Returns {0,0,0} on error.
 */
MagData_t Mag_Read(I2C_HandleTypeDef *hi2c);

#endif /* MLX90393_H_ */
