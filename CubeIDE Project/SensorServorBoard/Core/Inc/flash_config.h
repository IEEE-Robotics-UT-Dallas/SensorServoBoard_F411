#ifndef FLASH_CONFIG_H
#define FLASH_CONFIG_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define CONFIG_MAGIC 0x53534234  // "SSB4" (Sensor Servo Board 411)
#define CONFIG_VERSION 1

typedef struct {
    uint32_t magic;
    uint32_t version;
    float servo_offsets[5];
    uint32_t crc;
} BoardConfig_t;

/* Use Sector 7 (0x08060000) - 128KB on F411 */
#define CONFIG_FLASH_ADDR 0x08060000
#define CONFIG_FLASH_SECTOR FLASH_SECTOR_7

HAL_StatusTypeDef Flash_LoadConfig(BoardConfig_t *config);
HAL_StatusTypeDef Flash_SaveConfig(const BoardConfig_t *config);
void Flash_GetDefaultConfig(BoardConfig_t *config);

#endif /* FLASH_CONFIG_H */
