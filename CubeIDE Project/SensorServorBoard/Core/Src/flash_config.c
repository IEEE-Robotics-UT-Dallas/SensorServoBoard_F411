#include "flash_config.h"
#include <string.h>

static uint32_t CalculateCRC(const BoardConfig_t *config) {
    uint32_t crc = 0;
    const uint32_t *ptr = (const uint32_t *)config;
    /* CRC everything except the CRC field itself */
    for (size_t i = 0; i < (sizeof(BoardConfig_t) / 4) - 1; i++) {
        crc ^= ptr[i];
    }
    return crc;
}

void Flash_GetDefaultConfig(BoardConfig_t *config) {
    memset(config, 0, sizeof(BoardConfig_t));
    config->magic = CONFIG_MAGIC;
    config->version = CONFIG_VERSION;
    /* Default offsets from your previous calibrated values */
    config->servo_offsets[0] = 0.0f;   // We zeroed this for testing
    config->servo_offsets[1] = 87.0f;
    config->servo_offsets[2] = 0.0f;
    config->servo_offsets[3] = 0.0f;
    config->servo_offsets[4] = 0.0f;
    config->crc = CalculateCRC(config);
}

HAL_StatusTypeDef Flash_LoadConfig(BoardConfig_t *config) {
    memcpy(config, (void *)CONFIG_FLASH_ADDR, sizeof(BoardConfig_t));

    if (config->magic != CONFIG_MAGIC) {
        return HAL_ERROR;
    }

    if (config->crc != CalculateCRC(config)) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef Flash_SaveConfig(const BoardConfig_t *config) {
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error;
    HAL_StatusTypeDef status;

    BoardConfig_t config_to_save = *config;
    config_to_save.crc = CalculateCRC(&config_to_save);

    HAL_FLASH_Unlock();

    /* Erase Sector */
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase_init.Sector = CONFIG_FLASH_SECTOR;
    erase_init.NbSectors = 1;

    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }

    /* Program Word by Word */
    uint32_t *data_ptr = (uint32_t *)&config_to_save;
    for (size_t i = 0; i < sizeof(BoardConfig_t) / 4; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CONFIG_FLASH_ADDR + (i * 4), data_ptr[i]);
        if (status != HAL_OK) break;
    }

    HAL_FLASH_Lock();
    return status;
}
