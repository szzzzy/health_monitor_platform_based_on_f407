#ifndef __EEPROM_DRIVER_H__
#define __EEPROM_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* M24C02 7 位 I2C 地址为 0x50（A0=A1=A2=GND），HAL 使用左移后的 8 位地址 */
#define EEPROM_I2C_ADDR       ((uint8_t)(0x50U << 1))
#define EEPROM_PAGE_SIZE      8U
#define EEPROM_SIZE           256U
#define EEPROM_TIMEOUT_MS     10U
#define EEPROM_WRITE_CYCLE_MS 5U

HAL_StatusTypeDef eeprom_read(uint8_t mem_addr, uint8_t *data, uint16_t len);
HAL_StatusTypeDef eeprom_write(uint8_t mem_addr, const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_DRIVER_H__ */
