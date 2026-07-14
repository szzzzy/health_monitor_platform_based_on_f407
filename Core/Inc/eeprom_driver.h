#ifndef __EEPROM_DRIVER_H__
#define __EEPROM_DRIVER_H__

/**
 * @file eeprom_driver.h
 * @brief M24C02 低层 I2C 读写接口；页拆分由驱动完成，互斥由调用层负责。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* M24C02 7 位 I2C 地址为 0x50（A0=A1=A2=GND），HAL 使用左移后的 8 位地址 */
#define EEPROM_I2C_ADDR       ((uint8_t)(0x50U << 1))
#define EEPROM_PAGE_SIZE      8U    /* 物理页大小，单位：字节 */
#define EEPROM_SIZE           256U  /* 可寻址总容量，单位：字节 */
#define EEPROM_TIMEOUT_MS     10U   /* 单次 HAL I2C 传输超时，单位：ms */
#define EEPROM_WRITE_CYCLE_MS 5U    /* 单次就绪探测等待时间，单位：ms */

/** @brief 连续读取；参数无效、越界或 I2C 失败时返回非 HAL_OK。 */
HAL_StatusTypeDef eeprom_read(uint8_t mem_addr, uint8_t *data, uint16_t len);
/** @brief 按页边界拆分连续写入，并在每页后等待器件完成内部写周期。 */
HAL_StatusTypeDef eeprom_write(uint8_t mem_addr, const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_DRIVER_H__ */
