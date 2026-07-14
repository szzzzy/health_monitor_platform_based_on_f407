/**
  ******************************************************************************
  * @file    eeprom_driver.c
  * @brief   M24C02 EEPROM 驱动 — I2C 读写封装，含页边界处理
  ******************************************************************************
  */

#include "eeprom_driver.h"
#include "i2c.h"

/**
 * @brief  从 M24C02 连续读取字节。
 * @param  mem_addr 芯片内 8 位起始地址。
 * @param  data 目标缓冲，不可为 NULL。
 * @param  len 读取长度，必须非零且不得越过 256 字节容量。
 * @return HAL I2C 状态；参数或范围无效返回 HAL_ERROR。
 * @note   本函数不获取 I2C1 互斥量，调用层负责总线串行化。
 */
HAL_StatusTypeDef eeprom_read(uint8_t mem_addr, uint8_t *data, uint16_t len)
{
  if ((data == NULL) || (len == 0U))
  {
    return HAL_ERROR;
  }
  if (((uint16_t)mem_addr + len) > EEPROM_SIZE)
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Mem_Read(&hi2c1,
                          EEPROM_I2C_ADDR,
                          (uint16_t)mem_addr,
                          I2C_MEMADD_SIZE_8BIT,
                          data,
                          len,
                          EEPROM_TIMEOUT_MS);
}

/**
 * @brief  按 M24C02 的 8 字节页边界拆分并写入连续数据。
 * @param  mem_addr 芯片内 8 位起始地址。
 * @param  data 源缓冲，不可为 NULL。
 * @param  len 写入长度，必须非零且不得越过芯片容量。
 * @return 全部分片写入并完成 ACK 轮询返回 HAL_OK，否则返回首个 HAL 错误。
 * @note   跨页一次写会在 EEPROM 内部回卷并覆盖同页数据，因此每次只写到
 *         当前页末；每个分片后用设备就绪轮询等待内部写周期完成。
 */
HAL_StatusTypeDef eeprom_write(uint8_t mem_addr, const uint8_t *data, uint16_t len)
{
  uint16_t remaining = len;
  uint16_t offset    = 0U;
  uint16_t chunk;
  uint16_t space;
  HAL_StatusTypeDef status;

  if ((data == NULL) || (len == 0U))
  {
    return HAL_ERROR;
  }
  if (((uint16_t)mem_addr + len) > EEPROM_SIZE)
  {
    return HAL_ERROR;
  }

  while (remaining > 0U)
  {
    space = (uint16_t)EEPROM_PAGE_SIZE
            - (uint16_t)((mem_addr + offset) % EEPROM_PAGE_SIZE);
    chunk = (remaining < space) ? remaining : space;

    status = HAL_I2C_Mem_Write(&hi2c1,
                               EEPROM_I2C_ADDR,
                               (uint16_t)(mem_addr + offset),
                               I2C_MEMADD_SIZE_8BIT,
                               (uint8_t *)(data + offset),
                               chunk,
                               EEPROM_TIMEOUT_MS);
    if (status != HAL_OK)
    {
      return status;
    }

    status = HAL_I2C_IsDeviceReady(&hi2c1,
                                   EEPROM_I2C_ADDR,
                                   8U,
                                   EEPROM_WRITE_CYCLE_MS);
    if (status != HAL_OK)
    {
      return status;
    }

    offset    += chunk;
    remaining -= chunk;
  }

  return HAL_OK;
}
