/**
  ******************************************************************************
  * @file    eeprom_driver.c
  * @brief   M24C02 EEPROM 驱动 — I2C 读写封装，含页边界处理
  ******************************************************************************
  */

#include "eeprom_driver.h"
#include "i2c.h"

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

    HAL_Delay(EEPROM_WRITE_CYCLE_MS);

    offset    += chunk;
    remaining -= chunk;
  }

  return HAL_OK;
}
