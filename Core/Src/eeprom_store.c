/**
  ******************************************************************************
  * @file    eeprom_store.c
  * @brief   EEPROM 参数管理 — 布局序列化/反序列化、CRC16 校验、
  *          版本迁移、出厂默认回退
  ******************************************************************************
  */

#include "eeprom_store.h"
#include "app_rtos.h"
#include "eeprom_driver.h"

#include <string.h>

#define EEPROM_MAGIC         EEPROM_MAGIC_VALUE
#define EEPROM_VERSION       EEPROM_LAYOUT_VERSION

/* Layout v2:
 * 0x00..0x07 header: magic, version, crc16
 * 0x08..0x29 identity and MAX30102 LED current settings
 * 0x2A..0x4F reserved for future layout-compatible fields
 * 0x50..0x5F runtime counters
 * 0x60..0x6F reserved for layout-compatible additions
 * 0x70..0xE9 error log ring, write pointer, total count
 * 0xEA..0xFF reserved tail, kept zero by serialization
 */
#define EEPROM_ADDR_SERIAL   0x08U
#define EEPROM_ADDR_HW_REV   0x20U
#define EEPROM_ADDR_MFG_DATE 0x24U
#define EEPROM_ADDR_LED_RED  0x28U
#define EEPROM_ADDR_LED_IR   0x29U
#define EEPROM_ADDR_RESERVED_A 0x2AU
#define EEPROM_RESERVED_A_SIZE 0x26U
#define EEPROM_ADDR_RUNTIME  0x50U
#define EEPROM_ADDR_RESERVED_B 0x60U
#define EEPROM_RESERVED_B_SIZE 0x10U
#define EEPROM_ADDR_ERRLOG   0x70U
#define EEPROM_ADDR_ERR_WRP  0xE8U
#define EEPROM_ADDR_ERR_CNT  0xE9U

#define ERROR_ENTRY_SIZE     15U
#define EEPROM_STORE_I2C_LOCK_TIMEOUT_MS 500U

eeprom_data_t g_eeprom;

/* ---- CRC16-CCITT (poly 0x1021, init 0xFFFF) ---- */
static uint16_t eeprom_crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;
  uint16_t i;
  uint8_t  j;

  for (i = 0U; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (j = 0U; j < 8U; j++)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      }
      else
      {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

/* ---- CRC8 (poly 0x07, init 0x00) ---- */
static uint8_t eeprom_crc8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0x00U;
  uint8_t i;
  uint8_t j;

  for (i = 0U; i < len; i++)
  {
    crc ^= data[i];
    for (j = 0U; j < 8U; j++)
    {
      if ((crc & 0x80U) != 0U)
      {
        crc = (uint8_t)((crc << 1) ^ 0x07U);
      }
      else
      {
        crc = (uint8_t)(crc << 1);
      }
    }
  }
  return crc;
}

/* ---- 默认值（在 Flash 中） ---- */
static const eeprom_data_t defaults = {
    .magic              = EEPROM_MAGIC,
    .version            = EEPROM_VERSION,
    .crc16              = 0U,
    .serial             = "BME-2026-00001",
    .hw_rev             = "V1.0",
    .mfg_date_ymd       = 0U,
    .max30102_led_red_pa = EEPROM_DEFAULT_LED_RED_PA,
    .max30102_led_ir_pa  = EEPROM_DEFAULT_LED_IR_PA,
    .total_run_hours_x10 = 0U,
    .total_boot_count    = 0U,
    .sensor_read_error_count = 0U,
    .sensor_recovery_count = 0U,
    .error_log           = {{0U, 0U, 0U, 0U, 0U}},
    .error_log_wr_ptr    = 0U,
    .error_log_count     = 0U,
    .initialized         = false
};

/* ---- 序列化 ---- */
static uint16_t eeprom_serialize(const eeprom_data_t *d, uint8_t buf[EEPROM_SIZE])
{
  uint16_t calc_crc;
  uint8_t i;

  (void)memset(buf, 0, EEPROM_SIZE);

  /* 魔数 (LE) */
  buf[0] = (uint8_t)(d->magic        & 0xFFU);
  buf[1] = (uint8_t)((d->magic >> 8)  & 0xFFU);
  buf[2] = (uint8_t)((d->magic >> 16) & 0xFFU);
  buf[3] = (uint8_t)((d->magic >> 24) & 0xFFU);

  /* 版本 */
  buf[4] = (uint8_t)(d->version & 0xFFU);
  buf[5] = (uint8_t)((d->version >> 8) & 0xFFU);

  /* 序列号 */
  for (i = 0U; i < sizeof(d->serial); i++)      { buf[EEPROM_ADDR_SERIAL + i]    = (uint8_t)d->serial[i]; }
  /* 硬件版本 */
  for (i = 0U; i < EEPROM_HW_REV_STORED_LEN; i++) { buf[EEPROM_ADDR_HW_REV + i]    = (uint8_t)d->hw_rev[i]; }
  /* 出厂日期 */
  buf[EEPROM_ADDR_MFG_DATE + 0] = (uint8_t)(d->mfg_date_ymd        & 0xFFU);
  buf[EEPROM_ADDR_MFG_DATE + 1] = (uint8_t)((d->mfg_date_ymd >> 8)  & 0xFFU);
  buf[EEPROM_ADDR_MFG_DATE + 2] = (uint8_t)((d->mfg_date_ymd >> 16) & 0xFFU);
  buf[EEPROM_ADDR_MFG_DATE + 3] = (uint8_t)((d->mfg_date_ymd >> 24) & 0xFFU);

  /* 校准 */
  buf[EEPROM_ADDR_LED_RED] = d->max30102_led_red_pa;
  buf[EEPROM_ADDR_LED_IR]  = d->max30102_led_ir_pa;
  for (i = 0U; i < EEPROM_RESERVED_A_SIZE; i++)
  {
    buf[EEPROM_ADDR_RESERVED_A + i] = 0U;
  }

  /* 运行统计 */
  buf[EEPROM_ADDR_RUNTIME + 0] = (uint8_t)(d->total_run_hours_x10        & 0xFFU);
  buf[EEPROM_ADDR_RUNTIME + 1] = (uint8_t)((d->total_run_hours_x10 >> 8)  & 0xFFU);
  buf[EEPROM_ADDR_RUNTIME + 2] = (uint8_t)((d->total_run_hours_x10 >> 16) & 0xFFU);
  buf[EEPROM_ADDR_RUNTIME + 3] = (uint8_t)((d->total_run_hours_x10 >> 24) & 0xFFU);
  buf[EEPROM_ADDR_RUNTIME + 4] = (uint8_t)(d->total_boot_count        & 0xFFU);
  buf[EEPROM_ADDR_RUNTIME + 5] = (uint8_t)((d->total_boot_count >> 8)  & 0xFFU);
  buf[EEPROM_ADDR_RUNTIME + 6] = (uint8_t)((d->total_boot_count >> 16) & 0xFFU);
  buf[EEPROM_ADDR_RUNTIME + 7] = (uint8_t)((d->total_boot_count >> 24) & 0xFFU);
  buf[EEPROM_ADDR_RUNTIME + 8] = (uint8_t)(d->sensor_read_error_count        & 0xFFU);
  buf[EEPROM_ADDR_RUNTIME + 9] = (uint8_t)((d->sensor_read_error_count >> 8)  & 0xFFU);
  buf[EEPROM_ADDR_RUNTIME + 10] = (uint8_t)((d->sensor_read_error_count >> 16) & 0xFFU);
  buf[EEPROM_ADDR_RUNTIME + 11] = (uint8_t)((d->sensor_read_error_count >> 24) & 0xFFU);
  buf[EEPROM_ADDR_RUNTIME + 12] = (uint8_t)(d->sensor_recovery_count        & 0xFFU);
  buf[EEPROM_ADDR_RUNTIME + 13] = (uint8_t)((d->sensor_recovery_count >> 8)  & 0xFFU);
  buf[EEPROM_ADDR_RUNTIME + 14] = (uint8_t)((d->sensor_recovery_count >> 16) & 0xFFU);
  buf[EEPROM_ADDR_RUNTIME + 15] = (uint8_t)((d->sensor_recovery_count >> 24) & 0xFFU);
  /* 0x60..0x6F is reserved for future layout-compatible fields. */
  for (i = 0U; i < EEPROM_RESERVED_B_SIZE; i++)
  {
    buf[EEPROM_ADDR_RESERVED_B + i] = 0U;
  }

  /* 错误日志 */
  for (i = 0U; i < EEPROM_ERROR_LOG_COUNT; i++)
  {
    uint8_t base = (uint8_t)(EEPROM_ADDR_ERRLOG + i * ERROR_ENTRY_SIZE);
    buf[base + 0] = d->error_log[i].type;
    buf[base + 1] = d->error_log[i].phase;
    buf[base + 2] = (uint8_t)(d->error_log[i].context & 0xFFU);
    buf[base + 3] = (uint8_t)((d->error_log[i].context >> 8) & 0xFFU);
    buf[base + 4] = (uint8_t)((d->error_log[i].context >> 16) & 0xFFU);
    buf[base + 5] = (uint8_t)((d->error_log[i].context >> 24) & 0xFFU);
    buf[base + 6] = (uint8_t)(d->error_log[i].run_hours_x10 & 0xFFU);
    buf[base + 7] = (uint8_t)((d->error_log[i].run_hours_x10 >> 8) & 0xFFU);
    buf[base + 8] = (uint8_t)((d->error_log[i].run_hours_x10 >> 16) & 0xFFU);
    buf[base + 9] = (uint8_t)((d->error_log[i].run_hours_x10 >> 24) & 0xFFU);
    buf[base + 10] = eeprom_crc8(&buf[base], 10U);
    buf[base + 11] = 0U;
    buf[base + 12] = 0U;
    buf[base + 13] = 0U;
    /* Reserved for future per-entry fields. */
    buf[base + 14] = 0U;
  }
  buf[EEPROM_ADDR_ERR_WRP] = d->error_log_wr_ptr;
  buf[EEPROM_ADDR_ERR_CNT] = d->error_log_count;

  /* CRC16 覆盖数据区 */
  calc_crc = eeprom_crc16(&buf[EEPROM_OFFSET_DATA], EEPROM_DATA_SIZE);
  buf[6] = (uint8_t)(calc_crc & 0xFFU);
  buf[7] = (uint8_t)((calc_crc >> 8) & 0xFFU);
  return calc_crc;
}

/* ---- 反序列化 —— */
static bool eeprom_deserialize(const uint8_t buf[EEPROM_SIZE], eeprom_data_t *d)
{
  uint16_t stored_crc;
  uint16_t calc_crc;
  uint8_t i;

  if (d == NULL)
  {
    return false;
  }

  stored_crc = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
  calc_crc   = eeprom_crc16(&buf[EEPROM_OFFSET_DATA], EEPROM_DATA_SIZE);

  if (stored_crc != calc_crc)
  {
    return false;
  }

  d->magic   = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8)
             | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
  d->version = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
  d->crc16   = stored_crc;

  for (i = 0U; i < sizeof(d->serial); i++)  { d->serial[i]  = (char)buf[EEPROM_ADDR_SERIAL + i]; }
  for (i = 0U; i < EEPROM_HW_REV_STORED_LEN; i++)  { d->hw_rev[i]  = (char)buf[EEPROM_ADDR_HW_REV + i]; }
  d->serial[sizeof(d->serial) - 1U] = '\0';
  d->hw_rev[EEPROM_HW_REV_STORED_LEN] = '\0';

  d->mfg_date_ymd = (uint32_t)buf[EEPROM_ADDR_MFG_DATE + 0]
                  | ((uint32_t)buf[EEPROM_ADDR_MFG_DATE + 1] << 8)
                  | ((uint32_t)buf[EEPROM_ADDR_MFG_DATE + 2] << 16)
                  | ((uint32_t)buf[EEPROM_ADDR_MFG_DATE + 3] << 24);

  d->max30102_led_red_pa = buf[EEPROM_ADDR_LED_RED];
  d->max30102_led_ir_pa = buf[EEPROM_ADDR_LED_IR];
  if (d->max30102_led_red_pa == 0U)
  {
    d->max30102_led_red_pa = EEPROM_DEFAULT_LED_RED_PA;
  }
  if (d->max30102_led_ir_pa == 0U)
  {
    d->max30102_led_ir_pa = EEPROM_DEFAULT_LED_IR_PA;
  }

  d->total_run_hours_x10 = (uint32_t)buf[EEPROM_ADDR_RUNTIME + 0]
                         | ((uint32_t)buf[EEPROM_ADDR_RUNTIME + 1] << 8)
                         | ((uint32_t)buf[EEPROM_ADDR_RUNTIME + 2] << 16)
                         | ((uint32_t)buf[EEPROM_ADDR_RUNTIME + 3] << 24);
  d->total_boot_count = (uint32_t)buf[EEPROM_ADDR_RUNTIME + 4]
                      | ((uint32_t)buf[EEPROM_ADDR_RUNTIME + 5] << 8)
                      | ((uint32_t)buf[EEPROM_ADDR_RUNTIME + 6] << 16)
                      | ((uint32_t)buf[EEPROM_ADDR_RUNTIME + 7] << 24);
  d->sensor_read_error_count = (uint32_t)buf[EEPROM_ADDR_RUNTIME + 8]
                      | ((uint32_t)buf[EEPROM_ADDR_RUNTIME + 9] << 8)
                      | ((uint32_t)buf[EEPROM_ADDR_RUNTIME + 10] << 16)
                      | ((uint32_t)buf[EEPROM_ADDR_RUNTIME + 11] << 24);
  d->sensor_recovery_count = (uint32_t)buf[EEPROM_ADDR_RUNTIME + 12]
                           | ((uint32_t)buf[EEPROM_ADDR_RUNTIME + 13] << 8)
                           | ((uint32_t)buf[EEPROM_ADDR_RUNTIME + 14] << 16)
                           | ((uint32_t)buf[EEPROM_ADDR_RUNTIME + 15] << 24);

  for (i = 0U; i < EEPROM_ERROR_LOG_COUNT; i++)
  {
    uint8_t base = (uint8_t)(EEPROM_ADDR_ERRLOG + i * ERROR_ENTRY_SIZE);
    uint8_t entry_crc;
    d->error_log[i].type = buf[base + 0];
    d->error_log[i].phase = buf[base + 1];
    d->error_log[i].context = (uint32_t)buf[base + 2]
                            | ((uint32_t)buf[base + 3] << 8)
                            | ((uint32_t)buf[base + 4] << 16)
                            | ((uint32_t)buf[base + 5] << 24);
    d->error_log[i].run_hours_x10 = (uint32_t)buf[base + 6]
                                  | ((uint32_t)buf[base + 7] << 8)
                                  | ((uint32_t)buf[base + 8] << 16)
                                  | ((uint32_t)buf[base + 9] << 24);
    d->error_log[i].crc8 = buf[base + 10];
    entry_crc = eeprom_crc8(&buf[base], 10U);
    if (entry_crc != d->error_log[i].crc8)
    {
      (void)memset(&d->error_log[i], 0, sizeof(d->error_log[i]));
    }
  }
  d->error_log_wr_ptr = buf[EEPROM_ADDR_ERR_WRP];
  d->error_log_count  = buf[EEPROM_ADDR_ERR_CNT];
  if (d->error_log_wr_ptr >= EEPROM_ERROR_LOG_COUNT)
  {
    d->error_log_wr_ptr = 0U;
  }

  d->initialized = true;
  return true;
}

/* ---- 写入 EEPROM ---- */
static HAL_StatusTypeDef eeprom_read_full(uint8_t buf[EEPROM_SIZE])
{
  HAL_StatusTypeDef status;

  if (app_rtos_i2c_acquire(EEPROM_STORE_I2C_LOCK_TIMEOUT_MS) == 0U)
  {
    return HAL_BUSY;
  }

  status = eeprom_read(0U, buf, EEPROM_SIZE);
  app_rtos_i2c_release();
  return status;
}

static bool eeprom_write_full(eeprom_data_t *d)
{
  uint8_t buf[EEPROM_SIZE];
  uint16_t crc16;
  HAL_StatusTypeDef status;

  crc16 = eeprom_serialize(d, buf);
  d->crc16 = crc16;
  if (app_rtos_i2c_acquire(EEPROM_STORE_I2C_LOCK_TIMEOUT_MS) == 0U)
  {
    return false;
  }

  status = eeprom_write(0U, buf, EEPROM_SIZE);
  app_rtos_i2c_release();
  return (status == HAL_OK);
}

/* ---- 公共 API ---- */

void eeprom_store_init(void)
{
  uint8_t buf[EEPROM_SIZE];
  uint32_t raw_magic;
  uint16_t raw_version;

  /* 尝试读取 EEPROM */
  if (eeprom_read_full(buf) != HAL_OK)
  {
    /* I2C 失败 → 使用默认值 */
    (void)memcpy(&g_eeprom, &defaults, sizeof(g_eeprom));
    g_eeprom.initialized = false;
    return;
  }

  raw_magic   = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8)
              | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
  raw_version = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);

  if ((raw_magic != EEPROM_MAGIC) || (raw_version != EEPROM_VERSION))
  {
    /* 空白芯片或版本不匹配 → 写入默认值 */
    (void)memcpy(&g_eeprom, &defaults, sizeof(g_eeprom));
    if (g_eeprom.total_boot_count < 0xFFFFFFFFUL)
      { g_eeprom.total_boot_count++; }
    g_eeprom.initialized = eeprom_write_full(&g_eeprom);
    return;
  }

  if (!eeprom_deserialize(buf, &g_eeprom))
  {
    /* CRC 失败 → 恢复默认值 */
    (void)memcpy(&g_eeprom, &defaults, sizeof(g_eeprom));
    if (g_eeprom.total_boot_count < 0xFFFFFFFFUL)
      { g_eeprom.total_boot_count++; }
    g_eeprom.initialized = eeprom_write_full(&g_eeprom);
    return;
  }

  /* 成功加载——递增启动计数 */
  if (g_eeprom.total_boot_count < 0xFFFFFFFFUL)
  {
    g_eeprom.total_boot_count++;
  }
  (void)eeprom_write_full(&g_eeprom);
}

bool eeprom_store_sync(void)
{
  bool ok = eeprom_write_full(&g_eeprom);
  if (ok)
  {
    g_eeprom.initialized = true;
  }
  return ok;
}

bool eeprom_store_reset_defaults(void)
{
  bool ok;

  (void)memcpy(&g_eeprom, &defaults, sizeof(g_eeprom));
  g_eeprom.initialized = true;
  ok = eeprom_write_full(&g_eeprom);
  g_eeprom.initialized = ok;
  return ok;
}

void eeprom_store_log_error(uint8_t type, uint8_t phase, uint32_t context)
{
  eeprom_error_entry_t *entry;
  uint8_t idx;

  if (!g_eeprom.initialized)
  {
    return;
  }

  if (g_eeprom.error_log_wr_ptr >= EEPROM_ERROR_LOG_COUNT)
  {
    g_eeprom.error_log_wr_ptr = 0U;
  }

  idx   = g_eeprom.error_log_wr_ptr;
  entry = &g_eeprom.error_log[idx];

  entry->type      = type;
  entry->phase     = phase;
  entry->context   = context;
  entry->run_hours_x10 = g_eeprom.total_run_hours_x10;

  g_eeprom.error_log_wr_ptr = (uint8_t)((idx + 1U) % EEPROM_ERROR_LOG_COUNT);
  if (g_eeprom.error_log_count < 0xFFU)
  {
    g_eeprom.error_log_count++;
  }

  (void)eeprom_write_full(&g_eeprom);
}

void eeprom_store_update_runtime(uint32_t run_hours_x10,
                                  uint32_t sensor_read_errors,
                                  uint32_t recoveries)
{
  if (!g_eeprom.initialized)
  {
    return;
  }

  g_eeprom.total_run_hours_x10   = run_hours_x10;
  g_eeprom.sensor_read_error_count = sensor_read_errors;
  g_eeprom.sensor_recovery_count = recoveries;
  (void)eeprom_write_full(&g_eeprom);
}
