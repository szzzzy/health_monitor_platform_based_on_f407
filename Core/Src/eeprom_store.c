/**
  ******************************************************************************
  * @file    eeprom_store.c
  * @brief   EEPROM 参数管理 — 布局序列化/反序列化、CRC16 校验、
  *          版本检查、出厂默认回退与延迟写回
  *
  * RAM 中的 g_eeprom 是运行期镜像。落盘采用“数据区先写、8 字节头后写”的
  * 两阶段提交；若中途掉电，旧头与新数据的 CRC 不匹配，下次启动会回退默认值，
  * 不会把半新半旧内容当作有效布局。测量活跃时写请求只标记 dirty，Uitask
  * 在安全窗口调用延迟服务，避免 EEPROM 页写周期占用共享 I2C1。
  ******************************************************************************
  */

#include "eeprom_store.h"
#include "app_rtos.h"
#include "eeprom_driver.h"

#include <string.h>

#define EEPROM_MAGIC         EEPROM_MAGIC_VALUE
#define EEPROM_VERSION       EEPROM_LAYOUT_VERSION

/* 布局 v2：
 * 0x00..0x07 头部：魔数、版本、CRC16
 * 0x08..0x29 设备身份与 MAX30102 LED 电流设置
 * 0x2A..0x4F 预留的布局兼容字段
 * 0x50..0x5F 运行统计计数器
 * 0x60..0x6F 预留的布局兼容字段
 * 0x70..0xE9 错误日志环、写指针与累计条数
 * 0xEA..0xFF 尾部预留，序列化时保持为零
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
#define EEPROM_STORE_RETRY_BACKOFF_MS     5000U

eeprom_data_t g_eeprom;                    /* EEPROM 内容的唯一 RAM 镜像 */
static bool eeprom_measurement_active;      /* 为 true 时禁止物理 EEPROM 写入 */
static bool eeprom_dirty;                   /* RAM 镜像尚未成功持久化 */
static uint32_t eeprom_retry_deadline;      /* 写失败后的最早重试时刻，单位：ms */
static uint32_t eeprom_write_error_count;   /* 实际 I2C 写失败累计次数 */

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
  /* 0x60..0x6F 为未来可兼容扩展字段预留。 */
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
    /* 单条错误记录的尾部保留给未来扩展。 */
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

  if (eeprom_measurement_active)
  {
    eeprom_dirty = true;
    return false;
  }

  crc16 = eeprom_serialize(d, buf);
  d->crc16 = crc16;
  if (app_rtos_i2c_acquire(EEPROM_STORE_I2C_LOCK_TIMEOUT_MS) == 0U)
  {
    return false;
  }

  /* 两阶段提交：先写数据区，最后写含魔数/版本/CRC 的头部。若末次 8 字节写入前
   * 掉电，头部与数据区会 CRC 不匹配，从而拒绝半完成镜像。 */
  status = eeprom_write(EEPROM_OFFSET_DATA,
                        &buf[EEPROM_OFFSET_DATA],
                        EEPROM_DATA_SIZE);
  if (status == HAL_OK)
  {
    status = eeprom_write(0U, buf, EEPROM_OFFSET_DATA);
  }
  app_rtos_i2c_release();
  if (status != HAL_OK)
  {
    eeprom_dirty = true;
    eeprom_retry_deadline = HAL_GetTick() + EEPROM_STORE_RETRY_BACKOFF_MS;
    eeprom_write_error_count++;
    return false;
  }

  eeprom_dirty = false;
  return true;
}

/* ---- 公共 API：启动装载、显式同步、错误环和测量期延迟写回 ---- */

void eeprom_store_init(void)
{
  uint8_t buf[EEPROM_SIZE];
  uint32_t raw_magic;
  uint16_t raw_version;

  eeprom_measurement_active = false;
  eeprom_dirty = false;
  eeprom_retry_deadline = 0UL;
  eeprom_write_error_count = 0UL;

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
  if (!eeprom_write_full(&g_eeprom))
  {
    g_eeprom.initialized = false;
  }
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
  g_eeprom.initialized = true;
  return ok;
}

bool eeprom_store_log_error(uint8_t type, uint8_t phase, uint32_t context)
{
  eeprom_error_entry_t *entry;
  uint8_t idx;

  if (!g_eeprom.initialized)
  {
    return false;
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

  return eeprom_write_full(&g_eeprom);
}

bool eeprom_store_update_runtime(uint32_t run_hours_x10,
                                 uint32_t sensor_read_errors,
                                 uint32_t recoveries)
{
  if (!g_eeprom.initialized)
  {
    return false;
  }

  g_eeprom.total_run_hours_x10   = run_hours_x10;
  g_eeprom.sensor_read_error_count = sensor_read_errors;
  g_eeprom.sensor_recovery_count = recoveries;
  return eeprom_write_full(&g_eeprom);
}

void eeprom_store_set_measurement_active(bool active)
{
  eeprom_measurement_active = active;
}

bool eeprom_store_service_deferred(void)
{
  bool ok;

  if (!eeprom_dirty)
  {
    return true;
  }
  if (eeprom_measurement_active)
  {
    return false;
  }
  if ((eeprom_retry_deadline != 0UL) &&
      ((int32_t)(HAL_GetTick() - eeprom_retry_deadline) < 0))
  {
    return false;
  }

  eeprom_retry_deadline = 0UL;
  ok = eeprom_write_full(&g_eeprom);
  if (ok)
  {
    g_eeprom.initialized = true;
  }
  return ok;
}

uint32_t eeprom_store_get_write_error_count(void)
{
  return eeprom_write_error_count;
}
