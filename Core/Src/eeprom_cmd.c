/**
  ******************************************************************************
  * @file    eeprom_cmd.c
  * @brief   EEPROM UART CLI commands.
  ******************************************************************************
  */

#include "eeprom_cmd.h"
#include "app_rtos.h"
#include "eeprom_driver.h"
#include "eeprom_store.h"
#include "max30102.h"
#include "usart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RESPONSE_BUF_SIZE 384U
#define EEPROM_CMD_I2C_LOCK_TIMEOUT_MS 200U

static void cmd_send(const char *str)
{
  uint16_t len = (uint16_t)strlen(str);

  if (len > 0U)
  {
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)str, len, 100U);
  }
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2U, 100U);
}

static const char *skip_spaces(const char *s)
{
  while ((*s == ' ') || (*s == '\t'))
  {
    s++;
  }
  return s;
}

static bool keyword_match(const char *text, const char *keyword)
{
  size_t len;

  if ((text == NULL) || (keyword == NULL))
  {
    return false;
  }

  len = strlen(keyword);
  if (strncmp(text, keyword, len) != 0)
  {
    return false;
  }

  return ((text[len] == '\0') || (text[len] == ' ') || (text[len] == '\t'));
}

static bool parse_long_arg(const char *text, long *value)
{
  char *end;
  long parsed;

  if ((text == NULL) || (value == NULL))
  {
    return false;
  }

  text = skip_spaces(text);
  if (*text == '\0')
  {
    return false;
  }

  parsed = strtol(text, &end, 0);
  if (end == text)
  {
    return false;
  }

  end = (char *)skip_spaces(end);
  if (*end != '\0')
  {
    return false;
  }

  *value = parsed;
  return true;
}

static bool parse_ymd_arg(const char *text, uint32_t *ymd)
{
  static const uint8_t days_in_month[12] =
      {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};
  long value;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t max_day;

  if ((ymd == NULL) || (!parse_long_arg(text, &value)) ||
      (value < 20000101L) || (value > 20991231L))
  {
    return false;
  }

  year = (uint16_t)((uint32_t)value / 10000UL);
  month = (uint8_t)(((uint32_t)value / 100UL) % 100UL);
  day = (uint8_t)((uint32_t)value % 100UL);
  if ((month == 0U) || (month > 12U) || (day == 0U))
  {
    return false;
  }

  max_day = days_in_month[month - 1U];
  if ((month == 2U) &&
      (((year % 400U) == 0U) || (((year % 4U) == 0U) && ((year % 100U) != 0U))))
  {
    max_day = 29U;
  }

  if (day > max_day)
  {
    return false;
  }

  *ymd = (uint32_t)value;
  return true;
}

static bool parse_two_led_args(const char *args, uint8_t *red, uint8_t *ir)
{
  char *end;
  long parsed_red;
  long parsed_ir;

  if ((args == NULL) || (red == NULL) || (ir == NULL))
  {
    return false;
  }

  args = skip_spaces(args);
  parsed_red = strtol(args, &end, 0);
  if (end == args)
  {
    return false;
  }

  args = skip_spaces(end);
  parsed_ir = strtol(args, &end, 0);
  if (end == args)
  {
    return false;
  }

  end = (char *)skip_spaces(end);
  if ((*end != '\0') ||
      (parsed_red < 1L) || (parsed_red > 255L) ||
      (parsed_ir < 1L) || (parsed_ir > 255L))
  {
    return false;
  }

  *red = (uint8_t)parsed_red;
  *ir = (uint8_t)parsed_ir;
  return true;
}

static bool apply_led_settings_live(void)
{
  HAL_StatusTypeDef status;

  if (app_rtos_i2c_acquire(EEPROM_CMD_I2C_LOCK_TIMEOUT_MS) == 0U)
  {
    return false;
  }

  status = max30102_write_reg(MAX30102_REG_LED1_PA, g_eeprom.max30102_led_red_pa);
  if (status == HAL_OK)
  {
    status = max30102_write_reg(MAX30102_REG_LED2_PA, g_eeprom.max30102_led_ir_pa);
  }

  app_rtos_i2c_release();
  return (status == HAL_OK);
}

static void cmd_info(char *buf, uint16_t buf_size)
{
  (void)snprintf(buf, buf_size,
    "EEPROM Info:\r\n"
    "  Magic:   0x%08lX %s\r\n"
    "  Version: %u\r\n"
    "  CRC:     0x%04X %s\r\n"
    "  Init:    %s\r\n"
    "  Serial:  %.22s\r\n"
    "  HW Rev:  %.4s\r\n"
    "  MFG:     %08lu\r\n"
    "  LED:     RED=0x%02X IR=0x%02X\r\n"
    "  Boots:   %lu\r\n"
    "  Runtime: %lu.%uh\r\n"
    "  ReadErr: %lu\r\n"
    "  Recover: %lu\r\n"
    "  Errors:  %u",
    (unsigned long)g_eeprom.magic,
    (g_eeprom.magic == EEPROM_MAGIC_VALUE) ? "OK" : "BAD",
    (unsigned int)g_eeprom.version,
    (unsigned int)g_eeprom.crc16,
    g_eeprom.initialized ? "OK" : "BAD",
    g_eeprom.initialized ? "yes" : "no (defaults)",
    g_eeprom.serial,
    g_eeprom.hw_rev,
    (unsigned long)g_eeprom.mfg_date_ymd,
    (unsigned int)g_eeprom.max30102_led_red_pa,
    (unsigned int)g_eeprom.max30102_led_ir_pa,
    (unsigned long)g_eeprom.total_boot_count,
    (unsigned long)(g_eeprom.total_run_hours_x10 / 10U),
    (unsigned int)(g_eeprom.total_run_hours_x10 % 10U),
    (unsigned long)g_eeprom.sensor_read_error_count,
    (unsigned long)g_eeprom.sensor_recovery_count,
    (unsigned int)g_eeprom.error_log_count);
  cmd_send(buf);
}

static void cmd_dump(char *buf, uint16_t buf_size)
{
  uint8_t raw[EEPROM_SIZE];
  uint16_t offset;
  uint16_t pos;
  uint8_t i;

  if (app_rtos_i2c_acquire(EEPROM_CMD_I2C_LOCK_TIMEOUT_MS) == 0U)
  {
    cmd_send("EEPROM bus busy");
    return;
  }

  if (eeprom_read(0U, raw, EEPROM_SIZE) != HAL_OK)
  {
    app_rtos_i2c_release();
    cmd_send("EEPROM read failed");
    return;
  }
  app_rtos_i2c_release();

  cmd_send("EEPROM Dump (256 bytes):");
  for (offset = 0U; offset < EEPROM_SIZE; offset += 16U)
  {
    pos = (uint16_t)snprintf(buf, buf_size, "  %04X: ", (unsigned int)offset);
    for (i = 0U; (i < 16U) && (pos < (buf_size - 4U)); i++)
    {
      pos += (uint16_t)snprintf(buf + pos, buf_size - pos, "%02X ",
                                (unsigned int)raw[offset + i]);
    }
    buf[buf_size - 1U] = '\0';
    cmd_send(buf);
  }
}

static void cmd_reset(char *buf, uint16_t buf_size)
{
  (void)buf;
  (void)buf_size;

  if (!eeprom_store_reset_defaults())
  {
    cmd_send("EEPROM write failed during reset");
    return;
  }

  if (apply_led_settings_live())
  {
    cmd_send("EEPROM reset to defaults and applied");
  }
  else
  {
    cmd_send("EEPROM reset written; live LED apply failed");
  }
}

static void cmd_setsn(const char *args, char *buf, uint16_t buf_size)
{
  const char *sn;
  size_t len;

  sn = skip_spaces(args);
  len = strlen(sn);
  if (len == 0U)
  {
    cmd_send("Usage: eeprom setsn <serial>");
    return;
  }

  (void)memset(g_eeprom.serial, 0, sizeof(g_eeprom.serial));
  if (len >= sizeof(g_eeprom.serial))
  {
    len = sizeof(g_eeprom.serial) - 1U;
  }
  (void)memcpy(g_eeprom.serial, sn, len);
  g_eeprom.serial[len] = '\0';

  if (eeprom_store_sync())
  {
    (void)snprintf(buf, buf_size, "Serial set to: %.22s", g_eeprom.serial);
    cmd_send(buf);
  }
  else
  {
    cmd_send("EEPROM write failed");
  }
}

static void cmd_sethw(const char *args, char *buf, uint16_t buf_size)
{
  const char *rev;
  size_t len;

  rev = skip_spaces(args);
  len = strlen(rev);
  if ((len == 0U) || (len > EEPROM_HW_REV_STORED_LEN))
  {
    cmd_send("Usage: eeprom sethw <1..4 chars>");
    return;
  }

  (void)memset(g_eeprom.hw_rev, 0, sizeof(g_eeprom.hw_rev));
  (void)memcpy(g_eeprom.hw_rev, rev, len);
  g_eeprom.hw_rev[len] = '\0';

  if (eeprom_store_sync())
  {
    (void)snprintf(buf, buf_size, "HW Rev set to: %.4s", g_eeprom.hw_rev);
    cmd_send(buf);
  }
  else
  {
    cmd_send("EEPROM write failed");
  }
}

static void cmd_setmfg(const char *args, char *buf, uint16_t buf_size)
{
  uint32_t ymd;

  if (!parse_ymd_arg(args, &ymd))
  {
    cmd_send("Usage: eeprom setmfg <YYYYMMDD>");
    return;
  }

  g_eeprom.mfg_date_ymd = ymd;
  if (eeprom_store_sync())
  {
    (void)snprintf(buf, buf_size, "MFG date set to: %08lu", (unsigned long)ymd);
    cmd_send(buf);
  }
  else
  {
    cmd_send("EEPROM write failed");
  }
}

static void cmd_setled(const char *args, char *buf, uint16_t buf_size)
{
  uint8_t red;
  uint8_t ir;

  if (!parse_two_led_args(args, &red, &ir))
  {
    cmd_send("Usage: eeprom setled <red 1..255> <ir 1..255>");
    return;
  }

  g_eeprom.max30102_led_red_pa = red;
  g_eeprom.max30102_led_ir_pa = ir;
  if (!eeprom_store_sync())
  {
    cmd_send("EEPROM write failed");
    return;
  }

  if (apply_led_settings_live())
  {
    (void)snprintf(buf, buf_size, "LED set: RED=0x%02X IR=0x%02X",
                   (unsigned int)red, (unsigned int)ir);
  }
  else
  {
    (void)snprintf(buf, buf_size,
                   "LED stored: RED=0x%02X IR=0x%02X; live apply failed",
                   (unsigned int)red, (unsigned int)ir);
  }
  cmd_send(buf);
}

bool eeprom_cmd_process(AppState_t *app, const char *line)
{
  char buf[RESPONSE_BUF_SIZE];
  const char *cmd;

  (void)app;

  if (line == NULL)
  {
    return false;
  }

  line = skip_spaces(line);
  if (!keyword_match(line, "eeprom"))
  {
    return false;
  }

  cmd = skip_spaces(line + 6U);
  if (keyword_match(cmd, "info"))
  {
    cmd_info(buf, sizeof(buf));
  }
  else if (keyword_match(cmd, "dump"))
  {
    cmd_dump(buf, sizeof(buf));
  }
  else if (keyword_match(cmd, "reset"))
  {
    cmd_reset(buf, sizeof(buf));
  }
  else if (keyword_match(cmd, "setsn"))
  {
    cmd_setsn(skip_spaces(cmd + 5U), buf, sizeof(buf));
  }
  else if (keyword_match(cmd, "sethw"))
  {
    cmd_sethw(skip_spaces(cmd + 5U), buf, sizeof(buf));
  }
  else if (keyword_match(cmd, "setmfg"))
  {
    cmd_setmfg(skip_spaces(cmd + 6U), buf, sizeof(buf));
  }
  else if (keyword_match(cmd, "setled"))
  {
    cmd_setled(skip_spaces(cmd + 6U), buf, sizeof(buf));
  }
  else
  {
    cmd_send("eeprom: unknown cmd. Try: info dump reset setsn sethw setmfg setled");
  }

  return true;
}
