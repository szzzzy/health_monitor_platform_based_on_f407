/**
  ******************************************************************************
  * @file    app_settings.c
  * @brief   OLED settings pages for EEPROM-backed device data.
  ******************************************************************************
  */

#include "app_settings.h"
#include "eeprom_store.h"
#include "ssd1306.h"

#include <stdio.h>

static void draw_lines(const char line[8][24])
{
  ssd1306_Clear(SSD1306_COLOR_BLACK);
  ssd1306_DrawString(0, 0,  line[0]);
  ssd1306_DrawString(0, 8,  line[1]);
  ssd1306_DrawString(0, 16, line[2]);
  ssd1306_DrawString(0, 24, line[3]);
  ssd1306_DrawString(0, 32, line[4]);
  ssd1306_DrawString(0, 40, line[5]);
  ssd1306_DrawString(0, 48, line[6]);
  ssd1306_DrawString(0, 56, line[7]);
  ssd1306_UpdateScreen();
}

static const char *error_type_name(uint8_t type)
{
  switch (type)
  {
  case EEPROM_ERR_HARDFAULT:      return "HF";
  case EEPROM_ERR_MAX30102_FAIL:  return "M30";
  case EEPROM_ERR_SD_FAIL:        return "SD";
  case EEPROM_ERR_ECG_LEAD_OFF:   return "LdO";
  case EEPROM_ERR_I2C_BUS:        return "I2C";
  case EEPROM_ERR_SENSOR_STALE:   return "STL";
  case EEPROM_ERR_OLED_FAIL:      return "OLE";
  case EEPROM_ERR_CRASH_RECORD:   return "CRS";
  default:                        return "--";
  }
}

static void render_identity(const AppState_t *app)
{
  char line[8][24];
  uint32_t mfg = g_eeprom.mfg_date_ymd;
  uint16_t y;
  uint16_t m;
  uint16_t d;

  (void)app;
  y = (uint16_t)(mfg / 10000UL);
  m = (uint16_t)((mfg / 100UL) % 100UL);
  d = (uint16_t)(mfg % 100UL);

  (void)snprintf(line[0], sizeof(line[0]), "SET:Identity 1/4");
  (void)snprintf(line[1], sizeof(line[1]), "SN:%.22s", g_eeprom.serial);
  (void)snprintf(line[2], sizeof(line[2]), "HW:%.4s", g_eeprom.hw_rev);
  if (mfg != 0U)
  {
    (void)snprintf(line[3], sizeof(line[3]), "MFG:%04u-%02u-%02u",
                   (unsigned int)y, (unsigned int)m, (unsigned int)d);
  }
  else
  {
    (void)snprintf(line[3], sizeof(line[3]), "MFG:--");
  }
  (void)snprintf(line[4], sizeof(line[4]), "EEProm:%s",
                 g_eeprom.initialized ? "OK" : "DFL");
  (void)snprintf(line[5], sizeof(line[5]), "Boots:%lu",
                 (unsigned long)g_eeprom.total_boot_count);
  (void)snprintf(line[6], sizeof(line[6]), "Ver:%u",
                 (unsigned int)g_eeprom.version);
  (void)snprintf(line[7], sizeof(line[7]), "");

  draw_lines(line);
}

static void render_calibrate(const AppState_t *app)
{
  char line[8][24];
  uint16_t red_ma_x10;
  uint16_t ir_ma_x10;

  (void)app;
  red_ma_x10 = (uint16_t)g_eeprom.max30102_led_red_pa * 2U;
  ir_ma_x10 = (uint16_t)g_eeprom.max30102_led_ir_pa * 2U;

  (void)snprintf(line[0], sizeof(line[0]), "SET:Calibrate 2/4");
  (void)snprintf(line[1], sizeof(line[1]), "LED_RED:0x%02X",
                 (unsigned int)g_eeprom.max30102_led_red_pa);
  (void)snprintf(line[2], sizeof(line[2]), "LED_IR :0x%02X",
                 (unsigned int)g_eeprom.max30102_led_ir_pa);
  (void)snprintf(line[3], sizeof(line[3]), "RED:%u.%umA",
                 (unsigned int)(red_ma_x10 / 10U),
                 (unsigned int)(red_ma_x10 % 10U));
  (void)snprintf(line[4], sizeof(line[4]), "IR :%u.%umA",
                 (unsigned int)(ir_ma_x10 / 10U),
                 (unsigned int)(ir_ma_x10 % 10U));
  (void)snprintf(line[5], sizeof(line[5]), "");
  (void)snprintf(line[6], sizeof(line[6]), "");
  (void)snprintf(line[7], sizeof(line[7]), "");

  draw_lines(line);
}

static void render_runtime(const AppState_t *app)
{
  char line[8][24];
  uint32_t hours;
  uint8_t tenths;

  (void)app;
  hours = g_eeprom.total_run_hours_x10 / 10U;
  tenths = (uint8_t)(g_eeprom.total_run_hours_x10 % 10U);

  (void)snprintf(line[0], sizeof(line[0]), "SET:Runtime  3/4");
  (void)snprintf(line[1], sizeof(line[1]), "Total:%lu.%uh",
                 (unsigned long)hours, (unsigned int)tenths);
  (void)snprintf(line[2], sizeof(line[2]), "Boots:%lu",
                 (unsigned long)g_eeprom.total_boot_count);
  (void)snprintf(line[3], sizeof(line[3]), "ReadErr:%lu",
                 (unsigned long)g_eeprom.sensor_read_error_count);
  (void)snprintf(line[4], sizeof(line[4]), "Recover:%lu",
                 (unsigned long)g_eeprom.sensor_recovery_count);
  (void)snprintf(line[5], sizeof(line[5]), "ErrCnt:%u",
                 (unsigned int)g_eeprom.error_log_count);
  (void)snprintf(line[6], sizeof(line[6]), "");
  (void)snprintf(line[7], sizeof(line[7]), "");

  draw_lines(line);
}

static void render_errors(const AppState_t *app)
{
  char line[8][24];
  uint8_t i;
  uint8_t entry_count;
  int16_t idx;

  (void)app;
  if (g_eeprom.error_log_count >= EEPROM_ERROR_LOG_COUNT)
  {
    entry_count = EEPROM_ERROR_LOG_COUNT;
  }
  else
  {
    entry_count = g_eeprom.error_log_count;
  }

  (void)snprintf(line[0], sizeof(line[0]), "SET:Errors   4/4");
  for (i = 0U; i < 7U; i++)
  {
    if (i >= entry_count)
    {
      (void)snprintf(line[i + 1U], sizeof(line[0]), "");
      continue;
    }

    idx = (int16_t)g_eeprom.error_log_wr_ptr - 1 - (int16_t)i;
    while (idx < 0)
    {
      idx += EEPROM_ERROR_LOG_COUNT;
    }

    {
      const eeprom_error_entry_t *e = &g_eeprom.error_log[idx];
      uint32_t h = e->run_hours_x10 / 10U;
      uint8_t t = (uint8_t)(e->run_hours_x10 % 10U);

      (void)snprintf(line[i + 1U], sizeof(line[0]),
                     "%u:%s T%u@%lu.%uh",
                     (unsigned int)(i + 1U),
                     error_type_name(e->type),
                     (unsigned int)e->phase,
                     (unsigned long)h,
                     (unsigned int)t);
    }
  }

  draw_lines(line);
}

void app_settings_render_page(const AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  switch (app->settings_sub_page)
  {
  case SETTINGS_SUB_CALIBRATE: render_calibrate(app); break;
  case SETTINGS_SUB_RUNTIME:   render_runtime(app);   break;
  case SETTINGS_SUB_ERRORS:    render_errors(app);    break;
  case SETTINGS_SUB_IDENTITY:
  default:                     render_identity(app);  break;
  }
}
