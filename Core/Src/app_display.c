#include "app_display.h"

#include <stdio.h>

#include "ssd1306.h"

/* PE3 / PE4 当前作为页面切换按键输入，按下时为低电平。 */
#define BRIGHTNESS_BUTTON_PORT     GPIOE
#define BRIGHTNESS_BUTTON_PIN      GPIO_PIN_2
#define PAGE_PREV_BUTTON_PORT      GPIOE
#define PAGE_PREV_BUTTON_PIN       GPIO_PIN_3
#define PAGE_NEXT_BUTTON_PORT      GPIOE
#define PAGE_NEXT_BUTTON_PIN       GPIO_PIN_4
/* 轮询按键的简单消抖时间。 */
#define PAGE_BUTTON_DEBOUNCE_MS    120U
#define PAGE_BUTTON_DEBOUNCE_TICKS ((PAGE_BUTTON_DEBOUNCE_MS + APP_SAMPLE_PERIOD_MS - 1U) / APP_SAMPLE_PERIOD_MS)
#define DISPLAY_BRIGHTNESS_LEVEL_COUNT 3U
#define DISPLAY_BRIGHTNESS_DEFAULT_INDEX 1U
#define DISPLAY_SENSOR_RECOVER_AGE_MS  3000U
#define DISPLAY_SENSOR_ERROR_STREAK       8U

/* 波形区域使用 OLED 下半部分，上半部分留给时间与数值文本。 */
#define OLED_WAVEFORM_TOP_Y        32U
#define OLED_WAVEFORM_HEIGHT       (SSD1306_HEIGHT - OLED_WAVEFORM_TOP_Y)

/*
 * OLED 只显示已经过 MAX30102 算法带通滤波的 AC 波形。
 * AGC 使用慢释放、快响应，避免把无意义的小噪声拉成满屏方波。
 */
#define WAVEFORM_SETTLE_SAMPLES      25U
#define WAVEFORM_AGC_MIN_SCALE       32UL
#define WAVEFORM_AGC_MAX_SCALE       20000UL
#define WAVEFORM_AGC_HEADROOM_NUM    2UL
#define WAVEFORM_AGC_HEADROOM_DEN    1UL
#define WAVEFORM_AGC_ATTACK_SHIFT    2U
#define WAVEFORM_AGC_RELEASE_SHIFT   5U

/* OLED 波形缓冲：保存最近一屏带通 AC 样本及显示增益状态。 */
typedef struct
{
  int32_t samples[SSD1306_WIDTH];
  uint8_t markers[SSD1306_WIDTH];
  uint16_t write_index;
  uint16_t sample_count;
  uint16_t settle_count;
  uint32_t scale_estimate;
} WaveformBuffer_t;

static WaveformBuffer_t ir_waveform;
static WaveformBuffer_t red_waveform;
static WaveformBuffer_t ecg_waveform;
static const uint8_t display_brightness_table[DISPLAY_BRIGHTNESS_LEVEL_COUNT] = {0x20U, 0x7FU, 0xFFU};
static const char *const display_brightness_label_table[DISPLAY_BRIGHTNESS_LEVEL_COUNT] = {"LOW", "MID", "HIGH"};

static uint8_t page_button_poll_pressed(PageButton_t *button);
static void waveform_buffer_reset(WaveformBuffer_t *waveform);
static uint32_t waveform_abs_i32(int32_t value);
static void waveform_buffer_add_sample(WaveformBuffer_t *waveform, int32_t filtered_value);
static void waveform_buffer_mark_latest(WaveformBuffer_t *waveform);
static void waveform_buffer_draw(const WaveformBuffer_t *waveform,
                                 uint8_t x,
                                 uint8_t y,
                                 uint8_t width,
                                 uint8_t height,
                                 uint8_t dotted,
                                 uint8_t markers);
static uint8_t app_display_text_width_px(const char *s);
static void app_display_draw_right(uint8_t y, const char *s);
static const char *app_get_finger_status(const AppState_t *app, char *buf, size_t buf_size);
static void app_display_pulse_page(const AppState_t *app);
static void app_display_oxy_page(const AppState_t *app);
static void app_display_vitals_page(const AppState_t *app);
static void app_display_ecg_page(const AppState_t *app);
static void app_display_debug_page(const AppState_t *app);
static const char *app_get_quality_label(const AppState_t *app);
static const char *app_get_balance_label(uint8_t balance_status);
static const char *app_get_regular_label(const AppState_t *app);
static const char *app_get_rtc_status_label(const AppState_t *app);
static const char *app_get_weekday_string(uint8_t weekday);
static const char *app_get_sensor_status_string(const AppState_t *app);
static const char *app_get_brightness_label(uint8_t brightness_index);
static const char *app_get_brightness_short_label(uint8_t brightness_index);
static void app_display_cycle_brightness(AppState_t *app);
static void app_format_rtc_lines(const AppState_t *app,
                                 char *time_line,
                                 size_t time_line_size,
                                 char *date_line,
                                 size_t date_line_size);

/* 初始化显示状态，并设置默认页面和按键引脚。 */
void app_display_init_state(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app->current_page = DISPLAY_PAGE_BPM;
  app->display_refresh_requested = 1U;
  app->display_brightness_index = DISPLAY_BRIGHTNESS_DEFAULT_INDEX;
  app->brightness_button.port = BRIGHTNESS_BUTTON_PORT;
  app->brightness_button.pin = BRIGHTNESS_BUTTON_PIN;
  app->page_prev_button.port = PAGE_PREV_BUTTON_PORT;
  app->page_prev_button.pin = PAGE_PREV_BUTTON_PIN;
  app->page_next_button.port = PAGE_NEXT_BUTTON_PORT;
  app->page_next_button.pin = PAGE_NEXT_BUTTON_PIN;
  waveform_buffer_reset(&ecg_waveform);
}

/* 同时清空 IR / RED 两条波形缓冲。 */
void app_display_reset_waveforms(void)
{
  waveform_buffer_reset(&ir_waveform);
  waveform_buffer_reset(&red_waveform);
}

/* IR 页面使用的波形样本入口。 */
void app_display_add_ir_sample(int32_t filtered_value)
{
  waveform_buffer_add_sample(&ir_waveform, filtered_value);
}

/* SpO2 页面使用的波形样本入口。 */
void app_display_add_red_sample(int32_t filtered_value)
{
  waveform_buffer_add_sample(&red_waveform, filtered_value);
}

void app_display_add_ir_pulse_marker(void)
{
  waveform_buffer_mark_latest(&ir_waveform);
}

void app_display_reset_ecg_waveform(void)
{
  waveform_buffer_reset(&ecg_waveform);
}

void app_display_add_ecg_sample(int32_t filtered_value)
{
  waveform_buffer_add_sample(&ecg_waveform, filtered_value);
}

void app_display_add_ecg_r_peak_marker(void)
{
  waveform_buffer_mark_latest(&ecg_waveform);
}

/* 处理页面切换按键，让 UI 切换与测量处理保持解耦。 */
void app_display_handle_buttons(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  if (page_button_poll_pressed(&app->brightness_button) != 0U)
  {
    app_display_cycle_brightness(app);
    app->display_refresh_requested = 1U;
  }

  if (page_button_poll_pressed(&app->page_prev_button) != 0U)
  {
    if (app->current_page == DISPLAY_PAGE_BPM)
    {
      app->current_page = (DisplayPage_t)(DISPLAY_PAGE_COUNT - 1U);
    }
    else
    {
      app->current_page = (DisplayPage_t)(app->current_page - 1U);
    }

    app->display_refresh_requested = 1U;
  }

  if (page_button_poll_pressed(&app->page_next_button) != 0U)
  {
    app->current_page = (DisplayPage_t)((app->current_page + 1U) % DISPLAY_PAGE_COUNT);
    app->display_refresh_requested = 1U;
  }
}

/* 根据当前页面与状态，绘制主测量页面。 */
void app_display_measurement_page(const AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  switch (app->current_page)
  {
    case DISPLAY_PAGE_SPO2:
      app_display_oxy_page(app);
      break;

    case DISPLAY_PAGE_VITALS:
      app_display_vitals_page(app);
      break;

    case DISPLAY_PAGE_ECG:
      app_display_ecg_page(app);
      break;

    case DISPLAY_PAGE_DEBUG:
      app_display_debug_page(app);
      break;

    case DISPLAY_PAGE_BPM:
    default:
      app_display_pulse_page(app);
      break;
  }
}

/*
 * PULSE 页：2 行文本 + 全高 IR 波形 (y=16..63, 48px)。
 * y=0: 左侧 HR+IBI，右侧 SQ（如 "SQ42" 或 "M"）。
 * y=8: 左侧 REG/状态标签，右侧页面标题。
 */
static void app_display_pulse_page(const AppState_t *app)
{
  char status_buf[16];
  const char *fs;
  char line0[32];
  char line0r[16];
  char line1[32];
  char line1r[8];

  if (app == NULL) { return; }

  fs = app_get_finger_status(app, status_buf, sizeof(status_buf));
  if (fs != NULL)
  {
    ssd1306_Clear(SSD1306_COLOR_BLACK);
    ssd1306_DrawString(0, 0, "PULSE");
    ssd1306_DrawString(24, 24, fs);
    ssd1306_UpdateScreen();
    return;
  }

  /* --- y=0 左侧：HR + IBI；两者独立显示，避免 HR 短暂无效时隐藏 IBI。 --- */
  if (app->finger_present != 0U)
  {
    uint8_t pos;

    if ((app->bpm_valid != 0U) || (app->bpm_value != 0U))
    {
      pos = (uint8_t)snprintf(line0, sizeof(line0), "HR:%u%s",
                              (unsigned int)app->bpm_value,
                              (app->bpm_valid != 0U) ? "" : "?");
    }
    else
    {
      pos = (uint8_t)snprintf(line0, sizeof(line0), "HR:--");
    }

    if ((app->ibi_valid != 0U) || (app->latest_ibi_ms != 0U))
    {
      (void)snprintf(line0 + pos, sizeof(line0) - (size_t)pos,
                     " I:%u%s", (unsigned int)app->latest_ibi_ms,
                     (app->ibi_valid != 0U) ? "" : "?");
    }
    else
    {
      (void)snprintf(line0 + pos, sizeof(line0) - (size_t)pos, " I:--");
    }
  }
  else
  {
    (void)snprintf(line0, sizeof(line0), "HR:-- I:--");
  }

  /* --- y=0 右侧：SQ --- */
  if (app->motion_artifact != 0U)
  {
    (void)snprintf(line0r, sizeof(line0r), "M");
  }
  else
  {
    (void)snprintf(line0r, sizeof(line0r), "SQ%u", (unsigned int)app->signal_quality);
  }

  /* --- y=8 左侧：心律标签 --- */
  if (app->motion_artifact != 0U)
  {
    (void)snprintf(line1, sizeof(line1), "MOT");
  }
  else
  {
    (void)snprintf(line1, sizeof(line1), "REG:%s", app_get_regular_label(app));
  }

  /* --- y=8 右侧：页面标题（原亮度位置） --- */
  (void)snprintf(line1r, sizeof(line1r), "PULSE");

  ssd1306_Clear(SSD1306_COLOR_BLACK);
  ssd1306_DrawString(0, 0, line0);
  app_display_draw_right(0, line0r);
  ssd1306_DrawString(0, 8, line1);
  app_display_draw_right(8, line1r);
  waveform_buffer_draw(&ir_waveform, 0U, 16U, SSD1306_WIDTH, 48U, 0U, 1U);
  ssd1306_UpdateScreen();
}

/*
 * OXY 页：2 行文本 + IR 波形 (y=16..39, 24px) + RED 波形 (y=40..63, 24px)。
 * y=0: 左侧 SpO2+R，右侧 "O" 标题 + PI。
 * y=8: 左侧 BAL/MOT/SQ，右侧亮度和简短状态。
 */
static void app_display_oxy_page(const AppState_t *app)
{
  char status_buf[16];
  const char *fs;
  char line0[32];
  char line0r[16];
  char line1[32];
  char line1r[8];

  if (app == NULL) { return; }

  fs = app_get_finger_status(app, status_buf, sizeof(status_buf));
  if (fs != NULL)
  {
    ssd1306_Clear(SSD1306_COLOR_BLACK);
    ssd1306_DrawString(0, 0, "OXY");
    ssd1306_DrawString(24, 24, fs);
    ssd1306_UpdateScreen();
    return;
  }

  /* --- y=0 左侧：SpO2 + R --- */
  if ((app->finger_present != 0U) &&
      ((app->spo2_valid != 0U) || (app->spo2_value != 0U)))
  {
    uint8_t sp_len = (uint8_t)(snprintf(line0, sizeof(line0), "SpO2:%u%s",
                                        (unsigned int)app->spo2_value,
                                        (app->spo2_valid != 0U) ? "" : "?"));
    if ((app->spo2_ratio_valid != 0U) || (app->spo2_ratio_x1000 != 0U))
    {
      (void)snprintf(line0 + sp_len, sizeof(line0) - (size_t)sp_len,
                     " R:%u.%02u%s",
                     (unsigned int)(app->spo2_ratio_x1000 / 1000U),
                     (unsigned int)((app->spo2_ratio_x1000 % 1000U) / 10U),
                     (app->spo2_ratio_valid != 0U) ? "" : "?");
    }
  }
  else
  {
    (void)snprintf(line0, sizeof(line0), "SpO2:-- R:--");
  }

  /* --- y=0 右侧：PI --- */
  if (app->motion_artifact != 0U)
  {
    (void)snprintf(line0r, sizeof(line0r), "M");
  }
  else
  {
    (void)snprintf(line0r, sizeof(line0r), "PI%u.%u",
                   (unsigned int)(app->signal_ir_pi_x1000 / 10U),
                   (unsigned int)(app->signal_ir_pi_x1000 % 10U));
  }

  /* --- y=8 左侧：BAL + SQ --- */
  if (app->motion_artifact != 0U)
  {
    (void)snprintf(line1, sizeof(line1), "MOT");
  }
  else
  {
    (void)snprintf(line1, sizeof(line1), "BAL:%s SQ:%u",
                   app_get_balance_label(app->spo2_balance_status),
                   (unsigned int)app->signal_quality);
  }

  /* --- y=8 右侧：页面标题（原亮度位置） --- */
  (void)snprintf(line1r, sizeof(line1r), "OXY");

  ssd1306_Clear(SSD1306_COLOR_BLACK);
  ssd1306_DrawString(0, 0, line0);
  app_display_draw_right(0, line0r);
  ssd1306_DrawString(0, 8, line1);
  app_display_draw_right(8, line1r);
  waveform_buffer_draw(&ir_waveform,  0U, 16U, SSD1306_WIDTH, 24U, 0U, 0U);
  waveform_buffer_draw(&red_waveform, 0U, 40U, SSD1306_WIDTH, 24U, 1U, 0U);
  ssd1306_UpdateScreen();
}

/*
 * VITALS 汇总页：8 行纯文本。
 * y=0: 左侧 HR+RR，右侧 "V" 标题 + SQ。
 */
static void app_display_vitals_page(const AppState_t *app)
{
  char status_buf[16];
  const char *fs;
  char line0[32];
  char line0r[16];
  char line1[24];
  char line2[24];
  char line3[24];
  char line4[24];
  char line5[24];
  char time_line[32];
  char date_line[32];
  uint8_t hr_visible;
  uint8_t rr_visible;
  uint16_t hrv_sdnn_display;
  uint16_t hrv_rmssd_display;

  if (app == NULL) { return; }

  fs = app_get_finger_status(app, status_buf, sizeof(status_buf));
  if (fs != NULL)
  {
    app_format_rtc_lines(app, time_line, sizeof(time_line), date_line, sizeof(date_line));
    ssd1306_Clear(SSD1306_COLOR_BLACK);
    ssd1306_DrawString(0, 0, "VITALS");
    ssd1306_DrawString(24, 24, fs);
    ssd1306_DrawString(0, 48, time_line);
    ssd1306_DrawString(0, 56, date_line);
    ssd1306_UpdateScreen();
    return;
  }

  /* --- y=0 左侧：HR + RR --- */
  hr_visible = ((app->finger_present != 0U) &&
                ((app->bpm_valid != 0U) || (app->bpm_value != 0U))) ? 1U : 0U;
  rr_visible = ((app->finger_present != 0U) &&
                ((app->rr_valid != 0U) || (app->rr_bpm != 0U))) ? 1U : 0U;

  if ((hr_visible != 0U) || (rr_visible != 0U))
  {
    uint8_t pos = 0U;
    if (hr_visible != 0U)
    {
      pos += (uint8_t)(snprintf(line0 + pos, sizeof(line0) - (size_t)pos,
                                "HR:%u%s",
                                (unsigned int)app->bpm_value,
                                (app->bpm_valid != 0U) ? "" : "?"));
    }
    else { pos += (uint8_t)(snprintf(line0 + pos, sizeof(line0) - (size_t)pos, "HR:--")); }
    pos += (uint8_t)(snprintf(line0 + pos, sizeof(line0) - (size_t)pos, " "));
    if (rr_visible != 0U)
    {
      (void)snprintf(line0 + pos, sizeof(line0) - (size_t)pos,
                     "RR:%u%s", (unsigned int)app->rr_bpm,
                     (app->rr_valid != 0U) ? "" : "?");
    }
    else { (void)snprintf(line0 + pos, sizeof(line0) - (size_t)pos, "RR:--"); }
  }
  else { (void)snprintf(line0, sizeof(line0), "HR:-- RR:--"); }

  /* --- y=0 右侧：短标题 + SQ --- */
  if (app->motion_artifact != 0U)
  {
    (void)snprintf(line0r, sizeof(line0r), "V M");
  }
  else
  {
    (void)snprintf(line0r, sizeof(line0r), "V SQ%u", (unsigned int)app->signal_quality);
  }

  /* --- y=8: IBI --- */
  if ((app->finger_present != 0U) &&
      ((app->ibi_valid != 0U) || (app->latest_ibi_ms != 0U)))
  {
    (void)snprintf(line1, sizeof(line1), "IBI:%u%sMS",
                   (unsigned int)app->latest_ibi_ms,
                   (app->ibi_valid != 0U) ? "" : "?");
  }
  else { (void)snprintf(line1, sizeof(line1), "IBI:--MS"); }

  /* --- y=16: SDNN + RMSSD --- */
  if ((app->finger_present != 0U) &&
      ((app->hrv_valid != 0U) || (app->hrv_sdnn_ms != 0U) || (app->hrv_rmssd_ms != 0U)))
  {
    hrv_sdnn_display = (app->hrv_sdnn_ms > 9999U) ? 9999U : app->hrv_sdnn_ms;
    hrv_rmssd_display = (app->hrv_rmssd_ms > 9999U) ? 9999U : app->hrv_rmssd_ms;
    (void)snprintf(line2, sizeof(line2), "SDNN:%u%s RMSSD:%u%s",
                   (unsigned int)hrv_sdnn_display,
                   (app->hrv_valid != 0U) ? "" : "?",
                   (unsigned int)hrv_rmssd_display,
                   (app->hrv_valid != 0U) ? "" : "?");
  }
  else { (void)snprintf(line2, sizeof(line2), "SDNN:-- RMSSD:--"); }

  /* --- y=24: SD1 + SD2 --- */
  if (app->hrv_valid != 0U)
  {
    (void)snprintf(line3, sizeof(line3), "SD1:%u SD2:%u",
                   (unsigned int)app->hrv_sd1_ms, (unsigned int)app->hrv_sd2_ms);
  }
  else { (void)snprintf(line3, sizeof(line3), "SD1:-- SD2:--"); }

  /* --- y=32: PI --- */
  (void)snprintf(line4, sizeof(line4), "PI:%u.%u",
                 (unsigned int)(app->signal_ir_pi_x1000 / 10U),
                 (unsigned int)(app->signal_ir_pi_x1000 % 10U));

  /* --- y=40: RTC status --- */
  (void)snprintf(line5, sizeof(line5), "%s", app_get_rtc_status_label(app));
  app_format_rtc_lines(app, time_line, sizeof(time_line), date_line, sizeof(date_line));

  /* --- Draw --- */
  ssd1306_Clear(SSD1306_COLOR_BLACK);
  ssd1306_DrawString(0, 0, line0);
  app_display_draw_right(0, line0r);
  ssd1306_DrawString(0, 8, line1);
  ssd1306_DrawString(0, 16, line2);
  ssd1306_DrawString(0, 24, line3);
  ssd1306_DrawString(0, 32, line4);
  ssd1306_DrawString(0, 40, line5);
  ssd1306_DrawString(0, 48, time_line);   /* 蓝色 RTC */
  ssd1306_DrawString(0, 56, date_line);   /* 蓝色 RTC */
  ssd1306_UpdateScreen();
}

/*
 * ECG page: ECG HR/RR + PTT text and a full-height filtered ECG waveform.
 * The waveform buffer is independent of MAX30102 contact state.
 */
static void app_display_ecg_page(const AppState_t *app)
{
  char line0[32];
  char line0r[8];
  char line1[32];
  char line1r[16];

  if (app == NULL) { return; }

  (void)snprintf(line0r, sizeof(line0r), "ECG");

  if (app->ecg_lead_off != 0U)
  {
    (void)snprintf(line0, sizeof(line0), "LEAD OFF:%u",
                   (unsigned int)app->ecg_lead_off);
    (void)snprintf(line1, sizeof(line1), "CHECK ELECTRODES");
    line1r[0] = '\0';
  }
  else
  {
    if ((app->ecg_valid != 0U) || (app->ecg_hr != 0U))
    {
      if (app->ecg_rr_ms != 0U)
      {
        (void)snprintf(line0, sizeof(line0), "HR:%u%s R:%u",
                       (unsigned int)app->ecg_hr,
                       (app->ecg_valid != 0U) ? "" : "?",
                       (unsigned int)app->ecg_rr_ms);
      }
      else
      {
        (void)snprintf(line0, sizeof(line0), "HR:%u%s R:--",
                       (unsigned int)app->ecg_hr,
                       (app->ecg_valid != 0U) ? "" : "?");
      }
    }
    else
    {
      (void)snprintf(line0, sizeof(line0), "HR:-- R:--");
    }

    if ((app->ptt_valid != 0U) || (app->ptt_ms != 0U))
    {
      (void)snprintf(line1, sizeof(line1), "PTT:%u%sMS",
                     (unsigned int)app->ptt_ms,
                     (app->ptt_valid != 0U) ? "" : "?");
    }
    else
    {
      (void)snprintf(line1, sizeof(line1), "PTT:--MS");
    }
    (void)snprintf(line1r, sizeof(line1r), "A:%d", (int)app->ecg_filtered);
  }

  ssd1306_Clear(SSD1306_COLOR_BLACK);
  ssd1306_DrawString(0, 0, line0);
  app_display_draw_right(0, line0r);
  ssd1306_DrawString(0, 8, line1);
  app_display_draw_right(8, line1r);
  waveform_buffer_draw(&ecg_waveform, 0U, 16U, SSD1306_WIDTH, 48U, 0U, 1U);
  ssd1306_UpdateScreen();
}

/*
 * 调试页：黄色区 (y=0,8) 放原始值，蓝色区 (y>=16) 放 RTC 和运行统计。
 */
static void app_display_debug_page(const AppState_t *app)
{
  char time_line[32];
  char line_signal[32];
  char line_signal_r[8];
  char line_fifo[24];
  char line_ok_busy[24];
  char line_err_recover[24];
  char line_quality[24];
  char line_tail[32];
  char line_sd[24];
  const char *sd_status;
  uint32_t sd_written;

  if (app == NULL) { return; }

  /* RTC 时间 + 样本年龄放在蓝色区 y=16 */
  if (app->rtc_read_ok != 0U)
  {
    uint32_t sample_age_ms = (app->sensor_last_sample_tick != 0U) ?
                             (HAL_GetTick() - app->sensor_last_sample_tick) : 0U;
    (void)snprintf(time_line, sizeof(time_line),
                   "%02u:%02u:%02u AGE:%lu",
                   (unsigned int)app->rtc_datetime.hours,
                   (unsigned int)app->rtc_datetime.minutes,
                   (unsigned int)app->rtc_datetime.seconds,
                   (unsigned long)sample_age_ms);
  }
  else { (void)snprintf(time_line, sizeof(time_line), "--:--:--"); }

  /* --- y=0 左侧：原始 R/I，右侧："D" 或 warm-up 倒计时 --- */
  (void)snprintf(line_signal, sizeof(line_signal),
                 "R:%lu I:%lu",
                 (unsigned long)app->red_value,
                 (unsigned long)app->ir_value);
  if (app->contact_settle_samples > 0U)
  {
    (void)snprintf(line_signal_r, sizeof(line_signal_r), "W%u",
                   (unsigned int)app->contact_settle_samples);
  }
  else
  {
    (void)snprintf(line_signal_r, sizeof(line_signal_r), "D");
  }

  (void)snprintf(line_fifo, sizeof(line_fifo),
                 "W:%u R:%u A:%u",
                 (unsigned int)app->sensor_fifo_write_ptr,
                 (unsigned int)app->sensor_fifo_read_ptr,
                 (unsigned int)app->sensor_fifo_available_samples);
  (void)snprintf(line_ok_busy, sizeof(line_ok_busy),
                 "OK:%lu B:%lu ER:%lu",
                 (unsigned long)app->sensor_read_ok_count,
                 (unsigned long)app->sensor_read_busy_count,
                 (unsigned long)app->sensor_read_error_count);
  (void)snprintf(line_err_recover, sizeof(line_err_recover),
                 "RC:%lu M:%u a%u",
                 (unsigned long)app->sensor_recover_count,
                 (unsigned int)app->motion_score,
                 (unsigned int)(app->motion_artifact));
  (void)snprintf(line_quality, sizeof(line_quality),
                 "SQ:%u PI:%u.%u I:%u",
                 (unsigned int)app->signal_quality,
                 (unsigned int)(app->signal_ir_pi_x1000 / 10U),
                 (unsigned int)(app->signal_ir_pi_x1000 % 10U),
                 (unsigned int)app->accepted_ibi_count);
  (void)snprintf(line_tail, sizeof(line_tail),
                 "F:%u RAW:%u ON:%u OFF:%u",
                 (unsigned int)app->finger_present,
                 (unsigned int)app->raw_signal_present,
                 (unsigned int)app->finger_on_confirm_count,
                 (unsigned int)app->finger_off_confirm_count);
  sd_status = (app->sd_log_active != 0U) ? "OK" :
              ((app->sd_log_error != 0U) ? "ERR" : "IDLE");
  sd_written = (app->sd_total_written > 9999UL) ? 9999UL : app->sd_total_written;
  (void)snprintf(line_sd, sizeof(line_sd),
                 "SD:%s W:%lu PTT:%u%s",
                 sd_status,
                 (unsigned long)sd_written,
                 (unsigned int)app->ptt_ms,
                 (app->ptt_valid != 0U) ? "" : "?");

  ssd1306_Clear(SSD1306_COLOR_BLACK);
  ssd1306_DrawString(0, 0, line_signal);        /* 黄色：原始 RED/IR */
  app_display_draw_right(0, line_signal_r);      /* 黄色右侧："D" */
  ssd1306_DrawString(0, 8, line_fifo);           /* 黄色：FIFO */
  ssd1306_DrawString(0, 16, time_line);          /* 蓝色 RTC + sample age */
  ssd1306_DrawString(0, 24, line_ok_busy);       /* OK / BUSY / ERROR */
  ssd1306_DrawString(0, 32, line_err_recover);   /* RC + motion */
  ssd1306_DrawString(0, 40, line_quality);       /* SQ + PI + IBI 计数 */
  ssd1306_DrawString(0, 48, line_tail);          /* finger debug */
  ssd1306_DrawString(0, 56, line_sd);            /* SD 状态 */
  ssd1306_UpdateScreen();
}

/*
 * 手指状态 → OLED 提示字符串。
 * 返回 NULL 表示处于正常测量状态，上层应绘制原有内容。
 * 否则返回的字符串应被醒目地绘制在页面中央。
 * WAIT / SENSOR RECOVER 状态需要 buf 提供格式化空间 (>=15 bytes)。
 */
static const char *app_get_finger_status(const AppState_t *app, char *buf, size_t buf_size)
{
  uint32_t sample_age_ms;

  if (app == NULL) { return NULL; }

  /* Sensor liveness takes priority over the last latched contact state. */
  if (app->sensor_last_sample_tick != 0U)
  {
    sample_age_ms = HAL_GetTick() - app->sensor_last_sample_tick;
    if ((sample_age_ms > DISPLAY_SENSOR_RECOVER_AGE_MS) &&
        (app->sensor_error_streak >= DISPLAY_SENSOR_ERROR_STREAK))
    {
      (void)snprintf(buf, buf_size, "SENSOR RECOVER");
      return buf;
    }
  }

  if (app->finger_present == 0U)
  {
    if (app->finger_on_confirm_count == 0U)
    {
      return "PLACE FINGER";
    }
    return "DETECTING";
  }

  if (app->contact_settle_samples > 0U)
  {
    uint16_t s = app->contact_settle_samples;
    (void)snprintf(buf, buf_size, "WAIT %u.%uS",
                   (unsigned int)(s / 100U),
                   (unsigned int)((s % 100U) / 10U));
    return buf;
  }

  return NULL;
}

static const char *app_get_sensor_status_string(const AppState_t *app)
{
  uint32_t now;

  if (app == NULL)
  {
    return "NA";
  }

  now = HAL_GetTick();
  if ((app->sensor_read_ok_count != 0U) &&
      ((now - app->sensor_last_sample_tick) <= 300U))
  {
    return "LIVE";
  }

  switch (app->sensor_last_read_status)
  {
    case 1U:
      return "OK";

    case 2U:
      return "ERR";

    default:
      return "WAIT";
  }
}

/* SQ → OLED 短标签：>=60 OK，>=30 LOW，否则 "--"。 */
static const char *app_get_quality_label(const AppState_t *app)
{
  if ((app == NULL) || (app->finger_present == 0U))
  {
    return "--";
  }

  if (app->signal_quality >= 60U)
  {
    return "OK";
  }

  if (app->signal_quality >= 30U)
  {
    return "LOW";
  }

  return "--";
}

/* R/BAL 分级 → OLED 短标签：<0.5 LOW，0.5–2.0 OK，>2.0 HIGH。 */
static const char *app_get_balance_label(uint8_t balance_status)
{
  switch (balance_status)
  {
    case APP_OXY_BALANCE_OK:
      return "OK";

    case APP_OXY_BALANCE_LOW:
      return "LOW";

    case APP_OXY_BALANCE_HIGH:
      return "HIGH";

    default:
      return "--";
  }
}

/*
 * 心律规整度标签（短窗口提示，不做诊断）：
 * - finger_present==0 或 latest_ibi_ms==0 → "--"
 * - hrv_valid==0 但有 IBI → "WAIT"（积累中）
 * - hrv_valid==1 且 rhythm_irregular==1 → "IRR"
 * - hrv_valid==1 且 RMSSD > 120ms → "VAR"
 * - hrv_valid==1 且以上不满足 → "OK"
 */
static const char *app_get_regular_label(const AppState_t *app)
{
  if ((app == NULL) || (app->finger_present == 0U))
  {
    return "--";
  }

  if (app->latest_ibi_ms == 0U)
  {
    return "--";
  }

  if (app->hrv_valid == 0U)
  {
    return "WAIT";
  }

  if (app->rhythm_irregular != 0U)
  {
    return "IRR";
  }

  if (app->hrv_rmssd_ms > 120U)
  {
    return "VAR";
  }

  return "OK";
}

/* RTC 状态短标签：ERR(读失败) / SET(已校时) / RUN(运行中未校时)。 */
static const char *app_get_rtc_status_label(const AppState_t *app)
{
  if (app == NULL)
  {
    return "RTC --";
  }

  if (app->rtc_read_ok == 0U)
  {
    return "RTC ERR";
  }

  if (app->rtc_time_valid != 0U)
  {
    return "RTC SET";
  }

  return "RTC RUN";
}

/*
 * 绘制简化状态页，常用于启动、自检与基线采集阶段。
 * 黄色区 (y=0) 放标题，蓝色区 (y>=16) 放 RTC 时间/日期。
 */
void app_display_status_page(const AppState_t *app, const char *status_line_1, const char *status_line_2)
{
  char time_line[32];
  char date_line[32];
  char uart_line[24];

  app_format_rtc_lines(app, time_line, sizeof(time_line), date_line, sizeof(date_line));

  (void)snprintf(uart_line, sizeof(uart_line),
                 "UART R:%s T:%s",
                 ((app != NULL) && app->uart_rx_message_valid) ? "VLD" : "INV",
                 ((app != NULL) && app->uart_tx_message_valid) ? "VLD" : "INV");

  ssd1306_Clear(SSD1306_COLOR_BLACK);
  ssd1306_DrawString(0, 0, "STATUS");          /* 黄色标题 */
  ssd1306_DrawString(0, 16, time_line);        /* 蓝色 RTC */
  ssd1306_DrawString(0, 24, date_line);        /* 蓝色 RTC */
  ssd1306_DrawString(0, 32, (status_line_1 != NULL) ? status_line_1 : "");
  ssd1306_DrawString(0, 40, (status_line_2 != NULL) ? status_line_2 : "");
  ssd1306_DrawString(0, 48, uart_line);
  ssd1306_UpdateScreen();
}

static const char *app_get_brightness_label(uint8_t brightness_index)
{
  if (brightness_index >= DISPLAY_BRIGHTNESS_LEVEL_COUNT)
  {
    return display_brightness_label_table[DISPLAY_BRIGHTNESS_DEFAULT_INDEX];
  }

  return display_brightness_label_table[brightness_index];
}

static const char *app_get_brightness_short_label(uint8_t brightness_index)
{
  switch (brightness_index)
  {
    case 0U:
      return "L";

    case 1U:
      return "M";

    case 2U:
      return "H";

    default:
      return "M";
  }
}

static void app_display_cycle_brightness(AppState_t *app)
{
  uint8_t brightness_index;

  if (app == NULL)
  {
    return;
  }

  brightness_index = (uint8_t)((app->display_brightness_index + 1U) % DISPLAY_BRIGHTNESS_LEVEL_COUNT);
  app->display_brightness_index = brightness_index;
  ssd1306_SetContrast(display_brightness_table[brightness_index]);
}

/*
 * 轮询式按键消抖：
 * 只有检测到“新的按下沿”时才返回 1，长按不会连续翻页。
 */
static uint8_t page_button_poll_pressed(PageButton_t *button)
{
  uint8_t current_pressed;

  if (button == NULL)
  {
    return 0U;
  }

  current_pressed = (HAL_GPIO_ReadPin(button->port, button->pin) == GPIO_PIN_RESET) ? 1U : 0U;

  if (button->cooldown_ticks > 0U)
  {
    button->cooldown_ticks--;
  }

  if (current_pressed == 0U)
  {
    button->pressed_latch = 0U;
    return 0U;
  }

  if ((button->pressed_latch == 0U) && (button->cooldown_ticks == 0U))
  {
    button->pressed_latch = 1U;
    button->cooldown_ticks = PAGE_BUTTON_DEBOUNCE_TICKS;
    return 1U;
  }

  return 0U;
}

/* 将一条波形缓冲恢复到“尚无样本”的初始状态。 */
static void waveform_buffer_reset(WaveformBuffer_t *waveform)
{
  uint16_t i;

  if (waveform == NULL)
  {
    return;
  }

  for (i = 0U; i < SSD1306_WIDTH; i++)
  {
    waveform->samples[i] = 0;
    waveform->markers[i] = 0U;
  }

  waveform->write_index = 0U;
  waveform->sample_count = 0U;
  waveform->settle_count = 0U;
  waveform->scale_estimate = WAVEFORM_AGC_MIN_SCALE;
}

static uint32_t waveform_abs_i32(int32_t value)
{
  if (value < 0)
  {
    return (uint32_t)(-value);
  }

  return (uint32_t)value;
}

/*
 * 压入一个已经带通滤波后的 AC 样本。
 * 前几个样本只用于等待滤波器阶跃响应消退，不进入显示窗口。
 */
static void waveform_buffer_add_sample(WaveformBuffer_t *waveform, int32_t filtered_value)
{
  uint32_t abs_value;
  uint32_t delta;

  if (waveform == NULL)
  {
    return;
  }

  if (waveform->settle_count < WAVEFORM_SETTLE_SAMPLES)
  {
    waveform->settle_count++;
    return;
  }

  abs_value = waveform_abs_i32(filtered_value);
  if (abs_value > WAVEFORM_AGC_MAX_SCALE)
  {
    abs_value = WAVEFORM_AGC_MAX_SCALE;
  }

  if ((waveform->sample_count == 0U) && (abs_value > waveform->scale_estimate))
  {
    waveform->scale_estimate = abs_value;
  }
  else if (abs_value > waveform->scale_estimate)
  {
    delta = abs_value - waveform->scale_estimate;
    waveform->scale_estimate += ((delta >> WAVEFORM_AGC_ATTACK_SHIFT) != 0U) ?
                                (delta >> WAVEFORM_AGC_ATTACK_SHIFT) : 1U;
  }
  else
  {
    delta = waveform->scale_estimate - abs_value;
    waveform->scale_estimate -= (delta >> WAVEFORM_AGC_RELEASE_SHIFT);
  }

  if (waveform->scale_estimate < WAVEFORM_AGC_MIN_SCALE)
  {
    waveform->scale_estimate = WAVEFORM_AGC_MIN_SCALE;
  }
  else if (waveform->scale_estimate > WAVEFORM_AGC_MAX_SCALE)
  {
    waveform->scale_estimate = WAVEFORM_AGC_MAX_SCALE;
  }

  waveform->markers[waveform->write_index] = 0U;
  waveform->samples[waveform->write_index] = filtered_value;
  waveform->write_index++;
  if (waveform->write_index >= SSD1306_WIDTH)
  {
    waveform->write_index = 0U;
  }

  if (waveform->sample_count < SSD1306_WIDTH)
  {
    waveform->sample_count++;
  }
}

/* 在当前波形写入位置标记一次脉搏检测点，供波形绘制时在底部显示脉搏指示箭头。 */
static void waveform_buffer_mark_latest(WaveformBuffer_t *waveform)
{
  uint16_t marker_index;

  if ((waveform == NULL) || (waveform->sample_count == 0U))
  {
    return;
  }

  marker_index = (waveform->write_index == 0U) ?
                 (uint16_t)(SSD1306_WIDTH - 1U) :
                 (uint16_t)(waveform->write_index - 1U);
  waveform->markers[marker_index] = 1U;
}

/*
 * 动态缩放波形绘制：扫描可见样本的 min/max，把数据映射到 y+1..y+height-2，
 * 上下各留 1px 边距，避免峰/谷被屏幕边缘截断。
 * 若所有样本相等，绘制水平中线。
 * 脉搏标记放在波形区域底部 3 行，不进入主要波形显示区。
 */
static void waveform_buffer_draw(const WaveformBuffer_t *waveform,
                                 uint8_t x,
                                 uint8_t y,
                                 uint8_t width,
                                 uint8_t height,
                                 uint8_t dotted,
                                 uint8_t markers)
{
  uint16_t i;
  uint16_t sample_count;
  uint16_t start_index;
  uint16_t sample_index;
  uint8_t  prev_y = 0U;
  uint8_t  draw_y;
  uint8_t  line_y;
  int32_t  sample_value;
  int32_t  vis_min;
  int32_t  vis_max;
  int32_t  vis_range;
  int32_t  map_top;       /* 映射目标区域上界 (y+1) */
  int32_t  map_bot;       /* 映射目标区域下界 (y+height-2) */
  int32_t  map_range;     /* map_bot - map_top */
  int32_t  draw_y_i;
  uint8_t  marker_base;   /* 脉搏标记底部起始 y */

  if ((waveform == NULL) || (width == 0U) || (height == 0U)) { return; }

  /* 空缓冲 → 画中线 */
  if (waveform->sample_count == 0U)
  {
    draw_y = (uint8_t)(y + (height / 2U));
    for (i = 0U; i < width; i++)
    {
      if ((dotted == 0U) || ((i & 1U) == 0U))
      {
        ssd1306_DrawPixel((uint8_t)(x + i), draw_y, SSD1306_COLOR_WHITE);
      }
    }
    return;
  }

  sample_count = waveform->sample_count;
  if (sample_count > width) { sample_count = width; }

  if (waveform->sample_count < SSD1306_WIDTH)
  {
    start_index = 0U;
  }
  else
  {
    start_index = waveform->write_index;
  }

  /* 第 1 遍：扫描可见样本范围 */
  vis_min = INT32_MAX;
  vis_max = INT32_MIN;
  for (i = 0U; i < sample_count; i++)
  {
    sample_index = (uint16_t)((start_index + i) % SSD1306_WIDTH);
    sample_value = waveform->samples[sample_index];
    if (sample_value < vis_min) { vis_min = sample_value; }
    if (sample_value > vis_max) { vis_max = sample_value; }
  }

  vis_range = vis_max - vis_min;
  if (vis_range < 4) { vis_range = 4; }  /* 极小信号时保持最小显示幅度 */

  /* 映射区间：y+1 到 y+height-2，上下各 1px margin */
  map_top = (int32_t)y + 1;
  map_bot = (int32_t)y + (int32_t)height - 2;
  if (map_bot < map_top) { map_bot = map_top; }
  map_range = map_bot - map_top;

  /* 第 2 遍：绘制 */
  for (i = 0U; i < sample_count; i++)
  {
    sample_index = (uint16_t)((start_index + i) % SSD1306_WIDTH);
    sample_value = waveform->samples[sample_index];

    /* clamp 到可见范围，防单个毛刺压扁整体 */
    if (sample_value > vis_max) { sample_value = vis_max; }
    if (sample_value < vis_min) { sample_value = vis_min; }

    /* 映射：vis_min → map_bot(底部), vis_max → map_top(顶部) */
    draw_y_i = map_bot - (int32_t)(((int64_t)(sample_value - vis_min) *
                                    (int64_t)map_range + ((int64_t)vis_range / 2LL)) /
                                   (int64_t)vis_range);

    /* 硬钳位防止越界 */
    if (draw_y_i < (int32_t)y)       { draw_y_i = (int32_t)y; }
    if (draw_y_i >= (int32_t)(y + height)) { draw_y_i = (int32_t)(y + height - 1); }

    draw_y = (uint8_t)draw_y_i;

    /* 虚线模式时隔点绘制，实线时绘制 + 连线 */
    if ((dotted == 0U) || ((i & 1U) == 0U))
    {
      ssd1306_DrawPixel((uint8_t)(x + i), draw_y, SSD1306_COLOR_WHITE);
    }

    if ((dotted == 0U) && (i != 0U))
    {
      if (prev_y < draw_y)
      {
        for (line_y = (uint8_t)(prev_y + 1U); line_y <= draw_y; line_y++)
        {
          ssd1306_DrawPixel((uint8_t)(x + i), line_y, SSD1306_COLOR_WHITE);
        }
      }
      else if (prev_y > draw_y)
      {
        for (line_y = (uint8_t)(draw_y + 1U); line_y <= prev_y; line_y++)
        {
          ssd1306_DrawPixel((uint8_t)(x + i), line_y, SSD1306_COLOR_WHITE);
        }
      }
    }

    /* 脉搏标记：放在波形区域底部 3 行（小箭头），不占用主显示空间 */
    if ((markers != 0U) && (waveform->markers[sample_index] != 0U) && (height >= 6U))
    {
      marker_base = (uint8_t)(y + height - 3U);
      /* 垂直 2px 竖线 */
      ssd1306_DrawPixel((uint8_t)(x + i), marker_base, SSD1306_COLOR_WHITE);
      if (marker_base + 1U < (uint8_t)(y + height))
      {
        ssd1306_DrawPixel((uint8_t)(x + i), (uint8_t)(marker_base + 1U), SSD1306_COLOR_WHITE);
      }
      /* 底部三角：左右各 1px */
      if ((i > 0U) && (marker_base + 2U < (uint8_t)(y + height)))
      {
        ssd1306_DrawPixel((uint8_t)(x + i - 1U), (uint8_t)(marker_base + 2U), SSD1306_COLOR_WHITE);
      }
      if (((uint8_t)(x + i + 1U) < SSD1306_WIDTH) && (marker_base + 2U < (uint8_t)(y + height)))
      {
        ssd1306_DrawPixel((uint8_t)(x + i + 1U), (uint8_t)(marker_base + 2U), SSD1306_COLOR_WHITE);
      }
    }

    prev_y = draw_y;
  }
}

/* 计算字符串在 SSD1306 上的像素宽度（5x7 字体 + 1px 间距 ≈ 6px/字符）。 */
static uint8_t app_display_text_width_px(const char *s)
{
  uint8_t w = 0U;
  if (s != NULL) { while (*s) { w += 6U; s++; } }
  return w;
}

/* 在指定行右对齐绘制字符串。x = 128 - 文本宽度，不越界。 */
static void app_display_draw_right(uint8_t y, const char *s)
{
  uint8_t w;
  int16_t pos_x;

  if (s == NULL) { return; }
  w = app_display_text_width_px(s);
  pos_x = (int16_t)SSD1306_WIDTH - (int16_t)w;
  if (pos_x < 0) { pos_x = 0; }
  ssd1306_DrawString((uint8_t)pos_x, y, s);
}

/* 把 RTC 星期枚举转成 OLED 上显示的简短英文缩写。 */
static const char *app_get_weekday_string(uint8_t weekday)
{
  switch (weekday)
  {
    case RTC_WEEKDAY_MONDAY:
      return "MON";

    case RTC_WEEKDAY_TUESDAY:
      return "TUE";

    case RTC_WEEKDAY_WEDNESDAY:
      return "WED";

    case RTC_WEEKDAY_THURSDAY:
      return "THU";

    case RTC_WEEKDAY_FRIDAY:
      return "FRI";

    case RTC_WEEKDAY_SATURDAY:
      return "SAT";

    case RTC_WEEKDAY_SUNDAY:
      return "SUN";

    default:
      return "---";
  }
}

/* 统一格式化 OLED 上方两行时间/日期文本。 */
static void app_format_rtc_lines(const AppState_t *app,
                                 char *time_line,
                                 size_t time_line_size,
                                 char *date_line,
                                 size_t date_line_size)
{
  if ((time_line == NULL) || (date_line == NULL) || (time_line_size == 0U) || (date_line_size == 0U))
  {
    return;
  }

  if ((app != NULL) && (app->rtc_read_ok != 0U))
  {
    (void)snprintf(time_line,
                   time_line_size,
                   "TIME %02u:%02u:%02u",
                   (unsigned int)app->rtc_datetime.hours,
                   (unsigned int)app->rtc_datetime.minutes,
                   (unsigned int)app->rtc_datetime.seconds);
    (void)snprintf(date_line,
                   date_line_size,
                   "%04u-%02u-%02u %s",
                   (unsigned int)app->rtc_datetime.year,
                   (unsigned int)app->rtc_datetime.month,
                   (unsigned int)app->rtc_datetime.date,
                   app_get_weekday_string(app->rtc_datetime.weekday));
    return;
  }

  (void)snprintf(time_line, time_line_size, "TIME --:--:--");
  (void)snprintf(date_line, date_line_size, "DATE ----.--.--");
}
