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
static void app_display_pulse_page(const AppState_t *app);
static void app_display_oxy_page(const AppState_t *app);
static void app_display_vitals_page(const AppState_t *app);
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
 * PULSE 页（原 BPM 页升级）：
 * 显示 HR + IBI + 心律标签 + IR 波形 + 脉搏检测标记。
 * HR/IBI 的 valid 字段控制末尾 "?" 后缀：valid=0 但值非零时显示旧值+"?"，
 * 表示信号短暂中断但保留上一帧结果供参考。
 *
 * FIXME: RTC 时间/日期已从此页移除，旧 BPM 页在顶部两行显示时间和日期。
 * 生理信号监测中 RTC 是必要参考，应恢复到此页或通过 TIME/STATUS 页承接。
 */
static void app_display_pulse_page(const AppState_t *app)
{
  char title_line[24];
  char hr_line[24];
  char ibi_line[24];
  char state_line[24];

  if (app == NULL)
  {
    return;
  }

  (void)snprintf(title_line,
                 sizeof(title_line),
                 "PULSE     SQ:%u",
                 (unsigned int)app->signal_quality);

  /* HR 末尾的 "?" 表示 bpm_valid=0 但 bpm_value 保留旧值。 */
  if ((app->finger_present != 0U) &&
      ((app->bpm_valid != 0U) || (app->bpm_value != 0U)))
  {
    (void)snprintf(hr_line,
                   sizeof(hr_line),
                   "HR %u%s BPM",
                   (unsigned int)app->bpm_value,
                   (app->bpm_valid != 0U) ? "" : "?");
  }
  else
  {
    (void)snprintf(hr_line, sizeof(hr_line), "HR -- BPM");
  }

  /* IBI 末尾 "?" 含义同上；REG 标签基于 RMSSD 判断心律是否规整。 */
  if ((app->finger_present != 0U) &&
      ((app->ibi_valid != 0U) || (app->latest_ibi_ms != 0U)))
  {
    (void)snprintf(ibi_line,
                   sizeof(ibi_line),
                   "IBI %u%sMS REG:%s",
                   (unsigned int)app->latest_ibi_ms,
                   (app->ibi_valid != 0U) ? "" : "?",
                   app_get_regular_label(app));
  }
  else
  {
    (void)snprintf(ibi_line, sizeof(ibi_line), "IBI --MS REG:--");
  }

  (void)snprintf(state_line,
                 sizeof(state_line),
                 "IR %s B:%s",
                 (app->finger_present != 0U) ? "WAVE" : "----",
                 app_get_brightness_short_label(app->display_brightness_index));

  ssd1306_Clear(SSD1306_COLOR_BLACK);
  ssd1306_DrawString(0, 0, title_line);
  ssd1306_DrawString(0, 8, hr_line);
  ssd1306_DrawString(0, 16, ibi_line);
  ssd1306_DrawString(0, 24, state_line);
  waveform_buffer_draw(&ir_waveform,
                       0U,
                       OLED_WAVEFORM_TOP_Y,
                       SSD1306_WIDTH,
                       OLED_WAVEFORM_HEIGHT,
                       0U,
                       1U);
  ssd1306_UpdateScreen();
}

/*
 * OXY 页（原 SpO2 页升级）：
 * 显示 SpO2 + R/BAL 比值 + 双通道波形（IR 实线 + RED 虚线）。
 * R 末尾 "?" 同 PULSE 页含义：spo2_ratio_valid=0 但旧值保留。
 *
 * FIXME: RTC 时间/日期已从此页移除，同 PULSE 页的问题。
 */
static void app_display_oxy_page(const AppState_t *app)
{
  char title_line[24];
  char spo2_line[24];
  char ratio_line[24];
  char wave_line[24];

  if (app == NULL)
  {
    return;
  }

  /* PI 显示：存储值 = PI% × 10（例 PI=5.3% → 值 53），除以 10 取整+余数显示 X.X 格式。 */
  (void)snprintf(title_line,
                 sizeof(title_line),
                 "OXY      PI:%u.%u",
                 (unsigned int)(app->signal_ir_pi_x1000 / 10U),
                 (unsigned int)(app->signal_ir_pi_x1000 % 10U));

  /* SpO2 末尾 "?" 含义同 HR：spo2_valid=0 但 spo2_value 保留旧值。 */
  if ((app->finger_present != 0U) &&
      ((app->spo2_valid != 0U) || (app->spo2_value != 0U)))
  {
    (void)snprintf(spo2_line,
                   sizeof(spo2_line),
                   "SPO2 %u%sP",
                   (unsigned int)app->spo2_value,
                   (app->spo2_valid != 0U) ? "" : "?");
  }
  else
  {
    (void)snprintf(spo2_line, sizeof(spo2_line), "SPO2 --P");
  }

  /* R/BAL：R 比值 ×1000 显示为 X.XX 格式；BAL 分级 LOW/OK/HIGH。 */
  if ((app->finger_present != 0U) &&
      ((app->spo2_ratio_valid != 0U) || (app->spo2_ratio_x1000 != 0U)))
  {
    (void)snprintf(ratio_line,
                   sizeof(ratio_line),
                   "R:%u.%02u%s BAL:%s",
                   (unsigned int)(app->spo2_ratio_x1000 / 1000U),
                   (unsigned int)((app->spo2_ratio_x1000 % 1000U) / 10U),
                   (app->spo2_ratio_valid != 0U) ? "" : "?",
                   app_get_balance_label(app->spo2_balance_status));
  }
  else
  {
    (void)snprintf(ratio_line, sizeof(ratio_line), "R:-- BAL:--");
  }

  /* 双通道波形：上半部 IR 实线 + 下半部 RED 虚线（隔点绘制）。 */
  (void)snprintf(wave_line, sizeof(wave_line), "IR/RD WAVE");

  ssd1306_Clear(SSD1306_COLOR_BLACK);
  ssd1306_DrawString(0, 0, title_line);
  ssd1306_DrawString(0, 8, spo2_line);
  ssd1306_DrawString(0, 16, ratio_line);
  ssd1306_DrawString(0, 24, wave_line);
  waveform_buffer_draw(&ir_waveform, 0U, 32U, SSD1306_WIDTH, 16U, 0U, 0U);
  waveform_buffer_draw(&red_waveform, 0U, 48U, SSD1306_WIDTH, 16U, 1U, 0U);
  ssd1306_UpdateScreen();
}

/*
 * VITALS 汇总页：在一个页面同时展示 HR / RR / IBI / SDNN / RMSSD / PI / SQ / RTC。
 * 无波形，纯文本 8 行布局，充分利用 128×64 OLED 的垂直空间。
 * 每行末尾 "?" 遵循统一约定：valid=0 但值非零时显示旧值。
 */
static void app_display_vitals_page(const AppState_t *app)
{
  char line0[24];
  char line1[24];
  char line2[24];
  char line3[24];
  char line4[24];
  char line5[24];
  char time_line[32];
  char date_line[32];
  uint8_t hr_visible;
  uint8_t rr_visible;

  if (app == NULL)
  {
    return;
  }

  (void)snprintf(line0, sizeof(line0), "VITALS   SQ:%u", (unsigned int)app->signal_quality);
  /* HR 和 RR 各自判断是否有数据可显示（互不依赖）。 */
  hr_visible = ((app->finger_present != 0U) &&
                ((app->bpm_valid != 0U) || (app->bpm_value != 0U))) ? 1U : 0U;
  rr_visible = ((app->finger_present != 0U) &&
                ((app->rr_valid != 0U) || (app->rr_bpm != 0U))) ? 1U : 0U;

  if ((hr_visible != 0U) && (rr_visible != 0U))
  {
    (void)snprintf(line1,
                   sizeof(line1),
                   "HR:%u%s RR:%u%s",
                   (unsigned int)app->bpm_value,
                   (app->bpm_valid != 0U) ? "" : "?",
                   (unsigned int)app->rr_bpm,
                   (app->rr_valid != 0U) ? "" : "?");
  }
  else if (hr_visible != 0U)
  {
    (void)snprintf(line1,
                   sizeof(line1),
                   "HR:%u%s RR:--",
                   (unsigned int)app->bpm_value,
                   (app->bpm_valid != 0U) ? "" : "?");
  }
  else if (rr_visible != 0U)
  {
    (void)snprintf(line1,
                   sizeof(line1),
                   "HR:-- RR:%u%s",
                   (unsigned int)app->rr_bpm,
                   (app->rr_valid != 0U) ? "" : "?");
  }
  else
  {
    (void)snprintf(line1, sizeof(line1), "HR:-- RR:--");
  }

  if ((app->finger_present != 0U) &&
      ((app->ibi_valid != 0U) || (app->latest_ibi_ms != 0U)))
  {
    (void)snprintf(line2,
                   sizeof(line2),
                   "IBI:%u%sMS",
                   (unsigned int)app->latest_ibi_ms,
                   (app->ibi_valid != 0U) ? "" : "?");
  }
  else
  {
    (void)snprintf(line2, sizeof(line2), "IBI:--MS");
  }

  /* FIXME: SDNN/RMSSD 未遵循 "?" 约定——hrv_valid=0 时直接显示 "--"，
   * 而 HR/IBI/RR 在 valid=0 但值非零时会显示旧值+"?"。
   * 若希望统一行为，应改为：hrv_valid!=0 时显示数值，否则数值非零时显示 "value?"，全零时 "--"。 */
  if ((app->finger_present != 0U) && (app->hrv_valid != 0U))
  {
    (void)snprintf(line3,
                   sizeof(line3),
                   "SDNN:%u RMSSD:%u",
                   (unsigned int)((app->hrv_sdnn_ms > 9999U) ? 9999U : app->hrv_sdnn_ms),
                   (unsigned int)((app->hrv_rmssd_ms > 9999U) ? 9999U : app->hrv_rmssd_ms));
  }
  else
  {
    (void)snprintf(line3, sizeof(line3), "SDNN:-- RMSSD:--");
  }

  (void)snprintf(line4,
                 sizeof(line4),
                 "PI:%u.%u Q:%s",
                 (unsigned int)(app->signal_ir_pi_x1000 / 10U),
                 (unsigned int)(app->signal_ir_pi_x1000 % 10U),
                 app_get_quality_label(app));
  (void)snprintf(line5, sizeof(line5), "%s", app_get_rtc_status_label(app));
  app_format_rtc_lines(app, time_line, sizeof(time_line), date_line, sizeof(date_line));

  ssd1306_Clear(SSD1306_COLOR_BLACK);
  ssd1306_DrawString(0, 0, line0);
  ssd1306_DrawString(0, 8, line1);
  ssd1306_DrawString(0, 16, line2);
  ssd1306_DrawString(0, 24, line3);
  ssd1306_DrawString(0, 32, line4);
  ssd1306_DrawString(0, 40, line5);
  ssd1306_DrawString(0, 48, time_line);
  ssd1306_DrawString(0, 56, date_line);
  ssd1306_UpdateScreen();
}

/* 调试页改为文本总览，方便现场直接看关键运行参数。 */
static void app_display_debug_page(const AppState_t *app)
{
  char time_line[32];
  char line_signal[24];
  char line_fifo[24];
  char line_ok_busy[24];
  char line_err_recover[24];
  char line_quality[24];
  char line_display[32];
  char line_status[24];

  if (app == NULL)
  {
    return;
  }

  if (app->rtc_read_ok != 0U)
  {
    (void)snprintf(time_line,
                   sizeof(time_line),
                   "DEBUG %02u:%02u:%02u",
                   (unsigned int)app->rtc_datetime.hours,
                   (unsigned int)app->rtc_datetime.minutes,
                   (unsigned int)app->rtc_datetime.seconds);
  }
  else
  {
    (void)snprintf(time_line, sizeof(time_line), "DEBUG --:--:--");
  }

  (void)snprintf(line_signal,
                 sizeof(line_signal),
                 "R:%lu I:%lu",
                 (unsigned long)app->red_value,
                 (unsigned long)app->ir_value);
  (void)snprintf(line_fifo,
                 sizeof(line_fifo),
                 "W:%u R:%u A:%u",
                 (unsigned int)app->sensor_fifo_write_ptr,
                 (unsigned int)app->sensor_fifo_read_ptr,
                 (unsigned int)app->sensor_fifo_available_samples);
  (void)snprintf(line_ok_busy,
                 sizeof(line_ok_busy),
                 "OK:%lu B:%lu",
                 (unsigned long)app->sensor_read_ok_count,
                 (unsigned long)app->sensor_read_busy_count);
  (void)snprintf(line_err_recover,
                 sizeof(line_err_recover),
                 "ER:%lu RC:%lu",
                 (unsigned long)app->sensor_read_error_count,
                 (unsigned long)app->sensor_recover_count);
  (void)snprintf(line_quality,
                 sizeof(line_quality),
                 "Q:%u PI:%u/%u",
                 (unsigned int)app->signal_quality,
                 (unsigned int)app->signal_ir_pi_x1000,
                 (unsigned int)app->signal_red_pi_x1000);
  (void)snprintf(line_display,
                 sizeof(line_display),
                 "D:%lu O:%u B:%u",
                 (unsigned long)app->display_refresh_count,
                 (unsigned int)app->sensor_fifo_overflow_count,
                 (unsigned int)(app->display_brightness_index + 1U));

  (void)snprintf(line_status,
                 sizeof(line_status),
                 "S:%s E:%lX",
                 app_get_sensor_status_string(app),
                 (unsigned long)app->sensor_last_i2c_error);

  ssd1306_Clear(SSD1306_COLOR_BLACK);
  ssd1306_DrawString(0, 0, time_line);
  ssd1306_DrawString(0, 8, line_signal);
  ssd1306_DrawString(0, 16, line_fifo);
  ssd1306_DrawString(0, 24, line_ok_busy);
  ssd1306_DrawString(0, 32, line_err_recover);
  ssd1306_DrawString(0, 40, line_quality);
  ssd1306_DrawString(0, 48, line_display);
  ssd1306_DrawString(0, 56, line_status);
  ssd1306_UpdateScreen();
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
 * 心律规整度标签（基于 RMSSD）：
 * - RMSSD > 120 ms → "VAR"（变异性高，常见于年轻/健康人群）
 * - RMSSD <= 120 或 HRV 无效 → "OK"（心律较规整）
 * 仅在手指在位且 IBI 有效时输出，否则 "--"。
 */
static const char *app_get_regular_label(const AppState_t *app)
{
  if ((app == NULL) || (app->finger_present == 0U) || (app->ibi_valid == 0U))
  {
    return "--";
  }

  if ((app->hrv_valid != 0U) && (app->hrv_rmssd_ms > 120U))
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

/* 绘制简化状态页，常用于启动、自检与基线采集阶段。 */
void app_display_status_page(const AppState_t *app, const char *status_line_1, const char *status_line_2)
{
  char time_line[32];
  char date_line[32];
  char uart_line[24];
  char brightness_line[24];

  app_format_rtc_lines(app, time_line, sizeof(time_line), date_line, sizeof(date_line));

  (void)snprintf(uart_line,
                 sizeof(uart_line),
                 "UART R:%s T:%s",
                 ((app != NULL) && app->uart_rx_message_valid) ? "VLD" : "INV",
                 ((app != NULL) && app->uart_tx_message_valid) ? "VLD" : "INV");
  (void)snprintf(brightness_line,
                 sizeof(brightness_line),
                 "OLED BRT:%s",
                 (app != NULL) ? app_get_brightness_label(app->display_brightness_index) : "MID");

  ssd1306_Clear(SSD1306_COLOR_BLACK);
  ssd1306_DrawString(0, 0, time_line);
  ssd1306_DrawString(0, 8, date_line);
  ssd1306_DrawString(0, 16, (status_line_1 != NULL) ? status_line_1 : "");
  ssd1306_DrawString(0, 24, (status_line_2 != NULL) ? status_line_2 : "");
  ssd1306_DrawString(0, 32, uart_line);
  ssd1306_DrawString(0, 40, brightness_line);
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

/* 把最近一屏样本映射到 OLED 指定区域，绘制成连续折线。 */
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
  uint8_t prev_y = 0U;
  uint8_t draw_y;
  uint8_t line_y;
  uint16_t sample_index;
  int32_t center_y;
  int32_t half_height;
  int32_t draw_y_i;
  int32_t sample_value;
  uint32_t display_scale;

  if ((waveform == NULL) || (width == 0U) || (height == 0U))
  {
    return;
  }

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
  if (sample_count > width)
  {
    sample_count = width;
  }

  if (waveform->sample_count < SSD1306_WIDTH)
  {
    start_index = 0U;
  }
  else
  {
    start_index = waveform->write_index;
  }

  center_y = (int32_t)y + ((int32_t)height / 2L);
  half_height = ((int32_t)height / 2L) - 1L;
  if (half_height < 1L)
  {
    half_height = 1L;
  }

  display_scale = (waveform->scale_estimate * WAVEFORM_AGC_HEADROOM_NUM) /
                  WAVEFORM_AGC_HEADROOM_DEN;
  if (display_scale < WAVEFORM_AGC_MIN_SCALE)
  {
    display_scale = WAVEFORM_AGC_MIN_SCALE;
  }
  else if (display_scale > WAVEFORM_AGC_MAX_SCALE)
  {
    display_scale = WAVEFORM_AGC_MAX_SCALE;
  }

  for (i = 0U; i < sample_count; i++)
  {
    sample_index = (uint16_t)((start_index + i) % SSD1306_WIDTH);
    sample_value = waveform->samples[sample_index];
    if (sample_value > (int32_t)display_scale)
    {
      sample_value = (int32_t)display_scale;
    }
    else if (sample_value < -((int32_t)display_scale))
    {
      sample_value = -((int32_t)display_scale);
    }

    draw_y_i = center_y - (int32_t)(((int64_t)sample_value * (int64_t)half_height) /
                                    (int64_t)display_scale);
    if (draw_y_i < (int32_t)y)
    {
      draw_y_i = (int32_t)y;
    }
    else if (draw_y_i >= ((int32_t)y + (int32_t)height))
    {
      draw_y_i = (int32_t)y + (int32_t)height - 1L;
    }

    draw_y = (uint8_t)draw_y_i;
    if ((dotted == 0U) || ((i & 1U) == 0U))
    {
      ssd1306_DrawPixel((uint8_t)(x + i), draw_y, SSD1306_COLOR_WHITE);
    }

    if ((dotted == 0U) && (i != 0U))
    {
      if (prev_y < draw_y)
      {
        for (line_y = prev_y; line_y <= draw_y; line_y++)
        {
          ssd1306_DrawPixel((uint8_t)(x + i), line_y, SSD1306_COLOR_WHITE);
        }
      }
      else
      {
        for (line_y = draw_y; line_y <= prev_y; line_y++)
        {
          ssd1306_DrawPixel((uint8_t)(x + i), line_y, SSD1306_COLOR_WHITE);
        }
      }
    }

    /* 在波形底部绘制脉搏标记箭头（向下三角形），标识每个检测到的心搏位置。 */
    if ((markers != 0U) && (waveform->markers[sample_index] != 0U) && (height >= 8U))
    {
      uint8_t marker_y = (uint8_t)(y + height - 4U);
      ssd1306_DrawPixel((uint8_t)(x + i), marker_y, SSD1306_COLOR_WHITE);
      if (marker_y + 1U < (uint8_t)(y + height))
      {
        ssd1306_DrawPixel((uint8_t)(x + i), (uint8_t)(marker_y + 1U), SSD1306_COLOR_WHITE);
      }
      if ((i > 0U) && (marker_y + 2U < (uint8_t)(y + height)))
      {
        ssd1306_DrawPixel((uint8_t)(x + i - 1U), (uint8_t)(marker_y + 2U), SSD1306_COLOR_WHITE);
      }
      if (((uint8_t)(x + i + 1U) < SSD1306_WIDTH) && (marker_y + 2U < (uint8_t)(y + height)))
      {
        ssd1306_DrawPixel((uint8_t)(x + i + 1U), (uint8_t)(marker_y + 2U), SSD1306_COLOR_WHITE);
      }
    }

    prev_y = draw_y;
  }
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

