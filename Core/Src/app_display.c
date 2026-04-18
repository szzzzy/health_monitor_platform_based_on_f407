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
 * 波形显示先做慢速直流跟踪，再做归一化。
 * 这样即使原始值量级很大，也能在 OLED 上看到真正有意义的脉搏起伏。
 */
#define WAVEFORM_DC_FILTER_SHIFT   5U
#define WAVEFORM_NORMALIZE_SCALE   4096L

/* OLED 波形缓冲：保存最近一屏样本及当前 DC 估计。 */
typedef struct
{
  int16_t samples[SSD1306_WIDTH];
  uint16_t write_index;
  uint16_t sample_count;
  uint32_t dc_estimate;
} WaveformBuffer_t;

static WaveformBuffer_t ir_waveform;
static WaveformBuffer_t red_waveform;
static const uint8_t display_brightness_table[DISPLAY_BRIGHTNESS_LEVEL_COUNT] = {0x20U, 0x7FU, 0xFFU};
static const char *const display_brightness_label_table[DISPLAY_BRIGHTNESS_LEVEL_COUNT] = {"LOW", "MID", "HIGH"};

static uint8_t page_button_poll_pressed(PageButton_t *button);
static void waveform_buffer_reset(WaveformBuffer_t *waveform);
static void waveform_buffer_add_sample(WaveformBuffer_t *waveform, uint32_t raw_value);
static void waveform_buffer_draw(const WaveformBuffer_t *waveform,
                                 uint8_t x,
                                 uint8_t y,
                                 uint8_t width,
                                 uint8_t height);
static void app_display_debug_page(const AppState_t *app);
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
void app_display_add_ir_sample(uint32_t raw_value)
{
  waveform_buffer_add_sample(&ir_waveform, raw_value);
}

/* SpO2 页面使用的波形样本入口。 */
void app_display_add_red_sample(uint32_t raw_value)
{
  waveform_buffer_add_sample(&red_waveform, raw_value);
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
  char time_line[32];
  char date_line[32];
  char measurement_line[32];
  char status_line[32];
  const char *signal_label;
  const WaveformBuffer_t *selected_waveform;

  if (app == NULL)
  {
    return;
  }

  if (app->current_page == DISPLAY_PAGE_DEBUG)
  {
    app_display_debug_page(app);
    return;
  }

  app_format_rtc_lines(app, time_line, sizeof(time_line), date_line, sizeof(date_line));

  if (app->finger_present != 0U)
  {
    signal_label = (app->current_page == DISPLAY_PAGE_BPM) ? "IR WAVE" : "RED WAVE";
  }
  else if (app->raw_signal_present != 0U)
  {
    signal_label = (app->current_page == DISPLAY_PAGE_BPM) ? "IR LIVE" : "RED LIVE";
  }
  else
  {
    signal_label = "NO FINGER";
  }

  if (app->current_page == DISPLAY_PAGE_BPM)
  {
    selected_waveform = &ir_waveform;

    if (app->finger_present == 0U)
    {
      (void)snprintf(measurement_line,
                     sizeof(measurement_line),
                     "BPM:-- %s",
                     signal_label);
    }
    else if (app->bpm_valid != 0U)
    {
      (void)snprintf(measurement_line,
                     sizeof(measurement_line),
                     "BPM:%u %s",
                     (unsigned int)app->bpm_value,
                     signal_label);
    }
    else
    {
      (void)snprintf(measurement_line,
                     sizeof(measurement_line),
                     "BPM:-- %s",
                     signal_label);
    }
  }
  else
  {
    selected_waveform = &red_waveform;

    if (app->finger_present == 0U)
    {
      (void)snprintf(measurement_line,
                     sizeof(measurement_line),
                     "SPO2:-- %s",
                     signal_label);
    }
    else if (app->spo2_valid != 0U)
    {
      (void)snprintf(measurement_line,
                     sizeof(measurement_line),
                     "SPO2:%uP %s",
                     (unsigned int)app->spo2_value,
                     signal_label);
    }
    else
    {
      (void)snprintf(measurement_line,
                     sizeof(measurement_line),
                     "SPO2:-- %s",
                     signal_label);
    }
  }

  if (app->rtc_read_ok == 0U)
  {
    (void)snprintf(status_line,
                   sizeof(status_line),
                   "RTC ERR S:%s B:%s",
                   app_get_sensor_status_string(app),
                   app_get_brightness_short_label(app->display_brightness_index));
  }
  else if (app->rtc_time_valid != 0U)
  {
    (void)snprintf(status_line,
                   sizeof(status_line),
                   "RTC SYNC S:%s B:%s",
                   app_get_sensor_status_string(app),
                   app_get_brightness_short_label(app->display_brightness_index));
  }
  else
  {
    (void)snprintf(status_line,
                   sizeof(status_line),
                   "RTC RUN S:%s B:%s",
                   app_get_sensor_status_string(app),
                   app_get_brightness_short_label(app->display_brightness_index));
  }

  ssd1306_Clear(SSD1306_COLOR_BLACK);
  ssd1306_DrawString(0, 0, time_line);
  ssd1306_DrawString(0, 8, date_line);
  ssd1306_DrawString(0, 16, measurement_line);
  ssd1306_DrawString(0, 24, status_line);
  waveform_buffer_draw(selected_waveform,
                       0U,
                       OLED_WAVEFORM_TOP_Y,
                       SSD1306_WIDTH,
                       OLED_WAVEFORM_HEIGHT);
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
  }

  waveform->write_index = 0U;
  waveform->sample_count = 0U;
  waveform->dc_estimate = 0U;
}

/*
 * 压入一个原始样本，并顺手完成去直流与归一化。
 * 这样后续显示逻辑只需要关心“画图”，不必再处理原始值尺度问题。
 */
static void waveform_buffer_add_sample(WaveformBuffer_t *waveform, uint32_t raw_value)
{
  int32_t ac_component;
  int32_t normalized_sample;
  uint32_t dc_delta;

  if (waveform == NULL)
  {
    return;
  }

  if (waveform->dc_estimate == 0U)
  {
    waveform->dc_estimate = raw_value;
  }
  else if (waveform->dc_estimate < raw_value)
  {
    dc_delta = raw_value - waveform->dc_estimate;
    waveform->dc_estimate += ((dc_delta >> WAVEFORM_DC_FILTER_SHIFT) != 0U) ?
                             (dc_delta >> WAVEFORM_DC_FILTER_SHIFT) : 1U;
  }
  else if (waveform->dc_estimate > raw_value)
  {
    dc_delta = waveform->dc_estimate - raw_value;
    waveform->dc_estimate -= ((dc_delta >> WAVEFORM_DC_FILTER_SHIFT) != 0U) ?
                             (dc_delta >> WAVEFORM_DC_FILTER_SHIFT) : 1U;
  }

  ac_component = (int32_t)raw_value - (int32_t)waveform->dc_estimate;
  if (waveform->dc_estimate == 0U)
  {
    normalized_sample = 0;
  }
  else
  {
    normalized_sample = (int32_t)(((int64_t)ac_component * WAVEFORM_NORMALIZE_SCALE) /
                                  (int64_t)waveform->dc_estimate);
  }

  if (normalized_sample > 32767)
  {
    normalized_sample = 32767;
  }
  else if (normalized_sample < -32768)
  {
    normalized_sample = -32768;
  }

  waveform->samples[waveform->write_index] = (int16_t)normalized_sample;
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

/* 把最近一屏样本映射到 OLED 指定区域，绘制成连续折线。 */
static void waveform_buffer_draw(const WaveformBuffer_t *waveform,
                                 uint8_t x,
                                 uint8_t y,
                                 uint8_t width,
                                 uint8_t height)
{
  uint16_t i;
  uint16_t sample_count;
  uint16_t start_index;
  int16_t sample_min;
  int16_t sample_max;
  int16_t sample_value;
  uint8_t prev_y = 0U;
  uint8_t draw_y;
  uint8_t line_y;
  uint16_t sample_index;
  int32_t range;

  if ((waveform == NULL) || (width == 0U) || (height == 0U))
  {
    return;
  }

  if (waveform->sample_count == 0U)
  {
    draw_y = (uint8_t)(y + (height / 2U));
    for (i = 0U; i < width; i++)
    {
      ssd1306_DrawPixel((uint8_t)(x + i), draw_y, SSD1306_COLOR_WHITE);
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

  sample_index = start_index;
  sample_min = waveform->samples[sample_index];
  sample_max = waveform->samples[sample_index];

  for (i = 0U; i < sample_count; i++)
  {
    sample_index = (uint16_t)((start_index + i) % SSD1306_WIDTH);
    sample_value = waveform->samples[sample_index];
    if (sample_value < sample_min)
    {
      sample_min = sample_value;
    }
    if (sample_value > sample_max)
    {
      sample_max = sample_value;
    }
  }

  range = (int32_t)sample_max - (int32_t)sample_min;
  if (range == 0)
  {
    range = 1;
  }

  for (i = 0U; i < sample_count; i++)
  {
    sample_index = (uint16_t)((start_index + i) % SSD1306_WIDTH);
    sample_value = waveform->samples[sample_index];
    draw_y = (uint8_t)(y + ((int32_t)(sample_max - sample_value) * (int32_t)(height - 1U)) / range);
    ssd1306_DrawPixel((uint8_t)(x + i), draw_y, SSD1306_COLOR_WHITE);

    if (i != 0U)
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

