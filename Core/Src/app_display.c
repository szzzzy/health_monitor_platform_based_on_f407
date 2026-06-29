#include "app_display.h"

#include <stdio.h>

#include "adc.h"
#include "app_data_log.h"
#include "app_diag.h"
#include "app_ecg.h"
#include "app_measurement.h"
#include "app_sched_diag.h"
#include "app_sd_card.h"
#include "app_settings.h"
#include "ssd1306.h"

/* PE2/PE3/PE4 当前作为页面切换和调试按键输入，按下时为低电平。 */
#define DEBUG_TOGGLE_BUTTON_PORT     GPIOE
#define DEBUG_TOGGLE_BUTTON_PIN      GPIO_PIN_2
#define PAGE_PREV_BUTTON_PORT      GPIOE
#define PAGE_PREV_BUTTON_PIN       GPIO_PIN_3
#define PAGE_NEXT_BUTTON_PORT      GPIOE
#define PAGE_NEXT_BUTTON_PIN       GPIO_PIN_4
/* 轮询按键的简单消抖时间。 */
#define PAGE_BUTTON_DEBOUNCE_MS    120U
#define PAGE_BUTTON_DEBOUNCE_TICKS ((PAGE_BUTTON_DEBOUNCE_MS + APP_SAMPLE_PERIOD_MS - 1U) / APP_SAMPLE_PERIOD_MS)
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
  uint32_t min_scale;       /* AGC 最小缩放，PPG=32, ECG=200 */
} WaveformBuffer_t;

static WaveformBuffer_t ir_waveform;
static WaveformBuffer_t red_waveform;
static WaveformBuffer_t ecg_waveform;

/* 设置 ECG 波形独立最小 AGC scale，避免低幅噪声被放大 */
void app_display_set_ecg_min_scale(uint32_t min_scale)
{
  ecg_waveform.min_scale = min_scale;
  if (ecg_waveform.scale_estimate < min_scale) {
    ecg_waveform.scale_estimate = min_scale;
  }
}

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
                                 uint8_t markers,
                                 uint8_t stable_scale);
static uint8_t app_display_text_width_px(const char *s);
static void app_display_draw_right(uint8_t y, const char *s);
static const char *app_get_finger_status(const AppState_t *app, char *buf, size_t buf_size);
static void app_display_pulse_page(const AppState_t *app);
static void app_display_oxy_page(const AppState_t *app);
static void app_display_vitals_page(const AppState_t *app);
static void app_display_ecg_page(const AppState_t *app);
static void app_display_ecg_quality_page(const AppState_t *app);
static void app_display_debug_d1_max(const AppState_t *app);
static void app_display_debug_d2_fifo(const AppState_t *app);
static void app_display_debug_d3_ppg_raw(const AppState_t *app);
static void app_display_debug_d4_ppg_q(const AppState_t *app);
static void app_display_debug_d5_algo(const AppState_t *app);
static void app_display_debug_d6_sys(const AppState_t *app);
static void app_display_debug_d7_sd(const AppState_t *app);
static void app_display_debug_d8_ecg(const AppState_t *app);
static void app_display_debug_d9_sched(const AppState_t *app);
static void app_display_debug_d10_ecg_q(const AppState_t *app);
static void app_display_draw_lines_8(const char line[8][24]);
static uint32_t app_display_cap_u32(uint32_t value, uint32_t cap);
static const char *app_get_quality_label(const AppState_t *app);
static const char *app_get_balance_label(uint8_t balance_status);
static const char *app_get_regular_label(const AppState_t *app);
static const char *app_get_rtc_status_label(const AppState_t *app);
static const char *app_get_weekday_string(uint8_t weekday);
static void app_format_rtc_lines(const AppState_t *app,
                                 char *time_line,
                                 size_t time_line_size,
                                 char *date_line,
                                 size_t date_line_size);

/**
 * @brief  初始化应用结构体中的显示状态。
 * @param  app 指向待初始化的应用状态结构体。
 * @note   将默认页面设置为 DISPLAY_PAGE_BPM，重置调试模式，
 *         配置按键端口/引脚分配，并重置 ECG 波形。
 *         display_refresh_requested 被置为 1 以强制首次渲染。
 */
void app_display_init_state(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  app->current_page = DISPLAY_PAGE_BPM;
  app->display_refresh_requested = 1U;
  app->page_mode = PAGE_MODE_NORMAL;
  app->debug_sub_page = DBG_SUB_D1_MAX;
  app->settings_sub_page = SETTINGS_SUB_IDENTITY;
  app->saved_normal_page = DISPLAY_PAGE_BPM;
  app->debug_toggle_button.port = DEBUG_TOGGLE_BUTTON_PORT;
  app->debug_toggle_button.pin = DEBUG_TOGGLE_BUTTON_PIN;
  app->page_prev_button.port = PAGE_PREV_BUTTON_PORT;
  app->page_prev_button.pin = PAGE_PREV_BUTTON_PIN;
  app->page_next_button.port = PAGE_NEXT_BUTTON_PORT;
  app->page_next_button.pin = PAGE_NEXT_BUTTON_PIN;

  /* PPG 波形保持原始 MIN_SCALE=32 */
  ir_waveform.min_scale  = WAVEFORM_AGC_MIN_SCALE;
  red_waveform.min_scale = WAVEFORM_AGC_MIN_SCALE;
  /* ECG 独立 min_scale=200：低幅噪声在 ±600 可视范围仅占 ~1.3 px/pixel
   * 而 R 峰 (200-600 LSB 视觉输出) 仍占 16-48 px，清晰可见 */
  ecg_waveform.min_scale = 200UL;

  waveform_buffer_reset(&ecg_waveform);
}

/**
 * @brief  同时清空 IR 和 RED 波形缓冲。
 * @note   便捷包装函数，重置 ir_waveform 和 red_waveform。
 *         通常在手指接触状态变化或传感器恢复后被调用，以丢弃过时的波形数据。
 */
/* 同时清空 IR / RED 两条波形缓冲。 */
void app_display_reset_waveforms(void)
{
  waveform_buffer_reset(&ir_waveform);
  waveform_buffer_reset(&red_waveform);
}

/**
 * @brief  向 IR 波形缓冲（脉搏页面）压入一个带通滤波后的 IR 样本。
 * @param  filtered_value 带通滤波后的 IR 交流样本值。
 * @note   委托 waveform_buffer_add_sample 操作 IR 波形缓冲。
 *         样本将被绘制在 PULSE 页面的主波形区域。
 */
/* IR 页面使用的波形样本入口。 */
void app_display_add_ir_sample(int32_t filtered_value)
{
  waveform_buffer_add_sample(&ir_waveform, filtered_value);
}

/**
 * @brief  向 RED 波形缓冲（SpO2 页面）压入一个带通滤波后的 RED 样本。
 * @param  filtered_value 带通滤波后的 RED 交流样本值。
 * @note   委托 waveform_buffer_add_sample 操作 RED 波形缓冲。
 *         样本将被绘制在 OXY 页面的下方波形区域。
 */
/* SpO2 页面使用的波形样本入口。 */
void app_display_add_red_sample(int32_t filtered_value)
{
  waveform_buffer_add_sample(&red_waveform, filtered_value);
}

/**
 * @brief  在 IR 波形缓冲中将最新样本标记为检测到的脉搏点。
 * @note   委托 waveform_buffer_mark_latest 操作 IR 缓冲。
 *         标记在 PULSE 页面上渲染为波形区域底部的小箭头。
 */
void app_display_add_ir_pulse_marker(void)
{
  waveform_buffer_mark_latest(&ir_waveform);
}

/**
 * @brief  将 ECG 波形缓冲重置为初始空状态。
 * @note   委托 waveform_buffer_reset 操作 ecg_waveform 缓冲。
 *         在 ECG 导联配置更改或导联脱落恢复后被调用，以丢弃过时的波形数据。
 */
void app_display_reset_ecg_waveform(void)
{
  waveform_buffer_reset(&ecg_waveform);
}

/**
 * @brief  向 ECG 波形缓冲压入一个带通滤波后的 ECG 样本。
 * @param  filtered_value 滤波后的 ECG 样本值。
 * @note   委托 waveform_buffer_add_sample 操作 ecg_waveform 缓冲。
 *         样本将被绘制在 ECG 页面的全高波形区域。
 *         该缓冲与 MAX30102 接触状态无关。
 */
void app_display_add_ecg_sample(int32_t filtered_value)
{
  waveform_buffer_add_sample(&ecg_waveform, filtered_value);
}

/**
 * @brief  在 ECG 波形缓冲中将最新样本标记为检测到的 R 峰。
 * @note   委托 waveform_buffer_mark_latest 操作 ecg_waveform 缓冲。
 *         标记在 ECG 页面上渲染为 ECG 波形区域底部的小指示符。
 */
void app_display_add_ecg_r_peak_marker(void)
{
  waveform_buffer_mark_latest(&ecg_waveform);
}

/**
 * @brief  轮询页面导航和调试切换按键。
 * @param  app 指向应用状态的指针（页面和调试状态被更新）。
 * @note   PE2 切换调试模式开/关。正常模式下，PE3/PE4 循环切换
 *         测量页面 (DISPLAY_PAGE_NORMAL_COUNT)。调试模式下，
 *         PE3/PE4 循环切换调试子页面。使用 page_button_poll_pressed()
 *         进行消抖。任何页面切换后都会设置 display_refresh_requested。
 */
/* 处理页面切换按键，让 UI 切换与测量处理保持解耦。 */
void app_display_handle_buttons(AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  /* PE2：在 NORMAL → DEBUG → SETTINGS 三种模式间循环 */
  if (page_button_poll_pressed(&app->debug_toggle_button) != 0U)
  {
    if (app->page_mode == PAGE_MODE_NORMAL)
    {
      app->saved_normal_page = app->current_page;
      app->page_mode = PAGE_MODE_DEBUG;
      app->debug_sub_page = DBG_SUB_D1_MAX;
    }
    else if (app->page_mode == PAGE_MODE_DEBUG)
    {
      app->page_mode = PAGE_MODE_SETTINGS;
      app->settings_sub_page = SETTINGS_SUB_IDENTITY;
    }
    else
    {
      app->page_mode = PAGE_MODE_NORMAL;
      app->current_page = app->saved_normal_page;
    }
    app->display_refresh_requested = 1U;
    return;
  }

  if (app->page_mode == PAGE_MODE_DEBUG)
  {
    /* 调试模式下 PE3/PE4 在调试子页之间切换 */
    if (page_button_poll_pressed(&app->page_prev_button) != 0U)
    {
      if (app->debug_sub_page == DBG_SUB_D1_MAX)
      {
        app->debug_sub_page = (DebugSubPage_t)(DBG_SUB_COUNT - 1U);
      }
      else
      {
        app->debug_sub_page = (DebugSubPage_t)(app->debug_sub_page - 1U);
      }
      app->display_refresh_requested = 1U;
    }

    if (page_button_poll_pressed(&app->page_next_button) != 0U)
    {
      app->debug_sub_page = (DebugSubPage_t)((app->debug_sub_page + 1U) % DBG_SUB_COUNT);
      app->display_refresh_requested = 1U;
    }
  }
  else if (app->page_mode == PAGE_MODE_SETTINGS)
  {
    /* 设置模式下 PE3/PE4 在设置子页之间切换 */
    if (page_button_poll_pressed(&app->page_prev_button) != 0U)
    {
      if (app->settings_sub_page == SETTINGS_SUB_IDENTITY)
      {
        app->settings_sub_page = (SettingsSubPage_t)(SETTINGS_SUB_COUNT - 1U);
      }
      else
      {
        app->settings_sub_page = (SettingsSubPage_t)(app->settings_sub_page - 1U);
      }
      app->display_refresh_requested = 1U;
    }

    if (page_button_poll_pressed(&app->page_next_button) != 0U)
    {
      app->settings_sub_page = (SettingsSubPage_t)((app->settings_sub_page + 1U) % SETTINGS_SUB_COUNT);
      app->display_refresh_requested = 1U;
    }
  }
  else
  {
    /* 常规模式下 PE3/PE4 在普通页面之间切换（不含 DEBUG） */
    if (page_button_poll_pressed(&app->page_prev_button) != 0U)
    {
      if (app->current_page == DISPLAY_PAGE_BPM)
      {
        app->current_page = (DisplayPage_t)(DISPLAY_PAGE_NORMAL_COUNT - 1U);
      }
      else
      {
        app->current_page = (DisplayPage_t)(app->current_page - 1U);
      }
      app->display_refresh_requested = 1U;
    }

    if (page_button_poll_pressed(&app->page_next_button) != 0U)
    {
      app->current_page = (DisplayPage_t)((app->current_page + 1U) % DISPLAY_PAGE_NORMAL_COUNT);
      app->display_refresh_requested = 1U;
    }
  }
}

/**
 * @brief  根据当前页面和调试模式渲染主测量页面。
 * @param  app 指向应用状态的指针（页面选择、测量数据）。
 * @note   调试模式下，派发到对应的调试子页面渲染器（d1 至 d7）。
 *         正常模式下，派发到当前页面渲染器：PULSE、OXY、VITALS 或 ECG。
 *         如果 app 为 NULL，则立即返回而不渲染。
 */
/* 根据当前页面与状态，绘制主测量页面。 */
void app_display_measurement_page(const AppState_t *app)
{
  if (app == NULL)
  {
    return;
  }

  /* 调试模式下绘制调试子页面 */
  if (app->page_mode == PAGE_MODE_DEBUG)
  {
    switch (app->debug_sub_page)
    {
      case DBG_SUB_D2_FIFO:     app_display_debug_d2_fifo(app);     break;
      case DBG_SUB_D3_PPG_RAW:  app_display_debug_d3_ppg_raw(app);  break;
      case DBG_SUB_D4_PPG_Q:    app_display_debug_d4_ppg_q(app);    break;
      case DBG_SUB_D5_ALGO:     app_display_debug_d5_algo(app);     break;
      case DBG_SUB_D6_SYS:      app_display_debug_d6_sys(app);      break;
      case DBG_SUB_D7_SD:       app_display_debug_d7_sd(app);       break;
      case DBG_SUB_D8_ECG:      app_display_debug_d8_ecg(app);      break;
      case DBG_SUB_D9_SCHED:    app_display_debug_d9_sched(app);    break;
      case DBG_SUB_D10_ECG_Q:  app_display_debug_d10_ecg_q(app);  break;
      case DBG_SUB_D1_MAX:
      default:                  app_display_debug_d1_max(app);      break;
    }
    return;
  }

  /* 设置模式下绘制设置子页面 */
  if (app->page_mode == PAGE_MODE_SETTINGS)
  {
    app_settings_render_page(app);
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

    case DISPLAY_PAGE_ECG_QUALITY:
      app_display_ecg_quality_page(app);
      break;

    case DISPLAY_PAGE_BPM:
    default:
      app_display_pulse_page(app);
      break;
  }
}

/**
 * @brief  渲染 PULSE 页面：心率、IBI、SQ 和全高 IR 波形。
 * @param  app 指向应用状态的指针（HR、IBI、SQ、手指状态）。
 * @note   布局：y=0 显示 HR+IBI（左侧）和 SQ（右侧）；y=8 显示 REG/标签
 *         （左侧）和页面标题（右侧）；y=16..63 为 48px 的 IR 波形。
 *         如果手指未放置，则居中显示状态信息。
 */
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
  waveform_buffer_draw(&ir_waveform, 0U, 16U, SSD1306_WIDTH, 48U, 0U, 1U, 0U);
  ssd1306_UpdateScreen();
}

/**
 * @brief  渲染 OXY 页面：SpO2、比率、PI、IR + RED 双波形。
 * @param  app 指向应用状态的指针（SpO2、比率、PI、平衡）。
 * @note   布局：y=0 显示 SpO2+R（左侧）和 PI（右侧）；y=8 显示 BAL+SQ
 *         （左侧）和标题（右侧）；y=16..39 为 24px 的 IR 波形；
 *         y=40..63 为 24px 的 RED 波形（虚线）。如果手指未放置，
 *         则居中显示状态信息。
 */
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
  waveform_buffer_draw(&ir_waveform,  0U, 16U, SSD1306_WIDTH, 24U, 0U, 0U, 0U);
  waveform_buffer_draw(&red_waveform, 0U, 40U, SSD1306_WIDTH, 24U, 1U, 0U, 0U);
  ssd1306_UpdateScreen();
}

/**
 * @brief  渲染 VITALS 汇总页面：HR、RR、IBI、HRV、PI、RTC。
 * @param  app 指向应用状态的指针（所有生命体征字段）。
 * @note   布局：8 行文本，显示 HR+RR、IBI、SDNN+RMSSD、SD1+SD2、PI、
 *         RTC 状态、RTC 时间和 RTC 日期。无波形区域——纯文本。
 *         如果手指未放置，则显示状态信息和 RTC 信息。
 */
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

  /* --- y=40: RTC 状态 --- */
  (void)snprintf(line5, sizeof(line5), "%s", app_get_rtc_status_label(app));
  app_format_rtc_lines(app, time_line, sizeof(time_line), date_line, sizeof(date_line));

  /* --- 绘制 --- */
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

/**
 * @brief  渲染 ECG 页面：ECG 心率、RR 间期、PTT 和滤波波形。
 * @param  app 指向应用状态的指针（ECG 和 PTT 字段）。
 * @note   布局：y=0 显示 HR+RR（左侧）和 "ECG" 标题（右侧）；y=8 显示
 *         PTT（左侧）和滤波幅度（右侧）；y=16..63 为 48px 的 ECG 波形。
 *         导联脱落时显示 "LEAD OFF" 和电极检查信息。ECG 波形与 MAX30102 接触无关。
 */
/*
 * ECG 页面：ECG HR/RR + PTT 文本和一个全高滤波 ECG 波形。
 * 波形缓冲与 MAX30102 接触状态无关。
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
    /* 红(LO-)/绿(LO+) 电极脱落。黄色 RL 无法通过 LO 引脚检测，此处不体现。 */
    (void)snprintf(line0, sizeof(line0), "LEAD OFF:%u",
                   (unsigned int)app->ecg_lead_off);
    (void)snprintf(line1, sizeof(line1), "CHECK RED/GREEN");
    (void)snprintf(line1r, sizeof(line1r), "RL n/a");
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
    (void)snprintf(line1r, sizeof(line1r), "LO:%u",
                   (unsigned int)app_ecg_read_lead_off_raw());
  }

  ssd1306_Clear(SSD1306_COLOR_BLACK);
  ssd1306_DrawString(0, 0, line0);
  app_display_draw_right(0, line0r);
  ssd1306_DrawString(0, 8, line1);
  app_display_draw_right(8, line1r);
  waveform_buffer_draw(&ecg_waveform, 0U, 16U, SSD1306_WIDTH, 48U, 0U, 1U, 1U);
  ssd1306_UpdateScreen();
}

/**
 * @brief  渲染 ECG 质量页面：SQ/reason、HR/RR/PTT 和全高 ECG 波形。
 * @param  app 指向应用状态的指针（ECG 质量和 PTT 字段）。
 * @note   布局：y=0 显示 SQ+原因（左侧）和 "E Q" 标题（右侧）；y=8 显示
 *         HR/RR/PTT（左侧）和 LO 状态（右侧）；y=16..63 为 48px 的 ECG 波形。
 */
static void app_display_ecg_quality_page(const AppState_t *app)
{
  char line0[32];
  char line0r[8];
  char line1[32];
  char line1r[16];
  const char *reason_str;

  if (app == NULL) { return; }

  (void)snprintf(line0r, sizeof(line0r), "E Q");

  /* reason string */
  switch (app->ecg_invalid_reason) {
  case ECG_INVALID_OK:            reason_str = "OK";    break;
  case ECG_INVALID_LEAD_OFF:      reason_str = "LEAD";  break;
  case ECG_INVALID_ADC_SAT:       reason_str = "SAT";   break;
  case ECG_INVALID_DMA_OVERFLOW:  reason_str = "DMA";   break;
  case ECG_INVALID_NO_R_PEAK:     reason_str = "NoR";   break;
  case ECG_INVALID_LOW_AMPLITUDE: reason_str = "LoA";   break;
  case ECG_INVALID_NOISY:         reason_str = "NOIS";  break;
  case ECG_INVALID_RAW_FLATLINE:  reason_str = "FLAT";  break;
  default:                         reason_str = "?";     break;
  }

  (void)snprintf(line0, sizeof(line0), "SQ:%u R:%s",
                 (unsigned int)app->ecg_signal_quality, reason_str);

  if (app->ecg_lead_off != 0U)
  {
    (void)snprintf(line1, sizeof(line1), "LEAD OFF");
    (void)snprintf(line1r, sizeof(line1r), "LO:%u",
                   (unsigned int)app_ecg_read_lead_off_raw());
  }
  else
  {
    char hr_str[8], rr_str[8], ptt_str[8];

    if ((app->ecg_valid != 0U) || (app->ecg_hr != 0U))
      (void)snprintf(hr_str, sizeof(hr_str), "%u", (unsigned int)app->ecg_hr);
    else
      (void)snprintf(hr_str, sizeof(hr_str), "--");

    if (app->ecg_rr_ms != 0U)
      (void)snprintf(rr_str, sizeof(rr_str), "%u", (unsigned int)app->ecg_rr_ms);
    else
      (void)snprintf(rr_str, sizeof(rr_str), "--");

    if ((app->ptt_valid != 0U) || (app->ptt_ms != 0U))
      (void)snprintf(ptt_str, sizeof(ptt_str), "%u", (unsigned int)app->ptt_ms);
    else
      (void)snprintf(ptt_str, sizeof(ptt_str), "--");

    (void)snprintf(line1, sizeof(line1), "H%s R%s P%s", hr_str, rr_str, ptt_str);
    (void)snprintf(line1r, sizeof(line1r), "LO:%u",
                   (unsigned int)app_ecg_read_lead_off_raw());
  }

  ssd1306_Clear(SSD1306_COLOR_BLACK);
  ssd1306_DrawString(0, 0, line0);
  app_display_draw_right(0, line0r);
  ssd1306_DrawString(0, 8, line1);
  app_display_draw_right(8, line1r);
  waveform_buffer_draw(&ecg_waveform, 0U, 16U, SSD1306_WIDTH, 48U, 0U, 1U, 1U);
  ssd1306_UpdateScreen();
}

/**
 * @brief  调试页面 D1：MAX30102 传感器健康状态、I2C 错误、恢复状态。
 * @param  app 指向应用状态的指针（传感器健康字段）。
 * @note   显示传感器健康状态、I2C 错误码、错误连续次数、
 *         恢复次数、恢复失败次数、样本年龄和过时计数。
 *         读取状态显示为 OK/W/E。
 */
/* =========================================================================
 * D1 MAX — MAX30102 传感器健康、I2C、恢复状态
 * ========================================================================= */

static void app_display_draw_lines_8(const char line[8][24])
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

static uint32_t app_display_cap_u32(uint32_t value, uint32_t cap)
{
  return (value > cap) ? cap : value;
}

static void app_display_debug_d1_max(const AppState_t *app)
{
  char line[8][24];
  const char *health_str;
  const char *read_str;
  uint32_t age_s;

  if (app == NULL) { return; }

  age_s = (app->sensor_last_sample_tick != 0U) ?
          ((HAL_GetTick() - app->sensor_last_sample_tick) / 1000UL) : 0UL;
  age_s = app_display_cap_u32(age_s, 9999UL);

  switch (app->sensor_health)
  {
  case SENSOR_HEALTH_OK:         health_str = "OK"; break;
  case SENSOR_HEALTH_STALE:      health_str = "STALE"; break;
  case SENSOR_HEALTH_RECOVERING: health_str = "RECOV"; break;
  case SENSOR_HEALTH_I2C_ERR:    health_str = "I2C"; break;
  case SENSOR_HEALTH_INIT_FAIL:  health_str = "INIT"; break;
  case SENSOR_HEALTH_FIFO_CLEAR_FAIL: health_str = "FIFO"; break;
  default:                       health_str = "?"; break;
  }
  switch (app->sensor_last_read_status)
  {
  case APP_MEASUREMENT_READ_OK:   read_str = "OK"; break;
  case APP_MEASUREMENT_READ_WAIT: read_str = "W"; break;
  default:                        read_str = "E"; break;
  }

  (void)snprintf(line[0], sizeof(line[0]), "D1 MAX H:%s", health_str);
  (void)snprintf(line[1], sizeof(line[1]), "ID-- MODE--");
  (void)snprintf(line[2], sizeof(line[2]), "SPO2 -- FIFO --");
  (void)snprintf(line[3], sizeof(line[3]), "LED1 -- LED2 --");
  (void)snprintf(line[4], sizeof(line[4]), "I2C %lu STR %u",
                 (unsigned long)(app->sensor_last_i2c_error > 9999UL ?
                                 9999UL : app->sensor_last_i2c_error),
                 (unsigned int)app->sensor_error_streak);
  (void)snprintf(line[5], sizeof(line[5]), "REC %lu F %lu",
                 (unsigned long)(app->sensor_recover_count > 9999UL ?
                                 9999UL : app->sensor_recover_count),
                 (unsigned long)(app->sensor_recovery_fail_count > 9999UL ?
                                 9999UL : app->sensor_recovery_fail_count));
  (void)snprintf(line[6], sizeof(line[6]), "AGE %lus READ %s",
                 (unsigned long)age_s, read_str);
  (void)snprintf(line[7], sizeof(line[7]), "STALE %lu",
                 (unsigned long)(app->sensor_stale_count > 9999UL ?
                                 9999UL : app->sensor_stale_count));

  app_display_draw_lines_8(line);
}

/**
 * @brief  调试页面 D2：FIFO 状态、读样本链路统计。
 * @param  app 指向应用状态的指针（FIFO 和读取计数器）。
 * @note   显示读取状态、尝试/成功/忙/错误计数、溢出计数、
 *         FIFO 写/读指针和可用样本数、过时计数，
 *         以及自上次成功读取以来的时间。
 */
/* =========================================================================
 * D2 FIFO — FIFO 状态 + 读样本链路统计
 * ========================================================================= */
static void app_display_debug_d2_fifo(const AppState_t *app)
{
  char line[8][24];
  const char *read_str;
  uint32_t att;
  uint32_t ok;
  uint32_t busy;
  uint32_t err;
  uint32_t last_ok_age;

  if (app == NULL) { return; }

  switch (app->sensor_last_read_status)
  {
  case APP_MEASUREMENT_READ_OK:   read_str = "OK"; break;
  case APP_MEASUREMENT_READ_WAIT: read_str = "W"; break;
  default:                        read_str = "E"; break;
  }

  att  = app_display_cap_u32(app->sensor_read_attempt_count, 999999UL);
  ok   = app_display_cap_u32(app->sensor_read_ok_count,      999999UL);
  busy = app_display_cap_u32(app->sensor_read_busy_count,    999999UL);
  err  = app_display_cap_u32(app->sensor_read_error_count,   999999UL);
  last_ok_age = (app->sensor_last_ok_tick != 0U) ?
                ((HAL_GetTick() - app->sensor_last_ok_tick) / 1000UL) : 0UL;
  last_ok_age = app_display_cap_u32(last_ok_age, 9999UL);

  (void)snprintf(line[0], sizeof(line[0]), "D2 FIFO READ %s", read_str);
  (void)snprintf(line[1], sizeof(line[1]), "ATT %lu", (unsigned long)att);
  (void)snprintf(line[2], sizeof(line[2]), "OK %lu B %lu", (unsigned long)ok, (unsigned long)busy);
  (void)snprintf(line[3], sizeof(line[3]), "ERR %lu OVF %u",
                 (unsigned long)err, (unsigned int)app->sensor_fifo_overflow_count);
  (void)snprintf(line[4], sizeof(line[4]), "W %u R %u N %u",
                 (unsigned int)app->sensor_fifo_write_ptr,
                 (unsigned int)app->sensor_fifo_read_ptr,
                 (unsigned int)app->sensor_fifo_available_samples);
  (void)snprintf(line[5], sizeof(line[5]), "STALE %lu",
                 (unsigned long)(app->sensor_stale_count > 9999UL ?
                                 9999UL : app->sensor_stale_count));
  (void)snprintf(line[6], sizeof(line[6]), "LASTOK %lus", (unsigned long)last_ok_age);
  (void)snprintf(line[7], sizeof(line[7]), "");

  app_display_draw_lines_8(line);
}

/**
 * @brief  调试页面 D3：PPG 原始值 — RED、IR、基线、手指检测。
 * @param  app 指向应用状态的指针（原始 PPG 字段）。
 * @note   显示 RED 和 IR 原始值、IR 基线、信号差值和跨度、
 *         手指存在标志和原始信号标志，以及开/关确认计数。
 */
/* =========================================================================
 * D3 PPG RAW — RED/IR 原始值、基线、手指检测
 * ========================================================================= */
static void app_display_debug_d3_ppg_raw(const AppState_t *app)
{
  char line[8][24];

  if (app == NULL) { return; }

  (void)snprintf(line[0], sizeof(line[0]), "D3 PPG RAW");
  (void)snprintf(line[1], sizeof(line[1]), "RED %lu",
                 (unsigned long)app_display_cap_u32(app->red_value, 999999UL));
  (void)snprintf(line[2], sizeof(line[2]), "IR %lu",
                 (unsigned long)app_display_cap_u32(app->ir_value, 999999UL));
  (void)snprintf(line[3], sizeof(line[3]), "BASE %lu",
                 (unsigned long)app_display_cap_u32(app->baseline_ir, 999999UL));
  (void)snprintf(line[4], sizeof(line[4]), "DELTA %lu",
                 (unsigned long)app_display_cap_u32(app->ir_signal_delta, 999999UL));
  (void)snprintf(line[5], sizeof(line[5]), "SPAN I%lu R%lu",
                 (unsigned long)app_display_cap_u32(app->ir_signal_span, 999999UL),
                 (unsigned long)app_display_cap_u32(app->red_signal_span, 999999UL));
  (void)snprintf(line[6], sizeof(line[6]), "F %u RAW %u",
                 (unsigned int)app->finger_present, (unsigned int)app->raw_signal_present);
  (void)snprintf(line[7], sizeof(line[7]), "ON %u OFF %u",
                 (unsigned int)app->finger_on_confirm_count,
                 (unsigned int)app->finger_off_confirm_count);

  app_display_draw_lines_8(line);
}

/**
 * @brief  调试页面 D4：PPG 信号质量、PI、运动伪影、SpO2 比率。
 * @param  app 指向应用状态的指针（信号质量字段）。
 * @note   显示信号质量评分、平衡状态、IR/RED PI 值、
 *         IR/RED 交流 RMS 幅度、运动标志和分数，以及 SpO2 比率。
 */
/* =========================================================================
 * D4 PPG Q — 信号质量、PI、运动伪影、SpO2 比率
 * ========================================================================= */
static void app_display_debug_d4_ppg_q(const AppState_t *app)
{
  char line[8][24];
  const char *bal;
  uint32_t pi_i;
  uint32_t pi_r;

  if (app == NULL) { return; }

  bal = app_get_balance_label(app->spo2_balance_status);
  pi_i = (uint32_t)app->signal_ir_pi_x1000;
  pi_r = (uint32_t)app->signal_red_pi_x1000;

  (void)snprintf(line[0], sizeof(line[0]), "D4 PPG Q");
  (void)snprintf(line[1], sizeof(line[1]), "SQ %u BAL %s",
                 (unsigned int)app->signal_quality, bal);
  (void)snprintf(line[2], sizeof(line[2]), "PI I%u.%u",
                 (unsigned int)(pi_i / 10UL), (unsigned int)(pi_i % 10UL));
  (void)snprintf(line[3], sizeof(line[3]), "PI R%u.%u",
                 (unsigned int)(pi_r / 10UL), (unsigned int)(pi_r % 10UL));
  (void)snprintf(line[4], sizeof(line[4]), "AC I%lu R%lu",
                 (unsigned long)app_display_cap_u32(app->signal_ir_ac_rms, 999999UL),
                 (unsigned long)app_display_cap_u32(app->signal_red_ac_rms, 999999UL));
  (void)snprintf(line[5], sizeof(line[5]), "MOT %u SC %u",
                 (unsigned int)app->motion_artifact, (unsigned int)app->motion_score);
  (void)snprintf(line[6], sizeof(line[6]), "RATIO %u",
                 (unsigned int)app->spo2_ratio_x1000);
  (void)snprintf(line[7], sizeof(line[7]), "");

  app_display_draw_lines_8(line);
}

/**
 * @brief  调试页面 D5：算法输出结果 — HR、SpO2、RR、IBI、HRV、PTT。
 * @param  app 指向应用状态的指针（算法输出字段）。
 * @note   显示所有主要生命体征及其有效标志：
 *         HR、SpO2、RR、IBI（各带有效标志）、SDNN、RMSSD 和 PTT。
 */
/* =========================================================================
 * D5 ALGO — 算法输出结果
 * ========================================================================= */
static void app_display_debug_d5_algo(const AppState_t *app)
{
  char line[8][24];

  if (app == NULL) { return; }

  (void)snprintf(line[0], sizeof(line[0]), "D5 ALGO");
  (void)snprintf(line[1], sizeof(line[1]), "HR %u V%u",
                 (unsigned int)app->bpm_value, (unsigned int)app->bpm_valid);
  (void)snprintf(line[2], sizeof(line[2]), "SPO2 %u V%u",
                 (unsigned int)app->spo2_value, (unsigned int)app->spo2_valid);
  (void)snprintf(line[3], sizeof(line[3]), "RR %u V%u",
                 (unsigned int)app->rr_bpm, (unsigned int)app->rr_valid);
  (void)snprintf(line[4], sizeof(line[4]), "IBI %u V%u",
                 (unsigned int)app->latest_ibi_ms, (unsigned int)app->ibi_valid);
  (void)snprintf(line[5], sizeof(line[5]), "SDNN %u",
                 (unsigned int)(app->hrv_sdnn_ms > 9999U ? 9999U : app->hrv_sdnn_ms));
  (void)snprintf(line[6], sizeof(line[6]), "RMSSD %u",
                 (unsigned int)(app->hrv_rmssd_ms > 9999U ? 9999U : app->hrv_rmssd_ms));
  (void)snprintf(line[7], sizeof(line[7]), "PTT %u V%u",
                 (unsigned int)app->ptt_ms, (unsigned int)app->ptt_valid);

  app_display_draw_lines_8(line);
}

/**
 * @brief  调试页面 D6：系统级状态 — RTC、UART、SD 卡、显示。
 * @param  app 指向应用状态的指针（系统诊断信息）。
 * @note   显示重启次数、崩溃信息（来源/任务/阶段）、栈高水位
 *         标记（MAX/UI/SD 任务）、任务阶段码、RTC 状态、SD 状态、
 *         缓冲/写入/丢弃/暂停计数，以及显示刷新统计。
 */
/* =========================================================================
 * D6 SYS — 系统层状态（RTC、UART、SD、显示）
 * ========================================================================= */
static void app_display_debug_d6_sys(const AppState_t *app)
{
  char line[8][24];
  char crash_line[24];
  char hwm_line[24];
  char phase_line[24];
  const char *sd_state_str;
  const char *rtc;
  const char *crash_name;

  if (app == NULL) { return; }

  if (app->rtc_read_ok == 0U)       rtc = "ERR";
  else if (app->rtc_time_valid != 0U) rtc = "SET";
  else                               rtc = "RUN";

  sd_state_str = APP_DataLog_StateLabel(app->sd_state);

  /* 崩溃源名称 */
  switch (app->crash_source)
  {
  case DIAG_CRASH_HARDFAULT:     crash_name = "HF"; break;
  case DIAG_CRASH_MEMMANAGE:     crash_name = "MM"; break;
  case DIAG_CRASH_BUSFAULT:      crash_name = "BF"; break;
  case DIAG_CRASH_USAGEFAULT:    crash_name = "UF"; break;
  case DIAG_CRASH_STACKOVF:      crash_name = "SO"; break;
  case DIAG_CRASH_ASSERT:        crash_name = "AS"; break;
  case DIAG_CRASH_MALLOCFAIL:    crash_name = "MF"; break;
  case DIAG_CRASH_NMI:           crash_name = "NM"; break;
  case DIAG_CRASH_ERROR_HANDLER: crash_name = "EH"; break;
  default:                       crash_name = "--"; break;
  }

  if (app->crash_flag != 0U)
    (void)snprintf(crash_line, sizeof(crash_line), "CR %s T%u@%u",
                   crash_name,
                   (unsigned int)app->crash_task,
                   (unsigned int)app->crash_phase);
  else
    (void)snprintf(crash_line, sizeof(crash_line), "RST=0x%04lX",
                   (unsigned long)(app->reset_flags & 0xFFFFU));

  (void)snprintf(hwm_line, sizeof(hwm_line), "M%u U%u S%u",
                 (unsigned int)app->max_task_stack_hwm,
                 (unsigned int)app->ui_task_stack_hwm,
                 (unsigned int)app->sd_task_stack_hwm);

  (void)snprintf(phase_line, sizeof(phase_line), "PH M%uU%uS%uW%u",
                 (unsigned int)app->max_task_phase,
                 (unsigned int)app->ui_task_phase,
                 (unsigned int)app->sd_task_phase,
                 (unsigned int)app->wdt_task_phase);

  (void)snprintf(line[0], sizeof(line[0]), "D6 SYS RB%lu",
                 (unsigned long)app->reboot_count);
  (void)snprintf(line[1], sizeof(line[1]), "%s", crash_line);
  (void)snprintf(line[2], sizeof(line[2]), "%s", hwm_line);
  (void)snprintf(line[3], sizeof(line[3]), "%s", phase_line);
  (void)snprintf(line[4], sizeof(line[4]), "RTC %s SD %s",
                 rtc, sd_state_str);
  (void)snprintf(line[5], sizeof(line[5]), "BUF%u%s WR%uDR%u",
                 (unsigned int)app->sd_buffered,
                 (app->sd_paused != 0U) ? "P" : "",
                 (unsigned int)app->sd_written,
                 (unsigned int)app->sd_dropped);
  (void)snprintf(line[6], sizeof(line[6]), "SKIP%u ERR%u",
                 (unsigned int)app->display_skipped_count,
                 (unsigned int)app->sd_error);
  (void)snprintf(line[7], sizeof(line[7]), "DISP%lu HWM%u",
                 (unsigned long)(app->display_refresh_count > 999999UL ?
                                 999999UL : app->display_refresh_count),
                 (unsigned int)app->fifo_high_watermark);

  app_display_draw_lines_8(line);
}

/**
 * @brief  调试页面 D7：SD 卡日志状态（从 D6 扩展而来）。
 * @param  app 指向应用状态的指针（SD 日志诊断信息）。
 * @note   显示 SD 状态和总线模式、HAL 错误码、显示跳帧和
 *         强制刷新计数、I2C/OLED 恢复次数、任务心跳、
 *         缓冲/丢弃计数、FIFO 溢出总数和最大样本间隔、
 *         以及 FIFO 高水位标记。
 */
/* =========================================================================
 * D7 SD — SD 卡日志状态（新增页，替代原 D6 中放不下的字段）
 * ========================================================================= */
static void app_display_debug_d7_sd(const AppState_t *app)
{
  char line[8][24];
  const char *sd_state_str;
  const char *bus_mode_str;
  uint32_t sd_hal_error;

  if (app == NULL) { return; }

  sd_state_str = APP_DataLog_StateLabel(app->sd_state);

  switch (APP_SD_Card_GetMode())
  {
  case 1U: bus_mode_str = "4B"; break;
  case 2U: bus_mode_str = "FB"; break;
  default: bus_mode_str = "1B"; break;
  }

  sd_hal_error = APP_SD_Card_GetLastError();

  (void)snprintf(line[0], sizeof(line[0]), "D7 SD %s BUS %s",
                 sd_state_str,
                 bus_mode_str);
  (void)snprintf(line[1], sizeof(line[1]), "HERR %08lX FE %u%s",
                 (unsigned long)sd_hal_error,
                 (unsigned int)app->sd_error,
                 (app->sd_paused != 0U) ? " P" : "");
  (void)snprintf(line[2], sizeof(line[2]), "DSKIP %u FRC %u",
                 (unsigned int)app->display_skipped_count,
                 (unsigned int)app->ui_forced_count);
  (void)snprintf(line[3], sizeof(line[3]), "I2CREC %lu OLEDR %lu",
                 (unsigned long)(app->i2c_recover_count > 9999UL ?
                                 9999UL : app->i2c_recover_count),
                 (unsigned long)(app->oled_reinit_count > 9999UL ?
                                 9999UL : app->oled_reinit_count));
  (void)snprintf(line[4], sizeof(line[4]), "MHB %lu UHB %lu",
                 (unsigned long)(app->max_task_heartbeat > 999999UL ?
                                 999999UL : app->max_task_heartbeat),
                 (unsigned long)(app->ui_task_heartbeat > 999999UL ?
                                 999999UL : app->ui_task_heartbeat));
  (void)snprintf(line[5], sizeof(line[5]), "BUF %u DR %u",
                 (unsigned int)app->sd_buffered,
                 (unsigned int)app->sd_dropped);
  (void)snprintf(line[6], sizeof(line[6]), "OVF %lu GAP %u",
                 (unsigned long)(app->fifo_overflow_total > 999999UL ?
                                 999999UL : app->fifo_overflow_total),
                 (unsigned int)app->max_sample_gap_ms);
  (void)snprintf(line[7], sizeof(line[7]), "HWM %u",
                 (unsigned int)app->fifo_high_watermark);

  app_display_draw_lines_8(line);
}

/* =========================================================================
 * D8 ECG/PPG：ECG 与 PPG 对比，显示 HR、BPM、差值、RR、
 *              原始/滤波范围、导联脱落、显示模式、错误计数和 ADC 通道。
 *              黄色 RL/RLD 电极未接到 LO+/- 引脚；
 *              移除它不会改变 lead_off_raw，这是预期硬件行为。
 * ========================================================================= */
static void app_display_debug_d8_ecg(const AppState_t *app)
{
  char line[8][24];
  AppEcgDebugSnapshot_t snap;
  const char *mode_str;
  const char *diff_flag;
  uint8_t  ecg_hr_disp;
  uint8_t  ppg_bpm_disp;
  int      diff_disp;
  uint8_t  both_valid;

  if (app == NULL) { return; }

  app_ecg_get_debug_snapshot(app, &snap);

#if (APP_ECG_DEBUG_DISPLAY_RAW != 0U)
  mode_str = "R";
#elif (APP_ECG_DEBUG_DISPLAY_FILTERED != 0U)
  mode_str = "F";
#elif (APP_ECG_DEBUG_DISPLAY_VISUAL != 0U)
  mode_str = "V";
#else
  mode_str = "N";
#endif

  /* ECG 与 PPG 对比：调试时标记较大的差异。
   * 仅用于观察，不改变 ECG/PPG 算法。 */
  both_valid = ((snap.ecg_valid != 0U) && (snap.ppg_valid != 0U)
                && (snap.ecg_hr > 0U) && (snap.ppg_bpm > 0U)) ? 1U : 0U;

  ecg_hr_disp  = (snap.ecg_valid != 0U) ? snap.ecg_hr : 0U;
  ppg_bpm_disp = (snap.ppg_valid != 0U) ? snap.ppg_bpm : 0U;

  if (both_valid != 0U)
  {
    diff_disp = (int)snap.hr_diff;
    if (diff_disp < 0)
      diff_flag = (diff_disp > -11) ? "OK" : "!";
    else
      diff_flag = (diff_disp < 11) ? "OK" : "!";
  }
  else
  {
    diff_disp = 0;
    diff_flag = "--";
  }

  (void)snprintf(line[0], sizeof(line[0]), "D8 ECG/PPG");
  (void)snprintf(line[1], sizeof(line[1]), "E%03u P%03u D%+03d %s",
                 (unsigned int)ecg_hr_disp,
                 (unsigned int)ppg_bpm_disp,
                 diff_disp, diff_flag);
  (void)snprintf(line[2], sizeof(line[2]), "RR:%04u V%u L:%02X",
                 (unsigned int)snap.ecg_rr_ms,
                 (unsigned int)snap.ecg_valid,
                 (unsigned int)snap.lead_raw);
  (void)snprintf(line[3], sizeof(line[3]), "R:%u-%u d:%u",
                 (unsigned int)snap.raw_min,
                 (unsigned int)snap.raw_max,
                 (unsigned int)(snap.raw_max - snap.raw_min));
  (void)snprintf(line[4], sizeof(line[4]), "F:%d~%d M:%s",
                 (int)snap.filt_min, (int)snap.filt_max, mode_str);
  (void)snprintf(line[5], sizeof(line[5]), "SAT:%lu DMA:%lu",
                 (unsigned long)(snap.adc_sat_count > 999999UL ?
                                 999999UL : snap.adc_sat_count),
                 (unsigned long)(snap.dma_overflow_count > 999999UL ?
                                 999999UL : snap.dma_overflow_count));
  (void)snprintf(line[6], sizeof(line[6]), "TO:%lu PA5 IN5",
                 (unsigned long)(snap.no_r_peak_timeout_count > 999999UL ?
                                 999999UL : snap.no_r_peak_timeout_count));
  (void)snprintf(line[7], sizeof(line[7]), "SC:%lu",
                 (unsigned long)(snap.sample_count > 999999UL ?
                                 999999UL : snap.sample_count));

  app_display_draw_lines_8(line);
}

/* =========================================================================
 * D9 SCHED — 调度诊断：TIM6 → MAXtask 100 Hz 闭环 + 运行时指标
 *
 * 本页面仅用于收尾验证。低频速率计算（约1秒一次），不做浮点、
 * 不增加刷新频率、不参与实时采样路径。
 * ========================================================================= */
static void app_display_debug_d9_sched(const AppState_t *app)
{
  char line[8][24];
  const char *sd_state_str;
  uint32_t t6_count;
  uint32_t mhb;
  uint32_t uhb;

  /* 低频差值计算 */
  static uint32_t last_t6    = 0U;
  static uint32_t last_mhb   = 0U;
  static uint32_t last_tick  = 0U;
  static uint32_t t6_rate    = 0U;
  static uint32_t max_rate   = 0U;
  uint32_t now_tick;
  uint32_t elapsed;

  if (app == NULL) { return; }

  t6_count = APP_TIM6_GetIsrCount();
  mhb      = app->max_task_heartbeat;
  uhb      = app->ui_task_heartbeat;
  now_tick = HAL_GetTick();

  if (last_tick != 0U)
  {
    elapsed = now_tick - last_tick;
    if (elapsed >= 1000U)
    {
      t6_rate  = (t6_count - last_t6) * 1000U / elapsed;
      max_rate = (mhb  - last_mhb)  * 1000U / elapsed;
      last_t6   = t6_count;
      last_mhb  = mhb;
      last_tick = now_tick;
    }
  }
  else
  {
    last_t6   = t6_count;
    last_mhb  = mhb;
    last_tick = now_tick;
  }

  sd_state_str = APP_DataLog_StateLabel(app->sd_state);

  (void)snprintf(line[0], sizeof(line[0]), "D9 SCHED");
  (void)snprintf(line[1], sizeof(line[1]), "T6%4lu/s M%4lu/s",
                 (unsigned long)t6_rate, (unsigned long)max_rate);
  (void)snprintf(line[2], sizeof(line[2]), "TO%6lu TGAP%3u",
                 (unsigned long)app->max_task_timeout_count,
                 (unsigned int)app->max_task_gap_ms);
  (void)snprintf(line[3], sizeof(line[3]), "MHB%6lu",
                 (unsigned long)(mhb > 999999UL ? 999999UL : mhb));
  (void)snprintf(line[4], sizeof(line[4]), "UHB%6lu",
                 (unsigned long)(uhb > 999999UL ? 999999UL : uhb));
  (void)snprintf(line[5], sizeof(line[5]), "SD %s",
                 sd_state_str);
  (void)snprintf(line[6], sizeof(line[6]), "OVF%5lu SGAP%3u",
                 (unsigned long)(app->fifo_overflow_total > 999999UL ?
                                 999999UL : app->fifo_overflow_total),
                 (unsigned int)app->max_sample_gap_ms);
  (void)snprintf(line[7], sizeof(line[7]), "HWM%3u",
                 (unsigned int)app->fifo_high_watermark);

  app_display_draw_lines_8(line);
}

/* =========================================================================
 * D10 ECG Q — ECG 信号质量诊断：SQ/reason、raw_span/filt_span、
 *              noise/threshold、peak_snr、DMA high watermark / overflow
 * ========================================================================= */
static void app_display_debug_d10_ecg_q(const AppState_t *app)
{
  char line[8][24];
  const char *reason_str;
  const char *last_reason_str;

  if (app == NULL) { return; }

  switch (app->ecg_invalid_reason) {
  case ECG_INVALID_OK:            reason_str = "OK";   break;
  case ECG_INVALID_LEAD_OFF:      reason_str = "LEAD"; break;
  case ECG_INVALID_ADC_SAT:       reason_str = "SAT";  break;
  case ECG_INVALID_DMA_OVERFLOW:  reason_str = "DMA";  break;
  case ECG_INVALID_NO_R_PEAK:     reason_str = "NoR";  break;
  case ECG_INVALID_LOW_AMPLITUDE: reason_str = "LoA";  break;
  case ECG_INVALID_NOISY:         reason_str = "NOIS"; break;
  case ECG_INVALID_RAW_FLATLINE:  reason_str = "FLAT"; break;
  default:                         reason_str = "?";    break;
  }

  switch (app->ecg_last_drop_reason) {
  case ECG_INVALID_OK:            last_reason_str = "OK";  break;
  case ECG_INVALID_LEAD_OFF:      last_reason_str = "LD";  break;
  case ECG_INVALID_ADC_SAT:       last_reason_str = "SAT"; break;
  case ECG_INVALID_DMA_OVERFLOW:  last_reason_str = "DMA"; break;
  case ECG_INVALID_NO_R_PEAK:     last_reason_str = "NR";  break;
  case ECG_INVALID_LOW_AMPLITUDE: last_reason_str = "LA";  break;
  case ECG_INVALID_NOISY:         last_reason_str = "NS";  break;
  case ECG_INVALID_RAW_FLATLINE:  last_reason_str = "FL";  break;
  default:                         last_reason_str = "?";   break;
  }

  (void)snprintf(line[0], sizeof(line[0]), "D10 ECGQ SQ%u %s",
                 (unsigned int)app->ecg_signal_quality, reason_str);
  (void)snprintf(line[1], sizeof(line[1]), "Rsp%u Fsp%u",
                 (unsigned int)app->ecg_raw_span,
                 (unsigned int)app->ecg_filtered_span);
  (void)snprintf(line[2], sizeof(line[2]), "N%lu T%lu",
                 (unsigned long)(app->ecg_noise_level > 999999UL ?
                                 999999UL : app->ecg_noise_level),
                 (unsigned long)(app->ecg_qrs_threshold > 999999UL ?
                                 999999UL : app->ecg_qrs_threshold));
  (void)snprintf(line[3], sizeof(line[3]), "SNR %u/100",
                 (unsigned int)app->ecg_peak_snr_x100);
  (void)snprintf(line[4], sizeof(line[4]), "DMAhwm %u OVF %lu",
                 (unsigned int)app->ecg_dma_available_high_watermark,
                 (unsigned long)(app->ecg_dma_overflow_count > 999999UL ?
                                 999999UL : app->ecg_dma_overflow_count));
  (void)snprintf(line[5], sizeof(line[5]), "SAT %lu LOFF %lu",
                 (unsigned long)(app->ecg_adc_sat_count > 999999UL ?
                                 999999UL : app->ecg_adc_sat_count),
                 (unsigned long)(app->ecg_lead_off_count > 999999UL ?
                                 999999UL : app->ecg_lead_off_count));
  (void)snprintf(line[6], sizeof(line[6]), "V%u H%u R%u P%u",
                 (unsigned int)app->ecg_valid,
                 (unsigned int)app->ecg_hr,
                 (unsigned int)app->ecg_rr_ms,
                 (unsigned int)app->ptt_ms);
  (void)snprintf(line[7], sizeof(line[7]), "L%s Q%u R%u F%u",
                 last_reason_str,
                 (unsigned int)app->ecg_last_drop_sq,
                 (unsigned int)app->ecg_last_drop_raw_span,
                 (unsigned int)app->ecg_last_drop_filtered_span);

  app_display_draw_lines_8(line);
}

/*
 * 手指状态 → OLED 提示字符串。
 * 返回 NULL 表示处于正常测量状态，上层应绘制原有内容。
 * 否则返回的字符串应被醒目地绘制在页面中央。
 * WAIT 状态需要 buf 提供格式化空间（至少 15 字节）。
 */
static const char *app_get_finger_status(const AppState_t *app, char *buf, size_t buf_size)
{
  if (app == NULL) { return NULL; }

  /* 传感器链路异常优先显示，覆盖手指状态 */
  if (app->sensor_health == (uint8_t)SENSOR_HEALTH_I2C_ERR)
  {
    (void)snprintf(buf, buf_size, "I2C ERR %lu",
                   (unsigned long)app->sensor_last_i2c_error);
    return buf;
  }

  if (app->sensor_health == (uint8_t)SENSOR_HEALTH_INIT_FAIL)
  {
    return "INIT FAIL";
  }

  if (app->sensor_health == (uint8_t)SENSOR_HEALTH_RECOVERING)
  {
    return "RECOVERING";
  }

  if (app->sensor_health == (uint8_t)SENSOR_HEALTH_STALE)
  {
    uint32_t age = HAL_GetTick() - app->sensor_last_sample_tick;
    (void)snprintf(buf, buf_size, "WAIT %lu.%luS",
                   (unsigned long)(age / 1000U),
                   (unsigned long)((age % 1000U) / 100U));
    return buf;
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

/**
 * @brief  渲染简化的状态页面（用于启动/自检期间）。
 * @param  app          指向应用状态的指针（用于 RTC 和 UART 数据）。
 * @param  status_line_1 第一行状态文本（例如 "INIT..."）。
 * @param  status_line_2 第二行状态文本（例如 "CALIBRATING"）。
 * @note   布局：标题在 y=0，RTC 时间在 y=16，RTC 日期在 y=24，
 *         状态行在 y=32/40，UART 状态在 y=48。
 *         常用于初始传感器校准阶段。
 */
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

/**
 * @brief  带消抖的按键轮询，仅检测新的按下沿。
 * @param  button 指向按键状态结构的指针（端口、引脚、冷却时间）。
 * @return 如果检测到新的按下沿（下降沿且冷却时间已过）则返回 1，
 *         如果空闲、持续按住或冷却中则返回 0。
 * @note   长按不会重复触发；仅首次按下沿触发一次。
 *         冷却时间为 PAGE_BUTTON_DEBOUNCE_TICKS（与采样周期对齐）。
 */
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

/**
 * @brief  将波形缓冲重置为初始空状态。
 * @param  waveform 指向待重置的波形缓冲。
 * @note   清零所有样本和标记，将 write_index 和 sample_count
 *         重置为 0，并将 AGC 缩放估计重新初始化为 WAVEFORM_AGC_MIN_SCALE。
 *         settle_count 也被重置，使稳定窗口重新开始。
 */
/* 将一条波形缓冲恢复到”尚无样本”的初始状态。 */
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
  waveform->scale_estimate = waveform->min_scale;
}

static uint32_t waveform_abs_i32(int32_t value)
{
  if (value < 0)
  {
    return (uint32_t)(-value);
  }

  return (uint32_t)value;
}

/**
 * @brief  向波形缓冲压入一个带通滤波后的 AC 样本。
 * @param  waveform       指向波形缓冲。
 * @param  filtered_value 带通滤波后的样本值（int32）。
 * @note   前 WAVEFORM_SETTLE_SAMPLES 个样本被丢弃，让带通滤波器的
 *         阶跃响应在填充显示窗口之前达到稳定。使用具有快攻击/慢释放
 *         特性的 AGC 来将垂直缩放自适应到信号幅度。缩放估计值被钳位在
 *         WAVEFORM_AGC_MIN_SCALE 和 WAVEFORM_AGC_MAX_SCALE 之间。
 */
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

  if (waveform->scale_estimate < waveform->min_scale)
  {
    waveform->scale_estimate = waveform->min_scale;
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

/**
 * @brief  将波形缓冲中的最新样本标记为脉搏检测点。
 * @param  waveform 指向待标记的波形缓冲。
 * @note   标记被放置在最近写入的样本位置。
 *         在 waveform_buffer_draw() 期间，标记被渲染为波形区域底部
 *         3 行的小箭头。如果缓冲中没有样本（sample_count == 0）则无操作。
 */
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

/**
 * @brief  在 OLED 上绘制动态缩放的波形。
 * @param  waveform 指向包含样本和标记的波形缓冲。
 * @param  x        波形绘制区域的 X 原点。
 * @param  y        波形绘制区域的 Y 原点。
 * @param  width    绘制区域的宽度（像素）。
 * @param  height   绘制区域的高度（像素）。
 * @param  dotted   非零表示绘制虚线（每隔一个像素），0 表示实线。
 * @param  markers  非零表示在底部渲染脉搏检测标记。
 * @note   两遍渲染：第一遍扫描最小/最大值用于垂直自动缩放，
 *         第二遍将每个样本映射到像素坐标，上下留 1 像素边距。
 *         如果缓冲为空，则绘制水平中心线。
 *         如果可视范围 < 4，则强制最小范围为 4，以避免除零并保持小信号可见。
 */
/*
 * 动态缩放波形绘制：扫描可见样本的最小/最大值，把数据映射到 y+1..y+height-2，
 * 上下各留 1 像素边距，避免峰/谷被屏幕边缘截断。
 * 若所有样本相等，绘制水平中线。
 * 脉搏标记放在波形区域底部 3 行，不进入主要波形显示区。
 */
static void waveform_buffer_draw(const WaveformBuffer_t *waveform,
                                 uint8_t x,
                                 uint8_t y,
                                 uint8_t width,
                                 uint8_t height,
                                 uint8_t dotted,
                                 uint8_t markers,
                                 uint8_t stable_scale)
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
  int32_t  map_top;
  int32_t  map_bot;
  int32_t  map_range;
  int32_t  draw_y_i;
  uint8_t  marker_base;

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

  /* 确定可视范围 */
  if (stable_scale != 0U)
  {
    /* 稳定缩放：3× ±scale_estimate，中心 = 0（ECG 已去直流）。
     * 较宽范围让波形更紧凑，R 峰占屏约 1/3 高度。 */
    int32_t half = (int32_t)waveform->scale_estimate * 3;
    if (half < 50) { half = 50; }
    vis_min = -half;
    vis_max =  half;
  }
  else
  {
    /* 动态缩放：扫描可见样本 min/max（PPG 原始行为） */
    vis_min = INT32_MAX;
    vis_max = INT32_MIN;
    for (i = 0U; i < sample_count; i++)
    {
      sample_index = (uint16_t)((start_index + i) % SSD1306_WIDTH);
      sample_value = waveform->samples[sample_index];
      if (sample_value < vis_min) { vis_min = sample_value; }
      if (sample_value > vis_max) { vis_max = sample_value; }
    }
  }

  vis_range = vis_max - vis_min;
  if (vis_range < 4) { vis_range = 4; }

  /* 映射区间：y+1 到 y+height-2，上下各 1 像素边距 */
  map_top = (int32_t)y + 1;
  map_bot = (int32_t)y + (int32_t)height - 2;
  if (map_bot < map_top) { map_bot = map_top; }
  map_range = map_bot - map_top;

  /* 第 2 遍：绘制 */
  for (i = 0U; i < sample_count; i++)
  {
    sample_index = (uint16_t)((start_index + i) % SSD1306_WIDTH);
    sample_value = waveform->samples[sample_index];

    /* 钳位到可见范围 */
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
