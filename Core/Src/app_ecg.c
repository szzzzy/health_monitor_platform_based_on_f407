/**
  ******************************************************************************
  * @file    app_ecg.c
  * @brief   ECG QRS 检测 — 250 Hz 采样，简化 Pan-Tompkins 状态机
  *
  * 每 10 ms 主循环调用 app_ecg_process_samples()，从 DMA 环形缓冲消费
  * 全部待处理样本。单个样本的处理流水线：
  *   1. DMA 覆盖检测 → 丢弃旧数据 + 重置检测器
  *   2. 导联脱落检查 → 脱落则重置检测器并跳过后继续
  *   3. ADC 饱和检查 (>=4090 LSB) → 跳过该样本
  *   4. DC 漂移消除 (一阶高通 IIR, fc≈1.24 Hz)
  *   5. 滑动平均平滑 (一阶低通, fc≈13.3 Hz)
  *   6. 自适应阈值 QRS 状态机
  *   7. 有效 R 峰 → 至少 2 峰才更新 HR/RR → 通知 PTT
  *
  * 每个消费样本配有推算的 4 ms 间隔时间戳，用于 RR 与 PTT 计算。
  *
  * 仅为工程观测/趋势提示，不声称临床诊断能力。
  ******************************************************************************
  */

#include "app_ecg.h"
#include "app_ptt.h"
#include "app_display.h"
#include "adc.h"
#include <string.h>

#if (APP_ECG_DEBUG_PRINTF != 0U)
#include <stdio.h>
#include "usart.h"
#endif

/* === ECG 采样率 ============================================================ */
#define APP_ECG_SAMPLE_RATE_HZ      250U
#define APP_ECG_SAMPLE_PERIOD_MS    4U    /* 1000 / 250 = 4 ms */

/* === DC 漂移消除 — 一阶高通 IIR ============================================ */
/* cutoff ≈ fs / (2π·2^DC_SHIFT) ≈ 250 / (2π·32) ≈ 1.24 Hz */
#define APP_ECG_DC_SHIFT            5U

/* === 滑动平均平滑 — 一阶低通 ================================================ */
/* cutoff ≈ fs / (2π·SMOOTH_DIV) ≈ 250 / (2π·3) ≈ 13.3 Hz */
#define APP_ECG_SMOOTH_DIV          3

/* === QRS 检测器参数 ======================================================== */
#define APP_ECG_SETTLE_SAMPLES      125U /* 500 ms 启动稳定期，×250 Hz */
#define APP_ECG_MIN_RR_MS           300U /* 生理最小 RR (200 bpm 上限) */
#define APP_ECG_MAX_RR_MS           2000U/* 生理最大 RR (30 bpm 下限) */
#define APP_ECG_STALE_MS            3000U/* 无新 R 峰超时 */
#define APP_ECG_QRS_MIN_THRESHOLD   45U  /* QRS 检测绝对最小阈值 (LSB) */
#define APP_ECG_QRS_MAX_THRESHOLD   1000U/* QRS 检测绝对最大阈值 (LSB) */
#define APP_ECG_QRS_NOISE_GAIN      3U   /* 动态阈值 = 噪声基线 × 增益 */
#define APP_ECG_QRS_END_DIV         2U   /* 波群结束：幅值跌至候选峰值/2 */
#define APP_ECG_QRS_MAX_WIDTH_MS    160U /* QRS 波群最大宽度 */

/* === ADC 饱和检测阈值 ====================================================== */
#define APP_ECG_ADC_SAT_THRESHOLD   4090U

/* === Visual display filter parameters (OLED only, not used for QRS) ======== */
/* DC removal: alpha = 1/256, cutoff ≈ 250/(2π·256) ≈ 0.16 Hz — very slow,
 * keeps baseline stable without distorting ST segment morphology. */
#define APP_ECG_VISUAL_DC_SHIFT     8U
/* Low-pass: alpha = 1/4, cutoff ≈ 250/(2π·4) ≈ 10 Hz — smooths noise
 * while preserving R-peak sharpness. */
#define APP_ECG_VISUAL_LP_SHIFT     2U
/* Fixed gain applied after filtering. 3× makes ~200 LSB AC visible on OLED. */
#define APP_ECG_VISUAL_GAIN         3
/* Display clamp: keeps visual output within ±1200 to prevent OLED overdraw. */
#define APP_ECG_VISUAL_CLAMP        1200
/* Decimation: push 1 sample per 3, 250/3 ≈ 83 px/s, 128px ≈ 1.5 s window. */
#define APP_ECG_VISUAL_DECIM        3U
/* Reserved: 50 Hz notch for 250 Hz sample rate. Keep 0 until biquad verified. */
#define APP_ECG_VISUAL_ENABLE_NOTCH 0U

/* 保持 ECG 波形运行同时观察原始 AD8232 导联脱落引脚。
 * 在验证 PE5/PE6 接线后设置为 1。 */
#define APP_ECG_ENABLE_LEAD_OFF_GATE 0U

/* === 内部状态结构 ========================================================== */
typedef struct
{
  int32_t dc_estimate;
  int32_t smooth_value;
  uint32_t noise_level;
  uint32_t sample_count;
  uint32_t last_r_peak_ms;

  uint8_t initialized;
  uint8_t in_candidate;
  uint8_t peak_count;     /* 本次检出周期内的有效 R 峰计数 */

  uint32_t candidate_peak_abs;
  uint32_t candidate_peak_ms;
  uint32_t candidate_start_ms;
} AppEcgState_t;

static AppEcgState_t ecg_state;

/* === OLED 显示滤波 ========================================================= */
/* 检测链路使用 ecg_state.smooth_value（不变）。
 * 显示链路额外做 5 点滑动平均 + 慢速振幅跟踪 + 软限幅，
 * 目的是让 OLED 波形平滑稳定，不被单个 R 峰拉满屏。 */
typedef struct
{
  int32_t buf[5];
  uint8_t idx;
  uint8_t count;
  int32_t amplitude;
  uint8_t decim;   /* 降采样计数器：每 2 个样本推 1 次，减慢滚动 */
} EcgDispFilter_t;

static EcgDispFilter_t ecg_disp;
static uint8_t ecg_debug_disp_decim; /* debug 显示模式独立降采样计数器 */

/* === OLED visual display filter — independent of QRS detection chain ======= */
/* Output only goes to app_display_add_ecg_sample(). Never feeds back into
 * ecg_state or app->ecg_filtered. Reset on lead-off, DMA overflow, etc. */
typedef struct
{
  int32_t dc;        /* slow DC tracking for baseline wander removal */
  int32_t lp;        /* low-pass state for high-frequency noise reduction */
  uint8_t decim;     /* independent decimation counter */
} EcgVisualFilter_t;

static EcgVisualFilter_t ecg_visual;

/* === ECG 调试统计 ========================================================== */
typedef struct
{
  uint16_t raw_min;
  uint16_t raw_max;
  int16_t  filt_min;
  int16_t  filt_max;
  uint16_t sample_cnt;   /* 当前统计窗口内已累计样本数 */
} EcgDebugStats_t;

static EcgDebugStats_t ecg_dbg_stats;

#if (APP_ECG_DEBUG_PRINTF != 0U)
static void app_ecg_debug_print_stats(const EcgDebugStats_t *s,
                                      const AppState_t *app)
{
  char line[96];
  int len;

  if ((s == NULL) || (app == NULL)) return;

  len = snprintf(line, sizeof(line),
                 "ECGDBG raw_min=%u raw_max=%u raw_diff=%u"
                 " filt_min=%d filt_max=%d lead=0x%02X hr=%u rr=%u\r\n",
                 (unsigned int)s->raw_min,
                 (unsigned int)s->raw_max,
                 (unsigned int)(s->raw_max - s->raw_min),
                 (int)s->filt_min,
                 (int)s->filt_max,
                 (unsigned int)app_ecg_read_lead_off_raw(),
                 (unsigned int)app->ecg_hr,
                 (unsigned int)app->ecg_rr_ms);

  if ((len > 0) && ((size_t)len < sizeof(line)))
  {
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, 100U);
  }
}
#endif /* APP_ECG_DEBUG_PRINTF */

/* ---- ECG 调试快照 getter（供 OLED D8 页面使用） ---- */
void app_ecg_get_debug_snapshot(const AppState_t *app,
                                AppEcgDebugSnapshot_t *out)
{
  if (out == NULL) return;

  (void)memset(out, 0, sizeof(*out));

  out->raw_min  = ecg_dbg_stats.raw_min;
  out->raw_max  = ecg_dbg_stats.raw_max;
  out->filt_min = ecg_dbg_stats.filt_min;
  out->filt_max = ecg_dbg_stats.filt_max;
  out->lead_raw = app_ecg_read_lead_off_raw();

  if (app != NULL)
  {
    out->ecg_valid               = app->ecg_valid;
    out->ecg_hr                  = app->ecg_hr;
    out->ecg_rr_ms              = app->ecg_rr_ms;
    out->sample_count           = app->ecg_sample_count;
    out->dma_overflow_count     = app->ecg_dma_overflow_count;
    out->adc_sat_count          = app->ecg_adc_sat_count;
    out->lead_off_count         = app->ecg_lead_off_count;
    out->no_r_peak_timeout_count = app->ecg_no_r_peak_timeout_count;

    /* PPG fields from existing MAX30102/PPG BPM pipeline.
     * ppg_bpm > 0 is used as secondary validity guard in case bpm_valid
     * is set but the value was never populated with a real measurement. */
    out->ppg_bpm   = app->bpm_value;
    out->ppg_valid = app->bpm_valid;

    if ((out->ecg_valid != 0U) && (out->ppg_valid != 0U)
        && (out->ecg_hr > 0U) && (out->ppg_bpm > 0U))
    {
      out->hr_diff = (int16_t)out->ecg_hr - (int16_t)out->ppg_bpm;
    }
    else
    {
      out->hr_diff = 0;
    }
  }
}

/* === 前向声明 ============================================================== */
static void     app_ecg_reset_detector(void);
static void     app_ecg_reset_display_filter(void);
static void     app_ecg_reset_visual_filter(void);
static void     app_ecg_reset_debug_stats(void);
static void     app_ecg_debug_update_raw(uint16_t raw_value,
                                         const AppState_t *app);
static void     app_ecg_debug_update_filtered(int16_t filtered_value);
static uint32_t app_ecg_abs_i32(int32_t v);
static int16_t  app_ecg_clamp_i16(int32_t v);
static AppEcgUpdate_t app_ecg_process_sample(AppState_t *app,
                                             int32_t filtered,
                                             uint32_t timestamp_ms);

/**
 ******************************************************************************
 * @brief  重置 ECG 模块：检测器、AppState 字段、PTT、显示波形。
 * @param  app AppState 指针（可为 NULL）。
 * @note   清除 ecg_raw、ecg_filtered、lead_off、valid、HR、RR、R 峰
 *         时间戳、PTT 字段，并重置 QRS 检测器状态机。
 ******************************************************************************
 */
void app_ecg_reset(AppState_t *app)
{
  app_ecg_reset_detector();
  app_ecg_reset_display_filter();
  app_ecg_reset_debug_stats();
  if (app == NULL) return;

  app->ecg_raw       = 0U;
  app->ecg_filtered  = 0;
  app->ecg_lead_off  = 0U;
  app->ecg_valid     = 0U;
  app->ecg_hr        = 0U;
  app->ecg_rr_ms     = 0U;
  app->ecg_r_peak_ms = 0UL;
  app->ptt_valid     = 0U;
  app->ptt_ms        = 0U;
  app_ptt_reset(app);
  app_display_reset_ecg_waveform();
}

/**
 ******************************************************************************
 * @brief  读取 AD8232 导联脱落引脚（LO- = PE5，LO+ = PE6）。
 * @return APP_ECG_LEAD_OFF_MINUS (0x01 = 红色电极) 和/或
 *         APP_ECG_LEAD_OFF_PLUS (0x02 = 绿色电极) 的位掩码。置位表示
 *         该电极已断开或接触不良。
 * @note   仅红色 (LO-) 和绿色 (LO+) 电极可检测。
 *         黄色 RL/RLD 参考电极不经过 AD8232 LO 引脚，
 *         因此移除它不会改变返回值。
 *         这是预期的硬件行为，非软件错误。
 ******************************************************************************
 */
uint8_t app_ecg_read_lead_off(void)
{
  uint8_t lead_off = app_ecg_read_lead_off_raw();

#if (APP_ECG_ENABLE_LEAD_OFF_GATE != 0U)
  return lead_off;
#else
  (void)lead_off;
  return 0U;
#endif
}

uint8_t app_ecg_read_lead_off_raw(void)
{
  uint8_t lead_off = 0U;

  if (HAL_GPIO_ReadPin(AD8232_LD_MINUS_GPIO_Port, AD8232_LD_MINUS_Pin) == GPIO_PIN_SET)
  {
    lead_off |= APP_ECG_LEAD_OFF_MINUS;
  }
  if (HAL_GPIO_ReadPin(AD8232_LD_PLUS_GPIO_Port, AD8232_LD_PLUS_Pin) == GPIO_PIN_SET)
  {
    lead_off |= APP_ECG_LEAD_OFF_PLUS;
  }
  return lead_off;
}

/**
 ******************************************************************************
 * @brief  处理来自 DMA 环形缓冲的所有可用 ECG 样本。
 * @param  app AppState 指针。
 * @return 若该批次至少检测到一个 R 峰则返回 1，否则返回 0。
 * @note   每个样本获得独立的 4 ms 插值时间戳。
 *         DMA 溢出导致检测器重置并清除 ecg_valid/ptt_valid。
 *         流水线：导联脱落检查 -> ADC 饱和 -> DC 消除 ->
 *         低通平滑 -> QRS 状态机。即使无新样本到达也检查
 *         R 峰超时（APP_ECG_STALE_MS）。
 ******************************************************************************
 */
uint8_t app_ecg_process_samples(AppState_t *app)
{
  uint16_t raw_value;
  uint8_t  lead_off;
  int32_t  raw_i32, ac_value, delta;
  uint32_t now_ms;
  uint16_t avail;
  uint8_t  r_peak_found = 0U;
  uint16_t consumed = 0U;
  uint8_t  dma_overflow_occurred = 0U;

  if (app == NULL) return 0U;

  /* 进入批量消费前取一次 HAL Tick + 可用样本数 */
  now_ms = HAL_GetTick();
  avail = app_ecg_adc_get_available_count();

  /* DMA 覆盖检测：DMA 写入跑赢了软件消费，环形缓冲数据已丢失 */
  if (app_ecg_adc_had_overflow() != 0U)
  {
    dma_overflow_occurred = 1U;
    app->ecg_dma_overflow_count++;
    app_ecg_adc_clear_overflow();

    /* 丢弃已被覆盖的旧样本已在 adc.c read_sample 中处理，
     * 这里仅重置检测器状态和 ECG/PTT 输出 */
    app_ecg_reset_detector();
    app_ecg_reset_display_filter();
    app_ecg_reset_debug_stats();
    app->ecg_valid = 0U;
    app->ecg_hr = 0U;
    app->ecg_rr_ms = 0U;
    app->ptt_valid = 0U;
    app->ptt_ms = 0U;
    app_ptt_reset(app);
    app_display_reset_ecg_waveform();

    /* 刷新可用样本数（丢弃旧数据后） */
    avail = app_ecg_adc_get_available_count();
  }

  /* 消费 DMA 环形缓冲中所有可用样本 */
  {
    uint32_t sample_ts;
    uint16_t remaining = avail;

    while (remaining > 0U)
    {
      /* 每个样本独立推算时间戳，间隔 4 ms */
      if (app_ecg_adc_read_sample(&raw_value, &sample_ts, now_ms, remaining) == 0U)
      {
        break;
      }
      remaining--;
      consumed++;
      app->ecg_sample_count++;

      /* 1. 导联脱落检查 */
      lead_off = app_ecg_read_lead_off();
      if ((lead_off != 0U) && (app->ecg_lead_off == 0U))
      {
        app_display_reset_ecg_waveform();
        app_ecg_reset_display_filter();
        app_ecg_reset_debug_stats();
      }
      app->ecg_lead_off = lead_off;
      if (lead_off != 0U)
      {
        app->ecg_lead_off_count++;
        app->ecg_valid = 0U;
        app->ecg_hr = 0U;
        app->ecg_rr_ms = 0U;
        app->ptt_valid = 0U;
        app->ptt_ms = 0U;
        app_ecg_reset_detector();
        app_ptt_reset(app);
        app->ecg_filtered = 0;
        /* 导联脱落时不放弃剩余样本，但跳过进入下一次循环 */
        continue;
      }

      app->ecg_raw = raw_value;

      /* 2a. 调试统计 — raw min/max 在饱和检测之前，以观察 4095 附近饱和样本 */
      app_ecg_debug_update_raw(raw_value, app);

      /* 2b. ADC 饱和检测 — 饱和样本不可用于 QRS 检测 */
      if (raw_value >= APP_ECG_ADC_SAT_THRESHOLD)
      {
        app->ecg_adc_sat_count++;
        continue;
      }

      /* 3. 首次初始化 */
      raw_i32 = (int32_t)raw_value;
      if (ecg_state.initialized == 0U)
      {
        ecg_state.dc_estimate  = raw_i32;
        ecg_state.smooth_value = 0;
        ecg_state.noise_level  = APP_ECG_QRS_MIN_THRESHOLD / APP_ECG_QRS_NOISE_GAIN;
        ecg_state.sample_count = 0U;
        ecg_state.peak_count   = 0U;
        ecg_state.initialized  = 1U;
      }

      /* 4. DC 漂移消除 — 一阶高通 IIR */
      delta = raw_i32 - ecg_state.dc_estimate;
      ecg_state.dc_estimate += (delta / (int32_t)(1UL << APP_ECG_DC_SHIFT));
      ac_value = raw_i32 - ecg_state.dc_estimate;

      /* 5. 检测用低通平滑 (fc≈13.3 Hz, 不变) */
      ecg_state.smooth_value += ((ac_value - ecg_state.smooth_value) / APP_ECG_SMOOTH_DIV);
      app->ecg_filtered = app_ecg_clamp_i16(ecg_state.smooth_value);

      /* 5a. 调试统计 — filtered min/max 在滤波计算完成后更新 */
      app_ecg_debug_update_filtered(app->ecg_filtered);

      /* 5b. OLED 显示 */
#if (APP_ECG_DEBUG_DISPLAY_RAW != 0U)
      {
        /* Direct raw ADC display: raw - 2048, 3:1 decimation */
        ecg_debug_disp_decim++;
        if ((ecg_debug_disp_decim % 3U) == 0U)
        {
          app_display_add_ecg_sample(raw_i32 - 2048);
        }
      }
#elif (APP_ECG_DEBUG_DISPLAY_FILTERED != 0U)
      {
        /* Direct filtered display: ecg_filtered, 3:1 decimation */
        ecg_debug_disp_decim++;
        if ((ecg_debug_disp_decim % 3U) == 0U)
        {
          app_display_add_ecg_sample((int32_t)app->ecg_filtered);
        }
      }
#elif (APP_ECG_DEBUG_DISPLAY_VISUAL != 0U)
      {
        /* Visual-optimized display filter chain (OLED only, not for QRS):
         *   raw_i32 -> slow DC removal -> mild LP -> gain -> clamp -> display
         * Independent of ecg_state / app->ecg_filtered / QRS detection. */
        int32_t ac, disp;

        /* A. Slow DC removal: fc ~0.16 Hz, baseline-stable, ST-preserving */
        ecg_visual.dc += (raw_i32 - ecg_visual.dc) >> APP_ECG_VISUAL_DC_SHIFT;
        ac = raw_i32 - ecg_visual.dc;

        /* B. Mild low-pass: fc ~10 Hz, reduces noise, keeps R-peak sharp */
        ecg_visual.lp += (ac - ecg_visual.lp) >> APP_ECG_VISUAL_LP_SHIFT;

#if (APP_ECG_VISUAL_ENABLE_NOTCH != 0U)
        /* Reserved: 50 Hz notch for 250 Hz sample rate — not implemented */
#endif

        /* C. Fixed gain */
        disp = ecg_visual.lp * APP_ECG_VISUAL_GAIN;

        /* D. Soft clamp to prevent OLED overdraw */
        if (disp > APP_ECG_VISUAL_CLAMP)       { disp =  APP_ECG_VISUAL_CLAMP; }
        else if (disp < -APP_ECG_VISUAL_CLAMP) { disp = -APP_ECG_VISUAL_CLAMP; }

        /* E. Decimation: push 1 per VISUAL_DECIM samples */
        ecg_visual.decim++;
        if ((ecg_visual.decim % APP_ECG_VISUAL_DECIM) == 0U)
        {
          app_display_add_ecg_sample(disp);
        }
      }
#else
      /* Original display chain: 5-point MA + slow amplitude tracking
       * + soft limiting + 3:1 decimation. Preserved as fallback. */
      {
        int32_t disp_val, abs_val, limit;
        uint8_t  k;

        ecg_disp.buf[ecg_disp.idx] = (int32_t)app->ecg_filtered;
        ecg_disp.idx = (uint8_t)((ecg_disp.idx + 1U) % 5U);
        if (ecg_disp.count < 5U) { ecg_disp.count++; }

        disp_val = 0;
        for (k = 0U; k < ecg_disp.count; k++) { disp_val += ecg_disp.buf[k]; }
        disp_val /= (int32_t)ecg_disp.count;

        abs_val = (disp_val >= 0) ? disp_val : -disp_val;
        if (ecg_disp.amplitude == 0) { ecg_disp.amplitude = abs_val; }
        /* Slow EMA: alpha ~1/64, amplitude does not track single spikes */
        ecg_disp.amplitude += (abs_val - ecg_disp.amplitude) / 64;
        if (ecg_disp.amplitude < 50) { ecg_disp.amplitude = 50; }

        /* Soft limit: 6x amplitude, keeps R-peak visible, suppresses spikes */
        limit = ecg_disp.amplitude * 6;
        if (disp_val > limit)      { disp_val = limit; }
        else if (disp_val < -limit) { disp_val = -limit; }

        /* 3:1 decimation: 128px ~1.5 s */
        ecg_disp.decim++;
        if ((ecg_disp.decim % 3U) == 0U)
        {
          app_display_add_ecg_sample(disp_val);
        }
      }
#endif

      if (ecg_state.sample_count < 0xFFFFFFFFUL)
      {
        ecg_state.sample_count++;
      }

      /* 6. QRS 检测 — 传入每样本独立时间戳 */
      AppEcgUpdate_t update = app_ecg_process_sample(app, ecg_state.smooth_value, sample_ts);
      if (update.r_peak_detected != 0U)
      {
        r_peak_found = 1U;
        app_display_add_ecg_r_peak_marker();
      }
    }
  }

  /* 7. R 峰超时检查 (即使无新样本也不断激活) */
  if ((ecg_state.last_r_peak_ms != 0UL) &&
      ((now_ms - ecg_state.last_r_peak_ms) > APP_ECG_STALE_MS))
  {
    app->ecg_valid = 0U;
    app->ecg_hr = 0U;
    app->ecg_rr_ms = 0U;
    app->ptt_valid = 0U;
    app->ptt_ms = 0U;
    app_ptt_reset(app);
    app->ecg_no_r_peak_timeout_count++;
  }

  return r_peak_found;
}

/* ---- 简化 Pan-Tompkins QRS 状态机（单样本迭代） ---- */
static AppEcgUpdate_t app_ecg_process_sample(AppState_t *app,
                                             int32_t filtered_value,
                                             uint32_t timestamp_ms)
{
  AppEcgUpdate_t update;
  uint32_t magnitude, threshold, noise_delta;
  uint32_t rr_ms, hr_bpm;

  (void)memset(&update, 0, sizeof(update));

  if (app == NULL) return update;

  /* 稳定期：让 DC 估计与平滑器收敛 */
  if (ecg_state.sample_count < APP_ECG_SETTLE_SAMPLES)
  {
    return update;
  }

  magnitude = app_ecg_abs_i32(filtered_value);

  /* 动态阈值 = 噪声基线 × 增益，钳位在 [MIN, MAX] */
  threshold = ecg_state.noise_level * APP_ECG_QRS_NOISE_GAIN;
  if (threshold < APP_ECG_QRS_MIN_THRESHOLD)
    threshold = APP_ECG_QRS_MIN_THRESHOLD;
  else if (threshold > APP_ECG_QRS_MAX_THRESHOLD)
    threshold = APP_ECG_QRS_MAX_THRESHOLD;

  /* ===== 空闲态 ===== */
  if (ecg_state.in_candidate == 0U)
  {
    /* 噪声基线自适应：快升慢降 */
    if (magnitude > ecg_state.noise_level)
    {
      noise_delta = magnitude - ecg_state.noise_level;
      ecg_state.noise_level += (noise_delta / 32U) ? (noise_delta / 32U) : 1U;
    }
    else
    {
      noise_delta = ecg_state.noise_level - magnitude;
      ecg_state.noise_level -= (noise_delta / 16U);
    }

    /* 触发：幅值超阈值 + 生理不应期满足 */
    if ((magnitude >= threshold) &&
        ((ecg_state.last_r_peak_ms == 0UL) ||
         ((timestamp_ms - ecg_state.last_r_peak_ms) >= APP_ECG_MIN_RR_MS)))
    {
      ecg_state.in_candidate       = 1U;
      ecg_state.candidate_peak_abs = magnitude;
      ecg_state.candidate_peak_ms  = timestamp_ms;
      ecg_state.candidate_start_ms = timestamp_ms;
    }
    return update;
  }

  /* ===== 候选态 ===== */

  /* 追踪波群最大峰值 */
  if (magnitude > ecg_state.candidate_peak_abs)
  {
    ecg_state.candidate_peak_abs = magnitude;
    ecg_state.candidate_peak_ms  = timestamp_ms;
  }

  /* 波群结束条件：幅值跌至候选峰值/END_DIV 以下，或宽度超限 */
  if ((magnitude > (ecg_state.candidate_peak_abs / APP_ECG_QRS_END_DIV)) &&
      ((timestamp_ms - ecg_state.candidate_start_ms) < APP_ECG_QRS_MAX_WIDTH_MS))
  {
    return update; /* 仍在波群内 */
  }

  /* 波群结束，退出候选态 */
  ecg_state.in_candidate = 0U;

  /* 候选峰值必须 ≥ 阈值 */
  if (ecg_state.candidate_peak_abs < threshold)
  {
    return update;
  }

  /* === 确认 R 峰 === */
  update.r_peak_detected = 1U;
  update.r_peak_ms       = ecg_state.candidate_peak_ms;
  app->ecg_r_peak_ms     = ecg_state.candidate_peak_ms;

  if (ecg_state.last_r_peak_ms != 0UL)
  {
    rr_ms = ecg_state.candidate_peak_ms - ecg_state.last_r_peak_ms;

    if ((rr_ms >= APP_ECG_MIN_RR_MS) && (rr_ms <= APP_ECG_MAX_RR_MS))
    {
      hr_bpm = (60000UL + (rr_ms / 2UL)) / rr_ms;
      if (hr_bpm > 255UL) hr_bpm = 255UL;

      app->ecg_rr_ms = (uint16_t)rr_ms;

      /* EMA 平滑, α=0.25 */
      if ((app->ecg_valid != 0U) && (app->ecg_hr != 0U))
      {
        app->ecg_hr = (uint8_t)((((uint16_t)app->ecg_hr * 3U) + (uint16_t)hr_bpm + 2U) / 4U);
      }
      else
      {
        app->ecg_hr = (uint8_t)hr_bpm;
      }

      app->ecg_valid = 1U;
      update.rr_ms   = (uint16_t)rr_ms;
      update.hr_bpm  = app->ecg_hr;

      /* PTT 链路：仅当 ecg_valid 已置位后，记录 ECG R 峰时间戳供 PTT 使用 */
      app_ptt_add_ecg_peak(ecg_state.candidate_peak_ms);
    }
    else
    {
      /* RR 超出生理范围 → 标无效，不清 peak_count（可能是偶发噪声） */
      app->ecg_valid = 0U;
      app->ecg_hr = 0U;
      app->ecg_rr_ms = 0U;
      app->ptt_valid = 0U;
      app->ptt_ms = 0U;
      app_ptt_reset(app);
    }
  }
  else
  {
    /* 首个 R 峰：只记录时间戳，不输出 ecg_hr / ecg_valid / PTT。
     * 需要第二个满足范围的峰才可验证 RR 是否合理。 */
    update.r_peak_detected = 1U;
    update.r_peak_ms       = ecg_state.candidate_peak_ms;
    /* ecg_valid 保持 0，ecg_hr 不变，ptt 不记录 */
  }

  ecg_state.last_r_peak_ms = ecg_state.candidate_peak_ms;
  return update;
}

/* ---- 清零整个 ECG 检测器状态 ---- */
static void app_ecg_reset_detector(void)
{
  (void)memset(&ecg_state, 0, sizeof(ecg_state));
}

/* ---- 清零 ECG 显示滤波器内部缓冲 ---- */
static void app_ecg_reset_display_filter(void)
{
  (void)memset(&ecg_disp, 0, sizeof(ecg_disp));
  ecg_debug_disp_decim = 0U;
  app_ecg_reset_visual_filter();
}

/* ---- 清零 ECG visual 显示滤波器 ---- */
static void app_ecg_reset_visual_filter(void)
{
  (void)memset(&ecg_visual, 0, sizeof(ecg_visual));
}

/* ---- 清零 ECG 调试统计窗口 ---- */
static void app_ecg_reset_debug_stats(void)
{
  (void)memset(&ecg_dbg_stats, 0, sizeof(ecg_dbg_stats));
}

/* ---- 更新 raw ADC 调试统计（在饱和检测之前调用，含饱和样本） ---- */
static void app_ecg_debug_update_raw(uint16_t raw_value,
                                     const AppState_t *app)
{
  if (ecg_dbg_stats.sample_cnt == 0U)
  {
    ecg_dbg_stats.raw_min = raw_value;
    ecg_dbg_stats.raw_max = raw_value;
  }
  else
  {
    if (raw_value < ecg_dbg_stats.raw_min)
      ecg_dbg_stats.raw_min = raw_value;
    if (raw_value > ecg_dbg_stats.raw_max)
      ecg_dbg_stats.raw_max = raw_value;
  }
  ecg_dbg_stats.sample_cnt++;

#if (APP_ECG_DEBUG_PRINTF != 0U)
  if (ecg_dbg_stats.sample_cnt >= 250U)
  {
    app_ecg_debug_print_stats(&ecg_dbg_stats, app);
    (void)memset(&ecg_dbg_stats, 0, sizeof(ecg_dbg_stats));
  }
#endif
}

/* ---- 更新 filtered ECG 调试统计（仅在滤波计算完成后调用） ---- */
static void app_ecg_debug_update_filtered(int16_t filtered_value)
{
  /* 复位后 filt_min == filt_max == 0 表示首个样本 */
  if (ecg_dbg_stats.filt_min == 0 && ecg_dbg_stats.filt_max == 0)
  {
    ecg_dbg_stats.filt_min = filtered_value;
    ecg_dbg_stats.filt_max = filtered_value;
  }
  else
  {
    if (filtered_value < ecg_dbg_stats.filt_min)
      ecg_dbg_stats.filt_min = filtered_value;
    if (filtered_value > ecg_dbg_stats.filt_max)
      ecg_dbg_stats.filt_max = filtered_value;
  }
}

/* ---- 32 位有符号绝对值 ---- */
static uint32_t app_ecg_abs_i32(int32_t v)
{
  return (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
}

/* ---- 将 int32 钳位到 int16 范围 ---- */
static int16_t app_ecg_clamp_i16(int32_t v)
{
  if (v > 32767L)  return 32767;
  if (v < -32768L) return -32768;
  return (int16_t)v;
}
